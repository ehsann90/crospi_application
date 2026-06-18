#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def ascending_factor(g, g0, g1, w_min, w_max):
    if g <= g0:
        return w_min
    if g >= g1:
        return w_max
    a = (g - g0) / max(1e-12, (g1 - g0))
    return w_min + a * (w_max - w_min)


def descending_factor(g, g0, g1, w_min, w_max):
    return w_max + w_min - ascending_factor(g, g0, g1, w_min, w_max)


class AutonomyManagerNode(Node):
    """
    First replication of paper Table II, G1:
      manual target position  <->  automatic target position

    Input:
      /ala/p_pred

    Outputs:
      /ala/G1
      /ala/w_target_user
      /ala/w_target_auto
    """

    def __init__(self):
        super().__init__("autonomy_manager_node")

        self.declare_parameter("p_min", 0.6)
        self.declare_parameter("p_max", 0.8)
        self.declare_parameter("w_min", 0.0)
        self.declare_parameter("w_max", 1.0)

        self.p_min = float(self.get_parameter("p_min").value)
        self.p_max = float(self.get_parameter("p_max").value)
        self.w_min = float(self.get_parameter("w_min").value)
        self.w_max = float(self.get_parameter("w_max").value)

        self.p_pred = 0.0

        self.create_subscription(Float64, "/ala/p_pred", self.p_pred_cb, 10)

        self.pub_g1 = self.create_publisher(Float64, "/ala/G1", 10)
        self.pub_user = self.create_publisher(Float64, "/ala/w_target_user", 10)
        self.pub_auto = self.create_publisher(Float64, "/ala/w_target_auto", 10)

        self.create_timer(0.02, self.step)

    def p_pred_cb(self, msg):
        self.p_pred = clamp(float(msg.data), 0.0, 1.0)

    def step(self):
        g1 = self.p_pred

        w_user = descending_factor(g1, self.p_min, self.p_max, self.w_min, self.w_max)
        w_auto = ascending_factor(g1, self.p_min, self.p_max, self.w_min, self.w_max)

        self.pub_g1.publish(Float64(data=g1))
        self.pub_user.publish(Float64(data=w_user))
        self.pub_auto.publish(Float64(data=w_auto))


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    