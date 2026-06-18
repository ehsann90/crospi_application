#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64


class TargetBlenderNode(Node):
    """
    Blends manual target and inferred automatic target:

      blended = (w_user*x_hmi + w_auto*x_pred) / (w_user + w_auto)

    Inputs:
      /ala/x_hmi
      /ala/x_pred
      /ala/w_target_user
      /ala/w_target_auto

    Output:
      /ala/blended_target
    """

    def __init__(self):
        super().__init__("target_blender_node")

        self.x_hmi = Point(x=0.50, y=0.00, z=0.30)
        self.x_pred = Point(x=0.50, y=0.00, z=0.30)
        self.w_user = 1.0
        self.w_auto = 0.0

        self.create_subscription(Point, "/ala/x_hmi", self.x_hmi_cb, 10)
        self.create_subscription(Point, "/ala/x_pred", self.x_pred_cb, 10)
        self.create_subscription(Float64, "/ala/w_target_user", self.w_user_cb, 10)
        self.create_subscription(Float64, "/ala/w_target_auto", self.w_auto_cb, 10)

        self.pub = self.create_publisher(Point, "/ala/blended_target", 10)
        self.create_timer(0.02, self.step)

    def x_hmi_cb(self, msg):
        self.x_hmi = msg

    def x_pred_cb(self, msg):
        self.x_pred = msg

    def w_user_cb(self, msg):
        self.w_user = float(msg.data)

    def w_auto_cb(self, msg):
        self.w_auto = float(msg.data)

    def step(self):
        denom = max(1e-9, self.w_user + self.w_auto)

        out = Point()
        out.x = (self.w_user * self.x_hmi.x + self.w_auto * self.x_pred.x) / denom
        out.y = (self.w_user * self.x_hmi.y + self.w_auto * self.x_pred.y) / denom
        out.z = (self.w_user * self.x_hmi.z + self.w_auto * self.x_pred.z) / denom

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TargetBlenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
