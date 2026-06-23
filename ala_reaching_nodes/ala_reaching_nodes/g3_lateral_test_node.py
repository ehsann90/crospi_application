#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
from crospi_interfaces.msg import Output


class G3LateralTestNode(Node):
    """
    Automated G3 test.

    Sequence:
      1. force manual/RVGF mode
      2. push laterally away from current path
      3. stop
      4. push back
      5. stop

    It uses /ala/debug_path to estimate the path direction:
      data[1:4] = path point
      data[4:7] = target point
    """

    def __init__(self):
        super().__init__("g3_lateral_test_node")

        self.declare_parameter("speed", 0.55)
        self.declare_parameter("push_time", 25.0)
        self.declare_parameter("stop_time", 1.0)

        self.path_x = 0.0
        self.path_y = 0.0
        self.target_x = 1.0
        self.target_y = 0.0

        self.g3 = 0.0
        self.d3 = 1.0
        self.w_rvgf = 0.0

        self.pub_auto = self.create_publisher(Point, "/ala/auto_cmd", 10)
        self.pub_twist = self.create_publisher(Twist, "/spacenav/twist", 10)

        self.create_subscription(Output, "/ala/debug_path", self.debug_cb, 10)
        self.create_subscription(Float64, "/ala/G3", self.g3_cb, 10)
        self.create_subscription(Float64, "/ala/D3", self.d3_cb, 10)
        self.create_subscription(Float64, "/ala/w_rvgf", self.w_rvgf_cb, 10)

        self.t = 0.0
        self.dt = 0.02
        self.create_timer(self.dt, self.step)

    def debug_cb(self, msg):
        if len(msg.data) > 6:
            self.path_x = float(msg.data[1])
            self.path_y = float(msg.data[2])
            self.target_x = float(msg.data[4])
            self.target_y = float(msg.data[5])

    def g3_cb(self, msg):
        self.g3 = float(msg.data)

    def d3_cb(self, msg):
        self.d3 = float(msg.data)

    def w_rvgf_cb(self, msg):
        self.w_rvgf = float(msg.data)

    def lateral_direction(self):
        dx = self.target_x - self.path_x
        dy = self.target_y - self.path_y
        n = math.sqrt(dx * dx + dy * dy)

        if n < 1e-6:
            return 0.0, 1.0

        dx /= n
        dy /= n

        # Perpendicular direction in xy-plane.
        return -dy, dx

    def publish_auto_off(self):
        p = Point()
        p.x = 0.0
        self.pub_auto.publish(p)

    def publish_twist(self, vx, vy, vz=0.0):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.linear.z = float(vz)
        self.pub_twist.publish(msg)

    def step(self):
        self.t += self.dt

        speed = float(self.get_parameter("speed").value)
        push_time = float(self.get_parameter("push_time").value)
        stop_time = float(self.get_parameter("stop_time").value)

        self.publish_auto_off()

        lx, ly = self.lateral_direction()

        if self.t < push_time:
            vx, vy = speed * lx, speed * ly
            phase = "push away"
        elif self.t < push_time + stop_time:
            vx, vy = 0.0, 0.0
            phase = "stop"
        elif self.t < 2.0 * push_time + stop_time:
            vx, vy = -speed * lx, -speed * ly
            phase = "push back"
        elif self.t < 2.0 * push_time + 2.0 * stop_time:
            vx, vy = 0.0, 0.0
            phase = "final stop"
        else:
            self.publish_twist(0.0, 0.0, 0.0)
            self.get_logger().info("G3 test finished.")
            rclpy.shutdown()
            return

        self.publish_twist(vx, vy, 0.0)

        if int(self.t * 10) % 10 == 0:
            self.get_logger().info(
                f"{phase}: G3={self.g3:.4f}, D3={self.d3:.3f}, w_rvgf={self.w_rvgf:.3f}, "
                f"twist=({vx:.3f}, {vy:.3f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = G3LateralTestNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
