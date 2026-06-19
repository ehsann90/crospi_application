#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class TargetLatchNode(Node):
    """
    Publishes /ala/active_target.

    If auto_cmd.x <= 0.5:
        active_target follows blended_target.

    When auto_cmd.x rises above 0.5:
        latch the current blended_target and hold it until auto is disabled.
    """

    def __init__(self):
        super().__init__("target_latch_node")

        self.blended = Point(x=0.50, y=0.0, z=0.30)
        self.active = Point(x=0.50, y=0.0, z=0.30)

        self.auto_enabled = False
        self.was_auto_enabled = False

        self.create_subscription(Point, "/ala/blended_target", self.blended_cb, 10)
        self.create_subscription(Point, "/ala/auto_cmd", self.auto_cb, 10)

        self.pub = self.create_publisher(Point, "/ala/active_target", 10)
        self.create_timer(0.02, self.step)

    def blended_cb(self, msg):
        self.blended = msg

    def auto_cb(self, msg):
        self.auto_enabled = msg.x > 0.5

    def copy_point(self, p):
        return Point(x=float(p.x), y=float(p.y), z=float(p.z))

    def step(self):
        if self.auto_enabled and not self.was_auto_enabled:
            self.active = self.copy_point(self.blended)
            self.get_logger().info(
                f"Latched target: x={self.active.x:.3f}, y={self.active.y:.3f}, z={self.active.z:.3f}"
            )

        if not self.auto_enabled:
            self.active = self.copy_point(self.blended)

        self.was_auto_enabled = self.auto_enabled
        self.pub.publish(self.active)


def main(args=None):
    rclpy.init(args=args)
    node = TargetLatchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
