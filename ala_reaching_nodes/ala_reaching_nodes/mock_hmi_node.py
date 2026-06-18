#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class MockHmiNode(Node):
    """
    Publishes:
      /ala/x_hmi
      /ala/object_points

    This lets us reproduce the paper's Fig. 6-like autonomy regions
    before connecting SpaceMouse, camera, eye tracker, or Crospi.
    """

    def __init__(self):
        super().__init__("mock_hmi_node")
        self.t = 0.0

        self.pub_x = self.create_publisher(Point, "/ala/x_hmi", 10)
        self.pub_objects = self.create_publisher(Float64MultiArray, "/ala/object_points", 10)

        self.create_timer(0.02, self.step)

    def step(self):
        self.t += 0.02

        objects = Float64MultiArray()
        objects.data = [
            0.50, -0.20, 0.30,
            0.55,  0.00, 0.30,
            0.50,  0.20, 0.30,
        ]
        self.pub_objects.publish(objects)

        p = Point()
        p.x = 0.52
        p.y = 0.28 * math.sin(0.35 * self.t)
        p.z = 0.30
        self.pub_x.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = MockHmiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    