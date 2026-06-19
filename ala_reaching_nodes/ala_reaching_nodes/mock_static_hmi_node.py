#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray


class MockStaticHmiNode(Node):
    """
    Controlled mock HMI for RVGF/automatic-motion tests.

    Publishes:
      /ala/object_points
      /ala/x_hmi

    Parameters:
      target_index: 0, 1, or 2
      offset_y: small offset from the selected object
    """

    def __init__(self):
        super().__init__("mock_static_hmi_node")

        self.declare_parameter("target_index", 1)
        self.declare_parameter("offset_y", 0.0)

        self.objects = [
            [0.50, -0.20, 0.30],
            [0.55,  0.00, 0.30],
            [0.50,  0.20, 0.30],
        ]

        self.pub_x = self.create_publisher(Point, "/ala/x_hmi", 10)
        self.pub_objects = self.create_publisher(Float64MultiArray, "/ala/object_points", 10)

        self.create_timer(0.02, self.step)

    def step(self):
        objects_msg = Float64MultiArray()
        objects_msg.data = [v for obj in self.objects for v in obj]
        self.pub_objects.publish(objects_msg)

        idx = int(self.get_parameter("target_index").value)
        idx = max(0, min(2, idx))
        offset_y = float(self.get_parameter("offset_y").value)

        obj = self.objects[idx]

        p = Point()
        p.x = obj[0]
        p.y = obj[1] + offset_y
        p.z = obj[2]

        self.pub_x.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = MockStaticHmiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
