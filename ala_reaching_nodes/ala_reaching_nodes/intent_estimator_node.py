#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Int32, Float64MultiArray


class IntentEstimatorNode(Node):
    """
    Replicates the intent estimator from Iregui et al. 2021, Section VI.

    Inputs:
      /ala/x_hmi          geometry_msgs/Point
      /ala/object_points  std_msgs/Float64MultiArray
                          flattened [x1,y1,z1,x2,y2,z2,...]

    Outputs:
      /ala/x_pred         geometry_msgs/Point
      /ala/object_pred    std_msgs/Int32
      /ala/p_pred         std_msgs/Float64
    """

    def __init__(self):
        super().__init__("intent_estimator_node")

        self.declare_parameter("sigma", 0.20)
        self.declare_parameter("p_none", 0.5)
        self.declare_parameter("d_min", 0.05)
        self.declare_parameter("d_max", 3.0)
        self.declare_parameter("use_xy_only", True)

        self.sigma = float(self.get_parameter("sigma").value)
        self.p_none = float(self.get_parameter("p_none").value)
        self.d_min = float(self.get_parameter("d_min").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.use_xy_only = bool(self.get_parameter("use_xy_only").value)

        self.x_hmi = None
        self.objects = np.array(
            [
                [0.50, -0.20, 0.30],
                [0.55,  0.00, 0.30],
                [0.50,  0.20, 0.30],
            ],
            dtype=float,
        )

        self.create_subscription(Point, "/ala/x_hmi", self.x_hmi_cb, 10)
        self.create_subscription(Float64MultiArray, "/ala/object_points", self.objects_cb, 10)

        self.pub_x_pred = self.create_publisher(Point, "/ala/x_pred", 10)
        self.pub_obj_pred = self.create_publisher(Int32, "/ala/object_pred", 10)
        self.pub_p_pred = self.create_publisher(Float64, "/ala/p_pred", 10)

        self.create_timer(0.02, self.step)

    def x_hmi_cb(self, msg: Point):
        self.x_hmi = np.array([msg.x, msg.y, msg.z], dtype=float)

    def objects_cb(self, msg: Float64MultiArray):
        data = np.array(msg.data, dtype=float)
        if data.size % 3 != 0:
            self.get_logger().warn("Ignoring /ala/object_points: length must be multiple of 3.")
            return
        self.objects = data.reshape((-1, 3))

    def gaussian_score(self, x, mu):
        diff = x - mu
        dm2 = float(np.dot(diff, diff) / (self.sigma * self.sigma))
        return math.exp(-0.5 * dm2), math.sqrt(dm2)

    def p_x_given_none(self, dm):
        dsat = max(self.d_min, min(self.d_max, dm))
        return (dsat - self.d_min) / max(1e-9, (self.d_max - self.d_min))

    def step(self):
        if self.x_hmi is None or self.objects.size == 0:
            return

        if self.use_xy_only:
            x = self.x_hmi[:2]
            objs = self.objects[:, :2]
        else:
            x = self.x_hmi
            objs = self.objects

        n = objs.shape[0]
        p_obj_prior = (1.0 - self.p_none) / max(1, n)

        scores = []
        dms = []
        for obj in objs:
            score, dm = self.gaussian_score(x, obj)
            scores.append(score)
            dms.append(dm)

        best_idx = int(np.argmax(scores))
        p_none_likelihood = self.p_x_given_none(dms[best_idx])

        numerator = scores[best_idx] * p_obj_prior
        denominator = p_none_likelihood * self.p_none
        denominator += sum(s * p_obj_prior for s in scores)

        p_pred = numerator / max(1e-12, denominator)

        x_pred = self.objects[best_idx]
        msg = Point()
        msg.x = float(x_pred[0])
        msg.y = float(x_pred[1])
        msg.z = float(x_pred[2])

        self.pub_x_pred.publish(msg)
        self.pub_obj_pred.publish(Int32(data=best_idx))
        self.pub_p_pred.publish(Float64(data=float(p_pred)))


def main(args=None):
    rclpy.init(args=args)
    node = IntentEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    