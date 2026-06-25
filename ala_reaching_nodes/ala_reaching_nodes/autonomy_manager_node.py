#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
from crospi_interfaces.msg import Output


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def ramp_up(x, x0, x1):
    if x1 <= x0:
        return 1.0 if x >= x1 else 0.0
    return clamp((x - x0) / (x1 - x0))


def ramp_down(x, x0, x1):
    return 1.0 - ramp_up(x, x0, x1)


class AutonomyManagerNode(Node):
    """
    Paper-aligned autonomy manager.

    G1:
      target selection autonomy
      low  -> x_hmi dominates
      high -> x_pred dominates

    G2:
      automatic motion vs RVGF/manual guidance
      G2 = 0 -> automatic motion
      G2 = 1 -> RVGF/manual guidance
      Internally smoothed, even if auto_cmd is discrete.

    G3:
      distance-to-path arbitration
      G3 = ||p(q) - f_p(s)||
      larger G3 weakens RVGF and s damping.
    """

    def __init__(self):
        super().__init__("autonomy_manager_node")

        # G1 thresholds
        self.declare_parameter("p_min", 0.40)
        self.declare_parameter("p_max", 0.50)

        # G2 smoothing
        self.declare_parameter("g2_tau", 0.35)

        # G3 thresholds relative to rtube
        self.declare_parameter("g3_start_offset", 0.0)
        self.declare_parameter("g3_escape_margin", 0.12)
        self.declare_parameter("rvgf_min_factor", 0.10)

        # G4 
        self.latest_twist_time = 0.0
        self.latest_twist_mag = 0.0
        self.G4 = 0.0

        self.twist_deadband = float(self.declare_parameter("twist_deadband", 0.01).value)
        self.twist_timeout = float(self.declare_parameter("twist_timeout", 0.35).value)
        self.g4_tau = float(self.declare_parameter("g4_tau", 0.20).value)

        # Gains for actual Crospi constraint weights
        self.declare_parameter("user_velocity_gain", 0.6)
        self.declare_parameter("rvgf_gain", 0.25)
        self.declare_parameter("s_damping_gain", 0.02)
        self.declare_parameter("auto_progress_gain", 1.0)
        self.declare_parameter("auto_follow_gain", 1.0)

        self.p_pred = 0.0
        self.auto_cmd = 0.0

        self.distance_to_path = 0.0
        self.rtube = 0.08

        # Normalized G2 state. Start in RVGF/manual mode.
        self.G2 = 1.0

        self.create_subscription(Float64, "/ala/p_pred", self.p_pred_cb, 10)
        self.create_subscription(Point, "/ala/auto_cmd", self.auto_cmd_cb, 10)
        self.create_subscription(Output, "/ala/debug_path", self.debug_path_cb, 10)
        self.create_subscription(Twist, "/spacenav/twist", self.twist_cb, 10)

        self.pub_G1 = self.create_publisher(Float64, "/ala/G1", 10)
        self.pub_G2 = self.create_publisher(Float64, "/ala/G2", 10)
        self.pub_G3 = self.create_publisher(Float64, "/ala/G3", 10)
        self.pub_D3 = self.create_publisher(Float64, "/ala/D3", 10)

        self.pub_w_target_user = self.create_publisher(Float64, "/ala/w_target_user", 10)
        self.pub_w_target_auto = self.create_publisher(Float64, "/ala/w_target_auto", 10)

        # x = user velocity, y = RVGF, z = s damping
        self.pub_motion_weights = self.create_publisher(Point, "/ala/motion_weights", 10)

        # x = auto progress, y = auto follow, z = G2 debug
        self.pub_auto_weights = self.create_publisher(Point, "/ala/auto_weights", 10)

        self.pub_w_user_velocity = self.create_publisher(Float64, "/ala/w_user_velocity", 10)
        self.pub_w_rvgf = self.create_publisher(Float64, "/ala/w_rvgf", 10)
        self.pub_w_s_damping = self.create_publisher(Float64, "/ala/w_s_damping", 10)
        self.pub_w_auto_progress = self.create_publisher(Float64, "/ala/w_auto_progress", 10)
        self.pub_w_auto_follow = self.create_publisher(Float64, "/ala/w_auto_follow", 10)
        self.pub_D3_raw = self.create_publisher(Float64, "/ala/D3_raw", 10)

        self.pub_w_user_velocity = self.create_publisher(Float64, "/ala/w_user_velocity", 10)
        self.pub_w_rvgf = self.create_publisher(Float64, "/ala/w_rvgf", 10)
        self.pub_w_s_damping = self.create_publisher(Float64, "/ala/w_s_damping", 10)
        self.pub_w_auto_progress = self.create_publisher(Float64, "/ala/w_auto_progress", 10)
        self.pub_w_auto_follow = self.create_publisher(Float64, "/ala/w_auto_follow", 10)
        self.pub_D3_raw = self.create_publisher(Float64, "/ala/D3_raw", 10)

        self.pub_w_user_velocity = self.create_publisher(Float64, "/ala/w_user_velocity", 10)
        self.pub_w_rvgf = self.create_publisher(Float64, "/ala/w_rvgf", 10)
        self.pub_w_s_damping = self.create_publisher(Float64, "/ala/w_s_damping", 10)
        self.pub_w_auto_progress = self.create_publisher(Float64, "/ala/w_auto_progress", 10)
        self.pub_w_auto_follow = self.create_publisher(Float64, "/ala/w_auto_follow", 10)
        self.pub_D3_raw = self.create_publisher(Float64, "/ala/D3_raw", 10)

        self.pub_G4 = self.create_publisher(Float64, "/ala/G4", 10)
        self.pub_interaction_gate = self.create_publisher(Float64, "/ala/interaction_gate", 10)

        self.create_timer(0.02, self.step)

    def p_pred_cb(self, msg):
        self.p_pred = float(msg.data)

    def auto_cmd_cb(self, msg):
        self.auto_cmd = float(msg.x)

    def debug_path_cb(self, msg):
        try:
            if len(msg.data) > 8:
                self.distance_to_path = float(msg.data[7])
                self.rtube = max(1e-6, float(msg.data[8]))
        except Exception as exc:
            self.get_logger().warn(f"Could not parse /ala/debug_path: {exc}")

    def twist_cb(self, msg: Twist):
        lin = msg.linear
        ang = msg.angular

        self.latest_twist_mag = math.sqrt(
            lin.x * lin.x +
            lin.y * lin.y +
            lin.z * lin.z +
            0.25 * (ang.x * ang.x + ang.y * ang.y + ang.z * ang.z)
        )

        self.latest_twist_time = time.time()

    def publish_float(self, pub, value):
        msg = Float64()
        msg.data = float(value)
        pub.publish(msg)

    def step(self):
        dt = 0.02

        p_min = float(self.get_parameter("p_min").value)
        p_max = float(self.get_parameter("p_max").value)

        g2_tau = max(1e-6, float(self.get_parameter("g2_tau").value))

        g3_start_offset = float(self.get_parameter("g3_start_offset").value)
        g3_escape_margin = max(1e-6, float(self.get_parameter("g3_escape_margin").value))
        rvgf_min_factor = clamp(float(self.get_parameter("rvgf_min_factor").value))
        rvgf_min_factor = clamp(float(self.get_parameter("rvgf_min_factor").value))
        rvgf_min_factor = clamp(float(self.get_parameter("rvgf_min_factor").value))

        user_velocity_gain = float(self.get_parameter("user_velocity_gain").value)
        rvgf_gain = float(self.get_parameter("rvgf_gain").value)
        s_damping_gain = float(self.get_parameter("s_damping_gain").value)
        auto_progress_gain = float(self.get_parameter("auto_progress_gain").value)
        auto_follow_gain = float(self.get_parameter("auto_follow_gain").value)

        # G1: confidence-based target arbitration.
        G1 = ramp_up(self.p_pred, p_min, p_max)
        D1 = 1.0 - G1
        A1 = G1

        # G2: paper direction.
        # auto_cmd.x = 1 means automatic mode, therefore G2 target = 0.
        # auto_cmd.x = 0 means RVGF/manual mode, therefore G2 target = 1.
        G2_target = 0.0 if self.auto_cmd > 0.5 else 1.0
        alpha = clamp(dt / g2_tau)
        self.G2 += alpha * (G2_target - self.G2)
        self.G2 = clamp(self.G2)

        A2 = self.G2
        D2 = 1.0 - self.G2

        # G3: raw arbitration variable is distance_to_path.
        G3 = max(0.0, self.distance_to_path)

        # Descending factor for G3: RVGF fades as distance grows.
        d0 = max(0.0, self.rtube + g3_start_offset)
        d1 = d0 + g3_escape_margin
        D3_raw = ramp_down(G3, d0, d1)
        rvgf_min_factor = float(self.get_parameter("rvgf_min_factor").value)
        D3 = rvgf_min_factor + (1.0 - rvgf_min_factor) * D3_raw

        now = time.time()
        twist_recent = (now - self.latest_twist_time) <= self.twist_timeout
        twist_active = twist_recent and (self.latest_twist_mag > self.twist_deadband)

        G4_target = 1.0 if twist_active else 0.0

        alpha4 = dt / (self.g4_tau + dt)
        self.G4 += alpha4 * (G4_target - self.G4)

        manual_gate = self.G4

        # Target weights: Table II G1 affects w4 and w9.
        w_target_user = D1
        w_target_auto = A1

        # Motion/RVGF weights: Table II G2 and G3.
        w_user_velocity = user_velocity_gain * A2 * manual_gate
        w_rvgf = rvgf_gain * A2 * D3 * manual_gate
        w_s_damping = s_damping_gain * A2 * D3 * manual_gate

        # Automatic approach weights: Table II G2 descending factors.
        w_auto_progress = auto_progress_gain * D2
        w_auto_follow = auto_follow_gain * D2

        self.publish_float(self.pub_G1, G1)
        self.publish_float(self.pub_G2, self.G2)
        self.publish_float(self.pub_G3, G3)
        self.publish_float(self.pub_D3, D3)
        self.publish_float(self.pub_D3_raw, D3_raw)

        self.publish_float(self.pub_w_target_user, w_target_user)
        self.publish_float(self.pub_w_target_auto, w_target_auto)

        self.publish_float(self.pub_w_user_velocity, w_user_velocity)
        self.publish_float(self.pub_w_rvgf, w_rvgf)
        self.publish_float(self.pub_w_s_damping, w_s_damping)
        self.publish_float(self.pub_w_auto_progress, w_auto_progress)
        self.publish_float(self.pub_w_auto_follow, w_auto_follow)

        self.publish_float(self.pub_G4, self.G4)
        self.publish_float(self.pub_interaction_gate, manual_gate)

        motion_msg = Point()
        motion_msg.x = float(w_user_velocity)
        motion_msg.y = float(w_rvgf)
        motion_msg.z = float(w_s_damping)
        self.pub_motion_weights.publish(motion_msg)

        auto_msg = Point()
        auto_msg.x = float(w_auto_progress)
        auto_msg.y = float(w_auto_follow)
        auto_msg.z = float(self.G2)
        self.pub_auto_weights.publish(auto_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
