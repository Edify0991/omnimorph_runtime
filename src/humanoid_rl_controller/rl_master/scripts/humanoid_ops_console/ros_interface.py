from __future__ import annotations

from collections.abc import Callable

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Int32

from .constants import TOPIC_MODE_CONTROL, TOPIC_ROBOT_STATE, TOPIC_TELEOP


class HumanoidOpsNode(Node):
    def __init__(self, on_state: Callable[[Float32MultiArray], None]) -> None:
        super().__init__("humanoid_ops_gui")
        qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.teleop_pub = self.create_publisher(Twist, TOPIC_TELEOP, qos_best_effort)
        self.mode_pub = self.create_publisher(Int32, TOPIC_MODE_CONTROL, qos_reliable)
        self.state_sub = self.create_subscription(Float32MultiArray, TOPIC_ROBOT_STATE, on_state, qos_best_effort)

    def publish_mode_control(self, control_word: int) -> None:
        msg = Int32()
        msg.data = int(control_word)
        self.mode_pub.publish(msg)

    def publish_teleop(self, vx: float, vy: float, yaw_rate: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(yaw_rate)
        self.teleop_pub.publish(msg)

    def spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)

