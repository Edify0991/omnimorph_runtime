from __future__ import annotations

from collections.abc import Callable

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Int32

from .constants import TOPIC_MODE_CONTROL, TOPIC_ROBOT_STATE, TOPIC_TELEOP


class OmnimorphOpsNode(Node):
    def __init__(
        self,
        on_state: Callable[[Float32MultiArray], None],
        *,
        on_camera: Callable[[Image], None] | None = None,
        on_depth: Callable[[Image], None] | None = None,
        on_camera_features: Callable[[Float32MultiArray], None] | None = None,
        camera_topic: str = "",
        depth_topic: str = "",
        camera_feature_topic: str = "",
    ) -> None:
        super().__init__("omnimorph_ops_gui")
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
        self.camera_sub = None
        self.depth_sub = None
        self.camera_feature_sub = None

        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        if on_camera is not None and camera_topic:
            self.camera_sub = self.create_subscription(Image, camera_topic, on_camera, qos_sensor)
        if on_depth is not None and depth_topic:
            self.depth_sub = self.create_subscription(Image, depth_topic, on_depth, qos_sensor)
        if on_camera_features is not None and camera_feature_topic:
            self.camera_feature_sub = self.create_subscription(
                Float32MultiArray,
                camera_feature_topic,
                on_camera_features,
                qos_sensor,
            )

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
