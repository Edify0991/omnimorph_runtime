from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import rclpy
from PyQt5.QtWidgets import QApplication
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

from .config import find_default_config, load_gui_config
from .constants import TOPIC_CAMERA_COLOR, TOPIC_CAMERA_DEPTH, TOPIC_CAMERA_FEATURES
from .main_window import MainWindow
from .ros_interface import HumanoidOpsNode


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Qt operator console for the JC01 humanoid deploy runtime.")
    parser.add_argument("--config", type=Path, default=find_default_config(), help="Path to rl_cfg.yaml")
    parser.add_argument("--camera-topic", default=TOPIC_CAMERA_COLOR, help="ROS topic for color image preview")
    parser.add_argument("--depth-topic", default=TOPIC_CAMERA_DEPTH, help="ROS topic for depth image preview")
    parser.add_argument(
        "--camera-feature-topic",
        default=TOPIC_CAMERA_FEATURES,
        help="ROS topic for standardized camera feature preview",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(list(argv or []))
    config = load_gui_config(args.config)

    app = QApplication([sys.argv[0]])
    window_ref: dict[str, MainWindow] = {}

    rclpy.init(args=None)

    def on_state(msg: Float32MultiArray) -> None:
        window = window_ref.get("window")
        if window is not None:
            window.on_state_msg(msg)

    def on_camera(msg: Image) -> None:
        window = window_ref.get("window")
        if window is not None:
            window.on_camera_msg(msg)

    def on_depth(msg: Image) -> None:
        window = window_ref.get("window")
        if window is not None:
            window.on_depth_msg(msg)

    def on_camera_features(msg: Float32MultiArray) -> None:
        window = window_ref.get("window")
        if window is not None:
            window.on_camera_features_msg(msg)

    node = HumanoidOpsNode(
        on_state,
        on_camera=on_camera,
        on_depth=on_depth,
        on_camera_features=on_camera_features,
        camera_topic=args.camera_topic,
        depth_topic=args.depth_topic,
        camera_feature_topic=args.camera_feature_topic,
    )
    window = MainWindow(node, config)
    window_ref["window"] = window
    window.statusBar().showMessage(f"Config: {config.path}", 5000)
    window.show()

    try:
        return int(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()
