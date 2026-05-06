from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Optional

import rclpy
from PyQt5.QtWidgets import QApplication
from std_msgs.msg import Float32MultiArray

from .config import find_default_config, load_gui_config
from .main_window import MainWindow
from .ros_interface import HumanoidOpsNode


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Qt operator console for the JC01 humanoid deploy runtime.")
    parser.add_argument("--config", type=Path, default=find_default_config(), help="Path to rl_cfg.yaml")
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

    node = HumanoidOpsNode(on_state)
    window = MainWindow(node, config)
    window_ref["window"] = window
    window.statusBar().showMessage(f"Config: {config.path}", 5000)
    window.show()

    try:
        return int(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()
