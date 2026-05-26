#!/usr/bin/python3
from __future__ import annotations

import select
import sys
import termios
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP_TEXT = """
Keyboard teleop
  w/s: forward/back
  a/d: strafe left/right
  q/e: yaw left/right
  space or x: zero velocity
  h or ?: show help
  Ctrl-C: quit
"""


@dataclass
class TerminalState:
    fd: int
    attrs: list


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("omnimorph_keyboard_teleop")

        self.declare_parameter("teleop_topic", "/omnimorph/rl/teleop")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("linear_step", 0.05)
        self.declare_parameter("lateral_step", 0.05)
        self.declare_parameter("yaw_step", 0.05)
        self.declare_parameter("max_linear", 0.6)
        self.declare_parameter("max_lateral", 0.4)
        self.declare_parameter("max_yaw", 0.8)

        publish_hz = float(self.get_parameter("publish_hz").value)
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.lateral_step = float(self.get_parameter("lateral_step").value)
        self.yaw_step = float(self.get_parameter("yaw_step").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_lateral = float(self.get_parameter("max_lateral").value)
        self.max_yaw = float(self.get_parameter("max_yaw").value)

        teleop_topic = str(self.get_parameter("teleop_topic").value)
        self.teleop_pub = self.create_publisher(Twist, teleop_topic, 10)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._terminal: Optional[TerminalState] = None

        if not sys.stdin.isatty():
            raise RuntimeError("keyboard_teleop.py must run in an interactive terminal")

        fd = sys.stdin.fileno()
        self._terminal = TerminalState(fd=fd, attrs=termios.tcgetattr(fd))
        tty.setcbreak(fd)

        self.timer = self.create_timer(1.0 / max(publish_hz, 1.0), self._tick)
        self.get_logger().info(HELP_TEXT)
        self._log_state()

    def destroy_node(self) -> bool:
        self._publish_zero()
        if self._terminal is not None:
            termios.tcsetattr(self._terminal.fd, termios.TCSADRAIN, self._terminal.attrs)
            self._terminal = None
        return super().destroy_node()

    def _tick(self) -> None:
        self._read_keys()
        self._publish_twist()

    def _read_keys(self) -> None:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1)
            if key:
                self._handle_key(key)

    def _handle_key(self, key: str) -> None:
        if key == "w":
            self.vx = self._clamp(self.vx + self.linear_step, -self.max_linear, self.max_linear)
        elif key == "s":
            self.vx = self._clamp(self.vx - self.linear_step, -self.max_linear, self.max_linear)
        elif key == "a":
            self.vy = self._clamp(self.vy + self.lateral_step, -self.max_lateral, self.max_lateral)
        elif key == "d":
            self.vy = self._clamp(self.vy - self.lateral_step, -self.max_lateral, self.max_lateral)
        elif key == "q":
            self.wz = self._clamp(self.wz + self.yaw_step, -self.max_yaw, self.max_yaw)
        elif key == "e":
            self.wz = self._clamp(self.wz - self.yaw_step, -self.max_yaw, self.max_yaw)
        elif key in (" ", "x"):
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
        elif key in ("h", "?"):
            self.get_logger().info(HELP_TEXT)
        elif key == "\x03":
            raise KeyboardInterrupt
        else:
            return
        self._log_state()

    def _publish_twist(self) -> None:
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.teleop_pub.publish(msg)

    def _publish_zero(self) -> None:
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self._publish_twist()

    def _log_state(self) -> None:
        self.get_logger().info(
            f"cmd vx={self.vx:.2f} vy={self.vy:.2f} wz={self.wz:.2f}"
        )

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return min(max(value, lo), hi)


def main() -> None:
    rclpy.init()
    node: Optional[KeyboardTeleop] = None
    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
