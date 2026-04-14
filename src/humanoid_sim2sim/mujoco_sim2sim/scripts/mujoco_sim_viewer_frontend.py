#!/usr/bin/python3
from __future__ import annotations

import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

_NUMPY_IMPORT_ERROR = None
try:
    import numpy as np
except Exception as exc:
    np = None  # type: ignore[assignment]
    _NUMPY_IMPORT_ERROR = exc

_MUJOCO_IMPORT_ERROR = None
try:
    import mujoco
    import mujoco.viewer
except Exception as exc:
    mujoco = None  # type: ignore[assignment]
    _MUJOCO_IMPORT_ERROR = exc

K_VIEWER_FRAME_TOPIC = "/humanoid/sim2sim/mujoco_viewer_frame"
K_VIEWER_FRAME_MAGIC = 260413.0
K_VIEWER_FRAME_VERSION = 1.0
K_HEADER_LEN = 8
K_VIEWER_INSPECTOR_TOPIC = "/humanoid/sim2sim/mujoco_viewer_inspector"


class MujocoViewerFrontend(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_sim_viewer_frontend")

        if np is None:
            raise RuntimeError(f"numpy import failed: {_NUMPY_IMPORT_ERROR}")
        if mujoco is None:
            raise RuntimeError(f"mujoco python import failed: {_MUJOCO_IMPORT_ERROR}")

        self.declare_parameter("model_path", "")
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("viewer_fps", 60.0)
        self.declare_parameter("viewer_title", "MuJoCo Python Viewer Frontend")
        self.declare_parameter("show_left_ui", True)
        self.declare_parameter("show_right_ui", True)
        self.declare_parameter("viewer_frame_topic", K_VIEWER_FRAME_TOPIC)
        self.declare_parameter("viewer_inspector_topic", K_VIEWER_INSPECTOR_TOPIC)
        self.declare_parameter("stale_frame_warn_sec", 1.0)
        self.declare_parameter("inspector_log_hz", 2.0)

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.enable_viewer = self.get_parameter("enable_viewer").get_parameter_value().bool_value
        self.viewer_fps = max(1.0, self.get_parameter("viewer_fps").get_parameter_value().double_value)
        self.viewer_title = self.get_parameter("viewer_title").get_parameter_value().string_value
        self.show_left_ui = self.get_parameter("show_left_ui").get_parameter_value().bool_value
        self.show_right_ui = self.get_parameter("show_right_ui").get_parameter_value().bool_value
        self.viewer_frame_topic = self.get_parameter("viewer_frame_topic").get_parameter_value().string_value or K_VIEWER_FRAME_TOPIC
        self.viewer_inspector_topic = self.get_parameter("viewer_inspector_topic").get_parameter_value().string_value or K_VIEWER_INSPECTOR_TOPIC
        self.stale_frame_warn_sec = max(0.1, self.get_parameter("stale_frame_warn_sec").get_parameter_value().double_value)
        self.inspector_log_hz = max(0.2, self.get_parameter("inspector_log_hz").get_parameter_value().double_value)

        if not self.model_path:
            raise RuntimeError("parameter 'model_path' is empty")

        self.model = self._load_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.frame_lock = threading.Lock()
        self.pending_qpos: Optional[np.ndarray] = None
        self.pending_qvel: Optional[np.ndarray] = None
        self.pending_ctrl: Optional[np.ndarray] = None
        self.pending_time: float = 0.0
        self.last_frame_wall_sec: float = 0.0
        self.last_stale_warn_sec: float = 0.0
        self.frame_count: int = 0
        self.latest_inspector_line: str = ""
        self.last_inspector_wall_sec: float = 0.0
        self.last_inspector_log_wall_sec: float = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.frame_sub = self.create_subscription(Float32MultiArray, self.viewer_frame_topic, self._on_frame, qos)
        self.inspector_sub = self.create_subscription(String, self.viewer_inspector_topic, self._on_inspector, qos)

        self.get_logger().info(
            "python viewer frontend ready: "
            f"model={self.model_path} frame_topic={self.viewer_frame_topic} inspector_topic={self.viewer_inspector_topic} "
            f"viewer={'on' if self.enable_viewer else 'off'} fps={self.viewer_fps:.1f}"
        )
        if self.viewer_title:
            self.get_logger().info(
                f"viewer_title='{self.viewer_title}' is informational only; mujoco.viewer.launch_passive controls the actual window title."
            )

    def _load_model(self, model_path: str):
        if model_path.endswith(".mjb"):
            model = mujoco.MjModel.from_binary_path(model_path)
        else:
            model = mujoco.MjModel.from_xml_path(model_path)
        return model

    def _on_frame(self, msg: Float32MultiArray) -> None:
        data = msg.data
        if len(data) < K_HEADER_LEN:
            self.get_logger().warn("viewer frame too short, ignore")
            return

        magic = float(data[0])
        version = float(data[1])
        nq = int(round(float(data[2])))
        nv = int(round(float(data[3])))
        nu = int(round(float(data[4])))
        sim_time = float(data[5])

        if abs(magic - K_VIEWER_FRAME_MAGIC) > 0.5:
            self.get_logger().warn(f"unexpected viewer frame magic={magic}, ignore")
            return
        if abs(version - K_VIEWER_FRAME_VERSION) > 0.5:
            self.get_logger().warn(f"unsupported viewer frame version={version}, ignore")
            return

        expected_size = K_HEADER_LEN + nq + nv + nu
        if len(data) != expected_size:
            self.get_logger().warn(
                f"viewer frame size mismatch: got={len(data)} expected={expected_size} (nq={nq}, nv={nv}, nu={nu})"
            )
            return
        if nq != self.model.nq or nv != self.model.nv or nu != self.model.nu:
            self.get_logger().warn(
                f"viewer frame model mismatch: frame(nq={nq}, nv={nv}, nu={nu}) != local(nq={self.model.nq}, nv={self.model.nv}, nu={self.model.nu})"
            )
            return

        offset = K_HEADER_LEN
        qpos = np.asarray(data[offset:offset + nq], dtype=np.float64)
        offset += nq
        qvel = np.asarray(data[offset:offset + nv], dtype=np.float64)
        offset += nv
        ctrl = np.asarray(data[offset:offset + nu], dtype=np.float64)

        with self.frame_lock:
            self.pending_qpos = qpos
            self.pending_qvel = qvel
            self.pending_ctrl = ctrl
            self.pending_time = sim_time
            self.last_frame_wall_sec = time.monotonic()
            self.frame_count += 1

    def _apply_pending_frame(self) -> bool:
        with self.frame_lock:
            if self.pending_qpos is None or self.pending_qvel is None or self.pending_ctrl is None:
                return False
            qpos = self.pending_qpos
            qvel = self.pending_qvel
            ctrl = self.pending_ctrl
            sim_time = self.pending_time
            self.pending_qpos = None
            self.pending_qvel = None
            self.pending_ctrl = None

        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.ctrl[:] = ctrl
        self.data.time = sim_time
        mujoco.mj_forward(self.model, self.data)
        return True

    def _on_inspector(self, msg: String) -> None:
        self.latest_inspector_line = msg.data.strip()
        self.last_inspector_wall_sec = time.monotonic()

    def _warn_if_stale(self) -> None:
        now = time.monotonic()
        if self.last_frame_wall_sec <= 0.0:
            if (now - self.last_stale_warn_sec) >= self.stale_frame_warn_sec:
                self.get_logger().warn("waiting for first fused viewer frame ...")
                self.last_stale_warn_sec = now
            return
        if (now - self.last_frame_wall_sec) >= self.stale_frame_warn_sec and (now - self.last_stale_warn_sec) >= self.stale_frame_warn_sec:
            self.get_logger().warn(
                f"viewer frame stream stale for {now - self.last_frame_wall_sec:.2f}s on topic {self.viewer_frame_topic}"
            )
            self.last_stale_warn_sec = now

    def _log_inspector_periodically(self) -> None:
        now = time.monotonic()
        period = 1.0 / self.inspector_log_hz
        if (now - self.last_inspector_log_wall_sec) < period:
            return
        self.last_inspector_log_wall_sec = now

        if self.last_frame_wall_sec > 0.0:
            frame_age = now - self.last_frame_wall_sec
            frame_text = f"frame_age={frame_age:.2f}s frames={self.frame_count}"
        else:
            frame_text = "frame_age=inf waiting_frame=1"

        if self.last_inspector_wall_sec > 0.0:
            inspector_age = now - self.last_inspector_wall_sec
            inspector_prefix = f"inspector_age={inspector_age:.2f}s"
            if self.latest_inspector_line:
                self.get_logger().info(f"{inspector_prefix} {frame_text} | {self.latest_inspector_line}")
                return

        self.get_logger().info(f"inspector_age=inf {frame_text} | waiting for fused inspector summary ...")

    def run(self) -> None:
        if not self.enable_viewer:
            self.get_logger().info("enable_viewer=false, enter headless inspector mode.")
            render_period = 1.0 / self.viewer_fps
            while rclpy.ok():
                self._apply_pending_frame()
                self._warn_if_stale()
                self._log_inspector_periodically()
                time.sleep(render_period)
            return

        render_period = 1.0 / self.viewer_fps
        with mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=self.show_left_ui,
            show_right_ui=self.show_right_ui,
        ) as viewer:
            while rclpy.ok() and viewer.is_running():
                updated = self._apply_pending_frame()
                if updated:
                    viewer.sync()
                else:
                    self._warn_if_stale()
                    viewer.sync()
                self._log_inspector_periodically()
                time.sleep(render_period)


def main() -> None:
    rclpy.init()
    node = MujocoViewerFrontend()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        node.run()
    finally:
        if sys.stdout and not sys.stdout.closed:
            sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
