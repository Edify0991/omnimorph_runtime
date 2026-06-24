#!/usr/bin/python3
from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

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

_IMAGEIO_IMPORT_ERROR = None
try:
    import imageio.v2 as imageio
except Exception as exc:
    try:
        import imageio  # type: ignore[no-redef]
    except Exception:
        imageio = None  # type: ignore[assignment]
        _IMAGEIO_IMPORT_ERROR = exc

K_VIEWER_FRAME_TOPIC = "/omnimorph/sim2sim/mujoco_viewer_frame"
K_VIEWER_FRAME_MAGIC = 260413.0
K_VIEWER_FRAME_VERSION = 2.0
K_HEADER_LEN = 8
K_VIEWER_INSPECTOR_TOPIC = "/omnimorph/sim2sim/mujoco_viewer_inspector"


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
        self.declare_parameter("follow_robot", False)
        self.declare_parameter("follow_body_name", "Body")
        self.declare_parameter("follow_distance", 3.0)
        self.declare_parameter("follow_azimuth", 180.0)
        self.declare_parameter("follow_elevation", -20.0)
        self.declare_parameter("follow_lookat_offset", [0.0, 0.0, 0.8])
        self.declare_parameter("enable_video_recording", False)
        self.declare_parameter("video_output_dir", "src/omnimorph_sim2sim/mujoco_sim2sim/videos")
        self.declare_parameter("video_output_path", "")
        self.declare_parameter("video_fps", 60.0)
        self.declare_parameter("video_width", 1280)
        self.declare_parameter("video_height", 720)

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
        self.follow_robot = self.get_parameter("follow_robot").get_parameter_value().bool_value
        self.follow_body_name = self.get_parameter("follow_body_name").get_parameter_value().string_value or "Body"
        self.follow_distance = max(0.1, self.get_parameter("follow_distance").get_parameter_value().double_value)
        self.follow_azimuth = self.get_parameter("follow_azimuth").get_parameter_value().double_value
        self.follow_elevation = self.get_parameter("follow_elevation").get_parameter_value().double_value
        self.follow_lookat_offset = self._read_vec3_parameter("follow_lookat_offset", [0.0, 0.0, 0.8])
        self.enable_video_recording = self.get_parameter("enable_video_recording").get_parameter_value().bool_value
        self.video_output_dir = self.get_parameter("video_output_dir").get_parameter_value().string_value
        self.video_output_path = self.get_parameter("video_output_path").get_parameter_value().string_value
        self.video_fps = max(1.0, self.get_parameter("video_fps").get_parameter_value().double_value)
        self.video_width = max(64, int(self.get_parameter("video_width").value))
        self.video_height = max(64, int(self.get_parameter("video_height").value))

        if not self.model_path:
            raise RuntimeError("parameter 'model_path' is empty")
        if self.enable_video_recording and imageio is None:
            raise RuntimeError(f"imageio import failed: {_IMAGEIO_IMPORT_ERROR}")

        self.model = self._load_model(self.model_path)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.frame_lock = threading.Lock()
        self.pending_qpos: Optional[np.ndarray] = None
        self.pending_qvel: Optional[np.ndarray] = None
        self.pending_ctrl: Optional[np.ndarray] = None
        self.pending_overlay = None
        self.current_overlay = None
        self.pending_time: float = 0.0
        self.last_frame_wall_sec: float = 0.0
        self.last_stale_warn_sec: float = 0.0
        self.frame_count: int = 0
        self.latest_inspector_line: str = ""
        self.last_inspector_wall_sec: float = 0.0
        self.last_inspector_log_wall_sec: float = 0.0
        self.follow_body_id = self._resolve_body_id(self.follow_body_name) if self.follow_robot else -1
        self.video_writer = None
        self.video_renderer = None
        self.video_next_sim_time: Optional[float] = None
        self.video_output_resolved = self._resolve_video_output_path()

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
            f"viewer={'on' if self.enable_viewer else 'off'} fps={self.viewer_fps:.1f} "
            f"follow_robot={'on' if self.follow_robot else 'off'} "
            f"video_recording={'on' if self.enable_video_recording else 'off'}"
        )
        if self.viewer_title:
            self.get_logger().info(
                f"viewer_title='{self.viewer_title}' is informational only; mujoco.viewer.launch_passive controls the actual window title."
            )
        if self.follow_robot:
            self.get_logger().info(
                f"follow camera target body='{self.follow_body_name}' "
                f"distance={self.follow_distance:.2f} azimuth={self.follow_azimuth:.1f} "
                f"elevation={self.follow_elevation:.1f} offset={self.follow_lookat_offset.tolist()}"
            )
        if self.enable_video_recording:
            self.get_logger().info(
                f"video recording armed: path={self.video_output_resolved} "
                f"size={self.video_width}x{self.video_height} fps={self.video_fps:.1f}"
            )

    def _load_model(self, model_path: str):
        if model_path.endswith(".mjb"):
            model = mujoco.MjModel.from_binary_path(model_path)
        else:
            model = mujoco.MjModel.from_xml_path(model_path)
        return model

    def _read_vec3_parameter(self, name: str, default: List[float]) -> np.ndarray:
        raw = self.get_parameter(name).value
        values = list(default)
        if isinstance(raw, (list, tuple)) and len(raw) >= 3:
            try:
                values = [float(raw[0]), float(raw[1]), float(raw[2])]
            except Exception:
                values = list(default)
        return np.asarray(values, dtype=np.float64)

    def _resolve_body_id(self, body_name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise RuntimeError(f"follow_body_name '{body_name}' does not exist in model '{self.model_path}'")
        return int(body_id)

    def _resolve_video_output_path(self) -> str:
        if not self.enable_video_recording:
            return ""
        raw = self.video_output_path.strip()
        if not raw:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            raw = f"mujoco_python_frontend_{stamp}.mp4"
        path = Path(os.path.expanduser(raw))
        if not path.is_absolute():
            output_dir = Path(os.path.expanduser(self.video_output_dir.strip() or "/tmp/omnimorph_sim2sim_videos"))
            path = output_dir / path
        path.parent.mkdir(parents=True, exist_ok=True)
        return str(path)

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
        version_i = int(round(version))
        if version_i not in (1, 2):
            self.get_logger().warn(f"unsupported viewer frame version={version}, ignore")
            return

        base_size = K_HEADER_LEN + nq + nv + nu
        if len(data) < base_size:
            self.get_logger().warn(
                f"viewer frame size mismatch: got={len(data)} expected_at_least={base_size} (nq={nq}, nv={nv}, nu={nu})"
            )
            return
        if version_i == 1 and len(data) != base_size:
            self.get_logger().warn(
                f"viewer frame v1 size mismatch: got={len(data)} expected={base_size} (nq={nq}, nv={nv}, nu={nu})"
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
        offset += nu

        overlay = None
        if version_i >= 2:
            overlay_header_len = 14
            if len(data) < offset + overlay_header_len:
                self.get_logger().warn(
                    f"viewer frame v2 overlay too short: got={len(data)} expected_at_least={offset + overlay_header_len}"
                )
                return
            overlay_valid = abs(float(data[offset])) > 0.5
            com = np.asarray(data[offset + 1:offset + 4], dtype=np.float64)
            dcm_valid = abs(float(data[offset + 4])) > 0.5
            dcm_inside = abs(float(data[offset + 5])) > 0.5
            dcm = np.asarray(data[offset + 6:offset + 9], dtype=np.float64)
            cop_valid = abs(float(data[offset + 9])) > 0.5
            cop = np.asarray(data[offset + 10:offset + 13], dtype=np.float64)
            polygon_count = max(0, int(round(float(data[offset + 13]))))
            offset += overlay_header_len
            expected_size = offset + polygon_count * 3
            if len(data) != expected_size:
                self.get_logger().warn(
                    f"viewer frame v2 overlay size mismatch: got={len(data)} expected={expected_size} polygon_count={polygon_count}"
                )
                return
            polygon = np.asarray(data[offset:offset + polygon_count * 3], dtype=np.float64).reshape((-1, 3))
            if overlay_valid:
                overlay = {
                    "com": com,
                    "dcm_valid": dcm_valid,
                    "dcm_inside": dcm_inside,
                    "dcm": dcm,
                    "cop_valid": cop_valid,
                    "cop": cop,
                    "polygon": polygon,
                }

        with self.frame_lock:
            self.pending_qpos = qpos
            self.pending_qvel = qvel
            self.pending_ctrl = ctrl
            self.pending_overlay = overlay
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
            overlay = self.pending_overlay
            sim_time = self.pending_time
            self.pending_qpos = None
            self.pending_qvel = None
            self.pending_ctrl = None
            self.pending_overlay = None

        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.ctrl[:] = ctrl
        self.data.time = sim_time
        self.current_overlay = overlay
        mujoco.mj_forward(self.model, self.data)
        return True

    def _add_marker(self, scene, geom_type, size, pos, rgba) -> None:
        if scene is None or scene.ngeom >= scene.maxgeom:
            return
        mat = np.eye(3, dtype=np.float64).reshape(-1)
        mujoco.mjv_initGeom(
            scene.geoms[scene.ngeom],
            geom_type,
            np.asarray(size, dtype=np.float64),
            np.asarray(pos, dtype=np.float64),
            mat,
            np.asarray(rgba, dtype=np.float32),
        )
        scene.ngeom += 1

    def _add_segment(self, scene, start, end, width, rgba) -> None:
        if scene is None or scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.asarray([width, 0.0, 0.0], dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            np.eye(3, dtype=np.float64).reshape(-1),
            np.asarray(rgba, dtype=np.float32),
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            float(width),
            np.asarray(start, dtype=np.float64),
            np.asarray(end, dtype=np.float64),
        )
        scene.ngeom += 1

    def _add_polygon_boundary(self, scene, polygon, z, width, vertex_radius, rgba) -> None:
        if polygon is None or len(polygon) < 2:
            return
        for i in range(len(polygon)):
            start_raw = polygon[i]
            end_raw = polygon[(i + 1) % len(polygon)]
            start = np.asarray([start_raw[0], start_raw[1], z], dtype=np.float64)
            end = np.asarray([end_raw[0], end_raw[1], z], dtype=np.float64)
            self._add_segment(scene, start, end, width, rgba)
            if vertex_radius > 0.0:
                self._add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [vertex_radius] * 3, start, rgba)

    def _apply_overlay(self, scene) -> None:
        if scene is None:
            return
        scene.ngeom = 0
        overlay = self.current_overlay
        if not overlay:
            return

        com = overlay["com"]
        projection = np.asarray([com[0], com[1], 0.02], dtype=np.float64)
        self._add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.035, 0.035, 0.035], com, [1.0, 0.18, 0.05, 1.0])
        self._add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.025, 0.025, 0.025], projection, [0.05, 0.45, 1.0, 1.0])
        self._add_segment(scene, com, projection, 0.006, [1.0, 0.18, 0.05, 0.95])

        if overlay["dcm_valid"]:
            dcm = overlay["dcm"]
            dcm_display = np.asarray([dcm[0], dcm[1], 0.035], dtype=np.float64)
            self._add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.028, 0.028, 0.028], dcm_display, [0.95, 0.10, 1.0, 1.0])
            self._add_segment(scene, projection, dcm_display, 0.006, [0.95, 0.10, 1.0, 0.95])

        if overlay["cop_valid"]:
            cop = overlay["cop"]
            cop_display = np.asarray([cop[0], cop[1], 0.02], dtype=np.float64)
            self._add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.025, 0.025, 0.025], cop_display, [1.0, 0.85, 0.10, 1.0])

        polygon = overlay["polygon"]
        if len(polygon) >= 2:
            self._add_polygon_boundary(scene, polygon, 0.018, 0.008, 0.018, [0.05, 0.85, 0.30, 0.95])
            capture_rgba = [0.0, 0.95, 0.85, 1.0] if overlay["dcm_inside"] else [1.0, 0.10, 0.20, 1.0]
            if overlay["dcm_valid"]:
                self._add_polygon_boundary(scene, polygon, 0.032, 0.012, 0.0, capture_rgba)

    def _apply_follow_camera(self, camera) -> None:
        if not self.follow_robot or self.follow_body_id < 0:
            return
        lookat = np.asarray(self.data.xpos[self.follow_body_id], dtype=np.float64) + self.follow_lookat_offset
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = lookat
        camera.distance = self.follow_distance
        camera.azimuth = self.follow_azimuth
        camera.elevation = self.follow_elevation

    def _ensure_video_recording_started(self) -> None:
        if not self.enable_video_recording:
            return
        if self.video_renderer is None:
            self.video_renderer = mujoco.Renderer(self.model, height=self.video_height, width=self.video_width)
        if self.video_writer is None:
            self.video_writer = imageio.get_writer(self.video_output_resolved, fps=self.video_fps)
            self.get_logger().info(f"video recording started: {self.video_output_resolved}")

    def _make_recording_camera(self, viewer=None):
        camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(camera)
        if viewer is not None:
            try:
                camera.type = viewer.cam.type
                camera.fixedcamid = viewer.cam.fixedcamid
                camera.trackbodyid = viewer.cam.trackbodyid
                camera.lookat[:] = np.asarray(viewer.cam.lookat, dtype=np.float64)
                camera.distance = viewer.cam.distance
                camera.azimuth = viewer.cam.azimuth
                camera.elevation = viewer.cam.elevation
                return camera
            except Exception:
                pass
        self._apply_follow_camera(camera)
        return camera

    def _record_video_frame(self, viewer=None) -> None:
        if not self.enable_video_recording:
            return
        self._ensure_video_recording_started()
        camera = self._make_recording_camera(viewer)
        self.video_renderer.update_scene(self.data, camera=camera)
        frame = self.video_renderer.render()
        self.video_writer.append_data(frame)

    def _record_video_frame_if_due(self, viewer=None) -> None:
        if not self.enable_video_recording:
            return
        sim_time = float(self.data.time)
        if self.video_next_sim_time is None:
            self.video_next_sim_time = sim_time
        period = 1.0 / self.video_fps
        while sim_time + 1.0e-9 >= self.video_next_sim_time:
            self._record_video_frame(viewer)
            self.video_next_sim_time += period

    def _close_video_recording(self) -> None:
        if self.video_writer is not None:
            self.video_writer.close()
            self.video_writer = None
            self.get_logger().info(f"video recording saved: {self.video_output_resolved}")
        if self.video_renderer is not None:
            close_fn = getattr(self.video_renderer, "close", None)
            if callable(close_fn):
                close_fn()
            self.video_renderer = None

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
                updated = self._apply_pending_frame()
                if updated and self.enable_video_recording:
                    self._record_video_frame_if_due()
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
                with viewer.lock():
                    self._apply_follow_camera(viewer.cam)
                    self._apply_overlay(viewer.user_scn)
                if updated:
                    viewer.sync()
                else:
                    self._warn_if_stale()
                    viewer.sync()
                if updated and self.enable_video_recording:
                    self._record_video_frame_if_due(viewer)
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
        node._close_video_recording()
        if sys.stdout and not sys.stdout.closed:
            sys.stdout.flush()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
