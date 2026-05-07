#!/usr/bin/env python3
from __future__ import annotations

import io
import copy
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from PIL import Image as PILImage


@dataclass
class FeatureState:
    color: list[float]
    depth: list[float]


class StandardizedCameraBridge(Node):
    def __init__(self) -> None:
        super().__init__("realsense_camera_bridge")

        self.declare_parameter("input_color_topic", "/camera/color/image_raw")
        self.declare_parameter("input_depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("input_color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("input_depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_color_topic", "/humanoid/camera/color/image_raw")
        self.declare_parameter("output_depth_topic", "/humanoid/camera/depth/image_raw")
        self.declare_parameter("output_color_info_topic", "/humanoid/camera/color/camera_info")
        self.declare_parameter("output_depth_info_topic", "/humanoid/camera/depth/camera_info")
        self.declare_parameter("output_color_compressed_topic", "/humanoid/camera/color/image_raw/compressed")
        self.declare_parameter("output_depth_compressed_topic", "/humanoid/camera/depth/image_raw/compressed")
        self.declare_parameter("output_feature_topic", "/humanoid/camera/features")
        self.declare_parameter("target_width", 424)
        self.declare_parameter("target_height", 240)
        self.declare_parameter("output_fps", 15.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("color_compressed_format", "jpeg")
        self.declare_parameter("depth_compressed_format", "png")

        self.input_color_topic = str(self.get_parameter("input_color_topic").value)
        self.input_depth_topic = str(self.get_parameter("input_depth_topic").value)
        self.input_color_info_topic = str(self.get_parameter("input_color_info_topic").value)
        self.input_depth_info_topic = str(self.get_parameter("input_depth_info_topic").value)
        self.output_color_topic = str(self.get_parameter("output_color_topic").value)
        self.output_depth_topic = str(self.get_parameter("output_depth_topic").value)
        self.output_color_info_topic = str(self.get_parameter("output_color_info_topic").value)
        self.output_depth_info_topic = str(self.get_parameter("output_depth_info_topic").value)
        self.output_color_compressed_topic = str(self.get_parameter("output_color_compressed_topic").value)
        self.output_depth_compressed_topic = str(self.get_parameter("output_depth_compressed_topic").value)
        self.output_feature_topic = str(self.get_parameter("output_feature_topic").value)
        self.target_width = max(1, int(self.get_parameter("target_width").value))
        self.target_height = max(1, int(self.get_parameter("target_height").value))
        self.output_fps = max(1.0, float(self.get_parameter("output_fps").value))
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.color_compressed_format = str(self.get_parameter("color_compressed_format").value).strip().lower()
        self.depth_compressed_format = str(self.get_parameter("depth_compressed_format").value).strip().lower()
        if self.color_compressed_format != "jpeg":
            self.get_logger().warning(
                f"unsupported color_compressed_format={self.color_compressed_format}, forcing jpeg"
            )
            self.color_compressed_format = "jpeg"
        if self.depth_compressed_format != "png":
            self.get_logger().warning(
                f"unsupported depth_compressed_format={self.depth_compressed_format}, forcing png"
            )
            self.depth_compressed_format = "png"
        self.publish_period_ns = int(1e9 / self.output_fps)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.color_pub = self.create_publisher(Image, self.output_color_topic, sensor_qos)
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, sensor_qos)
        self.color_info_pub = self.create_publisher(CameraInfo, self.output_color_info_topic, sensor_qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, self.output_depth_info_topic, sensor_qos)
        self.color_compressed_pub = self.create_publisher(
            CompressedImage, self.output_color_compressed_topic, sensor_qos
        )
        self.depth_compressed_pub = self.create_publisher(
            CompressedImage, self.output_depth_compressed_topic, sensor_qos
        )
        self.feature_pub = self.create_publisher(Float32MultiArray, self.output_feature_topic, sensor_qos)

        self.create_subscription(Image, self.input_color_topic, self._on_color, sensor_qos)
        self.create_subscription(Image, self.input_depth_topic, self._on_depth, sensor_qos)
        self.create_subscription(CameraInfo, self.input_color_info_topic, self._on_color_info, sensor_qos)
        self.create_subscription(CameraInfo, self.input_depth_info_topic, self._on_depth_info, sensor_qos)

        self._feature_state = FeatureState(color=[0.0, 0.0, 0.0, 0.0], depth=[0.0, 0.0, 0.0, 0.0])
        self._last_color_pub_ns = 0
        self._last_depth_pub_ns = 0
        self._last_feature_pub_ns = 0
        self._has_color_feature = False
        self._has_depth_feature = False

        self.get_logger().info(
            "camera bridge started: "
            f"{self.input_color_topic} -> {self.output_color_topic}, "
            f"{self.input_depth_topic} -> {self.output_depth_topic}, "
            f"size={self.target_width}x{self.target_height}, fps={self.output_fps:.1f}, "
            f"color_compressed={self.color_compressed_format}, depth_compressed={self.depth_compressed_format}"
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _should_publish(self, last_pub_ns: int) -> bool:
        return self._now_ns() - last_pub_ns >= self.publish_period_ns

    def _on_color(self, msg: Image) -> None:
        if not self._should_publish(self._last_color_pub_ns):
            return

        rgb = self._decode_color(msg)
        if rgb is None:
            return

        resized = self._resize_color(rgb)
        color_msg = Image()
        color_msg.header = msg.header
        color_msg.height = resized.shape[0]
        color_msg.width = resized.shape[1]
        color_msg.encoding = "rgb8"
        color_msg.is_bigendian = 0
        color_msg.step = resized.shape[1] * 3
        color_msg.data = resized.tobytes()
        self.color_pub.publish(color_msg)

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = self.color_compressed_format
        compressed.data = self._encode_jpeg(resized)
        self.color_compressed_pub.publish(compressed)

        luma = 0.299 * resized[:, :, 0] + 0.587 * resized[:, :, 1] + 0.114 * resized[:, :, 2]
        self._feature_state.color = [
            float(np.mean(resized[:, :, 0]) / 255.0),
            float(np.mean(resized[:, :, 1]) / 255.0),
            float(np.mean(resized[:, :, 2]) / 255.0),
            float(np.std(luma) / 255.0),
        ]
        self._has_color_feature = True
        self._last_color_pub_ns = self._now_ns()
        self._publish_features()

    def _on_depth(self, msg: Image) -> None:
        if not self._should_publish(self._last_depth_pub_ns):
            return

        depth_mm = self._decode_depth_mm(msg)
        if depth_mm is None:
            return

        resized = self._resize_depth(depth_mm)
        depth_msg = Image()
        depth_msg.header = msg.header
        depth_msg.height = resized.shape[0]
        depth_msg.width = resized.shape[1]
        depth_msg.encoding = "16UC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = resized.shape[1] * 2
        depth_msg.data = resized.tobytes()
        self.depth_pub.publish(depth_msg)

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = self.depth_compressed_format
        compressed.data = self._encode_png16(resized)
        self.depth_compressed_pub.publish(compressed)

        valid = resized > 0
        if np.any(valid):
            valid_depth_m = resized[valid].astype(np.float32) * self.depth_scale
            center_depth_m = float(resized[resized.shape[0] // 2, resized.shape[1] // 2]) * self.depth_scale
            depth_mean_m = float(np.mean(valid_depth_m))
            valid_ratio = float(np.mean(valid.astype(np.float32)))
            near_ratio = float(np.mean((valid_depth_m < 1.5).astype(np.float32)))
        else:
            center_depth_m = 0.0
            depth_mean_m = 0.0
            valid_ratio = 0.0
            near_ratio = 0.0

        self._feature_state.depth = [
            center_depth_m,
            depth_mean_m,
            valid_ratio,
            near_ratio,
        ]
        self._has_depth_feature = True
        self._last_depth_pub_ns = self._now_ns()
        self._publish_features()

    def _publish_features(self) -> None:
        if not self._has_color_feature or not self._has_depth_feature:
            return
        if not self._should_publish(self._last_feature_pub_ns):
            return
        msg = Float32MultiArray()
        msg.data = self._feature_state.color + self._feature_state.depth
        self.feature_pub.publish(msg)
        self._last_feature_pub_ns = self._now_ns()

    def _on_color_info(self, msg: CameraInfo) -> None:
        self.color_info_pub.publish(self._resize_camera_info(msg))

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self.depth_info_pub.publish(self._resize_camera_info(msg))

    def _decode_color(self, msg: Image) -> np.ndarray | None:
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return None

        encoding = msg.encoding.lower()
        channels = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }.get(encoding)
        if channels is None:
            self.get_logger().warning(f"unsupported color encoding: {msg.encoding}")
            return None

        row_stride = int(msg.step)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < row_stride * height:
            return None
        rows = raw[: row_stride * height].reshape((height, row_stride))
        pixels = rows[:, : width * channels].reshape((height, width, channels))

        if encoding == "rgb8":
            return np.ascontiguousarray(pixels)
        if encoding == "bgr8":
            return np.ascontiguousarray(pixels[:, :, ::-1])
        if encoding == "rgba8":
            return np.ascontiguousarray(pixels[:, :, :3])
        if encoding == "bgra8":
            return np.ascontiguousarray(pixels[:, :, [2, 1, 0]])

        mono = pixels[:, :, 0]
        return np.ascontiguousarray(np.repeat(mono[:, :, None], 3, axis=2))

    def _decode_depth_mm(self, msg: Image) -> np.ndarray | None:
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return None

        encoding = msg.encoding.lower()
        if encoding in {"16uc1", "mono16"}:
            row_stride = int(msg.step) // 2
            raw = np.frombuffer(msg.data, dtype=np.uint16)
            if raw.size < row_stride * height:
                return None
            rows = raw[: row_stride * height].reshape((height, row_stride))
            return np.ascontiguousarray(rows[:, :width])
        if encoding == "32fc1":
            row_stride = int(msg.step) // 4
            raw = np.frombuffer(msg.data, dtype=np.float32)
            if raw.size < row_stride * height:
                return None
            rows = raw[: row_stride * height].reshape((height, row_stride))
            depth_m = np.nan_to_num(rows[:, :width], nan=0.0, posinf=0.0, neginf=0.0)
            depth_mm = np.clip(depth_m / max(self.depth_scale, 1.0e-6), 0.0, 65535.0).astype(np.uint16)
            return np.ascontiguousarray(depth_mm)

        self.get_logger().warning(f"unsupported depth encoding: {msg.encoding}")
        return None

    def _resize_color(self, rgb: np.ndarray) -> np.ndarray:
        if rgb.shape[1] == self.target_width and rgb.shape[0] == self.target_height:
            return np.ascontiguousarray(rgb)
        image = PILImage.fromarray(rgb, mode="RGB")
        resampling = getattr(PILImage, "Resampling", PILImage)
        resized = image.resize((self.target_width, self.target_height), resample=resampling.BILINEAR)
        return np.ascontiguousarray(np.asarray(resized, dtype=np.uint8))

    def _resize_depth(self, depth_mm: np.ndarray) -> np.ndarray:
        if depth_mm.shape[1] == self.target_width and depth_mm.shape[0] == self.target_height:
            return np.ascontiguousarray(depth_mm)
        image = PILImage.fromarray(depth_mm, mode="I;16")
        resampling = getattr(PILImage, "Resampling", PILImage)
        resized = image.resize((self.target_width, self.target_height), resample=resampling.NEAREST)
        return np.ascontiguousarray(np.asarray(resized, dtype=np.uint16))

    def _resize_camera_info(self, msg: CameraInfo) -> CameraInfo:
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return msg
        if width == self.target_width and height == self.target_height:
            return msg

        sx = float(self.target_width) / float(width)
        sy = float(self.target_height) / float(height)

        out = copy.deepcopy(msg)
        out.width = self.target_width
        out.height = self.target_height

        k = list(out.k)
        if len(k) == 9:
            k[0] *= sx
            k[2] *= sx
            k[4] *= sy
            k[5] *= sy
            out.k = k

        p = list(out.p)
        if len(p) == 12:
            p[0] *= sx
            p[2] *= sx
            p[5] *= sy
            p[6] *= sy
            out.p = p

        out.roi.x_offset = int(round(out.roi.x_offset * sx))
        out.roi.y_offset = int(round(out.roi.y_offset * sy))
        out.roi.width = int(round(out.roi.width * sx)) if out.roi.width > 0 else 0
        out.roi.height = int(round(out.roi.height * sy)) if out.roi.height > 0 else 0
        return out

    def _encode_jpeg(self, rgb: np.ndarray) -> bytes:
        buffer = io.BytesIO()
        PILImage.fromarray(rgb, mode="RGB").save(buffer, format="JPEG", quality=self.jpeg_quality, optimize=False)
        return buffer.getvalue()

    def _encode_png16(self, depth_mm: np.ndarray) -> bytes:
        buffer = io.BytesIO()
        PILImage.fromarray(depth_mm, mode="I;16").save(buffer, format="PNG", optimize=False)
        return buffer.getvalue()


def main() -> int:
    rclpy.init(args=None)
    node = StandardizedCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
