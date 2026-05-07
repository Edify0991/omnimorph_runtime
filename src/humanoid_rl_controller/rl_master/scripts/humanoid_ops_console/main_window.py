from __future__ import annotations

import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QImage, QPixmap
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QStatusBar,
    QTableWidget,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from .constants import (
    CTRL_ESTOP,
    CTRL_SET_MODE_BASE,
    CTRL_START_LC,
    CTRL_START_MODE_BASE,
    CTRL_STOP,
    CTRL_ZERO,
    TOPIC_MODE_CONTROL,
)
from .models import GuiConfig, RobotStateSnapshot
from .protocol import decode_robot_state_payload
from .ros_interface import HumanoidOpsNode
from .widgets import RollingPlot, RobotTwinWidget, fmt_vec, make_readonly_item


class MainWindow(QMainWindow):
    def __init__(self, node: HumanoidOpsNode, config: GuiConfig) -> None:
        super().__init__()
        self.node = node
        self.config = config
        self.latest_snapshot = RobotStateSnapshot()
        self.state_message_count = 0
        self.last_camera_image: QImage | None = None
        self.last_depth_image: QImage | None = None
        self.last_camera_rx_time = 0.0
        self.last_depth_rx_time = 0.0
        self.last_camera_feature_rx_time = 0.0
        self.last_camera_interval_s: float | None = None
        self.last_depth_interval_s: float | None = None

        self.setWindowTitle("JC01 Humanoid Ops Console")
        self.resize(1240, 760)
        self.setStatusBar(QStatusBar(self))

        root = QSplitter(Qt.Horizontal)
        root.addWidget(self._build_controls())
        root.addWidget(self._build_monitor())
        root.addWidget(self._build_twin())
        root.setStretchFactor(0, 0)
        root.setStretchFactor(1, 2)
        root.setStretchFactor(2, 1)
        self.setCentralWidget(root)

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.node.spin_once)
        self.spin_timer.start(10)

        self.teleop_timer = QTimer(self)
        self.teleop_timer.timeout.connect(self._publish_teleop_if_enabled)
        self.teleop_timer.start(50)

        self.age_timer = QTimer(self)
        self.age_timer.timeout.connect(self._refresh_age_labels)
        self.age_timer.start(250)

    def _build_controls(self) -> QWidget:
        panel = QWidget()
        panel.setMinimumWidth(260)
        layout = QVBoxLayout(panel)

        mode_group = QGroupBox("Mode")
        mode_layout = QVBoxLayout(mode_group)
        self.mode_combo = QComboBox()
        profiles = self.config.profiles
        if profiles:
            for profile in profiles:
                self.mode_combo.addItem(f"{profile.mode_id}: {profile.tag}", profile.mode_id)
        else:
            for mode_id in range(4):
                self.mode_combo.addItem(f"{mode_id}: mode_{mode_id}", mode_id)
        mode_layout.addWidget(self.mode_combo)

        btn_grid = QGridLayout()
        buttons = [
            ("Start", self._send_start_mode, 0, 0),
            ("Set", self._send_set_mode, 0, 1),
            ("Start LC", lambda: self._send_control_word(CTRL_START_LC), 1, 0),
            ("Stop", lambda: self._send_control_word(CTRL_STOP), 1, 1),
            ("Zero", lambda: self._confirm_then_send("Send zeroing command?", CTRL_ZERO), 2, 0),
            ("E-Stop", lambda: self._confirm_then_send("Send emergency stop command?", CTRL_ESTOP), 2, 1),
        ]
        for text, callback, row, col in buttons:
            button = QPushButton(text)
            if text == "E-Stop":
                button.setStyleSheet("QPushButton { background: #b42318; color: white; font-weight: 600; }")
            button.clicked.connect(callback)
            btn_grid.addWidget(button, row, col)
        mode_layout.addLayout(btn_grid)
        layout.addWidget(mode_group)

        teleop_group = QGroupBox("Teleop")
        teleop_layout = QGridLayout(teleop_group)
        self.teleop_enabled = QCheckBox("stream")
        self.vx_spin = self._make_spin(-2.0, 2.0, 0.05)
        self.vy_spin = self._make_spin(-2.0, 2.0, 0.05)
        self.yaw_spin = self._make_spin(-4.0, 4.0, 0.05)
        teleop_layout.addWidget(QLabel("vx"), 0, 0)
        teleop_layout.addWidget(self.vx_spin, 0, 1)
        teleop_layout.addWidget(QLabel("vy"), 1, 0)
        teleop_layout.addWidget(self.vy_spin, 1, 1)
        teleop_layout.addWidget(QLabel("yaw"), 2, 0)
        teleop_layout.addWidget(self.yaw_spin, 2, 1)
        teleop_layout.addWidget(self.teleop_enabled, 3, 0)
        send_button = QPushButton("Send")
        send_button.clicked.connect(self._send_teleop_once)
        stop_button = QPushButton("Zero Cmd")
        stop_button.clicked.connect(self._zero_teleop)
        teleop_layout.addWidget(send_button, 3, 1)
        teleop_layout.addWidget(stop_button, 4, 1)
        layout.addWidget(teleop_group)

        state_group = QGroupBox("State")
        state_layout = QGridLayout(state_group)
        self.state_age_label = QLabel("age: inf")
        self.state_count_label = QLabel("messages: 0")
        self.joint_count_label = QLabel("joints: 0")
        self.base_rpy_label = QLabel("rpy: []")
        self.base_ang_label = QLabel("ang vel: []")
        self.base_lin_label = QLabel("lin vel: []")
        for row, label in enumerate(
            [
                self.state_age_label,
                self.state_count_label,
                self.joint_count_label,
                self.base_rpy_label,
                self.base_ang_label,
                self.base_lin_label,
            ]
        ):
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            state_layout.addWidget(label, row, 0)
        layout.addWidget(state_group)
        layout.addStretch(1)
        return panel

    def _build_monitor(self) -> QWidget:
        tabs = QTabWidget()

        joint_page = QWidget()
        joint_layout = QVBoxLayout(joint_page)
        self.joint_selector = QComboBox()
        self.joint_selector.addItem("0", 0)
        joint_layout.addWidget(self.joint_selector)
        self.joint_table = QTableWidget(0, 4)
        self.joint_table.setHorizontalHeaderLabels(["joint", "q rad", "dq rad/s", "tau"])
        self.joint_table.verticalHeader().setVisible(False)
        joint_layout.addWidget(self.joint_table)
        tabs.addTab(joint_page, "Joints")

        plot_page = QWidget()
        plot_layout = QVBoxLayout(plot_page)
        self.plot = RollingPlot()
        plot_layout.addWidget(self.plot)
        tabs.addTab(plot_page, "Plots")

        sensor_page = QWidget()
        sensor_layout = QVBoxLayout(sensor_page)
        preview_row = QHBoxLayout()
        self.camera_label = self._make_placeholder("Camera")
        self.depth_label = self._make_placeholder("Depth")
        preview_row.addWidget(self.camera_label, 1)
        preview_row.addWidget(self.depth_label, 1)
        sensor_layout.addLayout(preview_row)
        self.camera_status = QLabel("camera: waiting")
        self.depth_status = QLabel("depth: waiting")
        sensor_layout.addWidget(self.camera_status)
        sensor_layout.addWidget(self.depth_status)
        self.camera_feature_status = QLabel("camera_features: waiting")
        sensor_layout.addWidget(self.camera_feature_status)
        tabs.addTab(sensor_page, "Sensors")
        return tabs

    def _build_twin(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        title = QLabel("Twin")
        title.setFont(QFont(title.font().family(), 12, QFont.Bold))
        self.twin = RobotTwinWidget()
        layout.addWidget(title)
        layout.addWidget(self.twin)
        return panel

    def _make_placeholder(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumHeight(180)
        label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        label.setScaledContents(False)
        label.setStyleSheet("QLabel { background: #f3f4f6; color: #374151; border: 1px solid #d1d5db; border-radius: 4px; }")
        return label

    def _set_image_label(self, label: QLabel, image: QImage) -> None:
        pixmap = QPixmap.fromImage(image)
        scaled = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(scaled)

    def _format_stream_status(self, prefix: str, msg: Image, rx_time: float, interval_s: float | None) -> str:
        parts = [f"{prefix}: {msg.width}x{msg.height} {msg.encoding}"]
        if interval_s is not None and interval_s > 1.0e-6:
            parts.append(f"{(1.0 / interval_s):.1f} Hz")
        if rx_time > 0.0:
            parts.append(f"age {max(0.0, time.monotonic() - rx_time):.2f}s")
        return ", ".join(parts)

    def _refresh_sensor_previews(self) -> None:
        if self.last_camera_image is not None:
            self._set_image_label(self.camera_label, self.last_camera_image)
        if self.last_depth_image is not None:
            self._set_image_label(self.depth_label, self.last_depth_image)

    def _decode_image_msg(self, msg: Image) -> QImage | None:
        width = int(msg.width)
        height = int(msg.height)
        if width <= 0 or height <= 0:
            return None

        encoding = msg.encoding.lower()
        if encoding in {"rgb8", "bgr8"}:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            expected = height * width * 3
            if arr.size < expected:
                return None
            arr = arr[:expected].reshape((height, width, 3))
            image = QImage(arr.data, width, height, width * 3, QImage.Format_RGB888)
            if encoding == "bgr8":
                image = image.rgbSwapped()
            return image.copy()

        if encoding in {"rgba8", "bgra8"}:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            expected = height * width * 4
            if arr.size < expected:
                return None
            arr = arr[:expected].reshape((height, width, 4))
            image = QImage(arr.data, width, height, width * 4, QImage.Format_RGBA8888)
            if encoding == "bgra8":
                image = image.rgbSwapped()
            return image.copy()

        if encoding in {"mono8", "8uc1"}:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            expected = height * width
            if arr.size < expected:
                return None
            arr = arr[:expected].reshape((height, width))
            return QImage(arr.data, width, height, width, QImage.Format_Grayscale8).copy()

        if encoding in {"mono16", "16uc1", "16sc1"}:
            arr = np.frombuffer(msg.data, dtype=np.uint16)
            expected = height * width
            if arr.size < expected:
                return None
            arr = arr[:expected].reshape((height, width))
            max_value = int(arr.max()) if arr.size > 0 else 0
            if max_value <= 0:
                scaled = np.zeros((height, width), dtype=np.uint8)
            else:
                scaled = np.clip((arr.astype(np.float32) / float(max_value)) * 255.0, 0.0, 255.0).astype(np.uint8)
            return QImage(scaled.data, width, height, width, QImage.Format_Grayscale8).copy()

        return None

    def _make_spin(self, minimum: float, maximum: float, step: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setSingleStep(step)
        spin.setDecimals(3)
        return spin

    def _selected_mode_id(self) -> int:
        data = self.mode_combo.currentData()
        return int(data) if data is not None else 0

    def _send_start_mode(self) -> None:
        self._send_control_word(CTRL_START_MODE_BASE + self._selected_mode_id())

    def _send_set_mode(self) -> None:
        self._send_control_word(CTRL_SET_MODE_BASE + self._selected_mode_id())

    def _send_control_word(self, control_word: int) -> None:
        self.node.publish_mode_control(control_word)
        self.statusBar().showMessage(f"Published {TOPIC_MODE_CONTROL} = {control_word}", 2500)

    def _confirm_then_send(self, text: str, control_word: int) -> None:
        answer = QMessageBox.question(self, "Confirm", text, QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if answer == QMessageBox.Yes:
            self._send_control_word(control_word)

    def _send_teleop_once(self) -> None:
        self.node.publish_teleop(self.vx_spin.value(), self.vy_spin.value(), self.yaw_spin.value())
        self.statusBar().showMessage("Published teleop command", 1500)

    def _publish_teleop_if_enabled(self) -> None:
        if self.teleop_enabled.isChecked():
            self.node.publish_teleop(self.vx_spin.value(), self.vy_spin.value(), self.yaw_spin.value())

    def _zero_teleop(self) -> None:
        self.vx_spin.setValue(0.0)
        self.vy_spin.setValue(0.0)
        self.yaw_spin.setValue(0.0)
        self.node.publish_teleop(0.0, 0.0, 0.0)

    def on_state_msg(self, msg: Float32MultiArray) -> None:
        snapshot = decode_robot_state_payload(msg.data)
        self.latest_snapshot = snapshot
        if not snapshot.valid:
            self.statusBar().showMessage(snapshot.decode_error, 3000)
            return

        self.state_message_count += 1
        self._refresh_state_labels()
        self._refresh_joint_table(snapshot)
        self.plot.push(snapshot, self.joint_selector.currentData() or 0)
        self.twin.set_snapshot(snapshot)

    def on_camera_msg(self, msg: Image) -> None:
        image = self._decode_image_msg(msg)
        if image is None:
            self.camera_status.setText(f"camera: unsupported encoding {msg.encoding}")
            return
        now = time.monotonic()
        self.last_camera_interval_s = now - self.last_camera_rx_time if self.last_camera_rx_time > 0.0 else None
        self.last_camera_rx_time = now
        self.last_camera_image = image
        self._set_image_label(self.camera_label, image)
        self.camera_status.setText(self._format_stream_status("camera", msg, now, self.last_camera_interval_s))

    def on_depth_msg(self, msg: Image) -> None:
        image = self._decode_image_msg(msg)
        if image is None:
            self.depth_status.setText(f"depth: unsupported encoding {msg.encoding}")
            return
        now = time.monotonic()
        self.last_depth_interval_s = now - self.last_depth_rx_time if self.last_depth_rx_time > 0.0 else None
        self.last_depth_rx_time = now
        self.last_depth_image = image
        self._set_image_label(self.depth_label, image)
        self.depth_status.setText(self._format_stream_status("depth", msg, now, self.last_depth_interval_s))

    def on_camera_features_msg(self, msg: Float32MultiArray) -> None:
        self.last_camera_feature_rx_time = time.monotonic()
        values = list(msg.data)
        formatted = fmt_vec(values, 3)
        self.camera_feature_status.setText(f"camera_features[{len(values)}]: {formatted}")

    def _refresh_state_labels(self) -> None:
        snapshot = self.latest_snapshot
        self.state_count_label.setText(f"messages: {self.state_message_count}")
        self.joint_count_label.setText(f"joints: {snapshot.joint_count}")
        self.base_rpy_label.setText(f"rpy deg: {fmt_vec([math.degrees(v) for v in snapshot.base_rpy], 2)}")
        self.base_ang_label.setText(f"ang vel: {fmt_vec(snapshot.base_ang_vel, 3)}")
        suffix = "" if snapshot.base_lin_vel_valid else " (unverified)"
        self.base_lin_label.setText(f"lin vel: {fmt_vec(snapshot.base_lin_vel, 3)}{suffix}")
        self._refresh_age_labels()

    def _refresh_age_labels(self) -> None:
        if self.latest_snapshot.valid and self.latest_snapshot.stamp_monotonic > 0.0:
            age = time.monotonic() - self.latest_snapshot.stamp_monotonic
            self.state_age_label.setText(f"age: {age:.2f}s")
        else:
            self.state_age_label.setText("age: inf")

    def _refresh_joint_table(self, snapshot: RobotStateSnapshot) -> None:
        if self.joint_table.rowCount() != snapshot.joint_count:
            self.joint_table.setRowCount(snapshot.joint_count)
            self.joint_selector.clear()
            for idx in range(snapshot.joint_count):
                name = self.config.joint_names[idx] if idx < len(self.config.joint_names) else str(idx)
                self.joint_table.setItem(idx, 0, make_readonly_item(name))
                self.joint_selector.addItem(f"{idx}: {name}", idx)

        for idx in range(snapshot.joint_count):
            self.joint_table.setItem(idx, 1, make_readonly_item(f"{snapshot.joint_q[idx]: .5f}"))
            self.joint_table.setItem(idx, 2, make_readonly_item(f"{snapshot.joint_dq[idx]: .5f}"))
            self.joint_table.setItem(idx, 3, make_readonly_item(f"{snapshot.joint_tau[idx]: .5f}"))

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.teleop_enabled.setChecked(False)
        self.node.publish_teleop(0.0, 0.0, 0.0)
        super().closeEvent(event)

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        self._refresh_sensor_previews()
