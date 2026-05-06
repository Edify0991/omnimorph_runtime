from __future__ import annotations

import math
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
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
        self.camera_placeholder = self._make_placeholder("Camera")
        self.lidar_placeholder = self._make_placeholder("LiDAR / 3D")
        sensor_layout.addWidget(self.camera_placeholder)
        sensor_layout.addWidget(self.lidar_placeholder)
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
        label.setStyleSheet("QLabel { background: #f3f4f6; color: #374151; border: 1px solid #d1d5db; border-radius: 4px; }")
        return label

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

