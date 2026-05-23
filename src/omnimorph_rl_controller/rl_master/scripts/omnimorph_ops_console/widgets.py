from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QSizePolicy, QTableWidgetItem, QWidget

from .models import RobotStateSnapshot


def fmt_vec(values: List[float], precision: int = 3) -> str:
    return "[" + ", ".join(f"{v:.{precision}f}" for v in values) + "]"


def make_readonly_item(text: str) -> QTableWidgetItem:
    item = QTableWidgetItem(text)
    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
    return item


class RollingPlot(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.series: Dict[str, Deque[float]] = {
            "roll": deque(maxlen=240),
            "pitch": deque(maxlen=240),
            "yaw": deque(maxlen=240),
            "joint_q": deque(maxlen=240),
        }
        self.colors = {
            "roll": QColor("#2f80ed"),
            "pitch": QColor("#27ae60"),
            "yaw": QColor("#c0392b"),
            "joint_q": QColor("#7f8c8d"),
        }

    def push(self, snapshot: RobotStateSnapshot, selected_joint: int) -> None:
        if not snapshot.valid:
            return
        rpy_deg = [math.degrees(v) for v in snapshot.base_rpy]
        self.series["roll"].append(rpy_deg[0])
        self.series["pitch"].append(rpy_deg[1])
        self.series["yaw"].append(rpy_deg[2])
        if 0 <= selected_joint < len(snapshot.joint_q):
            self.series["joint_q"].append(math.degrees(snapshot.joint_q[selected_joint]))
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(12, 12, -12, -28)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QPen(QColor("#d8dee4"), 1))
        painter.drawRect(rect)

        values = [v for series in self.series.values() for v in series]
        if not values:
            painter.setPen(QColor("#6b7280"))
            painter.drawText(rect, Qt.AlignCenter, "Waiting for /omnimorph/rl/state")
            return

        y_min = min(values)
        y_max = max(values)
        if abs(y_max - y_min) < 1e-6:
            y_min -= 1.0
            y_max += 1.0
        pad = max(1.0, 0.08 * (y_max - y_min))
        y_min -= pad
        y_max += pad

        painter.setPen(QPen(QColor("#edf2f7"), 1))
        for i in range(1, 4):
            y = rect.top() + rect.height() * i / 4.0
            painter.drawLine(rect.left(), int(y), rect.right(), int(y))

        for name, series in self.series.items():
            if len(series) < 2:
                continue
            painter.setPen(QPen(self.colors[name], 2))
            n = len(series)
            points: List[Tuple[float, float]] = []
            for i, value in enumerate(series):
                x = rect.left() + rect.width() * i / max(1, n - 1)
                y = rect.bottom() - rect.height() * (value - y_min) / (y_max - y_min)
                points.append((x, y))
            for i in range(1, len(points)):
                painter.drawLine(int(points[i - 1][0]), int(points[i - 1][1]), int(points[i][0]), int(points[i][1]))

        legend_x = rect.left()
        legend_y = rect.bottom() + 20
        for name in ["roll", "pitch", "yaw", "joint_q"]:
            painter.setPen(QPen(self.colors[name], 3))
            painter.drawLine(legend_x, legend_y - 4, legend_x + 18, legend_y - 4)
            painter.setPen(QColor("#24292f"))
            painter.drawText(legend_x + 24, legend_y, name)
            legend_x += 92


class RobotTwinWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.snapshot = RobotStateSnapshot()
        self.setMinimumSize(280, 300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_snapshot(self, snapshot: RobotStateSnapshot) -> None:
        self.snapshot = snapshot
        self.update()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        rect = self.rect().adjusted(18, 18, -18, -18)
        painter.setPen(QPen(QColor("#d0d7de"), 1))
        painter.drawRoundedRect(rect, 6, 6)

        if not self.snapshot.valid:
            painter.setPen(QColor("#6b7280"))
            painter.drawText(rect, Qt.AlignCenter, "Twin")
            return

        cx = rect.center().x()
        cy = rect.center().y() - 10
        roll = self.snapshot.base_rpy[0]
        pitch = self.snapshot.base_rpy[1]

        painter.save()
        painter.translate(cx, cy)
        painter.rotate(math.degrees(roll) * 0.6)
        painter.setPen(QPen(QColor("#1f2937"), 5, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(0, -70, 0, 40)
        painter.drawLine(-48, -35, 48, -35)
        painter.drawLine(-22, 40, -42, 100)
        painter.drawLine(22, 40, 42, 100)
        painter.setBrush(QColor("#dbeafe"))
        painter.setPen(QPen(QColor("#1d4ed8"), 2))
        painter.drawEllipse(-18, -108, 36, 36)

        painter.setBrush(QColor("#10b981"))
        painter.setPen(Qt.NoPen)
        radius = max(5, min(18, int(abs(pitch) * 40)))
        painter.drawEllipse(-radius, -radius, radius * 2, radius * 2)
        painter.restore()

        painter.setPen(QColor("#24292f"))
        rpy_deg = [math.degrees(v) for v in self.snapshot.base_rpy]
        painter.drawText(rect.left() + 14, rect.top() + 26, f"RPY deg: {fmt_vec(rpy_deg, 1)}")
        painter.drawText(rect.left() + 14, rect.top() + 48, f"joints: {self.snapshot.joint_count}")
