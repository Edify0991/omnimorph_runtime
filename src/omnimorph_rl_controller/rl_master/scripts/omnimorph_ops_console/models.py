from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List


@dataclass(frozen=True)
class ModeProfile:
    mode_id: int
    tag: str
    config_section: str


@dataclass(frozen=True)
class GuiConfig:
    path: Path
    joint_names: List[str]
    profiles: List[ModeProfile]


@dataclass
class RobotStateSnapshot:
    valid: bool = False
    joint_count: int = 0
    joint_q: List[float] = field(default_factory=list)
    joint_dq: List[float] = field(default_factory=list)
    joint_tau: List[float] = field(default_factory=list)
    base_ang_vel: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_quat_xyzw: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    base_rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_lin_vel: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_lin_acc: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_lin_vel_valid: bool = False
    base_lin_acc_valid: bool = False
    stamp_monotonic: float = 0.0
    decode_error: str = ""
