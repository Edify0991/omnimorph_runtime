from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class ModeProfile:
    mode_id: int
    tag: str
    config_section: str


@dataclass(frozen=True)
class GuiConfig:
    path: Path
    joint_names: list[str]
    profiles: list[ModeProfile]


@dataclass
class RobotStateSnapshot:
    valid: bool = False
    joint_count: int = 0
    joint_q: list[float] = field(default_factory=list)
    joint_dq: list[float] = field(default_factory=list)
    joint_tau: list[float] = field(default_factory=list)
    base_ang_vel: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_quat_xyzw: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    base_rpy: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_lin_vel: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_lin_acc: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_lin_vel_valid: bool = False
    base_lin_acc_valid: bool = False
    stamp_monotonic: float = 0.0
    decode_error: str = ""

