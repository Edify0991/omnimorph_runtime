from __future__ import annotations

import time
from collections.abc import Sequence

from .constants import (
    BASE_STATE_TAIL_COUNT,
    PAYLOAD_ROBOT_STATE,
    PROTOCOL_V2_MAGIC,
    PROTOCOL_VERSION,
    STATE_HEADER_COUNT,
)
from .models import RobotStateSnapshot


def _to_int_field(value: float) -> int:
    return int(round(float(value)))


def decode_robot_state_payload(data: Sequence[float]) -> RobotStateSnapshot:
    snapshot = RobotStateSnapshot(stamp_monotonic=time.monotonic())
    if len(data) < STATE_HEADER_COUNT:
        snapshot.decode_error = f"state payload too short: {len(data)}"
        return snapshot

    if _to_int_field(data[0]) != PROTOCOL_V2_MAGIC:
        snapshot.decode_error = f"unexpected magic: {data[0]}"
        return snapshot
    if _to_int_field(data[1]) != PROTOCOL_VERSION:
        snapshot.decode_error = f"unsupported protocol version: {data[1]}"
        return snapshot
    if _to_int_field(data[2]) != PAYLOAD_ROBOT_STATE:
        snapshot.decode_error = f"unexpected payload type: {data[2]}"
        return snapshot

    joint_count = _to_int_field(data[3])
    if joint_count < 0:
        snapshot.decode_error = f"negative joint_count: {joint_count}"
        return snapshot

    expected_min = STATE_HEADER_COUNT + joint_count * 3 + BASE_STATE_TAIL_COUNT
    if len(data) < expected_min:
        snapshot.decode_error = f"state payload size mismatch: got {len(data)}, need at least {expected_min}"
        return snapshot

    cursor = STATE_HEADER_COUNT
    joint_q: list[float] = []
    joint_dq: list[float] = []
    joint_tau: list[float] = []
    for _ in range(joint_count):
        joint_q.append(float(data[cursor]))
        joint_dq.append(float(data[cursor + 1]))
        joint_tau.append(float(data[cursor + 2]))
        cursor += 3

    snapshot.joint_count = joint_count
    snapshot.joint_q = joint_q
    snapshot.joint_dq = joint_dq
    snapshot.joint_tau = joint_tau
    snapshot.base_ang_vel = [float(v) for v in data[cursor:cursor + 3]]
    cursor += 3
    snapshot.base_quat_xyzw = [float(v) for v in data[cursor:cursor + 4]]
    cursor += 4
    snapshot.base_rpy = [float(v) for v in data[cursor:cursor + 3]]
    cursor += 3

    if len(data) >= cursor + 3:
        snapshot.base_lin_vel = [float(v) for v in data[cursor:cursor + 3]]
        snapshot.base_lin_vel_valid = True
        cursor += 3
    if len(data) >= cursor + 3:
        snapshot.base_lin_acc = [float(v) for v in data[cursor:cursor + 3]]
        snapshot.base_lin_acc_valid = True
        cursor += 3
    if len(data) >= cursor + 2:
        snapshot.base_lin_vel_valid = float(data[cursor]) > 0.5
        snapshot.base_lin_acc_valid = float(data[cursor + 1]) > 0.5

    snapshot.valid = True
    return snapshot

