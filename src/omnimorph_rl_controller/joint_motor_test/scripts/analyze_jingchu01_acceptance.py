#!/usr/bin/env python3
"""Analyze Jingchu01 single-joint acceptance logs.

The joint-test JSONL provides phase labels and mapped joint feedback. The solver
MCAP provides the mapping-before motor feedback, including knee linear force in
motor_state_tau. MCAP chunks written with zstd are decoded through the system
zstd executable, so no Python mcap package is required.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import pathlib
import struct
import subprocess
from typing import Any, Iterable

import yaml

DEG = 180.0 / math.pi
MOVE_PHASES = {1, 3, 5}
MOTOR_RUN_MODE_CST = 10
DENSITY_REQUIREMENTS = {
    "right_hip_pitch": (60.0, "Nm/kg"),
    "right_knee_pitch": (1000.0, "N/kg"),
    "right_ankle_pitch": (60.0, "Nm/kg"),
    "right_ankle_roll": (60.0, "Nm/kg"),
}


def _u32(data: bytes, offset: int) -> tuple[int, int]:
    return struct.unpack_from("<I", data, offset)[0], offset + 4


def _string(data: bytes, offset: int) -> tuple[str, int]:
    length, offset = _u32(data, offset)
    end = offset + length
    return data[offset:end].decode("utf-8"), end


def _records(data: bytes, start: int = 0, end: int | None = None) -> Iterable[tuple[int, bytes]]:
    cursor = start
    limit = len(data) if end is None else end
    while cursor + 9 <= limit:
        opcode = data[cursor]
        length = struct.unpack_from("<Q", data, cursor + 1)[0]
        payload_start = cursor + 9
        payload_end = payload_start + length
        if payload_end > limit:
            raise ValueError("truncated MCAP record")
        yield opcode, data[payload_start:payload_end]
        cursor = payload_end


def _decode_chunk(payload: bytes) -> bytes:
    offset = 8 + 8
    uncompressed_size = struct.unpack_from("<Q", payload, offset)[0]
    offset += 8 + 4
    compression, offset = _string(payload, offset)
    compressed = payload[offset:]
    if compression in ("", "none"):
        result = compressed
    elif compression == "zstd":
        proc = subprocess.run(
            ["zstd", "-q", "-d", "-c"], input=compressed, capture_output=True, check=False
        )
        if proc.returncode != 0:
            raise RuntimeError(f"zstd MCAP decode failed: {proc.stderr.decode(errors='replace')}")
        result = proc.stdout
    else:
        raise RuntimeError(f"unsupported MCAP compression: {compression!r}")
    if len(result) != uncompressed_size:
        raise ValueError("MCAP chunk uncompressed-size mismatch")
    return result


def read_mcap(path: pathlib.Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    data = path.read_bytes()
    magic = b"\x89MCAP0\r\n"
    if not data.startswith(magic) or not data.endswith(magic):
        raise ValueError(f"not a complete MCAP file: {path}")
    channels: dict[int, str] = {}
    ticks: list[dict[str, Any]] = []
    configs: list[dict[str, Any]] = []

    def consume(records: Iterable[tuple[int, bytes]]) -> None:
        for opcode, payload in records:
            if opcode == 0x04:  # Channel
                channel_id = struct.unpack_from("<H", payload, 0)[0]
                topic, _ = _string(payload, 4)
                channels[channel_id] = topic
            elif opcode == 0x05:  # Message
                channel_id = struct.unpack_from("<H", payload, 0)[0]
                topic = channels.get(channel_id, "")
                message = json.loads(payload[22:].decode("utf-8"))
                if topic == "runtime/tick":
                    ticks.append(message)
                elif topic == "runtime/config":
                    configs.append(message)

    for opcode, payload in _records(data, len(magic), len(data) - len(magic)):
        if opcode == 0x06:
            consume(_records(_decode_chunk(payload)))
        else:
            consume([(opcode, payload)])
    return ticks, configs


def load_joint_records(path: pathlib.Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    metadata: dict[str, Any] = {}
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8-sig") as stream:
        for line in stream:
            item = json.loads(line)
            if item.get("kind") == "metadata":
                metadata = item
            elif item.get("kind") == "record" and item.get("channel") == "joint_motor_test":
                records.append(item)
    if not records:
        raise ValueError(f"no joint_motor_test records in {path}")
    return metadata, records


def recursively_find(obj: Any, key: str) -> Any | None:
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        for value in obj.values():
            found = recursively_find(value, key)
            if found is not None:
                return found
    elif isinstance(obj, list):
        for value in obj:
            found = recursively_find(value, key)
            if found is not None:
                return found
    return None


def percentile(values: list[float], probability: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def nearest_phase(times: list[float], records: list[dict[str, Any]], time_sec: float) -> tuple[int, int]:
    index = bisect.bisect_right(times, time_sec) - 1
    if index < 0 or index >= len(records):
        return -1, -1
    scalars = records[index]["scalars"]
    return int(round(scalars.get("acceptance_joint_cursor", -1))), int(
        round(scalars.get("acceptance_phase", -1))
    )


def resolve_motor_names(
    snapshots: list[dict[str, Any]],
    joint_names: list[str],
    acceptance_joints: list[dict[str, Any]],
) -> tuple[list[str], str]:
    for snapshot in snapshots:
        motor_names = recursively_find(snapshot, "installed_motor_order")
        if not motor_names:
            motor_names = recursively_find(snapshot, "robot_global_motor_order")
        if motor_names:
            return list(motor_names), "runtime/config"

    # Older/fused sim2sim runtime/config payloads do not contain the motor
    # order. For this right-leg-only test the leg motor slots are identical to
    # the leading joint slots, so the configured joint order is a safe fallback.
    # Real solver logs should use installed_motor_order above.
    tested_motor_names = {
        name
        for item in acceptance_joints
        for name in item.get("coupled_cst_joints", [item["name"]])
    }
    if not tested_motor_names.issubset(set(joint_names)):
        raise ValueError("cannot resolve tested motor names from runtime/config or joint_names")
    return list(joint_names), "joint_names_fallback"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint-log", required=True, type=pathlib.Path)
    parser.add_argument("--solver-mcap", required=True, type=pathlib.Path)
    parser.add_argument("--config", required=True, type=pathlib.Path)
    parser.add_argument("--output-dir", required=True, type=pathlib.Path)
    args = parser.parse_args()

    config_root = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    config = config_root.get("joint_motor_test", config_root)
    acceptance_joints = config["acceptance"]["joints"]
    joint_names = list(config["joint_names"])
    metadata, records = load_joint_records(args.joint_log)
    ticks, snapshots = read_mcap(args.solver_mcap)
    if not ticks:
        raise ValueError("solver MCAP has no runtime/tick messages")

    motor_names, motor_order_source = resolve_motor_names(snapshots, joint_names, acceptance_joints)
    record_times = [float(record["time_sec"]) for record in records]
    min_time, max_time = record_times[0], record_times[-1]

    results: list[dict[str, Any]] = []
    for cursor, joint_cfg in enumerate(acceptance_joints):
        name = joint_cfg["name"]
        joint_index = joint_names.index(name)
        raw_motor_names = set(joint_cfg.get("coupled_cst_joints", [name]))
        motor_indices = [i for i, motor_name in enumerate(motor_names) if motor_name in raw_motor_names]
        selected_records = [
            record
            for record in records
            if int(round(record["scalars"].get("acceptance_joint_cursor", -1))) == cursor
        ]
        moving_records = [
            record
            for record in selected_records
            if int(round(record["scalars"].get("acceptance_phase", -1))) in MOVE_PHASES
        ]
        actual_speeds = [abs(record["vectors"]["state_dq"][joint_index]) for record in moving_records]
        all_positions = [record["vectors"]["state_q"][joint_index] for record in selected_records]
        pd_torques = [abs(record["vectors"]["cmd_tau"][joint_index]) for record in moving_records]
        mapped_torques = [abs(record["vectors"]["state_tau"][joint_index]) for record in moving_records]
        accelerations = [
            abs(record["vectors"].get("state_ddq_estimated", [0.0] * len(joint_names))[joint_index])
            for record in moving_records
        ]

        raw_peaks = {motor_names[index]: 0.0 for index in motor_indices}
        raw_modes: dict[str, list[int]] = {motor_names[index]: [] for index in motor_indices}
        for tick in ticks:
            time_sec = float(tick.get("monotonic_time_sec", -1.0))
            if time_sec < min_time or time_sec > max_time:
                continue
            tick_cursor, tick_phase = nearest_phase(record_times, records, time_sec)
            if tick_cursor != cursor or tick_phase not in MOVE_PHASES:
                continue
            raw_tau = tick.get("motor_state_tau", [])
            motor_modes = tick.get("motor_cmd_mode", [])
            for index in motor_indices:
                if index < len(raw_tau):
                    raw_peaks[motor_names[index]] = max(raw_peaks[motor_names[index]], abs(raw_tau[index]))
                if index < len(motor_modes):
                    raw_modes[motor_names[index]].append(int(round(motor_modes[index])))

        expected_cst_indices = {joint_names.index(item) for item in joint_cfg.get("coupled_cst_joints", [name])}
        mask_pass = bool(moving_records) and all(
            len(record["vectors"].get("cst_mask", [])) == len(joint_names)
            and {
                i for i, value in enumerate(record["vectors"]["cst_mask"]) if value > 0.5
            } == expected_cst_indices
            for record in moving_records
        )
        cst_mode_fraction = {
            motor: (sum(value == MOTOR_RUN_MODE_CST for value in values) / len(values) if values else 0.0)
            for motor, values in raw_modes.items()
        }
        motor_mode_pass = bool(raw_modes) and all(value >= 0.98 for value in cst_mode_fraction.values())

        required_velocity = float(joint_cfg["required_actual_velocity"])
        max_speed = max(actual_speeds, default=0.0)
        speed_pass = max_speed >= required_velocity
        actual_range = max(all_positions) - min(all_positions) if all_positions else 0.0
        required_range = float(joint_cfg["required_actual_range"])
        range_pass = actual_range >= required_range
        actuator_mass_kg = float(joint_cfg.get("actuator_mass_kg", joint_cfg.get("mass_kg", 0.0)))
        density_required, density_unit = DENSITY_REQUIREMENTS[name]
        densities = {
            motor: peak / actuator_mass_kg for motor, peak in raw_peaks.items()
        } if actuator_mass_kg > 0.0 else {}
        if actuator_mass_kg <= 0.0 or not raw_peaks:
            density_status = "NOT_EVALUATED"
        else:
            density_status = "PASS" if all(value >= density_required for value in densities.values()) else "FAIL"

        result = {
            "joint": name,
            "sample_count": len(moving_records),
            "actual_max_velocity_deg_s": max_speed * DEG,
            "actual_p99_velocity_deg_s": percentile(actual_speeds, 0.99) * DEG,
            "required_velocity_deg_s": required_velocity * DEG,
            "velocity_status": "PASS" if speed_pass else "FAIL",
            "actual_max_acceleration_deg_s2": max(accelerations, default=0.0) * DEG,
            "actual_range_deg": actual_range * DEG,
            "required_range_deg": required_range * DEG,
            "range_status": "PASS" if range_pass else "FAIL",
            "pd_torque_peak_nm": max(pd_torques, default=0.0),
            "mapped_joint_torque_peak_nm": max(mapped_torques, default=0.0),
            "raw_motor_output_peak": raw_peaks,
            "raw_motor_mode_values": {key: sorted(set(value)) for key, value in raw_modes.items()},
            "raw_motor_cst_fraction": cst_mode_fraction,
            "cst_selection_status": "PASS" if mask_pass else "FAIL",
            "raw_motor_cst_status": "PASS" if motor_mode_pass else "FAIL",
            "actuator_mass_kg": actuator_mass_kg if actuator_mass_kg > 0.0 else None,
            "output_density": densities,
            "output_density_requirement": density_required,
            "output_density_unit": density_unit,
            "output_density_status": density_status,
        }
        result["overall_status"] = (
            "PASS" if speed_pass and range_pass and mask_pass and motor_mode_pass and density_status == "PASS" else
            "FAIL" if not speed_pass or not range_pass or not mask_pass or not motor_mode_pass or density_status == "FAIL" else
            "NOT_EVALUATED"
        )
        results.append(result)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    aborted = any(record["scalars"].get("acceptance_aborted", 0.0) > 0.5 for record in records)
    if aborted or any(item["overall_status"] == "FAIL" for item in results):
        overall_status = "FAIL"
    elif results and all(item["overall_status"] == "PASS" for item in results):
        overall_status = "PASS"
    else:
        overall_status = "NOT_EVALUATED"
    report = {
        "joint_log": str(args.joint_log),
        "solver_mcap": str(args.solver_mcap),
        "motor_order_source": motor_order_source,
        "aborted": aborted,
        "overall_status": overall_status,
        "results": results,
    }
    (args.output_dir / "acceptance_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    lines = [
        "# Jingchu01 单关节验收报告",
        "",
        "| 关节 | 最大速度 (deg/s) | P99 (deg/s) | 实际行程 (deg) | 行程状态 | 控制模式 | 原始输出 | 密度状态 | 总状态 |",
        "|---|---:|---:|---:|---|---|---|---|---|",
    ]
    for result in results:
        raw = ", ".join(f"{key}={value:.3f}" for key, value in result["raw_motor_output_peak"].items()) or "无"
        lines.append(
            f"| {result['joint']} | {result['actual_max_velocity_deg_s']:.2f} | "
            f"{result['actual_p99_velocity_deg_s']:.2f} | {result['actual_range_deg']:.2f} | "
            f"{result['range_status']} | {result['cst_selection_status']}/{result['raw_motor_cst_status']} | "
            f"{raw} | {result['output_density_status']} | {result['overall_status']} |"
        )
    lines += [
        "",
        "> 膝关节原始输出来自映射前 motor_state_tau，单位 N；其余腿部旋转电机单位 Nm。",
        "> actuator_mass_kg 是对应驱动器质量，不是整机质量；未配置时输出密度标记为 NOT_EVALUATED。",
        f"> 电机顺序来源：{motor_order_source}。",
        f"> 整轮状态：{report['overall_status']}；轨迹中止：{'是' if aborted else '否'}。",
    ]
    (args.output_dir / "acceptance_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    try:
        import matplotlib.pyplot as plt
        for cursor, joint_cfg in enumerate(acceptance_joints):
            name = joint_cfg["name"]
            index = joint_names.index(name)
            selected = [r for r in records if int(round(r["scalars"].get("acceptance_joint_cursor", -1))) == cursor]
            if not selected:
                continue
            time0 = float(selected[0]["time_sec"])
            times = [float(r["time_sec"]) - time0 for r in selected]
            fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 6))
            axes[0].plot(times, [r["vectors"]["cmd_q"][index] * DEG for r in selected], label="target")
            axes[0].plot(times, [r["vectors"]["state_q"][index] * DEG for r in selected], label="actual")
            axes[0].set_ylabel("position (deg)")
            axes[0].legend()
            axes[1].plot(times, [r["vectors"]["state_dq"][index] * DEG for r in selected])
            axes[1].axhline(float(joint_cfg["required_actual_velocity"]) * DEG, color="r", linestyle="--")
            axes[1].axhline(-float(joint_cfg["required_actual_velocity"]) * DEG, color="r", linestyle="--")
            axes[1].set_ylabel("velocity (deg/s)")
            axes[1].set_xlabel("time (s)")
            fig.tight_layout()
            fig.savefig(args.output_dir / f"{name}.png", dpi=160)
            plt.close(fig)
    except ImportError:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
