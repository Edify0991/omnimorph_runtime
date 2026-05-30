#!/usr/bin/env python3
"""Plot joint position/velocity/torque curves from an rl_master runtime MCAP log."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import yaml

from runtime_log_utils import load_runtime_messages


def _extract_vector(payload: Dict, prefix: str) -> List[float]:
    values: List[Tuple[int, float]] = []
    for key, value in payload.items():
        if not key.startswith(prefix):
            continue
        try:
            index = int(key.split("__", 1)[1])
            values.append((index, float(value)))
        except Exception:
            continue
    values.sort(key=lambda item: item[0])
    return [value for _, value in values]


def _extract_any_vector(payload: Dict, key: str, prefix: str) -> List[float]:
    value = payload.get(key)
    if isinstance(value, list):
        try:
            return [float(item) for item in value]
        except Exception:
            return []
    return _extract_vector(payload, prefix)


def _load_joint_names(path: Optional[Path], joint_count: int) -> List[str]:
    if path is None:
        return [f"joint_{i}" for i in range(joint_count)]

    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    names = data.get("robot_global_joint_order", [])
    if not isinstance(names, list):
        raise RuntimeError(f"robot_global_joint_order is not a list in {path}")
    if len(names) < joint_count:
        raise RuntimeError(
            f"joint name count mismatch in {path}: expected at least {joint_count}, got {len(names)}"
        )
    return [str(name) for name in names[:joint_count]]


def _load_tick_series(path: Path) -> Dict:
    messages = load_runtime_messages(path, topic="runtime/tick")
    if not messages:
        raise RuntimeError(f"No runtime/tick messages found in {path}")

    times_s: List[float] = []
    deploy_state: List[int] = []
    frame_index: List[int] = []
    joint_q: List[List[float]] = []
    joint_target_q: List[List[float]] = []
    joint_dq: List[List[float]] = []
    joint_cmd_dq: List[List[float]] = []
    joint_tau: List[List[float]] = []
    joint_target_tau: List[List[float]] = []

    t0_ns = messages[0]["log_time_ns"]
    for message in messages:
        payload = message.get("data")
        if not isinstance(payload, dict):
            continue
        times_s.append((float(message["log_time_ns"]) - float(t0_ns)) / 1.0e9)
        deploy_state.append(int(payload.get("deploy_state", 0)))
        frame_index.append(int(payload.get("frame_index", len(frame_index))))
        joint_q.append(_extract_any_vector(payload, "joint_q", "joint_q__"))
        joint_target_q.append(_extract_any_vector(payload, "joint_target_q", "joint_target_q__"))
        joint_dq.append(_extract_any_vector(payload, "joint_dq", "joint_dq__"))
        joint_cmd_dq.append(_extract_any_vector(payload, "joint_cmd_dq", "joint_cmd_dq__"))
        joint_tau.append(_extract_any_vector(payload, "joint_tau", "joint_tau__"))
        joint_target_tau.append(_extract_any_vector(payload, "joint_target_tau", "joint_target_tau__"))

    joint_count = len(joint_q[0])
    if joint_count == 0:
        raise RuntimeError(f"No joint_q fields found in {path}")

    return {
        "times_s": times_s,
        "deploy_state": deploy_state,
        "frame_index": frame_index,
        "joint_q": joint_q,
        "joint_target_q": joint_target_q,
        "joint_dq": joint_dq,
        "joint_cmd_dq": joint_cmd_dq,
        "joint_tau": joint_tau,
        "joint_target_tau": joint_target_tau,
        "joint_count": joint_count,
    }


def _first_running_time(times_s: Sequence[float], deploy_state: Sequence[int]) -> Optional[float]:
    for t, state in zip(times_s, deploy_state):
        if state == 3:
            return t
    return None


def _plot_group(
    out_path: Path,
    title: str,
    times_s: Sequence[float],
    deploy_state: Sequence[int],
    joint_names: Sequence[str],
    measured: Sequence[Sequence[float]],
    target: Sequence[Sequence[float]],
    y_label: str,
) -> None:
    joint_count = len(joint_names)
    cols = 4
    rows = math.ceil(joint_count / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(22, 3.2 * rows), sharex=True)
    axes_list = list(axes.flatten()) if hasattr(axes, "flatten") else [axes]

    running_t = _first_running_time(times_s, deploy_state)

    for joint_idx, ax in enumerate(axes_list):
        if joint_idx >= joint_count:
            ax.axis("off")
            continue
        meas = [row[joint_idx] if joint_idx < len(row) else float("nan") for row in measured]
        ax.plot(times_s, meas, color="#0f6cbd", linewidth=1.0, label="measured")

        if target and any(joint_idx < len(row) for row in target):
            tgt = [row[joint_idx] if joint_idx < len(row) else float("nan") for row in target]
            if any(not math.isnan(value) for value in tgt):
                ax.plot(times_s, tgt, color="#d83b01", linewidth=0.9, linestyle="--", label="target")

        if running_t is not None:
            ax.axvline(running_t, color="#888888", linewidth=0.8, linestyle=":")

        ax.set_title(joint_names[joint_idx], fontsize=9)
        ax.grid(alpha=0.25)
        if joint_idx % cols == 0:
            ax.set_ylabel(y_label)
        if joint_idx >= joint_count - cols:
            ax.set_xlabel("time [s]")

    handles, labels = axes_list[0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper right")
    fig.suptitle(title, fontsize=16)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot joint runtime curves from an MCAP log")
    parser.add_argument("--mcap", required=True, help="Path to runtime .mcap log")
    parser.add_argument(
        "--joint-name-yaml",
        default="src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml",
        help="YAML file containing robot_global_joint_order",
    )
    parser.add_argument("--output-dir", required=True, help="Directory for output PNG files")
    args = parser.parse_args()

    mcap_path = Path(args.mcap)
    if not mcap_path.exists():
        raise FileNotFoundError(mcap_path)

    output_dir = Path(args.output_dir)
    joint_yaml = Path(args.joint_name_yaml) if args.joint_name_yaml else None

    series = _load_tick_series(mcap_path)
    joint_names = _load_joint_names(joint_yaml, series["joint_count"])

    stem = mcap_path.stem
    _plot_group(
        output_dir / f"{stem}_joint_position.png",
        f"{stem} | Joint Position",
        series["times_s"],
        series["deploy_state"],
        joint_names,
        series["joint_q"],
        series["joint_target_q"],
        "rad",
    )
    _plot_group(
        output_dir / f"{stem}_joint_velocity.png",
        f"{stem} | Joint Velocity",
        series["times_s"],
        series["deploy_state"],
        joint_names,
        series["joint_dq"],
        series["joint_cmd_dq"],
        "rad/s",
    )
    _plot_group(
        output_dir / f"{stem}_joint_torque.png",
        f"{stem} | Joint Torque",
        series["times_s"],
        series["deploy_state"],
        joint_names,
        series["joint_tau"],
        series["joint_target_tau"],
        "Nm",
    )

    print(f"output_dir: {output_dir}")


if __name__ == "__main__":
    main()
