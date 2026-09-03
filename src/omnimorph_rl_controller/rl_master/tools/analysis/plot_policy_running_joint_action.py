#!/usr/bin/env python3
"""Create publication-quality joint-state and aligned policy-action plots from MCAP.

The plots use the finite policy-running interval: from the first inference tick
in RUNNING through the final tick that advances policy_step_index.  Joint data
remain at low-level control cadence, while policy actions retain their original
source timestamps and are drawn as zero-order-held 50 Hz step signals.
"""

from __future__ import annotations

import argparse
import bisect
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from runtime_log_utils import load_runtime_messages


RUNNING_DEPLOY_STATE = 3
OKABE_ITO = {
    "blue": "#0072B2",
    "orange": "#D55E00",
    "green": "#009E73",
    "purple": "#CC79A7",
    "gray": "#4D4D4D",
    "light_gray": "#D9D9D9",
}
COMPARISON_QUANTITIES = ("position", "velocity", "torque", "action")
DEFAULT_COMPARISON_JOINTS = (
    "right_hip_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
    "left_hip_pitch",
    "left_knee_pitch",
    "left_ankle_pitch",
)


def configure_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 220,
            "font.family": "DejaVu Sans",
            "font.size": 9,
            "axes.titlesize": 9,
            "axes.labelsize": 9,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.linewidth": 0.8,
            "axes.grid": True,
            "grid.color": "#D0D0D0",
            "grid.linewidth": 0.55,
            "grid.alpha": 0.72,
            "legend.frameon": False,
            "xtick.direction": "out",
            "ytick.direction": "out",
        }
    )


def payload_of(message: Dict[str, Any]) -> Dict[str, Any]:
    payload = message.get("data")
    if not isinstance(payload, dict):
        raise RuntimeError(f"Expected JSON object on topic {message.get('topic')!r}")
    return payload


def vector_of(payload: Dict[str, Any], field: str) -> List[float]:
    value = payload.get(field)
    if not isinstance(value, list):
        return []
    try:
        return [float(item) for item in value]
    except (TypeError, ValueError):
        return []


def find_policy_window(ticks: Sequence[Dict[str, Any]]) -> Tuple[int, int, int, int]:
    start: Optional[int] = None
    end: Optional[int] = None
    first_step: Optional[int] = None
    last_step: Optional[int] = None
    for index, message in enumerate(ticks):
        tick = payload_of(message)
        if tick.get("deploy_state") != RUNNING_DEPLOY_STATE or not tick.get("policy_ran_this_tick", False):
            continue
        step = int(tick.get("policy_step_index", 0))
        if start is None:
            start = end = index
            first_step = last_step = step
        elif step > int(last_step):
            end = index
            last_step = step
    if start is None or end is None or first_step is None or last_step is None:
        raise RuntimeError("No RUNNING policy inference window found")
    return start, end, first_step, last_step


def root_joint_order(path: Path, count: int) -> List[str]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    names = document.get("robot_global_joint_order", [])
    if not isinstance(names, list) or len(names) < count:
        raise RuntimeError(f"robot_global_joint_order in {path} does not contain {count} joints")
    return [str(name) for name in names[:count]]


def profile_action_order(path: Path, count: int) -> List[str]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not document:
        raise RuntimeError(f"Invalid profile YAML: {path}")
    profile = next(iter(document.values()))
    names = profile.get("action_joint_order", []) if isinstance(profile, dict) else []
    if not isinstance(names, list) or len(names) != count:
        raise RuntimeError(f"action_joint_order in {path} must contain exactly {count} joints")
    return [str(name) for name in names]


def matrix_from_ticks(ticks: Sequence[Dict[str, Any]], field: str, count: int) -> np.ndarray:
    matrix = np.full((len(ticks), count), np.nan, dtype=np.float64)
    for row_index, message in enumerate(ticks):
        values = vector_of(payload_of(message), field)
        matrix[row_index, : min(count, len(values))] = values[:count]
    return matrix


def source_actions(
    input_path: Path, start_ns: int, end_ns: int, count: int
) -> Tuple[np.ndarray, np.ndarray]:
    messages = load_runtime_messages(input_path, "runtime/source/policy_action")
    messages.sort(key=lambda message: int(message["log_time_ns"]))
    selected = [message for message in messages if start_ns <= int(message["log_time_ns"]) <= end_ns]
    if not selected:
        raise RuntimeError("No policy_action samples overlap the selected policy-running window")
    times = np.asarray([int(message["log_time_ns"]) for message in selected], dtype=np.int64)
    action = np.full((len(selected), count), np.nan, dtype=np.float64)
    for row_index, message in enumerate(selected):
        values = payload_of(message).get("values", {}).get("action", [])
        if isinstance(values, list):
            action[row_index, : min(count, len(values))] = [float(value) for value in values[:count]]
    return times, action


def decorate_axis(ax: plt.Axes) -> None:
    ax.axhline(0.0, color=OKABE_ITO["light_gray"], linewidth=0.75, zorder=0)
    ax.margins(x=0.012)


def plot_joint_quantity(
    output_path: Path,
    title: str,
    times_s: np.ndarray,
    names: Sequence[str],
    state: np.ndarray,
    command: np.ndarray,
    unit: str,
    state_label: str,
    command_label: str,
    color: str,
) -> None:
    columns = 4
    rows = math.ceil(len(names) / columns)
    fig, axes = plt.subplots(rows, columns, figsize=(22, 2.7 * rows), sharex=True, layout="constrained")
    axes_flat = list(np.asarray(axes).ravel())

    for joint_index, ax in enumerate(axes_flat):
        if joint_index >= len(names):
            ax.axis("off")
            continue
        ax.plot(times_s, state[:, joint_index], color=color, linewidth=0.85, label=state_label)
        if np.isfinite(command[:, joint_index]).any():
            ax.plot(
                times_s,
                command[:, joint_index],
                color=OKABE_ITO["orange"],
                linewidth=0.75,
                linestyle="--",
                alpha=0.9,
                label=command_label,
            )
        decorate_axis(ax)
        ax.set_title(names[joint_index], fontweight="semibold", pad=5)
        if joint_index % columns == 0:
            ax.set_ylabel(unit)
        if joint_index >= len(names) - columns:
            ax.set_xlabel("policy-running time [s]")

    handles, labels = axes_flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncols=2, bbox_to_anchor=(0.5, 1.015))
    fig.suptitle(title, fontsize=16, fontweight="bold", y=1.04)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def plot_actions(
    output_path: Path, times_s: np.ndarray, names: Sequence[str], actions: np.ndarray, title: str
) -> None:
    columns = 4
    rows = math.ceil(len(names) / columns)
    fig, axes = plt.subplots(rows, columns, figsize=(22, 2.7 * rows), sharex=True, layout="constrained")
    axes_flat = list(np.asarray(axes).ravel())
    for action_index, ax in enumerate(axes_flat):
        if action_index >= len(names):
            ax.axis("off")
            continue
        ax.step(
            times_s,
            actions[:, action_index],
            where="post",
            color=OKABE_ITO["green"],
            linewidth=0.95,
        )
        decorate_axis(ax)
        ax.set_title(names[action_index], fontweight="semibold", pad=5)
        if action_index % columns == 0:
            ax.set_ylabel("policy action")
        if action_index >= len(names) - columns:
            ax.set_xlabel("policy-running time [s]")

    fig.suptitle(title, fontsize=16, fontweight="bold", y=1.025)
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def nearest_tick_offsets_ms(tick_times_ns: np.ndarray, action_times_ns: np.ndarray) -> np.ndarray:
    offsets = np.empty(len(action_times_ns), dtype=np.float64)
    tick_times = tick_times_ns.tolist()
    for index, action_time in enumerate(action_times_ns.tolist()):
        pos = bisect.bisect_left(tick_times, action_time)
        candidates = []
        if pos < len(tick_times):
            candidates.append(tick_times[pos] - action_time)
        if pos > 0:
            candidates.append(tick_times[pos - 1] - action_time)
        offsets[index] = min(candidates, key=abs) / 1.0e6
    return offsets


def plot_alignment(output_path: Path, action_times_s: np.ndarray, offsets_ms: np.ndarray, title: str) -> None:
    fig, (ax_time, ax_hist) = plt.subplots(2, 1, figsize=(12, 6.5), height_ratios=(2.1, 1), layout="constrained")
    ax_time.axhline(0.0, color=OKABE_ITO["gray"], linewidth=0.8)
    ax_time.scatter(action_times_s, offsets_ms, s=12, color=OKABE_ITO["purple"], alpha=0.8, linewidths=0)
    ax_time.set_ylabel("nearest control-tick offset [ms]")
    ax_time.set_xlabel("policy-running time [s]")
    ax_time.set_title(
        f"mean |offset| = {np.mean(np.abs(offsets_ms)):.3f} ms, "
        f"max |offset| = {np.max(np.abs(offsets_ms)):.3f} ms",
        fontsize=10,
    )
    ax_time.grid(True)

    bins = min(60, max(15, int(np.sqrt(len(offsets_ms)))))
    ax_hist.hist(offsets_ms, bins=bins, color=OKABE_ITO["purple"], edgecolor="white", linewidth=0.4)
    ax_hist.set_xlabel("nearest control-tick offset [ms]")
    ax_hist.set_ylabel("action samples")
    ax_hist.grid(axis="y")
    fig.suptitle(title, fontsize=15, fontweight="bold")
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def parse_csv_argument(text: str) -> List[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def resolve_comparison_joints(requested: str, joint_names: Sequence[str]) -> List[str]:
    if requested.strip().lower() == "all":
        return list(joint_names)
    names = parse_csv_argument(requested)
    if not names:
        raise ValueError("--compare-joints must list at least one joint or use 'all'")
    unknown = [name for name in names if name not in joint_names]
    if unknown:
        raise ValueError(f"Unknown --compare-joints entries: {', '.join(unknown)}")
    return names


def resolve_comparison_quantities(requested: str) -> List[str]:
    quantities = parse_csv_argument(requested.lower())
    if not quantities:
        raise ValueError("--compare-quantities must list at least one quantity")
    unknown = [quantity for quantity in quantities if quantity not in COMPARISON_QUANTITIES]
    if unknown:
        raise ValueError(
            "Unknown --compare-quantities entries: "
            f"{', '.join(unknown)}. Supported: {', '.join(COMPARISON_QUANTITIES)}"
        )
    return quantities


def plot_selected_comparison(
    output_path: Path,
    tick_times_s: np.ndarray,
    action_times_s: np.ndarray,
    selected_joint_names: Sequence[str],
    joint_indices: Sequence[int],
    action_indices: Sequence[int],
    quantities: Sequence[str],
    state_q: np.ndarray,
    state_dq: np.ndarray,
    state_tau: np.ndarray,
    actions: np.ndarray,
) -> None:
    """Plot requested signals in a single quantity-by-joint comparison figure.

    Different units deliberately remain in separate rows, which is more honest
    than overlaying rad, rad/s, N·m, and unitless policy actions on one axis.
    Every panel still has the same time axis, enabling direct temporal
    comparison across physical quantities and the lower-rate action.
    """

    quantity_specs = {
        "position": (state_q, tick_times_s, "joint position [rad]", OKABE_ITO["blue"], "line"),
        "velocity": (state_dq, tick_times_s, "joint velocity [rad/s]", OKABE_ITO["purple"], "line"),
        "torque": (state_tau, tick_times_s, "joint torque [N·m]", OKABE_ITO["orange"], "line"),
        "action": (actions, action_times_s, "policy action [–]", OKABE_ITO["green"], "step"),
    }
    rows, columns = len(quantities), len(selected_joint_names)
    fig, axes = plt.subplots(
        rows,
        columns,
        figsize=(4.0 * columns, 2.55 * rows + 0.55),
        squeeze=False,
        sharex="col",
        layout="constrained",
    )

    for row_index, quantity in enumerate(quantities):
        values, times_s, ylabel, color, style = quantity_specs[quantity]
        source_indices = action_indices if quantity == "action" else joint_indices
        for column_index, (joint_name, source_index) in enumerate(zip(selected_joint_names, source_indices)):
            ax = axes[row_index, column_index]
            if style == "step":
                ax.step(times_s, values[:, source_index], where="post", color=color, linewidth=1.0)
            else:
                ax.plot(times_s, values[:, source_index], color=color, linewidth=0.9)
            decorate_axis(ax)
            if row_index == 0:
                ax.set_title(joint_name, fontweight="semibold", pad=6)
            if column_index == 0:
                ax.set_ylabel(ylabel)
            if row_index == rows - 1:
                ax.set_xlabel("policy-running time [s]")

    fig.suptitle(
        "Selected joint-state / policy-action comparison "
        "(state: 500 Hz, action: 50 Hz; original timestamps)",
        fontsize=15,
        fontweight="bold",
        y=1.02,
    )
    fig.savefig(output_path, bbox_inches="tight")
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, required=True, help="Input runtime MCAP.")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory for PNG figures.")
    parser.add_argument(
        "--joint-name-yaml",
        type=Path,
        default=Path("src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml"),
        help="Root YAML containing robot_global_joint_order.",
    )
    parser.add_argument(
        "--profile-yaml",
        type=Path,
        default=Path("src/omnimorph_rl_controller/rl_master/config/profiles/beyondmimic_jc01_dance_full_body_wo_state_estimation.yaml"),
        help="Profile YAML containing action_joint_order.",
    )
    parser.add_argument(
        "--compare-joints",
        default=",".join(DEFAULT_COMPARISON_JOINTS),
        help=(
            "Comma-separated canonical joint names for the combined comparison figure, "
            "or 'all'. Default: six hip/knee/ankle pitch joints."
        ),
    )
    parser.add_argument(
        "--compare-quantities",
        default=",".join(COMPARISON_QUANTITIES),
        help="Comma-separated quantities for the comparison figure: position,velocity,torque,action.",
    )
    return parser.parse_args()


def main() -> None:
    configure_style()
    args = parse_args()
    input_path = args.mcap.resolve()
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    if not args.joint_name_yaml.is_file() or not args.profile_yaml.is_file():
        raise FileNotFoundError("Joint-order YAML or profile YAML was not found")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    ticks = load_runtime_messages(input_path, "runtime/tick")
    ticks.sort(key=lambda message: int(message["log_time_ns"]))
    start_index, end_index, first_step, last_step = find_policy_window(ticks)
    selected_ticks = ticks[start_index : end_index + 1]
    tick_times_ns = np.asarray([int(message["log_time_ns"]) for message in selected_ticks], dtype=np.int64)
    start_ns, end_ns = int(tick_times_ns[0]), int(tick_times_ns[-1])
    tick_times_s = (tick_times_ns - start_ns).astype(np.float64) / 1.0e9

    joint_count = len(vector_of(payload_of(selected_ticks[0]), "joint_state_q"))
    if joint_count <= 0:
        raise RuntimeError("joint_state_q is missing from the selected runtime/tick messages")
    joint_names = root_joint_order(args.joint_name_yaml, joint_count)
    action_times_ns, action_values = source_actions(input_path, start_ns, end_ns, joint_count)
    action_names = profile_action_order(args.profile_yaml, action_values.shape[1])
    action_times_s = (action_times_ns - start_ns).astype(np.float64) / 1.0e9
    comparison_joints = resolve_comparison_joints(args.compare_joints, joint_names)
    comparison_quantities = resolve_comparison_quantities(args.compare_quantities)
    action_index_by_name = {name: index for index, name in enumerate(action_names)}
    missing_action_joints = [name for name in comparison_joints if name not in action_index_by_name]
    if missing_action_joints and "action" in comparison_quantities:
        raise RuntimeError(
            "Selected comparison joints have no policy action dimension: " + ", ".join(missing_action_joints)
        )
    joint_index_by_name = {name: index for index, name in enumerate(joint_names)}
    comparison_joint_indices = [joint_index_by_name[name] for name in comparison_joints]
    comparison_action_indices = [action_index_by_name.get(name, 0) for name in comparison_joints]

    state_q = matrix_from_ticks(selected_ticks, "joint_state_q", joint_count)
    state_dq = matrix_from_ticks(selected_ticks, "joint_state_dq", joint_count)
    state_tau = matrix_from_ticks(selected_ticks, "joint_state_tau", joint_count)

    stem = input_path.stem
    clip_label = f"policy steps {first_step}–{last_step} | {len(selected_ticks):,} control frames"
    plot_joint_quantity(
        args.output_dir / f"{stem}_policy_running_joint_position.png",
        f"Joint position — {clip_label}",
        tick_times_s,
        joint_names,
        state_q,
        matrix_from_ticks(selected_ticks, "joint_cmd_q", joint_count),
        "rad",
        "state",
        "command",
        OKABE_ITO["blue"],
    )
    plot_joint_quantity(
        args.output_dir / f"{stem}_policy_running_joint_velocity.png",
        f"Joint velocity — {clip_label}",
        tick_times_s,
        joint_names,
        state_dq,
        matrix_from_ticks(selected_ticks, "joint_cmd_dq", joint_count),
        "rad/s",
        "state",
        "command",
        OKABE_ITO["purple"],
    )
    plot_joint_quantity(
        args.output_dir / f"{stem}_policy_running_joint_torque.png",
        f"Joint torque — {clip_label}",
        tick_times_s,
        joint_names,
        state_tau,
        matrix_from_ticks(selected_ticks, "joint_cmd_tau", joint_count),
        "N·m",
        "state",
        "command",
        OKABE_ITO["orange"],
    )
    plot_actions(
        args.output_dir / f"{stem}_policy_running_action.png",
        action_times_s,
        action_names,
        action_values,
        f"Policy action (original 50 Hz timestamps) — {clip_label}",
    )
    offsets_ms = nearest_tick_offsets_ms(tick_times_ns, action_times_ns)
    plot_alignment(
        args.output_dir / f"{stem}_policy_action_alignment.png",
        action_times_s,
        offsets_ms,
        "Policy-action timestamps aligned to nearest 500 Hz control tick",
    )
    plot_selected_comparison(
        args.output_dir / f"{stem}_policy_running_selected_comparison.png",
        tick_times_s,
        action_times_s,
        comparison_joints,
        comparison_joint_indices,
        comparison_action_indices,
        comparison_quantities,
        state_q,
        state_dq,
        state_tau,
        action_values,
    )

    print(f"output_dir: {args.output_dir.resolve()}")
    print(f"policy_step_index: {first_step}..{last_step}")
    print(f"control_frames: {len(selected_ticks)}")
    print(f"action_samples: {len(action_times_ns)}")
    print(f"action_alignment_mean_abs_ms: {np.mean(np.abs(offsets_ms)):.6f}")
    print(f"action_alignment_max_abs_ms: {np.max(np.abs(offsets_ms)):.6f}")
    print(f"comparison_joints: {','.join(comparison_joints)}")
    print(f"comparison_quantities: {','.join(comparison_quantities)}")


if __name__ == "__main__":
    main()
