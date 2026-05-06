#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import subprocess
import sys
import types
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


def install_compression_fallbacks() -> None:
    try:
        import zstandard  # type: ignore # noqa: F401
    except Exception:
        class _ZstdDecompressor:
            def decompress(self, payload: bytes) -> bytes:
                proc = subprocess.run(
                    ["/usr/bin/zstd", "-d", "-q", "-c"],
                    input=payload,
                    stdout=subprocess.PIPE,
                    check=True,
                )
                return proc.stdout

        zstd_module = types.ModuleType("zstandard")
        zstd_module.ZstdDecompressor = _ZstdDecompressor
        sys.modules["zstandard"] = zstd_module

    try:
        import lz4.frame  # type: ignore # noqa: F401
    except Exception:
        lz4_module = types.ModuleType("lz4")
        frame_module = types.ModuleType("lz4.frame")

        def _unsupported_lz4(_payload: bytes) -> bytes:
            raise RuntimeError("lz4 Python module is unavailable; this script currently supports zstd-compressed logs.")

        frame_module.decompress = _unsupported_lz4
        lz4_module.frame = frame_module
        sys.modules["lz4"] = lz4_module
        sys.modules["lz4.frame"] = frame_module


install_compression_fallbacks()
sys.path.insert(0, str((Path(__file__).resolve().parents[1] / "src/humanoid_rl_controller/rl_master/tools/analysis").resolve()))

from runtime_log_utils import load_runtime_messages  # noqa: E402


def parse_indices(text: str) -> List[int]:
    if not text.strip():
        return []
    return [int(token.strip()) for token in text.split(",") if token.strip()]


def default_term_dim(term: Dict[str, Any]) -> int:
    name = str(term.get("name", "")).strip()
    components = term.get("components") or []
    count = term.get("count")

    if isinstance(count, int) and count > 0:
        return count
    if isinstance(components, list) and components:
        return len(components)

    if name == "phase":
        return 2
    if name == "command":
        return 3
    if name in {"joint_pos", "joint_vel", "last_action"}:
        return 12
    if name in {"base_lin_vel", "base_ang_vel", "base_rpy", "base_euler"}:
        return 3
    if name == "base_quat":
        return 4
    return 0


def load_manifest_terms(manifest_path: Path) -> List[Dict[str, Any]]:
    root = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest = root.get("observation_manifest", {})
    terms = manifest.get("terms", [])
    out: List[Dict[str, Any]] = []
    running_offset = 0

    for term in terms:
        if not isinstance(term, dict):
            continue
        if not term.get("enabled", True):
            continue
        dim = default_term_dim(term)
        if dim <= 0:
            continue
        out.append(
            {
                "name": str(term.get("name", "")),
                "start": running_offset,
                "end": running_offset + dim,
                "dim": dim,
            }
        )
        running_offset += dim
    return out


def resolve_manifest_path(config_payload: Dict[str, Any]) -> Optional[Path]:
    active_section = str(config_payload.get("active_config_section", "")).strip()
    if not active_section:
        return None

    profiles = config_payload.get("profiles")
    if not isinstance(profiles, list):
        return None

    for profile in profiles:
        if not isinstance(profile, dict):
            continue
        if str(profile.get("config_section", "")).strip() != active_section:
            continue
        raw_path = str(profile.get("observation_manifest_path", "")).strip()
        if not raw_path:
            return None
        path = Path(raw_path)
        return path if path.exists() else None
    return None


def extract_term_series(
    ticks: Sequence[Dict[str, Any]],
    term_slices: Dict[str, Tuple[int, int]],
    term_name: str,
) -> List[List[float]]:
    if term_name not in term_slices:
        return []
    start, end = term_slices[term_name]
    out: List[List[float]] = []
    for tick in ticks:
        obs = tick.get("observation")
        if not isinstance(obs, list) or len(obs) < end:
            out.append([])
            continue
        out.append([float(x) for x in obs[start:end]])
    return out


def select_ticks(
    ticks: Sequence[Dict[str, Any]],
    running_only: bool,
) -> List[Dict[str, Any]]:
    if not running_only:
        return [tick for tick in ticks if isinstance(tick, dict)]
    return [tick for tick in ticks if isinstance(tick, dict) and int(tick.get("deploy_state", 0)) == 3]


def print_event_timeline(events: Sequence[Dict[str, Any]]) -> None:
    print("=== Event Timeline ===")
    for event in events:
        if not isinstance(event, dict):
            continue
        print(event)


def print_command_changes(ticks: Sequence[Dict[str, Any]]) -> None:
    print("=== Teleop Command Changes ===")
    previous: Optional[Tuple[float, float, float]] = None
    for tick in ticks:
        current = (
            round(float(tick.get("cmd_vx", 0.0)), 4),
            round(float(tick.get("cmd_vy", 0.0)), 4),
            round(float(tick.get("cmd_dyaw", 0.0)), 4),
        )
        if current != previous:
            print(
                f"{tick.get('monotonic_time_sec', 0.0):.6f}  "
                f"vx={current[0]: .4f} vy={current[1]: .4f} dyaw={current[2]: .4f}"
            )
            previous = current


def make_output_dir(path: Optional[str]) -> Optional[Path]:
    if not path:
        return None
    out_dir = Path(path)
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def save_or_show(fig, output_dir: Optional[Path], stem: str, show: bool) -> None:
    if output_dir is not None:
        output_path = output_dir / f"{stem}.png"
        fig.savefig(output_path, dpi=160)
        print(f"saved_plot: {output_path}")
    if show:
        import matplotlib.pyplot as plt

        plt.show()
    else:
        import matplotlib.pyplot as plt

        plt.close(fig)


def plot_motion_overview(
    ticks: Sequence[Dict[str, Any]],
    term_slices: Dict[str, Tuple[int, int]],
    output_dir: Optional[Path],
    show: bool,
) -> None:
    import matplotlib.pyplot as plt

    if not ticks:
        raise RuntimeError("No ticks selected for plotting")

    t0 = float(ticks[0]["monotonic_time_sec"])
    ts = [float(tick["monotonic_time_sec"]) - t0 for tick in ticks]

    cmd_vx = [float(tick.get("cmd_vx", 0.0)) for tick in ticks]
    cmd_vy = [float(tick.get("cmd_vy", 0.0)) for tick in ticks]
    cmd_dyaw = [float(tick.get("cmd_dyaw", 0.0)) for tick in ticks]

    base_ang_vel = extract_term_series(ticks, term_slices, "base_ang_vel")
    base_rpy = extract_term_series(ticks, term_slices, "base_rpy")
    base_euler = extract_term_series(ticks, term_slices, "base_euler")
    observation_command = extract_term_series(ticks, term_slices, "command")

    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    axes[0].plot(ts, cmd_vx, label="cmd_vx")
    axes[0].plot(ts, cmd_vy, label="cmd_vy")
    axes[0].plot(ts, cmd_dyaw, label="cmd_dyaw")
    axes[0].set_ylabel("teleop")
    axes[0].grid(alpha=0.3)
    axes[0].legend()

    if observation_command and any(row for row in observation_command):
        axes[1].plot(ts, [row[0] if len(row) > 0 else math.nan for row in observation_command], label="obs_cmd_vx")
        axes[1].plot(ts, [row[1] if len(row) > 1 else math.nan for row in observation_command], label="obs_cmd_vy")
        axes[1].plot(ts, [row[2] if len(row) > 2 else math.nan for row in observation_command], label="obs_cmd_dyaw")
        axes[1].legend()
    axes[1].set_ylabel("obs command")
    axes[1].grid(alpha=0.3)

    if base_ang_vel and any(row for row in base_ang_vel):
        axes[2].plot(ts, [row[0] if len(row) > 0 else math.nan for row in base_ang_vel], label="wx")
        axes[2].plot(ts, [row[1] if len(row) > 1 else math.nan for row in base_ang_vel], label="wy")
        axes[2].plot(ts, [row[2] if len(row) > 2 else math.nan for row in base_ang_vel], label="wz")
        axes[2].legend()
    axes[2].set_ylabel("base ang vel")
    axes[2].grid(alpha=0.3)

    orientation_source = base_rpy if base_rpy and any(row for row in base_rpy) else base_euler
    if orientation_source and any(row for row in orientation_source):
        axes[3].plot(ts, [row[0] if len(row) > 0 else math.nan for row in orientation_source], label="roll")
        axes[3].plot(ts, [row[1] if len(row) > 1 else math.nan for row in orientation_source], label="pitch")
        axes[3].plot(ts, [row[2] if len(row) > 2 else math.nan for row in orientation_source], label="yaw")
        axes[3].legend()
    axes[3].set_ylabel("base rpy")
    axes[3].set_xlabel("time since selected window start (s)")
    axes[3].grid(alpha=0.3)

    fig.suptitle("Runtime Motion Overview")
    fig.tight_layout()
    save_or_show(fig, output_dir, "motion_overview", show)


def plot_joint_tracking(
    ticks: Sequence[Dict[str, Any]],
    joint_names: Sequence[str],
    joint_indices: Sequence[int],
    output_dir: Optional[Path],
    show: bool,
) -> None:
    import matplotlib.pyplot as plt

    if not ticks:
        raise RuntimeError("No ticks selected for plotting")
    if not joint_indices:
        raise RuntimeError("No joint indices selected")

    t0 = float(ticks[0]["monotonic_time_sec"])
    ts = [float(tick["monotonic_time_sec"]) - t0 for tick in ticks]

    count = len(joint_indices)
    cols = 3
    rows = math.ceil(count / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(14, max(4, rows * 2.8)), sharex=True)
    axes = axes if isinstance(axes, list) else getattr(axes, "flatten", lambda: [axes])()
    if not isinstance(axes, list):
        axes = list(axes)

    for plot_idx, joint_idx in enumerate(joint_indices):
        axis = axes[plot_idx]
        q = [
            (tick.get("joint_q") or [math.nan] * (joint_idx + 1))[joint_idx]
            if len(tick.get("joint_q") or []) > joint_idx
            else math.nan
            for tick in ticks
        ]
        target_q = [
            (tick.get("joint_target_q") or [math.nan] * (joint_idx + 1))[joint_idx]
            if len(tick.get("joint_target_q") or []) > joint_idx
            else math.nan
            for tick in ticks
        ]
        joint_label = joint_names[joint_idx] if joint_idx < len(joint_names) else f"joint_{joint_idx}"
        axis.plot(ts, q, label="q", linewidth=1.0)
        axis.plot(ts, target_q, label="target_q", linewidth=1.0)
        axis.set_title(f"{joint_idx}: {joint_label}")
        axis.grid(alpha=0.3)

    for axis in axes[count:]:
        axis.axis("off")

    axes[0].legend()
    fig.suptitle("Joint Tracking")
    fig.tight_layout()
    save_or_show(fig, output_dir, "joint_tracking", show)


def plot_torque_tracking(
    ticks: Sequence[Dict[str, Any]],
    joint_names: Sequence[str],
    joint_indices: Sequence[int],
    output_dir: Optional[Path],
    show: bool,
) -> None:
    import matplotlib.pyplot as plt

    if not ticks or not joint_indices:
        return

    t0 = float(ticks[0]["monotonic_time_sec"])
    ts = [float(tick["monotonic_time_sec"]) - t0 for tick in ticks]

    count = len(joint_indices)
    cols = 3
    rows = math.ceil(count / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(14, max(4, rows * 2.8)), sharex=True)
    axes = axes if isinstance(axes, list) else getattr(axes, "flatten", lambda: [axes])()
    if not isinstance(axes, list):
        axes = list(axes)

    for plot_idx, joint_idx in enumerate(joint_indices):
        axis = axes[plot_idx]
        joint_tau = [
            (tick.get("joint_tau") or [math.nan] * (joint_idx + 1))[joint_idx]
            if len(tick.get("joint_tau") or []) > joint_idx
            else math.nan
            for tick in ticks
        ]
        target_tau = [
            (tick.get("joint_target_tau") or [math.nan] * (joint_idx + 1))[joint_idx]
            if len(tick.get("joint_target_tau") or []) > joint_idx
            else math.nan
            for tick in ticks
        ]
        joint_label = joint_names[joint_idx] if joint_idx < len(joint_names) else f"joint_{joint_idx}"
        axis.plot(ts, joint_tau, label="joint_tau", linewidth=1.0)
        axis.plot(ts, target_tau, label="target_tau", linewidth=1.0)
        axis.set_title(f"{joint_idx}: {joint_label}")
        axis.grid(alpha=0.3)

    for axis in axes[count:]:
        axis.axis("off")

    axes[0].legend()
    fig.suptitle("Joint Torque Tracking")
    fig.tight_layout()
    save_or_show(fig, output_dir, "joint_torque_tracking", show)


def plot_motor_tracking(
    ticks: Sequence[Dict[str, Any]],
    motor_names: Sequence[str],
    motor_indices: Sequence[int],
    output_dir: Optional[Path],
    show: bool,
) -> None:
    import matplotlib.pyplot as plt

    if not ticks or not motor_indices:
        return

    t0 = float(ticks[0]["monotonic_time_sec"])
    ts = [float(tick["monotonic_time_sec"]) - t0 for tick in ticks]

    count = len(motor_indices)
    cols = 3
    rows = math.ceil(count / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(14, max(4, rows * 2.8)), sharex=True)
    axes = axes if isinstance(axes, list) else getattr(axes, "flatten", lambda: [axes])()
    if not isinstance(axes, list):
        axes = list(axes)

    for plot_idx, motor_idx in enumerate(motor_indices):
        axis = axes[plot_idx]
        motor_q = [
            (tick.get("motor_state_q") or [math.nan] * (motor_idx + 1))[motor_idx]
            if len(tick.get("motor_state_q") or []) > motor_idx
            else math.nan
            for tick in ticks
        ]
        motor_cmd_q = [
            (tick.get("motor_cmd_q") or [math.nan] * (motor_idx + 1))[motor_idx]
            if len(tick.get("motor_cmd_q") or []) > motor_idx
            else math.nan
            for tick in ticks
        ]
        motor_label = motor_names[motor_idx] if motor_idx < len(motor_names) else f"motor_{motor_idx}"
        axis.plot(ts, motor_q, label="motor_state_q", linewidth=1.0)
        axis.plot(ts, motor_cmd_q, label="motor_cmd_q", linewidth=1.0)
        axis.set_title(f"{motor_idx}: {motor_label}")
        axis.grid(alpha=0.3)

    for axis in axes[count:]:
        axis.axis("off")

    axes[0].legend()
    fig.suptitle("Motor Position Tracking")
    fig.tight_layout()
    save_or_show(fig, output_dir, "motor_tracking", show)


def plot_motor_torque_tracking(
    ticks: Sequence[Dict[str, Any]],
    motor_names: Sequence[str],
    motor_indices: Sequence[int],
    output_dir: Optional[Path],
    show: bool,
) -> None:
    import matplotlib.pyplot as plt

    if not ticks or not motor_indices:
        return

    t0 = float(ticks[0]["monotonic_time_sec"])
    ts = [float(tick["monotonic_time_sec"]) - t0 for tick in ticks]

    count = len(motor_indices)
    cols = 3
    rows = math.ceil(count / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(14, max(4, rows * 2.8)), sharex=True)
    axes = axes if isinstance(axes, list) else getattr(axes, "flatten", lambda: [axes])()
    if not isinstance(axes, list):
        axes = list(axes)

    for plot_idx, motor_idx in enumerate(motor_indices):
        axis = axes[plot_idx]
        motor_tau = [
            (tick.get("motor_state_tau") or [math.nan] * (motor_idx + 1))[motor_idx]
            if len(tick.get("motor_state_tau") or []) > motor_idx
            else math.nan
            for tick in ticks
        ]
        motor_cmd_tau = [
            (tick.get("motor_cmd_tau") or [math.nan] * (motor_idx + 1))[motor_idx]
            if len(tick.get("motor_cmd_tau") or []) > motor_idx
            else math.nan
            for tick in ticks
        ]
        motor_mode = [
            (tick.get("motor_cmd_mode") or [math.nan] * (motor_idx + 1))[motor_idx]
            if len(tick.get("motor_cmd_mode") or []) > motor_idx
            else math.nan
            for tick in ticks
        ]
        motor_label = motor_names[motor_idx] if motor_idx < len(motor_names) else f"motor_{motor_idx}"
        axis.plot(ts, motor_tau, label="motor_state_tau", linewidth=1.0)
        axis.plot(ts, motor_cmd_tau, label="motor_cmd_tau", linewidth=1.0)
        mode_axis = axis.twinx()
        mode_axis.plot(ts, motor_mode, label="motor_cmd_mode", linewidth=0.8, linestyle="--", color="tab:green", alpha=0.7)
        mode_axis.set_ylabel("mode")
        axis.set_title(f"{motor_idx}: {motor_label}")
        axis.grid(alpha=0.3)

        if plot_idx == 0:
            lines, labels = axis.get_legend_handles_labels()
            mode_lines, mode_labels = mode_axis.get_legend_handles_labels()
            axis.legend(lines + mode_lines, labels + mode_labels, loc="upper right")

    for axis in axes[count:]:
        axis.axis("off")

    fig.suptitle("Motor Torque and Mode Tracking")
    fig.tight_layout()
    save_or_show(fig, output_dir, "motor_torque_tracking", show)


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze and plot rl_master runtime MCAP logs")
    parser.add_argument("--mcap", required=True, help="Path to runtime .mcap log")
    parser.add_argument("--joint-indices", default="", help="Comma-separated joint indices to plot")
    parser.add_argument("--motor-indices", default="", help="Comma-separated motor indices to plot")
    parser.add_argument("--output-dir", default="", help="Optional directory for PNG outputs")
    parser.add_argument("--no-show", action="store_true", help="Do not open matplotlib windows")
    parser.add_argument("--include-non-running", action="store_true", help="Plot all ticks, not only RUNNING")
    parser.add_argument("--skip-torque", action="store_true", help="Skip joint torque plots")
    parser.add_argument("--skip-motor", action="store_true", help="Skip motor position/torque plots")
    args = parser.parse_args()

    mcap_path = Path(args.mcap)
    if not mcap_path.exists():
        raise FileNotFoundError(mcap_path)

    config_messages = load_runtime_messages(mcap_path, topic="runtime/config")
    if not config_messages:
        raise RuntimeError("No runtime/config message found in MCAP")
    config_payload = config_messages[0].get("data")
    if not isinstance(config_payload, dict):
        raise RuntimeError("runtime/config payload is not a JSON object")

    events = [
        message.get("data")
        for message in load_runtime_messages(mcap_path, topic="runtime/event")
        if isinstance(message.get("data"), dict)
    ]
    ticks = [
        message.get("data")
        for message in load_runtime_messages(mcap_path, topic="runtime/tick")
        if isinstance(message.get("data"), dict)
    ]
    selected_ticks = select_ticks(ticks, running_only=not args.include_non_running)
    if not selected_ticks:
        raise RuntimeError("No matching ticks found for the selected filter")

    manifest_path = resolve_manifest_path(config_payload)
    term_slices: Dict[str, Tuple[int, int]] = {}
    if manifest_path is not None:
        for term in load_manifest_terms(manifest_path):
            term_slices[term["name"]] = (int(term["start"]), int(term["end"]))

    joint_names = config_payload.get("runtime_joint_order") or config_payload.get("installed_joint_names") or []
    if not isinstance(joint_names, list):
        joint_names = []
    joint_names = [str(name) for name in joint_names]
    motor_names = config_payload.get("installed_motor_order") or []
    if not isinstance(motor_names, list):
        motor_names = []
    motor_names = [str(name) for name in motor_names]

    joint_indices = parse_indices(args.joint_indices)
    if not joint_indices:
        default_joint_count = min(12, len(joint_names) if joint_names else 12)
        joint_indices = list(range(default_joint_count))
    motor_indices = parse_indices(args.motor_indices)
    if not motor_indices:
        default_motor_count = min(12, len(motor_names) if motor_names else 12)
        motor_indices = list(range(default_motor_count))

    print(f"mcap_path: {mcap_path}")
    print(
        f"active_mode_id: {config_payload.get('active_mode_id')}  "
        f"active_config_section: {config_payload.get('active_config_section')}  "
        f"policy: {config_payload.get('active_policy_name')}"
    )
    print(f"selected_tick_count: {len(selected_ticks)}")
    print(f"manifest_path: {manifest_path if manifest_path is not None else '<none>'}")
    print(f"joint_indices: {joint_indices}")
    print(f"motor_indices: {motor_indices}")

    print_event_timeline(events)
    print_command_changes(selected_ticks)

    output_dir = make_output_dir(args.output_dir)
    show = not args.no_show

    plot_motion_overview(selected_ticks, term_slices, output_dir, show)
    plot_joint_tracking(selected_ticks, joint_names, joint_indices, output_dir, show)
    if not args.skip_torque:
        plot_torque_tracking(selected_ticks, joint_names, joint_indices, output_dir, show)
    if not args.skip_motor:
        plot_motor_tracking(selected_ticks, motor_names, motor_indices, output_dir, show)
        plot_motor_torque_tracking(selected_ticks, motor_names, motor_indices, output_dir, show)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
