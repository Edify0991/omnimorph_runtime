#!/usr/bin/env python3
"""Reconstruct and visualize COM trajectories from rl_master runtime MCAP logs.

The current runtime logs usually contain joint-space state but may not contain a
world base pose. In that case this tool computes COM in a fixed free-flyer
frame, which is still useful for checking whole-body posture and COM projection
relative to the robot base.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time
import types
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
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
            raise RuntimeError("lz4 Python module is unavailable; only zstd/none logs can be read")

        frame_module.decompress = _unsupported_lz4
        lz4_module.frame = frame_module
        sys.modules["lz4"] = lz4_module
        sys.modules["lz4.frame"] = frame_module


install_compression_fallbacks()
REPO_ROOT = Path(__file__).resolve().parents[1]
RL_MASTER_ROOT = REPO_ROOT / "src/omnimorph_rl_controller/rl_master"
sys.path.insert(0, str(RL_MASTER_ROOT / "tools/analysis"))

from runtime_log_utils import load_runtime_messages  # noqa: E402


DEFAULT_MCAP = RL_MASTER_ROOT / "data/runtime_logs/Jun12_09-46-01_beyondmimic_jc01_dance_wo_state_estimation.mcap"
DEFAULT_ROOT_CONFIG = RL_MASTER_ROOT / "config/rl_cfg_jc01.yaml"


def import_pinocchio():
    try:
        import pinocchio as pin  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Python Pinocchio is required for COM reconstruction. "
            "Install the Python bindings in this environment, then rerun this tool."
        ) from exc
    return pin


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root is not a mapping: {path}")
    return data


def expand_vars(value: str, variables: Dict[str, str]) -> str:
    out = value
    for _ in range(8):
        before = out
        for key, replacement in variables.items():
            out = out.replace("${" + key + "}", replacement)
        out = os.path.expandvars(out)
        if out == before:
            break
    return out


def repo_relative(path_text: str, anchor: Path) -> Path:
    path = Path(path_text)
    return path if path.is_absolute() else (anchor / path).resolve()


def runtime_config_payload(mcap_path: Path) -> Dict[str, Any]:
    messages = load_runtime_messages(mcap_path, "runtime/config")
    if not messages:
        return {}
    data = messages[0].get("data")
    return data if isinstance(data, dict) else {}


def resolve_profile_section(args: argparse.Namespace, mcap_path: Path) -> str:
    if args.profile:
        return args.profile
    runtime_config = runtime_config_payload(mcap_path)
    section = str(runtime_config.get("config_section", "")).strip()
    if section:
        return section
    raise RuntimeError("Could not infer profile section; pass --profile explicitly")


def load_profile(root_config_path: Path, section: str) -> Tuple[Dict[str, Any], Path, Dict[str, Any]]:
    root = load_yaml(root_config_path)
    config_files = root.get("config_files")
    if not isinstance(config_files, dict) or section not in config_files:
        raise RuntimeError(f"Profile section '{section}' is not listed in {root_config_path}")

    profile_path = repo_relative(str(config_files[section]), root_config_path.parent)
    profile_doc = load_yaml(profile_path)
    profile = profile_doc.get(section)
    if not isinstance(profile, dict):
        raise RuntimeError(f"Profile file {profile_path} does not contain section '{section}'")
    return profile, profile_path, root


def path_variables(root_config: Dict[str, Any]) -> Dict[str, str]:
    variables: Dict[str, str] = {key: value for key, value in os.environ.items()}
    raw = root_config.get("path_variables")
    if isinstance(raw, dict):
        for key, value in raw.items():
            variables[str(key)] = expand_vars(str(value), variables)
    return variables


def candidate_urdf_paths(
    args: argparse.Namespace,
    profile: Dict[str, Any],
    root_config: Dict[str, Any],
    root_config_path: Path,
) -> List[Path]:
    variables = path_variables(root_config)
    candidates: List[Path] = []
    if args.urdf:
        candidates.append(Path(args.urdf).expanduser())

    profile_urdf = str(profile.get("pinocchio_urdf_path", "")).strip()
    if profile_urdf:
        candidates.append(repo_relative(expand_vars(profile_urdf, variables), root_config_path.parent))

    robot_assets_dir = variables.get("ROBOT_ASSETS_DIR") or variables.get("robot_assets_dir")
    if robot_assets_dir:
        candidates.append(Path(robot_assets_dir).expanduser() / "JC01-URDF.urdf")

    candidates.extend(
        [
            Path("/home/edify/Code/jc01-model/JC01-URDF.urdf"),
            Path("/home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/JC01-URDF.urdf"),
        ]
    )

    deduped: List[Path] = []
    seen = set()
    for candidate in candidates:
        resolved = candidate.expanduser()
        key = str(resolved)
        if key not in seen:
            deduped.append(resolved)
            seen.add(key)
    return deduped


def resolve_urdf_path(
    args: argparse.Namespace,
    profile: Dict[str, Any],
    root_config: Dict[str, Any],
    root_config_path: Path,
) -> Path:
    candidates = candidate_urdf_paths(args, profile, root_config, root_config_path)
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    rendered = "\n  ".join(str(path) for path in candidates)
    raise RuntimeError(f"Could not find Pinocchio URDF. Checked:\n  {rendered}\nPass --urdf /path/to/JC01-URDF.urdf")


def resolve_joint_order(profile: Dict[str, Any], root_config: Dict[str, Any], args: argparse.Namespace) -> List[str]:
    if args.joint_order:
        return [token.strip() for token in args.joint_order.split(",") if token.strip()]
    order = root_config.get("robot_global_joint_order")
    if isinstance(order, list) and order:
        return [str(name) for name in order]
    order = profile.get("reference_joint_order") or profile.get("obs_joint_order") or profile.get("action_joint_order")
    if isinstance(order, list) and order:
        return [str(name) for name in order]
    raise RuntimeError("Could not resolve joint order from config; pass --joint-order name1,name2,...")


def parse_vec(text: str, expected_len: int, name: str) -> List[float]:
    values = [float(token.strip()) for token in text.split(",") if token.strip()]
    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(f"{name} must contain {expected_len} comma-separated numbers")
    return values


def finite_vector(value: Any, minimum_len: int) -> Optional[List[float]]:
    if not isinstance(value, list) or len(value) < minimum_len:
        return None
    out: List[float] = []
    for item in value:
        try:
            number = float(item)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        out.append(number)
    return out


def normalized_xyzw(quat: Sequence[float]) -> List[float]:
    values = np.asarray(quat[:4], dtype=float)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm < 1.0e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return (values / norm).tolist()


def select_ticks(messages: Sequence[Dict[str, Any]], running_only: bool) -> List[Dict[str, Any]]:
    ticks: List[Dict[str, Any]] = []
    for message in messages:
        data = message.get("data")
        if not isinstance(data, dict):
            continue
        if running_only and int(data.get("deploy_state", 0)) != 3:
            continue
        ticks.append(data)
    return ticks


def apply_stride_and_limit(
    ticks: Sequence[Dict[str, Any]],
    stride: int,
    max_samples: Optional[int],
) -> List[Dict[str, Any]]:
    sampled = list(ticks[:: max(1, stride)])
    if max_samples is None or max_samples <= 0 or len(sampled) <= max_samples:
        return sampled
    indices = np.linspace(0, len(sampled) - 1, max_samples, dtype=int)
    return [sampled[int(index)] for index in indices]


def pin_property(value: Any) -> Any:
    return value() if callable(value) else value


def build_joint_index_map(model: Any, joint_order: Sequence[str]) -> List[int]:
    q_indices: List[int] = []
    missing: List[str] = []
    non_scalar: List[str] = []
    for joint_name in joint_order:
        if not model.existJointName(joint_name):
            missing.append(joint_name)
            continue
        joint_id = model.getJointId(joint_name)
        joint_model = model.joints[joint_id]
        nq = int(pin_property(getattr(joint_model, "nq")))
        if nq != 1:
            non_scalar.append(joint_name)
            continue
        q_indices.append(int(pin_property(getattr(joint_model, "idx_q"))))
    if missing:
        raise RuntimeError("URDF is missing joints from runtime order: " + ", ".join(missing))
    if non_scalar:
        raise RuntimeError("Expected 1-DOF joints, got non-scalar joints: " + ", ".join(non_scalar))
    return q_indices


def set_base_configuration(
    q: np.ndarray,
    tick: Dict[str, Any],
    args: argparse.Namespace,
    fallback_pos: Sequence[float],
    fallback_quat_xyzw: Sequence[float],
) -> Tuple[List[float], List[float], bool]:
    pos = finite_vector(tick.get(args.base_pos_field), 3) if args.base_pos_field else None
    quat = finite_vector(tick.get(args.base_quat_field), 4) if args.base_quat_field else None
    used_logged_base = pos is not None and quat is not None
    if pos is None:
        pos = list(fallback_pos)
    if quat is None:
        quat = list(fallback_quat_xyzw)

    q[0:3] = pos[:3]
    q[3:7] = normalized_xyzw(quat)
    return pos[:3], quat[:4], used_logged_base


def reconstruct_com(
    pin: Any,
    model: Any,
    ticks: Sequence[Dict[str, Any]],
    joint_q_indices: Sequence[int],
    args: argparse.Namespace,
) -> Tuple[List[Dict[str, float]], List[np.ndarray], bool]:
    data = model.createData()
    neutral = pin.neutral(model)
    fallback_pos = parse_vec(args.base_pos, 3, "--base-pos")
    fallback_quat = parse_vec(args.base_quat_xyzw, 4, "--base-quat-xyzw")
    rows: List[Dict[str, float]] = []
    q_traj: List[np.ndarray] = []
    any_logged_base = False

    first_time: Optional[float] = None
    for tick in ticks:
        joint_values = finite_vector(tick.get(args.joint_field), len(joint_q_indices))
        if joint_values is None:
            continue

        q = neutral.copy()
        _base_pos, _base_quat, used_logged_base = set_base_configuration(q, tick, args, fallback_pos, fallback_quat)
        any_logged_base = any_logged_base or used_logged_base
        for src_idx, q_idx in enumerate(joint_q_indices):
            q[q_idx] = joint_values[src_idx]

        com = np.asarray(pin.centerOfMass(model, data, q, False)).reshape(3)
        monotonic_time = float(tick.get("monotonic_time_sec", 0.0))
        if first_time is None:
            first_time = monotonic_time
        rows.append(
            {
                "time_sec": monotonic_time,
                "time_from_start_sec": monotonic_time - first_time,
                "com_x": float(com[0]),
                "com_y": float(com[1]),
                "com_z": float(com[2]),
                "projection_x": float(com[0]),
                "projection_y": float(com[1]),
                "deploy_state": float(tick.get("deploy_state", 0)),
                "frame_index": float(tick.get("frame_index", len(rows))),
            }
        )
        q_traj.append(q)
    return rows, q_traj, any_logged_base


def write_csv(rows: Sequence[Dict[str, float]], output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "time_sec",
        "time_from_start_sec",
        "com_x",
        "com_y",
        "com_z",
        "projection_x",
        "projection_y",
        "deploy_state",
        "frame_index",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_plotly_html(rows: Sequence[Dict[str, float]], output_path: Path) -> bool:
    try:
        import plotly.graph_objects as go  # type: ignore
        from plotly.subplots import make_subplots  # type: ignore
    except Exception:
        return False

    t = [row["time_from_start_sec"] for row in rows]
    x = [row["com_x"] for row in rows]
    y = [row["com_y"] for row in rows]
    z = [row["com_z"] for row in rows]

    fig = make_subplots(
        rows=1,
        cols=2,
        specs=[[{"type": "scatter3d"}, {"type": "xy"}]],
        subplot_titles=("COM trajectory", "Ground projection"),
    )
    fig.add_trace(
        go.Scatter3d(
            x=x,
            y=y,
            z=z,
            mode="lines",
            line=dict(color=t, colorscale="Viridis", width=5),
            name="COM",
        ),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=x,
            y=y,
            mode="lines",
            line=dict(color="#d65f5f", width=2),
            name="COM projection",
        ),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(
            x=[x[0]],
            y=[y[0]],
            mode="markers",
            marker=dict(color="#2c7fb8", size=8),
            name="start",
        ),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(
            x=[x[-1]],
            y=[y[-1]],
            mode="markers",
            marker=dict(color="#d95f02", size=8),
            name="end",
        ),
        row=1,
        col=2,
    )
    fig.update_xaxes(title_text="x (m)", scaleanchor="y", scaleratio=1, row=1, col=2)
    fig.update_yaxes(title_text="y (m)", row=1, col=2)
    fig.update_layout(
        title="Runtime COM Reconstruction",
        scene=dict(xaxis_title="x (m)", yaxis_title="y (m)", zaxis_title="z (m)", aspectmode="data"),
        margin=dict(l=20, r=20, t=60, b=20),
        legend=dict(orientation="h"),
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(output_path), include_plotlyjs="cdn")
    return True


def write_matplotlib_png(rows: Sequence[Dict[str, float]], output_path: Path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        return False

    t = [row["time_from_start_sec"] for row in rows]
    x = [row["com_x"] for row in rows]
    y = [row["com_y"] for row in rows]
    z = [row["com_z"] for row in rows]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(13, 5))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    ax3d.plot(x, y, z, linewidth=1.5)
    ax3d.scatter([x[0]], [y[0]], [z[0]], label="start", s=30)
    ax3d.scatter([x[-1]], [y[-1]], [z[-1]], label="end", s=30)
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("COM trajectory")
    ax3d.legend()

    ax2d = fig.add_subplot(1, 2, 2)
    ax2d.plot(x, y, linewidth=1.4)
    ax2d.scatter([x[0]], [y[0]], label="start", s=30)
    ax2d.scatter([x[-1]], [y[-1]], label="end", s=30)
    ax2d.set_xlabel("x (m)")
    ax2d.set_ylabel("y (m)")
    ax2d.set_title("Ground projection")
    ax2d.axis("equal")
    ax2d.grid(alpha=0.3)
    ax2d.legend()

    fig.suptitle(f"Runtime COM Reconstruction ({len(rows)} samples, {t[-1]:.2f}s)")
    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return True


def run_meshcat(
    pin: Any,
    model: Any,
    urdf_path: Path,
    q_traj: Sequence[np.ndarray],
    rows: Sequence[Dict[str, float]],
    args: argparse.Namespace,
) -> None:
    try:
        from pinocchio.visualize import MeshcatVisualizer  # type: ignore
        import meshcat.geometry as g  # type: ignore
        import meshcat.transformations as tf  # type: ignore
    except Exception as exc:
        raise RuntimeError("Meshcat visualization requires pinocchio.visualize and meshcat Python packages") from exc

    try:
        geom_model = pin.buildGeomFromUrdf(model, str(urdf_path), pin.GeometryType.VISUAL)
    except Exception:
        geom_model = pin.GeometryModel()
    vis = MeshcatVisualizer(model, geom_model, pin.GeometryModel())
    vis.initViewer(open=True)
    vis.loadViewerModel(rootNodeName="jc01_com_replay")

    viewer = vis.viewer
    viewer["com"].set_object(g.Sphere(0.025), g.MeshLambertMaterial(color=0xE4572E))
    viewer["com_projection"].set_object(g.Sphere(0.018), g.MeshLambertMaterial(color=0x2E86AB))

    stride = max(1, args.meshcat_stride)
    for q, row in zip(q_traj[::stride], rows[::stride]):
        vis.display(q)
        viewer["com"].set_transform(tf.translation_matrix([row["com_x"], row["com_y"], row["com_z"]]))
        viewer["com_projection"].set_transform(tf.translation_matrix([row["projection_x"], row["projection_y"], 0.0]))
        time.sleep(max(0.0, args.meshcat_dt))


def summarize(rows: Sequence[Dict[str, float]], any_logged_base: bool) -> Dict[str, float]:
    xs = np.asarray([row["com_x"] for row in rows])
    ys = np.asarray([row["com_y"] for row in rows])
    zs = np.asarray([row["com_z"] for row in rows])
    return {
        "samples": float(len(rows)),
        "duration_sec": float(rows[-1]["time_from_start_sec"] if rows else 0.0),
        "used_logged_base_pose": float(1 if any_logged_base else 0),
        "com_x_min": float(xs.min()),
        "com_x_max": float(xs.max()),
        "com_y_min": float(ys.min()),
        "com_y_max": float(ys.max()),
        "com_z_min": float(zs.min()),
        "com_z_max": float(zs.max()),
    }


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, default=DEFAULT_MCAP, help="Runtime MCAP log path")
    parser.add_argument("--config", type=Path, default=DEFAULT_ROOT_CONFIG, help="Root rl_cfg YAML")
    parser.add_argument("--profile", default="", help="Config section/profile name; inferred from runtime/config when omitted")
    parser.add_argument("--urdf", default="", help="Pinocchio URDF path; defaults to profile pinocchio_urdf_path")
    parser.add_argument("--joint-order", default="", help="Comma-separated joint order if config inference is not desired")
    parser.add_argument(
        "--joint-field",
        default="joint_q",
        choices=["joint_q", "joint_state_q", "joint_target_q", "joint_cmd_q", "motor_state_q"],
        help="Tick vector used as reconstructed joint positions",
    )
    parser.add_argument("--base-pos-field", default="base_pos_w", help="Tick field for base position, if present")
    parser.add_argument("--base-quat-field", default="base_quat", help="Tick field for base quaternion xyzw, if present")
    parser.add_argument("--base-pos", default="0,0,0", help="Fallback fixed base position xyz")
    parser.add_argument("--base-quat-xyzw", default="0,0,0,1", help="Fallback fixed base quaternion xyzw")
    parser.add_argument("--running-only", action="store_true", help="Only use ticks with deploy_state == 3")
    parser.add_argument("--stride", type=int, default=1, help="Keep every Nth selected tick")
    parser.add_argument("--max-samples", type=int, default=0, help="Downsample to at most this many samples after stride")
    parser.add_argument("--output-dir", type=Path, default=Path("analysis_outputs/com_trajectory"), help="Output directory")
    parser.add_argument("--no-html", action="store_true", help="Skip Plotly HTML output")
    parser.add_argument("--no-png", action="store_true", help="Skip Matplotlib PNG output")
    parser.add_argument("--meshcat", action="store_true", help="Open Meshcat and replay the robot with COM markers")
    parser.add_argument("--meshcat-stride", type=int, default=10, help="Display every Nth computed sample in Meshcat")
    parser.add_argument("--meshcat-dt", type=float, default=0.02, help="Delay between Meshcat frames")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    pin = import_pinocchio()

    mcap_path = args.mcap.resolve()
    root_config_path = args.config.resolve()
    if not mcap_path.exists():
        raise RuntimeError(f"MCAP path does not exist: {mcap_path}")
    if not root_config_path.exists():
        raise RuntimeError(f"Config path does not exist: {root_config_path}")

    profile_section = resolve_profile_section(args, mcap_path)
    profile, profile_path, root_config = load_profile(root_config_path, profile_section)
    urdf_path = resolve_urdf_path(args, profile, root_config, root_config_path)
    joint_order = resolve_joint_order(profile, root_config, args)

    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    joint_q_indices = build_joint_index_map(model, joint_order)

    messages = load_runtime_messages(mcap_path, "runtime/tick")
    ticks = select_ticks(messages, args.running_only)
    ticks = apply_stride_and_limit(ticks, args.stride, args.max_samples if args.max_samples > 0 else None)
    if not ticks:
        raise RuntimeError("No runtime/tick samples selected")

    rows, q_traj, any_logged_base = reconstruct_com(pin, model, ticks, joint_q_indices, args)
    if not rows:
        raise RuntimeError(f"No rows reconstructed; selected ticks lack usable '{args.joint_field}' vectors")

    output_dir = args.output_dir / mcap_path.stem
    csv_path = output_dir / "com_trajectory.csv"
    html_path = output_dir / "com_trajectory.html"
    png_path = output_dir / "com_trajectory.png"
    write_csv(rows, csv_path)

    wrote_html = False
    wrote_png = False
    if not args.no_html:
        wrote_html = write_plotly_html(rows, html_path)
    if not args.no_png:
        wrote_png = write_matplotlib_png(rows, png_path)

    stats = summarize(rows, any_logged_base)
    print(f"profile: {profile_section}")
    print(f"profile_file: {profile_path}")
    print(f"urdf: {urdf_path}")
    print(f"joint_field: {args.joint_field}")
    print(f"samples: {int(stats['samples'])}, duration_sec: {stats['duration_sec']:.3f}")
    if any_logged_base:
        print("base_pose: used logged base_pos/base_quat where available")
    else:
        print(f"base_pose: fixed fallback pos={args.base_pos} quat_xyzw={args.base_quat_xyzw}")
    print(
        "com_bounds: "
        f"x=[{stats['com_x_min']:.4f}, {stats['com_x_max']:.4f}] "
        f"y=[{stats['com_y_min']:.4f}, {stats['com_y_max']:.4f}] "
        f"z=[{stats['com_z_min']:.4f}, {stats['com_z_max']:.4f}]"
    )
    print(f"csv: {csv_path}")
    if wrote_html:
        print(f"html: {html_path}")
    elif not args.no_html:
        print("html: skipped because plotly is unavailable")
    if wrote_png:
        print(f"png: {png_path}")
    elif not args.no_png:
        print("png: skipped because matplotlib is unavailable")

    if args.meshcat:
        run_meshcat(pin, model, urdf_path, q_traj, rows, args)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
