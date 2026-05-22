#!/usr/bin/env python3
"""Compare AMP play NPZ policy IO against deploy ONNX and runtime CSV logs."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
import yaml

try:
    import onnxruntime as ort
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit("onnxruntime is required. Install with: pip install onnxruntime") from exc


DEFAULT_POLICY_IO = Path("/home/edify/Code/AMP_mjlab/logs/sim2sim/jingchu01_policy_io.npz")
DEFAULT_PROFILE = (
    Path(__file__).resolve().parents[2] / "config" / "profiles" / "amp_mjlab_jc01_full_body.yaml"
)
DEFAULT_SECTION = "amp_mjlab_jc01_full_body"
DEFAULT_ONNX = (
    Path(__file__).resolve().parents[2] / "policies" / "Jingchu01-AMP-Flat_model_5200.onnx"
)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a map: {path}")
    return data


def profile_section(profile_path: Path, section: str) -> Dict[str, Any]:
    root = load_yaml(profile_path)
    if section not in root:
        raise KeyError(f"profile file {profile_path} does not contain section '{section}'")
    cfg = root[section]
    if not isinstance(cfg, dict):
        raise ValueError(f"profile section must be a map: {section}")
    return cfg


def make_session(onnx_path: Path, cfg: Mapping[str, Any]) -> ort.InferenceSession:
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = max(1, int(cfg.get("onnx_intra_threads", 1)))
    opts.inter_op_num_threads = max(1, int(cfg.get("onnx_inter_threads", 1)))
    return ort.InferenceSession(str(onnx_path), sess_options=opts, providers=["CPUExecutionProvider"])


def run_model(session: ort.InferenceSession, obs: np.ndarray, obs_input_name: str, action_output_name: str) -> np.ndarray:
    outputs = session.run([action_output_name], {obs_input_name: obs.astype(np.float32)})
    return np.asarray(outputs[0], dtype=np.float32)


def replay_metrics(pred: np.ndarray, ref: np.ndarray) -> Dict[str, float]:
    diff = pred - ref
    abs_diff = np.abs(diff)
    return {
        "mae": float(np.mean(abs_diff)),
        "rmse": math.sqrt(float(np.mean(diff * diff))),
        "max_abs": float(np.max(abs_diff)),
        "p95": float(np.percentile(abs_diff, 95)),
    }


def print_replay_metrics(label: str, pred: np.ndarray, ref: np.ndarray) -> None:
    metrics = replay_metrics(pred, ref)
    print(f"\n{label}:")
    print(f"  mae={metrics['mae']:.9g} rmse={metrics['rmse']:.9g} max_abs={metrics['max_abs']:.9g} p95={metrics['p95']:.9g}")
    print(
        "  pred_stats="
        f"(min={float(pred.min()):.6g}, max={float(pred.max()):.6g}, mean={float(pred.mean()):.6g}, std={float(pred.std()):.6g})"
    )


def load_runtime_csv(path: Path, obs_dim: int, action_dim: int) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    obs_cols = [f"observation__{i}" for i in range(obs_dim)]
    act_cols = [f"policy_action__{i}" for i in range(action_dim)]
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            if not all(name in row for name in obs_cols):
                continue
            try:
                obs = np.asarray([float(row[name]) for name in obs_cols], dtype=np.float32)
            except ValueError:
                continue
            action_values: List[float] = []
            has_action = True
            for name in act_cols:
                text = row.get(name, "")
                if text == "":
                    has_action = False
                    break
                action_values.append(float(text))
            rows.append(
                {
                    "row_index": idx,
                    "frame_index": int(float(row.get("frame_index", idx) or idx)),
                    "deploy_state": str(row.get("deploy_state", "")),
                    "obs": obs,
                    "action": np.asarray(action_values, dtype=np.float32) if has_action else None,
                }
            )
    return rows


def first_running_row(rows: Sequence[Mapping[str, Any]]) -> Dict[str, Any]:
    for row in rows:
        action = row.get("action")
        if row.get("deploy_state") != "3" or action is None:
            continue
        if np.any(np.abs(np.asarray(action, dtype=np.float32)) > 1.0e-9):
            return dict(row)
    raise RuntimeError("No runtime row with deploy_state==3 and non-zero policy_action found")


def candidate_stacks(obs: np.ndarray, stack_n: int) -> Dict[str, np.ndarray]:
    zeros = np.zeros_like(obs)
    repeated = np.concatenate([obs] * stack_n, axis=0)
    out = {"repeat_all": repeated}
    if stack_n >= 2:
        zero_then_obs: List[np.ndarray] = [zeros] * (stack_n - 1) + [obs]
        obs_then_zero: List[np.ndarray] = [obs] + [zeros] * (stack_n - 1)
        out["zero_prefix"] = np.concatenate(zero_then_obs, axis=0)
        out["zero_suffix"] = np.concatenate(obs_then_zero, axis=0)
    if stack_n == 4:
        out["zero2_obs2"] = np.concatenate([zeros, zeros, obs, obs], axis=0)
        out["obs2_zero2"] = np.concatenate([obs, obs, zeros, zeros], axis=0)
    return out


def describe_term(labels: Sequence[str], values: np.ndarray) -> str:
    return (
        f"mean={float(values.mean()):.6g} std={float(values.std()):.6g} "
        f"min={float(values.min()):.6g} max={float(values.max()):.6g}"
    )


def obs_term_slices(cfg: Mapping[str, Any]) -> List[Tuple[str, int, int]]:
    joint_count = int(cfg["motor_N"])
    return [
        ("base_ang_vel", 0, 3),
        ("projected_gravity", 3, 6),
        ("command", 6, 9),
        ("joint_pos", 9, 9 + joint_count),
        ("joint_vel", 9 + joint_count, 9 + joint_count * 2),
        ("last_action", 9 + joint_count * 2, 9 + joint_count * 3),
    ]


def obs_labels(cfg: Mapping[str, Any]) -> List[str]:
    joints = [str(x) for x in cfg.get("obs_joint_order", [])]
    labels = [f"base_ang_vel[{i}]" for i in range(3)]
    labels += [f"projected_gravity[{i}]" for i in range(3)]
    labels += [f"command[{i}]" for i in range(3)]
    labels += [f"joint_pos[{name}]" for name in joints]
    labels += [f"joint_vel[{name}]" for name in joints]
    labels += [f"last_action[{name}]" for name in joints]
    return labels


def compare_runtime(
    runtime_rows: Sequence[Mapping[str, Any]],
    train_raw: np.ndarray,
    train_actions: np.ndarray,
    cfg: Mapping[str, Any],
    session: ort.InferenceSession,
    obs_input_name: str,
    action_output_name: str,
) -> None:
    labels = obs_labels(cfg)
    term_slices = obs_term_slices(cfg)
    stack_n = int(cfg["obs_stack_N"])
    obs_dim = int(cfg["obs_dim"])

    runtime_obs = np.stack([np.asarray(row["obs"], dtype=np.float32) for row in runtime_rows], axis=0)
    runtime_actions = np.stack(
        [np.asarray(row["action"], dtype=np.float32) for row in runtime_rows if row.get("action") is not None],
        axis=0,
    )
    train_frames = train_raw.reshape(-1, obs_dim)
    first_row = first_running_row(runtime_rows)
    first_obs = np.asarray(first_row["obs"], dtype=np.float32)
    first_action = np.asarray(first_row["action"], dtype=np.float32)

    print("\nRuntime vs play distribution:")
    for name, begin, end in term_slices:
        train_term = train_frames[:, begin:end].reshape(-1)
        runtime_term = runtime_obs[:, begin:end].reshape(-1)
        outside = np.mean((runtime_term < train_term.min()) | (runtime_term > train_term.max()))
        print(f"  {name}:")
        print(f"    train   {describe_term(labels, train_term)}")
        print(f"    runtime {describe_term(labels, runtime_term)}")
        print(f"    runtime_outside_train_range={float(outside):.6g}")

    print("\nAction distribution:")
    print(f"  train   {describe_term([], train_actions.reshape(-1))}")
    print(f"  runtime {describe_term([], runtime_actions.reshape(-1))}")

    print("\nFirst RUNNING row:")
    print(
        f"  row_index={first_row['row_index']} frame_index={first_row['frame_index']} "
        f"deploy_state={first_row['deploy_state']}"
    )
    print(f"  obs_stats={describe_term(labels, first_obs)}")
    print(f"  action_stats={describe_term([], first_action)}")

    print("\nFirst RUNNING history-stack check:")
    for name, stacked in candidate_stacks(first_obs, stack_n).items():
        pred = run_model(session, stacked[None, :], obs_input_name, action_output_name)[0]
        metrics = replay_metrics(pred[None, :], first_action[None, :])
        print(f"  {name}: mae={metrics['mae']:.9g} max_abs={metrics['max_abs']:.9g}")

    repeated_stack = np.concatenate([first_obs] * stack_n, axis=0)
    repeated_pred = run_model(session, repeated_stack[None, :], obs_input_name, action_output_name)[0]
    print("\nFirst RUNNING repeated-stack replay:")
    print_replay_metrics("  deploy_logged_action vs onnx(repeat_all)", repeated_pred[None, :], first_action[None, :])

    frame_dist = np.linalg.norm(train_frames - first_obs[None, :], axis=1)
    nearest_frame_idx = int(np.argmin(frame_dist))
    nearest_frame = train_frames[nearest_frame_idx]
    frame_diff = np.abs(first_obs - nearest_frame)
    print(
        "\nNearest play single-frame obs:"
        f" sample={nearest_frame_idx // stack_n} chunk={nearest_frame_idx % stack_n}"
        f" l2={float(frame_dist[nearest_frame_idx]):.6g}"
        f" linf={float(frame_diff.max()):.6g}"
    )
    top = np.argsort(-frame_diff)[:10]
    for idx in top:
        label = labels[idx] if idx < len(labels) else str(idx)
        print(
            f"  {label}: runtime={float(first_obs[idx]): .6g} "
            f"play={float(nearest_frame[idx]): .6g} absdiff={float(frame_diff[idx]):.6g}"
        )

    train_stacked = train_raw.reshape(-1, obs_dim * stack_n)
    stack_dist = np.linalg.norm(train_stacked - repeated_stack[None, :], axis=1)
    nearest_stack_idx = int(np.argmin(stack_dist))
    nearest_stack_action = train_actions[nearest_stack_idx]
    action_diff = np.abs(first_action - nearest_stack_action)
    print(
        "\nNearest play stacked obs to runtime repeat-all:"
        f" sample={nearest_stack_idx} step={nearest_stack_idx}"
        f" l2={float(stack_dist[nearest_stack_idx]):.6g}"
        f" action_l2={float(np.linalg.norm(first_action - nearest_stack_action)):.6g}"
        f" action_linf={float(action_diff.max()):.6g}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy-io-npz", type=Path, default=DEFAULT_POLICY_IO)
    parser.add_argument("--onnx", type=Path, default=DEFAULT_ONNX)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--section", default=DEFAULT_SECTION)
    parser.add_argument("--runtime-csv", type=Path, default=None)
    args = parser.parse_args()

    npz_path = args.policy_io_npz.expanduser().resolve()
    onnx_path = args.onnx.expanduser().resolve()
    profile_path = args.profile.expanduser().resolve()
    runtime_csv = args.runtime_csv.expanduser().resolve() if args.runtime_csv else None

    if not npz_path.exists():
        raise FileNotFoundError(npz_path)
    if not onnx_path.exists():
        raise FileNotFoundError(onnx_path)
    if not profile_path.exists():
        raise FileNotFoundError(profile_path)

    cfg = profile_section(profile_path, args.section)
    policy_io = cfg.get("policy_io", {}) if isinstance(cfg.get("policy_io"), dict) else {}
    obs_input_name = str(policy_io.get("obs_input_name", "obs"))
    action_output_name = str(policy_io.get("action_output_name", "actions"))

    session = make_session(onnx_path, cfg)
    data = np.load(npz_path)
    raw = np.asarray(data["actor_obs_raw"], dtype=np.float32).reshape(-1, int(cfg["obs_dim"]) * int(cfg["obs_stack_N"]))
    norm = np.asarray(data["actor_obs_normalized"], dtype=np.float32).reshape(
        -1, int(cfg["obs_dim"]) * int(cfg["obs_stack_N"])
    )
    actions = np.asarray(data["actions"], dtype=np.float32).reshape(-1, int(cfg["action_dim"]))

    print(f"NPZ: {npz_path}")
    print(f"ONNX: {onnx_path}")
    print(f"profile: {profile_path} [{args.section}]")
    print(f"actor_obs_raw shape={raw.shape}")
    print(f"actor_obs_normalized shape={norm.shape}")
    print(f"actions shape={actions.shape}")

    pred_raw = run_model(session, raw, obs_input_name, action_output_name)
    pred_norm = run_model(session, norm, obs_input_name, action_output_name)
    print_replay_metrics("Replay with actor_obs_raw", pred_raw, actions)
    print_replay_metrics("Replay with actor_obs_normalized", pred_norm, actions)

    if runtime_csv is not None:
        runtime_rows = load_runtime_csv(runtime_csv, int(cfg["obs_dim"]), int(cfg["action_dim"]))
        if not runtime_rows:
            raise RuntimeError(f"No usable rows found in runtime csv: {runtime_csv}")
        print(f"\nruntime_csv: {runtime_csv}")
        compare_runtime(
            runtime_rows,
            raw.reshape(-1, int(cfg["obs_stack_N"]), int(cfg["obs_dim"])),
            actions,
            cfg,
            session,
            obs_input_name,
            action_output_name,
        )

    return 0


if __name__ == "__main__":
    sys.exit(main())
