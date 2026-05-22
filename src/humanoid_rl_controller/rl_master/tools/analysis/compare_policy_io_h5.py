#!/usr/bin/env python3
"""Compare a recorded BeyondMimic policy_io HDF5 file with deploy ONNX inference.

The H5 file produced by BeyondMimic play stores the policy input observation and
the raw policy action for each frame. This tool replays those observations through
the ONNX model configured by this deploy repository and reports numerical drift
plus the joint-order contracts that matter when the deploy pipeline rebuilds the
next observation from ONNX extra outputs.
"""

from __future__ import annotations

import argparse
import ast
import json
import math
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import h5py
import numpy as np
import yaml

try:
    import onnxruntime as ort
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit("onnxruntime is required. Install with: pip install onnxruntime") from exc


DEFAULT_H5 = Path("/home/edify/Code/beyondmimic/outputs/policy_io/jc01_walk2_policy_io.h5")
DEFAULT_CFG = Path(__file__).resolve().parents[2] / "config" / "rl_cfg_jc01.yaml"
DEFAULT_SECTION = "beyondmimic_leg12_strict_walk2"


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a map: {path}")
    return data


def resolve_path(raw: str, base: Path) -> Path:
    p = Path(raw).expanduser()
    if p.is_absolute():
        return p
    return (base / p).resolve()


def profile_for_section(root_cfg_path: Path, section: str) -> Tuple[Dict[str, Any], Dict[str, Any], Path]:
    root = load_yaml(root_cfg_path)
    root_dir = root_cfg_path.parent
    config_files = root.get("config_files", {})
    if section not in config_files:
        raise KeyError(f"section '{section}' is not present in config_files")
    profile_path = resolve_path(str(config_files[section]), root_dir)
    profile_root = load_yaml(profile_path)
    if section not in profile_root:
        raise KeyError(f"profile file {profile_path} does not contain section '{section}'")
    section_cfg = profile_root[section]
    if not isinstance(section_cfg, dict):
        raise ValueError(f"profile section must be a map: {section}")
    return root, section_cfg, profile_path


def rl_master_root(root_cfg: Mapping[str, Any], root_cfg_path: Path) -> Path:
    raw = str(root_cfg.get("humanoid_rl_root_dir", "")).strip()
    if raw:
        return resolve_path(raw, root_cfg_path.parent)
    return root_cfg_path.parent.parent


def policy_path_for(section_cfg: Mapping[str, Any], root_dir: Path) -> Path:
    if str(section_cfg.get("policy_path", "")).strip():
        return resolve_path(str(section_cfg["policy_path"]), root_dir)
    if str(section_cfg.get("policy_file", "")).strip():
        return resolve_path(str(section_cfg["policy_file"]), root_dir)
    return root_dir / "policies" / f"{section_cfg['policy_name']}.onnx"


def parse_attr(value: Any) -> Any:
    if isinstance(value, bytes):
        value = value.decode("utf-8")
    if not isinstance(value, str):
        return value
    text = value.strip()
    if not text:
        return text
    try:
        return json.loads(text)
    except Exception:
        pass
    try:
        return ast.literal_eval(text)
    except Exception:
        pass
    if "," in text:
        return [part.strip() for part in text.split(",") if part.strip()]
    return text


def as_float_array(value: Any) -> Optional[np.ndarray]:
    parsed = parse_attr(value)
    if parsed is None or isinstance(parsed, str):
        return None
    try:
        return np.asarray(parsed, dtype=np.float32)
    except Exception:
        return None


def as_str_list(value: Any) -> List[str]:
    parsed = parse_attr(value)
    if isinstance(parsed, list):
        return [str(x) for x in parsed]
    if isinstance(parsed, tuple):
        return [str(x) for x in parsed]
    if isinstance(parsed, str):
        return [x.strip() for x in parsed.split(",") if x.strip()]
    return []


def named_vector_from_map(names: Sequence[str], values_by_name: Mapping[str, Any]) -> np.ndarray:
    return np.asarray([float(values_by_name[name]) for name in names], dtype=np.float32)


def remap(values: np.ndarray, source_order: Sequence[str], target_order: Sequence[str]) -> np.ndarray:
    source_index = {name: i for i, name in enumerate(source_order)}
    out = np.zeros((len(target_order),), dtype=np.float32)
    for i, name in enumerate(target_order):
        j = source_index.get(name)
        if j is not None and j < values.shape[0]:
            out[i] = values[j]
    return out


def compare_vector(name: str, a: np.ndarray, b: np.ndarray, labels: Sequence[str]) -> None:
    diff = a - b
    print(f"\n{name}:")
    print(f"  mae={np.mean(np.abs(diff)):.9g} rmse={math.sqrt(float(np.mean(diff * diff))):.9g} max_abs={np.max(np.abs(diff)):.9g}")
    top = np.argsort(-np.abs(diff))[: min(5, diff.shape[0])]
    for i in top:
        label = labels[i] if i < len(labels) else str(i)
        print(f"  {label}: deploy={a[i]: .9f} h5={b[i]: .9f} diff={diff[i]: .9f}")


def print_action_metrics(pred: np.ndarray, ref: np.ndarray, labels: Sequence[str]) -> Tuple[float, float]:
    diff = pred - ref
    abs_diff = np.abs(diff)
    mae = float(np.mean(abs_diff))
    rmse = math.sqrt(float(np.mean(diff * diff)))
    max_abs = float(np.max(abs_diff))
    print("\nAction replay error:")
    print(f"  frames={pred.shape[0]} action_dim={pred.shape[1]}")
    print(f"  mae={mae:.9g} rmse={rmse:.9g} max_abs={max_abs:.9g}")
    print(f"  p50={np.percentile(abs_diff, 50):.9g} p95={np.percentile(abs_diff, 95):.9g} p99={np.percentile(abs_diff, 99):.9g}")
    per_joint = np.mean(abs_diff, axis=0)
    print("  per-joint MAE:")
    for i, value in enumerate(per_joint):
        label = labels[i] if i < len(labels) else str(i)
        print(f"    {label}: {value:.9g}")
    worst_flat = np.argsort(-abs_diff.reshape(-1))[:5]
    print("  worst samples:")
    for flat in worst_flat:
        frame, joint = np.unravel_index(int(flat), abs_diff.shape)
        label = labels[joint] if joint < len(labels) else str(joint)
        print(
            f"    frame={frame} joint={label} pred={pred[frame, joint]: .9f} "
            f"h5={ref[frame, joint]: .9f} diff={diff[frame, joint]: .9f}"
        )
    return mae, max_abs


def build_time_steps(
    mode: str,
    h5_steps: np.ndarray,
    count: int,
    section_cfg: Mapping[str, Any],
) -> np.ndarray:
    policy_io = section_cfg.get("policy_io", {}) if isinstance(section_cfg.get("policy_io"), dict) else {}
    start = int(policy_io.get("time_step_start", section_cfg.get("time_step_start", 0)))
    if mode == "h5":
        return h5_steps.reshape(-1).astype(np.float32)
    if mode == "config":
        return (np.arange(count, dtype=np.float32) + float(start)).astype(np.float32)
    if mode == "zero":
        return np.zeros((count,), dtype=np.float32)
    raise ValueError(f"unsupported time step mode: {mode}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--h5", type=Path, default=DEFAULT_H5)
    parser.add_argument("--rl-cfg", type=Path, default=DEFAULT_CFG)
    parser.add_argument("--section", default=DEFAULT_SECTION)
    parser.add_argument("--time-step-mode", choices=("h5", "config", "zero"), default="h5")
    parser.add_argument("--mae-threshold", type=float, default=1.0e-5)
    parser.add_argument("--max-threshold", type=float, default=1.0e-4)
    parser.add_argument("--limit", type=int, default=0, help="Only replay the first N frames; 0 means all frames.")
    args = parser.parse_args()

    root_cfg_path = args.rl_cfg.expanduser().resolve()
    h5_path = args.h5.expanduser().resolve()
    root_cfg, section_cfg, profile_path = profile_for_section(root_cfg_path, args.section)
    root_dir = rl_master_root(root_cfg, root_cfg_path)
    onnx_path = policy_path_for(section_cfg, root_dir)

    if not h5_path.exists():
        raise FileNotFoundError(h5_path)
    if not onnx_path.exists():
        raise FileNotFoundError(onnx_path)

    with h5py.File(h5_path, "r") as f:
        obs = np.asarray(f["policy/obs"][:, 0, :], dtype=np.float32)
        ref_actions = np.asarray(f["policy/actions"][:, 0, :], dtype=np.float32)
        h5_time_steps = np.asarray(f["policy/time_step"][:, 0], dtype=np.float32)
        attrs = {key: parse_attr(value) for key, value in f.attrs.items()}

    if args.limit > 0:
        obs = obs[: args.limit]
        ref_actions = ref_actions[: args.limit]
        h5_time_steps = h5_time_steps[: args.limit]

    print(f"H5: {h5_path}")
    print(f"profile: {profile_path} [{args.section}]")
    print(f"ONNX: {onnx_path}")
    print(f"obs shape={obs.shape} actions shape={ref_actions.shape}")

    h5_action_names = as_str_list(attrs.get("action_joint_names"))
    h5_joint_names = as_str_list(attrs.get("joint_names"))
    h5_command_joint_names = as_str_list(attrs.get("command_joint_names"))
    h5_body_names = as_str_list(attrs.get("body_names"))
    h5_command_body_names = as_str_list(attrs.get("command_body_names"))
    cfg_action_names = [str(x) for x in section_cfg.get("action_joint_order", [])]
    cfg_reference_names = [str(x) for x in section_cfg.get("reference_joint_order", [])]
    cfg_reference_body_names = [str(x) for x in section_cfg.get("reference_body_names", [])]
    global_joint_order = [str(x) for x in root_cfg.get("robot_global_joint_order", [])]

    print("\nJoint-order contract:")
    print(f"  H5 action_joint_names == cfg action_joint_order: {h5_action_names == cfg_action_names}")
    print(f"  H5 joint_names == cfg reference_joint_order: {h5_joint_names == cfg_reference_names}")
    print(f"  H5 command_joint_names == root robot_global_joint_order: {h5_command_joint_names == global_joint_order}")
    print(f"  H5 command_body_names == cfg reference_body_names: {h5_command_body_names == cfg_reference_body_names}")
    if h5_body_names:
        print(f"  H5 body_names == cfg reference_body_names: {h5_body_names == cfg_reference_body_names}")
    print(f"  cfg preserve_reference_joint_order: {section_cfg.get('source_contract', {}).get('policy_extra_outputs', {}).get('preserve_reference_joint_order')}")

    sess_options = ort.SessionOptions()
    sess_options.intra_op_num_threads = max(1, int(section_cfg.get("onnx_intra_threads", 1)))
    sess_options.inter_op_num_threads = max(1, int(section_cfg.get("onnx_inter_threads", 1)))
    session = ort.InferenceSession(str(onnx_path), sess_options=sess_options, providers=["CPUExecutionProvider"])
    metadata = session.get_modelmeta().custom_metadata_map
    onnx_body_names = as_str_list(metadata.get("body_names"))
    if onnx_body_names:
        print(f"  ONNX body_names == cfg reference_body_names: {onnx_body_names == cfg_reference_body_names}")
    input_names = [x.name for x in session.get_inputs()]
    output_names = [x.name for x in session.get_outputs()]
    policy_io = section_cfg.get("policy_io", {}) if isinstance(section_cfg.get("policy_io"), dict) else {}
    obs_input_name = str(policy_io.get("obs_input_name", "obs"))
    action_output_name = str(policy_io.get("action_output_name", "actions"))
    time_step_input_name = str(policy_io.get("time_step_input_name", "time_step"))
    time_steps = build_time_steps(args.time_step_mode, h5_time_steps, obs.shape[0], section_cfg)

    if obs_input_name not in input_names:
        raise RuntimeError(f"obs input '{obs_input_name}' not in ONNX inputs {input_names}")
    if action_output_name not in output_names:
        raise RuntimeError(f"action output '{action_output_name}' not in ONNX outputs {output_names}")

    pred_actions: List[np.ndarray] = []
    first_extra: Dict[str, np.ndarray] = {}
    extra_outputs = [name for name in ("joint_pos", "joint_vel") if name in output_names]
    requested_outputs = [action_output_name] + extra_outputs
    for i, frame_obs in enumerate(obs):
        inputs = {obs_input_name: frame_obs[None, :].astype(np.float32)}
        if time_step_input_name in input_names:
            inputs[time_step_input_name] = np.asarray([[time_steps[i]]], dtype=np.float32)
        outputs = session.run(requested_outputs, inputs)
        pred_actions.append(outputs[0][0].astype(np.float32))
        if i == 0:
            for name, value in zip(requested_outputs[1:], outputs[1:]):
                first_extra[name] = value[0].astype(np.float32)
    pred = np.asarray(pred_actions, dtype=np.float32)
    mae, max_abs = print_action_metrics(pred, ref_actions, h5_action_names or cfg_action_names)

    h5_action_offset = as_float_array(attrs.get("action_offset"))
    h5_action_scale = as_float_array(attrs.get("action_scale"))
    robot_cfg = section_cfg.get("robot", {}) if isinstance(section_cfg.get("robot"), dict) else {}
    default_angles = robot_cfg.get("default_joint_angles", {})
    cfg_scales_map = section_cfg.get("action_scales", {})
    if h5_action_names and isinstance(default_angles, dict) and h5_action_offset is not None:
        cfg_default_in_h5_order = named_vector_from_map(h5_action_names, default_angles)
        compare_vector("Action offset/default_joint_angles", cfg_default_in_h5_order, h5_action_offset, h5_action_names)
    if h5_action_names and isinstance(cfg_scales_map, dict) and h5_action_scale is not None:
        cfg_scale_in_h5_order = named_vector_from_map(h5_action_names, cfg_scales_map)
        compare_vector("Action scales", cfg_scale_in_h5_order, h5_action_scale, h5_action_names)

    slices = attrs.get("policy_observation_slices", {})
    if isinstance(slices, str):
        slices = parse_attr(slices)
    command_slice = slices.get("command") if isinstance(slices, dict) else None
    if command_slice and len(command_slice) == 2:
        c0, c1 = int(command_slice[0]), int(command_slice[1])
        command = obs[0, c0:c1]
        print(f"\nCommand slice check: command=[{c0}, {c1}) dim={command.shape[0]}")
        if "joint_pos" in first_extra and h5_command_joint_names and cfg_reference_names and global_joint_order:
            extra_pos = first_extra["joint_pos"]
            direct_mae = float(np.mean(np.abs(command[: extra_pos.shape[0]] - extra_pos)))
            remapped = remap(extra_pos, cfg_reference_names, global_joint_order)
            remap_mae = float(np.mean(np.abs(command[: remapped.shape[0]] - remapped)))
            print(f"  first command joint_pos vs ONNX joint_pos direct MAE: {direct_mae:.9g}")
            print(f"  first command joint_pos vs ONNX joint_pos remapped cfg.reference->global MAE: {remap_mae:.9g}")
        if "joint_vel" in first_extra and h5_command_joint_names and cfg_reference_names and global_joint_order:
            half = command.shape[0] // 2
            extra_vel = first_extra["joint_vel"]
            direct_mae = float(np.mean(np.abs(command[half : half + extra_vel.shape[0]] - extra_vel)))
            remapped = remap(extra_vel, cfg_reference_names, global_joint_order)
            remap_mae = float(np.mean(np.abs(command[half : half + remapped.shape[0]] - remapped)))
            print(f"  first command joint_vel vs ONNX joint_vel direct MAE: {direct_mae:.9g}")
            print(f"  first command joint_vel vs ONNX joint_vel remapped cfg.reference->global MAE: {remap_mae:.9g}")

    if mae > args.mae_threshold or max_abs > args.max_threshold:
        print(
            f"\n[FAIL] action replay error exceeds thresholds: "
            f"mae {mae:.9g} > {args.mae_threshold} or max_abs {max_abs:.9g} > {args.max_threshold}"
        )
        return 2
    print("\n[PASS] action replay matches H5 within thresholds.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
