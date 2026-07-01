#!/usr/bin/env python3
"""Convert generic G1 motion npz into tracker reference assets.

Outputs:
1. runtime-npz: directly loadable by rl_master ReferenceMotionProvider
2. opentrack-onnx: OpenTrack-compatible constant ref_data.onnx (optional)
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Iterable, Sequence

import mujoco
import numpy as np


TARGET_JOINT_ORDER = [
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

DEFAULT_QPOS = np.asarray(
    [
        0.0,
        0.0,
        0.79,
        1.0,
        0.0,
        0.0,
        0.0,
        -0.1,
        0.0,
        0.0,
        0.3,
        -0.2,
        0.0,
        -0.1,
        0.0,
        0.0,
        0.3,
        -0.2,
        0.0,
        0.0,
        0.0,
        0.0,
        0.2,
        0.3,
        0.0,
        1.28,
        0.0,
        0.0,
        0.0,
        0.2,
        -0.3,
        0.0,
        1.28,
        0.0,
        0.0,
        0.0,
    ],
    dtype=np.float32,
)

FEET_ALL_SITES = [
    "left_foot",
    "right_foot",
    "left_foot_top",
    "right_foot_top",
]


def _as_name_list(raw: np.ndarray | Sequence[str]) -> list[str]:
    if isinstance(raw, np.ndarray):
        values = raw.tolist()
    else:
        values = list(raw)
    out = [str(item) for item in values]
    if out and out[0] == "root":
        out = out[1:]
    return out


def _normalize_quat_wxyz(quat: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quat, axis=-1, keepdims=True)
    norm = np.clip(norm, 1.0e-9, None)
    return quat / norm


def _quat_to_wxyz(quat: np.ndarray, order: str) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float32)
    if quat.shape[-1] != 4:
        raise ValueError(f"quat last dimension must be 4, got shape {quat.shape}")
    if order == "wxyz":
        out = quat
    elif order == "xyzw":
        out = np.concatenate([quat[..., 3:4], quat[..., :3]], axis=-1)
    else:
        raise ValueError(f"unsupported quaternion order: {order}")
    return _normalize_quat_wxyz(out.astype(np.float32, copy=False))


def _slerp_pair(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = q0.astype(np.float64, copy=False)
    q1 = q1.astype(np.float64, copy=False)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        out = q0 + alpha * (q1 - q0)
        out /= max(np.linalg.norm(out), 1.0e-9)
        return out.astype(np.float32)
    theta_0 = math.acos(dot)
    sin_theta_0 = math.sin(theta_0)
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    out = s0 * q0 + s1 * q1
    out /= max(np.linalg.norm(out), 1.0e-9)
    return out.astype(np.float32)


def _resample_linear(values: np.ndarray, src_times: np.ndarray, dst_times: np.ndarray) -> np.ndarray:
    out = np.empty((dst_times.shape[0], values.shape[1]), dtype=np.float32)
    for dim in range(values.shape[1]):
        out[:, dim] = np.interp(dst_times, src_times, values[:, dim]).astype(np.float32)
    return out


def _resample_qpos(qpos: np.ndarray, source_fps: float, target_fps: float) -> np.ndarray:
    if qpos.shape[0] <= 1 or abs(source_fps - target_fps) < 1.0e-6:
        result = qpos.astype(np.float32, copy=True)
        result[:, 3:7] = _normalize_quat_wxyz(result[:, 3:7])
        return result

    duration = (qpos.shape[0] - 1) / source_fps
    dst_count = max(2, int(round(duration * target_fps)) + 1)
    src_times = np.arange(qpos.shape[0], dtype=np.float64) / source_fps
    dst_times = np.arange(dst_count, dtype=np.float64) / target_fps
    dst_times[-1] = duration

    out = _resample_linear(qpos, src_times, dst_times)
    out[:, 3:7] = 0.0
    src_quat = _normalize_quat_wxyz(qpos[:, 3:7].astype(np.float32))
    src_index = np.searchsorted(src_times, dst_times, side="right") - 1
    src_index = np.clip(src_index, 0, qpos.shape[0] - 2)
    for i, t in enumerate(dst_times):
        idx = int(src_index[i])
        t0 = src_times[idx]
        t1 = src_times[idx + 1]
        alpha = 0.0 if t1 <= t0 else float((t - t0) / (t1 - t0))
        out[i, 3:7] = _slerp_pair(src_quat[idx], src_quat[idx + 1], alpha)
    out[:, 3:7] = _normalize_quat_wxyz(out[:, 3:7])
    return out


def _quat_angle_axis_between(q0: np.ndarray, q1: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    w1, x1, y1, z1 = q0[:, 0], q0[:, 1], q0[:, 2], q0[:, 3]
    w2, x2, y2, z2 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    s = 2.0 * (w**2) - 1.0
    angle = np.arccos(np.clip(s, -1.0, 1.0))
    axis = np.stack([x, y, z], axis=1)
    axis /= np.linalg.norm(axis, axis=-1, keepdims=True).clip(min=1.0e-9)
    return angle.astype(np.float32), axis.astype(np.float32)


def _recalculate_qvel_from_qpos(qpos: np.ndarray, fps: float) -> np.ndarray:
    qvel = np.zeros((qpos.shape[0], 35), dtype=np.float32)
    qvel[:-1, 0:3] = (qpos[1:, 0:3] - qpos[:-1, 0:3]) * fps
    quat = _normalize_quat_wxyz(qpos[:, 3:7])
    quat_inv = np.concatenate([quat[:, :1], -quat[:, 1:]], axis=1)
    angle, axis = _quat_angle_axis_between(quat_inv[:-1], quat[1:])
    qvel[:-1, 3:6] = axis * angle[:, None] * fps
    qvel[:-1, 6:] = (qpos[1:, 7:] - qpos[:-1, 7:]) * fps
    return qvel


def _add_transition_frames(qpos: np.ndarray, transition_frames: int) -> np.ndarray:
    if transition_frames <= 0:
        return qpos.astype(np.float32, copy=True)

    out = np.zeros((qpos.shape[0] + transition_frames * 2, qpos.shape[1]), dtype=np.float32)
    out[:transition_frames] = np.linspace(DEFAULT_QPOS, qpos[0], transition_frames, dtype=np.float32)
    out[transition_frames : transition_frames + qpos.shape[0]] = qpos
    out[transition_frames + qpos.shape[0] :] = np.linspace(qpos[-1], DEFAULT_QPOS, transition_frames, dtype=np.float32)

    out[:transition_frames, :7] = qpos[0, :7]
    out[transition_frames + qpos.shape[0] :, :7] = qpos[-1, :7]
    out[:, 3:7] = _normalize_quat_wxyz(out[:, 3:7])
    return out


def _map_source_motion_to_target(
    qpos: np.ndarray,
    source_joint_names: Sequence[str],
) -> tuple[np.ndarray, list[str]]:
    if qpos.ndim != 2:
        raise ValueError(f"qpos must be rank-2, got shape {qpos.shape}")
    source_joint_dim = qpos.shape[1] - 7
    if source_joint_dim != len(source_joint_names):
        raise ValueError(
            f"qpos joint dimension mismatch: got {source_joint_dim}, "
            f"but source_joint_names has {len(source_joint_names)} items"
        )

    target_qpos = np.tile(DEFAULT_QPOS[None, :], (qpos.shape[0], 1)).astype(np.float32)
    target_qpos[:, :7] = qpos[:, :7].astype(np.float32)

    name_to_target_index = {name: i for i, name in enumerate(TARGET_JOINT_ORDER)}
    missing = []
    for source_idx, joint_name in enumerate(source_joint_names):
        target_idx = name_to_target_index.get(joint_name)
        if target_idx is None:
            missing.append(joint_name)
            continue
        target_qpos[:, 7 + target_idx] = qpos[:, 7 + source_idx]
    return target_qpos, missing


def _build_qpos_from_root_and_dofs(
    root_pos_w: np.ndarray,
    root_quat_wxyz: np.ndarray,
    dof_pos: np.ndarray,
    source_joint_names: Sequence[str],
) -> tuple[np.ndarray, list[str]]:
    if root_pos_w.ndim != 2 or root_pos_w.shape[1] != 3:
        raise ValueError(f"root_pos_w must have shape [T,3], got {root_pos_w.shape}")
    if root_quat_wxyz.ndim != 2 or root_quat_wxyz.shape[1] != 4:
        raise ValueError(f"root_quat_wxyz must have shape [T,4], got {root_quat_wxyz.shape}")
    if dof_pos.ndim != 2:
        raise ValueError(f"dof_pos must be rank-2, got shape {dof_pos.shape}")
    if root_pos_w.shape[0] != dof_pos.shape[0] or root_quat_wxyz.shape[0] != dof_pos.shape[0]:
        raise ValueError(
            "root/body frame count mismatch: "
            f"root_pos={root_pos_w.shape[0]}, root_quat={root_quat_wxyz.shape[0]}, dof_pos={dof_pos.shape[0]}"
        )

    qpos = np.concatenate([root_pos_w, root_quat_wxyz, dof_pos], axis=1).astype(np.float32)
    return _map_source_motion_to_target(qpos, source_joint_names)


def _score_root_quat_order(
    root_pos_w: np.ndarray,
    root_quat_raw: np.ndarray,
    dof_pos: np.ndarray,
    source_joint_names: Sequence[str],
    scene_xml: Path,
    feet_sites: Sequence[str],
    quat_order: str,
) -> float:
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    site_ids = np.asarray([model.site(name).id for name in feet_sites], dtype=np.int32)
    root_quat_wxyz = _quat_to_wxyz(root_quat_raw, quat_order)
    qpos, _ = _build_qpos_from_root_and_dofs(root_pos_w, root_quat_wxyz, dof_pos, source_joint_names)

    if model.nq != qpos.shape[1]:
        raise ValueError(f"scene model nq={model.nq} but qpos has {qpos.shape[1]}")

    sample_count = min(32, qpos.shape[0])
    sample_indices = np.linspace(0, qpos.shape[0] - 1, sample_count, dtype=np.int32)
    score = 0.0
    for idx in sample_indices:
        data.qpos[:] = qpos[idx]
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        feet_z = data.site_xpos[site_ids, 2]
        below_ground = np.clip(-feet_z, 0.0, None)
        # Prefer solutions with feet close to the floor but not penetrating it.
        score += float(np.mean(feet_z) + 10.0 * np.mean(below_ground))
    return score / float(sample_count)


def _infer_root_quat_order(
    root_pos_w: np.ndarray,
    root_quat_raw: np.ndarray,
    dof_pos: np.ndarray,
    source_joint_names: Sequence[str],
    scene_xml: Path,
    feet_sites: Sequence[str],
) -> str:
    scores = {
        "wxyz": _score_root_quat_order(
            root_pos_w,
            root_quat_raw,
            dof_pos,
            source_joint_names,
            scene_xml,
            feet_sites,
            "wxyz",
        ),
        "xyzw": _score_root_quat_order(
            root_pos_w,
            root_quat_raw,
            dof_pos,
            source_joint_names,
            scene_xml,
            feet_sites,
            "xyzw",
        ),
    }
    best = min(scores, key=scores.get)
    print(
        "auto-detected root quaternion order:",
        f"{best} (score wxyz={scores['wxyz']:.6f}, xyzw={scores['xyzw']:.6f})",
    )
    return best


def _compute_reference_features(
    qpos: np.ndarray,
    qvel: np.ndarray,
    scene_xml: Path,
    feet_sites: Sequence[str],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(scene_xml))
    data = mujoco.MjData(model)
    if model.nq != qpos.shape[1]:
        raise ValueError(f"scene model nq={model.nq} but qpos has {qpos.shape[1]}")
    if model.nv != qvel.shape[1]:
        raise ValueError(f"scene model nv={model.nv} but qvel has {qvel.shape[1]}")

    site_ids = np.asarray([model.site(name).id for name in feet_sites], dtype=np.int32)
    body_ids = np.arange(1, model.nbody, dtype=np.int32)
    feet_height = np.zeros((qpos.shape[0], len(feet_sites)), dtype=np.float32)
    root_height = np.zeros((qpos.shape[0], 1), dtype=np.float32)
    body_pos_w = np.zeros((qpos.shape[0], body_ids.shape[0], 3), dtype=np.float32)
    body_quat_w = np.zeros((qpos.shape[0], body_ids.shape[0], 4), dtype=np.float32)
    for i in range(qpos.shape[0]):
        data.qpos[:] = qpos[i]
        data.qvel[:] = qvel[i]
        mujoco.mj_forward(model, data)
        feet_height[i] = data.site_xpos[site_ids, 2]
        root_height[i, 0] = data.qpos[2]
        body_pos_w[i] = data.xpos[body_ids]
        body_quat_w[i] = data.xquat[body_ids]
    return feet_height, root_height, body_pos_w, body_quat_w


def _save_runtime_npz(
    output_path: Path,
    qpos: np.ndarray,
    qvel: np.ndarray,
    feet_height: np.ndarray,
    root_height: np.ndarray,
    body_pos_w: np.ndarray,
    body_quat_w: np.ndarray,
    fps: float,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        reference_motion=np.concatenate([feet_height, root_height], axis=1).astype(np.float32),
        joint_pos=qpos[:, 7:].astype(np.float32),
        joint_vel=qvel[:, 6:].astype(np.float32),
        body_pos_w=body_pos_w.astype(np.float32),
        body_quat_w=body_quat_w.astype(np.float32),
        fps=np.asarray([fps], dtype=np.float32),
    )


def _save_opentrack_onnx(
    output_path: Path,
    qpos: np.ndarray,
    qvel: np.ndarray,
    feet_height: np.ndarray,
    root_height: np.ndarray,
) -> None:
    try:
        import onnx
        from onnx import TensorProto, helper
        from onnx import StringStringEntryProto
    except ImportError as exc:
        raise RuntimeError(
            "Python package 'onnx' is required for --output-format opentrack-onnx/both"
        ) from exc

    def create_constant_model(data_list: Sequence[np.ndarray], output_names: Sequence[str]):
        nodes = []
        outputs = []
        for idx, (data, name) in enumerate(zip(data_list, output_names)):
            tensor = helper.make_tensor(
                name=name,
                data_type=TensorProto.FLOAT,
                dims=data.shape,
                vals=data.astype(np.float32).ravel().tolist(),
            )
            outputs.append(helper.make_tensor_value_info(name, TensorProto.FLOAT, data.shape))
            nodes.append(
                helper.make_node(
                    "Constant",
                    inputs=[],
                    outputs=[name],
                    name=f"Constant_Node_{idx}",
                    value=tensor,
                )
            )
        graph = helper.make_graph(nodes, "ConstantOutputModel", inputs=[], outputs=outputs)
        model = helper.make_model(graph, producer_name="convert_motion_npz_to_g1_reference", opset_imports=[helper.make_opsetid("", 13)])
        model.metadata_props.append(StringStringEntryProto(key="total_steps", value=str(data_list[0].shape[0])))
        return model

    model = create_constant_model(
        [qpos, qvel, feet_height, root_height],
        ["qpos", "qvel", "feet_height", "root_height"],
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, str(output_path))


def _resolve_source_fps(arrays: np.lib.npyio.NpzFile, override: float | None, default_fps: float) -> float:
    if override is not None and override > 1.0e-6:
        return float(override)
    if "fps" in arrays:
        fps = arrays["fps"]
        if np.asarray(fps).size > 0:
            return float(np.asarray(fps).reshape(-1)[0])
    if "frequency" in arrays:
        freq = arrays["frequency"]
        if np.asarray(freq).size > 0:
            return float(np.asarray(freq).reshape(-1)[0])
    return float(default_fps)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert motion npz to G1 tracker reference assets."
    )
    parser.add_argument("input", type=Path, help="Source motion .npz")
    parser.add_argument(
        "--runtime-npz-out",
        type=Path,
        default=Path("src/omnimorph_rl_controller/rl_master/data/reference_motion/opentrack/dance2_subject5_runtime.npz"),
        help="Output runtime .npz path",
    )
    parser.add_argument(
        "--opentrack-onnx-out",
        type=Path,
        default=Path("src/omnimorph_rl_controller/rl_master/data/reference_motion/opentrack/dance2_subject5_ref_data.onnx"),
        help="Output OpenTrack-style ref_data.onnx path",
    )
    parser.add_argument(
        "--scene-xml",
        type=Path,
        default=Path("/home/edify/Code/OpenTrack/storage/assets/unitree_g1/scene_mjx_flat_terrain.xml"),
        help="MuJoCo scene xml used to compute foot heights",
    )
    parser.add_argument(
        "--output-format",
        choices=["runtime-npz", "opentrack-onnx", "both"],
        default="runtime-npz",
        help="Which asset(s) to write",
    )
    parser.add_argument(
        "--target-fps",
        type=float,
        default=50.0,
        help="Target output frame rate used by tracker/runtime",
    )
    parser.add_argument(
        "--source-fps",
        type=float,
        default=None,
        help="Optional override of input motion frame rate",
    )
    parser.add_argument(
        "--transition-frames",
        type=int,
        default=50,
        help="Frames appended before/after motion to transition from/to DEFAULT_QPOS",
    )
    parser.add_argument(
        "--root-body-name",
        type=str,
        default="pelvis",
        help="Root body name used when input stores body_positions/body_rotations instead of qpos",
    )
    parser.add_argument(
        "--input-body-quat-order",
        choices=["auto", "wxyz", "xyzw"],
        default="auto",
        help="Quaternion order for body_rotations when using body-based motion input",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    arrays = np.load(args.input, allow_pickle=True)
    source_fps = _resolve_source_fps(arrays, args.source_fps, args.target_fps)
    if "qpos" in arrays and "joint_names" in arrays:
        source_joint_names = _as_name_list(arrays["joint_names"])
        source_qpos = np.asarray(arrays["qpos"], dtype=np.float32)
        target_qpos, unmapped = _map_source_motion_to_target(source_qpos, source_joint_names)
        input_summary = "qpos+joint_names"
    elif "dof_positions" in arrays and "dof_names" in arrays and "body_positions" in arrays and "body_rotations" in arrays:
        source_joint_names = _as_name_list(arrays["dof_names"])
        dof_pos = np.asarray(arrays["dof_positions"], dtype=np.float32)
        body_pos = np.asarray(arrays["body_positions"], dtype=np.float32)
        body_rot = np.asarray(arrays["body_rotations"], dtype=np.float32)
        body_names = _as_name_list(arrays["body_names"]) if "body_names" in arrays else []
        if body_pos.ndim != 3 or body_pos.shape[2] != 3:
            raise SystemExit(f"{args.input} key 'body_positions' must have shape [T,B,3], got {body_pos.shape}")
        if body_rot.ndim != 3 or body_rot.shape[2] != 4:
            raise SystemExit(f"{args.input} key 'body_rotations' must have shape [T,B,4], got {body_rot.shape}")
        if body_pos.shape[:2] != body_rot.shape[:2]:
            raise SystemExit(
                f"{args.input} body_positions/body_rotations shape mismatch: {body_pos.shape} vs {body_rot.shape}"
            )
        if body_names and len(body_names) != body_pos.shape[1]:
            raise SystemExit(
                f"{args.input} body_names count mismatch: {len(body_names)} vs body_positions second dim {body_pos.shape[1]}"
            )
        try:
            root_body_index = body_names.index(args.root_body_name)
        except ValueError as exc:
            raise SystemExit(
                f"{args.input} missing root body '{args.root_body_name}' in body_names={body_names}"
            ) from exc

        root_pos_w = body_pos[:, root_body_index, :]
        quat_order = args.input_body_quat_order
        if quat_order == "auto":
            quat_order = _infer_root_quat_order(
                root_pos_w,
                body_rot[:, root_body_index, :],
                dof_pos,
                source_joint_names,
                args.scene_xml,
                FEET_ALL_SITES,
            )
        root_quat_wxyz = _quat_to_wxyz(body_rot[:, root_body_index, :], quat_order)
        source_qpos = np.concatenate([root_pos_w, root_quat_wxyz, dof_pos], axis=1).astype(np.float32)
        target_qpos, unmapped = _map_source_motion_to_target(source_qpos, source_joint_names)
        input_summary = f"dof_positions+body_pose(root={args.root_body_name}, quat_order={quat_order})"
    else:
        raise SystemExit(
            f"{args.input} unsupported schema. Need either "
            "'qpos'+'joint_names' or "
            "'dof_positions'+'dof_names'+'body_positions'+'body_rotations'."
        )

    target_qpos = _resample_qpos(target_qpos, source_fps, args.target_fps)
    target_qpos = _add_transition_frames(target_qpos, args.transition_frames)
    target_qvel = _recalculate_qvel_from_qpos(target_qpos, args.target_fps)
    feet_height, root_height, body_pos_w, body_quat_w = _compute_reference_features(
        target_qpos,
        target_qvel,
        args.scene_xml,
        FEET_ALL_SITES,
    )

    if args.output_format in ("runtime-npz", "both"):
        _save_runtime_npz(
            args.runtime_npz_out,
            target_qpos,
            target_qvel,
            feet_height,
            root_height,
            body_pos_w,
            body_quat_w,
            args.target_fps,
        )
        print(f"runtime npz written: {args.runtime_npz_out}")
    if args.output_format in ("opentrack-onnx", "both"):
        _save_opentrack_onnx(
            args.opentrack_onnx_out,
            target_qpos,
            target_qvel,
            feet_height,
            root_height,
        )
        print(f"OpenTrack ref_data.onnx written: {args.opentrack_onnx_out}")

    missing_target = [name for name in TARGET_JOINT_ORDER if name not in source_joint_names]
    if missing_target:
        print("missing source joints padded from DEFAULT_QPOS/zero velocity:")
        for name in missing_target:
            print(f"  - {name}")
    if unmapped:
        print("source joints ignored because they are not in target order:")
        for name in unmapped:
            print(f"  - {name}")
    print(
        "summary:",
        f"input_schema={input_summary}",
        f"input_frames={source_qpos.shape[0]}",
        f"source_fps={source_fps}",
        f"output_frames={target_qpos.shape[0]}",
        f"target_fps={args.target_fps}",
        f"joint_dim={len(TARGET_JOINT_ORDER)}",
    )


if __name__ == "__main__":
    main()
