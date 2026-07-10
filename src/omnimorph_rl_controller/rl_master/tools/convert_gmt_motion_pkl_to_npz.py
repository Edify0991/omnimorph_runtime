#!/usr/bin/env python3
"""Convert an official GMT motion pickle into rl_master runtime reference data."""

from __future__ import annotations

import argparse
import io
import pickle
import zipfile
from pathlib import Path

import numpy as np


FUTURE_STEPS = np.asarray(
    [1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95],
    dtype=np.float64,
)


def quat_conjugate_xyzw(q: np.ndarray) -> np.ndarray:
    out = np.array(q, copy=True)
    out[..., :3] *= -1.0
    return out


def quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    av, aw = a[..., :3], a[..., 3:4]
    bv, bw = b[..., :3], b[..., 3:4]
    xyz = aw * bv + bw * av + np.cross(av, bv)
    w = aw * bw - np.sum(av * bv, axis=-1, keepdims=True)
    return np.concatenate([xyz, w], axis=-1)


def quat_to_exp_map_xyzw(q: np.ndarray) -> np.ndarray:
    w = np.clip(q[..., 3], -1.0, 1.0)
    sin_theta = np.sqrt(np.clip(1.0 - w * w, 0.0, None))
    angle = np.arctan2(np.sin(2.0 * np.arccos(w)), np.cos(2.0 * np.arccos(w)))
    axis = np.zeros_like(q[..., :3])
    valid = np.abs(sin_theta) > 1.0e-5
    axis[valid] = q[..., :3][valid] / sin_theta[valid, None]
    axis[~valid, 2] = 1.0
    angle = np.where(valid, angle, 0.0)
    return angle[..., None] * axis


def quat_rotate_inverse_xyzw(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q_vec = q[..., :3]
    q_w = q[..., 3:4]
    a = v * (2.0 * q_w * q_w - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.sum(q_vec * v, axis=-1, keepdims=True) * 2.0
    return a - b + c


def quat_to_roll_pitch_xyzw(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x, y, z, w = np.moveaxis(q, -1, 0)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch


def slerp_xyzw(q0: np.ndarray, q1: np.ndarray, blend: np.ndarray) -> np.ndarray:
    dot = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(dot < 0.0, -q1, q1)
    dot = np.clip(np.abs(dot), 0.0, 1.0)
    half_theta = np.arccos(dot)
    sin_half_theta = np.sqrt(np.clip(1.0 - dot * dot, 0.0, None))
    t = blend[..., None]
    denominator = np.where(sin_half_theta > 1.0e-8, sin_half_theta, 1.0)
    out = np.sin((1.0 - t) * half_theta) / denominator * q0
    out += np.sin(t * half_theta) / denominator * q1
    linear = 0.5 * q0 + 0.5 * q1
    out = np.where(sin_half_theta < 0.001, linear, out)
    out = np.where(dot >= 1.0, q0, out)
    return out


def smooth_like_gmt(values: np.ndarray, window: int = 19) -> np.ndarray:
    kernel = np.ones(window, dtype=np.float64) / float(window)
    start = (window - 1) // 2
    return np.stack(
        [np.convolve(values[:, i], kernel, mode="full")[start : start + values.shape[0]] for i in range(values.shape[1])],
        axis=1,
    )


def build_motion_state(data: dict) -> dict[str, np.ndarray | float]:
    fps = float(data["fps"])
    root_pos = np.asarray(data["root_pos"], dtype=np.float64)
    # Preserve GMT's published quaternion magnitudes. The official runtime does
    # not normalize them before interpolation or observation construction.
    root_rot = np.asarray(data["root_rot"], dtype=np.float64)
    dof_pos = np.asarray(data["dof_pos"], dtype=np.float64)
    if root_pos.ndim != 2 or root_pos.shape[1] != 3:
        raise ValueError(f"root_pos must have shape [T,3], got {root_pos.shape}")
    if root_rot.shape != (root_pos.shape[0], 4):
        raise ValueError(f"root_rot must have shape [T,4], got {root_rot.shape}")
    if dof_pos.shape != (root_pos.shape[0], 23):
        raise ValueError(f"GMT G1 dof_pos must have shape [T,23], got {dof_pos.shape}")
    if root_pos.shape[0] < 2 or fps <= 0.0:
        raise ValueError("motion needs at least two frames and a positive fps")

    root_vel = np.zeros_like(root_pos)
    root_vel[:-1] = fps * (root_pos[1:] - root_pos[:-1])
    root_vel[-1] = root_vel[-2]
    root_vel = smooth_like_gmt(root_vel)

    root_drot = quat_mul_xyzw(root_rot[1:], quat_conjugate_xyzw(root_rot[:-1]))
    root_ang_vel = np.zeros_like(root_pos)
    root_ang_vel[:-1] = fps * quat_to_exp_map_xyzw(root_drot)
    root_ang_vel[-1] = root_ang_vel[-2]
    root_ang_vel = smooth_like_gmt(root_ang_vel)

    return {
        "fps": fps,
        "length": (root_pos.shape[0] - 1) / fps,
        "root_pos": root_pos,
        "root_rot": root_rot,
        "root_vel": root_vel,
        "root_ang_vel": root_ang_vel,
        "dof_pos": dof_pos,
    }


def sample_motion(state: dict[str, np.ndarray | float], times: np.ndarray) -> tuple[np.ndarray, ...]:
    length = float(state["length"])
    fps = float(state["fps"])
    root_pos = np.asarray(state["root_pos"])
    wrapped = np.mod(times, length)
    phase_frame = wrapped * fps
    idx0 = np.floor(phase_frame).astype(np.int64)
    idx0 = np.clip(idx0, 0, root_pos.shape[0] - 1)
    idx1 = np.minimum(idx0 + 1, root_pos.shape[0] - 1)
    blend = phase_frame - idx0

    sampled_pos = (1.0 - blend[:, None]) * root_pos[idx0] + blend[:, None] * root_pos[idx1]
    sampled_rot = slerp_xyzw(np.asarray(state["root_rot"])[idx0], np.asarray(state["root_rot"])[idx1], blend)
    sampled_dof = ((1.0 - blend[:, None]) * np.asarray(state["dof_pos"])[idx0] +
                   blend[:, None] * np.asarray(state["dof_pos"])[idx1])
    return (
        sampled_pos,
        sampled_rot,
        np.asarray(state["root_vel"])[idx0],
        np.asarray(state["root_ang_vel"])[idx0],
        sampled_dof,
    )


def build_runtime_reference(data: dict, control_fps: float) -> np.ndarray:
    state = build_motion_state(data)
    length = float(state["length"])
    frame_count = max(1, int(round(length * control_fps)))
    current_times = np.arange(frame_count, dtype=np.float64) / control_fps
    future_times = current_times[:, None] + FUTURE_STEPS[None, :] / control_fps
    flat_times = future_times.reshape(-1)
    root_pos, root_rot, root_vel, root_ang_vel, dof_pos = sample_motion(state, flat_times)

    roll, pitch = quat_to_roll_pitch_xyzw(root_rot)
    root_vel_local = quat_rotate_inverse_xyzw(root_rot, root_vel)
    root_ang_vel_local = quat_rotate_inverse_xyzw(root_rot, root_ang_vel)
    per_target = np.concatenate(
        [
            root_pos[:, 2:3],
            roll[:, None],
            pitch[:, None],
            root_vel_local,
            root_ang_vel_local[:, 2:3],
            dof_pos,
        ],
        axis=1,
    )
    if per_target.shape[1] != 30:
        raise RuntimeError(f"unexpected GMT target feature dim: {per_target.shape}")
    return per_target.reshape(frame_count, 600).astype(np.float32)


def write_npz_stored(path: Path, arrays: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, mode="w", compression=zipfile.ZIP_STORED, allowZip64=False) as archive:
        for key, value in arrays.items():
            buffer = io.BytesIO()
            np.lib.format.write_array(buffer, np.asanyarray(value), allow_pickle=False)
            archive.writestr(f"{key}.npy", buffer.getvalue(), compress_type=zipfile.ZIP_STORED)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Official GMT motion .pkl")
    parser.add_argument("--output", required=True, type=Path, help="Runtime .npz output")
    parser.add_argument("--control-fps", type=float, default=50.0)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    if args.output.exists() and not args.force:
        raise FileExistsError(f"output exists: {args.output}")
    if args.control_fps <= 0.0:
        raise ValueError("--control-fps must be positive")

    # GMT motion files are trusted Python pickles from the official repository.
    with args.input.open("rb") as stream:
        data = pickle.load(stream)
    reference_motion = build_runtime_reference(data, args.control_fps)
    write_npz_stored(
        args.output,
        {
            "reference_motion": reference_motion,
            "fps": np.asarray([args.control_fps], dtype=np.float32),
        },
    )
    print(f"input: {args.input}")
    print(f"output: {args.output}")
    print(f"source_fps: {float(data['fps']):.9f}")
    print(f"runtime_fps: {args.control_fps:.9f}")
    print(f"frames: {reference_motion.shape[0]}")
    print(f"reference_dim: {reference_motion.shape[1]}")


if __name__ == "__main__":
    main()
