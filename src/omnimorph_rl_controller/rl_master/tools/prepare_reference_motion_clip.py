#!/usr/bin/env python3
"""Trim and yaw/xy-align a reference motion npz for deployment."""

from __future__ import annotations

import argparse
import io
import math
import zipfile
from pathlib import Path

import numpy as np


def normalize_quat_wxyz(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    norm = np.where(norm > 1e-12, norm, 1.0)
    return q / norm


def quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=-1,
    )


def quat_conj_wxyz(q: np.ndarray) -> np.ndarray:
    out = np.array(q, copy=True)
    out[..., 1:] *= -1.0
    return out


def yaw_from_quat_wxyz(q: np.ndarray) -> float:
    q = normalize_quat_wxyz(q)
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def rotation_z(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def enforce_quat_continuity(q: np.ndarray) -> np.ndarray:
    q = normalize_quat_wxyz(q)
    out = np.array(q, copy=True)
    for i in range(1, out.shape[0]):
        flip = np.sum(out[i - 1] * out[i], axis=-1) < 0.0
        out[i, flip] *= -1.0
    return out


def angular_velocity_from_quat_wxyz(q: np.ndarray, fps: float) -> np.ndarray:
    q = enforce_quat_continuity(q)
    omega = np.zeros(q.shape[:-1] + (3,), dtype=np.float64)
    if q.shape[0] <= 1:
        return omega

    def delta_to_omega(q0: np.ndarray, q1: np.ndarray, dt: float) -> np.ndarray:
        dq = normalize_quat_wxyz(quat_mul_wxyz(q1, quat_conj_wxyz(q0)))
        dq = np.where(dq[..., :1] < 0.0, -dq, dq)
        v = dq[..., 1:]
        v_norm = np.linalg.norm(v, axis=-1, keepdims=True)
        angle = 2.0 * np.arctan2(v_norm, np.clip(dq[..., :1], -1.0, 1.0))
        axis = np.divide(v, v_norm, out=np.zeros_like(v), where=v_norm > 1e-12)
        return axis * angle / dt

    dt = 1.0 / fps
    omega[0] = delta_to_omega(q[0], q[1], dt)
    omega[-1] = delta_to_omega(q[-2], q[-1], dt)
    if q.shape[0] > 2:
        omega[1:-1] = delta_to_omega(q[:-2], q[2:], 2.0 * dt)
    return omega


def gradient_velocity(values: np.ndarray, fps: float) -> np.ndarray:
    if values.shape[0] <= 1:
        return np.zeros_like(values)
    edge_order = 2 if values.shape[0] > 2 else 1
    return np.gradient(values.astype(np.float64), 1.0 / fps, axis=0, edge_order=edge_order)


def write_npz_stored_no_zip64(path: Path, arrays: dict[str, np.ndarray]) -> None:
    with zipfile.ZipFile(path, mode="w", compression=zipfile.ZIP_STORED, allowZip64=False) as zf:
        for key, array in arrays.items():
            bio = io.BytesIO()
            np.lib.format.write_array(bio, np.asanyarray(array), allow_pickle=False)
            zf.writestr(f"{key}.npy", bio.getvalue(), compress_type=zipfile.ZIP_STORED)


def prepare_motion_clip(
    input_path: Path,
    output_path: Path,
    start_frame: int,
    end_frame: int | None,
    anchor_body_index: int = 0,
    align_xy_yaw: bool = True,
    recompute_velocities: bool = True,
    force: bool = False,
) -> dict[str, float | int | bool]:
    if output_path.exists() and not force:
        raise FileExistsError(f"output exists: {output_path}")

    with np.load(input_path, allow_pickle=False) as data:
        arrays = {key: data[key] for key in data.files}

    fps = float(np.asarray(arrays["fps"]).reshape(-1)[0])
    frame_count = int(arrays["joint_pos"].shape[0])
    if start_frame < 0 or start_frame >= frame_count:
        raise ValueError(f"start frame {start_frame} is outside [0, {frame_count})")
    if end_frame is None:
        end_frame = frame_count - 1
    if end_frame < start_frame or end_frame >= frame_count:
        raise ValueError(f"end frame {end_frame} is outside [{start_frame}, {frame_count})")
    if anchor_body_index < 0 or anchor_body_index >= arrays["body_pos_w"].shape[1]:
        raise ValueError("anchor body index is outside body_pos_w body dimension")

    out: dict[str, np.ndarray] = {}
    end_exclusive = end_frame + 1
    for key, value in arrays.items():
        if value.shape[:1] == (frame_count,):
            out[key] = np.array(value[start_frame:end_exclusive], copy=True)
        else:
            out[key] = np.array(value, copy=True)

    yaw0 = yaw_from_quat_wxyz(out["body_quat_w"][0, anchor_body_index])
    if align_xy_yaw:
        yaw_inv = -yaw0
        rot = rotation_z(yaw_inv)
        yaw_inv_quat = yaw_quat_wxyz(yaw_inv)

        body_pos = out["body_pos_w"].astype(np.float64)
        origin = body_pos[0, anchor_body_index].copy()
        origin[2] = 0.0
        body_pos = (body_pos - origin) @ rot.T
        out["body_pos_w"] = body_pos.astype(arrays["body_pos_w"].dtype, copy=False)

        body_quat = normalize_quat_wxyz(out["body_quat_w"])
        body_quat = quat_mul_wxyz(yaw_inv_quat, body_quat)
        body_quat = enforce_quat_continuity(body_quat)
        out["body_quat_w"] = body_quat.astype(arrays["body_quat_w"].dtype, copy=False)

    if recompute_velocities:
        if "joint_pos" in out and "joint_vel" in out:
            out["joint_vel"] = gradient_velocity(out["joint_pos"], fps).astype(arrays["joint_vel"].dtype, copy=False)
        if "body_pos_w" in out and "body_lin_vel_w" in out:
            out["body_lin_vel_w"] = gradient_velocity(out["body_pos_w"], fps).astype(
                arrays["body_lin_vel_w"].dtype, copy=False
            )
        if "body_quat_w" in out and "body_ang_vel_w" in out:
            out["body_ang_vel_w"] = angular_velocity_from_quat_wxyz(out["body_quat_w"], fps).astype(
                arrays["body_ang_vel_w"].dtype, copy=False
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    write_npz_stored_no_zip64(output_path, out)

    return {
        "fps": fps,
        "input_frames": frame_count,
        "output_frames": int(out["joint_pos"].shape[0]),
        "start_frame": start_frame,
        "end_frame": end_frame,
        "start_time_sec": start_frame / fps,
        "end_time_sec": end_frame / fps,
        "anchor_body_index": anchor_body_index,
        "align_xy_yaw": align_xy_yaw,
        "recompute_velocities": recompute_velocities,
        "removed_initial_yaw_deg": math.degrees(yaw0) if align_xy_yaw else 0.0,
        "anchor_x_after_alignment": float(out["body_pos_w"][0, anchor_body_index, 0]),
        "anchor_y_after_alignment": float(out["body_pos_w"][0, anchor_body_index, 1]),
        "anchor_yaw_after_alignment_deg": math.degrees(yaw_from_quat_wxyz(out["body_quat_w"][0, anchor_body_index])),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Trim a motion npz, align the new first frame xy/yaw to zero, and recompute velocities."
    )
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--start-frame", type=int)
    group.add_argument("--start-time", type=float, help="Start time in seconds.")
    end_group = parser.add_mutually_exclusive_group()
    end_group.add_argument("--end-frame", type=int, help="Inclusive end frame. Defaults to the last frame.")
    end_group.add_argument("--end-time", type=float, help="Inclusive end time in seconds.")
    parser.add_argument("--anchor-body-index", type=int, default=0)
    parser.add_argument("--no-align", action="store_true", help="Only trim; do not reset xy/yaw.")
    parser.add_argument("--keep-velocities", action="store_true", help="Do not recompute velocity arrays.")
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with np.load(args.input, allow_pickle=False) as data:
        fps = float(np.asarray(data["fps"]).reshape(-1)[0])
        frame_count = int(data["joint_pos"].shape[0])
    start_frame = args.start_frame
    if start_frame is None:
        start_frame = int(round(args.start_time * fps))
    end_frame = args.end_frame
    if end_frame is None and args.end_time is not None:
        end_frame = int(round(args.end_time * fps))
    info = prepare_motion_clip(
        args.input,
        args.output,
        start_frame,
        end_frame,
        anchor_body_index=args.anchor_body_index,
        align_xy_yaw=not args.no_align,
        recompute_velocities=not args.keep_velocities,
        force=args.force,
    )
    print(f"input: {args.input}")
    print(f"output: {args.output}")
    print(f"start_frame: {info['start_frame']}")
    print(f"end_frame: {info['end_frame']}")
    print(f"start_time_sec: {info['start_time_sec']:.6f}")
    print(f"end_time_sec: {info['end_time_sec']:.6f}")
    print(f"frames: {info['input_frames']} -> {info['output_frames']}")
    print(f"anchor_body_index: {info['anchor_body_index']}")
    print(f"align_xy_yaw: {info['align_xy_yaw']}")
    print(f"recompute_velocities: {info['recompute_velocities']}")
    print(f"removed_initial_yaw_deg: {info['removed_initial_yaw_deg']:.6f}")
    print(f"anchor_xy_after_alignment: [{info['anchor_x_after_alignment']}, {info['anchor_y_after_alignment']}]")
    print(f"anchor_yaw_after_alignment_deg: {info['anchor_yaw_after_alignment_deg']:.6f}")


if __name__ == "__main__":
    main()
