#!/usr/bin/env python3
"""Convert OpenTrack ref_data.onnx constants to rl_master runtime .npz."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort


def _outputs_by_name(model_path: Path) -> dict[str, np.ndarray]:
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    feeds: dict[str, np.ndarray] = {}
    if session.get_inputs():
        raise SystemExit(f"{model_path} is expected to have no ONNX inputs")
    names = [out.name for out in session.get_outputs()]
    values = session.run(names, feeds)
    return {name: np.asarray(value, dtype=np.float32) for name, value in zip(names, values)}


def _require(outputs: dict[str, np.ndarray], name: str) -> np.ndarray:
    if name not in outputs:
        available = ", ".join(sorted(outputs))
        raise SystemExit(f"Missing ONNX output '{name}'. Available outputs: {available}")
    return outputs[name]


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Convert OpenTrack deploy/storage/data/<motion>/ref_data.onnx to a "
            "ReferenceMotionProvider-compatible .npz."
        )
    )
    parser.add_argument("input", type=Path, help="OpenTrack ref_data.onnx")
    parser.add_argument("output", type=Path, help="Output .npz path")
    parser.add_argument(
        "--fps",
        type=float,
        default=50.0,
        help="Reference frame rate metadata to store in the .npz",
    )
    parser.add_argument(
        "--qpos-joint-start",
        type=int,
        default=7,
        help="Start index of G1 joint qpos in OpenTrack qpos output",
    )
    parser.add_argument(
        "--qvel-joint-start",
        type=int,
        default=6,
        help="Start index of G1 joint qvel in OpenTrack qvel output",
    )
    parser.add_argument(
        "--joint-count",
        type=int,
        default=29,
        help="Number of G1 joints to export",
    )
    args = parser.parse_args()

    outputs = _outputs_by_name(args.input)
    qpos = _require(outputs, "qpos")
    qvel = _require(outputs, "qvel")
    feet_height = _require(outputs, "feet_height")
    root_height = _require(outputs, "root_height")

    if qpos.ndim != 2 or qvel.ndim != 2:
        raise SystemExit("qpos and qvel must be rank-2 arrays")
    if feet_height.ndim != 2 or feet_height.shape[1] < 4:
        raise SystemExit("feet_height must be a rank-2 array with at least 4 columns")
    if root_height.ndim == 1:
        root_height = root_height[:, None]
    if root_height.ndim != 2 or root_height.shape[1] < 1:
        raise SystemExit("root_height must be a rank-1 or rank-2 array")

    frame_count = qpos.shape[0]
    for name, value in {
        "qvel": qvel,
        "feet_height": feet_height,
        "root_height": root_height,
    }.items():
        if value.shape[0] != frame_count:
            raise SystemExit(
                f"{name} frame count {value.shape[0]} does not match qpos frame count {frame_count}"
            )

    joint_pos_end = args.qpos_joint_start + args.joint_count
    joint_vel_end = args.qvel_joint_start + args.joint_count
    joint_pos = qpos[:, args.qpos_joint_start:joint_pos_end]
    joint_vel = qvel[:, args.qvel_joint_start:joint_vel_end]
    if joint_pos.shape[1] != args.joint_count:
        raise SystemExit(f"qpos does not contain {args.joint_count} joints from index {args.qpos_joint_start}")
    if joint_vel.shape[1] != args.joint_count:
        raise SystemExit(f"qvel does not contain {args.joint_count} joints from index {args.qvel_joint_start}")

    reference_motion = np.concatenate([feet_height[:, :4], root_height[:, :1]], axis=1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        reference_motion=reference_motion.astype(np.float32),
        joint_pos=joint_pos.astype(np.float32),
        joint_vel=joint_vel.astype(np.float32),
        fps=np.asarray([args.fps], dtype=np.float32),
    )
    print(
        f"Wrote {args.output} with {frame_count} frames: "
        f"reference_motion={reference_motion.shape}, joint_pos={joint_pos.shape}, joint_vel={joint_vel.shape}"
    )


if __name__ == "__main__":
    main()
