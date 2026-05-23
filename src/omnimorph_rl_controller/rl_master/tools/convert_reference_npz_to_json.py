#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from typing import List

import numpy as np


def _load_string_list(data: np.lib.npyio.NpzFile, key: str) -> List[str]:
    if key not in data:
        return []
    values = data[key]
    return [str(v) for v in values.tolist()]


def _flatten_frame(frame: np.ndarray) -> List[float]:
    return [float(v) for v in np.asarray(frame, dtype=np.float32).reshape(-1).tolist()]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Convert a BeyondMimic/Isaac-style reference NPZ into a structured JSON file "
        "that RL runtime ReferenceMotionProvider can load with reference_motion_source=file."
    )
    parser.add_argument("--input", required=True, help="Input reference .npz path")
    parser.add_argument("--output", required=True, help="Output structured .json path")
    parser.add_argument("--anchor-body", default="", help="Anchor body name to store in metadata")
    parser.add_argument(
        "--quat-order",
        default="wxyz",
        choices=["wxyz", "xyzw"],
        help="Quaternion order stored in body_quat_w arrays inside the NPZ",
    )
    parser.add_argument(
        "--include-reference-motion",
        action="store_true",
        help="Also write concatenated reference_motion = joint_pos + joint_vel for each frame",
    )
    args = parser.parse_args()

    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    data = np.load(input_path, allow_pickle=True)

    joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
    joint_vel = np.asarray(data["joint_vel"], dtype=np.float32)
    body_pos_w = np.asarray(data["body_pos_w"], dtype=np.float32)
    body_quat_w = np.asarray(data["body_quat_w"], dtype=np.float32)

    if joint_pos.ndim != 2 or joint_vel.ndim != 2:
        raise ValueError("joint_pos/joint_vel must be rank-2 arrays")
    if body_pos_w.ndim != 3 or body_pos_w.shape[2] != 3:
        raise ValueError("body_pos_w must have shape [T, B, 3]")
    if body_quat_w.ndim != 3 or body_quat_w.shape[2] != 4:
        raise ValueError("body_quat_w must have shape [T, B, 4]")

    frame_count = int(joint_pos.shape[0])
    if joint_vel.shape[0] != frame_count or body_pos_w.shape[0] != frame_count or body_quat_w.shape[0] != frame_count:
        raise ValueError("all reference arrays must share the same frame count")

    fps_raw = np.asarray(data["fps"]).reshape(-1)
    fps = float(fps_raw[0]) if fps_raw.size > 0 else 0.0
    body_names = _load_string_list(data, "body_names")
    joint_names = _load_string_list(data, "joint_names")
    source_joint_names = _load_string_list(data, "source_joint_names")

    frames = []
    for i in range(frame_count):
        frame = {
            "joint_pos": _flatten_frame(joint_pos[i]),
            "joint_vel": _flatten_frame(joint_vel[i]),
            "body_pos_w": _flatten_frame(body_pos_w[i]),
            "body_quat_w": _flatten_frame(body_quat_w[i]),
        }
        if args.include_reference_motion:
            frame["reference_motion"] = frame["joint_pos"] + frame["joint_vel"]
        frames.append(frame)

    payload = {
        "reference_motion": {
            "source_format": "reference_npz_json_v1",
            "source_npz": str(input_path),
            "anchor_body": args.anchor_body,
            "body_names": body_names,
            "joint_names": joint_names,
            "source_joint_names": source_joint_names,
            "body_quat_format": args.quat_order,
            "fps": fps,
            "frame_dt": (1.0 / fps) if fps > 1.0e-9 else 0.0,
            "frames": frames,
        }
    }

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=True, separators=(",", ":"))

    print(f"Wrote structured reference file: {output_path}")
    print(f"Frames: {frame_count}, joints: {joint_pos.shape[1]}, bodies: {body_pos_w.shape[1]}, fps: {fps}")
    if joint_names:
        print("Saved joint_names order:")
        print(",".join(joint_names))
    if source_joint_names:
        print("Source joint_names order:")
        print(",".join(source_joint_names))


if __name__ == "__main__":
    main()
