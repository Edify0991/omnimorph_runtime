#!/usr/bin/env python3
"""Convert official SONIC motion assets into rl_master runtime reference npz."""

from __future__ import annotations

import argparse
import csv
import io
import pickle
import re
import zipfile
from pathlib import Path
from typing import Any

import numpy as np


REQUIRED_KEYS = ("joint_pos", "joint_vel", "body_pos_w", "body_quat_w")


def write_npz_stored(path: Path, arrays: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, mode="w", compression=zipfile.ZIP_STORED, allowZip64=False) as archive:
        for key, value in arrays.items():
            buffer = io.BytesIO()
            np.lib.format.write_array(buffer, np.asanyarray(value), allow_pickle=False)
            archive.writestr(f"{key}.npy", buffer.getvalue(), compress_type=zipfile.ZIP_STORED)


def load_pickle_like(path: Path) -> Any:
    try:
        import joblib  # type: ignore

        return joblib.load(path)
    except ImportError:
        pass
    except Exception:
        pass

    with path.open("rb") as stream:
        return pickle.load(stream)


def coerce_motion_dict(raw: Any, motion_name: str | None) -> tuple[str, dict[str, Any]]:
    if not isinstance(raw, dict):
        raise ValueError(f"expected dict-like motion data, got {type(raw)!r}")

    if all(key in raw for key in REQUIRED_KEYS):
        return (motion_name or "motion", raw)

    if motion_name:
        if motion_name not in raw:
            available = ", ".join(sorted(str(key) for key in raw.keys()))
            raise KeyError(f"motion '{motion_name}' not found. available: {available}")
        motion = raw[motion_name]
        if not isinstance(motion, dict):
            raise ValueError(f"motion '{motion_name}' is not a dict")
        return motion_name, motion

    if len(raw) == 1:
        key, motion = next(iter(raw.items()))
        if not isinstance(motion, dict):
            raise ValueError(f"single motion entry '{key}' is not a dict")
        return str(key), motion

    available = ", ".join(sorted(str(key) for key in raw.keys()))
    raise ValueError(
        "pickle contains multiple motions; pass --motion-name. "
        f"available: {available}"
    )


def load_csv_matrix(path: Path) -> np.ndarray:
    with path.open("r", newline="") as stream:
        reader = csv.reader(stream)
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty csv: {path}")
    if rows[0] and rows[0][0].startswith("version https://git-lfs.github.com/spec/v1"):
        raise ValueError(
            f"{path} is a Git LFS pointer, not the real motion csv. "
            "Run 'git lfs pull' in the SONIC repository first."
        )
    data_rows = rows[1:] if rows[0] and not _row_is_numeric(rows[0]) else rows
    if not data_rows:
        raise ValueError(f"no data rows in csv: {path}")
    return np.asarray([[float(item) for item in row] for row in data_rows], dtype=np.float32)


def _row_is_numeric(row: list[str]) -> bool:
    try:
        for item in row:
            float(item)
        return True
    except ValueError:
        return False


def infer_fps_from_metadata(metadata_path: Path) -> float | None:
    if not metadata_path.exists():
        return None
    text = metadata_path.read_text(encoding="utf-8", errors="ignore")
    match = re.search(r"fps\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)", text, flags=re.IGNORECASE)
    if match:
        return float(match.group(1))
    return None


def load_motion_dir(path: Path, fps_override: float | None) -> tuple[str, dict[str, np.ndarray], float]:
    joint_pos = load_csv_matrix(path / "joint_pos.csv")
    joint_vel = load_csv_matrix(path / "joint_vel.csv")
    body_pos_flat = load_csv_matrix(path / "body_pos.csv")
    body_quat_flat = load_csv_matrix(path / "body_quat.csv")

    if joint_pos.ndim != 2 or joint_pos.shape[1] != 29:
        raise ValueError(f"joint_pos.csv must be [T,29], got {joint_pos.shape}")
    if joint_vel.shape != joint_pos.shape:
        raise ValueError(f"joint_vel.csv shape mismatch: {joint_vel.shape} vs {joint_pos.shape}")
    if body_pos_flat.shape[0] != joint_pos.shape[0] or body_pos_flat.shape[1] % 3 != 0:
        raise ValueError(f"body_pos.csv must be [T,B*3], got {body_pos_flat.shape}")
    if body_quat_flat.shape[0] != joint_pos.shape[0] or body_quat_flat.shape[1] % 4 != 0:
        raise ValueError(f"body_quat.csv must be [T,B*4], got {body_quat_flat.shape}")

    body_count_pos = body_pos_flat.shape[1] // 3
    body_count_quat = body_quat_flat.shape[1] // 4
    if body_count_pos != body_count_quat:
        raise ValueError(
            f"body count mismatch between body_pos.csv ({body_count_pos}) and body_quat.csv ({body_count_quat})"
        )

    fps = fps_override if fps_override is not None else infer_fps_from_metadata(path / "metadata.txt")
    if fps is None:
        fps = 50.0

    arrays = {
        "joint_pos": joint_pos.astype(np.float32, copy=False),
        "joint_vel": joint_vel.astype(np.float32, copy=False),
        "body_pos_w": body_pos_flat.reshape(joint_pos.shape[0], body_count_pos, 3).astype(np.float32, copy=False),
        "body_quat_w": body_quat_flat.reshape(joint_pos.shape[0], body_count_quat, 4).astype(np.float32, copy=False),
    }
    return path.name, arrays, float(fps)


def normalize_motion_arrays(motion: dict[str, Any]) -> dict[str, np.ndarray]:
    arrays: dict[str, np.ndarray] = {}
    for key in REQUIRED_KEYS:
        if key not in motion:
            raise KeyError(f"motion is missing required key: {key}")
        arrays[key] = np.asarray(motion[key], dtype=np.float32)

    joint_pos = arrays["joint_pos"]
    joint_vel = arrays["joint_vel"]
    body_pos_w = arrays["body_pos_w"]
    body_quat_w = arrays["body_quat_w"]

    if joint_pos.ndim != 2 or joint_pos.shape[1] != 29:
        raise ValueError(f"joint_pos must be [T,29], got {joint_pos.shape}")
    if joint_vel.shape != joint_pos.shape:
        raise ValueError(f"joint_vel shape mismatch: {joint_vel.shape} vs {joint_pos.shape}")
    if body_pos_w.ndim != 3 or body_pos_w.shape[0] != joint_pos.shape[0] or body_pos_w.shape[2] != 3:
        raise ValueError(f"body_pos_w must be [T,B,3], got {body_pos_w.shape}")
    if body_quat_w.ndim != 3 or body_quat_w.shape[0] != joint_pos.shape[0] or body_quat_w.shape[2] != 4:
        raise ValueError(f"body_quat_w must be [T,B,4], got {body_quat_w.shape}")
    if body_pos_w.shape[1] != body_quat_w.shape[1]:
        raise ValueError(
            f"body_pos_w/body_quat_w body count mismatch: {body_pos_w.shape[1]} vs {body_quat_w.shape[1]}"
        )
    return arrays


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="SONIC motion directory or .pkl/.joblib file")
    parser.add_argument("--output", required=True, type=Path, help="Runtime .npz output")
    parser.add_argument("--motion-name", default="", help="Motion name inside a multi-motion pickle")
    parser.add_argument("--fps", type=float, default=None, help="Override fps metadata (default: 50 for csv dirs)")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    if args.output.exists() and not args.force:
        raise FileExistsError(f"output exists: {args.output}")
    if args.fps is not None and args.fps <= 0.0:
        raise ValueError("--fps must be positive")

    if args.input.is_dir():
        motion_name, arrays, fps = load_motion_dir(args.input, args.fps)
    else:
        raw = load_pickle_like(args.input)
        motion_name, motion = coerce_motion_dict(raw, args.motion_name or None)
        arrays = normalize_motion_arrays(motion)
        fps = float(args.fps if args.fps is not None else motion.get("fps", 50.0))

    write_npz_stored(
        args.output,
        {
            "joint_pos": arrays["joint_pos"],
            "joint_vel": arrays["joint_vel"],
            "body_pos_w": arrays["body_pos_w"],
            "body_quat_w": arrays["body_quat_w"],
            "fps": np.asarray([fps], dtype=np.float32),
        },
    )

    print(f"motion: {motion_name}")
    print(f"input: {args.input}")
    print(f"output: {args.output}")
    print(f"fps: {fps:.6f}")
    print(f"frames: {arrays['joint_pos'].shape[0]}")
    print(f"bodies: {arrays['body_pos_w'].shape[1]}")


if __name__ == "__main__":
    main()
