#!/usr/bin/env python3
"""Export BeyondMimic ONNX reference outputs for joint_motor_test playback.

The generated CSV uses the joint_motor_test file trajectory format:
  time_s, q[0..N-1], dq[0..N-1]

By default the script reads the ONNX reference output order from metadata
`joint_names` and remaps it to metadata `command_joint_names`.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
from typing import Dict, Iterable, List, Optional, Sequence

import numpy as np


def _split_names(raw: str) -> List[str]:
    return [item.strip() for item in str(raw).split(",") if item.strip()]


def _parse_name_list(raw: str) -> List[str]:
    if not raw:
        return []
    path = pathlib.Path(raw).expanduser()
    if path.exists():
        text = path.read_text(encoding="utf-8")
        return [item.strip() for item in text.replace("\n", ",").split(",") if item.strip()]
    return _split_names(raw)


def _metadata(session: object) -> Dict[str, str]:
    return dict(session.get_modelmeta().custom_metadata_map)


def _input_by_name(inputs: Sequence[object], preferred: str) -> Optional[object]:
    for item in inputs:
        if getattr(item, "name", "") == preferred:
            return item
    return None


def _first_float_input(inputs: Sequence[object], exclude: Iterable[str]) -> object:
    excluded = set(exclude)
    for item in inputs:
        name = getattr(item, "name", "")
        if name not in excluded and "float" in str(getattr(item, "type", "")):
            return item
    raise RuntimeError("failed to resolve observation input from ONNX model")


def _dim_value(dim: object, fallback: int) -> int:
    if isinstance(dim, int) and dim > 0:
        return dim
    return fallback


def _resolve_mapping(source_names: Sequence[str], target_names: Sequence[str]) -> List[int]:
    index_by_name = {name: idx for idx, name in enumerate(source_names)}
    missing = [name for name in target_names if name not in index_by_name]
    if missing:
        raise RuntimeError(
            "target joint order contains names missing from source order: "
            + ", ".join(missing)
        )
    return [index_by_name[name] for name in target_names]


def _format_float(value: float, precision: int) -> str:
    if not math.isfinite(value):
        raise RuntimeError(f"non-finite value while writing CSV: {value}")
    return f"{value:.{precision}f}"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export BeyondMimic ONNX joint_pos/joint_vel as a joint_motor_test CSV."
    )
    parser.add_argument("--onnx", required=True, help="Path to BeyondMimic policy.onnx")
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--start-step", type=int, default=50, help="First ONNX time_step to export")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Number of 50 Hz reference frames to export. 0 means motion_num_frames - start_step.",
    )
    parser.add_argument("--source-hz", type=float, default=0.0, help="Reference FPS override. 0 reads ONNX metadata")
    parser.add_argument("--output-hz", type=float, default=500.0, help="CSV playback rate")
    parser.add_argument(
        "--source-order-key",
        default="joint_names",
        help="ONNX metadata key describing joint_pos/joint_vel output order",
    )
    parser.add_argument(
        "--target-order-key",
        default="command_joint_names",
        help="ONNX metadata key for desired CSV joint order",
    )
    parser.add_argument(
        "--target-joint-order",
        default="",
        help="Comma-separated names or a text file. Overrides --target-order-key.",
    )
    parser.add_argument("--obs-input-name", default="obs", help="ONNX observation input name")
    parser.add_argument("--time-step-input-name", default="time_step", help="ONNX time step input name")
    parser.add_argument("--joint-pos-output-name", default="joint_pos", help="ONNX joint position output")
    parser.add_argument("--joint-vel-output-name", default="joint_vel", help="ONNX joint velocity output")
    parser.add_argument(
        "--zero-dq",
        action="store_true",
        help="Write zero target velocity instead of ONNX joint_vel for pure position-hold PD checks.",
    )
    parser.add_argument("--precision", type=int, default=9, help="Decimal places in output CSV")

    args = parser.parse_args()

    import onnxruntime as ort  # type: ignore

    onnx_path = pathlib.Path(args.onnx).expanduser().resolve()
    output_path = pathlib.Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    meta = _metadata(session)

    source_names = _split_names(meta.get(args.source_order_key, ""))
    if not source_names:
        raise RuntimeError(f"ONNX metadata '{args.source_order_key}' is missing or empty")

    target_names = _parse_name_list(args.target_joint_order)
    if not target_names:
        target_names = _split_names(meta.get(args.target_order_key, ""))
    if not target_names:
        target_names = list(source_names)

    remap = _resolve_mapping(source_names, target_names)

    source_hz = args.source_hz
    if source_hz <= 0.0:
        source_hz = float(meta.get("motion_fps") or (1.0 / float(meta.get("control_dt", 0.02))))
    if source_hz <= 0.0:
        raise RuntimeError(f"invalid source_hz={source_hz}")
    if args.output_hz <= 0.0:
        raise RuntimeError(f"invalid output_hz={args.output_hz}")

    hold_ratio_f = args.output_hz / source_hz
    hold_ratio = int(round(hold_ratio_f))
    if hold_ratio < 1 or abs(hold_ratio_f - hold_ratio) > 1e-6:
        raise RuntimeError(
            f"output_hz/source_hz must be an integer hold ratio, got {args.output_hz}/{source_hz}"
        )

    motion_num_frames = int(float(meta.get("motion_num_frames", "0") or 0))
    frames = args.frames
    if frames <= 0:
        frames = max(1, motion_num_frames - args.start_step) if motion_num_frames > 0 else 500
    if args.start_step < 0:
        raise RuntimeError("--start-step must be >= 0")

    inputs = session.get_inputs()
    obs_input = _input_by_name(inputs, args.obs_input_name) or _first_float_input(inputs, [args.time_step_input_name])
    time_input = _input_by_name(inputs, args.time_step_input_name)
    if time_input is None:
        raise RuntimeError(f"failed to find ONNX time step input '{args.time_step_input_name}'")

    obs_shape = list(getattr(obs_input, "shape", []))
    obs_dim = _dim_value(obs_shape[-1] if obs_shape else 75, 75)
    obs_name = getattr(obs_input, "name")
    time_name = getattr(time_input, "name")

    obs = np.zeros((1, obs_dim), dtype=np.float32)

    output_names = [args.joint_pos_output_name, args.joint_vel_output_name]
    header = ["time_s"] + [f"q:{name}" for name in target_names] + [f"dq:{name}" for name in target_names]

    with output_path.open("w", newline="", encoding="utf-8") as f:
        f.write(f"# source_onnx={onnx_path}\n")
        f.write(f"# source_order={','.join(source_names)}\n")
        f.write(f"# target_order={','.join(target_names)}\n")
        f.write(
            f"# start_step={args.start_step} source_hz={source_hz:.9f} "
            f"output_hz={args.output_hz:.9f} hold_ratio={hold_ratio}\n"
        )
        writer = csv.writer(f)
        writer.writerow(header)

        out_step = 0
        for frame_idx in range(frames):
            time_step = np.array([[float(args.start_step + frame_idx)]], dtype=np.float32)
            joint_pos, joint_vel = session.run(output_names, {obs_name: obs, time_name: time_step})
            q = np.asarray(joint_pos, dtype=np.float64).reshape(-1)
            dq = np.asarray(joint_vel, dtype=np.float64).reshape(-1)
            if len(q) != len(source_names) or len(dq) != len(source_names):
                raise RuntimeError(
                    f"ONNX output size mismatch at frame {frame_idx}: "
                    f"joint_pos={len(q)} joint_vel={len(dq)} source_names={len(source_names)}"
                )
            q_target = q[remap]
            dq_target = np.zeros_like(q_target) if args.zero_dq else dq[remap]

            for _ in range(hold_ratio):
                t = out_step / args.output_hz
                row = [_format_float(t, 6)]
                row.extend(_format_float(float(v), args.precision) for v in q_target)
                row.extend(_format_float(float(v), args.precision) for v in dq_target)
                writer.writerow(row)
                out_step += 1

    print(
        f"wrote {frames * hold_ratio} rows to {output_path} "
        f"({frames} source frames, {len(target_names)} joints, target order: {','.join(target_names)})"
    )


if __name__ == "__main__":
    main()
