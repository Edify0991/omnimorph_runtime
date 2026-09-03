#!/usr/bin/env python3
"""Extract one aligned sim2sim policy-running clip from a runtime MCAP log.

The output MCAP has one JSON message per low-level control frame on
``runtime/processed/policy_running_frame``.  Each message preserves the full
``runtime/tick`` payload and adds the nearest timestamped base IMU, base state,
and foot-force source samples.  Foot contact labels are derived from the norm
of each local-frame foot force vector.

The selected interval starts at the first inference tick while the deploy state
is RUNNING and ends at the final tick that advances ``policy_step_index``.  It
therefore excludes the later control frames that keep executing after a finite
policy clip has reached its final timestep.
"""

from __future__ import annotations

import argparse
import bisect
import json
import struct
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import zstandard

from runtime_log_utils import MAGIC, load_runtime_messages


RUNNING_DEPLOY_STATE = 3
OUTPUT_TOPIC = "runtime/processed/policy_running_frame"


def _encode_string(value: str) -> bytes:
    encoded = value.encode("utf-8")
    return struct.pack("<I", len(encoded)) + encoded


def _encode_string_map(values: Dict[str, str]) -> bytes:
    payload = b"".join(_encode_string(key) + _encode_string(value) for key, value in values.items())
    return struct.pack("<I", len(payload)) + payload


class JsonMcapWriter:
    """Small standards-compatible JSON MCAP writer for processed artifacts.

    The in-tree runtime reader supports this chunked MCAP subset. Keeping the
    processed data in MCAP lets existing analysis commands inspect it without
    installing another Python package.
    """

    def __init__(self, path: Path):
        self._path = path
        self._file = path.open("wb")
        self._sequence = 0
        self._channel_id = 1
        self._chunk_records = bytearray()
        self._chunk_start_time_ns: Optional[int] = None
        self._chunk_end_time_ns: Optional[int] = None
        self._target_chunk_size_bytes = 1024 * 1024
        self._compressor = zstandard.ZstdCompressor(level=3)
        self._write(MAGIC)
        self._write_record(0x01, _encode_string("") + _encode_string("rl_master_policy_clip_extractor"))
        channel_payload = (
            struct.pack("<HH", self._channel_id, 0)
            + _encode_string(OUTPUT_TOPIC)
            + _encode_string("json")
            + _encode_string_map({"profile": "processed_policy_running_frames"})
        )
        self._write_record(0x04, channel_payload)

    def _write(self, payload: bytes) -> None:
        self._file.write(payload)

    def _write_record(self, opcode: int, payload: bytes) -> None:
        self._write(bytes([opcode]) + struct.pack("<Q", len(payload)) + payload)

    @staticmethod
    def _record(opcode: int, payload: bytes) -> bytes:
        return bytes([opcode]) + struct.pack("<Q", len(payload)) + payload

    def _flush_chunk(self) -> None:
        if not self._chunk_records:
            return
        compressed = self._compressor.compress(bytes(self._chunk_records))
        payload = (
            struct.pack(
                "<QQQI",
                int(self._chunk_start_time_ns),
                int(self._chunk_end_time_ns),
                len(self._chunk_records),
                0,
            )
            + _encode_string("zstd")
            + compressed
        )
        self._write_record(0x06, payload)
        self._chunk_records.clear()
        self._chunk_start_time_ns = None
        self._chunk_end_time_ns = None

    def write_message(self, payload: Dict[str, Any], log_time_ns: int, publish_time_ns: int) -> None:
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode("utf-8")
        message_payload = (
            struct.pack("<HIQQ", self._channel_id, self._sequence, log_time_ns, publish_time_ns) + encoded
        )
        record = self._record(0x05, message_payload)
        if self._chunk_records and len(self._chunk_records) + len(record) > self._target_chunk_size_bytes:
            self._flush_chunk()
        self._chunk_records.extend(record)
        self._chunk_start_time_ns = (
            log_time_ns if self._chunk_start_time_ns is None else min(self._chunk_start_time_ns, log_time_ns)
        )
        self._chunk_end_time_ns = (
            log_time_ns if self._chunk_end_time_ns is None else max(self._chunk_end_time_ns, log_time_ns)
        )
        self._sequence += 1

    def close(self) -> None:
        if self._file.closed:
            return
        self._flush_chunk()
        self._write_record(0x0F, struct.pack("<I", 0))
        self._write_record(0x02, struct.pack("<QQI", 0, 0, 0))
        self._write(MAGIC)
        self._file.close()

    def __enter__(self) -> "JsonMcapWriter":
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()


def _payload(message: Dict[str, Any]) -> Dict[str, Any]:
    payload = message.get("data")
    if not isinstance(payload, dict):
        raise RuntimeError(f"Expected JSON object payload on topic {message.get('topic')!r}")
    return payload


def _find_policy_clip_indices(ticks: Sequence[Dict[str, Any]]) -> Tuple[int, int, int, int]:
    first_index: Optional[int] = None
    last_index: Optional[int] = None
    first_policy_step: Optional[int] = None
    last_policy_step: Optional[int] = None

    for index, message in enumerate(ticks):
        tick = _payload(message)
        if tick.get("deploy_state") != RUNNING_DEPLOY_STATE or not tick.get("policy_ran_this_tick", False):
            continue
        policy_step = int(tick.get("policy_step_index", 0))
        if first_index is None:
            first_index = index
            first_policy_step = policy_step
            last_index = index
            last_policy_step = policy_step
            continue
        if policy_step > int(last_policy_step):
            last_index = index
            last_policy_step = policy_step

    if first_index is None or last_index is None or first_policy_step is None or last_policy_step is None:
        raise RuntimeError("No RUNNING policy inference frames found in runtime/tick")
    if last_policy_step <= first_policy_step:
        raise RuntimeError("policy_step_index did not advance after the first RUNNING inference frame")
    return first_index, last_index, first_policy_step, last_policy_step


def _nearest_source(
    messages: Sequence[Dict[str, Any]], timestamps_ns: Sequence[int], target_time_ns: int
) -> Optional[Dict[str, Any]]:
    if not messages:
        return None
    pos = bisect.bisect_left(timestamps_ns, target_time_ns)
    candidates: List[int] = []
    if pos < len(messages):
        candidates.append(pos)
    if pos > 0:
        candidates.append(pos - 1)
    source_index = min(candidates, key=lambda index: abs(timestamps_ns[index] - target_time_ns))
    source = messages[source_index]
    return {
        "log_time_ns": int(source["log_time_ns"]),
        "publish_time_ns": int(source["publish_time_ns"]),
        "offset_ns": int(source["log_time_ns"]) - target_time_ns,
        "sample": _payload(source),
    }


def _force_vector(source: Optional[Dict[str, Any]], sensor_name: str) -> Optional[List[float]]:
    if source is None:
        return None
    values = source["sample"].get("values")
    if not isinstance(values, dict):
        return None
    raw = values.get(sensor_name)
    if not isinstance(raw, list) or len(raw) != 3:
        return None
    try:
        return [float(raw[0]), float(raw[1]), float(raw[2])]
    except (TypeError, ValueError):
        return None


def _contact_label(vector: Optional[Sequence[float]], threshold_n: float) -> Dict[str, Any]:
    if vector is None:
        return {"valid": False, "force_norm_n": None, "in_contact": None}
    norm_n = sum(component * component for component in vector) ** 0.5
    return {
        "valid": True,
        "force_norm_n": norm_n,
        "in_contact": norm_n >= threshold_n,
    }


def _source_messages(input_path: Path, topic: str) -> Tuple[List[Dict[str, Any]], List[int]]:
    messages = load_runtime_messages(input_path, topic)
    messages.sort(key=lambda message: int(message["log_time_ns"]))
    return messages, [int(message["log_time_ns"]) for message in messages]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mcap", type=Path, required=True, help="Input runtime MCAP.")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output MCAP. Default: <input_stem>_policy_running_frames.mcap next to the input.",
    )
    parser.add_argument("--contact-force-threshold", type=float, default=20.0, help="Contact threshold on force norm, in N.")
    parser.add_argument("--left-force-sensor", default="left_foot_sensor", help="Left force vector key in foot_force_sensors values.")
    parser.add_argument("--right-force-sensor", default="right_foot_sensor", help="Right force vector key in foot_force_sensors values.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = args.mcap.resolve()
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    if args.contact_force_threshold < 0.0:
        raise ValueError("--contact-force-threshold must be non-negative")

    output_path = (
        args.output.resolve()
        if args.output
        else input_path.with_name(f"{input_path.stem}_policy_running_frames.mcap")
    )
    if output_path == input_path:
        raise ValueError("--output must differ from --mcap")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    ticks = load_runtime_messages(input_path, "runtime/tick")
    ticks.sort(key=lambda message: int(message["log_time_ns"]))
    start_index, end_index, first_policy_step, last_policy_step = _find_policy_clip_indices(ticks)
    selected_ticks = ticks[start_index : end_index + 1]

    source_data = {}
    for source_name, topic in (
        ("base_imu", "runtime/source/base_imu"),
        ("base_state", "runtime/source/base_state"),
        ("foot_force_sensors", "runtime/source/foot_force_sensors"),
    ):
        source_data[source_name] = _source_messages(input_path, topic)

    offset_abs_ns: Dict[str, List[int]] = {name: [] for name in source_data}
    contact_counts = {"left": 0, "right": 0, "double_support": 0}
    with JsonMcapWriter(output_path) as writer:
        for message in selected_ticks:
            tick_time_ns = int(message["log_time_ns"])
            sources = {}
            for source_name, (source_messages, source_times) in source_data.items():
                aligned = _nearest_source(source_messages, source_times, tick_time_ns)
                sources[source_name] = aligned
                if aligned is not None:
                    offset_abs_ns[source_name].append(abs(int(aligned["offset_ns"])))

            foot_source = sources["foot_force_sensors"]
            left_vector = _force_vector(foot_source, args.left_force_sensor)
            right_vector = _force_vector(foot_source, args.right_force_sensor)
            left_contact = _contact_label(left_vector, args.contact_force_threshold)
            right_contact = _contact_label(right_vector, args.contact_force_threshold)
            if left_contact["in_contact"]:
                contact_counts["left"] += 1
            if right_contact["in_contact"]:
                contact_counts["right"] += 1
            if left_contact["in_contact"] and right_contact["in_contact"]:
                contact_counts["double_support"] += 1

            tick = _payload(message)
            frame = {
                "frame_index": tick.get("frame_index"),
                "control_tick_log_time_ns": tick_time_ns,
                "control_tick_publish_time_ns": int(message["publish_time_ns"]),
                "tick": tick,
                "sources": sources,
                "foot_contact": {
                    "criterion": "norm(sensor_local_force_N) >= threshold_n",
                    "threshold_n": args.contact_force_threshold,
                    "left_sensor": args.left_force_sensor,
                    "right_sensor": args.right_force_sensor,
                    "left": {"force_local_n": left_vector, **left_contact},
                    "right": {"force_local_n": right_vector, **right_contact},
                },
            }
            writer.write_message(frame, tick_time_ns, int(message["publish_time_ns"]))

    summary = {
        "input_mcap": str(input_path),
        "output_mcap": str(output_path),
        "output_topic": OUTPUT_TOPIC,
        "selection": {
            "deploy_state": "RUNNING (3)",
            "start_tick_message_index": start_index,
            "end_tick_message_index": end_index,
            "start_frame_index": _payload(selected_ticks[0]).get("frame_index"),
            "end_frame_index": _payload(selected_ticks[-1]).get("frame_index"),
            "frame_count": len(selected_ticks),
            "first_policy_step_index": first_policy_step,
            "last_policy_step_index": last_policy_step,
            "start_log_time_ns": int(selected_ticks[0]["log_time_ns"]),
            "end_log_time_ns": int(selected_ticks[-1]["log_time_ns"]),
        },
        "foot_contact": {
            "criterion": "norm(sensor_local_force_N) >= threshold_n",
            "threshold_n": args.contact_force_threshold,
            "left_sensor": args.left_force_sensor,
            "right_sensor": args.right_force_sensor,
            "frame_counts": contact_counts,
        },
        "source_alignment": {
            name: {
                "source_message_count": len(source_data[name][0]),
                "aligned_frame_count": len(offsets),
                "max_abs_offset_ns": max(offsets) if offsets else None,
                "mean_abs_offset_ns": sum(offsets) / len(offsets) if offsets else None,
            }
            for name, offsets in offset_abs_ns.items()
        },
    }
    summary_path = output_path.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"Wrote {len(selected_ticks)} processed control frames to: {output_path}")
    print(f"Wrote extraction summary to: {summary_path}")
    print(
        "Policy steps: "
        f"{first_policy_step}..{last_policy_step}; control frames: "
        f"{summary['selection']['start_frame_index']}..{summary['selection']['end_frame_index']}"
    )


if __name__ == "__main__":
    main()
