#!/usr/bin/env python3
"""Analyze rl_master runtime MCAP logs."""

from __future__ import annotations

import argparse
import bisect
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

from runtime_log_utils import load_runtime_messages, load_runtime_metadata


def parse_indices(text: str) -> List[int]:
    if not text:
        return []
    return [int(token.strip()) for token in text.split(",") if token.strip()]


def collect_series(messages: Sequence[Dict], field: str):
    timestamps_ms: List[float] = []
    values: List[List[float]] = []
    available_fields = set()

    for message in messages:
        payload = message.get("data")
        if not isinstance(payload, dict):
            continue
        available_fields.update(payload.keys())
        if field not in payload:
            continue
        value = payload[field]
        if not isinstance(value, list):
            continue
        timestamps_ms.append(float(message["log_time_ns"]) / 1.0e6)
        values.append([float(x) for x in value])

    return timestamps_ms, values, sorted(available_fields)


def summarize_vector_series(timestamps_ms: Sequence[float], values: Sequence[Sequence[float]]) -> Dict[str, float]:
    if not timestamps_ms or not values:
        return {}
    flattened = [x for row in values for x in row]
    if not flattened:
        return {}
    stats: Dict[str, float] = {
        "sample_count": float(len(values)),
        "vector_dim": float(len(values[0])),
        "time_start_ms": timestamps_ms[0],
        "time_end_ms": timestamps_ms[-1],
        "value_min": min(flattened),
        "value_max": max(flattened),
        "value_mean": statistics.mean(flattened),
    }
    stats.update(summarize_timestamps(timestamps_ms))
    return stats


def maybe_plot(
    timestamps_ms: Sequence[float],
    values: Sequence[Sequence[float]],
    indices: Sequence[int],
    title: str,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - optional dependency
        raise RuntimeError("matplotlib is required for --plot") from exc

    if not values:
        raise RuntimeError("No values to plot")

    if not indices:
        indices = list(range(min(4, len(values[0]))))

    plt.figure(figsize=(12, 5))
    for idx in indices:
        ys = [row[idx] for row in values if idx < len(row)]
        if ys:
            plt.plot(timestamps_ms[: len(ys)], ys, label=f"idx={idx}")
    plt.xlabel("timestamp (ms)")
    plt.ylabel("value")
    plt.title(title)
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.show()


def percentile(sorted_values: Sequence[float], q: float) -> float:
    if not sorted_values:
        return float("nan")
    if len(sorted_values) == 1:
        return float(sorted_values[0])
    q = min(max(q, 0.0), 1.0)
    pos = q * (len(sorted_values) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return float(sorted_values[lo])
    frac = pos - lo
    return float(sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac)


def summarize_timestamps(timestamps_ms: Sequence[float]) -> Dict[str, float]:
    if not timestamps_ms:
        return {}
    stats: Dict[str, float] = {
        "sample_count": float(len(timestamps_ms)),
        "time_start_ms": float(timestamps_ms[0]),
        "time_end_ms": float(timestamps_ms[-1]),
    }
    if len(timestamps_ms) < 2:
        return stats

    deltas = [
        timestamps_ms[i + 1] - timestamps_ms[i]
        for i in range(len(timestamps_ms) - 1)
        if timestamps_ms[i + 1] > timestamps_ms[i]
    ]
    if not deltas:
        return stats

    deltas_sorted = sorted(deltas)
    dt_mean = statistics.mean(deltas)
    dt_std = statistics.pstdev(deltas) if len(deltas) > 1 else 0.0
    stats.update(
        {
            "dt_mean_ms": dt_mean,
            "dt_std_ms": dt_std,
            "dt_min_ms": min(deltas),
            "dt_max_ms": max(deltas),
            "dt_p50_ms": percentile(deltas_sorted, 0.50),
            "dt_p90_ms": percentile(deltas_sorted, 0.90),
            "dt_p99_ms": percentile(deltas_sorted, 0.99),
            "avg_hz": 1000.0 / dt_mean if dt_mean > 1.0e-9 else float("inf"),
        }
    )
    return stats


def topic_timestamps(messages: Sequence[Dict]) -> List[float]:
    return [float(message["log_time_ns"]) / 1.0e6 for message in messages]


def nearest_offset_stats(topic_ts_ms: Sequence[float], ref_ts_ms: Sequence[float]) -> Dict[str, float]:
    if not topic_ts_ms or not ref_ts_ms:
        return {}
    offsets: List[float] = []
    for timestamp in topic_ts_ms:
        pos = bisect.bisect_left(ref_ts_ms, timestamp)
        candidates: List[float] = []
        if pos < len(ref_ts_ms):
            candidates.append(abs(ref_ts_ms[pos] - timestamp))
        if pos > 0:
            candidates.append(abs(ref_ts_ms[pos - 1] - timestamp))
        if candidates:
            offsets.append(min(candidates))
    if not offsets:
        return {}
    offsets_sorted = sorted(offsets)
    return {
        "nearest_offset_mean_ms": statistics.mean(offsets),
        "nearest_offset_max_ms": max(offsets),
        "nearest_offset_p50_ms": percentile(offsets_sorted, 0.50),
        "nearest_offset_p90_ms": percentile(offsets_sorted, 0.90),
        "nearest_offset_p99_ms": percentile(offsets_sorted, 0.99),
    }


def hold_alignment_stats(source_ts_ms: Sequence[float], ref_ts_ms: Sequence[float]) -> Dict[str, float]:
    if not source_ts_ms or not ref_ts_ms:
        return {}

    ages_ms: List[float] = []
    reused_run_lengths: List[int] = []
    last_source_idx = -1
    current_run = 0
    source_idx = -1
    for ref_time in ref_ts_ms:
        while (source_idx + 1) < len(source_ts_ms) and source_ts_ms[source_idx + 1] <= ref_time:
            source_idx += 1
        if source_idx < 0:
            continue
        ages_ms.append(ref_time - source_ts_ms[source_idx])
        if source_idx == last_source_idx:
            current_run += 1
        else:
            if current_run > 0:
                reused_run_lengths.append(current_run)
            current_run = 1
            last_source_idx = source_idx
    if current_run > 0:
        reused_run_lengths.append(current_run)

    if not ages_ms:
        return {
            "covered_ref_samples": 0.0,
            "ref_sample_count": float(len(ref_ts_ms)),
        }

    ages_sorted = sorted(ages_ms)
    run_lengths_sorted = sorted(float(x) for x in reused_run_lengths)
    return {
        "covered_ref_samples": float(len(ages_ms)),
        "ref_sample_count": float(len(ref_ts_ms)),
        "hold_age_mean_ms": statistics.mean(ages_ms),
        "hold_age_max_ms": max(ages_ms),
        "hold_age_p50_ms": percentile(ages_sorted, 0.50),
        "hold_age_p90_ms": percentile(ages_sorted, 0.90),
        "hold_age_p99_ms": percentile(ages_sorted, 0.99),
        "reuse_run_mean_ticks": statistics.mean(reused_run_lengths),
        "reuse_run_max_ticks": float(max(reused_run_lengths)),
        "reuse_run_p50_ticks": percentile(run_lengths_sorted, 0.50),
        "reuse_run_p90_ticks": percentile(run_lengths_sorted, 0.90),
        "reuse_run_p99_ticks": percentile(run_lengths_sorted, 0.99),
    }


def print_stats_block(title: str, stats: Dict[str, float]) -> None:
    print(title)
    for key in sorted(stats.keys()):
        value = stats[key]
        if isinstance(value, float) and math.isfinite(value):
            print(f"  {key}: {value:.6f}")
        else:
            print(f"  {key}: {value}")


def split_topics(values: Sequence[str]) -> List[str]:
    out: List[str] = []
    for value in values:
        for item in value.split(","):
            item = item.strip()
            if item:
                out.append(item)
    return out


def command_series(args: argparse.Namespace) -> None:
    path = Path(args.mcap)
    if not path.exists():
        raise FileNotFoundError(path)

    metadata = load_runtime_metadata(path)
    messages = load_runtime_messages(path, topic=args.topic)
    timestamps_ms, values, available_fields = collect_series(messages, args.field)
    if not values:
        available = ", ".join(available_fields) if available_fields else "<none>"
        raise RuntimeError(
            f"No vector series found for topic='{args.topic}', field='{args.field}'. Available fields: {available}"
        )

    print("=== Runtime Log Summary ===")
    print(f"mcap_path: {path}")
    if metadata:
        print("metadata:")
        for name, kv in metadata.items():
            print(f"  {name}: {kv}")

    stats = summarize_vector_series(timestamps_ms, values)
    print_stats_block("series_stats:", stats)

    if args.plot:
        maybe_plot(timestamps_ms, values, parse_indices(args.plot_indices), f"{args.topic}/{args.field}")


def command_timing(args: argparse.Namespace) -> None:
    path = Path(args.mcap)
    if not path.exists():
        raise FileNotFoundError(path)

    topics = split_topics(args.topics) if args.topics else [
        "runtime/tick",
        "runtime/source/base_imu",
        "runtime/source/policy_observation",
        "runtime/source/policy_action",
    ]
    topic_to_messages = {
        topic: load_runtime_messages(path, topic=topic)
        for topic in topics
    }
    topic_to_ts = {
        topic: topic_timestamps(messages)
        for topic, messages in topic_to_messages.items()
    }

    print("=== Timing Summary ===")
    print(f"mcap_path: {path}")
    for topic in topics:
        stats = summarize_timestamps(topic_to_ts[topic])
        print_stats_block(f"topic={topic}", stats)

    if args.reference_topic:
        ref_ts_ms = topic_to_ts.get(args.reference_topic)
        if ref_ts_ms is None:
            ref_ts_ms = topic_timestamps(load_runtime_messages(path, topic=args.reference_topic))
        print(f"reference_topic: {args.reference_topic}")
        for topic in topics:
            if topic == args.reference_topic:
                continue
            hold_stats = hold_alignment_stats(topic_to_ts[topic], ref_ts_ms)
            nearest_stats = nearest_offset_stats(topic_to_ts[topic], ref_ts_ms)
            merged = {}
            merged.update(hold_stats)
            merged.update(nearest_stats)
            print_stats_block(f"alignment source={topic} -> reference={args.reference_topic}", merged)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Analyze runtime MCAP logs")
    subparsers = parser.add_subparsers(dest="command")

    series_parser = subparsers.add_parser("series", help="Analyze one vector field in one topic")
    series_parser.add_argument("--mcap", required=True, help="Path to runtime .mcap log")
    series_parser.add_argument("--topic", default="runtime/tick", help="MCAP topic/channel to analyze")
    series_parser.add_argument("--field", default="motor_state_tau", help="Vector field inside the JSON payload")
    series_parser.add_argument("--plot", action="store_true", help="Plot selected vector indices")
    series_parser.add_argument("--plot-indices", default="", help="Comma-separated vector indices to plot")
    series_parser.set_defaults(func=command_series)

    timing_parser = subparsers.add_parser("timing", help="Analyze topic timing, jitter, and hold alignment")
    timing_parser.add_argument("--mcap", required=True, help="Path to runtime .mcap log")
    timing_parser.add_argument(
        "--topics",
        nargs="*",
        default=[],
        help="Topics to analyze. Accepts repeated values or comma-separated lists.",
    )
    timing_parser.add_argument(
        "--reference-topic",
        default="runtime/tick",
        help="Reference topic for hold-age and nearest-offset alignment stats",
    )
    timing_parser.set_defaults(func=command_timing)

    return parser


def main() -> None:
    parser = build_parser()
    argv = sys.argv[1:]
    if argv and argv[0] not in {"series", "timing", "-h", "--help"}:
        argv = ["series"] + argv
    elif not argv:
        argv = ["series", "--help"]
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
