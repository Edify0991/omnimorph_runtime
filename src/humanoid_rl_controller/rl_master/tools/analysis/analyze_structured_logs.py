#!/usr/bin/env python3
"""Analyze structured RL deploy logs (.jsonl + metadata.json)."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


def load_metadata(path: Path) -> Dict:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def iter_records(path: Path) -> Iterable[Dict]:
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                yield json.loads(text)
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"JSON parse error at line {line_no}: {exc}") from exc


def collect_vector_series(
    records_path: Path,
    record_type: str,
    vector_field: str,
) -> Tuple[List[float], List[List[float]], List[str]]:
    timestamps: List[float] = []
    values: List[List[float]] = []
    available_fields: set[str] = set()

    for record in iter_records(records_path):
        if record.get("record_type") != record_type:
            continue

        vectors = record.get("vectors", {})
        if isinstance(vectors, dict):
            available_fields.update(vectors.keys())

        if vector_field not in vectors:
            continue

        series = vectors.get(vector_field)
        if not isinstance(series, list):
            continue

        timestamp_ms = None
        if isinstance(vectors.get("timestamp_ms"), list) and vectors["timestamp_ms"]:
            timestamp_ms = float(vectors["timestamp_ms"][0])
        else:
            monotonic_time_sec = record.get("monotonic_time_sec")
            if monotonic_time_sec is not None:
                timestamp_ms = float(monotonic_time_sec) * 1000.0

        if timestamp_ms is None:
            continue

        timestamps.append(timestamp_ms)
        values.append([float(x) for x in series])

    return timestamps, values, sorted(available_fields)


def summarize_series(timestamps_ms: Sequence[float], values: Sequence[Sequence[float]]) -> Dict[str, float]:
    if not timestamps_ms or not values:
        return {}

    dt_ms = [
        timestamps_ms[i + 1] - timestamps_ms[i]
        for i in range(len(timestamps_ms) - 1)
        if timestamps_ms[i + 1] > timestamps_ms[i]
    ]

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
        "value_mean": mean(flattened),
    }

    if dt_ms:
        avg_dt_ms = mean(dt_ms)
        stats["avg_dt_ms"] = avg_dt_ms
        if avg_dt_ms > 1e-9:
            stats["avg_hz"] = 1000.0 / avg_dt_ms

    return stats


def write_csv(path: Path, timestamps_ms: Sequence[float], values: Sequence[Sequence[float]], prefix: str) -> None:
    if not timestamps_ms or not values:
        raise RuntimeError("No data available to export CSV.")

    dim = len(values[0])
    headers = ["timestamp_ms"] + [f"{prefix}_{i}" for i in range(dim)]

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for ts, row in zip(timestamps_ms, values):
            writer.writerow([f"{ts:.6f}"] + [f"{x:.9f}" for x in row])


def maybe_plot(
    timestamps_ms: Sequence[float],
    values: Sequence[Sequence[float]],
    selected_indices: Sequence[int],
    title: str,
) -> None:
    if not timestamps_ms or not values:
        raise RuntimeError("No data available for plotting.")

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise RuntimeError("matplotlib is required for --plot") from exc

    dim = len(values[0])
    if not selected_indices:
        selected_indices = tuple(range(min(dim, 4)))

    plt.figure(figsize=(12, 5))
    for idx in selected_indices:
        if idx < 0 or idx >= dim:
            continue
        ys = [row[idx] for row in values]
        plt.plot(timestamps_ms, ys, label=f"idx={idx}")

    plt.xlabel("timestamp (ms)")
    plt.ylabel("value")
    plt.title(title)
    plt.legend()
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.show()


def parse_indices(text: str) -> List[int]:
    if not text:
        return []
    out: List[int] = []
    for token in text.split(","):
        stripped = token.strip()
        if not stripped:
            continue
        out.append(int(stripped))
    return out


def infer_metadata_path(records_path: Path) -> Optional[Path]:
    candidate = Path(str(records_path).replace("_records.jsonl", "_metadata.json"))
    if candidate.exists():
        return candidate
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Analyze structured sim2real deploy logs")
    parser.add_argument("--records", required=True, help="Path to *_records.jsonl")
    parser.add_argument("--metadata", default="", help="Path to *_metadata.json")
    parser.add_argument("--record-type", default="solver_loop", help="Record type to analyze")
    parser.add_argument("--vector-field", default="motor_state_tau", help="Vector field name under vectors")
    parser.add_argument("--csv-out", default="", help="Optional CSV output path")
    parser.add_argument("--plot", action="store_true", help="Plot selected indices")
    parser.add_argument("--plot-indices", default="", help="Comma-separated indices for plotting")
    args = parser.parse_args()

    records_path = Path(args.records)
    if not records_path.exists():
        raise FileNotFoundError(f"Records file not found: {records_path}")

    metadata_path = Path(args.metadata) if args.metadata else infer_metadata_path(records_path)
    metadata = load_metadata(metadata_path) if metadata_path else {}

    timestamps_ms, values, available_fields = collect_vector_series(
        records_path=records_path,
        record_type=args.record_type,
        vector_field=args.vector_field,
    )

    if not values:
        available = ", ".join(available_fields) if available_fields else "<none>"
        raise RuntimeError(
            f"No data found for record_type='{args.record_type}', vector_field='{args.vector_field}'. "
            f"Available vector fields: {available}"
        )

    stats = summarize_series(timestamps_ms, values)

    print("=== Structured Log Summary ===")
    print(f"records_path: {records_path}")
    if metadata_path:
        print(f"metadata_path: {metadata_path}")

    string_fields = metadata.get("string_fields", {}) if isinstance(metadata, dict) else {}
    numeric_fields = metadata.get("numeric_fields", {}) if isinstance(metadata, dict) else {}
    if string_fields:
        print("module:", string_fields.get("module", "<unknown>"))
        if "policy_name" in string_fields:
            print("policy_name:", string_fields["policy_name"])
        if "policy_path" in string_fields:
            print("policy_path:", string_fields["policy_path"])
    if numeric_fields and "control_hz" in numeric_fields:
        print("configured_control_hz:", numeric_fields["control_hz"])

    for key in sorted(stats.keys()):
        value = stats[key]
        if math.isfinite(value):
            print(f"{key}: {value:.6f}")
        else:
            print(f"{key}: {value}")

    if args.csv_out:
        csv_path = Path(args.csv_out)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        write_csv(csv_path, timestamps_ms, values, args.vector_field)
        print(f"csv_export: {csv_path}")

    if args.plot:
        indices = parse_indices(args.plot_indices)
        maybe_plot(
            timestamps_ms=timestamps_ms,
            values=values,
            selected_indices=indices,
            title=f"{args.record_type}/{args.vector_field}",
        )


if __name__ == "__main__":
    main()
