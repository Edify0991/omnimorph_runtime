#!/usr/bin/env python3
"""Export runtime MCAP logs to parquet/csv/npz."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List

from runtime_log_utils import flatten_message_rows, load_runtime_messages


def write_csv(path: Path, rows: List[Dict]) -> None:
    if not rows:
        raise RuntimeError("No rows to export")
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_npz(path: Path, rows: List[Dict]) -> None:
    try:
        import numpy as np
    except Exception as exc:  # pragma: no cover - optional dependency
        raise RuntimeError("numpy is required for npz export") from exc

    if not rows:
        raise RuntimeError("No rows to export")
    fieldnames = sorted({key for row in rows for key in row.keys()})
    arrays = {}
    for key in fieldnames:
        column = [row.get(key, None) for row in rows]
        if all(isinstance(v, (int, float, bool)) or v is None for v in column):
            arrays[key] = np.array([float(v) if v is not None else float("nan") for v in column], dtype=np.float64)
        else:
            arrays[key] = np.array([json.dumps(v, ensure_ascii=False) if isinstance(v, (dict, list)) else ("" if v is None else str(v)) for v in column], dtype=object)
    np.savez(path, **arrays)


def write_parquet(path: Path, rows: List[Dict]) -> None:
    try:
        import pyarrow as pa
        import pyarrow.parquet as pq
    except Exception as exc:  # pragma: no cover - optional dependency
        raise RuntimeError("pyarrow is required for parquet export") from exc

    if not rows:
        raise RuntimeError("No rows to export")
    fieldnames = sorted({key for row in rows for key in row.keys()})
    columns = {key: [row.get(key, None) for row in rows] for key in fieldnames}
    table = pa.table(columns)
    pq.write_table(table, path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Export runtime MCAP logs")
    parser.add_argument("--mcap", required=True, help="Path to runtime .mcap log")
    parser.add_argument("--topic", default="runtime/tick", help="MCAP topic/channel to export")
    parser.add_argument("--format", required=True, choices=["csv", "npz", "parquet"], help="Export format")
    parser.add_argument("--output", required=True, help="Output path")
    args = parser.parse_args()

    path = Path(args.mcap)
    if not path.exists():
        raise FileNotFoundError(path)

    messages = load_runtime_messages(path, topic=args.topic)
    rows = flatten_message_rows(messages)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if args.format == "csv":
        write_csv(output_path, rows)
    elif args.format == "npz":
        write_npz(output_path, rows)
    else:
        write_parquet(output_path, rows)

    print(f"exported_rows: {len(rows)}")
    print(f"output_path: {output_path}")


if __name__ == "__main__":
    main()
