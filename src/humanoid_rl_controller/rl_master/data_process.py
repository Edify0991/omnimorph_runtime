#!/usr/bin/env python3
"""Legacy entrypoint kept for compatibility.

Please use tools/analysis/analyze_structured_logs.py for new structured logs.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description="Compatibility wrapper for structured log analysis")
    parser.add_argument("--records", required=True, help="Path to *_records.jsonl")
    parser.add_argument("--metadata", default="", help="Path to *_metadata.json")
    parser.add_argument("--record-type", default="solver_loop")
    parser.add_argument("--vector-field", default="motor_state_tau")
    parser.add_argument("--csv-out", default="")
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--plot-indices", default="")
    args = parser.parse_args()

    tool = Path(__file__).resolve().parent / "tools" / "analysis" / "analyze_structured_logs.py"
    cmd = [
        sys.executable,
        str(tool),
        "--records",
        args.records,
        "--record-type",
        args.record_type,
        "--vector-field",
        args.vector_field,
    ]

    if args.metadata:
        cmd += ["--metadata", args.metadata]
    if args.csv_out:
        cmd += ["--csv-out", args.csv_out]
    if args.plot:
        cmd.append("--plot")
    if args.plot_indices:
        cmd += ["--plot-indices", args.plot_indices]

    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
