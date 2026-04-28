#!/usr/bin/env python3
"""Expand the first RUNNING observation window from a runtime MCAP log."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

from runtime_log_utils import load_runtime_messages


def _default_term_dim(term: Dict[str, Any]) -> int:
    name = str(term.get("name", "")).strip()
    components = term.get("components") or []
    target_order = term.get("target_order") or []
    count = term.get("count")

    if isinstance(count, int) and count > 0:
        return count
    if isinstance(components, list) and components:
        return len(components)
    if isinstance(target_order, list) and target_order:
        return len(target_order)

    if name == "phase":
        return 2
    if name == "command":
        return 3
    if name in {"joint_pos", "joint_vel", "last_action"}:
        return 12
    if name in {"base_ang_vel", "base_rpy", "base_euler"}:
        return 3
    if name == "base_quat":
        return 4
    return 0


def _load_manifest_terms(manifest_path: Path) -> List[Dict[str, Any]]:
    root = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    manifest = root.get("observation_manifest", {})
    terms = manifest.get("terms", [])
    out: List[Dict[str, Any]] = []
    running_offset = 0

    for term in terms:
        if not isinstance(term, dict):
            continue
        if not term.get("enabled", True):
            continue
        dim = _default_term_dim(term)
        if dim <= 0:
            continue
        out.append(
            {
                "name": str(term.get("name", "")),
                "start": running_offset,
                "end": running_offset + dim,
                "dim": dim,
                "components": term.get("components") or [],
            }
        )
        running_offset += dim
    return out


def _resolve_manifest_path(mcap_path: Path, explicit_path: Optional[str]) -> Optional[Path]:
    if explicit_path:
        path = Path(explicit_path)
        return path if path.exists() else None

    config_messages = load_runtime_messages(mcap_path, topic="runtime/config")
    if not config_messages:
        return None
    payload = config_messages[0].get("data")
    if not isinstance(payload, dict):
        return None

    config_section = str(payload.get("config_section", "")).strip()
    if not config_section:
        return None

    profiles_dir = Path(__file__).resolve().parents[2] / "config" / "profiles"
    candidate = profiles_dir / f"{config_section}.yaml"
    if not candidate.exists():
        return None

    root = yaml.safe_load(candidate.read_text(encoding="utf-8"))
    section = root.get(config_section, {})
    if not isinstance(section, dict):
        return None

    manifest_path_raw = str(section.get("observation_manifest_path", "")).strip()
    if manifest_path_raw:
        manifest_path = Path(manifest_path_raw)
        return manifest_path if manifest_path.exists() else None

    manifest_file = str(section.get("observation_manifest_file", "")).strip()
    if manifest_file:
        manifest_path = profiles_dir.parent / manifest_file
        return manifest_path if manifest_path.exists() else None
    return None


def _slice_terms(values: Sequence[float], terms: Sequence[Dict[str, Any]]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for term in terms:
        start = int(term["start"])
        end = int(term["end"])
        out.append(
            {
                "name": term["name"],
                "start": start,
                "end": end,
                "dim": term["dim"],
                "components": term["components"],
                "values": list(values[start:end]),
            }
        )
    return out


def _load_tick_payloads(mcap_path: Path) -> List[Dict[str, Any]]:
    messages = load_runtime_messages(mcap_path, topic="runtime/tick")
    payloads: List[Dict[str, Any]] = []
    for message in messages:
        payload = message.get("data")
        if isinstance(payload, dict):
            payloads.append(payload)
    return payloads


def _find_first_running_index(ticks: Sequence[Dict[str, Any]]) -> int:
    for idx, payload in enumerate(ticks):
        if payload.get("deploy_state") == 3:
            return idx
    raise RuntimeError("No RUNNING tick found in runtime/tick stream")


def _named_feature_status(window: Sequence[Dict[str, Any]]) -> Tuple[bool, List[str]]:
    feature_keys = set()
    has_any = False
    for payload in window:
        named = payload.get("named_features")
        if isinstance(named, dict) and named:
            has_any = True
            feature_keys.update(named.keys())
    return has_any, sorted(feature_keys)


def build_report(
    mcap_path: Path,
    tick_count: int,
    running_index: Optional[int],
    manifest_path: Optional[Path],
) -> Dict[str, Any]:
    ticks = _load_tick_payloads(mcap_path)
    if not ticks:
        raise RuntimeError("No runtime/tick payloads found")

    first_running = _find_first_running_index(ticks) if running_index is None else running_index
    if first_running < 0 or first_running >= len(ticks):
        raise RuntimeError(f"RUNNING start index out of range: {first_running}")

    window = ticks[first_running : min(len(ticks), first_running + max(1, tick_count))]
    terms = _load_manifest_terms(manifest_path) if manifest_path else []
    has_named_features, feature_keys = _named_feature_status(window)

    rows: List[Dict[str, Any]] = []
    for local_idx, payload in enumerate(window):
        obs = payload.get("observation")
        if not isinstance(obs, list):
            obs = []
        named_features = payload.get("named_features")
        if not isinstance(named_features, dict):
            named_features = {}

        row: Dict[str, Any] = {
            "window_tick": local_idx,
            "runtime_tick_index": first_running + local_idx,
            "frame_index": payload.get("frame_index"),
            "monotonic_time_sec": payload.get("monotonic_time_sec"),
            "phase_t": payload.get("phase_t"),
            "policy_step_index": payload.get("policy_step_index"),
            "policy_ran_this_tick": payload.get("policy_ran_this_tick"),
            "deploy_state": payload.get("deploy_state"),
            "open_rl": payload.get("open_rl"),
            "observation_dim": len(obs),
            "observation": obs,
            "observation_terms": _slice_terms(obs, terms) if terms else [],
            "named_features": named_features,
            "named_feature_keys": sorted(named_features.keys()),
            "policy_action": payload.get("policy_action") if isinstance(payload.get("policy_action"), list) else [],
        }
        rows.append(row)

    return {
        "mcap_path": str(mcap_path),
        "running_start_tick_index": first_running,
        "tick_count": len(rows),
        "manifest_path": str(manifest_path) if manifest_path else None,
        "manifest_terms": terms,
        "named_features_present": has_named_features,
        "named_feature_keys": feature_keys,
        "notes": [
            "named_features missing usually means runtime logging config disabled include_external_observations"
            if not has_named_features
            else "named_features were found in this window"
        ],
        "ticks": rows,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Expand observation and named feature payloads for the first RUNNING ticks"
    )
    parser.add_argument("--mcap", required=True, help="Path to runtime .mcap log")
    parser.add_argument("--tick-count", type=int, default=100, help="Number of RUNNING ticks to dump")
    parser.add_argument(
        "--running-start-index",
        type=int,
        default=None,
        help="Override the RUNNING start tick index instead of auto-detecting the first deploy_state==3",
    )
    parser.add_argument(
        "--manifest-path",
        default="",
        help="Optional observation manifest path. If omitted, auto-resolve from runtime/config + profile yaml.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional output JSON path. If omitted, print to stdout only.",
    )
    args = parser.parse_args()

    mcap_path = Path(args.mcap)
    if not mcap_path.exists():
        raise FileNotFoundError(mcap_path)

    manifest_path = _resolve_manifest_path(mcap_path, args.manifest_path or None)
    report = build_report(mcap_path, args.tick_count, args.running_start_index, manifest_path)

    print("=== RUNNING Observation Window ===")
    print(f"mcap_path: {report['mcap_path']}")
    print(f"running_start_tick_index: {report['running_start_tick_index']}")
    print(f"tick_count: {report['tick_count']}")
    print(f"manifest_path: {report['manifest_path']}")
    print(f"named_features_present: {report['named_features_present']}")
    if report["named_feature_keys"]:
        print("named_feature_keys:")
        for key in report["named_feature_keys"]:
            print(f"  - {key}")
    for note in report["notes"]:
        print(f"note: {note}")

    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(rendered, encoding="utf-8")
        print(f"output_path: {output_path}")
    else:
        print(rendered)


if __name__ == "__main__":
    main()
