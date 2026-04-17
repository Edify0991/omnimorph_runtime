#!/usr/bin/env python3
"""Validate RL deploy config/manifest/ONNX contracts before runtime."""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import yaml
except ImportError as exc:  # pragma: no cover - runtime dependency
    raise SystemExit(
        "PyYAML is required. Install with: pip install pyyaml"
    ) from exc

try:
    import onnxruntime as ort
except ImportError:  # pragma: no cover - optional at runtime
    ort = None


CANONICAL_JOINT_ORDER: List[str] = [
    "right_hip_roll",
    "right_hip_yaw",
    "right_hip_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
    "right_ankle_roll",
    "left_hip_roll",
    "left_hip_yaw",
    "left_hip_pitch",
    "left_knee_pitch",
    "left_ankle_pitch",
    "left_ankle_roll",
]

SUPPORTED_MANIFEST_TERMS = {
    "phase",
    "command",
    "joint_pos",
    "joint_vel",
    "last_action",
    "base_ang_vel",
    "base_rpy",
    "base_euler",
    "base_quat",
    "reference_motion",
    "reference_joint_pos",
    "reference_joint_vel",
    "motion_anchor_pos_b",
    "motion_ref_pos_b",
    "motion_anchor_ori_b",
    "motion_ref_ori_b",
    "motion_body_pos_b",
    "motion_body_ori_b",
    "robot_body_pos",
    "robot_body_ori",
    "external_sensor",
    "feature",
}

DEFAULT_TERM_DIM = {
    "phase": 2,
    "command": 3,
    "joint_pos": 12,
    "joint_vel": 12,
    "last_action": 12,
    "reference_joint_pos": 12,
    "reference_joint_vel": 12,
    "base_ang_vel": 3,
    "base_rpy": 3,
    "base_euler": 3,
    "base_quat": 4,
    "motion_anchor_pos_b": 3,
    "motion_ref_pos_b": 3,
    "motion_anchor_ori_b": 6,
    "motion_ref_ori_b": 6,
    "motion_body_pos_b": 0,
    "motion_body_ori_b": 0,
    "robot_body_pos": 0,
    "robot_body_ori": 0,
    "reference_motion": 0,
    "external_sensor": 0,
    "feature": 0,
}

REQUIRES_POSITIVE_COUNT = {
    "reference_motion",
    "external_sensor",
    "feature",
    "motion_body_pos_b",
    "motion_body_ori_b",
    "robot_body_pos",
    "robot_body_ori",
}

SUPPORTED_COMPONENTS = {
    "command": {"vx", "vy", "dyaw"},
    "base_ang_vel": {"wx", "wy", "wz", "x", "y", "z"},
    "base_euler": {"roll", "pitch", "yaw"},
    "base_quat": {"x", "y", "z", "w"},
}

SUPPORTED_ONNX_INPUT_SOURCES = {
    "stacked_observation",
    "observation",
    "time_step",
    "feature",
    "constant",
}

SUPPORTED_FILL_POLICIES = {"error", "zero"}
SUPPORTED_IMU_PAYLOADS = {"euler_compat", "quaternion"}
SUPPORTED_EULER_UNITS = {"rad", "deg"}
SUPPORTED_QUAT_ORDERS = {"xyzw", "wxyz"}
SUPPORTED_REFERENCE_SOURCES = {"auto", "file", "policy_outputs"}


@dataclass
class Issue:
    level: str  # ERROR / WARN / INFO
    context: str
    message: str


class IssueCollector:
    def __init__(self) -> None:
        self._items: List[Issue] = []

    def error(self, context: str, message: str) -> None:
        self._items.append(Issue(level="ERROR", context=context, message=message))

    def warn(self, context: str, message: str) -> None:
        self._items.append(Issue(level="WARN", context=context, message=message))

    def info(self, context: str, message: str) -> None:
        self._items.append(Issue(level="INFO", context=context, message=message))

    @property
    def items(self) -> Sequence[Issue]:
        return self._items

    @property
    def error_count(self) -> int:
        return sum(1 for x in self._items if x.level == "ERROR")

    @property
    def warning_count(self) -> int:
        return sum(1 for x in self._items if x.level == "WARN")

    def print(self) -> None:
        for item in self._items:
            print(f"[{item.level}] {item.context}: {item.message}")


def to_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def to_list(value: Any) -> List[Any]:
    return value if isinstance(value, list) else []


def as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def as_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        norm = value.strip().lower()
        if norm in {"1", "true", "yes", "on"}:
            return True
        if norm in {"0", "false", "no", "off"}:
            return False
    return default


def normalize_token(value: Any) -> str:
    return str(value).strip().lower()


def validate_exact_token_order(
    values: Sequence[Any],
    expected_tokens: Sequence[str],
    issues: IssueCollector,
    context: str,
    field_name: str,
) -> List[str]:
    normalized = [normalize_token(x) for x in values]
    expected = [normalize_token(x) for x in expected_tokens]
    if len(normalized) != len(expected):
        issues.error(context, f"{field_name} must contain exactly {len(expected)} items")
        return normalized
    if len(set(normalized)) != len(normalized):
        issues.error(context, f"{field_name} contains duplicates")
        return normalized
    if set(normalized) != set(expected):
        issues.error(context, f"{field_name} must be a permutation of {list(expected_tokens)}")
    return normalized


def resolve_path(raw: str, root_dir: Path) -> Path:
    path = Path(raw)
    if path.is_absolute():
        return path
    return root_dir / path


def find_by_name(seq: Sequence[Tuple[str, Any]], key: str) -> Optional[Any]:
    for name, value in seq:
        if name == key:
            return value
    return None


def load_rl_cfg(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Top-level YAML must be a map: {path}")
    return data


def get_mode_profile_specs(root_cfg: Dict[str, Any]) -> List[Tuple[int, str, str]]:
    raw = root_cfg.get("deploy_mode_profiles")
    specs: List[Tuple[int, str, str]] = []
    if isinstance(raw, list) and raw:
        for item in raw:
            node = to_dict(item)
            if "mode_id" not in node or "config_section" not in node:
                continue
            mode_id = as_int(node.get("mode_id"))
            section = str(node.get("config_section"))
            tag = str(node.get("tag", section))
            specs.append((mode_id, section, tag))
    if specs:
        return specs
    return [(0, "sim2real", "mode_0")]


def get_policy_file_path(section_cfg: Dict[str, Any], root_dir: Path) -> Path:
    policy_name = str(section_cfg.get("policy_name", ""))
    policy_path_raw = str(section_cfg.get("policy_path", ""))
    policy_file_raw = str(section_cfg.get("policy_file", ""))
    if policy_path_raw:
        return resolve_path(policy_path_raw, root_dir)
    if policy_file_raw:
        return resolve_path(policy_file_raw, root_dir)
    return root_dir / "policies" / f"{policy_name}.onnx"


def get_manifest_path(section_cfg: Dict[str, Any], root_dir: Path) -> Path:
    raw_manifest_path = str(section_cfg.get("observation_manifest_path", ""))
    if raw_manifest_path:
        candidate = resolve_path(raw_manifest_path, root_dir)
    else:
        manifest_file = str(section_cfg.get("observation_manifest_file", "observation_manifest.yaml"))
        candidate = root_dir / "config" / manifest_file
    if candidate.exists():
        return candidate
    return root_dir / "config" / "observation_manifest.yaml"


def get_reference_motion_path(section_cfg: Dict[str, Any], root_dir: Path) -> Path:
    reference_path_raw = str(section_cfg.get("reference_motion_path", ""))
    if reference_path_raw:
        return resolve_path(reference_path_raw, root_dir)
    reference_file_raw = str(section_cfg.get("reference_motion_file", ""))
    if reference_file_raw:
        return resolve_path(reference_file_raw, root_dir)
    return root_dir / "reference_motion" / "reference_motion.txt"


def manifest_term_name(term: Dict[str, Any]) -> str:
    if "name" in term:
        return str(term["name"])
    if "type" in term:
        return str(term["type"])
    return ""


def parse_manifest_dim(manifest_path: Path, issues: IssueCollector, context: str) -> int:
    if not manifest_path.exists():
        issues.error(context, f"manifest file not found: {manifest_path}")
        return 0
    try:
        with manifest_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception as exc:
        issues.error(context, f"failed to parse manifest YAML: {exc}")
        return 0

    root = to_dict(data.get("observation_manifest"))
    terms = to_list(root.get("terms"))
    if not terms:
        issues.error(context, "observation_manifest.terms is missing or empty")
        return 0

    total_dim = 0
    for idx, raw_term in enumerate(terms):
        term = to_dict(raw_term)
        name = manifest_term_name(term)
        term_context = f"{context} term[{idx}]"
        if not name:
            issues.error(term_context, "missing both 'name' and 'type'")
            continue
        enabled = as_bool(term.get("enabled", True), True)
        if not enabled:
            continue

        components = [str(x) for x in to_list(term.get("components"))]
        count_raw = term.get("count", None)

        if name not in SUPPORTED_MANIFEST_TERMS:
            issues.error(term_context, f"unsupported term name '{name}'")
            continue

        if components and name not in SUPPORTED_COMPONENTS:
            issues.error(term_context, f"components is not supported by term '{name}'")
            continue
        if components:
            allowed = SUPPORTED_COMPONENTS[name]
            normalized_components = [normalize_token(x) for x in components]
            for component in normalized_components:
                if component not in allowed:
                    issues.error(
                        term_context,
                        f"unsupported component '{component}' for term '{name}', allowed={sorted(allowed)}",
                    )
            components = normalized_components

        if name == "phase":
            default_count = 2
        elif name == "command":
            default_count = len(components) if components else 3
        elif name in {"joint_pos", "joint_vel", "last_action"}:
            default_count = 12
        elif name in {"reference_joint_pos", "reference_joint_vel"}:
            default_count = 12
        elif name in {"base_ang_vel", "base_rpy", "base_euler"}:
            default_count = 3
        elif name == "base_quat":
            default_count = 4
        elif name in {"motion_anchor_pos_b", "motion_ref_pos_b"}:
            default_count = 3
        elif name in {"motion_anchor_ori_b", "motion_ref_ori_b"}:
            default_count = 6
        else:
            default_count = 0

        count = default_count if count_raw is None else as_int(count_raw, default_count)

        if name == "phase" and count > 2:
            issues.error(term_context, "phase count cannot exceed 2")
            continue
        if name == "command":
            if components and count != len(components):
                issues.error(term_context, "command count must equal components length")
                continue
            if not components and count > 3:
                issues.error(term_context, "command count cannot exceed 3 when components is empty")
                continue
        if name in {"base_ang_vel", "base_euler", "base_quat"}:
            if components and count != len(components):
                issues.error(term_context, f"{name} count must equal components length")
                continue
            if not components and count > default_count:
                issues.error(
                    term_context,
                    f"{name} count cannot exceed {default_count} when components is empty",
                )
                continue
        if name in REQUIRES_POSITIVE_COUNT and count <= 0:
            issues.error(term_context, f"{name} requires count > 0")
            continue

        if name in SUPPORTED_COMPONENTS and components:
            dim = len(components)
        elif count > 0:
            dim = count
        else:
            dim = DEFAULT_TERM_DIM.get(name, 0)
        total_dim += max(dim, 0)

    return total_dim


def check_joint_order(
    action_dim: int,
    motor_n: int,
    action_order: List[str],
    obs_order_raw: List[str],
    issues: IssueCollector,
    context: str,
) -> None:
    if action_dim <= 0:
        issues.error(context, "action_dim must be > 0")
    if motor_n <= 0:
        issues.error(context, "motor_N must be > 0")
    if action_dim > len(CANONICAL_JOINT_ORDER):
        issues.error(context, "action_dim exceeds canonical joint count")
    if motor_n > len(CANONICAL_JOINT_ORDER):
        issues.error(context, "motor_N exceeds canonical joint count")

    if action_order:
        if len(action_order) != action_dim:
            issues.error(context, "action_joint_order length must equal action_dim")
        if len(set(action_order)) != len(action_order):
            issues.error(context, "action_joint_order contains duplicates")
        for name in action_order:
            if name not in CANONICAL_JOINT_ORDER:
                issues.warn(context, f"action_joint_order contains non-canonical joint name '{name}'")
        for canonical_name in CANONICAL_JOINT_ORDER[: max(action_dim, 0)]:
            if canonical_name not in action_order:
                issues.error(context, f"action_joint_order missing required canonical joint '{canonical_name}'")

    obs_order = list(obs_order_raw) if obs_order_raw else list(action_order)
    if obs_order:
        if len(obs_order) != motor_n:
            issues.error(context, "obs_joint_order length must equal motor_N")
        if len(set(obs_order)) != len(obs_order):
            issues.error(context, "obs_joint_order contains duplicates")
        for name in obs_order:
            if name not in CANONICAL_JOINT_ORDER:
                issues.error(context, f"obs_joint_order contains unknown joint '{name}'")


def check_reference_joint_order(
    motor_n: int,
    action_order: List[str],
    reference_order_raw: List[str],
    issues: IssueCollector,
    context: str,
) -> List[str]:
    reference_order = list(reference_order_raw) if reference_order_raw else list(action_order)
    if not reference_order:
        issues.error(context, "reference_joint_order is empty and action_joint_order is also empty")
        return reference_order
    if motor_n > 0 and len(reference_order) != motor_n:
        issues.error(context, "reference_joint_order length must equal motor_N")
    if len(set(reference_order)) != len(reference_order):
        issues.error(context, "reference_joint_order contains duplicates")
    for name in reference_order:
        if name not in CANONICAL_JOINT_ORDER:
            issues.error(context, f"reference_joint_order contains unknown joint '{name}'")
    return reference_order


def build_feature_dim_map(section_cfg: Dict[str, Any], reference_order: List[str]) -> Dict[str, int]:
    feature_dims: Dict[str, int] = {}
    motor_n = as_int(section_cfg.get("motor_N"), 0)
    reference_dim = len(reference_order) if reference_order else motor_n

    if reference_dim > 0:
        feature_dims["reference_joint_pos"] = reference_dim
        feature_dims["reference_joint_vel"] = reference_dim

    reference_motion_dim = as_int(section_cfg.get("reference_motion_dim"), 0)
    if reference_motion_dim > 0:
        feature_dims["reference_motion"] = reference_motion_dim

    feature_dims["motion_anchor_pos_b"] = 3
    feature_dims["motion_ref_pos_b"] = 3
    feature_dims["motion_anchor_ori_b"] = 6
    feature_dims["motion_ref_ori_b"] = 6

    body_names = [str(x) for x in to_list(section_cfg.get("reference_body_names"))]
    if body_names:
        feature_dims["motion_body_pos_b"] = len(body_names) * 3
        feature_dims["motion_body_ori_b"] = len(body_names) * 6
        feature_dims["robot_body_pos"] = len(body_names) * 3
        feature_dims["robot_body_ori"] = len(body_names) * 6

    for spec in to_list(section_cfg.get("external_observations")):
        node = to_dict(spec)
        name = str(node.get("name", "")).strip()
        dim = as_int(node.get("dim"), 0)
        if name and dim > 0:
            feature_dims[name] = dim

    return feature_dims


def check_source_contract(section_cfg: Dict[str, Any], issues: IssueCollector, context: str) -> None:
    source_contract = to_dict(section_cfg.get("source_contract"))
    if not source_contract:
        issues.error(context, "missing source_contract section")
        return

    imu_input = to_dict(source_contract.get("imu_input"))
    if not imu_input:
        issues.error(context, "missing source_contract.imu_input")
    else:
        payload = normalize_token(imu_input.get("payload", ""))
        if payload not in SUPPORTED_IMU_PAYLOADS:
            issues.error(
                context,
                f"source_contract.imu_input.payload must be one of {sorted(SUPPORTED_IMU_PAYLOADS)}",
            )

        validate_exact_token_order(
            to_list(imu_input.get("euler_order")),
            ["roll", "pitch", "yaw"],
            issues,
            context,
            "source_contract.imu_input.euler_order",
        )

        euler_unit = normalize_token(imu_input.get("euler_unit", ""))
        if euler_unit not in SUPPORTED_EULER_UNITS:
            issues.error(
                context,
                f"source_contract.imu_input.euler_unit must be one of {sorted(SUPPORTED_EULER_UNITS)}",
            )

        quat_order = normalize_token(imu_input.get("quat_order", ""))
        if quat_order not in SUPPORTED_QUAT_ORDERS:
            issues.error(
                context,
                f"source_contract.imu_input.quat_order must be one of {sorted(SUPPORTED_QUAT_ORDERS)}",
            )

        validate_exact_token_order(
            to_list(imu_input.get("ang_vel_order")),
            ["x", "y", "z"],
            issues,
            context,
            "source_contract.imu_input.ang_vel_order",
        )

        frame_alignment = to_list(imu_input.get("frame_alignment_rpy"))
        if len(frame_alignment) != 3:
            issues.error(context, "source_contract.imu_input.frame_alignment_rpy must contain exactly 3 items")
        else:
            for idx, value in enumerate(frame_alignment):
                try:
                    parsed = float(value)
                except Exception:
                    issues.error(
                        context,
                        f"source_contract.imu_input.frame_alignment_rpy[{idx}] is not numeric: {value}",
                    )
                    continue
                if not math.isfinite(parsed):
                    issues.error(
                        context,
                        f"source_contract.imu_input.frame_alignment_rpy[{idx}] is not finite: {value}",
                    )

    sim_base = to_dict(source_contract.get("sim_base"))
    if not sim_base:
        issues.error(context, "missing source_contract.sim_base")
    else:
        quat_source_order = normalize_token(sim_base.get("quat_source_order", ""))
        if quat_source_order != "wxyz":
            issues.error(context, "source_contract.sim_base.quat_source_order must be 'wxyz' for MuJoCo qpos")

    reference_file = to_dict(source_contract.get("reference_file"))
    if not reference_file:
        issues.error(context, "missing source_contract.reference_file")
    else:
        body_quat_order = normalize_token(reference_file.get("body_quat_order", ""))
        if body_quat_order not in SUPPORTED_QUAT_ORDERS:
            issues.error(
                context,
                f"source_contract.reference_file.body_quat_order must be one of {sorted(SUPPORTED_QUAT_ORDERS)}",
            )

    policy_extra_outputs = to_dict(source_contract.get("policy_extra_outputs"))
    if not policy_extra_outputs:
        issues.error(context, "missing source_contract.policy_extra_outputs")
    else:
        body_quat_order = normalize_token(policy_extra_outputs.get("body_quat_order", ""))
        if body_quat_order not in SUPPORTED_QUAT_ORDERS:
            issues.error(
                context,
                "source_contract.policy_extra_outputs.body_quat_order must be one of "
                f"{sorted(SUPPORTED_QUAT_ORDERS)}",
            )


def check_reference_contract(
    section_cfg: Dict[str, Any],
    root_dir: Path,
    reference_order: List[str],
    issues: IssueCollector,
    context: str,
) -> None:
    reference_source = normalize_token(section_cfg.get("reference_motion_source", "auto"))
    if reference_source not in SUPPORTED_REFERENCE_SOURCES:
        issues.error(
            context,
            f"reference_motion_source must be one of {sorted(SUPPORTED_REFERENCE_SOURCES)}",
        )

    body_names = [str(x) for x in to_list(section_cfg.get("reference_body_names"))]
    if body_names and len(set(body_names)) != len(body_names):
        issues.error(context, "reference_body_names contains duplicates")

    anchor_body = str(section_cfg.get("reference_anchor_body", "")).strip()
    if body_names and anchor_body and anchor_body not in body_names:
        issues.error(context, "reference_anchor_body must be listed in reference_body_names")

    if reference_order and as_int(section_cfg.get("reference_motion_dim"), 0) > 0:
        expected_joint_dim = len(reference_order) * 2
        configured_dim = as_int(section_cfg.get("reference_motion_dim"), 0)
        if configured_dim not in {0, expected_joint_dim}:
            issues.warn(
                context,
                f"reference_motion_dim={configured_dim} differs from 2 * len(reference_joint_order)={expected_joint_dim}",
            )

    if not as_bool(section_cfg.get("enable_reference_motion", False), False):
        return

    if reference_source in {"file", "auto"}:
        reference_path = get_reference_motion_path(section_cfg, root_dir)
        if not reference_path.exists():
            issues.error(context, f"reference motion file not found: {reference_path}")
            return

        if reference_path.suffix.lower() not in {".yaml", ".yml", ".json"}:
            return

        try:
            with reference_path.open("r", encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}
        except Exception as exc:
            issues.error(context, f"failed to parse structured reference motion file: {exc}")
            return

        motion_root = to_dict(raw.get("reference_motion", raw))
        file_body_names = [str(x) for x in to_list(motion_root.get("body_names"))]
        file_anchor = str(motion_root.get("anchor_body", "")).strip()
        file_quat_order = normalize_token(motion_root.get("body_quat_format", ""))

        if body_names and file_body_names and body_names != file_body_names:
            issues.error(
                context,
                "reference_body_names does not match structured reference file body_names",
            )
        if anchor_body and file_anchor and anchor_body != file_anchor:
            issues.warn(
                context,
                f"reference_anchor_body='{anchor_body}' differs from file anchor_body='{file_anchor}'; config overrides file metadata",
            )

        reference_file_contract = to_dict(to_dict(section_cfg.get("source_contract")).get("reference_file"))
        contract_quat_order = normalize_token(reference_file_contract.get("body_quat_order", ""))
        if file_quat_order and contract_quat_order and file_quat_order != contract_quat_order:
            issues.warn(
                context,
                "reference file body_quat_format differs from source_contract.reference_file.body_quat_order; "
                "runtime uses config as source of truth",
            )


def parse_int_metadata(value: str) -> Optional[int]:
    try:
        return int(str(value).strip())
    except Exception:
        return None


def get_model_metadata_map(session: "ort.InferenceSession") -> Dict[str, str]:
    meta = session.get_modelmeta()
    if not meta or not hasattr(meta, "custom_metadata_map"):
        return {}
    raw = getattr(meta, "custom_metadata_map")
    if not isinstance(raw, dict):
        return {}
    out: Dict[str, str] = {}
    for key, value in raw.items():
        out[str(key)] = str(value)
    return out


def check_metadata_contract(
    custom_metadata: Dict[str, str],
    model_cfg: Dict[str, Any],
    issues: IssueCollector,
    context: str,
) -> None:
    if not as_bool(model_cfg.get("enable_metadata_check", False), False):
        return
    strict = as_bool(model_cfg.get("metadata_check_strict", True), True)

    def add_metadata_issue(message: str) -> None:
        if strict:
            issues.error(context, message)
        else:
            issues.warn(context, message)

    if not custom_metadata:
        add_metadata_issue("metadata check enabled but custom metadata map is empty")
        return

    required_keys = [str(x) for x in to_list(model_cfg.get("required_metadata_keys"))]
    for key in required_keys:
        if key and key not in custom_metadata:
            add_metadata_issue(f"missing required metadata key '{key}'")

    expected_metadata = to_dict(model_cfg.get("expected_metadata"))
    for key, expected in expected_metadata.items():
        key_str = str(key)
        expected_str = str(expected).strip()
        actual = custom_metadata.get(key_str)
        if actual is None:
            add_metadata_issue(f"missing expected metadata key '{key_str}'")
            continue
        if str(actual).strip() != expected_str:
            add_metadata_issue(
                f"metadata value mismatch for '{key_str}': expected '{expected_str}', got '{actual}'"
            )

    if "obs_dim" in custom_metadata:
        parsed = parse_int_metadata(custom_metadata["obs_dim"])
        obs_dim = as_int(model_cfg.get("obs_dim"), 0)
        if parsed is None:
            add_metadata_issue(f"metadata 'obs_dim' is not an integer: '{custom_metadata['obs_dim']}'")
        elif obs_dim > 0 and parsed != obs_dim:
            add_metadata_issue(f"metadata 'obs_dim' mismatch: expected {obs_dim}, got {parsed}")

    if "action_dim" in custom_metadata:
        parsed = parse_int_metadata(custom_metadata["action_dim"])
        action_dim = as_int(model_cfg.get("action_dim"), 0)
        if parsed is None:
            add_metadata_issue(
                f"metadata 'action_dim' is not an integer: '{custom_metadata['action_dim']}'"
            )
        elif action_dim > 0 and parsed != action_dim:
            add_metadata_issue(f"metadata 'action_dim' mismatch: expected {action_dim}, got {parsed}")

    obs_stack = as_int(model_cfg.get("obs_stack_N"), 0)
    if "obs_stack_n" in custom_metadata:
        parsed = parse_int_metadata(custom_metadata["obs_stack_n"])
        if parsed is None:
            add_metadata_issue(
                f"metadata 'obs_stack_n' is not an integer: '{custom_metadata['obs_stack_n']}'"
            )
        elif obs_stack > 0 and parsed != obs_stack:
            add_metadata_issue(f"metadata 'obs_stack_n' mismatch: expected {obs_stack}, got {parsed}")
    if "obs_stack_N" in custom_metadata:
        parsed = parse_int_metadata(custom_metadata["obs_stack_N"])
        if parsed is None:
            add_metadata_issue(
                f"metadata 'obs_stack_N' is not an integer: '{custom_metadata['obs_stack_N']}'"
            )
        elif obs_stack > 0 and parsed != obs_stack:
            add_metadata_issue(f"metadata 'obs_stack_N' mismatch: expected {obs_stack}, got {parsed}")

    onnx_inputs = to_list(model_cfg.get("onnx_inputs"))
    if not onnx_inputs and "obs_input_name" in custom_metadata:
        expected = str(model_cfg.get("obs_input_name", "obs")).strip()
        actual = str(custom_metadata["obs_input_name"]).strip()
        if expected and actual != expected:
            add_metadata_issue(
                f"metadata 'obs_input_name' mismatch: expected '{expected}', got '{actual}'"
            )
    if "action_output_name" in custom_metadata:
        expected = str(model_cfg.get("action_output_name", "actions")).strip()
        actual = str(custom_metadata["action_output_name"]).strip()
        if expected and actual != expected:
            add_metadata_issue(
                f"metadata 'action_output_name' mismatch: expected '{expected}', got '{actual}'"
            )


def normalize_runtime_shape(shape: Sequence[Any]) -> List[int]:
    normalized: List[int] = []
    for dim in list(shape or []):
        if isinstance(dim, int) and dim > 0:
            normalized.append(dim)
        else:
            normalized.append(1)
    return normalized or [1]


def element_count_from_shape(shape: Sequence[int]) -> int:
    count = 1
    for dim in shape:
        count *= max(int(dim), 1)
    return count


def validate_shape_list(shape_value: Any, issues: IssueCollector, context: str, field_name: str) -> List[int]:
    raw = to_list(shape_value)
    if not raw:
        issues.error(context, f"{field_name} must be a non-empty list")
        return []
    parsed: List[int] = []
    for idx, value in enumerate(raw):
        try:
            dim = int(value)
        except Exception:
            issues.error(context, f"{field_name}[{idx}] is not an integer: {value}")
            continue
        if dim <= 0:
            issues.error(context, f"{field_name}[{idx}] must be > 0")
            continue
        parsed.append(dim)
    return parsed


def check_onnx_contract(
    model_path: Path,
    model_cfg: Dict[str, Any],
    issues: IssueCollector,
    context: str,
    skip_onnx: bool,
) -> None:
    strict_model_io = as_bool(model_cfg.get("strict_model_io", False), False)
    obs_input_name = str(model_cfg.get("obs_input_name", "obs"))
    action_output_name = str(model_cfg.get("action_output_name", "actions"))
    enable_time_step_input = as_bool(model_cfg.get("enable_time_step_input", False), False)
    time_step_input_name = str(model_cfg.get("time_step_input_name", "time_step"))
    action_dim = as_int(model_cfg.get("action_dim"), 0)
    obs_dim = as_int(model_cfg.get("obs_dim"), 0)
    obs_stack_n = as_int(model_cfg.get("obs_stack_N"), 0)
    feature_dims = to_dict(model_cfg.get("feature_dims"))
    configured_inputs = [to_dict(x) for x in to_list(model_cfg.get("onnx_inputs"))]

    session = None
    inputs: List[Any] = []
    outputs: List[Any] = []
    input_names: List[str] = []
    output_names: List[str] = []

    if not skip_onnx:
        if ort is None:
            issues.error(context, "onnxruntime is not installed. Install with: pip install onnxruntime")
            return
        if not model_path.exists():
            issues.error(context, f"ONNX model file not found: {model_path}")
            return

        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = max(1, as_int(model_cfg.get("onnx_intra_threads", 1), 1))
        session_options.inter_op_num_threads = max(1, as_int(model_cfg.get("onnx_inter_threads", 1), 1))

        try:
            session = ort.InferenceSession(
                str(model_path),
                sess_options=session_options,
                providers=["CPUExecutionProvider"],
            )
        except Exception:
            try:
                session = ort.InferenceSession(str(model_path), sess_options=session_options)
            except Exception as exc:
                issues.error(context, f"failed to load ONNX model: {exc}")
                return

        inputs = list(session.get_inputs())
        outputs = list(session.get_outputs())
        if not inputs or not outputs:
            issues.error(context, "invalid ONNX model IO count")
            return

        input_names = [x.name for x in inputs]
        output_names = [x.name for x in outputs]

    if configured_inputs:
        seen_configured_inputs = set()
        for idx, spec in enumerate(configured_inputs):
            spec_context = f"{context} onnx_inputs[{idx}]"
            name = str(spec.get("name", "")).strip()
            source = normalize_token(spec.get("source", "stacked_observation"))
            fill_policy = normalize_token(spec.get("fill_policy", "error"))
            feature_name = str(spec.get("feature_name", "")).strip()
            constant_values: List[float] = []
            for const_idx, value in enumerate(to_list(spec.get("constant"))):
                try:
                    constant_values.append(float(value))
                except Exception:
                    issues.error(
                        spec_context,
                        f"constant[{const_idx}] is not numeric: {value}",
                    )

            if not name:
                issues.error(spec_context, "missing name")
                continue
            if name in seen_configured_inputs:
                issues.error(spec_context, f"duplicate input binding name '{name}'")
            seen_configured_inputs.add(name)

            if source not in SUPPORTED_ONNX_INPUT_SOURCES:
                issues.error(
                    spec_context,
                    f"source must be one of {sorted(SUPPORTED_ONNX_INPUT_SOURCES)}",
                )

            if fill_policy not in SUPPORTED_FILL_POLICIES:
                issues.error(
                    spec_context,
                    f"fill_policy must be one of {sorted(SUPPORTED_FILL_POLICIES)}",
                )

            if source == "feature" and not feature_name:
                issues.error(spec_context, "feature source requires non-empty feature_name")
            if source != "feature" and feature_name:
                issues.warn(spec_context, "feature_name is ignored unless source='feature'")

            explicit_shape: List[int] = []
            if "shape" in spec:
                explicit_shape = validate_shape_list(spec.get("shape"), issues, spec_context, "shape")

            resolved_shape = list(explicit_shape)
            model_input = None
            if input_names:
                if name not in input_names:
                    issues.error(spec_context, f"configured input '{name}' not found in model inputs")
                    continue
                model_input = inputs[input_names.index(name)]
                model_shape = normalize_runtime_shape(list(model_input.shape or []))
                if explicit_shape:
                    if len(explicit_shape) != len(model_shape):
                        issues.error(
                            spec_context,
                            f"shape rank {len(explicit_shape)} does not match model rank {len(model_shape)}",
                        )
                    else:
                        for dim_idx, (cfg_dim, model_dim) in enumerate(zip(explicit_shape, model_shape)):
                            if model_dim > 0 and cfg_dim != model_dim:
                                issues.error(
                                    spec_context,
                                    f"shape[{dim_idx}]={cfg_dim} does not match model shape[{dim_idx}]={model_dim}",
                                )
                else:
                    resolved_shape = model_shape
            elif not explicit_shape:
                issues.warn(spec_context, "shape omitted; validator cannot check tensor size without loading ONNX")

            if not resolved_shape:
                continue

            target_count = element_count_from_shape(resolved_shape)
            expected_source_count: Optional[int] = None
            if source == "stacked_observation":
                if obs_dim > 0 and obs_stack_n > 0:
                    expected_source_count = obs_dim * obs_stack_n
            elif source == "observation":
                if obs_dim > 0:
                    expected_source_count = obs_dim
            elif source == "time_step":
                expected_source_count = 1
            elif source == "feature":
                if feature_name in feature_dims:
                    expected_source_count = as_int(feature_dims[feature_name], 0)
                else:
                    issues.warn(
                        spec_context,
                        f"feature_name '{feature_name}' is not in known feature dim map; size cannot be checked statically",
                    )

            if source == "constant":
                if not constant_values and fill_policy != "zero":
                    issues.error(spec_context, "constant source requires constant values or fill_policy='zero'")
                if constant_values and len(constant_values) > target_count:
                    issues.error(
                        spec_context,
                        f"constant values length {len(constant_values)} exceeds target tensor size {target_count}",
                    )
                if constant_values and len(constant_values) < target_count and fill_policy != "zero":
                    issues.error(
                        spec_context,
                        f"constant values length {len(constant_values)} is smaller than target tensor size {target_count}",
                    )
            elif expected_source_count is not None and expected_source_count > 0:
                if expected_source_count != target_count:
                    issues.error(
                        spec_context,
                        f"source '{source}' provides {expected_source_count} values but target tensor size is {target_count}",
                    )

        if input_names:
            missing_inputs = [name for name in input_names if name not in seen_configured_inputs]
            if missing_inputs:
                issues.error(
                    context,
                    "onnx_inputs does not cover model inputs: " + ", ".join(missing_inputs),
                )
    else:
        if not obs_input_name.strip():
            issues.error(context, "obs_input_name must be non-empty when onnx_inputs is not configured")
        if enable_time_step_input and not time_step_input_name.strip():
            issues.error(context, "time_step_input_name must be non-empty when enable_time_step_input=true")

        if input_names:
            obs_input_index = input_names.index(obs_input_name) if obs_input_name in input_names else -1
            if obs_input_index < 0 and not strict_model_io:
                obs_input_index = 0
                issues.warn(
                    context,
                    f"obs input '{obs_input_name}' not found, fallback to first input '{input_names[0]}'",
                )
            if obs_input_index < 0:
                issues.error(context, f"obs input '{obs_input_name}' not found")

            timestep_input_index = (
                input_names.index(time_step_input_name) if time_step_input_name in input_names else -1
            )
            if enable_time_step_input and timestep_input_index < 0:
                issues.error(context, f"time_step input '{time_step_input_name}' not found")
            if timestep_input_index == obs_input_index:
                timestep_input_index = -1

            unknown_inputs: List[str] = []
            for idx, name in enumerate(input_names):
                if idx == obs_input_index or idx == timestep_input_index:
                    continue
                unknown_inputs.append(name)
            if unknown_inputs:
                message = "unknown model inputs: " + ", ".join(unknown_inputs)
                if strict_model_io:
                    issues.error(context, message)
                else:
                    issues.warn(context, message + " (runtime fills zeros)")

    if output_names:
        action_output_index = output_names.index(action_output_name) if action_output_name in output_names else -1
        if action_output_index < 0 and not strict_model_io:
            action_output_index = 0
            issues.warn(
                context,
                f"action output '{action_output_name}' not found, fallback to first output '{output_names[0]}'",
            )
        if action_output_index < 0:
            issues.error(context, f"action output '{action_output_name}' not found")

        for name in [str(x) for x in to_list(model_cfg.get("extra_output_names"))]:
            if name and name not in output_names:
                if strict_model_io:
                    issues.error(context, f"extra output '{name}' not found")
                else:
                    issues.warn(context, f"extra output '{name}' not found (ignored by runtime)")

        if action_output_index >= 0 and action_output_index < len(outputs) and action_dim > 0:
            shape = normalize_runtime_shape(list(outputs[action_output_index].shape or []))
            static_count = element_count_from_shape(shape)
            if static_count < action_dim:
                issues.error(
                    context,
                    f"action output static element count {static_count} is smaller than action_dim {action_dim}",
                )
            elif static_count != action_dim:
                issues.warn(
                    context,
                    f"action output static element count {static_count} differs from action_dim {action_dim}",
                )

    if session is None:
        return

    metadata_cfg = {
        "enable_metadata_check": model_cfg.get("enable_metadata_check", False),
        "metadata_check_strict": model_cfg.get("metadata_check_strict", True),
        "required_metadata_keys": model_cfg.get("required_metadata_keys", []),
        "expected_metadata": model_cfg.get("expected_metadata", {}),
        "obs_dim": model_cfg.get("obs_dim", 0),
        "action_dim": model_cfg.get("action_dim", 0),
        "obs_stack_N": model_cfg.get("obs_stack_N", 0),
        "obs_input_name": model_cfg.get("obs_input_name", "obs"),
        "action_output_name": model_cfg.get("action_output_name", "actions"),
        "onnx_inputs": model_cfg.get("onnx_inputs", []),
    }
    custom_metadata = get_model_metadata_map(session)
    check_metadata_contract(custom_metadata, metadata_cfg, issues, context)


def merge_policy_io(base_cfg: Dict[str, Any], node: Dict[str, Any]) -> Dict[str, Any]:
    sub_io = to_dict(node.get("policy_io", node))
    merged = dict(base_cfg)
    merged["obs_input_name"] = str(sub_io.get("obs_input_name", merged.get("obs_input_name", "obs")))
    merged["action_output_name"] = str(
        sub_io.get("action_output_name", merged.get("action_output_name", "actions"))
    )
    merged["time_step_input_name"] = str(
        sub_io.get("time_step_input_name", merged.get("time_step_input_name", "time_step"))
    )
    merged["enable_time_step_input"] = as_bool(
        sub_io.get("enable_time_step_input", merged.get("enable_time_step_input", False)),
        False,
    )
    merged["strict_model_io"] = as_bool(
        sub_io.get("strict_model_io", merged.get("strict_model_io", False)),
        False,
    )
    merged["extra_output_names"] = list(
        sub_io.get("extra_output_names", merged.get("extra_output_names", []))
    )
    if "onnx_inputs" in sub_io:
        merged["onnx_inputs"] = to_list(sub_io.get("onnx_inputs"))
    merged["enable_metadata_check"] = as_bool(
        sub_io.get("enable_metadata_check", merged.get("enable_metadata_check", False)),
        False,
    )
    merged["metadata_check_strict"] = as_bool(
        sub_io.get("metadata_check_strict", merged.get("metadata_check_strict", True)),
        True,
    )
    merged["required_metadata_keys"] = list(
        sub_io.get("required_metadata_keys", merged.get("required_metadata_keys", []))
    )
    if "expected_metadata" in sub_io:
        merged["expected_metadata"] = to_dict(sub_io.get("expected_metadata"))
    return merged


def validate_profile(
    root_cfg: Dict[str, Any],
    root_dir: Path,
    mode_id: int,
    section_name: str,
    tag: str,
    issues: IssueCollector,
    skip_onnx: bool,
) -> None:
    context = f"profile(mode_id={mode_id}, tag={tag}, section={section_name})"
    section_cfg = to_dict(root_cfg.get(section_name))
    if not section_cfg:
        issues.error(context, f"config section not found: '{section_name}'")
        return

    obs_dim = as_int(section_cfg.get("obs_dim"), 0)
    action_dim = as_int(section_cfg.get("action_dim"), 0)
    motor_n = as_int(section_cfg.get("motor_N"), 0)
    obs_stack_n = as_int(section_cfg.get("obs_stack_N"), 0)

    if obs_stack_n <= 0:
        issues.error(context, "obs_stack_N must be > 0")
    if obs_dim <= 0:
        issues.error(context, "obs_dim must be > 0")

    action_order = [str(x) for x in to_list(section_cfg.get("action_joint_order"))]
    obs_order = [str(x) for x in to_list(section_cfg.get("obs_joint_order"))]
    check_joint_order(action_dim, motor_n, action_order, obs_order, issues, context)
    reference_order = check_reference_joint_order(
        motor_n,
        action_order,
        [str(x) for x in to_list(section_cfg.get("reference_joint_order"))],
        issues,
        context,
    )
    check_source_contract(section_cfg, issues, context)
    check_reference_contract(section_cfg, root_dir, reference_order, issues, context)
    feature_dims = build_feature_dim_map(section_cfg, reference_order)

    manifest_path = get_manifest_path(section_cfg, root_dir)
    manifest_dim = parse_manifest_dim(manifest_path, issues, context)
    if obs_dim > 0 and manifest_dim > 0 and manifest_dim != obs_dim:
        issues.error(
            context,
            f"manifest dim {manifest_dim} does not match cfg obs_dim {obs_dim}. manifest={manifest_path}",
        )

    policy_io_cfg = to_dict(section_cfg.get("policy_io", section_cfg))
    main_model_cfg = {
        "obs_input_name": str(policy_io_cfg.get("obs_input_name", "obs")),
        "action_output_name": str(policy_io_cfg.get("action_output_name", "actions")),
        "enable_time_step_input": as_bool(policy_io_cfg.get("enable_time_step_input", False), False),
        "time_step_input_name": str(policy_io_cfg.get("time_step_input_name", "time_step")),
        "strict_model_io": as_bool(policy_io_cfg.get("strict_model_io", False), False),
        "extra_output_names": [str(x) for x in to_list(policy_io_cfg.get("extra_output_names"))],
        "onnx_inputs": to_list(policy_io_cfg.get("onnx_inputs")),
        "enable_metadata_check": as_bool(policy_io_cfg.get("enable_metadata_check", False), False),
        "metadata_check_strict": as_bool(policy_io_cfg.get("metadata_check_strict", True), True),
        "required_metadata_keys": [str(x) for x in to_list(policy_io_cfg.get("required_metadata_keys"))],
        "expected_metadata": to_dict(policy_io_cfg.get("expected_metadata")),
        "obs_dim": obs_dim,
        "action_dim": action_dim,
        "obs_stack_N": obs_stack_n,
        "feature_dims": feature_dims,
        "onnx_intra_threads": as_int(section_cfg.get("onnx_intra_threads", 1), 1),
        "onnx_inter_threads": as_int(section_cfg.get("onnx_inter_threads", 1), 1),
    }

    policy_path = get_policy_file_path(section_cfg, root_dir)
    check_onnx_contract(policy_path, main_model_cfg, issues, context + " main_model", skip_onnx)

    for spec in to_list(section_cfg.get("external_observations")):
        node = to_dict(spec)
        name = str(node.get("name", ""))
        required = as_bool(node.get("required", False), False)
        if name and required:
            issues.warn(
                context,
                f"external_observations '{name}' sets required=true, but runtime currently pads missing data with zeros",
            )

    base_sub_cfg = dict(main_model_cfg)
    for idx, raw_sub in enumerate(to_list(section_cfg.get("sub_models"))):
        node = to_dict(raw_sub)
        if not as_bool(node.get("enabled", True), True):
            continue
        sub_name = str(node.get("name", f"sub_model_{idx}"))
        sub_context = context + f" sub_model[{sub_name}]"
        sub_cfg = merge_policy_io(base_sub_cfg, node)
        sub_action_dim = as_int(node.get("action_dim", -1), -1)
        if sub_action_dim > 0:
            sub_cfg["action_dim"] = sub_action_dim

        sub_path_raw = str(node.get("policy_path", ""))
        sub_file_raw = str(node.get("policy_file", ""))
        if sub_path_raw:
            sub_path = resolve_path(sub_path_raw, root_dir)
        elif sub_file_raw:
            sub_path = resolve_path(sub_file_raw, root_dir)
        else:
            issues.error(sub_context, "missing policy_path/policy_file")
            continue
        check_onnx_contract(sub_path, sub_cfg, issues, sub_context, skip_onnx)

    amp_node = to_dict(section_cfg.get("amp_discriminator"))
    if as_bool(amp_node.get("enabled", False), False):
        amp_context = context + " amp_discriminator"
        amp_io = to_dict(amp_node.get("policy_io", amp_node))
        amp_cfg = {
            "obs_input_name": str(amp_io.get("obs_input_name", "obs")),
            "action_output_name": str(amp_io.get("score_output_name", "disc_score")),
            "enable_time_step_input": as_bool(amp_io.get("enable_time_step_input", False), False),
            "time_step_input_name": str(amp_io.get("time_step_input_name", "time_step")),
            "strict_model_io": as_bool(amp_io.get("strict_model_io", False), False),
            "extra_output_names": [str(x) for x in to_list(amp_io.get("extra_output_names"))],
            "onnx_inputs": to_list(amp_io.get("onnx_inputs")),
            "enable_metadata_check": as_bool(amp_io.get("enable_metadata_check", False), False),
            "metadata_check_strict": as_bool(amp_io.get("metadata_check_strict", True), True),
            "required_metadata_keys": [str(x) for x in to_list(amp_io.get("required_metadata_keys"))],
            "expected_metadata": to_dict(amp_io.get("expected_metadata")),
            "obs_dim": obs_dim,
            "action_dim": 0,
            "obs_stack_N": obs_stack_n,
            "feature_dims": feature_dims,
            "onnx_intra_threads": as_int(section_cfg.get("onnx_intra_threads", 1), 1),
            "onnx_inter_threads": as_int(section_cfg.get("onnx_inter_threads", 1), 1),
        }
        amp_path_raw = str(amp_node.get("policy_path", ""))
        amp_file_raw = str(amp_node.get("policy_file", ""))
        if amp_path_raw:
            amp_path = resolve_path(amp_path_raw, root_dir)
        elif amp_file_raw:
            amp_path = resolve_path(amp_file_raw, root_dir)
        else:
            issues.error(amp_context, "enabled but missing policy_path/policy_file")
            amp_path = None
        if amp_path is not None:
            check_onnx_contract(amp_path, amp_cfg, issues, amp_context, skip_onnx)


def build_arg_parser(default_cfg: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate rl_cfg.yaml + observation manifest + ONNX model contracts."
    )
    parser.add_argument(
        "--rl-cfg",
        default=str(default_cfg),
        help="Path to rl_cfg.yaml",
    )
    parser.add_argument(
        "--mode-id",
        dest="mode_ids",
        action="append",
        type=int,
        default=[],
        help="Validate specific mode_id (repeatable).",
    )
    parser.add_argument(
        "--config-section",
        dest="sections",
        action="append",
        default=[],
        help="Validate specific config section name (repeatable).",
    )
    parser.add_argument(
        "--skip-onnx",
        action="store_true",
        help="Skip ONNX file/session checks (YAML/manifest only).",
    )
    parser.add_argument(
        "--fail-on-warning",
        action="store_true",
        help="Return non-zero when warnings exist.",
    )
    return parser


def main() -> int:
    default_cfg = Path(__file__).resolve().parents[2] / "config" / "rl_cfg.yaml"
    parser = build_arg_parser(default_cfg)
    args = parser.parse_args()

    cfg_path = Path(args.rl_cfg).expanduser().resolve()
    if not cfg_path.exists():
        print(f"[ERROR] rl_cfg path not found: {cfg_path}")
        return 2

    issues = IssueCollector()
    try:
        root_cfg = load_rl_cfg(cfg_path)
    except Exception as exc:
        print(f"[ERROR] failed to parse rl_cfg: {exc}")
        return 2

    root_dir_raw = str(root_cfg.get("humanoid_rl_root_dir", ""))
    if not root_dir_raw:
        issues.warn("global", "missing humanoid_rl_root_dir in rl_cfg, fallback to local rl_master path")
        root_dir = cfg_path.parent.parent
    else:
        configured_root = resolve_path(root_dir_raw, cfg_path.parent)
        if configured_root.exists():
            root_dir = configured_root
        else:
            root_dir = cfg_path.parent.parent
            issues.warn(
                "global",
                f"configured humanoid_rl_root_dir does not exist: {configured_root}. "
                f"fallback to local path: {root_dir}",
            )

    specs = get_mode_profile_specs(root_cfg)
    if not specs:
        issues.error("global", "no deploy mode profiles resolved")

    selected_specs: List[Tuple[int, str, str]] = []
    for mode_id, section, tag in specs:
        if args.mode_ids and mode_id not in args.mode_ids:
            continue
        if args.sections and section not in args.sections:
            continue
        selected_specs.append((mode_id, section, tag))

    if not selected_specs:
        issues.error("global", "no profiles matched the filters")

    for mode_id, section, tag in selected_specs:
        validate_profile(
            root_cfg=root_cfg,
            root_dir=root_dir,
            mode_id=mode_id,
            section_name=section,
            tag=tag,
            issues=issues,
            skip_onnx=args.skip_onnx,
        )

    issues.print()
    print(
        f"[SUMMARY] errors={issues.error_count}, warnings={issues.warning_count}, profiles={len(selected_specs)}"
    )

    if issues.error_count > 0:
        return 2
    if args.fail_on_warning and issues.warning_count > 0:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
