#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib
import math
import os
import subprocess
import sys
import time
import types
from bisect import bisect_right
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

try:
    import mujoco
    import mujoco.viewer
except Exception as exc:  # pragma: no cover - runtime dependency
    raise RuntimeError(f"mujoco python import failed: {exc}") from exc


def install_compression_fallbacks() -> None:
    try:
        import zstandard  # type: ignore # noqa: F401
    except Exception:

        class _ZstdDecompressor:
            def decompress(self, payload: bytes) -> bytes:
                return subprocess.run(
                    ["/usr/bin/zstd", "-d", "-q", "-c"],
                    input=payload,
                    stdout=subprocess.PIPE,
                    check=True,
                ).stdout

        zstd_module = types.ModuleType("zstandard")
        zstd_module.ZstdDecompressor = _ZstdDecompressor
        sys.modules["zstandard"] = zstd_module

    try:
        import lz4.frame  # type: ignore # noqa: F401
    except Exception:
        lz4_module = types.ModuleType("lz4")
        frame_module = types.ModuleType("lz4.frame")
        frame_module.decompress = lambda _payload: (_ for _ in ()).throw(
            RuntimeError("lz4 Python module is unavailable; only none/zstd logs can be read")
        )
        lz4_module.frame = frame_module
        sys.modules["lz4"] = lz4_module
        sys.modules["lz4.frame"] = frame_module


install_compression_fallbacks()

REPO_ROOT = Path(__file__).resolve().parents[4]
RL_MASTER_ROOT = REPO_ROOT / "src/omnimorph_rl_controller/rl_master"
sys.path.insert(0, str(RL_MASTER_ROOT / "tools/analysis"))
from runtime_log_utils import load_runtime_messages  # noqa: E402


DEFAULT_MCAP = RL_MASTER_ROOT / "data/runtime_logs/Jun12_09-46-01_beyondmimic_jc01_dance_wo_state_estimation.mcap"
DEFAULT_RL_CONFIG = RL_MASTER_ROOT / "config/rl_cfg_jc01.yaml"
DEFAULT_SIM_CONFIG = REPO_ROOT / "src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_fullbody_engineai_walk_sim2sim.yaml"


def import_pinocchio():
    errors: List[str] = []
    for module_name in ("pinocchio", "pinocchio.pinocchio_pywrap_default"):
        try:
            pin = importlib.import_module(module_name)
        except Exception as exc:
            errors.append(f"{module_name}: {exc}")
            continue
        if all(hasattr(pin, attr) for attr in ("JointModelFreeFlyer", "neutral", "centerOfMass")):
            return pin
        errors.append(
            f"{module_name}: imported from {getattr(pin, '__file__', '<unknown>')} "
            "but does not look like Pinocchio robotics bindings"
        )
    try:
        import pinocchio as pin  # type: ignore
    except ImportError as exc:
        raise RuntimeError(
            "Python Pinocchio is required for COM reconstruction. "
            "Install/source Pinocchio Python bindings and rerun this replay tool."
        ) from exc
    raise RuntimeError(
        "Imported a 'pinocchio' module, but it does not expose the expected robotics API. "
        "Check that you are not loading the unrelated pip package. Details: " + "; ".join(errors)
    )


def build_pinocchio_model(pin: Any, urdf_path: Path) -> Any:
    if hasattr(pin, "buildModelFromUrdf"):
        return pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    try:
        from pinocchio.robot_wrapper import RobotWrapper  # type: ignore

        robot = RobotWrapper.BuildFromURDF(
            str(urdf_path),
            [],
            pin.JointModelFreeFlyer(),
        )
        return robot.model
    except Exception as exc:
        raise RuntimeError(
            "Pinocchio Python binding does not provide buildModelFromUrdf or "
            f"RobotWrapper.BuildFromURDF. Loaded module: {getattr(pin, '__file__', '<unknown>')}"
        ) from exc


def load_yaml(path: Path) -> Dict[str, Any]:
    content = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(content, dict):
        raise RuntimeError(f"YAML root is not a mapping: {path}")
    return content


def expand_vars(text: str, variables: Dict[str, str]) -> str:
    out = text
    for _ in range(8):
        before = out
        for key, value in variables.items():
            out = out.replace("${" + key + "}", value)
        out = os.path.expandvars(out)
        if out == before:
            break
    return out


def resolve_relative(path_text: str, anchor: Path) -> Path:
    path = Path(path_text).expanduser()
    return path if path.is_absolute() else (anchor / path).resolve()


def root_path_variables(root_config: Dict[str, Any]) -> Dict[str, str]:
    variables: Dict[str, str] = {key: value for key, value in os.environ.items()}
    raw = root_config.get("path_variables")
    if isinstance(raw, dict):
        for key, value in raw.items():
            variables[str(key)] = expand_vars(str(value), variables)
    return variables


def resolve_profile(mcap_path: Path, rl_config_path: Path, explicit_profile: str) -> Tuple[str, Dict[str, Any], Dict[str, Any]]:
    root = load_yaml(rl_config_path)
    section = explicit_profile.strip()
    if not section:
        config_messages = load_runtime_messages(mcap_path, "runtime/config")
        if config_messages and isinstance(config_messages[0].get("data"), dict):
            section = str(config_messages[0]["data"].get("config_section", "")).strip()
    if not section:
        raise RuntimeError("Could not infer runtime profile; pass --profile")

    config_files = root.get("config_files")
    if not isinstance(config_files, dict) or section not in config_files:
        raise RuntimeError(f"Profile '{section}' is not listed in {rl_config_path}")
    profile_path = resolve_relative(str(config_files[section]), rl_config_path.parent)
    profile_doc = load_yaml(profile_path)
    profile = profile_doc.get(section)
    if not isinstance(profile, dict):
        raise RuntimeError(f"Profile file {profile_path} does not contain section '{section}'")
    return section, profile, root


def resolve_model_path(sim_config_path: Path, rl_config_path: Path, override: str) -> Path:
    if override:
        return Path(override).expanduser().resolve()

    sim_doc = load_yaml(sim_config_path)
    params = sim_doc.get("mujoco_sim_bridge", {}).get("ros__parameters", {})
    if not isinstance(params, dict):
        raise RuntimeError(f"Could not read mujoco_sim_bridge.ros__parameters from {sim_config_path}")
    raw_model_path = str(params.get("model_path", "")).strip()
    if not raw_model_path:
        raise RuntimeError("model_path is empty; pass --model")

    root = load_yaml(rl_config_path)
    variables = root_path_variables(root)
    expanded = expand_vars(raw_model_path, variables)
    fallback_models = [
        REPO_ROOT / "src/omnimorph_sim2sim/mujoco_sim2sim/models/jc01_fullbody_engineai_walk_sim2sim.xml",
        REPO_ROOT / "src/omnimorph_sim2sim/mujoco_sim2sim/models/jc01_fullbody_engineai_walk_passive_check.xml",
    ]
    if "${" in expanded:
        for candidate in fallback_models:
            if candidate.exists():
                return candidate.resolve()
    resolved = Path(expanded).expanduser()
    if resolved.is_absolute():
        if resolved.exists():
            return resolved
        for candidate in fallback_models:
            if candidate.exists():
                return candidate.resolve()
        return resolved
    root_dir = Path(str(root.get("omnimorph_root_dir", "..")))
    if not root_dir.is_absolute():
        root_dir = (rl_config_path.parent / root_dir).resolve()
    resolved = (root_dir / resolved).resolve()
    if resolved.exists():
        return resolved
    for candidate in fallback_models:
        if candidate.exists():
            return candidate.resolve()
    return resolved


def resolve_urdf_path(
    profile: Dict[str, Any],
    root_config: Dict[str, Any],
    rl_config_path: Path,
    override: str,
) -> Path:
    candidates: List[Path] = []
    if override:
        candidates.append(Path(override).expanduser())
    variables = root_path_variables(root_config)
    profile_urdf = str(profile.get("pinocchio_urdf_path", "")).strip()
    if profile_urdf:
        candidates.append(resolve_relative(expand_vars(profile_urdf, variables), rl_config_path.parent))
    assets_dir = variables.get("ROBOT_ASSETS_DIR") or variables.get("robot_assets_dir")
    if assets_dir:
        candidates.append(Path(assets_dir).expanduser() / "JC01-URDF.urdf")
    candidates.extend(
        [
            Path("/home/edify/Code/jc01-model/JC01-URDF.urdf"),
            Path("/home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/JC01-URDF.urdf"),
        ]
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise RuntimeError("Could not find Pinocchio URDF; pass --urdf")


def resolve_joint_order(root_config: Dict[str, Any], profile: Dict[str, Any]) -> List[str]:
    for key, source in (
        ("robot_global_joint_order", root_config),
        ("reference_joint_order", profile),
        ("obs_joint_order", profile),
        ("action_joint_order", profile),
    ):
        value = source.get(key)
        if isinstance(value, list) and value:
            return [str(item) for item in value]
    raise RuntimeError("Could not resolve joint order from config")


def normalized_xyzw(quat: Sequence[float]) -> np.ndarray:
    values = np.asarray(quat[:4], dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not math.isfinite(norm) or norm < 1.0e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return values / norm


def finite_vector(value: Any, minimum_len: int) -> Optional[List[float]]:
    if not isinstance(value, list) or len(value) < minimum_len:
        return None
    out: List[float] = []
    for item in value:
        try:
            number = float(item)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        out.append(number)
    return out


def selected_base_imu_quats(mcap_path: Path) -> List[Tuple[float, np.ndarray]]:
    messages = load_runtime_messages(mcap_path, "runtime/source/base_imu")
    samples: List[Tuple[float, np.ndarray]] = []
    for message in messages:
        data = message.get("data")
        if not isinstance(data, dict):
            continue
        try:
            timestamp = float(data.get("monotonic_time_sec"))
        except (TypeError, ValueError):
            continue
        values = data.get("values")
        if not isinstance(values, dict):
            continue
        quat_xyzw = finite_vector(values.get("quat_xyzw"), 4)
        if quat_xyzw is None:
            continue
        samples.append((timestamp, normalized_xyzw(quat_xyzw)))
    return samples


def align_base_imu_to_ticks(
    ticks: Sequence[Dict[str, Any]],
    imu_samples: Sequence[Tuple[float, np.ndarray]],
) -> int:
    if not ticks or not imu_samples:
        return 0
    imu_times = [sample[0] for sample in imu_samples]
    aligned = 0
    for tick in ticks:
        try:
            tick_time = float(tick.get("monotonic_time_sec"))
        except (TypeError, ValueError):
            continue
        sample_idx = bisect_right(imu_times, tick_time) - 1
        if sample_idx < 0:
            continue
        tick["_aligned_base_imu_quat_xyzw"] = imu_samples[sample_idx][1]
        aligned += 1
    return aligned


def selected_ticks(mcap_path: Path, running_only: bool, joint_field: str, stride: int, max_samples: int) -> List[Dict[str, Any]]:
    messages = load_runtime_messages(mcap_path, "runtime/tick")
    ticks: List[Dict[str, Any]] = []
    for message in messages:
        data = message.get("data")
        if not isinstance(data, dict):
            continue
        if running_only and int(data.get("deploy_state", 0)) != 3:
            continue
        if finite_vector(data.get(joint_field), 1) is None:
            continue
        ticks.append(data)
    ticks = ticks[:: max(1, stride)]
    if max_samples > 0 and len(ticks) > max_samples:
        indices = np.linspace(0, len(ticks) - 1, max_samples, dtype=int)
        ticks = [ticks[int(i)] for i in indices]
    return ticks


def pin_property(value: Any) -> Any:
    return value() if callable(value) else value


class PinocchioCom:
    def __init__(self, pin: Any, urdf_path: Path, joint_order: Sequence[str]) -> None:
        self.pin = pin
        self.model = build_pinocchio_model(pin, urdf_path)
        self.data = self.model.createData()
        self.neutral = pin.neutral(self.model)
        self.q_indices: List[int] = []
        missing: List[str] = []
        for name in joint_order:
            if not self.model.existJointName(name):
                missing.append(name)
                continue
            joint = self.model.joints[self.model.getJointId(name)]
            if int(pin_property(getattr(joint, "nq"))) != 1:
                raise RuntimeError(f"Pinocchio joint '{name}' is not 1-DOF")
            self.q_indices.append(int(pin_property(getattr(joint, "idx_q"))))
        if missing:
            raise RuntimeError("Pinocchio URDF is missing joints: " + ", ".join(missing))

    def compute(self, base_pos: Sequence[float], base_quat_xyzw: Sequence[float], joint_q: Sequence[float]) -> np.ndarray:
        q = self.neutral.copy()
        q[0:3] = np.asarray(base_pos[:3], dtype=np.float64)
        q[3:7] = normalized_xyzw(base_quat_xyzw)
        for src_idx, q_idx in enumerate(self.q_indices):
            if src_idx < len(joint_q):
                q[q_idx] = float(joint_q[src_idx])
        return np.asarray(self.pin.centerOfMass(self.model, self.data, q, False)).reshape(3)


def mujoco_joint_maps(model: mujoco.MjModel, joint_order: Sequence[str]) -> Tuple[List[int], int]:
    qpos_addrs: List[int] = []
    for name in joint_order:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise RuntimeError(f"MuJoCo model is missing joint '{name}'")
        qpos_addrs.append(int(model.jnt_qposadr[joint_id]))

    free_qpos = -1
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            free_qpos = int(model.jnt_qposadr[joint_id])
            break
    return qpos_addrs, free_qpos


def set_pose_from_tick(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    tick: Dict[str, Any],
    joint_field: str,
    qpos_addrs: Sequence[int],
    free_qpos: int,
    base_pos: Sequence[float],
    base_quat_xyzw: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray, List[float]]:
    joint_q = finite_vector(tick.get(joint_field), len(qpos_addrs))
    if joint_q is None:
        raise RuntimeError(f"Tick does not contain usable '{joint_field}'")

    logged_pos = finite_vector(tick.get("base_pos_w"), 3)
    aligned_imu_quat = tick.get("_aligned_base_imu_quat_xyzw")
    logged_quat = finite_vector(tick.get("base_quat"), 4)
    if logged_quat is None:
        logged_quat = finite_vector(tick.get("base_quat_xyzw"), 4)
    pos = np.asarray((logged_pos or list(base_pos))[:3], dtype=np.float64)
    if isinstance(aligned_imu_quat, np.ndarray) and aligned_imu_quat.shape == (4,):
        quat_xyzw = normalized_xyzw(aligned_imu_quat)
    else:
        quat_xyzw = normalized_xyzw((logged_quat or list(base_quat_xyzw))[:4])

    if free_qpos >= 0 and free_qpos + 6 < model.nq:
        data.qpos[free_qpos : free_qpos + 3] = pos
        data.qpos[free_qpos + 3 : free_qpos + 7] = np.array(
            [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64
        )
    for src_idx, qpos_idx in enumerate(qpos_addrs):
        if 0 <= qpos_idx < model.nq:
            data.qpos[qpos_idx] = joint_q[src_idx]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.time = float(tick.get("time_from_start_sec", 0.0))
    mujoco.mj_forward(model, data)
    return pos, quat_xyzw, joint_q


def convex_hull_xy(points: Sequence[np.ndarray]) -> List[np.ndarray]:
    unique = sorted({(round(float(p[0]), 9), round(float(p[1]), 9)) for p in points})
    if len(unique) <= 1:
        return [np.array([x, y, 0.0], dtype=np.float64) for x, y in unique]

    def cross(o: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: List[Tuple[float, float]] = []
    for p in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    upper: List[Tuple[float, float]] = []
    for p in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    return [np.array([x, y, 0.0], dtype=np.float64) for x, y in (lower[:-1] + upper[:-1])]


def support_polygon(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_names: Sequence[str],
    half_length: float,
    half_width: float,
    height_threshold: float,
) -> List[np.ndarray]:
    site_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        for name in site_names
    ]
    site_ids = [site_id for site_id in site_ids if site_id >= 0]
    if not site_ids:
        return []

    min_z = min(float(data.site_xpos[site_id][2]) for site_id in site_ids)
    points: List[np.ndarray] = []
    for site_id in site_ids:
        pos = np.asarray(data.site_xpos[site_id], dtype=np.float64)
        if float(pos[2]) > min_z + height_threshold:
            continue
        mat = np.asarray(data.site_xmat[site_id], dtype=np.float64).reshape(3, 3)
        x_axis = mat[:, 0]
        y_axis = mat[:, 1]
        for sx, sy in ((1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)):
            point = pos + sx * half_length * x_axis + sy * half_width * y_axis
            point[2] = 0.0
            points.append(point)
    return convex_hull_xy(points)


def support_foot_body_ids(model: mujoco.MjModel, site_names: Sequence[str]) -> List[int]:
    body_ids: List[int] = []
    for site_name in site_names:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if site_id < 0:
            continue
        body_id = int(model.site_bodyid[site_id])
        if body_id >= 0:
            body_ids.append(body_id)
    return sorted(set(body_ids))


def contact_frame_to_world(frame: Sequence[float], local: Sequence[float]) -> np.ndarray:
    mat = np.asarray(frame, dtype=np.float64).reshape(3, 3)
    vec = np.asarray(local, dtype=np.float64).reshape(3)
    return mat.T @ vec


def compute_cop(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    support_body_ids: Sequence[int],
) -> Optional[np.ndarray]:
    support_set = set(support_body_ids)
    if not support_set:
        return None

    selected_force = np.zeros(3, dtype=np.float64)
    selected_moment = np.zeros(3, dtype=np.float64)
    selected_force_z_abs = 0.0
    selected_plane_z_weight = 0.0
    selected_plane_z_sum = 0.0

    fallback_force = np.zeros(3, dtype=np.float64)
    fallback_moment = np.zeros(3, dtype=np.float64)
    fallback_force_z_abs = 0.0
    fallback_plane_z_weight = 0.0
    fallback_plane_z_sum = 0.0

    wrench = np.zeros(6, dtype=np.float64)
    for contact_idx in range(int(data.ncon)):
        contact = data.contact[contact_idx]
        if int(contact.geom1) < 0 or int(contact.geom2) < 0:
            continue
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        support1 = body1 in support_set
        support2 = body2 in support_set
        if support1 == support2:
            continue

        wrench[:] = 0.0
        mujoco.mj_contactForce(model, data, contact_idx, wrench)
        force_world = contact_frame_to_world(contact.frame, wrench[:3])
        torque_world = contact_frame_to_world(contact.frame, wrench[3:])
        pos_world = np.asarray(contact.pos, dtype=np.float64)
        moment_world = np.cross(pos_world, force_world) + torque_world
        force_z_abs = abs(float(force_world[2]))
        if force_z_abs <= 1.0e-8:
            continue

        is_ground_contact = body1 == 0 or body2 == 0
        force_acc = selected_force if is_ground_contact else fallback_force
        moment_acc = selected_moment if is_ground_contact else fallback_moment
        if is_ground_contact:
            selected_force_z_abs += force_z_abs
            selected_plane_z_weight += force_z_abs
            selected_plane_z_sum += force_z_abs * float(pos_world[2])
        else:
            fallback_force_z_abs += force_z_abs
            fallback_plane_z_weight += force_z_abs
            fallback_plane_z_sum += force_z_abs * float(pos_world[2])

        force_acc[:] += force_world
        moment_acc[:] += moment_world

    force = selected_force
    moment = selected_moment
    plane_z = selected_plane_z_sum / selected_plane_z_weight if selected_plane_z_weight > 1.0e-8 else 0.0
    if selected_force_z_abs <= 1.0e-8:
        force = fallback_force
        moment = fallback_moment
        if fallback_force_z_abs <= 1.0e-8:
            return None
        plane_z = fallback_plane_z_sum / fallback_plane_z_weight if fallback_plane_z_weight > 1.0e-8 else 0.0

    if abs(float(force[2])) <= 1.0e-8:
        return None
    return np.array(
        [
            (plane_z * force[0] - moment[1]) / force[2],
            (moment[0] + plane_z * force[1]) / force[2],
            plane_z,
        ],
        dtype=np.float64,
    )


def add_marker(
    scene: mujoco.MjvScene,
    geom_type: int,
    size: Sequence[float],
    pos: Sequence[float],
    rgba: Sequence[float],
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(
        scene.geoms[scene.ngeom],
        geom_type,
        np.asarray(size, dtype=np.float64),
        np.asarray(pos, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1


def add_segment(scene: mujoco.MjvScene, start: Sequence[float], end: Sequence[float], width: float, rgba: Sequence[float]) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.array([width, 0.0, 0.0], dtype=np.float64),
        np.zeros(3, dtype=np.float64),
        np.eye(3, dtype=np.float64).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        width,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )
    scene.ngeom += 1


def add_overlay(scene: mujoco.MjvScene, com: np.ndarray, cop: Optional[np.ndarray], polygon: Sequence[np.ndarray]) -> None:
    projection = np.array([com[0], com[1], 0.02], dtype=np.float64)
    add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.035, 0.035, 0.035], com, [1.0, 0.18, 0.05, 1.0])
    add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.025, 0.025, 0.025], projection, [0.05, 0.45, 1.0, 1.0])
    add_segment(scene, com, projection, 0.006, [1.0, 0.18, 0.05, 0.95])
    if cop is not None:
        cop_display = np.array([cop[0], cop[1], 0.02], dtype=np.float64)
        add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.025, 0.025, 0.025], cop_display, [1.0, 0.85, 0.10, 1.0])
    if len(polygon) >= 2:
        lifted = [np.array([p[0], p[1], 0.018], dtype=np.float64) for p in polygon]
        for idx, start in enumerate(lifted):
            add_segment(scene, start, lifted[(idx + 1) % len(lifted)], 0.008, [0.05, 0.85, 0.30, 0.95])
        for vertex in lifted:
            add_marker(scene, mujoco.mjtGeom.mjGEOM_SPHERE, [0.018, 0.018, 0.018], vertex, [0.05, 0.85, 0.30, 1.0])


def parse_vec(text: str, n: int) -> List[float]:
    values = [float(token.strip()) for token in text.split(",") if token.strip()]
    if len(values) != n:
        raise argparse.ArgumentTypeError(f"expected {n} comma-separated values")
    return values


def parse_optional_vec(text: str, n: int) -> Optional[List[float]]:
    if not text.strip():
        return None
    return parse_vec(text, n)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay runtime MCAP motion in MuJoCo with Pinocchio COM/support overlay.")
    parser.add_argument("--mcap", type=Path, default=DEFAULT_MCAP)
    parser.add_argument("--rl-config", type=Path, default=DEFAULT_RL_CONFIG)
    parser.add_argument("--sim-config", type=Path, default=DEFAULT_SIM_CONFIG)
    parser.add_argument("--profile", default="")
    parser.add_argument("--model", default="", help="Override MuJoCo XML/MJB path")
    parser.add_argument("--urdf", default="", help="Override Pinocchio URDF path")
    parser.add_argument("--joint-field", default="joint_q", choices=["joint_q", "joint_state_q", "joint_target_q", "joint_cmd_q"])
    parser.add_argument("--running-only", action="store_true")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-samples", type=int, default=0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument(
        "--base-pos",
        type=lambda s: parse_optional_vec(s, 3),
        default=None,
        help="Fallback base xyz when log has no base_pos_w. Empty uses the MuJoCo model initial freejoint pose.",
    )
    parser.add_argument(
        "--base-quat-xyzw",
        type=lambda s: parse_optional_vec(s, 4),
        default=None,
        help="Fallback base quat xyzw when log has no base_quat. Empty uses the MuJoCo model initial freejoint pose.",
    )
    parser.add_argument("--foot-sites", default="right_foot_site,left_foot_site")
    parser.add_argument("--foot-half-length", type=float, default=0.11)
    parser.add_argument("--foot-half-width", type=float, default=0.055)
    parser.add_argument("--support-height-threshold", type=float, default=0.05)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pin = import_pinocchio()

    mcap_path = args.mcap.resolve()
    rl_config_path = args.rl_config.resolve()
    sim_config_path = args.sim_config.resolve()
    section, profile, root_config = resolve_profile(mcap_path, rl_config_path, args.profile)
    model_path = resolve_model_path(sim_config_path, rl_config_path, args.model)
    urdf_path = resolve_urdf_path(profile, root_config, rl_config_path, args.urdf)
    joint_order = resolve_joint_order(root_config, profile)
    ticks = selected_ticks(mcap_path, args.running_only, args.joint_field, args.stride, args.max_samples)
    if not ticks:
        raise RuntimeError("No runtime/tick samples selected")
    imu_samples = selected_base_imu_quats(mcap_path)
    aligned_imu_count = align_base_imu_to_ticks(ticks, imu_samples)

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    qpos_addrs, free_qpos = mujoco_joint_maps(model, joint_order)
    fallback_base_pos = args.base_pos
    fallback_base_quat_xyzw = args.base_quat_xyzw
    if free_qpos >= 0 and free_qpos + 6 < model.nq:
        if fallback_base_pos is None:
            fallback_base_pos = data.qpos[free_qpos : free_qpos + 3].astype(np.float64).tolist()
        if fallback_base_quat_xyzw is None:
            qw, qx, qy, qz = [float(v) for v in data.qpos[free_qpos + 3 : free_qpos + 7].tolist()]
            fallback_base_quat_xyzw = [qx, qy, qz, qw]
    if fallback_base_pos is None:
        fallback_base_pos = [0.0, 0.0, 0.0]
    if fallback_base_quat_xyzw is None:
        fallback_base_quat_xyzw = [0.0, 0.0, 0.0, 1.0]
    com_solver = PinocchioCom(pin, urdf_path, joint_order)
    foot_sites = [token.strip() for token in args.foot_sites.split(",") if token.strip()]
    foot_body_ids = support_foot_body_ids(model, foot_sites)

    print(f"profile: {section}")
    print(f"mujoco_model: {model_path}")
    print(f"pinocchio_urdf: {urdf_path}")
    print(f"samples: {len(ticks)} joint_field={args.joint_field}")
    print(f"support_foot_body_ids: {foot_body_ids}")
    if imu_samples:
        print(
            "base_orientation_source: runtime/source/base_imu "
            f"(aligned {aligned_imu_count}/{len(ticks)} ticks from {len(imu_samples)} imu samples)"
        )
    else:
        print("base_orientation_source: runtime/tick base_quat/base_quat_xyzw or fallback base quat")

    period = 1.0 / max(1.0, args.fps)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            for tick in ticks:
                begin = time.monotonic()
                base_pos, base_quat_xyzw, joint_q = set_pose_from_tick(
                    model,
                    data,
                    tick,
                    args.joint_field,
                    qpos_addrs,
                    free_qpos,
                    fallback_base_pos,
                    fallback_base_quat_xyzw,
                )
                com = com_solver.compute(base_pos, base_quat_xyzw, joint_q)
                cop = compute_cop(model, data, foot_body_ids)
                polygon = support_polygon(
                    model,
                    data,
                    foot_sites,
                    args.foot_half_length,
                    args.foot_half_width,
                    args.support_height_threshold,
                )
                with viewer.lock():
                    viewer.user_scn.ngeom = 0
                    add_overlay(viewer.user_scn, com, cop, polygon)
                viewer.sync()
                sleep_s = period - (time.monotonic() - begin)
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                if not viewer.is_running():
                    break
            if not args.loop:
                break
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
