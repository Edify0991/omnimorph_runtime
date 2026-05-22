#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

try:
    import mujoco
except Exception as exc:  # pragma: no cover - runtime dependency
    raise RuntimeError(f"mujoco python import failed: {exc}") from exc


def _format_vec(values: np.ndarray) -> str:
    return "[" + ", ".join(f"{float(v): .6f}" for v in values.tolist()) + "]"


def _axis_name(index: int, sign: float) -> str:
    labels = ["X", "Y", "Z"]
    prefix = "+" if sign >= 0.0 else "-"
    return f"{prefix}{labels[index]}"


def _closest_world_axis(vec_world: np.ndarray) -> str:
    idx = int(np.argmax(np.abs(vec_world)))
    return _axis_name(idx, float(vec_world[idx]))


def _rotation_from_quat_wxyz(quat_wxyz: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = [float(v) for v in quat_wxyz.tolist()]
    norm = np.linalg.norm([qw, qx, qy, qz])
    if norm < 1.0e-12:
        return np.eye(3)
    qw /= norm
    qx /= norm
    qy /= norm
    qz /= norm
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def _resolve_config_value(path: Path) -> Dict[str, object]:
    content = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(content, dict):
        return {}
    root = content.get("mujoco_sim_bridge")
    if not isinstance(root, dict):
        return {}
    params = root.get("ros__parameters")
    return params if isinstance(params, dict) else {}


@dataclass
class ProbeConfig:
    model_path: str
    base_body_name: str
    base_free_joint_name: str


def _load_probe_config(sim2sim_config_path: Optional[str], model_path: Optional[str], base_body_name: Optional[str], base_free_joint_name: Optional[str]) -> ProbeConfig:
    params: Dict[str, object] = {}
    if sim2sim_config_path:
        config_path = Path(sim2sim_config_path).expanduser().resolve()
        if not config_path.exists():
            raise FileNotFoundError(f"sim2sim config not found: {config_path}")
        params = _resolve_config_value(config_path)

    resolved_model_path = model_path or str(params.get("model_path", "")).strip()
    resolved_base_body_name = base_body_name or str(params.get("base_body_name", "Body")).strip() or "Body"
    resolved_base_free_joint_name = base_free_joint_name or str(params.get("base_free_joint_name", "")).strip()

    if not resolved_model_path:
        raise RuntimeError("model_path is empty. Provide --model-path or --sim2sim-config.")

    return ProbeConfig(
        model_path=resolved_model_path,
        base_body_name=resolved_base_body_name,
        base_free_joint_name=resolved_base_free_joint_name,
    )


def _resolve_base_body_id(model: mujoco.MjModel, preferred_name: str) -> int:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, preferred_name)
    if body_id >= 0:
        return int(body_id)
    if model.nbody > 1:
        return 1
    return 0


def _resolve_base_free_joint_id(model: mujoco.MjModel, base_body_id: int, preferred_name: str) -> int:
    if preferred_name:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, preferred_name)
        if joint_id >= 0 and model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            return int(joint_id)
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE and model.jnt_bodyid[joint_id] == base_body_id:
            return int(joint_id)
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
            return int(joint_id)
    raise RuntimeError("No free joint found in model; cannot probe freejoint_qvel semantics.")


def _body_name(model: mujoco.MjModel, body_id: int) -> str:
    try:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    except Exception:
        name = None
    return name or f"body_{body_id}"


def _joint_name(model: mujoco.MjModel, joint_id: int) -> str:
    try:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
    except Exception:
        name = None
    return name or f"joint_{joint_id}"


def _print_frame_axes(title: str, rot_wf: np.ndarray) -> None:
    print(title)
    print("  Frame axes expressed in world frame:")
    for axis_name, axis_local in (
        ("x_frame", np.array([1.0, 0.0, 0.0], dtype=np.float64)),
        ("y_frame", np.array([0.0, 1.0, 0.0], dtype=np.float64)),
        ("z_frame", np.array([0.0, 0.0, 1.0], dtype=np.float64)),
    ):
        axis_world = rot_wf @ axis_local
        print(f"    {axis_name} -> world { _format_vec(axis_world) }  closest={_closest_world_axis(axis_world)}")
    print("  World axes expressed in frame:")
    rot_fw = rot_wf.T
    for axis_name, axis_world in (
        ("x_world", np.array([1.0, 0.0, 0.0], dtype=np.float64)),
        ("y_world", np.array([0.0, 1.0, 0.0], dtype=np.float64)),
        ("z_world", np.array([0.0, 0.0, 1.0], dtype=np.float64)),
    ):
        axis_frame = rot_fw @ axis_world
        print(f"    {axis_name} -> frame { _format_vec(axis_frame) }  closest={_closest_world_axis(axis_frame)}")
    print("")


def _base_pose_summary(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base_body_id: int,
    base_free_joint_id: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    qpos_adr = int(model.jnt_qposadr[base_free_joint_id])
    root_pos_w = np.asarray(data.qpos[qpos_adr : qpos_adr + 3], dtype=np.float64).copy()
    quat_wxyz = np.asarray(data.qpos[qpos_adr + 3 : qpos_adr + 7], dtype=np.float64).copy()
    rot_w_freejoint = _rotation_from_quat_wxyz(quat_wxyz)
    rot_w_xbody = np.asarray(data.xmat[base_body_id], dtype=np.float64).reshape(3, 3).copy()
    rot_w_inertial = np.asarray(data.ximat[base_body_id], dtype=np.float64).reshape(3, 3).copy()
    xbody_pos_w = np.asarray(data.xpos[base_body_id], dtype=np.float64).copy()
    inertial_pos_w = np.asarray(data.xipos[base_body_id], dtype=np.float64).copy()
    root_to_xbody_w = xbody_pos_w - root_pos_w
    root_to_inertial_w = inertial_pos_w - root_pos_w

    print("=== Initial Base Pose ===")
    print(f"root_freejoint origin in world: { _format_vec(root_pos_w) }")
    print(f"xbody origin in world:          { _format_vec(xbody_pos_w) }")
    print(f"inertial body origin in world:  { _format_vec(inertial_pos_w) }")
    print(f"root->xbody offset in world:    { _format_vec(root_to_xbody_w) }")
    print(f"root->inertial offset in world: { _format_vec(root_to_inertial_w) }")
    print(f"root->xbody offset norm: {float(np.linalg.norm(root_to_xbody_w)):.6f} m")
    print(f"root->inertial offset norm: {float(np.linalg.norm(root_to_inertial_w)):.6f} m")
    print(f"base_quat_wxyz: { _format_vec(quat_wxyz) }")
    print(f"base_quat_xyzw: { _format_vec(np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)) }")
    print("")
    _print_frame_axes("Freejoint/root frame from qpos quaternion:", rot_w_freejoint)
    _print_frame_axes("Regular xbody frame from data.xmat/data.xpos:", rot_w_xbody)
    _print_frame_axes("Inertial body frame from data.ximat/data.xipos:", rot_w_inertial)
    rot_xbody_freejoint = rot_w_xbody.T @ rot_w_freejoint
    rot_inertial_freejoint = rot_w_inertial.T @ rot_w_freejoint
    print("Transform from freejoint/root frame into xbody frame:")
    print(f"  x_root in xbody frame = { _format_vec(rot_xbody_freejoint @ np.array([1.0, 0.0, 0.0], dtype=np.float64)) }")
    print(f"  y_root in xbody frame = { _format_vec(rot_xbody_freejoint @ np.array([0.0, 1.0, 0.0], dtype=np.float64)) }")
    print(f"  z_root in xbody frame = { _format_vec(rot_xbody_freejoint @ np.array([0.0, 0.0, 1.0], dtype=np.float64)) }")
    print("")
    print("Transform from freejoint/root frame into inertial body frame:")
    print(f"  x_root in inertial frame = { _format_vec(rot_inertial_freejoint @ np.array([1.0, 0.0, 0.0], dtype=np.float64)) }")
    print(f"  y_root in inertial frame = { _format_vec(rot_inertial_freejoint @ np.array([0.0, 1.0, 0.0], dtype=np.float64)) }")
    print(f"  z_root in inertial frame = { _format_vec(rot_inertial_freejoint @ np.array([0.0, 0.0, 1.0], dtype=np.float64)) }")
    print("")
    return root_pos_w, xbody_pos_w, inertial_pos_w, quat_wxyz, rot_w_freejoint, rot_w_xbody, rot_w_inertial


def _read_velocity_views(model: mujoco.MjModel, data: mujoco.MjData, base_body_id: int, base_free_joint_id: int) -> Dict[str, np.ndarray]:
    qvel_adr = int(model.jnt_dofadr[base_free_joint_id])
    freejoint = np.asarray(data.qvel[qvel_adr : qvel_adr + 6], dtype=np.float64).copy()

    vel6_local = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base_body_id, vel6_local, 1)

    vel6_world = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, base_body_id, vel6_world, 0)

    cvel = np.asarray(data.cvel[base_body_id], dtype=np.float64).copy() if data.cvel is not None else np.full(6, np.nan, dtype=np.float64)

    return {
        "freejoint_lin_world": freejoint[0:3],
        "freejoint_ang_local": freejoint[3:6],
        "body_object_local_ang": vel6_local[0:3],
        "body_object_local_lin": vel6_local[3:6],
        "body_object_world_ang": vel6_world[0:3],
        "body_object_world_lin": vel6_world[3:6],
        "body_cvel_ang": cvel[0:3],
        "body_cvel_lin": cvel[3:6],
    }


def _print_velocity_views(
    title: str,
    views: Dict[str, np.ndarray],
    rot_w_root: np.ndarray,
    rot_w_inertial: np.ndarray,
    root_to_inertial_w: np.ndarray,
) -> None:
    rot_inertial_w = rot_w_inertial.T
    expected_world_ang_from_freejoint = rot_w_root @ views["freejoint_ang_local"]
    expected_world_lin_at_body_origin_from_freejoint = views["freejoint_lin_world"] + np.cross(
        expected_world_ang_from_freejoint,
        root_to_inertial_w,
    )
    expected_local_lin_at_body_origin_from_freejoint = rot_inertial_w @ expected_world_lin_at_body_origin_from_freejoint
    expected_local_ang_at_body_from_freejoint = rot_inertial_w @ expected_world_ang_from_freejoint

    print(title)
    print(f"  freejoint_qvel     lin_world = { _format_vec(views['freejoint_lin_world']) }")
    print(f"  freejoint_qvel     ang_local = { _format_vec(views['freejoint_ang_local']) }")
    print(f"  body_object_local  ang_local = { _format_vec(views['body_object_local_ang']) }")
    print(f"  body_object_local  lin_local = { _format_vec(views['body_object_local_lin']) }")
    print(f"  body_object_world  ang_world = { _format_vec(views['body_object_world_ang']) }")
    print(f"  body_object_world  lin_world = { _format_vec(views['body_object_world_lin']) }")
    print(f"  body_cvel          ang_local = { _format_vec(views['body_cvel_ang']) }")
    print(f"  body_cvel          lin_local = { _format_vec(views['body_cvel_lin']) }")
    print("")
    print("  Semantic checks:")
    print(
        "    freejoint ang -> world -> body       = "
        f"{ _format_vec(expected_local_ang_at_body_from_freejoint) }"
    )
    print(
        "    diff vs body_object_local ang        = "
        f"{ _format_vec(expected_local_ang_at_body_from_freejoint - views['body_object_local_ang']) }"
    )
    print(
        "    freejoint lin at body origin world   = "
        f"{ _format_vec(expected_world_lin_at_body_origin_from_freejoint) }"
    )
    print(
        "    diff vs body_object_world lin        = "
        f"{ _format_vec(expected_world_lin_at_body_origin_from_freejoint - views['body_object_world_lin']) }"
    )
    print(
        "    freejoint lin at body origin local   = "
        f"{ _format_vec(expected_local_lin_at_body_origin_from_freejoint) }"
    )
    print(
        "    diff vs body_object_local lin        = "
        f"{ _format_vec(expected_local_lin_at_body_origin_from_freejoint - views['body_object_local_lin']) }"
    )
    print("")


def _make_probe_data(model: mujoco.MjModel, qpos0: np.ndarray, qvel0: np.ndarray, base_free_joint_id: int, lin_world: np.ndarray, ang_local: np.ndarray) -> mujoco.MjData:
    data = mujoco.MjData(model)
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    qvel_adr = int(model.jnt_dofadr[base_free_joint_id])
    data.qvel[qvel_adr : qvel_adr + 3] = lin_world
    data.qvel[qvel_adr + 3 : qvel_adr + 6] = ang_local
    mujoco.mj_forward(model, data)
    return data


def _synthetic_cases() -> List[Tuple[str, np.ndarray, np.ndarray]]:
    return [
        ("world_lin_x", np.array([1.0, 0.0, 0.0], dtype=np.float64), np.array([0.0, 0.0, 0.0], dtype=np.float64)),
        ("world_lin_y", np.array([0.0, 1.0, 0.0], dtype=np.float64), np.array([0.0, 0.0, 0.0], dtype=np.float64)),
        ("world_lin_z", np.array([0.0, 0.0, 1.0], dtype=np.float64), np.array([0.0, 0.0, 0.0], dtype=np.float64)),
        ("local_ang_x", np.array([0.0, 0.0, 0.0], dtype=np.float64), np.array([1.0, 0.0, 0.0], dtype=np.float64)),
        ("local_ang_y", np.array([0.0, 0.0, 0.0], dtype=np.float64), np.array([0.0, 1.0, 0.0], dtype=np.float64)),
        ("local_ang_z", np.array([0.0, 0.0, 0.0], dtype=np.float64), np.array([0.0, 0.0, 1.0], dtype=np.float64)),
        ("mixed_case", np.array([0.7, -0.4, 0.2], dtype=np.float64), np.array([0.3, 0.5, -0.6], dtype=np.float64)),
    ]


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Probe MuJoCo base velocity semantics used by freejoint_qvel and "
            "body_object_velocity_local."
        )
    )
    parser.add_argument(
        "--sim2sim-config",
        default="src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_amp_full_body_sim2sim.yaml",
        help="Optional MuJoCo sim2sim yaml used to resolve model_path/base names.",
    )
    parser.add_argument("--model-path", default="", help="Override MuJoCo model xml/mjb path.")
    parser.add_argument("--base-body-name", default="", help="Override base body name.")
    parser.add_argument("--base-free-joint-name", default="", help="Override base free joint name.")
    args = parser.parse_args()

    cfg = _load_probe_config(
        args.sim2sim_config,
        args.model_path or None,
        args.base_body_name or None,
        args.base_free_joint_name or None,
    )

    model_path = Path(cfg.model_path).expanduser().resolve()
    if not model_path.exists():
        raise FileNotFoundError(f"model_path not found: {model_path}")

    if model_path.suffix.lower() == ".mjb":
        model = mujoco.MjModel.from_binary_path(str(model_path))
    else:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    base_body_id = _resolve_base_body_id(model, cfg.base_body_name)
    base_free_joint_id = _resolve_base_free_joint_id(model, base_body_id, cfg.base_free_joint_name)

    print("=== Model Resolution ===")
    print(f"model_path: {model_path}")
    print(f"base_body_name(requested): {cfg.base_body_name}")
    print(f"base_body_name(resolved): {_body_name(model, base_body_id)} (id={base_body_id})")
    print(f"base_free_joint_name(requested): {cfg.base_free_joint_name or '<auto>'}")
    print(f"base_free_joint_name(resolved): {_joint_name(model, base_free_joint_id)} (id={base_free_joint_id})")
    print(f"nq={model.nq} nv={model.nv} nbody={model.nbody} njnt={model.njnt}")
    print("")

    root_pos_w, xbody_pos_w, inertial_pos_w, _, rot_w_freejoint, rot_w_xbody, rot_w_inertial = _base_pose_summary(
        model,
        data,
        base_body_id,
        base_free_joint_id,
    )
    root_to_inertial_w = inertial_pos_w - root_pos_w

    print("=== Initial State Velocity Readout ===")
    initial_views = _read_velocity_views(model, data, base_body_id, base_free_joint_id)
    _print_velocity_views("Current initial qpos/qvel:", initial_views, rot_w_freejoint, rot_w_inertial, root_to_inertial_w)

    qpos0 = np.asarray(data.qpos, dtype=np.float64).copy()
    qvel0 = np.asarray(data.qvel, dtype=np.float64).copy()

    print("=== Synthetic Twist Injection Tests ===")
    print("These cases keep the same initial pose, but overwrite the base freejoint qvel.")
    print("They make the frame difference visible even if the original startup qvel is almost zero.")
    print("")

    for case_name, lin_world, ang_local in _synthetic_cases():
        probe_data = _make_probe_data(model, qpos0, qvel0, base_free_joint_id, lin_world, ang_local)
        probe_views = _read_velocity_views(model, probe_data, base_body_id, base_free_joint_id)
        print(f"--- {case_name} ---")
        print(f"Injected freejoint lin_world = { _format_vec(lin_world) }")
        print(f"Injected freejoint ang_local = { _format_vec(ang_local) }")
        _print_velocity_views("Result:", probe_views, rot_w_freejoint, rot_w_inertial, root_to_inertial_w)


if __name__ == "__main__":
    main()
