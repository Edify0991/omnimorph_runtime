#!/usr/bin/python3
from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

_NUMPY_IMPORT_ERROR = None
try:
    import numpy as np
except Exception as exc:
    np = None  # type: ignore[assignment]
    _NUMPY_IMPORT_ERROR = exc

_MUJOCO_IMPORT_ERROR = None
try:
    import mujoco
    import mujoco.viewer
except Exception as exc:
    mujoco = None  # type: ignore[assignment]
    _MUJOCO_IMPORT_ERROR = exc


K_JOINT_COUNT = 12
K_JOINT_STATE_VALUE_COUNT = K_JOINT_COUNT * 3
K_JOINT_CMD_VALUE_COUNT = K_JOINT_STATE_VALUE_COUNT + 3
K_ROBOT_STATE_VALUE_COUNT = K_JOINT_STATE_VALUE_COUNT + 3 + 4 + 3

K_OPEN_RL_DISABLED = 0.0
K_OPEN_RL_POLICY = 10.0
K_OPEN_RL_COMMAND_STREAM = 20.0
K_OPEN_RL_TEST_CSP = 30.0
K_OPEN_RL_TEST_CST = 40.0
K_OPEN_RL_TEST_R1 = 50.0
K_PROTOCOL_V2_MAGIC = 240426
K_PROTOCOL_V2_VERSION = 2
K_PROTOCOL_V2_PAYLOAD_POLICY_COMMAND = 1
K_PROTOCOL_V2_PAYLOAD_ROBOT_STATE = 2
K_ROBOT_STATE_V2_HEADER_COUNT = 4

# Legacy split-runtime command topic. The fused runtime does not use this path.
TOPIC_POLICY_COMMAND = "/humanoid/rl/command"
TOPIC_ROBOT_STATE = "/humanoid/rl/state"


def normalize_no_command_behavior(raw: str) -> str:
    value = (raw or "").strip().lower()
    if value in ("hold_position", "position_hold", "position", "hold-pos"):
        return "hold_position"
    if value in ("zero_torque", "zero", "torque_off", "off"):
        return "zero_torque"
    if value in ("hold_last", "hold", "last"):
        return "hold_last"
    return "hold_position"


def default_joint_names() -> List[str]:
    return [
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


def mode_match(value: float, target: float) -> bool:
    return (target - 0.5) <= value < (target + 0.5)


def quat_xyzw_to_rpy(quat_xyzw: np.ndarray) -> np.ndarray:
    x = float(quat_xyzw[0])
    y = float(quat_xyzw[1])
    z = float(quat_xyzw[2])
    w = float(quat_xyzw[3])

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float32)


def normalize_numeric_vector(values: List[float], fallback: float, expected_count: int) -> List[float]:
    if not values:
        return [fallback for _ in range(expected_count)]
    if len(values) == 1:
        return [float(values[0]) for _ in range(expected_count)]
    if len(values) != expected_count:
        raise RuntimeError(f"vector size mismatch: expect 1 or {expected_count}, got {len(values)}")
    return [float(v) for v in values]


def normalize_name_vector(values: List[str], fallback: List[str], expected_count: int) -> List[str]:
    if not values:
        return list(fallback)
    if len(values) != expected_count:
        raise RuntimeError(f"name vector size mismatch: expect {expected_count}, got {len(values)}")
    return list(values)


@dataclass
class CommandCache:
    q: np.ndarray = field(default_factory=lambda: np.zeros(K_JOINT_COUNT, dtype=np.float32))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(K_JOINT_COUNT, dtype=np.float32))
    tau: np.ndarray = field(default_factory=lambda: np.zeros(K_JOINT_COUNT, dtype=np.float32))
    open_rl: float = K_OPEN_RL_DISABLED
    protocol_version: int = 1
    active_joint_count: int = K_JOINT_COUNT
    sequence: int = 0
    remote_stamp_sec: float = 0.0
    recv_time_sec: float = 0.0
    valid: bool = False


class MujocoInteractiveBackend(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_sim_interactive_backend")

        self._declare_parameters()
        self._load_parameters()

        self.model = self._load_model()
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.sim_dt
        mujoco.mj_forward(self.model, self.data)

        control_period = 1.0 / self.control_hz
        self.substeps_per_control = max(1, int(round(control_period / self.sim_dt)))
        self.applied_tau = np.zeros(K_JOINT_COUNT, dtype=np.float32)

        self._resolve_mappings()
        self._initialize_base_lock_if_enabled()
        self._initialize_last_targets()
        self._log_startup_diagnostics()
        self._warned_idle_position_fallback = False

        qos_cmd = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        qos_state = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.state_pub = self.create_publisher(Float32MultiArray, TOPIC_ROBOT_STATE, qos_state)
        self.cmd_sub = self.create_subscription(Float32MultiArray, TOPIC_POLICY_COMMAND, self._on_command, qos_cmd)

        self.command_lock = threading.Lock()
        self.latest_command = CommandCache()
        self.last_timeout_warn_sec = 0.0

        self.get_logger().info(
            "interactive backend ready: "
            f"model={self.model_path} "
            f"control_hz={self.control_hz:.1f} "
            f"substeps={self.substeps_per_control} "
            f"viewer={'on' if self.enable_viewer else 'off'} "
            f"actuator_mode={'position' if self.use_position_actuator_control else 'torque'}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "")
        self.declare_parameter("base_body_name", "base_link")
        self.declare_parameter("base_free_joint_name", "")
        self.declare_parameter("joint_names", default_joint_names())
        self.declare_parameter("actuator_names", default_joint_names())
        self.declare_parameter("hold_joint_names", [])
        self.declare_parameter("hold_actuator_names", [])
        self.declare_parameter("control_hz", 100.0)
        self.declare_parameter("sim_dt", 0.001)
        self.declare_parameter("command_timeout_sec", 0.1)
        self.declare_parameter("no_command_behavior", "hold_position")
        self.declare_parameter("open_rl_enable_threshold", 1.0)
        self.declare_parameter("use_command_torque_ff", False)
        self.declare_parameter("pause_when_no_command", False)
        self.declare_parameter("fix_base", False)
        self.declare_parameter("fixed_base_height", -1.0)
        self.declare_parameter("actuator_control_mode", "auto")
        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("viewer_fps", 60.0)
        self.declare_parameter("viewer_width", 1280)
        self.declare_parameter("viewer_height", 720)
        self.declare_parameter("viewer_title", "MuJoCo Sim2Sim Viewer")
        self.declare_parameter("show_left_ui", True)
        self.declare_parameter("show_right_ui", True)
        self.declare_parameter("kp", [80.0])
        self.declare_parameter("kd", [2.0])
        self.declare_parameter("torque_limit", [120.0])
        self.declare_parameter("hold_joint_target_q", [])
        self.declare_parameter("hold_kp", [80.0])
        self.declare_parameter("hold_kd", [2.0])
        self.declare_parameter("hold_torque_limit", [120.0])

    def _load_parameters(self) -> None:
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.base_body_name = self.get_parameter("base_body_name").get_parameter_value().string_value
        self.base_free_joint_name = self.get_parameter("base_free_joint_name").get_parameter_value().string_value

        self.control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self.sim_dt = max(1e-5, float(self.get_parameter("sim_dt").value))
        self.command_timeout_sec = max(0.01, float(self.get_parameter("command_timeout_sec").value))
        self.no_command_behavior = normalize_no_command_behavior(str(self.get_parameter("no_command_behavior").value))
        self.open_rl_enable_threshold = float(self.get_parameter("open_rl_enable_threshold").value)
        self.use_command_torque_ff = bool(self.get_parameter("use_command_torque_ff").value)
        self.pause_when_no_command = bool(self.get_parameter("pause_when_no_command").value)
        self.fix_base = bool(self.get_parameter("fix_base").value)
        self.fixed_base_height = float(self.get_parameter("fixed_base_height").value)
        self.actuator_control_mode = str(self.get_parameter("actuator_control_mode").value).lower()
        self.enable_viewer = bool(self.get_parameter("enable_viewer").value)
        self.viewer_fps = max(1.0, float(self.get_parameter("viewer_fps").value))
        self.viewer_width = int(max(320, int(self.get_parameter("viewer_width").value)))
        self.viewer_height = int(max(240, int(self.get_parameter("viewer_height").value)))
        self.viewer_title = str(self.get_parameter("viewer_title").value)
        self.show_left_ui = bool(self.get_parameter("show_left_ui").value)
        self.show_right_ui = bool(self.get_parameter("show_right_ui").value)

        names_joint = self.get_parameter("joint_names").get_parameter_value().string_array_value
        names_act = self.get_parameter("actuator_names").get_parameter_value().string_array_value
        self.joint_names = normalize_name_vector(list(names_joint), default_joint_names(), K_JOINT_COUNT)
        self.actuator_names = normalize_name_vector(list(names_act), self.joint_names, K_JOINT_COUNT)
        hold_joint_names_raw = list(self.get_parameter("hold_joint_names").get_parameter_value().string_array_value)
        hold_actuator_names_raw = list(self.get_parameter("hold_actuator_names").get_parameter_value().string_array_value)
        self.hold_joint_names = hold_joint_names_raw
        if hold_actuator_names_raw:
            if len(hold_actuator_names_raw) != len(self.hold_joint_names):
                raise RuntimeError("hold_actuator_names size must match hold_joint_names")
            self.hold_actuator_names = hold_actuator_names_raw
        else:
            self.hold_actuator_names = list(self.hold_joint_names)

        kp_raw = list(self.get_parameter("kp").get_parameter_value().double_array_value)
        kd_raw = list(self.get_parameter("kd").get_parameter_value().double_array_value)
        tau_raw = list(self.get_parameter("torque_limit").get_parameter_value().double_array_value)
        self.kp = np.array(normalize_numeric_vector(kp_raw, 80.0, K_JOINT_COUNT), dtype=np.float64)
        self.kd = np.array(normalize_numeric_vector(kd_raw, 2.0, K_JOINT_COUNT), dtype=np.float64)
        self.torque_limit = np.array(normalize_numeric_vector(tau_raw, 120.0, K_JOINT_COUNT), dtype=np.float64)
        hold_count = len(self.hold_joint_names)
        hold_kp_raw = list(self.get_parameter("hold_kp").get_parameter_value().double_array_value)
        hold_kd_raw = list(self.get_parameter("hold_kd").get_parameter_value().double_array_value)
        hold_tau_raw = list(self.get_parameter("hold_torque_limit").get_parameter_value().double_array_value)
        self.hold_kp = np.array(normalize_numeric_vector(hold_kp_raw, 80.0, hold_count), dtype=np.float64)
        self.hold_kd = np.array(normalize_numeric_vector(hold_kd_raw, 2.0, hold_count), dtype=np.float64)
        self.hold_torque_limit = np.array(normalize_numeric_vector(hold_tau_raw, 120.0, hold_count), dtype=np.float64)
        hold_target_raw = list(self.get_parameter("hold_joint_target_q").get_parameter_value().double_array_value)
        if not hold_target_raw:
            self.hold_target_q = np.zeros(hold_count, dtype=np.float64)
            self._hold_target_q_provided = False
        elif len(hold_target_raw) == 1 and hold_count > 0:
            self.hold_target_q = np.full(hold_count, float(hold_target_raw[0]), dtype=np.float64)
            self._hold_target_q_provided = True
        elif len(hold_target_raw) == hold_count:
            self.hold_target_q = np.array([float(v) for v in hold_target_raw], dtype=np.float64)
            self._hold_target_q_provided = True
        else:
            raise RuntimeError("hold_joint_target_q size must be 1 or match hold_joint_names")

        if not self.model_path:
            raise RuntimeError("parameter 'model_path' is empty")

    def _load_model(self) -> mujoco.MjModel:
        if self.model_path.endswith(".mjb"):
            return mujoco.MjModel.from_binary_path(self.model_path)
        return mujoco.MjModel.from_xml_path(self.model_path)

    def _resolve_mappings(self) -> None:
        self.joint_ids = np.full(K_JOINT_COUNT, -1, dtype=np.int32)
        self.qpos_addrs = np.full(K_JOINT_COUNT, -1, dtype=np.int32)
        self.qvel_addrs = np.full(K_JOINT_COUNT, -1, dtype=np.int32)
        self.actuator_ids = np.full(K_JOINT_COUNT, -1, dtype=np.int32)
        self.hold_joint_ids: List[int] = [-1 for _ in self.hold_joint_names]
        self.hold_qpos_addrs: List[int] = [-1 for _ in self.hold_joint_names]
        self.hold_qvel_addrs: List[int] = [-1 for _ in self.hold_joint_names]
        self.hold_actuator_ids: List[int] = [-1 for _ in self.hold_joint_names]
        self.hold_applied_tau = np.zeros(len(self.hold_joint_names), dtype=np.float32)

        position_like_count = 0
        for i, name in enumerate(self.joint_names):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"joint not found: {name}")
            jtype = int(self.model.jnt_type[jid])
            if jtype not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise RuntimeError(f"joint type not supported (need hinge/slide): {name}")

            self.joint_ids[i] = jid
            self.qpos_addrs[i] = int(self.model.jnt_qposadr[jid])
            self.qvel_addrs[i] = int(self.model.jnt_dofadr[jid])

            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.actuator_names[i])
            if aid < 0:
                if int(self.model.nu) == K_JOINT_COUNT:
                    aid = i
                    self.get_logger().warn(
                        f"actuator '{self.actuator_names[i]}' not found, fallback to actuator index {aid}"
                    )
                else:
                    raise RuntimeError(f"actuator not found: {self.actuator_names[i]}")
            self.actuator_ids[i] = aid

            if int(self.model.actuator_biastype[aid]) != int(mujoco.mjtBias.mjBIAS_NONE):
                position_like_count += 1

        for i, name in enumerate(self.hold_joint_names):
            if name in self.joint_names:
                self.get_logger().warn(
                    f"hold_joint_names[{i}]={name} overlaps policy-controlled joint, skip extra-hold mapping"
                )
                continue
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise RuntimeError(f"hold joint not found: {name}")
            jtype = int(self.model.jnt_type[jid])
            if jtype not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
                raise RuntimeError(f"hold joint type not supported (need hinge/slide): {name}")
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.hold_actuator_names[i])
            if aid < 0:
                raise RuntimeError(f"hold actuator not found: {self.hold_actuator_names[i]}")
            self.hold_joint_ids[i] = jid
            self.hold_qpos_addrs[i] = int(self.model.jnt_qposadr[jid])
            self.hold_qvel_addrs[i] = int(self.model.jnt_dofadr[jid])
            self.hold_actuator_ids[i] = aid

        if self.actuator_control_mode == "position":
            self.use_position_actuator_control = True
        elif self.actuator_control_mode == "torque":
            self.use_position_actuator_control = False
        else:
            self.use_position_actuator_control = position_like_count > (K_JOINT_COUNT // 2)

        self.base_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.base_body_name)
        if self.base_body_id < 0:
            self.base_body_id = 1 if int(self.model.nbody) > 1 else 0

        self.base_free_joint_id = -1
        if self.base_free_joint_name:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, self.base_free_joint_name)
            if bid >= 0 and int(self.model.jnt_type[bid]) == int(mujoco.mjtJoint.mjJNT_FREE):
                self.base_free_joint_id = bid

        if self.base_free_joint_id < 0:
            for jid in range(int(self.model.njnt)):
                if int(self.model.jnt_type[jid]) != int(mujoco.mjtJoint.mjJNT_FREE):
                    continue
                if self.base_body_id >= 0 and int(self.model.jnt_bodyid[jid]) != self.base_body_id:
                    continue
                self.base_free_joint_id = jid
                break

        self.base_free_qpos_adr = -1
        self.base_free_qvel_adr = -1
        if self.base_free_joint_id >= 0:
            self.base_free_qpos_adr = int(self.model.jnt_qposadr[self.base_free_joint_id])
            self.base_free_qvel_adr = int(self.model.jnt_dofadr[self.base_free_joint_id])

    def _initialize_last_targets(self) -> None:
        self.last_target_q = np.zeros(K_JOINT_COUNT, dtype=np.float64)
        for i in range(K_JOINT_COUNT):
            adr = int(self.qpos_addrs[i])
            if 0 <= adr < int(self.model.nq):
                self.last_target_q[i] = float(self.data.qpos[adr])

        if (not self._hold_target_q_provided) and len(self.hold_joint_names) > 0:
            for i, qadr in enumerate(self.hold_qpos_addrs):
                if 0 <= qadr < int(self.model.nq):
                    self.hold_target_q[i] = float(self.data.qpos[qadr])
            self.get_logger().info(
                f"hold_joint_target_q not provided, latch {len(self.hold_joint_names)} hold joints from model initial qpos"
            )

    def _log_startup_diagnostics(self) -> None:
        eps = 1e-4
        near_limit_count = 0
        self.get_logger().info("===== MuJoCo startup diagnostics =====")
        self.get_logger().info(
            f"control_hz={self.control_hz:.1f} sim_dt={self.sim_dt:.6f} "
            f"substeps={self.substeps_per_control} pause_when_no_command={self.pause_when_no_command} "
            f"actuator_mode={'position' if self.use_position_actuator_control else 'torque'} "
            f"no_command_behavior={self.no_command_behavior} "
            f"hold_extra_joints={len(self.hold_joint_names)}"
        )

        for i in range(K_JOINT_COUNT):
            jid = int(self.joint_ids[i])
            aid = int(self.actuator_ids[i])
            qadr = int(self.qpos_addrs[i])
            vadr = int(self.qvel_addrs[i])
            q0 = float(self.last_target_q[i])
            dq0 = float(self.data.qvel[vadr]) if (0 <= vadr < int(self.model.nv)) else 0.0

            limit_desc = "unlimited"
            near_limit = False
            if 0 <= jid < int(self.model.njnt) and int(self.model.jnt_limited[jid]) != 0:
                q_min = float(self.model.jnt_range[jid][0])
                q_max = float(self.model.jnt_range[jid][1])
                limit_desc = f"[{q_min:.4f}, {q_max:.4f}]"
                near_limit = (q0 <= q_min + eps) or (q0 >= q_max - eps)
                if near_limit:
                    near_limit_count += 1

            ctrl_desc = "n/a"
            if 0 <= aid < int(self.model.nu):
                if int(self.model.actuator_ctrllimited[aid]) != 0:
                    cmin = float(self.model.actuator_ctrlrange[aid][0])
                    cmax = float(self.model.actuator_ctrlrange[aid][1])
                    ctrl_desc = f"[{cmin:.4f}, {cmax:.4f}]"
                else:
                    ctrl_desc = "unlimited"

            self.get_logger().info(
                f"[joint {i:02d}] name={self.joint_names[i]} jid={jid} qadr={qadr} vadr={vadr} "
                f"q0={q0:.6f} dq0={dq0:.6f} range={limit_desc} near_limit={near_limit} "
                f"actuator={self.actuator_names[i]} aid={aid} ctrl={ctrl_desc}"
            )

        if self.base_free_joint_id >= 0 and self.base_free_qpos_adr >= 0 and self.base_free_qvel_adr >= 0:
            if (self.base_free_qpos_adr + 6) < int(self.model.nq):
                px = float(self.data.qpos[self.base_free_qpos_adr + 0])
                py = float(self.data.qpos[self.base_free_qpos_adr + 1])
                pz = float(self.data.qpos[self.base_free_qpos_adr + 2])
                qw = float(self.data.qpos[self.base_free_qpos_adr + 3])
                qx = float(self.data.qpos[self.base_free_qpos_adr + 4])
                qy = float(self.data.qpos[self.base_free_qpos_adr + 5])
                qz = float(self.data.qpos[self.base_free_qpos_adr + 6])
                self.get_logger().info(
                    "base_free_joint: "
                    f"jid={self.base_free_joint_id} qpos_adr={self.base_free_qpos_adr} "
                    f"pos=({px:.6f}, {py:.6f}, {pz:.6f}) quat_wxyz=({qw:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})"
                )
            if (self.base_free_qvel_adr + 5) < int(self.model.nv):
                lvx = float(self.data.qvel[self.base_free_qvel_adr + 0])
                lvy = float(self.data.qvel[self.base_free_qvel_adr + 1])
                lvz = float(self.data.qvel[self.base_free_qvel_adr + 2])
                avx = float(self.data.qvel[self.base_free_qvel_adr + 3])
                avy = float(self.data.qvel[self.base_free_qvel_adr + 4])
                avz = float(self.data.qvel[self.base_free_qvel_adr + 5])
                self.get_logger().info(
                    "base_free_joint velocity: "
                    f"lin=({lvx:.6f}, {lvy:.6f}, {lvz:.6f}) "
                    f"ang=({avx:.6f}, {avy:.6f}, {avz:.6f})"
                )
        else:
            self.get_logger().warn(
                f"no free base joint resolved (base_body_id={self.base_body_id}, base_free_joint_id={self.base_free_joint_id})"
            )

        if near_limit_count > 0:
            self.get_logger().warn(
                f"{near_limit_count}/{K_JOINT_COUNT} joints start near their limits. "
                "This usually means model init pose or joint mapping needs checking."
            )
        self.get_logger().info("===== End startup diagnostics =====")

    def _initialize_base_lock_if_enabled(self) -> None:
        self.fixed_base_qpos = np.zeros(7, dtype=np.float64)
        self.fixed_base_pose_initialized = False

        if not self.fix_base:
            return

        if self.base_free_qpos_adr < 0 or self.base_free_qvel_adr < 0:
            self.get_logger().warn("fix_base=true but free joint unavailable, disabling base lock")
            self.fix_base = False
            return
        if (self.base_free_qpos_adr + 6) >= int(self.model.nq):
            self.get_logger().warn("fix_base=true but free joint qpos range invalid, disabling base lock")
            self.fix_base = False
            return

        for i in range(7):
            self.fixed_base_qpos[i] = float(self.data.qpos[self.base_free_qpos_adr + i])
        if self.fixed_base_height >= 0.0:
            self.fixed_base_qpos[2] = self.fixed_base_height
        self.fixed_base_pose_initialized = True
        self._enforce_base_lock()
        mujoco.mj_forward(self.model, self.data)

    def _enforce_base_lock(self) -> None:
        if not self.fix_base or not self.fixed_base_pose_initialized:
            return
        if self.base_free_qpos_adr < 0 or self.base_free_qvel_adr < 0:
            return
        if (self.base_free_qpos_adr + 6) >= int(self.model.nq):
            return
        if (self.base_free_qvel_adr + 5) >= int(self.model.nv):
            return

        for i in range(7):
            self.data.qpos[self.base_free_qpos_adr + i] = self.fixed_base_qpos[i]
        for i in range(6):
            self.data.qvel[self.base_free_qvel_adr + i] = 0.0

    def _on_command(self, msg: Float32MultiArray) -> None:
        cache = CommandCache()
        is_v2 = (
            len(msg.data) >= 7
            and int(round(float(msg.data[0]))) == K_PROTOCOL_V2_MAGIC
            and int(round(float(msg.data[1]))) == K_PROTOCOL_V2_VERSION
            and int(round(float(msg.data[2]))) == K_PROTOCOL_V2_PAYLOAD_POLICY_COMMAND
        )

        if is_v2:
            joint_count = int(max(0, int(round(float(msg.data[3])))))
            expected = 7 + 3 * joint_count
            if len(msg.data) < expected:
                return
            cache.protocol_version = K_PROTOCOL_V2_VERSION
            cache.active_joint_count = joint_count
            for i in range(min(K_JOINT_COUNT, joint_count)):
                off = 7 + i * 3
                cache.q[i] = msg.data[off + 0]
                cache.dq[i] = msg.data[off + 1]
                cache.tau[i] = msg.data[off + 2]
            cache.open_rl = float(msg.data[4])
            cache.sequence = int(max(0.0, float(msg.data[5])))
            cache.remote_stamp_sec = float(msg.data[6])
        else:
            if len(msg.data) < K_JOINT_CMD_VALUE_COUNT:
                return
            for i in range(K_JOINT_COUNT):
                off = i * 3
                cache.q[i] = msg.data[off + 0]
                cache.dq[i] = msg.data[off + 1]
                cache.tau[i] = msg.data[off + 2]

            cache.open_rl = float(msg.data[K_JOINT_STATE_VALUE_COUNT])
            cache.sequence = int(max(0.0, float(msg.data[K_JOINT_STATE_VALUE_COUNT + 1])))
            cache.remote_stamp_sec = float(msg.data[K_JOINT_STATE_VALUE_COUNT + 2])
            cache.protocol_version = 1
            cache.active_joint_count = K_JOINT_COUNT
        cache.recv_time_sec = time.monotonic()
        cache.valid = True

        with self.command_lock:
            self.latest_command = cache

    def _command_fresh(self, now_sec: float) -> bool:
        with self.command_lock:
            if not self.latest_command.valid:
                return False
            return (now_sec - self.latest_command.recv_time_sec) <= self.command_timeout_sec

    def _command_snapshot(self) -> CommandCache:
        with self.command_lock:
            return self.latest_command

    def _update_control_input(self, now_sec: float) -> None:
        cmd = self._command_snapshot()
        is_fresh = cmd.valid and ((now_sec - cmd.recv_time_sec) <= self.command_timeout_sec)
        no_command_idle = not is_fresh

        mode_policy = is_fresh and mode_match(cmd.open_rl, K_OPEN_RL_POLICY)
        mode_command_csp = is_fresh and (
            mode_match(cmd.open_rl, K_OPEN_RL_COMMAND_STREAM) or mode_match(cmd.open_rl, K_OPEN_RL_TEST_CSP)
        )
        mode_test_cst = is_fresh and mode_match(cmd.open_rl, K_OPEN_RL_TEST_CST)
        mode_test_r1 = is_fresh and mode_match(cmd.open_rl, K_OPEN_RL_TEST_R1)
        enable_rl = mode_policy or mode_command_csp or mode_test_cst or mode_test_r1

        if not is_fresh:
            if (now_sec - self.last_timeout_warn_sec) > 1.0:
                self.get_logger().warn("policy command timeout, fallback hold")
                self.last_timeout_warn_sec = now_sec
        elif (not enable_rl) and (cmd.open_rl > self.open_rl_enable_threshold) and ((now_sec - self.last_timeout_warn_sec) > 1.0):
            self.get_logger().warn(f"unknown open_rl mode {cmd.open_rl:.2f}, fallback hold")
            self.last_timeout_warn_sec = now_sec

        idle_hold_position = no_command_idle and (self.no_command_behavior == "hold_position")
        idle_zero_torque = no_command_idle and (self.no_command_behavior == "zero_torque")
        if idle_hold_position and (not self.use_position_actuator_control) and (not self._warned_idle_position_fallback):
            self.get_logger().warn(
                "no_command_behavior=hold_position requested, but current actuator mode is torque. "
                "fallback to torque PD hold_last. If you need strict position hold, use position actuators "
                "or set actuator_control_mode:=position and ensure model supports it."
            )
            self._warned_idle_position_fallback = True

        for i in range(K_JOINT_COUNT):
            qadr = int(self.qpos_addrs[i])
            vadr = int(self.qvel_addrs[i])
            aid = int(self.actuator_ids[i])
            if qadr < 0 or vadr < 0 or aid < 0:
                continue
            if qadr >= int(self.model.nq) or vadr >= int(self.model.nv) or aid >= int(self.model.nu):
                continue

            q = float(self.data.qpos[qadr])
            dq = float(self.data.qvel[vadr])

            q_des = float(self.last_target_q[i])
            dq_des = 0.0
            tau_ff = 0.0
            if mode_policy or mode_command_csp or mode_test_r1:
                q_des = float(cmd.q[i])
                dq_des = float(cmd.dq[i])
                tau_ff = float(cmd.tau[i])
                self.last_target_q[i] = q_des

            if idle_zero_torque:
                self.data.ctrl[aid] = 0.0
                self.applied_tau[i] = 0.0
                continue

            if self.use_position_actuator_control:
                if mode_test_cst:
                    q_des = float(self.last_target_q[i])
                self.data.ctrl[aid] = q_des
                self.applied_tau[i] = 0.0
            else:
                if mode_test_cst:
                    tau = float(cmd.tau[i])
                else:
                    tau = float(self.kp[i] * (q_des - q) + self.kd[i] * (dq_des - dq))
                    if (mode_policy or mode_test_r1) and self.use_command_torque_ff:
                        tau += tau_ff
                limit = max(1e-6, abs(float(self.torque_limit[i])))
                tau = float(np.clip(tau, -limit, limit))
                self.data.ctrl[aid] = tau
                self.applied_tau[i] = tau

        # Non-policy joints: keep configured fixed targets.
        for i in range(len(self.hold_joint_names)):
            qadr = int(self.hold_qpos_addrs[i])
            vadr = int(self.hold_qvel_addrs[i])
            aid = int(self.hold_actuator_ids[i])
            if qadr < 0 or vadr < 0 or aid < 0:
                continue
            if qadr >= int(self.model.nq) or vadr >= int(self.model.nv) or aid >= int(self.model.nu):
                continue

            q = float(self.data.qpos[qadr])
            dq = float(self.data.qvel[vadr])
            q_des = float(self.hold_target_q[i]) if i < len(self.hold_target_q) else q

            if self.use_position_actuator_control:
                self.data.ctrl[aid] = q_des
                self.hold_applied_tau[i] = 0.0
            else:
                tau = float(self.hold_kp[i] * (q_des - q) - self.hold_kd[i] * dq)
                limit = max(1e-6, abs(float(self.hold_torque_limit[i])))
                tau = float(np.clip(tau, -limit, limit))
                self.data.ctrl[aid] = tau
                self.hold_applied_tau[i] = tau

    def _publish_robot_state(self) -> None:
        msg = Float32MultiArray()
        data = np.zeros(K_ROBOT_STATE_V2_HEADER_COUNT + K_ROBOT_STATE_VALUE_COUNT, dtype=np.float32)
        data[0] = float(K_PROTOCOL_V2_MAGIC)
        data[1] = float(K_PROTOCOL_V2_VERSION)
        data[2] = float(K_PROTOCOL_V2_PAYLOAD_ROBOT_STATE)
        data[3] = float(K_JOINT_COUNT)

        cursor = K_ROBOT_STATE_V2_HEADER_COUNT
        for i in range(K_JOINT_COUNT):
            qadr = int(self.qpos_addrs[i])
            vadr = int(self.qvel_addrs[i])
            if 0 <= qadr < int(self.model.nq):
                data[cursor + 0] = float(self.data.qpos[qadr])
            if 0 <= vadr < int(self.model.nv):
                data[cursor + 1] = float(self.data.qvel[vadr])
            data[cursor + 2] = float(self.applied_tau[i])
            cursor += 3

        base_w = np.zeros(3, dtype=np.float32)
        if self.base_free_qvel_adr >= 0 and (self.base_free_qvel_adr + 5) < int(self.model.nv):
            base_w[0] = float(self.data.qvel[self.base_free_qvel_adr + 3])
            base_w[1] = float(self.data.qvel[self.base_free_qvel_adr + 4])
            base_w[2] = float(self.data.qvel[self.base_free_qvel_adr + 5])
        data[cursor : cursor + 3] = base_w
        cursor += 3

        quat_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        if self.base_free_qpos_adr >= 0 and (self.base_free_qpos_adr + 6) < int(self.model.nq):
            qw = float(self.data.qpos[self.base_free_qpos_adr + 3])
            qx = float(self.data.qpos[self.base_free_qpos_adr + 4])
            qy = float(self.data.qpos[self.base_free_qpos_adr + 5])
            qz = float(self.data.qpos[self.base_free_qpos_adr + 6])
            quat_xyzw[:] = [qx, qy, qz, qw]
        data[cursor : cursor + 4] = quat_xyzw
        cursor += 4

        rpy = quat_xyzw_to_rpy(quat_xyzw)
        data[cursor : cursor + 3] = rpy

        msg.data = data.tolist()
        self.state_pub.publish(msg)

    def _step_once(self, now_sec: float) -> None:
        has_fresh = self._command_fresh(now_sec)
        if self.pause_when_no_command and (not has_fresh):
            self._publish_robot_state()
            return

        self._update_control_input(now_sec)
        for _ in range(self.substeps_per_control):
            self._enforce_base_lock()
            mujoco.mj_step(self.model, self.data)
            self._enforce_base_lock()
        mujoco.mj_forward(self.model, self.data)
        self._publish_robot_state()

    def run(self) -> None:
        period = 1.0 / self.control_hz

        if self.enable_viewer:
            viewer_ctx = None
            try:
                viewer_ctx = mujoco.viewer.launch_passive(
                    self.model,
                    self.data,
                    show_left_ui=self.show_left_ui,
                    show_right_ui=self.show_right_ui,
                )
            except TypeError:
                self.get_logger().warn(
                    "current mujoco Python version does not support show_left_ui/show_right_ui args; fallback to default viewer"
                )
                viewer_ctx = mujoco.viewer.launch_passive(self.model, self.data)

            with viewer_ctx as viewer:
                while rclpy.ok() and viewer.is_running():
                    loop_start = time.monotonic()
                    rclpy.spin_once(self, timeout_sec=0.0)
                    with viewer.lock():
                        self._step_once(loop_start)
                    viewer.sync()
                    elapsed = time.monotonic() - loop_start
                    if elapsed < period:
                        time.sleep(period - elapsed)
        else:
            while rclpy.ok():
                loop_start = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0.0)
                self._step_once(loop_start)
                elapsed = time.monotonic() - loop_start
                if elapsed < period:
                    time.sleep(period - elapsed)


def main() -> None:
    if _NUMPY_IMPORT_ERROR is not None:
        print(
            "[mujoco_sim_interactive_backend] fatal error: failed to import numpy. "
            f"python={sys.executable}, error={_NUMPY_IMPORT_ERROR}",
            flush=True,
        )
        print(
            f"[mujoco_sim_interactive_backend] install hint: {sys.executable} -m pip install numpy",
            flush=True,
        )
        raise RuntimeError("numpy import failed")

    if _MUJOCO_IMPORT_ERROR is not None:
        print(
            "[mujoco_sim_interactive_backend] fatal error: failed to import mujoco. "
            f"python={sys.executable}, error={_MUJOCO_IMPORT_ERROR}",
            flush=True,
        )
        print(
            f"[mujoco_sim_interactive_backend] install hint: {sys.executable} -m pip install mujoco",
            flush=True,
        )
        print(
            "[mujoco_sim_interactive_backend] if you use ROS2 overlay, run the install command "
            "after `source /opt/ros/humble/setup.bash` and your workspace setup.",
            flush=True,
        )
        raise RuntimeError("mujoco import failed")

    rclpy.init()
    node = None
    try:
        node = MujocoInteractiveBackend()
        node.run()
    except Exception as exc:
        print(f"[mujoco_sim_interactive_backend] fatal error: {exc}", flush=True)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
