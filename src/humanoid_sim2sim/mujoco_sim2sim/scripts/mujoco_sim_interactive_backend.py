#!/usr/bin/env python3

import math
import threading
import time
from dataclasses import dataclass, field
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

import mujoco
import mujoco.viewer
import numpy as np


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

TOPIC_POLICY_COMMAND = "/humanoid/rl/command"
TOPIC_ROBOT_STATE = "/humanoid/rl/state"


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
            "interactive backend ready: model=%s control_hz=%.1f substeps=%d viewer=%s actuator_mode=%s",
            self.model_path,
            self.control_hz,
            self.substeps_per_control,
            "on" if self.enable_viewer else "off",
            "position" if self.use_position_actuator_control else "torque",
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "")
        self.declare_parameter("base_body_name", "base_link")
        self.declare_parameter("base_free_joint_name", "")
        self.declare_parameter("joint_names", default_joint_names())
        self.declare_parameter("actuator_names", default_joint_names())
        self.declare_parameter("control_hz", 100.0)
        self.declare_parameter("sim_dt", 0.001)
        self.declare_parameter("command_timeout_sec", 0.1)
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

    def _load_parameters(self) -> None:
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.base_body_name = self.get_parameter("base_body_name").get_parameter_value().string_value
        self.base_free_joint_name = self.get_parameter("base_free_joint_name").get_parameter_value().string_value

        self.control_hz = max(1.0, float(self.get_parameter("control_hz").value))
        self.sim_dt = max(1e-5, float(self.get_parameter("sim_dt").value))
        self.command_timeout_sec = max(0.01, float(self.get_parameter("command_timeout_sec").value))
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

        kp_raw = list(self.get_parameter("kp").get_parameter_value().double_array_value)
        kd_raw = list(self.get_parameter("kd").get_parameter_value().double_array_value)
        tau_raw = list(self.get_parameter("torque_limit").get_parameter_value().double_array_value)
        self.kp = np.array(normalize_numeric_vector(kp_raw, 80.0, K_JOINT_COUNT), dtype=np.float64)
        self.kd = np.array(normalize_numeric_vector(kd_raw, 2.0, K_JOINT_COUNT), dtype=np.float64)
        self.torque_limit = np.array(normalize_numeric_vector(tau_raw, 120.0, K_JOINT_COUNT), dtype=np.float64)

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
                        "actuator '%s' not found, fallback to actuator index %d", self.actuator_names[i], aid
                    )
                else:
                    raise RuntimeError(f"actuator not found: {self.actuator_names[i]}")
            self.actuator_ids[i] = aid

            if int(self.model.actuator_biastype[aid]) != int(mujoco.mjtBias.mjBIAS_NONE):
                position_like_count += 1

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
        if len(msg.data) < K_JOINT_CMD_VALUE_COUNT:
            return

        cache = CommandCache()
        for i in range(K_JOINT_COUNT):
            off = i * 3
            cache.q[i] = msg.data[off + 0]
            cache.dq[i] = msg.data[off + 1]
            cache.tau[i] = msg.data[off + 2]

        cache.open_rl = float(msg.data[K_JOINT_STATE_VALUE_COUNT])
        cache.sequence = int(max(0.0, float(msg.data[K_JOINT_STATE_VALUE_COUNT + 1])))
        cache.remote_stamp_sec = float(msg.data[K_JOINT_STATE_VALUE_COUNT + 2])
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
            self.get_logger().warn("unknown open_rl mode %.2f, fallback hold", cmd.open_rl)
            self.last_timeout_warn_sec = now_sec

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

    def _publish_robot_state(self) -> None:
        msg = Float32MultiArray()
        data = np.zeros(K_ROBOT_STATE_VALUE_COUNT, dtype=np.float32)

        for i in range(K_JOINT_COUNT):
            qadr = int(self.qpos_addrs[i])
            vadr = int(self.qvel_addrs[i])
            off = i * 3
            if 0 <= qadr < int(self.model.nq):
                data[off + 0] = float(self.data.qpos[qadr])
            if 0 <= vadr < int(self.model.nv):
                data[off + 1] = float(self.data.qvel[vadr])
            data[off + 2] = float(self.applied_tau[i])

        cursor = K_JOINT_STATE_VALUE_COUNT
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
