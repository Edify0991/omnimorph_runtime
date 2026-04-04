#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

try:
    import fcntl
except ImportError:
    fcntl = None

try:
    import evdev
    from evdev import ecodes
    EVDEV_IMPORT_ERROR = None
except ImportError as exc:
    evdev = None
    ecodes = None
    EVDEV_IMPORT_ERROR = exc

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Int32
    RCLPY_IMPORT_ERROR = None
except ImportError as exc:
    rclpy = None
    Twist = None
    Int32 = None
    RCLPY_IMPORT_ERROR = exc

try:
    from receiver import CmdDataStruct, Receiver
    RECEIVER_IMPORT_ERROR = None
except ImportError as exc:
    CmdDataStruct = object
    Receiver = None
    RECEIVER_IMPORT_ERROR = exc


class RobotWalkMode(Enum):
    WALK = 0
    STAND = 1
    FIX_STAND = 2
    START_POLICY = 10
    STOP_POLICY = 11
    ZEROING = 12
    ESTOP = 13
    START_WALK = 20
    START_STAND = 21
    START_FIX_STAND = 22


class RobotArmMode(Enum):
    RESET = 0
    SHAKE_HAND = 1
    WAVE_HAND = 2
    SALUTE = 4
    BOWING = 5
    CARRY_BOX1 = 6
    CARRY_BOX2 = 7


class RobotControlMode(Enum):
    JOYSTICK = 0
    NAVIGATOR = 1


@dataclass(frozen=True)
class RuntimeConfig:
    script_dir: Path
    workspace_dir: Path
    max_vx: float
    max_vy: float
    max_dyaw: float
    deadband: float
    poll_interval: float
    reconnect_interval: float
    receiver_enabled: bool
    receiver_ip: str
    receiver_port: int
    receiver_client_port: int
    navi_status_url: str
    sudo_mode: str
    arm_node_cmd: str
    lock_file: Path
    joystick_keywords: Tuple[str, ...]


def log(msg: str) -> None:
    print(msg, flush=True)


class DdsCommandWriter:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._owns_rclpy_context = False

        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy_context = True

        self._node = rclpy.create_node("joylaunch_dds_writer")
        self._cmd_pub = self._node.create_publisher(Twist, "/humanoid/rl/teleop", 20)
        self._walk_mode_pub = self._node.create_publisher(Int32, "/humanoid/rl/walk_mode", 20)

    @property
    def arm_enabled(self) -> bool:
        return False

    def _spin_once(self) -> None:
        rclpy.spin_once(self._node, timeout_sec=0.0)

    def write_cmd(self, vx: float, vy: float, dyaw: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(dyaw)
        with self._lock:
            self._cmd_pub.publish(msg)
            self._spin_once()

    def write_walk_mode(self, mode) -> None:
        if isinstance(mode, RobotWalkMode):
            mode_value = int(mode.value)
            mode_label = mode.name
        else:
            mode_value = int(mode)
            mode_label = str(mode_value)

        msg = Int32()
        msg.data = mode_value
        with self._lock:
            self._walk_mode_pub.publish(msg)
            self._spin_once()
        log(f"[MODE] Mode command -> {mode_label}")

    def write_arm_mode(self, mode: RobotArmMode) -> None:
        _ = mode
        log("[MODE][WARN] Arm mode DDS channel is not configured")

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            pass
        if self._owns_rclpy_context and rclpy.ok():
            rclpy.shutdown()


class ProcessManager:
    def __init__(self, sudo_mode: str) -> None:
        self._sudo_mode = sudo_mode
        self._processes: Dict[str, subprocess.Popen] = {}
        self._order: List[str] = []

    def _stream(self, pipe, prefix: str) -> None:
        try:
            for line in iter(pipe.readline, ""):
                if not line:
                    break
                print(f"[{prefix}] {line.rstrip()}")
        finally:
            pipe.close()

    def _wrap_sudo(self, cmd: Sequence[str], use_sudo: bool) -> List[str]:
        if not use_sudo:
            return list(cmd)

        if os.geteuid() == 0:
            return list(cmd)

        if self._sudo_mode == "never":
            raise RuntimeError("sudo is disabled by --sudo-mode=never")

        # Use non-interactive sudo to avoid blocking on hidden password prompt.
        return ["sudo", "-n", *cmd]

    def launch(self, key: str, cmd: Sequence[str], use_sudo: bool = False) -> Optional[subprocess.Popen]:
        current = self._processes.get(key)
        if current is not None and current.poll() is None:
            log(f"[PROC] {key} is already running (pid={current.pid})")
            return current

        try:
            full_cmd = self._wrap_sudo(cmd, use_sudo)
            log(f"[PROC] Launch: {' '.join(full_cmd)}")
            proc = subprocess.Popen(
                full_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                stdin=None,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
            )
        except Exception as exc:
            log(f"[PROC][ERR] launch failed for {key}: {exc}")
            return None

        self._processes[key] = proc
        if key in self._order:
            self._order.remove(key)
        self._order.append(key)

        threading.Thread(target=self._stream, args=(proc.stdout, key), daemon=True).start()
        threading.Thread(target=self._stream, args=(proc.stderr, f"{key}-ERR"), daemon=True).start()
        return proc

    def launch_script(self, script_path: Path, use_sudo: bool = False, extra_args: Sequence[str] = ()) -> Optional[subprocess.Popen]:
        if not script_path.exists():
            log(f"[PROC][ERR] script not found: {script_path}")
            return None
        script_path.chmod(script_path.stat().st_mode | 0o111)
        key = script_path.name
        return self.launch(key, ["bash", str(script_path), *extra_args], use_sudo=use_sudo)

    def stop(self, key: str) -> None:
        proc = self._processes.get(key)
        if proc is None:
            return

        if proc.poll() is not None:
            self._processes.pop(key, None)
            if key in self._order:
                self._order.remove(key)
            return

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=3)
            log(f"[PROC] Stopped: {key}")
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            log(f"[PROC] Force killed: {key}")
        except Exception as exc:
            log(f"[PROC][ERR] stop failed for {key}: {exc}")
        finally:
            self._processes.pop(key, None)
            if key in self._order:
                self._order.remove(key)

    def stop_all(self) -> None:
        for key in list(reversed(self._order)):
            self.stop(key)


class JoystickStateTracker:
    def __init__(self, device_path: str) -> None:
        self._device = evdev.InputDevice(device_path)
        self._state_lock = threading.Lock()
        self._running = True
        self._state = {
            "left_joystick_x": 0,
            "left_joystick_y": 0,
            "right_joystick_x": 0,
            "right_joystick_y": 0,
            "lt": 0,
            "rt": 0,
            "dpad_x": 0,
            "dpad_y": 0,
            "btn_a": False,
            "btn_b": False,
            "btn_x": False,
            "btn_y": False,
            "btn_l1": False,
            "btn_r1": False,
            "btn_select": False,
            "btn_start": False,
            "btn_ls": False,
            "btn_rs": False,
        }

        self._thread = threading.Thread(target=self._event_loop, daemon=True)
        self._thread.start()

    def _event_loop(self) -> None:
        try:
            for event in self._device.read_loop():
                if not self._running:
                    break

                with self._state_lock:
                    if event.type == ecodes.EV_ABS:
                        if event.code == ecodes.ABS_X:
                            self._state["left_joystick_x"] = event.value
                        elif event.code == ecodes.ABS_Y:
                            self._state["left_joystick_y"] = event.value
                        elif event.code == ecodes.ABS_RX:
                            self._state["right_joystick_x"] = event.value
                        elif event.code == ecodes.ABS_RY:
                            self._state["right_joystick_y"] = event.value
                        elif event.code == ecodes.ABS_Z:
                            self._state["lt"] = event.value
                        elif event.code == ecodes.ABS_RZ:
                            self._state["rt"] = event.value
                        elif event.code == ecodes.ABS_HAT0X:
                            self._state["dpad_x"] = event.value
                        elif event.code == ecodes.ABS_HAT0Y:
                            self._state["dpad_y"] = event.value
                    elif event.type == ecodes.EV_KEY:
                        value = event.value == 1
                        keymap = {
                            ecodes.BTN_SOUTH: "btn_a",
                            ecodes.BTN_EAST: "btn_b",
                            ecodes.BTN_NORTH: "btn_x",
                            ecodes.BTN_WEST: "btn_y",
                            ecodes.BTN_TL: "btn_l1",
                            ecodes.BTN_TR: "btn_r1",
                            ecodes.BTN_SELECT: "btn_select",
                            ecodes.BTN_START: "btn_start",
                            ecodes.BTN_THUMBL: "btn_ls",
                            ecodes.BTN_THUMBR: "btn_rs",
                        }
                        mapped = keymap.get(event.code)
                        if mapped is not None:
                            self._state[mapped] = value
        except Exception:
            pass

    def get_state(self) -> Dict[str, object]:
        with self._state_lock:
            return dict(self._state)

    def is_connected(self) -> bool:
        return self._thread.is_alive()

    def stop(self) -> None:
        self._running = False
        try:
            self._device.close()
        except Exception:
            pass
        self._thread.join(timeout=1)


def find_joystick_device(keywords: Sequence[str]) -> Optional[evdev.InputDevice]:
    for path in evdev.list_devices():
        device = evdev.InputDevice(path)
        if any(token in device.name for token in keywords):
            log(f"[JOY] Found joystick: {device.path} ({device.name})")
            return device
    return None


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def acquire_single_instance_lock(lock_file: Path) -> int:
    if fcntl is None:
        log("[WARN] fcntl is unavailable, single-instance lock disabled")
        return -1

    fd = os.open(str(lock_file), os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        log(f"[ERR] Another joyLaunch instance is running ({lock_file})")
        sys.exit(1)
    return fd


class JoyLaunchApp:
    def __init__(self, cfg: RuntimeConfig) -> None:
        self.cfg = cfg
        self.process_mgr = ProcessManager(sudo_mode=cfg.sudo_mode)
        self.shared = DdsCommandWriter()

        self.receiver: Optional[Receiver] = None
        self.tracker: Optional[JoystickStateTracker] = None

        self.control_mode = RobotControlMode.JOYSTICK
        self.stop_event = threading.Event()
        self.triggered_flags: Dict[Tuple[str, ...], bool] = {}

        self._last_navi_status_query = 0.0

        self.combo_actions: List[Tuple[List[str], Callable[[], None]]] = []
        self._build_combo_actions()

    def _build_combo_actions(self) -> None:
        sdir = self.cfg.script_dir

        self.combo_actions = [
            (["btn_start"], lambda: self.process_mgr.launch_script(sdir / "solver.sh", use_sudo=True)),
            (["btn_l1", "btn_x"], lambda: self.process_mgr.launch_script(sdir / "imu.sh", use_sudo=True)),
            (["btn_l1", "btn_a"], lambda: self.process_mgr.launch_script(sdir / "controller.sh", use_sudo=True)),
            (["btn_l1", "btn_r1"], self.process_mgr.stop_all),
            (["lt", "btn_y"], lambda: self.process_mgr.launch_script(sdir / "driver.sh", use_sudo=True)),
            (["btn_l1", "dpad_y:-1"], lambda: self.shared.write_walk_mode(RobotWalkMode.WALK)),
            (["btn_l1", "dpad_y:1"], lambda: self.shared.write_walk_mode(RobotWalkMode.START_POLICY)),
            (["btn_l1", "btn_b"], lambda: self.shared.write_walk_mode(RobotWalkMode.STAND)),
            (["btn_l1", "btn_y"], lambda: self.shared.write_walk_mode(RobotWalkMode.FIX_STAND)),
            (["btn_l1", "btn_ls"], lambda: self.shared.write_walk_mode(RobotWalkMode.STOP_POLICY)),
            (["btn_l1", "btn_rs"], lambda: self.shared.write_walk_mode(RobotWalkMode.ZEROING)),
            (["lt", "btn_b"], lambda: self.shared.write_walk_mode(RobotWalkMode.ESTOP)),
            (["btn_l1", "dpad_x:1"], lambda: self._set_control_mode(RobotControlMode.JOYSTICK)),
            (["btn_l1", "dpad_x:-1"], lambda: self._set_control_mode(RobotControlMode.NAVIGATOR)),
        ]

        if self.cfg.arm_node_cmd.strip():
            arm_cmd = shlex.split(self.cfg.arm_node_cmd)
            if arm_cmd:
                self.combo_actions.append((["btn_select"], lambda: self.process_mgr.launch("arm_node", arm_cmd)))

        if self.shared.arm_enabled:
            self.combo_actions.extend(
                [
                    (["btn_r1", "dpad_y:-1"], lambda: self.shared.write_arm_mode(RobotArmMode.SHAKE_HAND)),
                    (["btn_r1", "dpad_y:1"], lambda: self.shared.write_arm_mode(RobotArmMode.WAVE_HAND)),
                    (["btn_r1", "dpad_x:1"], lambda: self.shared.write_arm_mode(RobotArmMode.SALUTE)),
                    (["btn_r1", "dpad_x:-1"], lambda: self.shared.write_arm_mode(RobotArmMode.BOWING)),
                    (["btn_r1", "btn_b"], lambda: self.shared.write_arm_mode(RobotArmMode.CARRY_BOX1)),
                    (["btn_r1", "btn_y"], lambda: self.shared.write_arm_mode(RobotArmMode.CARRY_BOX2)),
                    (["btn_r1", "btn_a"], lambda: self.shared.write_arm_mode(RobotArmMode.RESET)),
                ]
            )
        else:
            log("[MODE][WARN] ARM combo actions disabled (ARM_MODE shm unavailable)")

        self.triggered_flags = {tuple(keys): False for keys, _ in self.combo_actions}

    def _set_control_mode(self, mode: RobotControlMode) -> None:
        self.control_mode = mode
        log(f"[MODE] Control mode -> {mode.name}")
        if mode == RobotControlMode.NAVIGATOR:
            self.shared.write_walk_mode(RobotWalkMode.WALK)

    def _receiver_cmd_callback(self, cmd: CmdDataStruct) -> None:
        if self.control_mode != RobotControlMode.NAVIGATOR:
            return

        self.shared.write_cmd(cmd.linear_x, cmd.linear_y, cmd.angular_z)

        now = time.monotonic()
        if now - self._last_navi_status_query < 0.5:
            return

        self._last_navi_status_query = now
        try:
            assert self.receiver is not None
            navi_state = self.receiver.test_get_navi_status(self.cfg.navi_status_url)
            if navi_state == 3:
                log("[NAV] target reached, switch to STAND")
                self.shared.write_cmd(0.0, 0.0, 0.0)
                self.shared.write_walk_mode(RobotWalkMode.STAND)
        except Exception as exc:
            log(f"[NAV][WARN] status check failed: {exc}")

    def _receiver_error_callback(self, msg: str) -> None:
        log(f"[NAV][ERR] {msg}")

    @staticmethod
    def _combo_is_triggered(keys: Sequence[str], state: Dict[str, object]) -> bool:
        for key in keys:
            if ":" in key and key.startswith("dpad_"):
                axis, target = key.split(":", 1)
                if int(state.get(axis, 0)) != int(target):
                    return False
                continue

            if key == "lt":
                if int(state.get("lt", 0)) < 50:
                    return False
                continue

            if not bool(state.get(key, False)):
                return False

        return True

    def _handle_combo_actions(self, state: Dict[str, object]) -> None:
        for keys, action in self.combo_actions:
            key_tuple = tuple(keys)
            triggered = self._combo_is_triggered(keys, state)

            if triggered and not self.triggered_flags.get(key_tuple, False):
                log(f"[COMBO] Triggered {keys}")
                try:
                    action()
                except Exception as exc:
                    log(f"[COMBO][ERR] {keys}: {exc}")
                self.triggered_flags[key_tuple] = True
            elif not triggered:
                self.triggered_flags[key_tuple] = False

    def _compute_cmd(self, state: Dict[str, object]) -> Tuple[float, float, float]:
        vx = -float(state["left_joystick_y"]) / 32767.0 * self.cfg.max_vx
        vy = -float(state["left_joystick_x"]) / 32767.0 * self.cfg.max_vy
        dyaw = -float(state["right_joystick_x"]) / 32767.0 * self.cfg.max_dyaw

        vx = apply_deadband(vx, self.cfg.deadband)
        vy = apply_deadband(vy, self.cfg.deadband)
        dyaw = apply_deadband(dyaw, self.cfg.deadband)
        return vx, vy, dyaw

    def _ensure_tracker(self) -> bool:
        if self.tracker is not None and self.tracker.is_connected():
            return True

        if self.tracker is not None:
            self.tracker.stop()
            self.tracker = None

        device = find_joystick_device(self.cfg.joystick_keywords)
        if device is None:
            log("[JOY] joystick not found, retry later")
            return False

        self.tracker = JoystickStateTracker(device.path)
        log("[JOY] joystick connected")
        return True

    def _cleanup(self) -> None:
        self.shared.write_cmd(0.0, 0.0, 0.0)
        self.shared.close()

        if self.receiver is not None:
            self.receiver.stop()

        if self.tracker is not None:
            self.tracker.stop()
            self.tracker = None

        self.process_mgr.stop_all()

    def run(self) -> int:
        def _request_stop(signum, _frame) -> None:
            log(f"[SYS] Signal {signum}, stopping...")
            self.stop_event.set()

        signal.signal(signal.SIGINT, _request_stop)
        signal.signal(signal.SIGTERM, _request_stop)

        if self.cfg.receiver_enabled:
            self.receiver = Receiver(
                server_ip=self.cfg.receiver_ip,
                server_port=self.cfg.receiver_port,
                client_port=self.cfg.receiver_client_port,
            )
            started = self.receiver.start(self._receiver_cmd_callback, self._receiver_error_callback)
            if not started:
                log("[NAV][WARN] receiver already started or failed to start")

        log("[SYS] joyLaunch started")
        try:
            while not self.stop_event.is_set():
                if not self._ensure_tracker():
                    time.sleep(self.cfg.reconnect_interval)
                    continue

                assert self.tracker is not None
                state = self.tracker.get_state()

                self._handle_combo_actions(state)

                if self.control_mode == RobotControlMode.JOYSTICK:
                    vx, vy, dyaw = self._compute_cmd(state)
                    self.shared.write_cmd(vx, vy, dyaw)

                time.sleep(self.cfg.poll_interval)
        finally:
            self._cleanup()

        log("[SYS] joyLaunch exited")
        return 0


def parse_args() -> RuntimeConfig:
    file_script_dir = Path(__file__).resolve().parent
    default_workspace = file_script_dir.parent

    parser = argparse.ArgumentParser(description="Joystick launcher for humanoid RL deployment")
    parser.add_argument("--workspace", type=Path, default=default_workspace)

    parser.add_argument("--max-vx", type=float, default=0.5)
    parser.add_argument("--max-vy", type=float, default=0.02)
    parser.add_argument("--max-dyaw", type=float, default=0.3)
    parser.add_argument("--deadband", type=float, default=0.05)
    parser.add_argument("--poll-interval", type=float, default=0.1)
    parser.add_argument("--reconnect-interval", type=float, default=3.0)

    parser.add_argument("--receiver-ip", default="192.168.168.125")
    parser.add_argument("--receiver-port", type=int, default=8888)
    parser.add_argument("--receiver-client-port", type=int, default=9999)
    parser.add_argument("--navi-status-url", default="http://192.168.168.125:10000")
    parser.add_argument("--disable-receiver", action="store_true")

    parser.add_argument("--sudo-mode", choices=["auto", "always", "never"], default="auto")
    parser.add_argument("--arm-node-cmd", default="")
    parser.add_argument("--lock-file", type=Path, default=Path("/tmp/joyLaunch.lock"))
    parser.add_argument("--joystick-keywords", default="X-Box,Xbox")

    args = parser.parse_args()

    workspace_dir = args.workspace.resolve()
    script_dir = (workspace_dir / "script").resolve()

    if not script_dir.exists():
        script_dir = file_script_dir

    keywords = tuple(k.strip() for k in args.joystick_keywords.split(",") if k.strip())
    if not keywords:
        keywords = ("Xbox",)

    return RuntimeConfig(
        script_dir=script_dir,
        workspace_dir=workspace_dir,
        max_vx=args.max_vx,
        max_vy=args.max_vy,
        max_dyaw=args.max_dyaw,
        deadband=args.deadband,
        poll_interval=args.poll_interval,
        reconnect_interval=args.reconnect_interval,
        receiver_enabled=not args.disable_receiver,
        receiver_ip=args.receiver_ip,
        receiver_port=args.receiver_port,
        receiver_client_port=args.receiver_client_port,
        navi_status_url=args.navi_status_url,
        sudo_mode=args.sudo_mode,
        arm_node_cmd=args.arm_node_cmd,
        lock_file=args.lock_file,
        joystick_keywords=keywords,
    )


def main() -> int:
    cfg = parse_args()

    if EVDEV_IMPORT_ERROR is not None:
        log(f"[ERR] Missing dependency evdev: {EVDEV_IMPORT_ERROR}")
        log("[ERR] Install with: sudo apt install python3-evdev")
        return 1

    if RCLPY_IMPORT_ERROR is not None:
        log(f"[ERR] Missing dependency rclpy: {RCLPY_IMPORT_ERROR}")
        log("[ERR] Install ROS2 python environment and source setup.bash")
        return 1

    if cfg.receiver_enabled and RECEIVER_IMPORT_ERROR is not None:
        log(f"[ERR] receiver.py import failed: {RECEIVER_IMPORT_ERROR}")
        return 1

    lock_fd = acquire_single_instance_lock(cfg.lock_file)
    try:
        app = JoyLaunchApp(cfg)
        return app.run()
    finally:
        if lock_fd >= 0:
            os.close(lock_fd)


if __name__ == "__main__":
    sys.exit(main())

