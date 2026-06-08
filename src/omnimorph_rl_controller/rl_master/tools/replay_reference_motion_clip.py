#!/usr/bin/env python3
"""Replay a reference motion npz in MuJoCo and interactively choose a clip."""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from prepare_reference_motion_clip import prepare_motion_clip


PERM_BM_TO_URDF = np.array(
    [0, 3, 6, 9, 13, 17, 1, 4, 7, 10, 14, 18, 2, 5, 8, 11, 15, 19, 21, 23, 25, 27, 12, 16, 20, 22, 24, 26, 28],
    dtype=np.int64,
)


try:
    import glfw
except Exception:  # pragma: no cover - glfw is provided by MuJoCo viewer on normal installs.
    glfw = None


def key_code(name: str, fallback: int) -> int:
    if glfw is None:
        return fallback
    return int(getattr(glfw, name, fallback))


KEY = {
    "space": key_code("KEY_SPACE", 32),
    "right": key_code("KEY_RIGHT", 262),
    "left": key_code("KEY_LEFT", 263),
    "page_up": key_code("KEY_PAGE_UP", 266),
    "page_down": key_code("KEY_PAGE_DOWN", 267),
    "home": key_code("KEY_HOME", 268),
    "end": key_code("KEY_END", 269),
    "q": ord("Q"),
    "s": ord("S"),
    "e": ord("E"),
    "w": ord("W"),
    "a": ord("A"),
    "b": ord("B"),
    "r": ord("R"),
    "y": ord("Y"),
    "lbracket": ord("["),
    "rbracket": ord("]"),
}


def yaw_from_quat_wxyz(q: np.ndarray) -> float:
    q = np.asarray(q, dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def yaw_quat_wxyz(yaw: float) -> np.ndarray:
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


def quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    q = np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )
    norm = np.linalg.norm(q)
    return q / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def rotation_z(yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


class MotionReplayApp:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.motion_path = args.motion
        self.output_path = args.output
        self.model = mujoco.MjModel.from_xml_path(str(args.xml))
        self.data = mujoco.MjData(self.model)
        self.motion = self.load_motion(args.motion)
        self.fps = float(np.asarray(self.motion["fps"]).reshape(-1)[0])
        self.dt = 1.0 / self.fps
        self.frame_count = int(self.motion["joint_pos"].shape[0])
        self.frame = max(0, min(args.start_frame, self.frame_count - 1))
        self.mark_start = self.frame
        self.mark_end = self.frame_count - 1
        self.playing = False
        self.align_on_save = not args.no_align_on_save
        self.recompute_velocities = not args.keep_velocities
        self.align_display = not args.no_align_display
        self.show_reference_bodies = args.show_reference_bodies
        self.status = "Ready"
        self.last_tick = time.monotonic()
        self.last_text_update = 0.0
        self.joint_qpos_addr = self.resolve_joint_qpos_addresses()
        self.viewer = None
        self.terminal_state = None
        self.terminal_buffer = ""
        self.display_anchor_frame = self.frame
        self.display_origin_xy = np.zeros(3, dtype=np.float64)
        self.display_yaw_inv = 0.0
        self.display_yaw_inv_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.display_rot = np.eye(3, dtype=np.float64)
        self.update_display_alignment(self.display_anchor_frame)

    @staticmethod
    def load_motion(path: Path) -> dict[str, np.ndarray]:
        with np.load(path, allow_pickle=False) as data:
            required = ["fps", "joint_pos", "body_pos_w", "body_quat_w"]
            missing = [key for key in required if key not in data.files]
            if missing:
                raise KeyError(f"{path} is missing required keys: {missing}")
            return {key: data[key] for key in data.files}

    def resolve_joint_qpos_addresses(self) -> np.ndarray:
        addrs: list[int] = []
        for joint_id in range(self.model.njnt):
            joint_type = self.model.jnt_type[joint_id]
            if joint_type == mujoco.mjtJoint.mjJNT_FREE:
                continue
            if joint_type != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError("Only free root + hinge joints are supported by this replay tool.")
            addrs.append(int(self.model.jnt_qposadr[joint_id]))
        if len(addrs) != self.motion["joint_pos"].shape[1]:
            raise ValueError(
                f"MuJoCo hinge joints ({len(addrs)}) do not match motion joint_pos dim "
                f"({self.motion['joint_pos'].shape[1]})."
            )
        return np.asarray(addrs, dtype=np.int64)

    def set_frame(self, frame: int) -> None:
        self.frame = max(0, min(frame, self.frame_count - 1))

    def update_display_alignment(self, frame: int) -> None:
        self.display_anchor_frame = max(0, min(frame, self.frame_count - 1))
        root_body = self.args.root_body_index
        self.display_origin_xy = self.motion["body_pos_w"][self.display_anchor_frame, root_body].astype(np.float64).copy()
        self.display_origin_xy[2] = 0.0
        self.display_yaw_inv = -yaw_from_quat_wxyz(self.motion["body_quat_w"][self.display_anchor_frame, root_body])
        self.display_yaw_inv_quat = yaw_quat_wxyz(self.display_yaw_inv)
        self.display_rot = rotation_z(self.display_yaw_inv)

    def display_body_positions(self, frame: int) -> np.ndarray:
        body_pos = self.motion["body_pos_w"][frame].astype(np.float64)
        if not self.align_display:
            return body_pos
        return (body_pos - self.display_origin_xy) @ self.display_rot.T

    def display_body_quat(self, frame: int, body_index: int) -> np.ndarray:
        quat = self.motion["body_quat_w"][frame, body_index].astype(np.float64)
        if not self.align_display:
            return quat
        return quat_mul_wxyz(self.display_yaw_inv_quat, quat)

    def apply_frame_to_mujoco(self) -> None:
        root_body = self.args.root_body_index
        body_pos = self.display_body_positions(self.frame)
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        self.data.qpos[0:3] = body_pos[root_body]
        self.data.qpos[3:7] = self.display_body_quat(self.frame, root_body)
        self.data.qpos[self.joint_qpos_addr] = self.motion["joint_pos"][self.frame, PERM_BM_TO_URDF]
        mujoco.mj_forward(self.model, self.data)

    def update_reference_markers(self) -> None:
        if self.viewer is None:
            return
        scene = self.viewer.user_scn
        if not self.show_reference_bodies:
            scene.ngeom = 0
            return
        body_pos = self.display_body_positions(self.frame)
        geom_count = min(body_pos.shape[0], scene.maxgeom)
        scene.ngeom = geom_count
        rgba = np.array([1.0, 0.72, 0.05, 0.85], dtype=np.float32)
        mat = np.eye(3, dtype=np.float64).reshape(-1)
        size = np.array([0.035, 0.0, 0.0], dtype=np.float64)
        for i in range(geom_count):
            mujoco.mjv_initGeom(
                scene.geoms[i],
                mujoco.mjtGeom.mjGEOM_SPHERE,
                size,
                body_pos[i].astype(np.float64),
                mat,
                rgba,
            )

    def save_clip(self) -> None:
        start = min(self.mark_start, self.mark_end)
        end = max(self.mark_start, self.mark_end)
        info = prepare_motion_clip(
            self.motion_path,
            self.output_path,
            start,
            end,
            anchor_body_index=self.args.anchor_body_index,
            align_xy_yaw=self.align_on_save,
            recompute_velocities=self.recompute_velocities,
            force=True,
        )
        self.status = (
            f"Saved {self.output_path.name}: {info['start_frame']}..{info['end_frame']} "
            f"({info['output_frames']} frames)"
        )
        print(self.status)

    def key_callback(self, key: int) -> None:
        key = int(key)
        if key == KEY["space"]:
            self.playing = not self.playing
        elif key in (KEY["right"], ord(".")):
            self.playing = False
            self.set_frame(self.frame + 1)
        elif key in (KEY["left"], ord(",")):
            self.playing = False
            self.set_frame(self.frame - 1)
        elif key in (KEY["rbracket"], KEY["page_up"]):
            self.playing = False
            self.set_frame(self.frame + self.args.big_step)
        elif key in (KEY["lbracket"], KEY["page_down"]):
            self.playing = False
            self.set_frame(self.frame - self.args.big_step)
        elif key == KEY["home"]:
            self.playing = False
            self.set_frame(0)
        elif key == KEY["end"]:
            self.playing = False
            self.set_frame(self.frame_count - 1)
        elif key == KEY["s"]:
            self.mark_start = self.frame
            self.update_display_alignment(self.frame)
            self.status = f"Start frame set to {self.mark_start}; display alignment anchor updated"
            print(self.status)
        elif key == KEY["e"]:
            self.mark_end = self.frame
            self.status = f"End frame set to {self.mark_end}"
            print(self.status)
        elif key == KEY["a"]:
            self.align_on_save = not self.align_on_save
            self.status = f"Align xy/yaw on save: {self.align_on_save}"
            print(self.status)
        elif key == KEY["b"]:
            self.show_reference_bodies = not self.show_reference_bodies
            self.status = f"Reference body markers: {self.show_reference_bodies}"
            print(self.status)
        elif key == KEY["r"]:
            self.recompute_velocities = not self.recompute_velocities
            self.status = f"Recompute velocities on save: {self.recompute_velocities}"
            print(self.status)
        elif key == KEY["y"]:
            self.align_display = not self.align_display
            self.status = f"Display xy/yaw alignment: {self.align_display}"
            print(self.status)
        elif key == KEY["w"]:
            try:
                self.save_clip()
            except Exception as exc:
                self.status = f"Save failed: {exc}"
                print(self.status)
        elif key == KEY["q"]:
            if self.viewer is not None:
                self.viewer.close()

    def terminal_key_to_viewer_key(self, char: str) -> int | None:
        mapping = {
            " ": KEY["space"],
            "l": KEY["right"],
            "L": KEY["right"],
            "j": KEY["left"],
            "J": KEY["left"],
            ".": KEY["right"],
            ",": KEY["left"],
            "]": KEY["rbracket"],
            "[": KEY["lbracket"],
            "s": KEY["s"],
            "S": KEY["s"],
            "e": KEY["e"],
            "E": KEY["e"],
            "w": KEY["w"],
            "W": KEY["w"],
            "a": KEY["a"],
            "A": KEY["a"],
            "b": KEY["b"],
            "B": KEY["b"],
            "r": KEY["r"],
            "R": KEY["r"],
            "y": KEY["y"],
            "Y": KEY["y"],
            "q": KEY["q"],
            "Q": KEY["q"],
        }
        return mapping.get(char)

    def process_terminal_input(self) -> None:
        if self.args.no_terminal_keys or not sys.stdin.isatty():
            return
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not readable:
                return
            chunk = sys.stdin.read(1)
            if not chunk:
                return
            self.terminal_buffer += chunk
            if self.terminal_buffer.endswith("\x1b[C"):
                self.key_callback(KEY["right"])
                self.terminal_buffer = ""
            elif self.terminal_buffer.endswith("\x1b[D"):
                self.key_callback(KEY["left"])
                self.terminal_buffer = ""
            elif self.terminal_buffer.endswith("\x1b[5~"):
                self.key_callback(KEY["page_up"])
                self.terminal_buffer = ""
            elif self.terminal_buffer.endswith("\x1b[6~"):
                self.key_callback(KEY["page_down"])
                self.terminal_buffer = ""
            elif len(self.terminal_buffer) == 1 and self.terminal_buffer != "\x1b":
                key = self.terminal_key_to_viewer_key(self.terminal_buffer)
                if key is not None:
                    self.key_callback(key)
                self.terminal_buffer = ""
            elif len(self.terminal_buffer) > 5:
                self.terminal_buffer = ""

    def enable_terminal_keys(self) -> None:
        if self.args.no_terminal_keys or not sys.stdin.isatty():
            return
        self.terminal_state = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self) -> None:
        if self.terminal_state is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_state)
            self.terminal_state = None

    def overlay_texts(self) -> list[tuple[int, int, str, str]]:
        raw_yaw_deg = math.degrees(yaw_from_quat_wxyz(self.motion["body_quat_w"][self.frame, self.args.root_body_index]))
        display_yaw_deg = math.degrees(yaw_from_quat_wxyz(self.display_body_quat(self.frame, self.args.root_body_index)))
        clip_start = min(self.mark_start, self.mark_end)
        clip_end = max(self.mark_start, self.mark_end)
        return [
            (
                int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                0,
                "Frame",
                f"{self.frame}/{self.frame_count - 1}  t={self.frame / self.fps:.3f}s  raw_yaw={raw_yaw_deg:.1f}deg  display_yaw={display_yaw_deg:.1f}deg",
            ),
            (
                int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                1,
                "Clip",
                f"{clip_start}..{clip_end}  ({(clip_end - clip_start + 1) / self.fps:.3f}s)",
            ),
            (
                int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                2,
                "Save",
                f"{self.output_path}  save_align={self.align_on_save}  display_align={self.align_display}@{self.display_anchor_frame}  recompute_vel={self.recompute_velocities}",
            ),
            (
                int(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT),
                0,
                "Keys",
                "Window: Space/Arrows | Terminal: Space j/l [/] | S start+align | E end | Y display | A save-align | R vel | B bodies | W save | Q quit",
            ),
            (int(mujoco.mjtGridPos.mjGRID_BOTTOMLEFT), 1, "Status", self.status),
        ]

    def run(self) -> None:
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self.key_callback,
            show_left_ui=True,
            show_right_ui=True,
        )
        print("MuJoCo replay started.")
        print("Window keys: Space play, Left/Right step, [/]/PgUp/PgDn jump, S start+display-align, E end, Y display align, A save align, R velocities, B bodies, W save, Q quit.")
        print("Terminal keys: Space play, j/l step, [/ ] jump, S start+display-align, E end, Y display align, W save, Q quit.")
        self.enable_terminal_keys()
        try:
            while self.viewer.is_running():
                now = time.monotonic()
                self.process_terminal_input()
                if self.playing and now - self.last_tick >= self.dt / max(self.args.speed, 1e-6):
                    self.set_frame(self.frame + 1)
                    if self.frame >= self.frame_count - 1:
                        self.playing = False
                    self.last_tick = now
                with self.viewer.lock():
                    self.apply_frame_to_mujoco()
                    self.update_reference_markers()
                if now - self.last_text_update > 0.1:
                    self.viewer.set_texts(self.overlay_texts())
                    self.last_text_update = now
                self.viewer.sync()
                time.sleep(1.0 / 60.0)
        finally:
            self.restore_terminal()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay an OmniXtreme reference motion in MuJoCo, mark start/end frames, and save a processed clip."
    )
    parser.add_argument("--motion", type=Path, default=Path("/home/edify/Code/OmniXtreme/policy/motion.npz"))
    parser.add_argument("--xml", type=Path, default=Path("/home/edify/Code/OmniXtreme/robots/g1/no_hand.xml"))
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/home/edify/Code/OmniXtreme/policy/motion_selected_aligned.npz"),
    )
    parser.add_argument("--start-frame", type=int, default=0, help="Initial frame shown by the viewer.")
    parser.add_argument("--root-body-index", type=int, default=0, help="Body used as MuJoCo free-joint pose.")
    parser.add_argument("--anchor-body-index", type=int, default=0, help="Body used for saved clip xy/yaw alignment.")
    parser.add_argument("--big-step", type=int, default=25, help="Frame step for [/] and PageUp/PageDown.")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument(
        "--no-align-display",
        action="store_true",
        help="Show raw world pose instead of yaw/xy-aligning the viewer to the display anchor frame.",
    )
    parser.add_argument(
        "--show-reference-bodies",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Show motion body positions as yellow markers.",
    )
    parser.add_argument("--no-align-on-save", action="store_true")
    parser.add_argument("--keep-velocities", action="store_true")
    parser.add_argument("--no-terminal-keys", action="store_true", help="Disable keyboard control from the terminal.")
    return parser.parse_args()


def main() -> None:
    app = MotionReplayApp(parse_args())
    app.run()


if __name__ == "__main__":
    main()
