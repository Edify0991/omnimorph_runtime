#!/usr/bin/env python3
"""Generate fixed-layout deploy architecture PNG diagrams.

The diagrams intentionally summarize the runtime at the deployment boundary:
external operator/config inputs, the shared policy-control core, and the two
environment-specific execution paths.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable, Sequence

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
ZH_FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
EN_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
EN_BOLD_FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"


def font(path: str, size: int, index: int = 0) -> ImageFont.FreeTypeFont:
    try:
        return ImageFont.truetype(path, size=size, index=index)
    except OSError:
        return ImageFont.load_default()


def text_size(draw: ImageDraw.ImageDraw, text: str, fnt: ImageFont.ImageFont) -> tuple[int, int]:
    box = draw.textbbox((0, 0), text, font=fnt)
    return box[2] - box[0], box[3] - box[1]


def wrap_line(
    draw: ImageDraw.ImageDraw,
    text: str,
    fnt: ImageFont.ImageFont,
    max_width: int,
    cjk: bool,
) -> list[str]:
    if not text:
        return [""]
    if not cjk and " " in text:
        words = text.split(" ")
        lines: list[str] = []
        cur = ""
        for word in words:
            trial = word if not cur else f"{cur} {word}"
            if text_size(draw, trial, fnt)[0] <= max_width:
                cur = trial
            else:
                if cur:
                    lines.append(cur)
                cur = word
        if cur:
            lines.append(cur)
        return lines

    lines = []
    cur = ""
    for ch in text:
        trial = cur + ch
        if text_size(draw, trial, fnt)[0] <= max_width:
            cur = trial
        else:
            if cur:
                lines.append(cur)
            cur = ch
    if cur:
        lines.append(cur)
    return lines


def wrap_text(
    draw: ImageDraw.ImageDraw,
    text: str,
    fnt: ImageFont.ImageFont,
    max_width: int,
    cjk: bool,
) -> list[str]:
    wrapped: list[str] = []
    for raw in text.split("\n"):
        wrapped.extend(wrap_line(draw, raw, fnt, max_width, cjk))
    return wrapped


def rounded_box(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    fill: str,
    outline: str,
    width: int = 3,
    radius: int = 22,
) -> None:
    draw.rounded_rectangle(rect, radius=radius, fill=fill, outline=outline, width=width)


def draw_centered_lines(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    lines: Sequence[str],
    fnt: ImageFont.ImageFont,
    fill: str,
    spacing: int = 8,
) -> None:
    heights = [text_size(draw, line, fnt)[1] for line in lines]
    total_h = sum(heights) + spacing * max(0, len(lines) - 1)
    y = rect[1] + (rect[3] - rect[1] - total_h) / 2
    for line, h in zip(lines, heights):
        w = text_size(draw, line, fnt)[0]
        draw.text(((rect[0] + rect[2] - w) / 2, y), line, font=fnt, fill=fill)
        y += h + spacing


def draw_node(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    title: str,
    body: str,
    fonts: dict[str, ImageFont.ImageFont],
    *,
    cjk: bool,
    fill: str = "#ffffff",
    outline: str = "#334155",
    title_fill: str = "#0f172a",
    body_fill: str = "#475569",
) -> None:
    rounded_box(draw, rect, fill, outline)
    x1, y1, x2, y2 = rect
    pad_x = 22
    title_lines = wrap_text(draw, title, fonts["node_title"], x2 - x1 - pad_x * 2, cjk)
    body_lines = wrap_text(draw, body, fonts["node_body"], x2 - x1 - pad_x * 2, cjk)
    title_heights = [text_size(draw, line, fonts["node_title"])[1] for line in title_lines]
    body_heights = [text_size(draw, line, fonts["node_body"])[1] for line in body_lines]
    total_h = (
        sum(title_heights)
        + 8 * max(0, len(title_lines) - 1)
        + 16
        + sum(body_heights)
        + 7 * max(0, len(body_lines) - 1)
    )
    y = y1 + (y2 - y1 - total_h) / 2
    for line, h in zip(title_lines, title_heights):
        w = text_size(draw, line, fonts["node_title"])[0]
        draw.text(((x1 + x2 - w) / 2, y), line, font=fonts["node_title"], fill=title_fill)
        y += h + 8
    y += 8
    for line, h in zip(body_lines, body_heights):
        w = text_size(draw, line, fonts["node_body"])[0]
        draw.text(((x1 + x2 - w) / 2, y), line, font=fonts["node_body"], fill=body_fill)
        y += h + 7


def draw_section(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    title: str,
    fonts: dict[str, ImageFont.ImageFont],
    *,
    fill: str,
    outline: str,
) -> None:
    rounded_box(draw, rect, fill, outline, width=3, radius=28)
    x1, y1, x2, _ = rect
    w = text_size(draw, title, fonts["section"])[0]
    draw.text(((x1 + x2 - w) / 2, y1 + 22), title, font=fonts["section"], fill="#0f172a")


def arrow_head(
    draw: ImageDraw.ImageDraw,
    p1: tuple[int, int],
    p2: tuple[int, int],
    color: str,
    size: int = 16,
) -> None:
    ang = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    left = (p2[0] - size * math.cos(ang - math.pi / 6), p2[1] - size * math.sin(ang - math.pi / 6))
    right = (p2[0] - size * math.cos(ang + math.pi / 6), p2[1] - size * math.sin(ang + math.pi / 6))
    draw.polygon([p2, left, right], fill=color)


def draw_arrow(
    draw: ImageDraw.ImageDraw,
    points: Sequence[tuple[int, int]],
    *,
    color: str = "#334155",
    width: int = 4,
    dash: bool = False,
    label: str | None = None,
    label_font: ImageFont.ImageFont | None = None,
) -> None:
    if dash:
        for a, b in zip(points, points[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            if seg_len == 0:
                continue
            ux, uy = (b[0] - a[0]) / seg_len, (b[1] - a[1]) / seg_len
            step = 24
            on = 14
            d = 0
            while d < seg_len:
                p_start = (int(a[0] + ux * d), int(a[1] + uy * d))
                p_end = (int(a[0] + ux * min(d + on, seg_len)), int(a[1] + uy * min(d + on, seg_len)))
                draw.line([p_start, p_end], fill=color, width=width)
                d += step
    else:
        draw.line(list(points), fill=color, width=width, joint="curve")
    arrow_head(draw, points[-2], points[-1], color)
    if label and label_font:
        mid = points[len(points) // 2]
        box = draw.textbbox((0, 0), label, font=label_font)
        pad = 7
        rect = (mid[0] - (box[2] - box[0]) // 2 - pad, mid[1] - 24, mid[0] + (box[2] - box[0]) // 2 + pad, mid[1] + 8)
        draw.rounded_rectangle(rect, radius=8, fill="#fbfaf7")
        draw.text((rect[0] + pad, rect[1] + 4), label, font=label_font, fill=color)


def node_center(rect: tuple[int, int, int, int]) -> tuple[int, int]:
    return ((rect[0] + rect[2]) // 2, (rect[1] + rect[3]) // 2)


def node_right(rect: tuple[int, int, int, int]) -> tuple[int, int]:
    return (rect[2], (rect[1] + rect[3]) // 2)


def node_left(rect: tuple[int, int, int, int]) -> tuple[int, int]:
    return (rect[0], (rect[1] + rect[3]) // 2)


def node_top(rect: tuple[int, int, int, int]) -> tuple[int, int]:
    return ((rect[0] + rect[2]) // 2, rect[1])


def node_bottom(rect: tuple[int, int, int, int]) -> tuple[int, int]:
    return ((rect[0] + rect[2]) // 2, rect[3])


def build_text(lang: str) -> dict[str, object]:
    if lang == "zh":
        return {
            "title": "人形机器人 RL 部署运行时架构与数据流",
            "subtitle": "实机 RL_solver 与 MuJoCo sim2sim 共用 IntegratedControllerRuntime / RL_controller 控制核心",
            "sections": ("外部输入与配置资产", "共享策略控制核心", "执行后端与可观测输出"),
            "legend": ("蓝色：状态/观测数据", "紫色：策略推理链", "橙色：控制命令", "虚线：遥测/日志/反馈"),
            "nodes": {
                "operator": ("操作员 GUI / 脚本", "omnimorph_ops_gui.py、手柄、shell 工具"),
                "topics": ("DDS / ROS 2 控制话题", "/omnimorph/rl/teleop\n/omnimorph/rl/mode_control"),
                "imu": ("IMU 输入", "/imu/yesense\n仅实机路径使用"),
                "cfg": ("模式与机器人配置", "rl_cfg_jc01.yaml + profiles/*.yaml\n关节顺序、增益、限幅、策略组"),
                "manifest": ("观测清单", "observation_manifest_*.yaml\n观测项顺序与维度合同"),
                "policy": ("策略资产", "policies/*.onnx"),
                "ref": ("参考轨迹 / 外部特征", "ReferenceMotionProvider\nExternalObservationProvider"),
                "registry": ("ModeProfileRegistry", "一次性加载 deploy_mode_profiles\n按 mode 解析 config_section"),
                "runtime": ("IntegratedControllerRuntime", "缓存最新 teleop 与 mode 命令\n计算 phase_t，调用控制器 step"),
                "controller": ("RL_controller::step", "更新状态/命令、模式切换、观测构造、策略推理、命令组装"),
                "state_machine": ("DeployStateMachine", "回零、启动、停止、急停\n解析 mode/lifecycle 控制字"),
                "profile": ("当前 ModeProfile", "obs/action/reference 关节合同\npolicy adapter 与 strategy 设置"),
                "feature": ("ObservationFeatureContext", "RobotState + teleop + phase_t\nlast_action + 参考/外部特征"),
                "obs": ("ObservationBuilder", "按 manifest 注册表拼接\n确定性的 obs 向量布局"),
                "stack": ("观测历史", "obs deque\nstacked observation buffer"),
                "strategy": ("PolicyInferenceStrategy", "sync weighted\n或 chunked receding"),
                "adapter": ("OnnxPolicyAdapter", "统一策略后端接口"),
                "runner": ("OnnxPolicyRunner", "ONNX Runtime 推理\naction + 可选 extra outputs"),
                "command": ("RobotCommandData", "目标 q、力矩、增益\nruntime command mode"),
                "real": ("实机路径", "RL_solver / RobotSolver\nDDS 输入缓存、电机状态读取、HOLD 处理"),
                "shm": ("电机共享内存 I/O", "MotorIoBackend + 运动学转换\n读关节反馈 / 写目标与力矩"),
                "robot": ("机器人电机与传感器", "joint q/dq、力矩、状态"),
                "sim": ("MuJoCo sim2sim 路径", "mujoco_sim_bridge C++ 后端\n从 qpos/qvel/base/body 构造 RobotState"),
                "act": ("MuJoCo 执行器写回", "策略关节 + HOLD 关节\nmj_step / hold"),
                "viewer": ("查看器 / 前端镜像", "GLFW 或 Python viewer 话题"),
                "telemetry": ("可观测输出", "/omnimorph/rl/state\nviewer_frame / inspector\nruntime log / MCAP"),
            },
            "labels": {
                "control": "速度 + 模式",
                "mode": "模式合同",
                "obs_contract": "观测合同",
                "model": "模型文件",
                "features": "参考/传感特征",
                "state_in": "RobotState + 输入",
                "obs_vec": "obs 向量",
                "policy": "推理",
                "action": "action",
                "cmd": "控制命令",
                "feedback": "关节反馈",
                "telemetry": "遥测/日志",
            },
        }
    return {
        "title": "Humanoid RL Deploy Runtime Architecture and Data Flow",
        "subtitle": "Real-robot RL_solver and MuJoCo sim2sim share IntegratedControllerRuntime / RL_controller",
        "sections": ("External Inputs and Assets", "Shared Policy-Control Core", "Execution Backends and Observability"),
        "legend": ("Blue: state/observation data", "Purple: policy inference chain", "Orange: control commands", "Dashed: telemetry/logging/feedback"),
        "nodes": {
            "operator": ("Operator GUI / Scripts", "omnimorph_ops_gui.py, joystick, shell tools"),
            "topics": ("DDS / ROS 2 Control Topics", "/omnimorph/rl/teleop\n/omnimorph/rl/mode_control"),
            "imu": ("IMU Input", "/imu/yesense\nreal robot path only"),
            "cfg": ("Mode and Robot Config", "rl_cfg_jc01.yaml + profiles/*.yaml\njoint order, gains, limits, policy groups"),
            "manifest": ("Observation Manifests", "observation_manifest_*.yaml\nterm order and feature dimensions"),
            "policy": ("Policy Assets", "policies/*.onnx"),
            "ref": ("Reference / External Features", "ReferenceMotionProvider\nExternalObservationProvider"),
            "registry": ("ModeProfileRegistry", "loads deploy_mode_profiles once\nresolves config_section per mode"),
            "runtime": ("IntegratedControllerRuntime", "caches latest teleop and mode command\ncomputes phase_t, calls controller step"),
            "controller": ("RL_controller::step", "state/command update, mode switch, observation build, policy inference, command assembly"),
            "state_machine": ("DeployStateMachine", "zero, start, stop, e-stop\nmode/lifecycle control words"),
            "profile": ("Active ModeProfile", "obs/action/reference joint contracts\npolicy adapter and strategy settings"),
            "feature": ("Observation Feature Context", "RobotState + teleop + phase_t\nlast_action + reference/external features"),
            "obs": ("ObservationBuilder", "manifest registry assembly\ndeterministic obs vector layout"),
            "stack": ("Observation History", "obs deque\nstacked observation buffer"),
            "strategy": ("PolicyInferenceStrategy", "sync weighted\nor chunked receding"),
            "adapter": ("OnnxPolicyAdapter", "backend-neutral policy interface"),
            "runner": ("OnnxPolicyRunner", "ONNX Runtime inference\naction + optional extra outputs"),
            "command": ("RobotCommandData", "target q, torque, gains\nruntime command mode"),
            "real": ("Real-Robot Path", "RL_solver / RobotSolver\nDDS input cache, motor state read, HOLD handling"),
            "shm": ("Motor Shared Memory I/O", "MotorIoBackend + kinematic conversion\nread joint feedback / write targets and torque"),
            "robot": ("Robot Motors and Sensors", "joint q/dq, torque, status"),
            "sim": ("MuJoCo Sim2Sim Path", "mujoco_sim_bridge C++ backend\nbuilds RobotState from qpos/qvel/base/body"),
            "act": ("MuJoCo Actuator Writeback", "policy joints + HOLD joints\nmj_step / hold"),
            "viewer": ("Viewer / Frontend Mirror", "GLFW or Python viewer topics"),
            "telemetry": ("Observable Outputs", "/omnimorph/rl/state\nviewer_frame / inspector\nruntime log / MCAP"),
        },
        "labels": {
            "control": "velocity + mode",
            "mode": "mode contract",
            "obs_contract": "obs contract",
            "model": "model file",
            "features": "reference/sensor features",
            "state_in": "RobotState + inputs",
            "obs_vec": "obs vector",
            "policy": "inference",
            "action": "action",
            "cmd": "control command",
            "feedback": "joint feedback",
            "telemetry": "telemetry/logs",
        },
    }


def render(lang: str, output: Path) -> None:
    cjk = lang == "zh"
    font_path = ZH_FONT if cjk else EN_FONT
    bold_path = ZH_FONT if cjk else EN_BOLD_FONT
    font_index = 2 if cjk else 0
    fonts = {
        "title": font(bold_path, 40, font_index),
        "subtitle": font(font_path, 23, font_index),
        "section": font(bold_path, 26, font_index),
        "node_title": font(bold_path, 23, font_index),
        "node_body": font(font_path, 17, font_index),
        "label": font(font_path, 16, font_index),
        "legend": font(font_path, 16, font_index),
    }
    text = build_text(lang)
    img = Image.new("RGB", (2400, 1600), "#fbfaf7")
    draw = ImageDraw.Draw(img)

    title_w = text_size(draw, text["title"], fonts["title"])[0]
    draw.text(((2400 - title_w) / 2, 34), text["title"], font=fonts["title"], fill="#0f172a")
    subtitle_w = text_size(draw, text["subtitle"], fonts["subtitle"])[0]
    draw.text(((2400 - subtitle_w) / 2, 88), text["subtitle"], font=fonts["subtitle"], fill="#475569")

    left_sec = (55, 150, 560, 1460)
    core_sec = (610, 150, 1530, 1460)
    right_sec = (1580, 150, 2345, 1460)
    draw_section(draw, left_sec, text["sections"][0], fonts, fill="#fff7df", outline="#d7c8a1")
    draw_section(draw, core_sec, text["sections"][1], fonts, fill="#f2edff", outline="#cbbfe8")
    draw_section(draw, right_sec, text["sections"][2], fonts, fill="#edf6ff", outline="#b7c6df")

    nodes = {
        "operator": (90, 235, 525, 335),
        "topics": (90, 370, 525, 485),
        "imu": (90, 520, 525, 620),
        "cfg": (90, 690, 525, 805),
        "manifest": (90, 840, 525, 945),
        "policy": (90, 980, 525, 1070),
        "ref": (90, 1105, 525, 1215),
        "registry": (650, 235, 995, 350),
        "runtime": (1045, 235, 1490, 350),
        "controller": (830, 405, 1305, 520),
        "state_machine": (650, 575, 1000, 690),
        "profile": (1048, 575, 1490, 690),
        "feature": (650, 760, 1045, 885),
        "obs": (1095, 760, 1490, 885),
        "stack": (650, 955, 1045, 1065),
        "strategy": (1095, 955, 1490, 1065),
        "adapter": (650, 1135, 1045, 1235),
        "runner": (1095, 1135, 1490, 1235),
        "command": (875, 1300, 1270, 1415),
        "real": (1620, 250, 2310, 365),
        "shm": (1620, 405, 2310, 520),
        "robot": (1620, 560, 2310, 665),
        "sim": (1620, 780, 2310, 900),
        "act": (1620, 940, 2310, 1045),
        "viewer": (1620, 1085, 2310, 1185),
        "telemetry": (1620, 1270, 2310, 1395),
    }

    node_colors = {
        "operator": "#fffdf6",
        "topics": "#fffdf6",
        "imu": "#fffdf6",
        "cfg": "#eef7ff",
        "manifest": "#eef7ff",
        "policy": "#eef7ff",
        "ref": "#eef7ff",
        "state_machine": "#fffdf4",
        "profile": "#fffdf4",
        "feature": "#ffffff",
        "obs": "#ffffff",
        "stack": "#ffffff",
        "strategy": "#ffffff",
        "adapter": "#ffffff",
        "runner": "#ffffff",
        "command": "#fff7ed",
        "real": "#ffffff",
        "shm": "#ffffff",
        "robot": "#ffffff",
        "sim": "#ffffff",
        "act": "#ffffff",
        "viewer": "#ffffff",
        "telemetry": "#f8fafc",
    }
    for key, rect in nodes.items():
        title, body = text["nodes"][key]
        outline = "#c2410c" if key == "command" else "#334155"
        draw_node(draw, rect, title, body, fonts, cjk=cjk, fill=node_colors.get(key, "#ffffff"), outline=outline)

    blue = "#2563eb"
    purple = "#7c3aed"
    orange = "#ea580c"
    slate = "#475569"

    # Input and config arrows.
    draw_arrow(draw, [node_bottom(nodes["operator"]), node_top(nodes["topics"])], color=orange, label=text["labels"]["control"], label_font=fonts["label"])
    draw_arrow(draw, [node_right(nodes["topics"]), (585, 428), (585, 205), (1268, 205), node_top(nodes["runtime"])], color=orange)
    draw_arrow(draw, [node_right(nodes["imu"]), (575, 570), (575, 205), (1555, 205), (1555, 310), node_left(nodes["real"])], color=blue)

    draw_arrow(draw, [node_right(nodes["cfg"]), (585, 748), (585, 293), node_left(nodes["registry"])], color=blue, label=text["labels"]["mode"], label_font=fonts["label"])
    draw_arrow(draw, [node_right(nodes["manifest"]), (1065, 893), node_left(nodes["obs"])], color=blue, label=text["labels"]["obs_contract"], label_font=fonts["label"])
    draw_arrow(draw, [node_right(nodes["policy"]), (585, 1025), (585, 1185), node_left(nodes["adapter"])], color=purple, label=text["labels"]["model"], label_font=fonts["label"])
    draw_arrow(draw, [node_right(nodes["ref"]), (590, 1160), (590, 820), node_left(nodes["feature"])], color=blue, label=text["labels"]["features"], label_font=fonts["label"])

    # Core flow.
    draw_arrow(draw, [node_right(nodes["registry"]), node_left(nodes["runtime"])], color=blue)
    draw_arrow(draw, [(1225, 350), (1225, 405)], color=blue, label=text["labels"]["state_in"], label_font=fonts["label"])
    draw_arrow(draw, [node_left(nodes["controller"]), node_right(nodes["state_machine"])], color=slate)
    draw_arrow(draw, [node_right(nodes["state_machine"]), node_left(nodes["profile"])], color=slate)
    draw_arrow(draw, [(1269, 690), (1269, 760)], color=blue)
    draw_arrow(draw, [(1030, 520), (850, 760)], color=blue)
    draw_arrow(draw, [node_right(nodes["feature"]), node_left(nodes["obs"])], color=blue)
    draw_arrow(draw, [(1293, 885), (1293, 955)], color=blue, label=text["labels"]["obs_vec"], label_font=fonts["label"])
    draw_arrow(draw, [node_right(nodes["stack"]), node_left(nodes["strategy"])], color=purple)
    draw_arrow(draw, [node_left(nodes["strategy"]), (1045, 1008), node_right(nodes["adapter"])], color=purple)
    draw_arrow(draw, [node_right(nodes["adapter"]), node_left(nodes["runner"])], color=purple, label=text["labels"]["policy"], label_font=fonts["label"])
    draw_arrow(draw, [(1293, 1135), (1293, 1085), node_right(nodes["strategy"])], color=purple, label=text["labels"]["action"], label_font=fonts["label"])
    draw_arrow(draw, [(1293, 1065), (1293, 1328), node_right(nodes["command"])], color=orange)
    draw_arrow(draw, [(875, 1358), (760, 1358), (760, 1008), node_left(nodes["stack"])], color=slate, dash=True)

    # Real and sim execution flow.
    draw_arrow(draw, [node_right(nodes["command"]), (1555, 1358), (1555, 310), node_left(nodes["real"])], color=orange, label=text["labels"]["cmd"], label_font=fonts["label"])
    draw_arrow(draw, [(1965, 365), (1965, 405)], color=orange)
    draw_arrow(draw, [(1965, 520), (1965, 560)], color=orange)
    draw_arrow(draw, [node_left(nodes["robot"]), (1550, 612), (1550, 462), node_left(nodes["shm"])], color=blue, label=text["labels"]["feedback"], label_font=fonts["label"])
    draw_arrow(draw, [node_left(nodes["shm"]), (1550, 462), (1550, 310), node_left(nodes["real"])], color=blue)
    draw_arrow(draw, [node_left(nodes["real"]), (1550, 310), (1550, 295), node_right(nodes["runtime"])], color=blue)

    draw_arrow(draw, [node_right(nodes["command"]), (1555, 1358), (1555, 840), node_left(nodes["sim"])], color=orange)
    draw_arrow(draw, [(1965, 900), (1965, 940)], color=orange)
    draw_arrow(draw, [(1965, 1045), (1965, 1085)], color=slate)
    draw_arrow(draw, [node_left(nodes["sim"]), (1550, 840), (1550, 310), node_left(nodes["runtime"])], color=blue)

    # Observability.
    draw_arrow(draw, [node_right(nodes["controller"]), (1550, 465), (1550, 1328), node_left(nodes["telemetry"])], color=slate, dash=True, label=text["labels"]["telemetry"], label_font=fonts["label"])
    draw_arrow(draw, [node_right(nodes["real"]), (2340, 310), (2340, 1328), node_right(nodes["telemetry"])], color=slate, dash=True)
    draw_arrow(draw, [node_right(nodes["viewer"]), (2340, 1135), (2340, 1328), node_right(nodes["telemetry"])], color=slate, dash=True)

    # Legend.
    legend_x, legend_y = 90, 1500
    for idx, (label, color) in enumerate(zip(text["legend"], [blue, purple, orange, slate])):
        x = legend_x + idx * 520
        if idx == 3:
            draw.line([(x, legend_y + 14), (x + 55, legend_y + 14)], fill=color, width=4)
            for dx in range(0, 55, 18):
                draw.line([(x + dx, legend_y + 14), (x + dx + 9, legend_y + 14)], fill="#fbfaf7", width=5)
        else:
            draw.line([(x, legend_y + 14), (x + 55, legend_y + 14)], fill=color, width=5)
        draw.text((x + 70, legend_y), label, font=fonts["legend"], fill="#334155")

    img.save(output)


def main() -> None:
    render("zh", ROOT / "deploy_architecture_zh.png")
    render("en", ROOT / "deploy_architecture_en.png")


if __name__ == "__main__":
    main()
