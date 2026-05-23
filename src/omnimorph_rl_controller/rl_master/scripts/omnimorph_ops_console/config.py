from __future__ import annotations

import os
from pathlib import Path
from typing import List

import yaml

from .models import GuiConfig, ModeProfile


def find_default_config() -> Path:
    candidates: List[Path] = [
        Path.cwd() / "src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml",
        Path.cwd() / "config/rl_cfg_jc01.yaml",
    ]

    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "share/rl_master/config/rl_cfg_jc01.yaml")

    for path in candidates:
        if path.is_file():
            return path
    return candidates[0]


def load_gui_config(path: Path) -> GuiConfig:
    if not path.is_file():
        return GuiConfig(path=path, joint_names=[], profiles=[])

    with path.open("r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}

    joint_names = [str(v) for v in cfg.get("robot_global_joint_order", [])]
    profiles: List[ModeProfile] = []
    for entry in cfg.get("deploy_mode_profiles", []) or []:
        try:
            mode_id = int(entry.get("mode_id", 0))
        except Exception:
            continue
        profiles.append(
            ModeProfile(
                mode_id=mode_id,
                tag=str(entry.get("tag", f"mode_{mode_id}")),
                config_section=str(entry.get("config_section", "")),
            )
        )

    profiles.sort(key=lambda item: item.mode_id)
    return GuiConfig(path=path, joint_names=joint_names, profiles=profiles)
