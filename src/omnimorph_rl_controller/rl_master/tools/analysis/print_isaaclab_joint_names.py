#!/usr/bin/env python3
"""Print IsaacLab articulation joint order for a URDF/USD asset.

Example:
    ./isaaclab.sh -p ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/tools/analysis/print_isaaclab_joint_names.py \
        --asset /home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/JC01-URDF.urdf \
        --headless
"""

import argparse
import os

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Load one robot asset in IsaacLab and print articulation joint order.")
parser.add_argument("--asset", required=True, help="Path to a robot asset. Supports .urdf and .usd.")
parser.add_argument("--fix-base", action="store_true", help="Load the articulation as fixed-base.")
parser.add_argument(
    "--replace-cylinders-with-capsules",
    action="store_true",
    help="When loading URDF, replace cylinders with capsules during conversion.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


def _make_spawn_cfg(asset_path: str):
    ext = os.path.splitext(asset_path)[1].lower()
    rigid_props = sim_utils.RigidBodyPropertiesCfg(
        disable_gravity=False,
        retain_accelerations=False,
        linear_damping=0.0,
        angular_damping=0.0,
        max_linear_velocity=1000.0,
        max_angular_velocity=1000.0,
        max_depenetration_velocity=1.0,
    )
    articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False,
        solver_position_iteration_count=8,
        solver_velocity_iteration_count=4,
    )

    if ext == ".urdf":
        return sim_utils.UrdfFileCfg(
            asset_path=asset_path,
            fix_base=args_cli.fix_base,
            replace_cylinders_with_capsules=args_cli.replace_cylinders_with_capsules,
            activate_contact_sensors=True,
            rigid_props=rigid_props,
            articulation_props=articulation_props,
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
            ),
        )

    if ext == ".usd":
        return sim_utils.UsdFileCfg(
            usd_path=asset_path,
            activate_contact_sensors=True,
            rigid_props=rigid_props,
            articulation_props=articulation_props,
        )

    raise ValueError(f"Unsupported asset extension '{ext}'. Please provide a .urdf or .usd file.")


ASSET_PATH = os.path.abspath(args_cli.asset)
SPAWN_CFG = _make_spawn_cfg(ASSET_PATH)


@configclass
class JointOrderSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())
    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=500.0,
            texture_file=f"{ISAAC_NUCLEUS_DIR}/Materials/Textures/Skies/PolyHaven/kloofendal_43d_clear_puresky_4k.hdr",
        ),
    )
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/Robot",
        spawn=SPAWN_CFG,
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
            joint_pos={".*": 0.0},
            joint_vel={".*": 0.0},
        ),
        actuators={},
    )


def main():
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt=0.01)
    sim = SimulationContext(sim_cfg)
    scene = InteractiveScene(JointOrderSceneCfg(num_envs=1, env_spacing=2.0))
    sim.reset()

    robot = scene["robot"]
    scene.update(sim.get_physics_dt())

    joint_names = list(robot.data.joint_names)
    default_joint_pos = robot.data.default_joint_pos[0].detach().cpu().tolist()
    default_joint_vel = robot.data.default_joint_vel[0].detach().cpu().tolist()

    print("=" * 80)
    print(f"Asset: {ASSET_PATH}")
    print(f"Joint count: {len(joint_names)}")
    print(f"Joint tensor shape: joint_pos={tuple(robot.data.joint_pos.shape)}, joint_vel={tuple(robot.data.joint_vel.shape)}")
    print("=" * 80)
    for idx, name in enumerate(joint_names):
        pos = default_joint_pos[idx] if idx < len(default_joint_pos) else float("nan")
        vel = default_joint_vel[idx] if idx < len(default_joint_vel) else float("nan")
        print(f"{idx:02d}  {name:<32} default_pos={pos: .6f}  default_vel={vel: .6f}")

    if hasattr(robot, "find_joints"):
        print("=" * 80)
        print("find_joints('.*', preserve_order=True)")
        joint_ids, resolved_names = robot.find_joints(".*", preserve_order=True)
        for idx, (joint_id, name) in enumerate(zip(joint_ids.tolist(), resolved_names)):
            print(f"{idx:02d}  joint_id={joint_id:02d}  {name}")

    # Keep a reference alive until we close the app.
    _ = torch.zeros(1, device=sim.device)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
