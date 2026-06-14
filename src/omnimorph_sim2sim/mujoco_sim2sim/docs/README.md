# MuJoCo Sim2Sim Docs Index

This folder contains the active documentation for the fused MuJoCo sim2sim
runtime centered on `mujoco_sim_bridge`.

## Core Guides

- Main deployment guide:
  [sim2sim_mujoco_deploy_guide.md](./sim2sim_mujoco_deploy_guide.md)
- Runtime environment and troubleshooting:
  [sim2sim_runtime_environment_notes.md](./sim2sim_runtime_environment_notes.md)
- Validation workflow:
  [sim2sim_validation_runbook.md](./sim2sim_validation_runbook.md)

## Manual Velocity Commands

The fused MuJoCo bridge listens to the same teleop topic as sim2real:

```bash
ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
ros2 topic pub -r 20 /omnimorph/rl/teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
ros2 topic pub -r 20 /omnimorph/rl/teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"
```

## Architecture

- Bridge structure:
  [mujoco_sim_bridge_structure.md](./mujoco_sim_bridge_structure.md)
- Mode-0 tick call flow:
  [amp_sim2sim_mode0_tick_call_flow.md](./amp_sim2sim_mode0_tick_call_flow.md)

## Visualization Tools

- Offline MCAP replay with MuJoCo plus Pinocchio COM/support overlay:
  `src/omnimorph_sim2sim/mujoco_sim2sim/scripts/replay_mcap_com_support.py`
- Live native viewer overlay:
  launch with `enable_viewer:=true enable_com_support_visualization:=true`.

## Maintainer Notes

- The standard path is the fused C++ backend.
- The Python frontend/viewer path is still supported, but it is only a viewer
  client layered on top of the fused backend.
