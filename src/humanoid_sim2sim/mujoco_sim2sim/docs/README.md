# MuJoCo Sim2Sim Docs Index

This folder contains the active documentation for the fused MuJoCo sim2sim
runtime centered on `mujoco_sim_bridge`.

## Start Here

- Main deployment guide:
  [sim2sim_mujoco_deploy_guide.md](./sim2sim_mujoco_deploy_guide.md)
- Runtime environment and troubleshooting:
  [sim2sim_runtime_environment_notes.md](./sim2sim_runtime_environment_notes.md)
- Validation workflow:
  [sim2sim_validation_runbook.md](./sim2sim_validation_runbook.md)

## Architecture

- Bridge structure:
  [mujoco_sim_bridge_structure.md](./mujoco_sim_bridge_structure.md)
- Mode-0 tick call flow:
  [amp_sim2sim_mode0_tick_call_flow.md](./amp_sim2sim_mode0_tick_call_flow.md)

## Maintainer Notes

- The standard path is the fused C++ backend.
- The Python frontend/viewer path is still supported, but it is only a viewer
  client layered on top of the fused backend.
