# Humanoid Deploy Runtime

This repository now uses a single-process runtime for both real robot deployment and MuJoCo sim2sim.

## Runtime Architecture

### Sim2Real

- `RL_solver` is the only real-time process you need to start.
- Inside `RL_solver`, the following logic is fused into one process:
  - motor shared-memory I/O
  - IMU + teleop + `walk_mode` DDS input
  - deploy state machine
  - observation assembly
  - ONNX policy inference
  - command mapping and motor command writeback

Dataflow:

```text
teleop / walk_mode / imu DDS
          |
          v
     RL_solver
       |- Motor SHM read
       |- IntegratedControllerRuntime
       |    |- RL_controller::step(...)
       |    |- DeployStateMachine
       |    |- ObservationBuilder
       |    |- OnnxPolicyRunner
       |- command apply
       |- Motor SHM write
       |- optional robot_state DDS publish (debug / tools)
```

### Sim2Sim

- `mujoco_sim_bridge` is now the standard sim2sim runtime.
- The C++ bridge embeds the same `IntegratedControllerRuntime` used by `RL_solver`.
- MuJoCo state extraction and actuator writeback are the only environment-specific parts.

Dataflow:

```text
teleop / walk_mode DDS
        |
        v
mujoco_sim_bridge
  |- build RobotStateData from MuJoCo
  |- IntegratedControllerRuntime
  |    |- RL_controller::step(...)
  |- apply target to MuJoCo actuators
  |- publish robot_state DDS (optional debug / tools)
```

## What Still Uses DDS

DDS is still used for cross-process operator inputs and observability:

- `/humanoid/rl/teleop`
- `/humanoid/rl/walk_mode`
- `/humanoid/rl/state`
- `/imu/yesense` on real robot path

What is no longer routed over DDS in the standard deploy path:

- `RL_controller -> RL_solver` internal command transport
- `RL_controller -> mujoco_sim_bridge` internal command transport

## Standard Startup Entry Points

### Real Robot

```bash
./script/sim2real_engineai.sh --mode-id 0
```

### MuJoCo Sim2Sim

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```

## Important Notes

- `script/controller.sh` and the standalone `rl_master/RL_controller` executable are now explicitly legacy compatibility tools.
- `sim2sim_mujoco.launch.py` keeps `start_rl_controller` only for the legacy Python interactive backend.
- The standard sim2sim path is `backend:=cpp`; the Python interactive path remains supported but is still a split two-process topology.

## Docs

- Real-robot deploy guide:
  - `src/humanoid_rl_controller/rl_master/docs/dds_sim2real_deploy_guide.md`
- MuJoCo sim2sim guide:
  - `src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_mujoco_deploy_guide.md`
- EngineAI Gym policy deploy guide:
  - `src/humanoid_rl_controller/rl_master/docs/engineai_gym_policy_deploy_guide.md`
- End-to-end function flow:
  - `src/humanoid_rl_controller/rl_master/docs/runtime_end_to_end_function_flow.md`
- Runtime checklist / runbook:
  - `src/humanoid_rl_controller/rl_master/docs/runbooks/runtime_checklist.md`
