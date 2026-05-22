# Humanoid Deploy Runtime

This repository now uses a single-process runtime for both real robot deployment and MuJoCo sim2sim.

## Runtime Architecture

### Sim2Real

- `RL_solver` is the only real-time process you need to start.
- Inside `RL_solver`, the following logic is fused into one process:
  - motor shared-memory I/O
  - IMU + teleop + `mode_control` DDS input
  - deploy state machine
  - observation assembly
  - ONNX policy inference
  - command mapping and motor command writeback

Dataflow:

```text
teleop / mode_control / imu DDS
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
teleop / mode_control DDS
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
- `/humanoid/rl/mode_control`
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

- The old standalone `rl_master/RL_controller` split-runtime path is no longer a standard entry point.
- The standard sim2sim path is `backend:=cpp`; the supported Python GUI path is `python_frontend`, which acts only as a viewer client on top of the fused backend.

## Docs

- Runtime/controller docs index:
  [src/humanoid_rl_controller/rl_master/docs/README.md](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/docs/README.md)
- MuJoCo sim2sim docs index:
  [src/humanoid_sim2sim/mujoco_sim2sim/docs/README.md](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/README.md)
- Script entry-point guide:
  [script/README.md](/home/edify/Code/jc01_deploy/script/README.md)
