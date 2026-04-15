# Topic Interface Matrix

This document summarizes the runtime topic interfaces after the fused-runtime refactor.

It separates:

- active topics used by the current fused runtimes
- legacy topics kept only for compatibility
- sim2sim-only viewer topics used by the Python frontend path

## 1. Active Fused-Runtime Topics

These topics are part of the current supported external interface.

| Topic | Direction | Producer | Consumer | Used In | Purpose |
| --- | --- | --- | --- | --- | --- |
| `/humanoid/rl/teleop` | input | joystick / teleop tools | fused `RL_solver`, fused `MujocoSimBridge`, legacy `DdsRobotIO` | sim2real, sim2sim, legacy | desired `vx / vy / yaw_rate` command |
| `/humanoid/rl/walk_mode` | input | operator tools / scripts | fused `RL_solver`, fused `MujocoSimBridge`, legacy `DdsRobotIO` | sim2real, sim2sim, legacy | deploy mode switch + lifecycle control words |
| `/imu/yesense` | input | IMU node | fused `RL_solver` | sim2real | base angular velocity / orientation input |
| `/humanoid/rl/state` | output | fused `RL_solver`, fused `MujocoSimBridge`, legacy sim backend | external monitors / bags / debug tools / legacy controller | sim2real, sim2sim, legacy | low-frequency runtime telemetry / state observability |

Notes:

- In fused runtimes, `/humanoid/rl/state` is asynchronous telemetry only.
- The fused controller path does not read `/humanoid/rl/state` back through DDS.

## 2. Legacy Split-Runtime Topics

These topics belong to the old two-process compatibility path only.

| Topic | Direction | Producer | Consumer | Used In | Purpose |
| --- | --- | --- | --- | --- | --- |
| `/humanoid/rl/command` | output from standalone controller | legacy standalone `RL_controller` / `DdsRobotIO` | legacy `python_interactive` backend or other legacy bridge | legacy only | policy command transport between split controller and split backend |

Notes:

- `/humanoid/rl/command` is no longer part of the active fused runtime.
- In code, it is now explicitly marked as `rl_master::dds::legacy::kTopicPolicyCommand`.

## 3. Sim2Sim Python Frontend Topics

These topics are only used by the fused C++ backend plus Python viewer frontend combination.

| Topic | Direction | Producer | Consumer | Used In | Purpose |
| --- | --- | --- | --- | --- | --- |
| `/humanoid/sim2sim/mujoco_viewer_frame` | output | fused `MujocoSimBridge` | `mujoco_sim_viewer_frontend.py` | sim2sim python_frontend | mirrored MuJoCo frame snapshot for Python viewer |
| `/humanoid/sim2sim/mujoco_viewer_inspector` | output | fused `MujocoSimBridge` | `mujoco_sim_viewer_frontend.py` | sim2sim python_frontend | human-readable runtime inspector summary |

Notes:

- These viewer topics are also asynchronous telemetry.
- They are not part of the control path.
- `viewer_frame` and `viewer_inspector` are sim2sim-only and should not be treated as robot deploy interfaces.

## 4. Runtime Ownership By Path

### 4.1 Sim2Real fused runtime

```text
external teleop/walk_mode/imu topics
  -> RL_solver edge IO
  -> IntegratedControllerRuntime
  -> motor SHM

low-frequency telemetry:
  RL_solver -> /humanoid/rl/state
```

### 4.2 Sim2Sim fused runtime

```text
external teleop/walk_mode topics
  -> MujocoSimBridge edge IO
  -> IntegratedControllerRuntime
  -> MuJoCo actuators

low-frequency telemetry:
  MujocoSimBridge -> /humanoid/rl/state
  MujocoSimBridge -> /humanoid/sim2sim/mujoco_viewer_frame
  MujocoSimBridge -> /humanoid/sim2sim/mujoco_viewer_inspector
```

### 4.3 Legacy split runtime

```text
/humanoid/rl/state
  -> standalone RL_controller
  -> /humanoid/rl/command
  -> legacy simulator/backend
```

## 5. Recommended Mental Model

When reading the codebase, use this split:

- active operator/control interface:
  - `/humanoid/rl/teleop`
  - `/humanoid/rl/walk_mode`
- active observability interface:
  - `/humanoid/rl/state`
- sim2sim Python viewer interface:
  - `/humanoid/sim2sim/mujoco_viewer_frame`
  - `/humanoid/sim2sim/mujoco_viewer_inspector`
- legacy split-runtime interface:
  - `/humanoid/rl/command`

That separation matches the current architecture and helps avoid mixing active and compatibility paths.
