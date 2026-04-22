# Topic Interface Matrix

This document summarizes the runtime topic interfaces after the fused-runtime refactor.

It separates:

- active topics used by the current fused runtimes
- sim2sim-only viewer topics used by the Python frontend path

## 1. Active Fused-Runtime Topics

These topics are part of the current supported external interface.

| Topic | Direction | Producer | Consumer | Used In | Purpose |
| --- | --- | --- | --- | --- | --- |
| `/humanoid/rl/teleop` | input | joystick / teleop tools | fused `RL_solver`, fused `MujocoSimBridge` | sim2real, sim2sim | desired `vx / vy / yaw_rate` command |
| `/humanoid/rl/walk_mode` | input | operator tools / scripts | fused `RL_solver`, fused `MujocoSimBridge` | sim2real, sim2sim | deploy mode switch + lifecycle control words |
| `/imu/yesense` | input | IMU node | fused `RL_solver` | sim2real | base angular velocity / orientation input |
| `/humanoid/rl/state` | output | fused `RL_solver`, fused `MujocoSimBridge` | external monitors / bags / debug tools | sim2real, sim2sim | low-frequency runtime telemetry / state observability |

Notes:

- In fused runtimes, `/humanoid/rl/state` is asynchronous telemetry only.
- The fused controller path does not read `/humanoid/rl/state` back through DDS.

## 2. Sim2Sim Python Frontend Topics

These topics are only used by the fused C++ backend plus Python viewer frontend combination.

| Topic | Direction | Producer | Consumer | Used In | Purpose |
| --- | --- | --- | --- | --- | --- |
| `/humanoid/sim2sim/mujoco_viewer_frame` | output | fused `MujocoSimBridge` | `mujoco_sim_viewer_frontend.py` | sim2sim python_frontend | mirrored MuJoCo frame snapshot for Python viewer |
| `/humanoid/sim2sim/mujoco_viewer_inspector` | output | fused `MujocoSimBridge` | `mujoco_sim_viewer_frontend.py` | sim2sim python_frontend | human-readable runtime inspector summary |

Notes:

- These viewer topics are also asynchronous telemetry.
- They are not part of the control path.
- `viewer_frame` and `viewer_inspector` are sim2sim-only and should not be treated as robot deploy interfaces.

## 3. Runtime Ownership By Path

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

## 4. Recommended Mental Model

When reading the codebase, use this split:

- active operator/control interface:
  - `/humanoid/rl/teleop`
  - `/humanoid/rl/walk_mode`
- active observability interface:
  - `/humanoid/rl/state`
- sim2sim Python viewer interface:
  - `/humanoid/sim2sim/mujoco_viewer_frame`
  - `/humanoid/sim2sim/mujoco_viewer_inspector`

That separation matches the current architecture.
