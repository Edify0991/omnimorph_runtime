# MuJoCo Sim2Sim Deploy Guide (Fused Runtime)

## 1. Standard Architecture

The standard sim2sim path is now:

```text
mujoco_sim_bridge (single process)
  |- MuJoCo state read
  |- IntegratedControllerRuntime
  |    |- RL_controller::step(...)
  |- MuJoCo actuator command write
  |- optional DDS state publish
```

This replaces the old standard path:

```text
RL_controller -> DDS -> mujoco_sim_bridge
```

## 2. Standard Startup

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --enable-viewer true \
  --auto-start-mode
```

Equivalent raw launch command:

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=cpp \
  start_rl_controller:=false \
  mode_id:=0 \
  control_hz:=100.0 \
  enable_viewer:=true
```

## 3. Launch Arguments

### Required

- `model_path`: MuJoCo XML / MJB path

### Important

- `backend:=cpp`: standard fused runtime backend
- `mode_id`: startup deploy mode id used by the embedded controller runtime
- `pause_when_no_command`: pause stepping when controller output is inactive
- `no_command_behavior`: inactive behavior for actuators
- `actuator_control_mode`: `auto | torque | position`

### Compatibility argument

- `start_rl_controller`: legacy argument kept only for the old Python interactive backend path

## 4. External Topics Still Used

- `/humanoid/rl/teleop`
- `/humanoid/rl/walk_mode`
- `/humanoid/rl/state` (debug / monitoring)

The bridge does not need `/humanoid/rl/command` anymore in the standard C++ path.

## 5. Internal Function Chain

1. `main()` in `mujoco_sim2sim/src/main.cpp`
2. `MujocoSimBridge::MujocoSimBridge()`
3. `loadParameters()`
4. `loadModel()`
5. `resolveModelMappings()`
6. `initializeState()`
7. `IntegratedControllerRuntime::initialize(startup_mode_id)`
8. ROS subscriptions + timer setup
9. each timer tick -> `controlLoopTick()`
10. `buildRobotState()` from MuJoCo `qpos/qvel`
11. `IntegratedControllerRuntime::step(...)`
12. `RL_controller::step(...)`
13. `updateControlInput(...)`
14. `mj_step(...)`
15. `publishRobotState(...)`

## 6. Inactive Behavior

When the embedded controller is not in policy / command-stream mode:

- controlled joints latch current pose and hold it
- extra non-policy joints listed in `hold_joint_names` hold their configured target
- if `no_command_behavior=zero_torque`, controlled joints output zero torque instead

This matches the intent used on the real-robot path: when policy is not actively running, the robot should stay safely held instead of drifting under a stale torque stream.

## 7. Validation Notes

In local sandbox validation, the fused runtime successfully reached:

- MuJoCo model load
- ONNX policy load
- mode-profile load
- embedded controller initialization
- fused runtime startup

If you see DDS socket errors such as `TRANSPORT_UDP Error` inside a restricted sandbox, that is an environment permission issue rather than a logic issue in the fused runtime.

## 8. Python GUI Frontend

If you need the friendlier Python MuJoCo GUI, use:

```bash
./script/sim2sim_engineai_python.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```

Current topology:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

So this path keeps the friendly Python viewer while still reusing the same fused control/runtime logic as the C++ sim2sim backend.

If you explicitly need the old split runtime for comparison, use:

```bash
./script/sim2sim_engineai_python_legacy.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```
