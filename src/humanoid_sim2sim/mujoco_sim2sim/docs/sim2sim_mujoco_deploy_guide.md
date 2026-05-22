# MuJoCo Sim2Sim Deploy Guide (Fused Runtime)

For runtime environment pitfalls and verification commands, see:

- [sim2sim_runtime_environment_notes.md](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)

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

## 2. Terminal Startup

Bring up the backend first:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=cpp \
  mode_id:=0 \
  control_hz:=100.0 \
  enable_viewer:=true
```

Then start the policy explicitly:

```bash
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
```

## 3. Launch Arguments

### Required

- `model_path`: MuJoCo XML / MJB path

### Important

- `backend:=cpp`: standard fused runtime backend
- `mode_id`: startup deploy mode id used by the embedded controller runtime
- `pause_when_no_command`: pause stepping when controller output is inactive
- `no_command_behavior`: inactive behavior for actuators
- MuJoCo actuator backend is inferred from the XML model per joint; there is no extra runtime `actuator_control_mode` selector in the strict path
- MuJoCo `<position>` actuators are retuned dynamically from controller semantics: policy joints use active mode-profile `kps/kds/tau_limit`, non-policy or inactive joints use `hold_kp/hold_kd/hold_torque_limit`
- `post_zeroing_hold_settle_ticks`: sim-only settle window after zeroing and before HOLD pose latch
- `enable_state_telemetry`: enable low-frequency asynchronous `/humanoid/rl/state`
- `state_telemetry_hz`: publish rate for asynchronous `/humanoid/rl/state`
- `viewer_inspector_hz`: asynchronous publish rate for Python viewer inspector text

## 4. External Topics Still Used

- `/humanoid/rl/teleop`
- `/humanoid/rl/mode_control`
- `/humanoid/rl/state` (debug / monitoring)
- `/humanoid/rl/runtime_command` (optional test-injection path such as `joint_motor_test`)

The legacy `/humanoid/rl/command` hop is removed. Use `/humanoid/rl/runtime_command`
only for explicit runtime-command injection tools.

## 5. Internal Function Chain

1. `main()` in `mujoco_sim2sim/src/main.cpp`
2. `MujocoSimBridge::MujocoSimBridge()`
3. `loadParameters()`
4. `loadModel()`
5. `resolveModelMappings()`
6. `initializeState()`
7. `IntegratedControllerRuntime::initialize(startup_mode_id_)`
8. `RL_controller::RL_controller_Init(startup_mode_id_)`
9. if no registry was injected, `RL_controller::initModeProfiles()` lazily creates `ModeProfileRegistry::loadFromYaml(...)`
10. `setupRosInterfaces()` creates timer + state publisher
11. `startInputExecutor()` starts dedicated teleop / mode_control ROS input thread
12. `startStateTelemetry()` starts low-frequency asynchronous state telemetry thread
13. `startViewerTelemetry()` starts asynchronous viewer frame / inspector publisher thread
14. each timer tick -> `controlLoopTick()`
15. `buildRobotState()` from MuJoCo `qpos/qvel`
16. sample latest teleop + mode-command caches
17. `IntegratedControllerRuntime::step(...)`
18. `RL_controller::step(...)`
19. `updateControlInput(...)`
20. `mj_step(...)`
21. `updateMirroredState(...)`
22. `updateViewerFrameMirror(...)`
23. `updateViewerInspectorMirror(...)`

That means sim2sim now matches sim2real more closely at the edge-I/O level too:

- input subscriptions are asynchronous
- `/humanoid/rl/state` is telemetry, not a control-path publish
- Python viewer frame / inspector topics are telemetry too, not control-path publishes

Important detail:

- sim2sim and sim2real now share the same controller/runtime logic
- the current difference is only where the mode/profile registry is first created:
- real runtime injects a shared registry from `main()`
- MuJoCo runtime currently lets `RL_controller` lazily create one on initialization

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
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=python_frontend \
  mode_id:=0 \
  enable_viewer:=true
```

Current topology:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

If `enable_viewer:=false` and Python frontend is still enabled, the frontend falls back to inspector mode and listens to:

- `/humanoid/sim2sim/mujoco_viewer_frame`
- `/humanoid/sim2sim/mujoco_viewer_inspector`

So this path keeps the friendly Python viewer while still reusing the same fused control/runtime logic as the C++ sim2sim backend.
Those two topics are now published by a dedicated non-real-time viewer telemetry thread rather than directly from the control tick.

For a complete topic summary across fused and Python frontend paths, see:

- [topic_interface_matrix.md](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/docs/topic_interface_matrix.md)

## 9. JC01 Legs `engineai_walk` Preset

For the JC01 legs-only `engineai_walk` policy with Python GUI frontend, use:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/home/edify/Code/jingchu01/jingchu01_legs.xml \
  backend:=python_frontend \
  mode_id:=0 \
  bridge_config:=/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
  enable_viewer:=true
```

This preset is intentionally narrow:

- 12 leg joints only
- `base_body_name=Body`
- `base_free_joint_name=""` with auto-discovery on `Body`
- `control_hz=500`
- `backend:=python_frontend`

Important:

- the XML must expose a free joint on `Body` if fixed-base zeroing / hold / release features are enabled
- if those safety features are enabled and the model has no base free joint, `mujoco_sim_bridge` now fails fast at startup instead of silently degrading

## 10. Passive Check XML For `simulate`

If you want to inspect passive free-fall and contact behavior directly in MuJoCo `simulate`, use:

- [jc01_fullbody_engineai_walk_passive_check.xml](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/models/jc01_fullbody_engineai_walk_passive_check.xml)

This model keeps the same joints, inertias, sensors, and contact setup as the sim2sim fullbody model, but converts the waist and upper-body actuators from MuJoCo `<position>` actuators to plain `<motor>` actuators.

That means:

- `ctrl=0` no longer implies active position tracking on waist and arms
- loading the XML in `simulate` tests passive body/contact behavior more directly
- this XML is not intended for the current `actuator_control_mode=mixed` sim2sim preset, because the mixed backend expects those joints to be MuJoCo `<position>` actuators
