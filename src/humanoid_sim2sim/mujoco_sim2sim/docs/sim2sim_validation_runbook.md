# MuJoCo Sim2Sim Validation Runbook

This runbook collects copy-paste terminal flows for validating MuJoCo sim2sim with the fused `mujoco_sim_bridge` runtime.

Current target flow:

- policy: `2026-04-09_15-00-05v2.onnx`
- profile: `engineai_walk`
- bridge preset: `jc01_fullbody_engineai_walk_sim2sim.yaml`
- model: `jc01_fullbody_engineai_walk_sim2sim.xml`

## 1. Shared Shell Setup

Run this at the top of every terminal before any ROS2 command:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog
```

Recommended:

- do not run from an active Conda shell
- keep all terminals on the same `RMW_IMPLEMENTATION`

## 2. Expected Startup Semantics

With the current sim-only safety configuration:

- cold start enters fixed-base `ZEROING`
- zeroing completes into `HOLD`
- explicit start releases the base and enters `RUNNING`

So seeing `HOLD` after startup is expected. It does not mean the backend failed.

## 3. Current Fullbody Control Semantics

For the current `jc01_fullbody_engineai_walk_sim2sim.yaml` preset:

- lower-body 12 joints are the policy-controlled joints for `engineai_walk`
- the remaining 16 joints are still present in MuJoCo, but are not driven by the walk policy
- those 16 non-policy joints are held by the sim bridge using `hold_joint_names`
- those 16 non-policy joints also use MuJoCo `<position>` actuators in the sim-only XML
- the hold target comes from `hold_target_source: zero_joint_angles`, so they are pulled toward `robot.zero_joint_angles` from the active profile
- the policy-controlled joints still resolve their runtime mode from profile `installed_joint_run_modes`, unless a sim-only `joint_runtime_mode_overrides` entry overrides one joint

This is the first-stage sim2sim alignment toward sim2real semantics:

- policy joints follow the mode profile contract
- non-policy joints are not left floating in sim
- non-policy upper-body and waist joints are on a MuJoCo position backend, not a fake torque-only hold path
- no XML-side fake joints are introduced
- no `rl_master` profile semantics were widened just for sim2sim

## 4. Headless Validation

### Terminal 1: Start the fused sim2sim backend

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog

ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file /home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_fullbody_engineai_walk_sim2sim.yaml \
  -p startup_mode_id:=0 \
  -p enable_viewer:=false
```

Expected early logs include:

- `Loaded MuJoCo model`
- `Actuator control mode: mixed (... resolved_torque_joints=12, resolved_position_joints=16 ... hold_target_source=zero_joint_angles)`
- `ONNX policy loaded`
- `MuJoCo sim2sim fused runtime ready`
- `lifecycle -> ZEROING`
- `lifecycle -> HOLD`

### Terminal 2: Start mode 0 and enter running

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
```

Expected logs in Terminal 1:

- `Dynamic base lock released: pre-running release`
- `lifecycle -> RUNNING`

After that, the terminal may become quiet. That is normal. The runtime does not print per-tick policy outputs by default.
Also, the warning about `inactive behavior hold_position ... fallback to torque PD hold-last` is expected before `RUNNING` when the backend is still inactive and actuators are torque-driven.

### Terminal 3: Publish teleop commands

Zero teleop:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 20 /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Forward command:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 20 /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Yaw command:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 20 /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### Terminal 4: Monitor state telemetry

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic hz /humanoid/rl/state
```

Optional one-shot raw message:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic echo /humanoid/rl/state --once
```

Headless validation is considered successful when:

- Terminal 1 reaches `RUNNING`
- Terminal 3 teleop publish succeeds
- Terminal 4 shows `/humanoid/rl/state` publishing steadily

## 5. Visual Validation with Python GUI

This path keeps the C++ fused backend for physics and control, and uses the Python viewer only as a frontend.

### Terminal 1: Start backend with Python viewer telemetry enabled

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/roslog
mkdir -p /tmp/roslog

ros2 run mujoco_sim2sim mujoco_sim_bridge --ros-args \
  --params-file /home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_fullbody_engineai_walk_sim2sim.yaml \
  -p startup_mode_id:=0 \
  -p enable_viewer:=false \
  -p enable_python_viewer_stream:=true \
  -p enable_python_viewer_inspector:=true \
  -p fixed_base_height:=0.4
```

### Terminal 2: Start the Python viewer frontend

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args \
  -p model_path:="/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/models/jc01_fullbody_engineai_walk_sim2sim.xml" \
  -p enable_viewer:=true \
  -p viewer_fps:=60.0
```

Expected:

- a MuJoCo viewer window opens
- before `RUNNING`, the robot may remain in fixed-base zeroing or fixed-base hold

### Terminal 3: Enter running

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
```

Expected in Terminal 1:

- `Dynamic base lock released: pre-running release`
- `lifecycle -> RUNNING`

Expected in the viewer:

- robot transitions from air-locked preparation into free-base physical simulation

### Terminal 4: Publish teleop commands

Zero teleop:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 20 /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Forward command:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 20 /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Yaw command:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub -r 20 /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Visual validation is considered successful when:

- the viewer opens and updates continuously
- Terminal 1 reaches `RUNNING`
- the robot visibly responds to Terminal 4 teleop commands

## 6. Common Control Commands

Stop policy and return to hold:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 11}"
```

Trigger zeroing:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 12}"
```

Emergency stop:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 13}"
```

Switch to another mode `N` without immediate running:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 2000 + N}"
```

Example for mode `1`:

```bash
source /opt/ros/humble/setup.bash
source /home/edify/Code/jc01_deploy/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 2001}"
```

## 6. Quick Success Checklist

Minimum signs that the sim2sim pipeline is healthy:

- startup prints `Loaded MuJoCo model`
- startup prints `ONNX policy loaded`
- startup prints `MuJoCo sim2sim fused runtime ready`
- explicit start prints `lifecycle -> RUNNING`
- teleop publish succeeds
- `/humanoid/rl/state` publishes steadily or the viewer shows motion updates
