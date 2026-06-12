# Scripts Guide

## Terminal-First Usage

### New Terminal Setup

After opening a new terminal, initialize the workspace runtime environment with:

```bash
export ROS_DOMAIN_ID=73  # choose one unique non-zero domain per robot/test setup
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
```

This prepares the shell by:

- sourcing ROS2 and this workspace
- exporting `OMNIMORPH_RUNTIME_ROOT` (and legacy `JC01_DEPLOY_ROOT`)
- exporting `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` by default
- refusing unset `ROS_DOMAIN_ID` and blocking default domain `0`
- scanning the selected domain for already-visible ROS nodes in warning mode
- exporting a writable `ROS_LOG_DIR`
- preferring system/ROS runtime libraries before Conda copies in `LD_LIBRARY_PATH`

`dev_env.sh` does not auto-detect robot asset paths. Export the YAML variables yourself first:

```bash
export OMNIMORPH_RUNTIME_ROOT=/abs/path/to/omnimorph_runtime
export ROS_DOMAIN_ID=73

# JC01
export ROBOT_ASSETS_DIR=/abs/path/to/JC01-URDF-18所

# Unitree G1
export G1_PINOCCHIO_URDF=/abs/path/to/g1_29dof.urdf
export G1_SCENE_XML=/abs/path/to/scene_29dof.xml

# Optional CLI convenience for sim2sim viewer/backend examples
export MUJOCO_MODEL_PATH="${ROBOT_ASSETS_DIR}/scene_jingchu01.xml"
```

When a startup command uses `sudo`, keep those variables with `sudo -E`.

To preflight a domain before starting any control node:

```bash
./script/check_ros_domain.sh --domain "${ROS_DOMAIN_ID}" --strict
```

If a startup script warns that nodes are already visible, verify they are
expected local nodes for this robot. Use another `ROS_DOMAIN_ID` if the domain
may belong to a different machine. The scan can be disabled with
`OMNIMORPH_ROS_DOMAIN_SCAN=off` after manual verification.

### Real Robot

Terminal 1:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
sudo -E ./script/start_driver_jc01.sh
```

For Unitree G1, use the official Unitree low-level runtime/DDS check instead:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic echo lowstate --once
```

Terminal 2:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
sudo -E ./script/start_imu_yesense.sh
```

Terminal 3:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
sudo -E ./script/start_jc01_policy.sh --mode-id 0
# Unitree G1:
sudo -E ./script/start_unitree_g1_policy.sh --mode-id 0
```

Terminal 4:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
sudo -E /usr/bin/python3 ${OMNIMORPH_RUNTIME_ROOT}/script/joyLaunch.py \
  --workspace ${OMNIMORPH_RUNTIME_ROOT}
```

### MuJoCo Sim2Sim

Terminal 1:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args \
  -p model_path:="${MUJOCO_MODEL_PATH}" \
  -p enable_viewer:=true \
  -p viewer_fps:=500.0
```

## Script Layout

### Manual runtime building blocks

- `start_rl_solver.sh`: wrapper around the installed `RL_solver` executable
- `start_jc01_policy.sh`: JC01 sim2real policy runtime entry
- `start_unitree_g1_policy.sh`: Unitree G1 sim2real policy runtime entry
- `start_driver_jc01.sh`: JC01 local driver wrapper
- `start_unitree_g1_bridge.sh`: legacy standalone Unitree G1 bridge wrapper; normal G1 deployment now uses `RL_solver` in-process Unitree motor IO selected by `robot_identity.unitree_transport`
- `start_imu_yesense.sh`: IMU node launcher
- `start_joylaunch.sh`: joystick/operator helper entry
- `joyLaunch.py`: joystick/operator implementation
- `publish_mode_control.sh`: helper around direct `/omnimorph/rl/mode_control` publishing

`joyLaunch.py` runtime mode semantics:

- launching `start_rl_solver.sh` from the hand controller passes
  `--mode-id <primary_mode_id>`
- `primary_mode_id` and `secondary_mode_id` are local joystick-side defaults
- the hand controller publishes lifecycle/mode words to `/omnimorph/rl/mode_control`
- it does not automatically inherit mode selection from a separately launched
  manual solver process

Example direct G1 solver startup:

Select `sdk2` or `ros2` in
`src/omnimorph_rl_controller/rl_master/config/rl_cfg_unitree_g1.yaml`:

```yaml
robot_identity:
  unitree_transport: sdk2  # sdk2 / ros2
```

```bash
source ./script/dev_env.sh
./script/start_unitree_g1_policy.sh --mode-id 0

# Equivalent direct solver command:
./script/start_rl_solver.sh \
  --rl-cfg src/omnimorph_rl_controller/rl_master/config/rl_cfg_unitree_g1.yaml \
  --mode-id 0
```

### Common helpers

- `common.sh`: shared shell helpers
- `run_ros_executable.sh`: generic ROS executable launcher
- `publish_mode_control.sh`: publish lifecycle / mode control words
- `dds_selfcheck.sh`: DDS topic smoke test for operator-facing topics
- `plot_runtime_mcap.py`: runtime MCAP plotting helper

### Hardware / test utilities

- `driver.sh`
- `initial.sh`
- `combined_test.sh`
- `joint_test.sh`
- `motor_test.sh`
- `motor_test_suite.sh`
- `move_zero.sh`
- `run_joint_pos.sh`
- `run_motor_test_case.sh`
- `trajectory_test.sh`
- `receive_test.sh`
- `reference_pose_replay_test.sh`

### Optional sensor / GUI utilities

- `start_realsense_bridge.sh`
- `start_realsense_driver.sh`
- `start_realsense_stack.sh`
- `start_omnimorph_ops_gui.sh`
- `joy_axis_probe.py`
- `receiver.py`

## Suggested Runtime Order

### Real Robot

See the terminal-by-terminal commands above.

### Sim2Sim

See the terminal-by-terminal commands above.

## Mode Control

The fused runtime still listens to the same operator topics:

- `/omnimorph/rl/teleop` (`geometry_msgs/msg/Twist`)
- `/omnimorph/rl/mode_control` (`std_msgs/msg/Int32`)

Useful helper:

```bash
./script/publish_mode_control.sh start --mode-id 0
./script/publish_mode_control.sh switch --mode-id 1
./script/publish_mode_control.sh stop
```

Typical joystick combos:

- `START`: launch solver with `primary_mode_id`
- `L1 + DPAD_DOWN`: publish `2000 + primary_mode_id`
- `L1 + B`: publish `2000 + secondary_mode_id`
- `L1 + DPAD_UP`: publish `1000 + primary_mode_id`
- `L1 + A`: publish `1000 + secondary_mode_id`
- `L1 + Y`: publish `11` (`STOP_POLICY`)
- `L1 + RS`: publish `12` (`ZEROING`)
- `LT + B`: publish `13` (`ESTOP`)

## Compatibility Notes

- The supported sim2sim paths no longer launch a standalone controller process.

## Python GUI Path

If you want the friendlier Python MuJoCo GUI while still keeping the fused C++ runtime for control and physics, use:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
ros2 run mujoco_sim2sim mujoco_sim_viewer_frontend.py --ros-args \
  -p model_path:="${MUJOCO_MODEL_PATH}" \
  -p enable_viewer:=true \
  -p viewer_fps:=500.0
```

This path uses:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

It is the only supported Python GUI path now.

## Native C++ Video Recording

For MP4 output, record from the fused C++ backend instead of the Python GUI:

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=${MUJOCO_MODEL_PATH} \
  backend:=cpp \
  enable_video_recording:=true \
  video_output_name:=sim2sim_check.mp4
```

The default output directory is `/tmp/omnimorph_sim2sim_videos`. Frames are
sampled from MuJoCo simulation time at `video_fps`, so recordings keep real
physics timing even when the GUI is slow or the sim runs faster than wall time.

## JC01 Legs Example

For the JC01 legs-only `engineai_walk` policy preset, use:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=${ROBOT_ASSETS_DIR}/jingchu01_legs.xml \
  backend:=python_frontend \
  mode_id:=0 \
  bridge_config:=${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
  enable_viewer:=true
```

This preset assumes the MuJoCo XML exposes:

- base body `Body`
- a free joint under `Body` (name can be omitted)
- 12 leg joints only

For environment troubleshooting around ROS setup, Conda, FastDDS, ONNX Runtime, and launch commands, see:

- [Sim2Sim Runtime Environment Notes](../src/omnimorph_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)
