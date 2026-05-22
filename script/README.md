# Scripts Guide

## Terminal-First Usage

### New Terminal Setup

After opening a new terminal, initialize the workspace runtime environment with:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
```

This prepares the shell by:

- sourcing ROS2 and this workspace
- exporting `JC01_DEPLOY_ROOT`
- exporting `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` by default
- exporting a writable `ROS_LOG_DIR`
- preferring system/ROS runtime libraries before Conda copies in `LD_LIBRARY_PATH`

### Real Robot

Terminal 1:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
sudo ./script/driver.sh
```

Terminal 2:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
sudo ./script/start_imu_yesense.sh
```

Terminal 3:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 run rl_master RL_solver --ros-args -p startup_mode_id:=0
```

Terminal 4:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
sudo /usr/bin/python3 /home/edify/Code/jc01_deploy/script/joyLaunch.py \
  --workspace /home/edify/Code/jc01_deploy
```

### MuJoCo Sim2Sim

Terminal 1:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=cpp \
  mode_id:=0 \
  control_hz:=100.0 \
  enable_viewer:=true
```

## Script Layout

### Manual runtime building blocks

- `start_rl_solver.sh`: wrapper around `ros2 run rl_master RL_solver`
- `start_imu_yesense.sh`: IMU node launcher
- `start_joylaunch.sh`: joystick/operator helper entry
- `joyLaunch.py`: joystick/operator implementation
- `publish_mode_control.sh`: helper around direct `/humanoid/rl/mode_control` publishing

`joyLaunch.py` runtime mode semantics:

- launching `start_rl_solver.sh` from the hand controller passes
  `--mode-id <primary_mode_id>`
- `primary_mode_id` and `secondary_mode_id` are local joystick-side defaults
- the hand controller publishes lifecycle/mode words to `/humanoid/rl/mode_control`
- it does not automatically inherit mode selection from a separately launched
  manual solver process

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
- `start_humanoid_ops_gui.sh`
- `joy_axis_probe.py`
- `receiver.py`

## Suggested Runtime Order

### Real Robot

See the terminal-by-terminal commands above.

### Sim2Sim

See the terminal-by-terminal commands above.

## Mode Control

The fused runtime still listens to the same operator topics:

- `/humanoid/rl/teleop` (`geometry_msgs/msg/Twist`)
- `/humanoid/rl/mode_control` (`std_msgs/msg/Int32`)

Useful helper:

```bash
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 2001}"
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 11}"
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
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=python_frontend \
  mode_id:=0 \
  control_hz:=100.0 \
  enable_viewer:=true
```

This path uses:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

It is the only supported Python GUI path now.

## JC01 Legs Example

For the JC01 legs-only `engineai_walk` policy preset, use:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/home/edify/Code/jingchu01/jingchu01_legs.xml \
  backend:=python_frontend \
  mode_id:=0 \
  bridge_config:=/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
  enable_viewer:=true
```

This preset assumes the MuJoCo XML exposes:

- base body `Body`
- a free joint under `Body` (name can be omitted)
- 12 leg joints only

For environment troubleshooting around ROS setup, Conda, FastDDS, ONNX Runtime, and launch commands, see:

- [Sim2Sim Runtime Environment Notes](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)
