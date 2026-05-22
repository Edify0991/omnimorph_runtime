# Scripts Guide

## Standard Entry Points

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

Use the fused single-process runtime:

```bash
./script/sim2real_engineai.sh --mode-id 0
```

This starts only `RL_solver`, which already embeds:

- `RL_controller`
- deploy state machine
- observation pipeline
- ONNX inference
- solver-side motor SHM I/O

### MuJoCo Sim2Sim

Use the fused C++ bridge runtime:

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```

This starts only `mujoco_sim_bridge` with `backend:=cpp`.

## Script Layout

### Standard entry points

- `sim2real_engineai.sh`: recommended one-command real-robot startup
- `sim2sim_engineai.sh`: recommended one-command MuJoCo fused sim2sim startup
- `sim2sim_engineai_python.sh`: recommended Python GUI frontend for fused sim2sim
- `start_rl_solver.sh`: launcher for `rl_master/RL_solver`
- `start_imu_yesense.sh`: IMU node launcher
- `start_joylaunch.sh`: joystick/operator launcher wrapper
- `joyLaunch.py`: joystick/operator implementation

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

## Recommended Runtime Order

### Real Robot

```bash
sudo ./script/driver.sh
sudo ./script/start_imu_yesense.sh
./script/sim2real_engineai.sh --mode-id 0
sudo ./script/start_joylaunch.sh
```

### Sim2Sim

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --enable-viewer true \
  --auto-start-mode
```

## Mode Control

The fused runtime still listens to the same operator topics:

- `/humanoid/rl/teleop` (`geometry_msgs/msg/Twist`)
- `/humanoid/rl/mode_control` (`std_msgs/msg/Int32`)

Useful helper:

```bash
./script/publish_mode_control.sh start --mode-id 0
./script/publish_mode_control.sh switch --mode-id 1
./script/publish_mode_control.sh stop
```

## Compatibility Notes

- The supported sim2sim paths no longer launch a standalone controller process.

## Python GUI Path

If you want the friendlier Python MuJoCo GUI while still keeping the fused C++ runtime for control and physics, use:

```bash
./script/sim2sim_engineai_python.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```

This path uses:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

It is the only supported Python GUI path now.

## JC01 Legs Example

For the JC01 legs-only `engineai_walk` policy preset, use:

```bash
./script/sim2sim_engineai_python.sh \
  --model-path /home/edify/Code/jingchu01/jingchu01_legs.xml \
  --mode-id 0 \
  --bridge-config /home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
  --auto-start-mode
```

This preset assumes the MuJoCo XML exposes:

- base body `Body`
- a free joint under `Body` (name can be omitted)
- 12 leg joints only

For environment troubleshooting around ROS setup, Conda, FastDDS, ONNX Runtime, and launch commands, see:

- [Sim2Sim Runtime Environment Notes](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)
