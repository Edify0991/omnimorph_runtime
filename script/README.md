# Scripts Guide

## Standard Entry Points

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

- `common.sh`: shared helpers
- `run_ros_executable.sh`: generic ROS executable launcher
- `solver.sh`: low-level launcher for `rl_master/RL_solver`
- `controller.sh`: legacy standalone `RL_controller` launcher
- `sim2real_engineai.sh`: recommended one-command real-robot startup
- `sim2sim_engineai.sh`: recommended one-command MuJoCo fused sim2sim startup
- `sim2sim_engineai_python.sh`: recommended Python GUI frontend for fused sim2sim
- `sim2sim_engineai_python_legacy.sh`: legacy split-runtime Python interactive sim2sim startup
- `publish_walk_mode.sh`: publish lifecycle / mode control words
- `imu.sh`: IMU node launcher
- `joyLaunch.py`: joystick bridge publishing teleop + `walk_mode`
- `dds_selfcheck.sh`: DDS topic smoke test for operator-facing topics

## Recommended Runtime Order

### Real Robot

```bash
sudo ./script/driver.sh
sudo ./script/imu.sh
./script/sim2real_engineai.sh --mode-id 0
sudo python3 ./script/joyLaunch.py
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
- `/humanoid/rl/walk_mode` (`std_msgs/msg/Int32`)

Useful helper:

```bash
./script/publish_walk_mode.sh start --mode-id 0
./script/publish_walk_mode.sh switch --mode-id 1
./script/publish_walk_mode.sh stop
```

## Compatibility Notes

- `controller.sh` exists only for compatibility / standalone debugging and is not part of the recommended deploy path anymore.
- The standard sim2sim path no longer requires `start_rl_controller:=true`.

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

It is still much closer to the real fused runtime than the old legacy split backend.

If you explicitly need the historical split topology for regression comparison, use:

```bash
./script/sim2sim_engineai_python_legacy.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```
