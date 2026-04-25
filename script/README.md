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
- `sim2real_engineai.sh`: recommended one-command real-robot startup
- `sim2sim_engineai.sh`: recommended one-command MuJoCo fused sim2sim startup
- `sim2sim_engineai_python.sh`: recommended Python GUI frontend for fused sim2sim
- `publish_mode_control.sh`: publish lifecycle / mode control words
- `imu.sh`: IMU node launcher
- `joyLaunch.py`: joystick bridge publishing teleop + `mode_control`
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
- free joint `root_free`
- 12 leg joints only
