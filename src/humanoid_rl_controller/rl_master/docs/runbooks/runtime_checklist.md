# Runtime Checklist

## 1. Before You Run

### Build

```bash
colcon build --packages-select rl_master mujoco_sim2sim
```

### Validate the target mode

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py --mode-id 0
```

Check that it passes for the exact mode you intend to start.

## 2. Real-Robot Bringup Checklist

1. motor driver is online
2. IMU topic `/imu/yesense` is alive
3. correct `mode_id` exists in `deploy_mode_profiles`
4. ONNX file exists
5. manifest path exists
6. startup script runs:

```bash
./script/sim2real_engineai.sh --mode-id 0
```

Optional auto-start:

```bash
./script/sim2real_engineai.sh --mode-id 0 --auto-start-mode
```

## 3. Sim2Sim Bringup Checklist

1. MuJoCo XML path is correct
2. ONNX and manifest pass precheck
3. fused bridge starts:

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --enable-viewer true
```

4. if needed, auto-start policy:

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```

## 4. Runtime Smoke Checks

### Operator topics

```bash
ros2 topic echo /humanoid/rl/walk_mode --once
ros2 topic echo /humanoid/rl/teleop --once
```

### State output

```bash
ros2 topic echo /humanoid/rl/state --once
```

### Manual mode control

```bash
./script/publish_walk_mode.sh start --mode-id 0
./script/publish_walk_mode.sh stop
./script/publish_walk_mode.sh switch --mode-id 1
```

## 5. If the Policy Does Not Move

Check in this order:

1. mode profile exists for that `mode_id`
2. ONNX loads without I/O mismatch
3. observation manifest matches expected dimension
4. `walk_mode` start word was actually published
5. sim2sim: actuator mapping and joint names are correct
6. sim2real: IMU and motor feedback are alive

## 6. Runtime Reminder

The supported runtime is now single-process in both paths:

- real robot: `RL_solver`
- MuJoCo sim: `mujoco_sim_bridge` with `backend:=cpp`

## 7. Python GUI Compatibility Path

If you specifically want the Python MuJoCo GUI, run:

```bash
./script/sim2sim_engineai_python.sh --model-path /abs/path/to/robot.xml --mode-id 0
```

This now runs the fused C++ runtime with a separate Python MuJoCo viewer frontend.
