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
6. solver starts cleanly:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 run rl_master RL_solver --ros-args -p startup_mode_id:=0
```

## 3. Sim2Sim Bringup Checklist

1. MuJoCo XML path is correct
2. ONNX and manifest pass precheck
3. fused bridge starts:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=cpp \
  mode_id:=0 \
  enable_viewer:=true
```

4. if needed, start policy manually:

```bash
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
```

## 4. Runtime Smoke Checks

### Operator topics

```bash
ros2 topic echo /humanoid/rl/mode_control --once
ros2 topic echo /humanoid/rl/teleop --once
```

### State output

```bash
ros2 topic echo /humanoid/rl/state --once
```

### Manual mode control

```bash
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 11}"
ros2 topic pub --once /humanoid/rl/mode_control std_msgs/msg/Int32 "{data: 2001}"
```

## 5. If the Policy Does Not Move

Check in this order:

1. mode profile exists for that `mode_id`
2. ONNX loads without I/O mismatch
3. observation manifest matches expected dimension
4. `mode_control` start word was actually published
5. sim2sim: actuator mapping and joint names are correct
6. sim2real: IMU and motor feedback are alive

## 6. Runtime Reminder

The supported runtime is now single-process in both paths:

- real robot: `RL_solver`
- MuJoCo sim: `mujoco_sim_bridge` with `backend:=cpp`

## 7. Python GUI Compatibility Path

If you specifically want the Python MuJoCo GUI, run:

```bash
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=python_frontend \
  mode_id:=0 \
  enable_viewer:=true
```

This now runs the fused C++ runtime with a separate Python MuJoCo viewer frontend.
