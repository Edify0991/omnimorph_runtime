# Scripts Guide

This directory contains runtime scripts for the humanoid RL deployment stack.

## What Was Simplified

The following scripts were previously copy-paste variants and are now unified:

- `initial.sh`
- `joint_test.sh`
- `trajectory_test.sh`
- `receive_test.sh`
- `combined_test.sh`
- `move_zero.sh`

All of them now delegate to:

- `run_motor_test_case.sh`
- `run_ros_executable.sh`
- `common.sh`

This keeps behavior consistent and reduces maintenance cost.

## Script Layout

- `common.sh`: shared helpers (ROS env, workspace, executable resolving, shm cleanup, serial reset)
- `run_ros_executable.sh`: generic ROS2 executable launcher
- `run_motor_test_case.sh`: generic `motor_test` package executable launcher
- `controller.sh`: run `rl_master/RL_controller`
- `solver.sh`: run `rl_master/RL_solver`
- `imu.sh`: run IMU node with optional shared-memory cleanup/startup
- `motor_test.sh`: run jc-driver `motor_test`
- `driver.sh`: interactive `motor_test` bringup automation (`AppStart POS`, `AppM18Enable EN_DRV`)
- `joyLaunch.py`: joystick process orchestration and DDS command writer
- `motor_test_suite.sh`: unified motor test runner (`smoke` / `zero` / `all`)
- `run_joint_pos.sh`: legacy utility script (kept as-is for compatibility)

## Runtime Checklist

1. System dependency checklist:

```bash
sudo apt update
sudo apt install -y expect python3-evdev python3-requests
# ROS2 python runtime (rclpy) is required for DDS command publishing.
```

2. Permission checklist:

```bash
sudo usermod -a -G dialout $USER
# relogin after modifying groups
```

3. Build checklist (from workspace root):

```bash
colcon build --packages-select SharedMemory imu_communication_yesense rl_master motor_test
```

4. Runtime order checklist (recommended):

```bash
cd script
sudo ./driver.sh
sudo ./imu.sh --start-shm-node
sudo ./solver.sh
sudo ./controller.sh
sudo python3 joyLaunch.py
```

5. Optional test executables:

```bash
sudo ./trajectory_test.sh
sudo ./receive_test.sh
sudo ./joint_test.sh
sudo ./combined_test.sh
sudo ./move_zero.sh
sudo ./initial.sh
```

Unified motor test suite:

```bash
sudo ./motor_test_suite.sh smoke
sudo ./motor_test_suite.sh zero
sudo ./motor_test_suite.sh all
```

## `imu.sh` Options

```bash
sudo ./imu.sh --cleanup-shm --serial-device /dev/ttyACM0
sudo ./imu.sh --no-start-shm-node
sudo ./imu.sh -- --ros-args -p some_param:=1
```

## `joyLaunch.py` Quick Usage

```bash
sudo python3 joyLaunch.py
```

Optional custom config:

```bash
sudo python3 joyLaunch.py \
  --receiver-ip 192.168.168.125 \
  --receiver-port 8888 \
  --receiver-client-port 9999 \
  --navi-status-url http://192.168.168.125:10000 \
  --max-vx 0.5 --max-vy 0.02 --max-dyaw 0.3
```

Key combinations (default):

- `START`: launch `solver.sh`
- `L1 + X`: launch `imu.sh`
- `L1 + A`: launch `controller.sh`
- `L1 + R1`: stop all processes started by `joyLaunch.py`
- `LT + Y`: launch `driver.sh`
- `L1 + DPAD_DOWN`: walk mode `WALK`
- `L1 + DPAD_UP`: lifecycle command `START_POLICY`
- `L1 + B`: walk mode `STAND`
- `L1 + Y`: walk mode `FIX_STAND`
- `L1 + LS`: lifecycle command `STOP_POLICY`
- `L1 + RS`: lifecycle command `ZEROING`
- `LT + B`: lifecycle command `ESTOP`
- `L1 + DPAD_RIGHT`: control mode `JOYSTICK`
- `L1 + DPAD_LEFT`: control mode `NAVIGATOR`
- `R1 + (...)`: arm mode combos (reserved, DDS arm channel not enabled in current refactor)

DDS topics used by `joyLaunch.py`:

- `/humanoid/rl/teleop` (`geometry_msgs/msg/Twist`)
- `/humanoid/rl/walk_mode` (`std_msgs/msg/Int32`)

## Troubleshooting

1. ROS environment not found:

```bash
source /opt/ros/humble/setup.bash
```

2. Executable missing:

```bash
colcon build --packages-select <package_name>
```

3. Serial permission denied:

```bash
groups
# ensure dialout is present
```

4. Shared memory stale state:

```bash
sudo ./imu.sh --cleanup-shm
```

5. `sudo` in `joyLaunch.py` fails silently:

- `joyLaunch.py` uses non-interactive sudo (`sudo -n`) by design.
- Run with root or pre-authorize sudo in current shell (`sudo -v`).

