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

- `common.sh`: shared helpers (ROS env, workspace, executable resolving, serial reset)
- `run_ros_executable.sh`: generic ROS2 executable launcher
- `run_motor_test_case.sh`: generic `motor_test` package executable launcher
- `controller.sh`: run `rl_master/RL_controller`
- `solver.sh`: run `rl_master/RL_solver`
- `imu.sh`: run IMU node and publish `/imu/yesense`
- `dds_selfcheck.sh`: DDS deploy self-check (topic/type/connectivity/rate + optional control-word publish)
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
sudo ./imu.sh
sudo ./solver.sh
sudo ./controller.sh
sudo python3 joyLaunch.py
sudo ./dds_selfcheck.sh
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
sudo ./imu.sh --serial-device /dev/ttyACM0
sudo ./imu.sh --no-serial-reset
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
  --max-vx 0.5 --max-vy 0.02 --max-dyaw 0.3 \
  --primary-mode-id 0 --secondary-mode-id 1
```

Key combinations (default):

- `START`: launch `solver.sh`
- `L1 + X`: launch `imu.sh`
- `L1 + A`: launch `controller.sh`
- `L1 + R1`: stop all processes started by `joyLaunch.py`
- `LT + Y`: launch `driver.sh`
- `L1 + DPAD_DOWN`: send `2000 + primary_mode_id` (switch mode only)
- `L1 + B`: send `2000 + secondary_mode_id` (switch mode only)
- `L1 + DPAD_UP`: send `1000 + primary_mode_id` (switch + start policy)
- `L1 + Y`: lifecycle command `STOP_POLICY` (hold current pose / CSP)
- `L1 + LS`: lifecycle command `STOP_POLICY`
- `L1 + RS`: lifecycle command `ZEROING`
- `LT + B`: lifecycle command `ESTOP`
- `L1 + DPAD_RIGHT`: control mode `JOYSTICK`
- `L1 + DPAD_LEFT`: control mode `NAVIGATOR`
- `R1 + (...)`: arm mode combos (reserved, DDS arm channel not enabled in current refactor)

DDS topics used by `joyLaunch.py`:

- `/humanoid/rl/teleop` (`geometry_msgs/msg/Twist`)
- `/humanoid/rl/walk_mode` (`std_msgs/msg/Int32`)

Mode control is now generic in controller:

- `1000 + mode_id`: switch mode and start policy
- `2000 + mode_id`: switch mode only
- `10/11/12/13`: `START_POLICY / STOP_POLICY / ZEROING / ESTOP`
- `3001/3002/3003/3004`: legacy lifecycle words (still accepted for compatibility)

Compatibility note:
- Direct mode switch using plain `mode_id` is intentionally disabled to avoid ambiguity with lifecycle words.
- For any `mode_id` switch, use `2000 + mode_id` (switch only) or `1000 + mode_id` (switch + start).

## `dds_selfcheck.sh` Quick Usage

Read-only checks (recommended default):

```bash
sudo ./dds_selfcheck.sh
```

With explicit publish smoke test:

```bash
sudo ./dds_selfcheck.sh --publish-smoke
```

With explicit state-machine control-word sequence:

```bash
sudo ./dds_selfcheck.sh --publish-sequence "11,12,1000"
```

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

4. IMU serial init abnormal:

```bash
sudo ./imu.sh --no-serial-reset
```

5. `sudo` in `joyLaunch.py` fails silently:

- `joyLaunch.py` uses non-interactive sudo (`sudo -n`) by design.
- Run with root or pre-authorize sudo in current shell (`sudo -v`).

## Structured Data Logs

Structured runtime logs (`*_metadata.json` + `*_records.jsonl`) and analysis flow are documented in:

- `src/humanoid_rl_controller/rl_master/docs/runbooks/runtime_checklist.md`

