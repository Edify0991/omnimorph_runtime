# DDS Sim2Real Deploy Guide (Motor SHM + Upper DDS)

## 1. Scope of This Refactor

This refactor applies the following rule:

- Keep shared memory only in motor closed-loop code paths:
  - `RobotSolver::getMotorState()`
  - `RobotSolver::sendMotorCmd()`
- Move all other deploy communication to DDS (ROS2 DDS transport).

## 2. Runtime Architecture

### 2.1 Policy Side (`RL_controller`)

- Main process: `sim2real_rl_controller.cpp`
- Robot I/O backend: `DdsRobotIO`
- Reads:
  - robot state (`/humanoid/rl/state`)
  - teleop command (`/humanoid/rl/teleop`)
  - walk/lifecycle mode (`/humanoid/rl/walk_mode`)
- Writes:
  - policy command (`/humanoid/rl/command`)

### 2.2 Solver Side (`RL_solver`)

- Motor feedback/command:
  - remains shared memory (`target_handle`, `feedback_handle`)
- DDS bridge: `SolverDdsBridge`
  - subscribes policy command (`/humanoid/rl/command`)
  - subscribes IMU (`/imu/yesense`)
  - publishes robot state (`/humanoid/rl/state`)

### 2.3 Joystick Side (`joyLaunch.py`)

- Publishes DDS teleop/walk mode topics directly
- No shared-memory command writer in joystick process

## 3. DDS Topic Contract

### 3.1 `/humanoid/rl/command`

- Type: `std_msgs/msg/Float32MultiArray`
- Direction: `RL_controller` -> `RL_solver`
- Layout:
  - `[q, dq, tau] * 12`
  - `open_rl`
  - `seq`
  - `stamp_sec`

### 3.2 `/humanoid/rl/state`

- Type: `std_msgs/msg/Float32MultiArray`
- Direction: `RL_solver` -> `RL_controller`
- Layout:
  - `[q, dq, tau] * 12`
  - `base_ang_vel(3)`
  - `base_quat(4)` (`x,y,z,w`)
  - `base_rpy(3)`

### 3.3 `/humanoid/rl/teleop`

- Type: `geometry_msgs/msg/Twist`
- Direction: joystick/navigation -> controller
- Mapping:
  - `linear.x -> vx`
  - `linear.y -> vy`
  - `angular.z -> dyaw`

### 3.4 `/humanoid/rl/walk_mode`

- Type: `std_msgs/msg/Int32`
- Direction: joystick/navigation -> controller
- Values:
  - `0/1/2`: WALK/STAND/FIX_STAND
  - `10/11/12/13`: START_POLICY/STOP_POLICY/ZEROING/ESTOP
  - `20/21/22`: START_WALK/START_STAND/START_FIX_STAND

## 4. Code Modules

- DDS protocol encode/decode:
  - `include/rl_master/dds_protocol.h`
  - `dds_protocol.cpp`
- Controller DDS I/O:
  - `include/rl_master/dds_robot_io.h`
  - `dds_robot_io.cpp`
- Solver DDS bridge:
  - `include/rl_master/solver_dds_bridge.h`
  - `solver_dds_bridge.cpp`
- Policy runtime/state machine:
  - `RL_controller.cpp`
  - `deploy_state_machine.*`

## 5. Removed/Reduced Redundancy

- `RobotState` no longer carries shared-memory command/state transport logic.
- `Cmd` is reduced to a lightweight command struct (no shared-memory writer).
- `RL_solver` removes RL command/state shared-memory segments and uses DDS bridge.

## 6. Build Dependencies

`rl_master` now explicitly depends on:

- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `SharedMemory` (still needed by motor closed-loop in `RL_solver`)

## 7. Bringup Sequence

1. Start motor driver stack (unchanged).
2. Start IMU node publishing `/imu/yesense`.
3. Start `RL_solver` (motor loop + DDS bridge).
4. Start `RL_controller` (DDS RobotIO + policy inference).
5. Start `joyLaunch.py` (DDS teleop/mode publisher).

## 8. Migration Notes

- The old `open_rl + seq + timestamp` semantics are preserved.
- Transport changed from shared-memory segments to DDS topics for upper-layer links.
- Motor control determinism remains in shared-memory loop.
