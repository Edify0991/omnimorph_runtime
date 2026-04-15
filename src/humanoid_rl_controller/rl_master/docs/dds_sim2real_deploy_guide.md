# Sim2Real Deploy Guide (Single-Process Runtime)

## 1. What Changed

The standard real-robot deploy path is no longer:

```text
RL_controller -> DDS -> RL_solver -> motor SHM
```

It is now:

```text
RL_solver (single process)
  |- read motor state via SHM
  |- read IMU / teleop / walk_mode via DDS
  |- run RL_controller::step(...)
  |- write motor command via SHM
```

DDS is still used for operator input and observability, but not for internal controller-to-solver command transport.

## 2. Standard Startup

### 2.1 One-command startup

```bash
./script/sim2real_engineai.sh --mode-id 0
```

Optional auto-start after bringup:

```bash
./script/sim2real_engineai.sh --mode-id 0 --auto-start-mode
```

### 2.2 Manual startup order

```bash
sudo ./script/driver.sh
sudo ./script/imu.sh
./script/solver.sh --mode-id 0
sudo python3 ./script/joyLaunch.py
```

Notes:

- `solver.sh` now launches the standard fused runtime.
- `controller.sh` is no longer required for standard deployment.

## 3. External Topics Still Used

### 3.1 Inputs

- `/humanoid/rl/teleop` (`geometry_msgs/msg/Twist`)
- `/humanoid/rl/walk_mode` (`std_msgs/msg/Int32`)
- `/imu/yesense` (`sensor_msgs/msg/Imu`)

### 3.2 Optional debug output

- `/humanoid/rl/state` (`std_msgs/msg/Float32MultiArray`)

This topic is still published so external tools can inspect state, but the controller no longer depends on reading it back through DDS in the standard path.
It is now asynchronous low-frequency telemetry rather than a real-time control-path message.

## 4. Mode / Lifecycle Control Words

Supported control words are unchanged:

- `1000 + mode_id`: switch mode and start policy
- `2000 + mode_id`: switch mode only
- `10`: `START_POLICY`
- `11`: `STOP_POLICY`
- `12`: `ZEROING`
- `13`: `ESTOP`

Helper examples:

```bash
./script/publish_walk_mode.sh start --mode-id 0
./script/publish_walk_mode.sh switch --mode-id 1
./script/publish_walk_mode.sh stop
```

## 5. Internal Function Chain

Real-robot runtime path:

1. `main()` in `rl_solver.cpp`
2. `ModeProfileRegistry::loadFromYaml(RL_CFG_PATH, "engineai_walk")`
3. `mode_registry->specForMode(startup_mode_id, true)`
4. `mode_registry->cfgForMode(startup_mode_id, true)`
5. configure realtime from the selected startup cfg
6. `RobotSolver::create(startup_mode_id, mode_registry)`
7. `RobotSolver::switchToModeConfig(startup_mode_id, true)`
8. `RobotSolver::initialize()`
9. `RobotSolver::initializeController()`
10. `IntegratedControllerRuntime::setModeProfileRegistry(mode_registry_)`
11. `IntegratedControllerRuntime::initialize(startup_mode_id)`
12. `RL_controller::RL_controller_Init(startup_mode_id)`
13. `RL_controller::initModeProfiles()`
14. `SolverDdsBridge` starts dedicated ROS input executor thread
15. `SolverDdsBridge` starts asynchronous low-frequency state telemetry thread
16. `RobotSolver::run()` loop
17. `motor_shm_io_.readFeedback(...)`
18. sample cached teleop / walk_mode / imu
19. `dds_bridge_.buildRobotStateData(...)`
20. `IntegratedControllerRuntime::step(...)`
21. `RL_controller::step(...)`
22. `RobotSolver::applyRuntimeCommand(...)`
23. `dds_bridge_.mirrorRobotState(...)`
24. `sendMotorCmd()`

Important details:

- `solver` and `controller` now share the same cached mode/profile registry
- runtime mode switching uses already-built in-memory profiles
- switching mode no longer depends on both sides separately re-reading `rl_cfg.yaml`
- ROS input callbacks and `/humanoid/rl/state` publish are kept off the control loop thread

## 6. Why This Is Better

Compared with the old two-process runtime, this path:

- removes one internal DDS hop on the critical control path
- keeps the same deploy state machine and observation logic
- keeps the same operator topics and tooling
- makes sim2real behavior closer to fused sim2sim behavior

## 7. Debugging Tips

### 7.1 Verify mode input

```bash
ros2 topic echo /humanoid/rl/walk_mode --once
```

### 7.2 Verify teleop input

```bash
ros2 topic echo /humanoid/rl/teleop --once
```

### 7.3 Verify solver-side state publish

```bash
ros2 topic echo /humanoid/rl/state --once
```

### 7.4 If policy does not start

Check these in order:

1. `walk_mode` control word was actually published
2. selected `mode_id` exists in `deploy_mode_profiles`
3. deploy precheck passes for that mode
4. IMU topic is alive on real robot path

## 8. Compatibility

The standalone `RL_controller` executable is still available for compatibility and isolated debugging, but it is no longer the recommended production startup method.

The old `/humanoid/rl/command` topic now belongs only to that legacy split-runtime path.
