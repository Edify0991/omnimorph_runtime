# Runtime End-to-End Function Flow

This document lists the current fused runtime call chain for both real robot and MuJoCo sim2sim.

## 1. Shared Runtime Core

The shared runtime entry is:

- `rl_master::runtime::IntegratedControllerRuntime`

Core methods:

- `initialize(startup_mode_id)`
- `step(state, teleop_sample, mode_command_sample)`

Inside it:

1. create `RL_controller`
2. call `RL_controller::RL_controller_Init()`
3. keep sampled teleop and `walk_mode`
4. compute local `phase_t`
5. call `RL_controller::step(...)`

## 2. RL_controller Internal Flow

`RL_controller::step(state, command, mode_command, phase_t)`:

1. `updateStateFromIO(state)`
2. `updateCommandFromIO(command)`
3. initialize deploy state machine if first tick
4. `deploy_state_machine_.update(mode_command, now_s, robot->joint_q)`
5. `refreshPolicyMode(...)`
6. `handlePolicySwitch()` if needed
7. depending on lifecycle output:
   - `get_robot_observation(...)`
   - `update_obs_deque(...)`
   - `run_policy(...)`
   - `get_joint_target_q(...)`
   - `get_joint_target_torque(...)`
8. assemble `RobotCommandData`
9. `logStepRecord(...)`

## 3. Sim2Real Function Flow

### Entry

- `main()` in `src/humanoid_rl_controller/rl_master/rl_solver.cpp`

### Startup

1. `RobotSolver::create(startup_mode_id)`
2. `RobotSolver::initialize()`
3. `motor_shm_io_.connect()`
4. `dds_bridge_.connect()`
5. `RobotSolver::initializeController()`
6. `IntegratedControllerRuntime::initialize(active_mode_id_)`

### Main loop

Inside `RobotSolver::run()` each cycle:

1. `dds_bridge_.spinOnce()`
2. sample latest `walk_mode`
3. sample latest teleop
4. `getMotorState()`
5. `dds_bridge_.buildRobotStateData(joint_state_, &io_state)`
6. `controller_runtime_.step(io_state, teleop_sample, walk_mode_sample)`
7. `syncRuntimeCfgFromController()`
8. `applyRuntimeCommand(controller_command, true)`
9. if controller active:
   - `dds_bridge_.publishRobotState(io_state)`
   - `sendMotorCmd()`
10. else:
   - latch hold pose if first inactive tick
   - overwrite commands to CSP hold
   - `sendMotorCmd()`
   - `dds_bridge_.publishRobotState(io_state)`

## 4. Sim2Sim Function Flow

### Entry

- `main()` in `src/humanoid_sim2sim/mujoco_sim2sim/src/main.cpp`

### Startup

1. `MujocoSimBridge::MujocoSimBridge()`
2. `loadParameters()`
3. `loadModel()`
4. `resolveModelMappings()`
5. `initializeState()`
6. `controller_runtime_.initialize(startup_mode_id_)`
7. `setupRosInterfaces()`

### Main loop

Inside `MujocoSimBridge::controlLoopTick()` each timer tick:

1. `buildRobotState()` from MuJoCo `qpos`, `qvel`, base state
2. `controller_runtime_.step(state, latest_teleop_command_, mode_command_cache_)`
3. `resolveCommandRuntimeMode(...)`
4. if controller inactive:
   - latch current joint pose into `last_target_q_`
5. `updateControlInput(command, control_active, now)`
6. `mj_step(...)` or hold/forward only
7. `publishRobotState(buildRobotState())`
8. `renderViewerFrame()`

## 5. Environment-Specific Split

### Shared logic

- deploy mode profile selection
- deploy state machine
- observation building
- ONNX inference
- policy switching
- output action interpretation

### Real-only logic

- motor shared-memory read/write
- kinematic conversion joint <-> motor
- IMU DDS sampling

### Sim-only logic

- MuJoCo `qpos/qvel` extraction
- MuJoCo actuator command write
- optional GLFW viewer rendering

## 6. Operator Topics

Shared external inputs:

- `/humanoid/rl/teleop`
- `/humanoid/rl/walk_mode`

Optional observable output:

- `/humanoid/rl/state`

These remain stable across sim2real and sim2sim.

## 7. Legacy Python Interactive Sim2Sim Flow

This path is still available for GUI convenience.

1. launch `rl_master/RL_controller` standalone executable
2. launch `mujoco_sim_interactive_backend.py`
3. Python backend publishes `/humanoid/rl/state`
4. standalone controller reads state + teleop + `walk_mode`
5. standalone controller publishes `/humanoid/rl/command`
6. Python backend subscribes command and applies it to MuJoCo

This is why the Python backend currently remains a split two-process runtime.
