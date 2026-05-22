# MuJoCo Sim Bridge Structure

`MujocoSimBridge` is intentionally split by runtime responsibility. Keep new
code in the file that owns the behavior instead of growing a single monolithic
translation unit.

## Files

- `mujoco_sim_bridge_core.cpp`
  - Node construction and shutdown

- `mujoco_sim_bridge_model.cpp`
  - MuJoCo model loading
  - Controlled/hold joint and actuator mapping
  - Base body/free-joint discovery
  - Position-actuator gain and force-limit tuning

- `mujoco_sim_bridge_parameters.cpp`
  - ROS parameter declaration and loading
  - Name/order normalization
  - Per-joint gain/limit parameter expansion and validation
  - Requires joint order to come from `ModeProfileRegistry::jointOrder()` or
    explicit ROS `joint_names`; no hard-coded joint-name fallback is allowed

- `mujoco_sim_bridge_control_config.cpp`
  - Initial state setup
  - Per-mode control parameter resolution
  - Joint runtime mode parsing

- `mujoco_sim_bridge_base_lock.cpp`
  - Fixed-base and dynamic base-lock helpers
  - Sim-only prepose snap
  - RUNNING-entry reference synchronization

- `mujoco_sim_bridge_control_loop.cpp`
  - Main control-loop tick
  - Deploy lifecycle glue
  - Runtime transition orchestration around base-lock/RUNNING helpers

- `mujoco_sim_bridge_backend_io.cpp`
  - Robot state extraction from MuJoCo `qpos/qvel/cvel`
  - Controller command conversion into MuJoCo `data->ctrl`
  - Torque/position backend behavior and inactive hold behavior

- `mujoco_sim_bridge_ros_io.cpp`
  - ROS publishers, subscriptions, timers, and input executor
  - Teleop and mode-control subscription callbacks

- `mujoco_sim_bridge_telemetry.cpp`
  - State telemetry worker thread
  - Python viewer telemetry worker thread
  - State/viewer mirror updates consumed by publishers

- `mujoco_sim_bridge_logging.cpp`
  - Runtime recorder setup
  - Runtime tick/event/source-sample emission
  - Sim2sim session metadata for MCAP logs

- `mujoco_sim_bridge_publishing_utils.cpp`
  - Robot-state publishing helpers
  - Python viewer frame/inspector message encoding

- `mujoco_sim_bridge_utils.cpp`
  - Shared math and parameter-normalization helpers

- `mujoco_sim_bridge_viewer.cpp`
  - Optional GLFW viewer initialization
  - Viewer event handling
  - Viewer rendering

- `mujoco_sim_bridge_internal.hpp`
  - Internal `MujocoSimBridge` class declaration.
  - Private member state and private method declarations.
  - Grouped by setup, control loop, ROS IO/logging, publishing, viewer, and
    runtime state.

- `mujoco_sim_bridge_internal_helpers.hpp`
  - Internal constants, string parsing helpers, and quaternion helpers used by
    multiple split implementation files.

- `mujoco_sim_bridge_viewer_state.hpp`
  - Optional `ViewerState` definition and GLFW/MuJoCo viewer resource cleanup.

- `include/mujoco_sim2sim/mujoco_sim_bridge.h`
  - Public package-facing factory only: `createMujocoSimBridgeNode()`.
  - External code should not depend on private bridge members.

## Sim2Sim To Sim2Real Flow Map

The sim2sim bridge mirrors the sim2real `RobotSolver` flow, but swaps physical
motor/shared-memory IO for MuJoCo state and control arrays. The shared policy
logic lives in `rl_master::runtime::IntegratedControllerRuntime`.

| Flow Step | Sim2Sim Function | Sim2Real Function | Notes |
| --- | --- | --- | --- |
| Startup object creation | `createMujocoSimBridgeNode()` / `MujocoSimBridge::MujocoSimBridge()` | `RobotSolver::create()` / `RobotSolver::initialize()` | Both create the runtime shell and prepare the active mode/profile. |
| Load robot/backend resources | `loadModel()` | `motor_shm_io_.connect()` in `RobotSolver::initialize()` | Sim2sim loads MuJoCo XML/MJB; sim2real connects to hardware shared memory. |
| Resolve joint layout | `resolveModelMappings()` | `initializeJointLayout()` | Both map configured joint names to backend indices. |
| Allocate runtime buffers | `initializeState()` and mapping vector initialization | `initializeBuffers()` | Sim2sim initializes MuJoCo-side target/cache vectors; sim2real initializes joint/motor command and feedback buffers. |
| Initialize shared controller | `controller_runtime_.initialize()` in constructor | `initializeController()` | Same integrated runtime owns policy, observation, lifecycle, and mode switching. |
| Receive operator commands | `teleopCallback()` / `modeControlCallback()` | `dds_bridge_.readLatestTeleopCommand()` / `readLatestModeControlWord()` in `run()` | Sim2sim consumes ROS topics; sim2real consumes DDS bridge inputs. |
| Read robot state | `buildRobotState()` | `getMotorState()` plus `buildControllerStateData()` | Sim2sim reads MuJoCo `qpos/qvel/cvel`; sim2real reads motor feedback and converts motor space to joint space. |
| Run controller step | `controller_runtime_.step(...)` in `controlLoopTick()` | `controller_runtime_.step(...)` in `RobotSolver::run()` | This is intentionally common between sim2sim and sim2real. |
| Apply mode/lifecycle glue | `prepareModeControlWordForTick()`, base-lock helpers, RUNNING sync helpers | `applyRuntimeCommand()` and hold handling in `RobotSolver::run()` | Sim2sim has extra base-lock/snap behavior because the simulated floating base can be edited directly. |
| Convert controller output to backend command | `updateControlInput()` | `applyRuntimeCommand()` | Both interpret `open_rl` runtime mode and map policy targets/torques into backend commands. |
| Enforce backend limits | `updateControlInput()` clamps by MuJoCo torque backend limits | `sendMotorCmd()` clamps by installed motor torque limits | Sim2sim writes `data_->ctrl`; sim2real writes motor targets. |
| Advance backend | `mj_step()` in `controlLoopTick()` | Physical robot advances continuously after `sendMotorCmd()` | Sim2sim explicitly steps physics; sim2real sends commands to hardware at loop rate. |
| Publish state telemetry | `publishRobotState()`, `startStateTelemetry()` | `sendRLState()` / `dds_bridge_.mirrorRobotState()` | Sim2sim uses ROS publishers; sim2real uses DDS bridge mirroring. |
| Runtime logging | `initRuntimeRecorder()`, `logLoopData()`, `emitDerivedRuntimeEvents()` | Same-named methods in `RobotSolver` | Log payload schemas are shared through `rl_master::logging`. |
| Base IMU source sample | `emitBaseImuSourceSample()` | `emitBaseImuSourceSample()` callback from DDS bridge | Sim2sim derives this from MuJoCo base state; sim2real uses live IMU samples. |
| Visualization/debug UI | `initializeViewer()`, `renderViewerFrame()`, viewer telemetry helpers | No direct equivalent | Sim2sim-only debug path for MuJoCo/interactive viewer. |

## Control-Loop Correspondence

Sim2sim `controlLoopTick()` follows the same high-level order as
`RobotSolver::run()`:

1. Read latest command inputs.
2. Build current robot state.
3. Call `controller_runtime_.step(...)`.
4. Sync/configure runtime-side mode information.
5. Convert `RobotCommandData` into backend command buffers.
6. Apply the backend command.
7. Emit telemetry and logs.

The main difference is that sim2sim owns the physics clock and therefore calls
`mj_step()`, while sim2real only sends commands to hardware and then waits for
the next real control period.

## Rules Of Thumb

- Keep controller semantics in `rl_master`; this package should translate
  between MuJoCo state/control and the integrated controller runtime.
- Put timing and lifecycle behavior in `control_loop`, not in publishing or
  viewer files.
- Put parameter parsing close to the subsystem it configures when possible.
- Keep `mujoco_sim_bridge.h` as the public factory/API. Put private bridge
  declarations in `mujoco_sim_bridge_internal.hpp`.
