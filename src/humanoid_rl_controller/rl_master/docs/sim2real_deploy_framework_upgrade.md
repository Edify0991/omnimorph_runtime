# Sim2Real Deploy Framework Upgrade (Shared-Memory Transport Preserved)

This upgrade keeps low-level motor IO (`sendMotorCmd` / `getMotorState`) on shared memory, and migrates upper-layer deploy communication to DDS (ROS2 DDS transport).

For full runtime details and topic contracts, see:
`docs/dds_sim2real_deploy_guide.md`.

## What Changed

1. DDS communication layer
   - Controller side: `DdsRobotIO` (`sim2real_rl_controller` -> `RL_controller`).
   - Solver side: `SolverDdsBridge` (`RL_solver` publishes state, subscribes policy command).
   - Shared memory is retained only for motor target/feedback path.

2. Lifecycle state machine (`deploy_state_machine.*`)
   - States: `HOLD`, `ZEROING`, `RUNNING`, `ESTOP`.
   - Supports start/stop/zero/estop commands from DDS `walk_mode` control words.

3. Multi-model policy runtime (`RL_controller`)
   - One primary policy + optional `sub_models` from YAML.
   - Weighted action fusion at runtime.

4. Observation feature context (`ObservationBuilder`)
   - Existing proprioceptive terms unchanged.
   - New terms: `reference_motion`, `external_sensor`, generic `feature`.
   - Enables BeyondMimic/AMP/multimodal style observation assembly via YAML.

5. Reference motion provider (`reference_motion_provider.*`)
   - Loads text/CSV-like frames.
   - Sampling modes: `phase` or `step`.

6. External observation provider (`external_observation_provider.*`)
   - Standardized interface for vision/lidar/etc.
   - Default behavior is zero-filled fallback if upstream sensor feature is absent.

## DDS Topics

- `/humanoid/rl/command` (`std_msgs/msg/Float32MultiArray`)
  - `RL_controller` -> `RL_solver`
  - layout: `[q, dq, tau] * 12 + open_rl + seq + stamp`
- `/humanoid/rl/state` (`std_msgs/msg/Float32MultiArray`)
  - `RL_solver` -> `RL_controller`
  - layout: `[q, dq, tau] * 12 + base_ang_vel(3) + base_quat(4) + base_rpy(3)`
- `/humanoid/rl/teleop` (`geometry_msgs/msg/Twist`)
  - joystick/navigation -> controller
- `/humanoid/rl/walk_mode` (`std_msgs/msg/Int32`)
  - joystick/navigation -> controller lifecycle/mode command

## walk_mode Control Words

- `0`: `WALK`
- `1`: `STAND`
- `2`: `FIX_STAND`
- `10`: `START_POLICY`
- `11`: `STOP_POLICY`
- `12`: `ZEROING`
- `13`: `ESTOP`
- `20`: `START_WALK`
- `21`: `START_STAND`
- `22`: `START_FIX_STAND`

## New Config Highlights (`rl_cfg.yaml`)

- `policy_family`
- `enable_reference_motion`, `reference_motion_dim`, `reference_motion_file`, `reference_motion_sampling`
- `external_observations`
- `sub_models`
- `auto_start_policy`, `zeroing_duration_s`, `zero_pose`

## Suggested Profiles

- AMP: `observation_manifest_amp.yaml`
- BeyondMimic: `observation_manifest_beyondmimic.yaml`

The shared-memory command/state protocol between `RL_controller` and `RL_solver` is unchanged (`open_rl + seq + timestamp` compatible).
The protocol layout (`open_rl + seq + timestamp`) is preserved but carried over DDS topics instead of shared-memory segments.
