# BeyondMimic Sim2Real Adaptation (DDS Transport + SHM Motor Loop)

This project keeps ROS2 build/packaging, migrates upper runtime transport to DDS topics, and retains shared-memory only for the motor target/feedback closed loop.

## What Was Added

- A unified ONNX runner: `OnnxPolicyRunner`
  - Supports named input/output binding.
  - Supports optional `time_step` input (BeyondMimic-style rollout counter).
  - Supports optional multi-output extraction (`extra_output_names`).
  - Supports strict/compat modes for model I/O checks.
- Per-policy observation builder separation
  - Walk and Stand can now use different observation manifests and dimensions.
- Policy-switch reset behavior
  - On mode-profile switch, stacked observation buffer and policy internal step can reset safely.
- Runtime lifecycle state machine
  - `START_POLICY / STOP_POLICY / ZEROING / ESTOP` via `walk_mode` control words.
  - Generic mode switching via `mode_id`, `1000+mode_id`, `2000+mode_id`.
- Multi-model runtime hook
  - Primary policy with optional weighted `sub_models` ensemble.
- Observation feature context
  - Added `reference_motion` and `external_sensor` terms for BeyondMimic-style and multimodal deploy.

## Config Keys

In each policy block (`sim2real`, `stand_sim2real`), use:

```yaml
policy_io:
  obs_input_name: "obs"
  action_output_name: "actions"
  enable_time_step_input: false
  time_step_input_name: "time_step"
  time_step_start: 0
  strict_model_io: false
  reset_policy_on_mode_switch: true
  extra_output_names: []
```

BeyondMimic-like export usually uses:

```yaml
policy_io:
  obs_input_name: "obs"
  action_output_name: "action"
  enable_time_step_input: true
  time_step_input_name: "time_step"
  extra_output_names: ["joint_pos", "joint_vel", "body_pos_w", "body_quat_w"]
```

## Runtime Notes

- DDS topics are used for policy command/state, teleop command, and mode/state command.
- Shared memory is retained only in `RL_solver` motor loop (`sendMotorCmd` / `getMotorState`).
- Controller loop period now follows `RL_control_f` in config instead of fixed 50 Hz.
- Suggested BeyondMimic manifest: `config/observation_manifest_beyondmimic.yaml`.
