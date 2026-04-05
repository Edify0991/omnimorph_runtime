# Joint / Motor Test Package Guide

## 1. Overview

`joint_motor_test` is a standalone test package for offline motor/joint verification.

It keeps the existing deployment pipeline:

- command publish: `/humanoid/rl/command`
- state subscribe: `/humanoid/rl/state`
- lifecycle/mode control: `/humanoid/rl/walk_mode`

So it can drive:

- real robot path: `joint_motor_test -> DDS -> RL_solver -> motor SHM`
- sim2sim path: `joint_motor_test -> DDS -> mujoco_sim_bridge`

## 2. Test Modes

Supported motor command streams:

- `csp`: position mode stream (`open_rl=30`)
- `cst`: torque mode stream (`open_rl=40`)
- `r1`: mixed mode stream (`open_rl=50`)

Existing RL deploy modes are unchanged:

- policy: `open_rl=10`
- non-policy command stream (e.g. zeroing): `open_rl=20`
- disabled/hold: `open_rl=0`

## 3. Trajectory Sources

### 3.1 File trajectory

Set in yaml:

- `trajectory_source: file`
- `trajectory_file: /abs/path/to/reference.csv`

Supported row formats:

- `q[12]`
- `time + q[12]`
- `q[12] + dq[12]`
- `time + q[12] + dq[12]`
- `q[12] + dq[12] + tau[12]`
- `time + q[12] + dq[12] + tau[12]`

If `dq` is missing, it is estimated from `q` and configured `control_hz`.
If `tau` is missing and mode is `cst/r1`, fallback PD is used (`fallback_kp/kd`).

Safety checks at load time:

- finite-value check for `q/dq/tau`
- `|q| <= max_abs_q`
- `|dq| <= max_abs_dq`
- `|tau| <= min(tau_limit, max_abs_tau)`

If `strict_safety_checks: true`, any violation aborts startup.

### 3.2 Sine trajectory

Set in yaml:

- `trajectory_source: sine`

Supports:

- all joints move together: `activation_mode: all`
- joints move one by one: `activation_mode: sequential`

Key params:

- `offset / amplitude / period_sec / phase_rad` (scalar or 12-dim vector)
- `sequential_joint_order`
- `sequential_segment_sec`

Export generated sine reference file:

- `sine.export_reference_path: /abs/path/to/generated_sine.csv`

## 4. State Machine Integration

This package uses the same `DeployStateMachine` as deploy framework.

Control words (via `/humanoid/rl/walk_mode`):

- `1000 + mode_id`: switch mode and start
- `2000 + mode_id`: switch mode only
- `10/11/12/13`: start / stop / zeroing / estop
- `3001/3002/3003/3004`: legacy lifecycle words

Recommended flow for this package:

1. set mode: `2000 + test_mode_id`
2. start: `10` or `1000 + test_mode_id`
3. stop hold: `11`
4. zeroing: `12`
5. estop: `13`

## 5. Build

```bash
colcon build --packages-select rl_master mujoco_sim2sim joint_motor_test
```

## 6. Run

### 6.1 Real robot path

```bash
# Terminal 1
ros2 run rl_master RL_solver

# Terminal 2
ros2 launch joint_motor_test joint_motor_test.launch.py \
  config_path:=/abs/path/to/joint_motor_test.yaml
```

### 6.2 Sim2sim path (MuJoCo)

```bash
ros2 launch joint_motor_test joint_motor_test_sim2sim.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=python_interactive \
  test_config_path:=/abs/path/to/joint_motor_test.yaml \
  fixed_base:=true \
  fixed_base_height:=-1.0 \
  actuator_control_mode:=auto \
  pause_when_no_command:=false \
  enable_viewer:=true \
  show_left_ui:=true \
  show_right_ui:=true
```

Viewer controls:

- `backend:=python_interactive`: use official MuJoCo Python viewer UI (left/right panels, camera, perturbation tools).
- `backend:=cpp`: use bridge built-in hotkeys below.

CPP backend hotkeys:

- `Space`: pause/resume
- `Right`: single-step (paused)
- `[` / `]`: speed down/up
- `C`: contact visualization on/off
- `B`: base angular velocity HUD on/off
- `H`: HUD on/off

## 7. Data Logging

When `save_data: true`:

- metadata: `*_joint_motor_test_metadata.json`
- records: `*_joint_motor_test_records.jsonl`

Logged data includes:

- command q/dq/tau
- robot state q/dq/tau
- mode/lifecycle/open_rl
- joint tracking error and RMSE

## 8. Safety Notes

- Always validate `tau_limit` before `cst/r1` tests.
- For sim2sim trajectory checks, enable MuJoCo fixed base (`fixed_base=true`) for clean comparison.
- Use `STOP_POLICY(11)` before stopping processes.
