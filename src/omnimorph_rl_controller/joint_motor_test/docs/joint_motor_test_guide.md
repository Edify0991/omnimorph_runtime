# Joint / Motor Test Package Guide

## 1. Overview

`joint_motor_test` is a standalone test package for offline motor/joint verification.

It now runs with dynamic joint-count trajectories. The active joint layout is resolved in this order:

- `joint_names` from `joint_motor_test.yaml` when explicitly provided
- otherwise the selected `test_mode_id` from `deploy_config_path` / `rl_cfg_jc01.yaml`
- otherwise startup fails instead of silently assuming a legacy 12-dof layout

It keeps the existing deployment pipeline:

- command publish: `/omnimorph/rl/runtime_command`
- state subscribe: `/omnimorph/rl/state`
- lifecycle/mode control: `/omnimorph/rl/mode_control`

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

- `q[N]`
- `time + q[N]`
- `q[N] + dq[N]`
- `time + q[N] + dq[N]`
- `q[N] + dq[N] + tau[N]`
- `time + q[N] + dq[N] + tau[N]`

Here `N` is the resolved active joint count, not a hard-coded 12.

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

- `offset / amplitude / period_sec / phase_rad` (scalar or N-dim vector)
- `sequential_joint_order`
- `sequential_segment_sec`

Export generated sine reference file:

- `sine.export_reference_path: /abs/path/to/generated_sine.csv`

## 4. State Machine Integration

This package uses the same `DeployStateMachine` as deploy framework.

Control words (via `/omnimorph/rl/mode_control`):

- `1000 + mode_id`: switch mode and start
- `2000 + mode_id`: switch mode only
- `10/11/12/13`: start / stop / zeroing / estop

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
./script/start_rl_solver.sh

# Terminal 2
ros2 launch joint_motor_test joint_motor_test.launch.py \
  config_path:=/abs/path/to/joint_motor_test.yaml
```

### 6.2 Sim2sim path (MuJoCo)

```bash
ros2 launch joint_motor_test joint_motor_test_sim2sim.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=python_frontend \
  test_config_path:=/abs/path/to/joint_motor_test.yaml \
  fixed_base:=true \
  fixed_base_height:=-1.0 \
  actuator_control_mode:=auto \
  pause_when_no_command:=false \
  enable_viewer:=true \
  show_left_ui:=true \
  show_right_ui:=true
```

### 6.3 BeyondMimic reference tracking check

This path exports the ONNX `joint_pos/joint_vel` reference and tracks it with
`joint_motor_test` in CST mode. Use `fixed_base:=true` so the robot hangs in
the air and the test isolates reference order, phase, and amplitude from
balance/contact issues.

Export a 500 Hz playback CSV:

```bash
ros2 run joint_motor_test export_beyondmimic_reference.py \
  --onnx ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/policies/2026-03-29_beyondmimic_jc01_leg12_strict_walk2.onnx \
  --output /tmp/beyondmimic_leg12_reference_500hz.csv \
  --start-step 50 \
  --output-hz 500
```

For a pure position-hold check, add `--zero-dq`.

Launch the fixed-base sim2sim test:

```bash
ros2 launch joint_motor_test joint_motor_test_sim2sim.launch.py \
  model_path:=/home/edify/Code/jingchu01/JC01-7DOF-URDF/JC01-URDF-18所/jingchu01_legs.xml \
  bridge_config:=${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/config/jc01_legs_engineai_walk_sim2sim.yaml \
  test_config_path:=${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/joint_motor_test/config/beyondmimic_leg12_reference_tracking.yaml \
  backend:=python_frontend \
  fixed_base:=true \
  fixed_base_height:=0.8780 \
  control_hz:=500.0 \
  actuator_control_mode:=torque \
  enable_viewer:=true
```

Start playback:

```bash
ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1090}"
```

Viewer controls:

- `backend:=python_frontend`: use the supported Python viewer client on top of the fused C++ backend.
- `backend:=cpp`: use the bridge built-in viewer/hotkeys below.

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
- resolved `joint_names` and `joint_count` in metadata

## 8. Safety Notes

- Always validate `tau_limit` before `cst/r1` tests.
- For sim2sim trajectory checks, enable MuJoCo fixed base (`fixed_base=true`) for clean comparison.
- Use `STOP_POLICY(11)` before stopping processes.
