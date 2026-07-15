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

The acceptance state machine reuses the existing `r1` stream. Its optional
per-joint CST selection keeps selected joints in CST and all others in CSP;
ordinary `r1` commands without that selection retain the legacy all-R1 behavior.

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

### 3.3 Jingchu01 single-joint acceptance trajectory

Use `config/jingchu01_right_leg_acceptance.yaml`. The runner performs, in
order, `right_hip_pitch -> right_knee_pitch -> right_ankle_pitch ->
right_ankle_roll`; hip yaw is intentionally excluded. Every joint begins with
a two-second all-CSP hold, then moves lower/upper/home with two-second dwell
segments.

The dedicated deploy profile is mode 11 (`jingchu01_right_leg_acceptance`). It
sets `external_command_only: true`, so a missing, crashed, or stale test runner
cannot fall back to the bootstrap walking policy. Before the first start
command, the runner captures the current feedback pose as its CSP hold/zeroing
target; it does not move to the configured nominal zero pose.

Each move uses the C3 seventh-order profile:

```text
S(u) = 35u^4 - 84u^5 + 70u^6 - 20u^7
T = max(2.1875*d/vmax, sqrt(7.5132*d/amax), cbrt(52.5*d/jmax))
```

The active joint uses host-side `Kp*(q_target-q)-Kd*dq`, clamped by its
configured torque limit. Desired velocity is deliberately zero; the analytic
trajectory derivatives are logged as reference data only. During an ankle
test both coupled axes enter CST, with the orthogonal axis PD-held at its
captured start angle.

The runner aborts to an all-CSP current-pose hold on stale state, non-finite or
dimension-invalid feedback, configured position violation, or actual speed
above `speed_abort_ratio * max_velocity`.

For the first suspended real-robot pass, reduce `acceptance.pd_gain_scale` and
`acceptance.torque_limit_scale` (for example `0.25` and `0.35`). Both scales
are constrained to `(0, 1]`; restore `1.0/1.0` only for formal acceptance.

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
- acceptance joint/phase, CST mask, theoretical target velocity,
  acceleration and jerk

The solver MCAP must keep `logging.tick.include_motor_io: true`. Its
`motor_state_tau` is the pre-mapping actuator feedback: knee is linear force
in N; rotating joints are torque in Nm. `joint_state_tau` is mapped equivalent
joint torque in Nm.

Generate the acceptance report after a run:

```bash
ros2 run joint_motor_test analyze_jingchu01_acceptance.py \
  --joint-log /path/to/joint_motor_test.jsonl \
  --solver-mcap /path/to/runtime.mcap \
  --config /path/to/jingchu01_right_leg_acceptance.yaml \
  --output-dir /path/to/report
```

The report uses actual `state_dq`, not the reference derivative. Set every
`actuator_mass_kg` in the acceptance config to the measured mass of the
corresponding rotary motor or knee linear actuator, not the whole-robot mass.
Missing/zero mass produces `NOT_EVALUATED` rather than a false pass.

## 8. Safety Notes

- Always validate `tau_limit` before `cst/r1` tests.
- Suspend/secure the real robot and complete mathematical tests, fixed-base
  sim2sim, and a reduced-gain real test before formal high-gain acceptance.
- Stop with control word `11`; estop remains control word `13`.
- For sim2sim trajectory checks, enable MuJoCo fixed base (`fixed_base=true`) for clean comparison.
- Use `STOP_POLICY(11)` before stopping processes.
