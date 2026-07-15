# joint_motor_test

Offline joint/motor trajectory test package for OmniMorph deploy pipeline.

See full guide:

- `docs/joint_motor_test_guide.md`

Manual run:

```bash
ros2 launch joint_motor_test joint_motor_test.launch.py
```

BeyondMimic suspended reference check:

```bash
ros2 run joint_motor_test export_beyondmimic_reference.py \
  --onnx ${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/policies/2026-03-29_beyondmimic_jc01_leg12_strict_walk2.onnx \
  --output /tmp/beyondmimic_leg12_reference_500hz.csv \
  --start-step 50
```

Then launch `joint_motor_test_sim2sim.launch.py` with
`config/beyondmimic_leg12_reference_tracking.yaml` and `fixed_base:=true`.

Jingchu01 right-leg acceptance test (suspended/fixed-base first):

```bash
ros2 launch joint_motor_test joint_motor_test.launch.py \
  config_path:=$(ros2 pkg prefix joint_motor_test)/share/joint_motor_test/config/jingchu01_right_leg_acceptance.yaml

ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1011}"
```

This sequence tests `right_hip_pitch`, `right_knee_pitch`,
`right_ankle_pitch`, and `right_ankle_roll`. It uses seventh-order S-curves,
host PD plus CST only on the selected/coupled axes, and CSP pose hold on every
other joint. Mode 11 is external-command-only and fails safe to all-CSP hold
when the runner is missing or stale. Walking mode 2 is unchanged.
