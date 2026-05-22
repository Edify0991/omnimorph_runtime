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
