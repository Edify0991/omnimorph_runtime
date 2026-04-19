# BeyondMimic Sim2Real Adaptation (DDS + SHM Motor Loop)

This repository now aligns the reference-motion path with the BeyondMimic-style deploy pattern while keeping your hard boundary:

- Motor closed loop stays on shared memory (`sendMotorCmd` / `getMotorState`).
- Upper-level transport stays on DDS.
- Reference motion can come from file, ONNX extra outputs, or both.

## 1. Added Observation Terms

`ObservationBuilder` now supports these BeyondMimic-related terms:

- `reference_joint_pos`
- `reference_joint_vel`
- `motion_anchor_pos_b` (alias: `motion_ref_pos_b`)
- `motion_anchor_ori_b` (alias: `motion_ref_ori_b`)
- `motion_body_pos_b`
- `motion_body_ori_b`
- `robot_body_pos`
- `robot_body_ori`

Meaning:

- `motion_*`: local-frame features derived from reference body trajectories.
- `robot_body_*`: local-frame robot body features derived from runtime robot body world poses (`robot_body_pos_w`, `robot_body_quat_w`).

## 2. Reference Motion Source Policy

New config keys in `rl_cfg.yaml`:

```yaml
enable_reference_motion: true
reference_motion_source: "auto"      # auto / file / policy_outputs
reference_motion_dim: 24
reference_motion_sampling: "phase"   # phase / step
reference_motion_file: "reference_motion/walk_ref.yaml"
reference_anchor_body: "base"
reference_body_names: ["base", "left_foot", "right_foot"]
```

Behavior:

1. `file`: file only.
2. `policy_outputs`: ONNX extra outputs only.
3. `auto`: load file first, then overwrite by ONNX extra outputs when present.

## 3. Reference Motion File Format (Recommended)

Recommended structured YAML:

```yaml
reference_motion:
  source_format: "beyondmimic_v1"
  anchor_body: "base"
  body_names: ["base", "left_foot", "right_foot"]
  body_quat_format: "wxyz"   # loader converts to internal xyzw
  fps: 50
  frames:
    - joint_pos: [ ... ]
      joint_vel: [ ... ]
      body_pos_w: [x0, y0, z0, x1, y1, z1, ...]
      body_quat_w: [w0, x0, y0, z0, w1, x1, y1, z1, ...]
      # reference_motion is optional; if missing, loader packs joint_pos + joint_vel
```

Legacy compatibility:

- Plain text rows (comma/space separated floats) are still supported as `reference_motion`.

## 4. Safety Checks During Loading

`ReferenceMotionProvider` rejects invalid files with strict checks:

1. Numeric validity: no NaN/Inf.
2. Range checks: bounds on `joint_pos`, `joint_vel`, `body_pos_w`, and generic vectors.
3. Dimension checks:
   - `body_pos_w` must be `3*N`.
   - `body_quat_w` must be `4*N`.
   - Body count must be consistent across frames.
4. Quaternion checks:
   - Zero/invalid quaternion is rejected.
   - Mild non-unit quaternions are normalized with warning.
5. Frame count guard:
   - Internal frame cap avoids loading unbounded files.

## 5. ONNX Output Contract (BeyondMimic Style)

Recommended policy export outputs:

```yaml
policy_io:
  obs_input_name: "obs"
  action_output_name: "action"
  enable_time_step_input: true
  time_step_input_name: "time_step"
  extra_output_names: ["joint_pos", "joint_vel", "body_pos_w", "body_quat_w"]
```

Note:

- `body_quat_w` is treated as `wxyz` (matching common BeyondMimic export), then converted to internal `xyzw`.
- If `policy_io.enable_metadata_check=true`, runtime also validates ONNX custom metadata
  (`required_metadata_keys` + `expected_metadata`).

## 6. Manifest Template

`config/observation_manifest_beyondmimic.yaml` now includes the missing terms as templates with `enabled: false` by default.

When enabling body terms, set explicit dimensions:

- `motion_body_pos_b = 3 * body_count`
- `motion_body_ori_b = 6 * body_count`
- `robot_body_pos = 3 * body_count`
- `robot_body_ori = 6 * body_count`

## 7. Runtime Metadata Extensions

For current runs, the active path is the fused single-file MCAP runtime logger. Equivalent config/profile snapshots now live inside the `.mcap` session file on `runtime/config`.

Current MCAP runtime config snapshot includes the same reference-motion alignment intent as fields such as:

- `reference_motion_source`
- `reference_motion_path`
- `reference_motion_dim_cfg`
- `reference_motion_dim_loaded`
- `reference_motion_frames`
- `reference_source_format`
- `reference_anchor_body_cfg`
- `reference_anchor_body_loaded`
- `reference_body_names_cfg`
- `reference_body_names_loaded`

## 8. AMP Discriminator Compatibility

The runtime now supports optional AMP discriminator inference through `amp_discriminator` config.

Usage intent:

- Keep main policy output unchanged.
- Run discriminator in parallel on `stacked_observation` or `observation`.
- Log discriminator score for online monitoring and post-run analysis.

Main config keys:

```yaml
amp_discriminator:
  enabled: true
  input_source: "stacked_observation"
  policy_file: "policies/amp_discriminator.onnx"
  policy_io:
    obs_input_name: "obs"
    score_output_name: "disc_score"
    extra_output_names: []
    warn_below: -0.2
```

When enabled, controller logs:

- vector: `amp_discriminator_score`
- scalar: `amp_discriminator_score_mean`
