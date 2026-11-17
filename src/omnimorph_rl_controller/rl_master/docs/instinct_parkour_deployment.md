# Instinct Lab JC01 Parkour Deployment

This runtime supports the two-graph ONNX export used by `instinct_jc`:

```text
depth_history -> 0-depth_encoder.onnx -> depth latent
term-major 8-frame proprioception + latent -> actor.onnx -> 28 joint actions
```

Use `policy_adapter: instinct_parkour`, `obs_stack_N: 8`,
`observation_stack_layout: term_major`, and
`observation_manifest_instinct_parkour_jc01.yaml`.  The proprioception frame
is 93 values and therefore the actor receives 744 proprioception values before
the encoder latent.  The joint action order must match the exported Isaac Lab
articulation order, rather than the hardware-global order.

Example profile additions (merge these into a complete JC01 full-body profile
containing the gains, limits, default pose, joint orders, and run modes for the
specific robot):

```yaml
policy_adapter: instinct_parkour
policy_file: ${instinct_run_dir}/exported/actor.onnx
policy_family: instinct_parkour
obs_stack_N: 8
observation_stack_layout: term_major
obs_dim: 93
action_dim: 28
motor_N: 28
RL_control_f: 50
observation_manifest_file: observation_manifest_instinct_parkour_jc01.yaml
prefill_observation_history_on_running_start: true

scales:
  lin_vel: 1.0
  ang_vel: 0.25
  dof_pos: 1.0
  dof_vel: 0.05
  command_lin_vel: 1.0
  command_ang_vel: 1.0
  quat: 1.0
  height_measurements: 1.0

instinct_parkour:
  depth_encoder_file: ${instinct_run_dir}/exported/0-depth_encoder.onnx
  # The selected JC01 camera-refine run uses 8 x 36 x 64 after crop.
  depth_input_shape: [1, 8, 36, 64]
  depth_input_name: input
  depth_output_name: output
  depth_feature_name: depth_history

external_observations:
  - name: depth_history
    dim: 18432  # 8 * 36 * 64
    required: true
    topic: /instinct_parkour/depth_history
    message_type: float32_multi_array
```

The input tensor must have the same values as the training camera pipeline:
oldest-to-newest eight frames, crop `(top=36, bottom=0, left=32, right=32)`,
Gaussian blur `(3, sigma=1)`, clipped to `[0, 2.5]` metres and normalized to
`[0, 1]`. These values are confirmed by the selected JC01 `camera_refine`
training configuration: source 128x72, crop to 64x36, eight history frames
with five-frame stride, and a 128-value encoder latent. The repository does
not contain the actual ONNX files, so verify input/output tensor names against
the selected export before enabling real motors.

The two deployment paths use the same feature contract but not the same camera
device:

- **sim2sim:** set `enable_instinct_depth_camera:=true`. The bridge renders
  the named XML camera directly, reproduces the training waist-yaw pose,
  terrain-only visibility, depth conversion, crop, blur, normalization, and
  temporal history, then injects `depth_history` into the controller. No ROS
  camera publisher or physical camera is involved.
- **sim2real:** `RL_solver` already subscribes to the same profile-declared
  `Float32MultiArray` external observation.  The real camera pipeline must
  publish the preprocessed tensor with the same history ordering and timing.

The XML camera `instinct_parkour_depth` is a MuJoCo rendering handle only. The
bridge overrides its pose every camera tick so it follows `waist_yaw` in yaw
only with the training offset; it does not add mass, collision geometry, or a
simulated physical sensor.

Example sim2sim launch (after selecting a root config that registers the
Parkour profile and supplying the two ONNX files):

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  bridge_config:=<jc01-full-body-sim2sim.yaml> \
  rl_cfg_path:=<jc01-instinct-root.yaml> mode_id:=<parkour-mode-id> \
  enable_instinct_depth_camera:=true
```

The existing deployment state machine remains in charge of zeroing, HOLD,
watchdog, and motor modes.  Do not enable a real robot profile until the actor
output order, camera calibration, gains, torque limits, and a sim2sim replay
have all been checked against the exact exported run.
