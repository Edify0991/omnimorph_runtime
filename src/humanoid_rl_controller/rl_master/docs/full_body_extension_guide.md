# Full-Body Mode Guide

The active runtime no longer assumes a fixed 12-dof transport width.

Current architecture:

- `RobotState` / `RobotStateData` / `RobotCommandData` are dynamic-vector based
- DDS command/state transport uses dynamic `joint_count`
- controller maps policy action indices by joint name
- non-controlled joints stay at the active mode's `default_angle`
- mode registry builds one global joint order across all configured deploy profiles
- if you configure `robot.zero_joint_angles`, it must cover the full runtime joint set; otherwise leave it unset and zeroing will fall back to `default_angle`

This means adding a full-body policy is now mainly a configuration job, not a protocol rewrite.

## What Still Stays Hardware-Specific

One boundary still matters:

- the real robot execution backend only drives the actually installed hardware joints

Today that lower layer is still lower-body specific in the motor / kinematics conversion chain, so:

- controller/runtime can understand a larger full-body joint space
- solver can mirror that larger joint space
- but the real hardware output stage still only executes the installed actuators it knows how to map

So full-body deploy is already supported at the runtime/config layer, while hardware expansion still needs the motor backend to grow with the robot.

## Recommended Config Pattern

For a new full-body mode, keep these fields aligned:

1. `robot.joint_order`
2. `action_joint_order`
3. `obs_joint_order`
4. `action_dim`
5. `motor_N`
6. `observation_manifest_file`
7. `obs_dim`

The easiest safe rule is:

- `robot.joint_order`: full runtime joint layout for this mode
- `action_joint_order`: exactly the ONNX action output order
- `obs_joint_order`: exactly the joint order expected by the observation builder for joint-based terms
- `action_dim == len(action_joint_order)`
- `motor_N == len(obs_joint_order)` for joint-based observation terms

## Example Files

This repository now includes a ready-to-copy example:

- profile section: [`rl_cfg.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg.yaml)
  Search for `engineai_full_body_example`
- observation manifest: [`observation_manifest_full_body_example.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/observation_manifest_full_body_example.yaml)

That example uses:

- 20 active joints
- lower body 12 + upper body 8
- `obs_dim = 71`

Observation dimension there is:

- `2 + 3 + 20 + 20 + 20 + 3 + 3 = 71`

If you change the active joint count from `20` to `M`, the common velloco-style base formula becomes:

- `obs_dim = 2 + 3 + M + M + M + 3 + 3`
- `obs_dim = 11 + 3M`

## Practical Workflow

1. Copy `engineai_full_body_example` to a new config section.
2. Replace `policy_name`, `policy_path` or `policy_file`.
3. Replace upper-body joint names with your robot's real names.
4. Keep `robot.joint_order`, `action_joint_order`, `obs_joint_order` mutually consistent.
5. Update the observation manifest joint-related `count` fields.
6. Recompute `obs_dim`.
7. Add the new section into `deploy_mode_profiles`.
8. Run config validation and sim2sim before real deployment.

## Lower-Body Policy in a Full-Body Robot

For lower-body-only policies on a full-body robot, keep the same global robot joint order, but only list the lower-body joints in `action_joint_order`.

Runtime behavior is then:

- lower-body joints receive policy outputs
- all other joints remain at `default_angle`

That is the intended mechanism for mixed lower-body-control / upper-body-hold deployment.
