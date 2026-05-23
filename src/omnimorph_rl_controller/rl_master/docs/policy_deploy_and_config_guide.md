# Policy Deploy And Config Guide

This guide explains the active deployment/config workflow for this repository.
It is intentionally policy-family agnostic: AMP, BeyondMimic, locomotion, full
body, and future policy variants all follow the same runtime/config structure.

## 1. Core Objects

The main objects you need to keep straight are:

- `mode_id`: the runtime-switchable deploy mode selected by lifecycle control.
- `config_section`: the static config name for one deployable policy instance.
- profile YAML top-level section: must be exactly the same as `config_section`.
- `tag`: runtime display name used in logs and some policy-group naming paths.
- `ModeProfile`: the in-memory runtime object materialized from one
  `config_section`.

The practical rule is:

- one deployable mode
- one `config_section`
- one profile YAML file
- one top-level YAML section with the same name

## 2. File Layout

The active config layout is:

```text
config/
├── rl_cfg_jc01.yaml
└── profiles/
    ├── <config_section_a>.yaml
    ├── <config_section_b>.yaml
    └── ...
```

### Root `rl_cfg`

The root file owns global/shared contracts such as:

- `robot_global_joint_order`
- `joint_groups`
- `deploy_mode_profiles`
- `config_files`
- runtime/logging defaults

### Per-policy profile file

Each profile file owns exactly one deployable policy instance, including:

- ONNX/model path
- observation/action dimensions
- joint-order contracts
- observation manifest selection
- source contracts
- external observation declarations
- reference motion settings
- robot/default angle settings

## 3. The Name Matching Rules

These names must match exactly:

1. `deploy_mode_profiles[].config_section`
2. `config_files.<config_section>`
3. profile YAML top-level section name

`tag` does not need to match, but keeping it aligned with `config_section` makes
debugging much easier.

## 4. Adding A New Policy

Use this order:

1. Add a new profile file under `config/profiles/`.
2. Make the file contain exactly one top-level section named after the new
   `config_section`.
3. Register `config_section -> profile path` in `config_files`.
4. Register `mode_id -> config_section -> tag` in `deploy_mode_profiles`.
5. Add or select the correct observation manifest.
6. Fill in joint orders, source contracts, robot settings, and policy I/O.
7. Run the config validator before trying sim2sim or sim2real.

## 5. Observation / Joint Contracts

The most important contracts to verify are:

- `robot_global_joint_order`
- `action_joint_order`
- `obs_joint_order`
- `reference_joint_order`
- `observation_manifest_file` or `observation_manifest_path`
- `obs_dim`
- `action_dim`

These define how the runtime maps:

- installed robot joints
- policy action order
- policy observation joint order
- reference motion order

The runtime does support different orders for these spaces, but they must all be
declared explicitly and consistently.

## 6. Observation Manifest

The manifest defines the final observation vector layout sent into the policy.

Typical terms include:

- `joint_pos`
- `joint_vel`
- `last_action`
- `base_ang_vel`
- `projected_gravity`
- `reference_joint_pos`
- `motion_anchor_ori_b`
- `feature`
- `external_sensor`

For observation-term semantics, see:

- [observation_pipeline_diagram.md](./observation_pipeline_diagram.md)
- [deploy_observation_order_contract_guide.md](./deploy_observation_order_contract_guide.md)

## 7. Validation Workflow

Always validate a new or edited mode before runtime testing:

```bash
python3 src/omnimorph_rl_controller/rl_master/tools/analysis/validate_deploy_config.py \
  --rl-cfg src/omnimorph_rl_controller/rl_master/config/rl_cfg_jc01.yaml \
  --mode-id <mode_id> \
  --skip-onnx
```

Use `--config-section <name>` if you want to validate a config entry that is
not currently attached to an active `mode_id`.

## 8. Standard Runtime Commands

### Real robot

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
./script/start_rl_solver.sh \
  --rl-cfg src/omnimorph_rl_controller/rl_master/config/rl_cfg_<robot>.yaml \
  --mode-id <mode_id>
```

Start the selected mode manually:

```bash
ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000 + <mode_id>}"
```

### MuJoCo sim2sim

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/model.xml \
  backend:=cpp \
  mode_id:=<mode_id>
```

## 9. Mode Switching And Hot Switch Constraints

`policy_family` is only the first compatibility gate.

Two modes are candidates for direct hot-switch only if they also keep the
relevant runtime contracts compatible, especially:

- `action_joint_order`
- `installed_joint_run_modes`
- `obs_joint_order`
- `reference_joint_order`
- `observation_manifest_path`
- control-mode assumptions

If these do not match, runtime will not treat the modes as directly
interchangeable.

## 10. Related Docs

- Runtime architecture:
  [policy_runtime_architecture.md](./policy_runtime_architecture.md)
- End-to-end call flow:
  [runtime_end_to_end_function_flow.md](./runtime_end_to_end_function_flow.md)
- Real-robot deploy path:
  [dds_sim2real_deploy_guide.md](./dds_sim2real_deploy_guide.md)
