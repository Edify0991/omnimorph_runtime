# EngineAI Gym Policy Deploy Guide (Sim2Sim + Sim2Real)

This guide explains how to deploy a policy trained in an EngineAI Gym style RL stack into this repository for both:

- MuJoCo sim2sim (`RL_controller -> DDS -> mujoco_sim_bridge`)
- real robot sim2real (`RL_controller -> DDS -> RL_solver -> motor SHM`)

It also includes the new preflight validator and ONNX metadata checks.

## 1. Prerequisites

From workspace root:

```bash
colcon build --symlink-install --packages-up-to rl_master mujoco_sim2sim
source install/setup.bash
```

Runtime Python dependencies:

```bash
python3 -m pip install pyyaml onnxruntime
```

Optional for editing ONNX metadata:

```bash
python3 -m pip install onnx
```

## 2. Export Policy From EngineAI Gym

When exporting ONNX, record these items from training/deploy code:

- observation dimension after stacking: `obs_dim * obs_stack_N`
- single-frame `obs_dim`
- `obs_stack_N`
- action dimension
- observation term order
- action joint order
- ONNX input/output tensor names

Recommended ONNX metadata keys (custom metadata map):

- `obs_dim`
- `action_dim`
- `obs_stack_n`
- `obs_input_name`
- `action_output_name`
- `policy_family`
- `obs_joint_order`
- `action_joint_order`

Example metadata patch script:

```python
import onnx

model = onnx.load("policy.onnx")
props = {
    "obs_dim": "47",
    "action_dim": "12",
    "obs_stack_n": "15",
    "obs_input_name": "obs",
    "action_output_name": "actions",
    "policy_family": "amp",
    "obs_joint_order": "right_hip_roll,right_hip_yaw,right_hip_pitch,right_knee_pitch,right_ankle_pitch,right_ankle_roll,left_hip_roll,left_hip_yaw,left_hip_pitch,left_knee_pitch,left_ankle_pitch,left_ankle_roll",
    "action_joint_order": "right_hip_roll,right_hip_yaw,right_hip_pitch,right_knee_pitch,right_ankle_pitch,right_ankle_roll,left_hip_roll,left_hip_yaw,left_hip_pitch,left_knee_pitch,left_ankle_pitch,left_ankle_roll",
}
del model.metadata_props[:]
for k, v in props.items():
    p = model.metadata_props.add()
    p.key = k
    p.value = v
onnx.save(model, "policy_with_metadata.onnx")
```

## 3. Add A New Deploy Profile

In `config/rl_cfg.yaml`:

1. Add a section for your new policy (for example `engineai_walk`).
2. Add a `deploy_mode_profiles` entry mapping a new `mode_id` to that section.
3. Set:
   - `policy_path` (or `policy_file`)
   - `obs_dim`, `obs_stack_N`, `action_dim`, `motor_N`
   - `policy_io.obs_input_name`, `policy_io.action_output_name`
   - `action_joint_order`, `obs_joint_order`
   - `observation_manifest_file` or `observation_manifest_path`

Recommended metadata-check block in `policy_io`:

```yaml
policy_io:
  obs_input_name: "obs"
  action_output_name: "actions"
  enable_time_step_input: false
  strict_model_io: true
  enable_metadata_check: true
  metadata_check_strict: true
  required_metadata_keys: ["obs_dim", "action_dim", "obs_stack_n"]
  expected_metadata:
    obs_dim: "47"
    action_dim: "12"
    obs_stack_n: "15"
    obs_input_name: "obs"
    action_output_name: "actions"
```

## 4. Add/Select Observation Manifest

If your policy uses a new observation layout:

1. Create a new manifest file under `config/`, for example:
   - `observation_manifest_engineai_walk.yaml`
2. Set `observation_manifest_file` (or `observation_manifest_path`) in your profile.
3. Ensure the manifest total dimension equals `obs_dim`.

Notes:

- Term order in manifest = observation concatenation order.
- `command.components` order = command feature order.
- `joint_pos/joint_vel` are reordered by `obs_joint_order`.

## 5. Run Offline Preflight Validation

Before launching anything, run:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py
```

Validate a specific mode only:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py --mode-id 2
```

YAML/manifest only (skip ONNX load):

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py --skip-onnx
```

The validator checks:

- mode profile mapping
- profile dimensions and joint orders
- manifest legality and resolved dimension
- ONNX IO contract (`strict_model_io` semantics)
- unknown model inputs (zero-filled vs strict error)
- action output static shape sanity
- ONNX metadata checks (when enabled)

## 6. Sim2Sim Deployment Flow

Launch MuJoCo bridge + controller:

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  start_rl_controller:=true \
  control_hz:=100.0 \
  fixed_base:=false
```

If policy does not auto-start, publish control word:

```bash
ros2 topic pub --once /humanoid/rl/walk_mode std_msgs/msg/Int32 "{data: 1002}"
```

Above example means `mode_id=2` and `START_POLICY` (`1000 + mode_id`).

Optional command input:

```bash
ros2 topic pub --once /humanoid/rl/teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Runtime checks:

```bash
ros2 topic echo /humanoid/rl/state --once
ros2 topic echo /humanoid/rl/command --once
```

## 7. Sim2Real Deployment Flow

Recommended order:

```bash
cd script
sudo ./driver.sh
sudo ./imu.sh
sudo ./solver.sh
sudo ./controller.sh
sudo python3 joyLaunch.py
sudo ./dds_selfcheck.sh --publish-smoke
```

Mode switch/start examples:

- switch and start mode 2: control word `1002`
- switch mode only mode 2: control word `2002`
- lifecycle only:
  - `10`: START_POLICY
  - `11`: STOP_POLICY (hold)
  - `12`: ZEROING
  - `13`: ESTOP

Safety recommendation:

- First run with conservative `action_scale`, `kps`, `kds`, `tau_limit`.
- Verify hold behavior (`11`) and estop (`13`) before aggressive commands.

## 8. Logs And Post-Run Analysis

When `save_data_flag: true`, controller and solver output:

- `*_controller_metadata.json`
- `*_controller_records.jsonl`
- `*_solver_metadata.json`
- `*_solver_records.jsonl`

Quick analysis:

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/analyze_structured_logs.py \
  --records /path/to/session_controller_records.jsonl \
  --vector-field policy_action
```

## 9. Typical Failure Modes

1. Manifest dim mismatch:
   - `obs_dim` does not equal resolved manifest dimension.
2. Joint name mismatch:
   - `action_joint_order` / `obs_joint_order` names differ from canonical names.
3. ONNX IO mismatch:
   - input/output names differ from profile `policy_io`.
4. ONNX metadata mismatch:
   - metadata check enabled but keys/values do not match profile.
5. External observation false confidence:
   - `required: true` is currently not hard-enforced; missing external features are zero-padded.

