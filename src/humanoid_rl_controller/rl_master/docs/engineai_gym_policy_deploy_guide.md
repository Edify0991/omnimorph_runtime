# EngineAI Gym Policy Deploy Guide

This guide describes how to take a policy trained in an EngineAI Gym-style RL framework and deploy it through this repository's fused runtime.

## 1. Target Runtime

### Sim2Sim

Use the fused C++ MuJoCo runtime:

```bash
./script/sim2sim_engineai.sh --model-path /abs/path/to/robot.xml --mode-id 0
```

### Sim2Real

Use the fused real-robot runtime:

```bash
./script/sim2real_engineai.sh --mode-id 0
```

## 2. Prepare the Policy

Place the exported ONNX file under:

```text
src/humanoid_rl_controller/rl_master/policies/
```

Example:

```text
src/humanoid_rl_controller/rl_master/policies/engineai_walk.onnx
```

## 3. Prepare the Config

Edit `src/humanoid_rl_controller/rl_master/config/rl_cfg.yaml`.

For the target mode profile, confirm:

- `policy_file` or `policy_path`
- `obs_dim`
- `action_dim`
- `action_joint_order`
- `obs_joint_order`
- `observation_manifest_file` / `observation_manifest_path`
- `policy_io.obs_input_name`
- `policy_io.action_output_name`
- any required `time_step` input settings

## 4. Prepare the Observation Manifest

Create or update the manifest under:

```text
src/humanoid_rl_controller/rl_master/config/
```

Typical pattern:

```text
observation_manifest_engineai_walk.yaml
```

Then point the mode config to that manifest.

## 5. Run the Offline Precheck

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py --mode-id 0
```

This checks at least:

- profile selection
- manifest parsing
- observation dimension consistency
- joint-order consistency
- ONNX presence and basic I/O compatibility
- optional metadata checks if enabled

## 6. Sim2Sim Workflow

1. confirm the XML model path
2. run precheck for the mode
3. launch fused sim2sim runtime
4. publish `walk_mode` start word or use `--auto-start-mode`
5. inspect viewer motion, joint behavior, and `/humanoid/rl/state`

Example:

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --enable-viewer true \
  --auto-start-mode
```

## 7. Sim2Real Workflow

1. bring up motor driver
2. bring up IMU DDS publisher
3. run fused real runtime
4. publish `walk_mode` start word
5. use joystick / teleop DDS as needed

Example:

```bash
sudo ./script/driver.sh
sudo ./script/imu.sh
./script/sim2real_engineai.sh --mode-id 0 --auto-start-mode
sudo python3 ./script/joyLaunch.py
```

## 8. How Sim2Sim and Sim2Real Stay Aligned

The two paths now share the same embedded controller runtime:

- same `RL_controller::step(...)`
- same deploy state machine
- same observation manifest parsing
- same policy switching logic
- same ONNX inference stack

The environment-specific part is only:

- where state comes from
- where commands are applied

On the real-robot fused path, deploy-mode config is loaded once through a shared `ModeProfileRegistry`, then reused by both solver-side execution logic and controller-side policy logic.

On the current MuJoCo fused path, the same mode/profile definitions are still used, but the registry is lazily created inside controller initialization if it was not injected earlier.

That means sim2sim is now a much closer validation target for the real deploy path.

## 9. Choosing Between C++ and Python Sim2Sim Backends

Use `sim2sim_engineai.sh` when you want the closest validation target to the real fused runtime.

Use `sim2sim_engineai_python.sh` when you want the friendlier Python MuJoCo GUI while still keeping the fused C++ control/physics runtime:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

Use `sim2sim_engineai_python_legacy.sh` only when you need the historical split runtime for comparison:

```text
RL_controller (legacy standalone process) <-> DDS <-> python_interactive backend
```
