# MuJoCo Sim2Sim

The standard MuJoCo sim2sim runtime is now the C++ fused bridge:

- executable: `mujoco_sim_bridge`
- launch: `sim2sim_mujoco.launch.py`
- standard backend: `backend:=cpp`

It embeds the same `IntegratedControllerRuntime` used by the real-robot `RL_solver` path, so sim2sim and sim2real now share:

- deploy state machine
- observation pipeline
- ONNX inference
- policy switching
- command interpretation

Manual terminal startup:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=cpp \
  mode_id:=0 \
  control_hz:=100.0 \
  enable_viewer:=true
```

Start/stop policy and publish velocity commands from another terminal:

```bash
source ${OMNIMORPH_RUNTIME_ROOT}/script/dev_env.sh
ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 1000}"

ros2 topic pub -r 20 /omnimorph/rl/teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub -r 20 /omnimorph/rl/teleop geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"

ros2 topic pub --once /omnimorph/rl/mode_control std_msgs/msg/Int32 "{data: 11}"
```

The standard Python GUI path is the `python_frontend` viewer client layered on
top of the fused C++ backend.

## COM And Support Visualization

Offline MCAP replay in MuJoCo with Pinocchio COM, COM projection, COP, and
support polygon overlay:

```bash
python3 src/omnimorph_sim2sim/mujoco_sim2sim/scripts/replay_mcap_com_support.py \
  --mcap src/omnimorph_rl_controller/rl_master/data/runtime_logs/Jun12_09-46-01_beyondmimic_jc01_dance_wo_state_estimation.mcap \
  --running-only \
  --joint-field joint_q
```

Live sim2sim native viewer overlay:

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  bridge_config:=/abs/path/to/jc01_fullbody_engineai_walk_sim2sim.yaml \
  enable_viewer:=true \
  enable_com_support_visualization:=true
```

The overlay uses Pinocchio for COM, MuJoCo contact forces for COP, and MuJoCo
foot sites (`right_foot_site`, `left_foot_site` by default) to construct an
approximate support polygon. If the runtime log does not contain
`base_pos_w/base_quat`, offline replay uses the fixed fallback base pose.

Useful references:

- [MuJoCo Sim2Sim Docs Index](${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/docs/README.md)
- [MuJoCo Sim2Sim Deploy Guide](${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/docs/sim2sim_mujoco_deploy_guide.md)
- [Sim2Sim Runtime Environment Notes](${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)
