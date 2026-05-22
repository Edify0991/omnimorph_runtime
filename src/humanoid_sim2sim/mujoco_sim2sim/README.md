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
source /home/edify/Code/jc01_deploy/script/dev_env.sh
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  backend:=cpp \
  mode_id:=0 \
  control_hz:=100.0 \
  enable_viewer:=true
```

The standard Python GUI path is the `python_frontend` viewer client layered on
top of the fused C++ backend.

Useful references:

- [MuJoCo Sim2Sim Docs Index](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/README.md)
- [MuJoCo Sim2Sim Deploy Guide](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_mujoco_deploy_guide.md)
- [Sim2Sim Runtime Environment Notes](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)
