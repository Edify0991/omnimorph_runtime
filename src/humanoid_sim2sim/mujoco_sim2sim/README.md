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

Standard startup:

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --auto-start-mode
```

The Python interactive backend is kept only as a legacy / experimental path and is no longer the standard deploy runtime.

Useful references:

- [MuJoCo Sim2Sim Deploy Guide](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_mujoco_deploy_guide.md)
- [Sim2Sim Runtime Environment Notes](/home/edify/Code/jc01_deploy/src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_runtime_environment_notes.md)
