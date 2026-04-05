# mujoco_sim2sim

`mujoco_sim2sim` is an independent sim2sim package that keeps your current `sim2real` framework unchanged.

- It subscribes policy commands from `/humanoid/rl/command`.
- It steps MuJoCo physics and publishes `/humanoid/rl/state`.
- `RL_controller` can run without code change, only the deployment object switches from real robot (`RL_solver`) to MuJoCo.
- For trajectory test scenarios, base lock is supported via launch args `fixed_base:=true` and optional `fixed_base_height:=...`.

Detailed guide:

- `docs/sim2sim_mujoco_deploy_guide.md`
