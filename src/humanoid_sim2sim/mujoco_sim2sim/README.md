# mujoco_sim2sim

`mujoco_sim2sim` is an independent sim2sim package that keeps your current `sim2real` framework unchanged.

- It subscribes policy commands from `/humanoid/rl/command`.
- It steps MuJoCo physics and publishes `/humanoid/rl/state`.
- `RL_controller` can run without code change, only the deployment object switches from real robot (`RL_solver`) to MuJoCo.
- For trajectory test scenarios, base lock is supported via launch args `fixed_base:=true` and optional `fixed_base_height:=...`.
- Optional visualization window is supported via `enable_viewer:=true` (requires build with GLFW support).
- Viewer hotkeys:
  - `Space`: pause/resume simulation
  - `Right Arrow`: step one frame when paused
  - `[` / `]`: simulation speed scale down/up
  - `C`: toggle contact points/forces
  - `B`: toggle base angular velocity overlay
  - `H`: toggle HUD
- Actuator compatibility:
  - `actuator_control_mode:=auto|torque|position` (default `auto`)
- Optional runtime freeze behavior:
  - `pause_when_no_command:=true` pauses physics when no fresh control stream exists.

Detailed guide:

- `docs/sim2sim_mujoco_deploy_guide.md`

Backends:

- `backend:=cpp`: C++ bridge backend.
- `backend:=python_interactive`: official MuJoCo Python viewer backend with left/right UI panels and interactive controls.

Python interactive backend runtime dependencies:

- `pip install mujoco numpy`
- ROS2 Python runtime (`rclpy`) available in your Humble environment.

Architecture-adaptive CMake options:

- `-DMUJOCO_SIM2SIM_ENABLE_ARCH_TUNING=ON|OFF`
- `-DMUJOCO_SIM2SIM_ENABLE_NATIVE_TUNING=ON|OFF`
- `-DMUJOCO_SIM2SIM_ARM_BASELINE=armv8-a` (for portable ARM builds)

`sim2sim_mujoco.launch.py` realtime override args for `RL_controller` (when `start_rl_controller:=true`):

- `controller_rt_enabled`
- `controller_rt_lock_memory`
- `controller_rt_set_affinity`
- `controller_rt_cpu_id`
- `controller_rt_use_fifo`
- `controller_rt_fifo_priority`
