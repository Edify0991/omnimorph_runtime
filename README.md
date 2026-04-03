# Modified Sim2Real Deploy Snapshot

This folder is a standalone snapshot of the current refactored codebase.

Included:

- `src/`
- `script/`
- `thirdparty/`
- `.gitignore`

Main architectural refactor in this snapshot:

- `rl_master/include/rl_master/robot_io.h`
- `rl_master/include/rl_master/shared_memory_robot_io.h`
- `rl_master/shared_memory_robot_io.cpp`
- `rl_master/include/rl_master/robot_types.h`
- `rl_master/RL_controller.cpp` / `sim2real_rl_controller.cpp` (now via `RobotIO`)
- `rl_master/docs/sim2real_deploy_framework_upgrade.md` (state machine + multi-model + configurable observation features)
- `rl_master/include/rl_master/dds_robot_io.h` + `rl_master/include/rl_master/solver_dds_bridge.h` (DDS transport for upper-layer deploy path)
- `rl_master/docs/dds_sim2real_deploy_guide.md` (complete DDS deploy architecture and topic contract)
