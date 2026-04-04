# Humanoid Sim2Real Deploy (DDS Upper Layer + Motor SHM Loop)

This repository uses ROS2/CMake for build and process management.
Runtime communication now follows a hybrid architecture:

- DDS (ROS2 middleware) for upper-layer deploy dataflow:
  - policy command/state
  - teleop command
  - walk/lifecycle mode
  - IMU stream
- Shared memory only for motor closed-loop in `RL_solver`:
  - `sendMotorCmd()`
  - `getMotorState()`

Controller policy selection is now mode-profile driven:

- `config/rl_cfg.yaml` -> `deploy_mode_profiles` maps `mode_id` to policy config sections.
- DDS control words support generic mode switching (`mode_id`, `1000+mode_id`, `2000+mode_id`) and legacy commands.

## Key Modules

- `src/humanoid_rl_controller/rl_master/include/rl_master/robot_io.h`: unified RobotIO interface
- `src/humanoid_rl_controller/rl_master/include/rl_master/dds_robot_io.h`: controller-side DDS RobotIO
- `src/humanoid_rl_controller/rl_master/include/rl_master/solver_dds_bridge.h`: solver-side DDS bridge
- `src/humanoid_rl_controller/rl_master/include/rl_master/solver/robot_solver.h`: modular solver main-loop interface
- `src/humanoid_rl_controller/rl_master/include/rl_master/solver/motor_shm_io.h`: motor shared-memory I/O abstraction
- `src/humanoid_rl_controller/rl_master/include/rl_master/deploy_state_machine.h`: deploy lifecycle state machine
- `src/humanoid_rl_controller/rl_master/include/rl_master/logging/structured_logger.h`: structured runtime logging interface
- `src/humanoid_rl_controller/rl_master/docs/dds_sim2real_deploy_guide.md`: full deploy guide and topic contract
- `src/humanoid_rl_controller/rl_master/docs/sim2real_deploy_framework_upgrade.md`: framework upgrade summary

## Docs

- Main guide:
  - `src/humanoid_rl_controller/rl_master/docs/dds_sim2real_deploy_guide.md`
- MuJoCo sim2sim guide:
  - `src/humanoid_sim2sim/mujoco_sim2sim/docs/sim2sim_mujoco_deploy_guide.md`
- BeyondMimic/AMP adaptation:
  - `src/humanoid_rl_controller/rl_master/docs/beyondmimic_sim2real_adaptation.md`
- Script runtime notes:
  - `script/README.md`
- Full runtime checklist and analysis workflow:
  - `src/humanoid_rl_controller/rl_master/docs/runbooks/runtime_checklist.md`
