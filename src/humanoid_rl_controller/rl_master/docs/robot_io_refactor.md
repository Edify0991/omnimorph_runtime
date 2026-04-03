# RobotIO Refactor (DDS Runtime)

## New Layering

- Control/Policy: `RL_controller`
- Unified robot interface: `RobotIO`
- Active backend: `DdsRobotIO`
- Transport:
  - DDS topics for policy/state/teleop/mode
  - Shared memory only for motor target/feedback in `RL_solver`

## Updated Runtime Chain

1. `robot_io.read_state(state)` from DDS `/humanoid/rl/state`
2. `robot_io.read_control_command(teleop_cmd)` from DDS `/humanoid/rl/teleop`
3. `walk_mode = robot_io.read_walk_mode(...)` from DDS `/humanoid/rl/walk_mode`
4. `cmd = controller.step(state, teleop_cmd, walk_mode, phase_t)`
5. `robot_io.write_command(cmd)` to DDS `/humanoid/rl/command`

On stop/fault:

- `controller.estop()`
- `robot_io.estop()`

## Compatibility

- Command payload layout keeps `open_rl + seq + timestamp`.
- Solver watchdog logic is preserved.
- `walk_mode` lifecycle control words are preserved:
  `10/11/12/13/20/21/22`.
