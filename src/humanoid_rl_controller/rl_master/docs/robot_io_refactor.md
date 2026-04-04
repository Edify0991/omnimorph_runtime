# RobotIO Refactor (DDS Runtime)

## 目标

把控制器 I/O 与传输方式解耦：

- `RL_controller` 只依赖 `RobotIO` 抽象
- 当前实现为 `DdsRobotIO`
- 未来可按需扩展其他后端（不影响控制器主逻辑）

## 运行链路

1. `read_state()` <- `/humanoid/rl/state`
2. `read_control_command()` <- `/humanoid/rl/teleop`
3. `read_mode_command()` <- `/humanoid/rl/walk_mode`
4. `controller.step(...)` 产出动作
5. `write_command()` -> `/humanoid/rl/command`

异常/停机路径：

- `controller.estop()`
- `robot_io.estop()`

## 协议兼容性

- 命令布局语义保留：`open_rl + seq + timestamp`
- `RL_solver` 的 watchdog 逻辑可直接复用
- 状态机控制字已泛化：支持 `mode_id`、`1000+mode_id`、`2000+mode_id`
- 旧控制字仍兼容：`10/11/12/13/20/21/22`

## 清理结果

- 旧的 `shared_memory_robot_io.*` 已删除
- 电机闭环 SHM 逻辑仅保留在 `RL_solver` 中
