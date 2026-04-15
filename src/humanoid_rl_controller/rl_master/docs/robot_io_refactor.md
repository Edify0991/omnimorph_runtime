# RobotIO Refactor (Legacy DDS Runtime)

## 目标

把控制器 I/O 与传输方式解耦：

- `RL_controller` 只依赖 `RobotIO` 抽象
- 当前实现为 `DdsRobotIO`
- 未来可按需扩展其他后端（不影响控制器主逻辑）

注意：

- 这份文档描述的是 legacy 双进程 `RobotIO` 路径
- 现役 fused runtime 已不再使用 `/humanoid/rl/command`

## 运行链路

1. `read_state()` <- `/humanoid/rl/state`
2. `read_control_command()` <- `/humanoid/rl/teleop`
3. `read_mode_command()` <- `/humanoid/rl/walk_mode`
4. `controller.step(...)` 产出动作
5. `write_command()` -> legacy `/humanoid/rl/command`

异常/停机路径：

- `controller.estop()`
- `robot_io.estop()`

## 协议兼容性

- 命令布局语义保留：`open_rl + seq + timestamp`
- `open_rl` 用作安全联锁 + 执行模式编码：
  - `0`: hold / CSP
  - `10`: policy torque
  - `20`: non-policy command stream (CSP tracking)
- `RL_solver` 的 watchdog 逻辑可直接复用
- 状态机控制字已泛化：支持 `1000+mode_id`、`2000+mode_id`
- 生命周期控制字（推荐）：`10/11/12/13`
- 生命周期控制字（兼容）：`3001/3002/3003/3004`

## 清理结果

- 旧的 `shared_memory_robot_io.*` 已删除
- 电机闭环 SHM 逻辑仅保留在 `RL_solver` 中
- `/humanoid/rl/command` 已收口为 legacy 接口，不再属于现役 fused runtime
