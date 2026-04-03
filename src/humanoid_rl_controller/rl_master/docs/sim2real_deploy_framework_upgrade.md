# Sim2Real Deploy Framework Upgrade Summary

## 升级结论

当前框架已完成“上层 DDS、底层电机 SHM”混合架构：

- 保留共享内存：仅 `RL_solver` 的电机闭环
  - `sendMotorCmd()`
  - `getMotorState()`
- 迁移到 DDS：其余部署通信
  - 策略命令、机器人状态、遥控命令、模式控制、IMU 上行

完整运行说明请看：`docs/dds_sim2real_deploy_guide.md`。

## 本次已完成改造

1. 传输层重构
- `RL_controller` 使用 `DdsRobotIO`
- `RL_solver` 使用 `SolverDdsBridge`
- 统一 topic 协议由 `dds_protocol.*` 管理

2. 状态机标准化
- 引入 `deploy_state_machine.*`
- 支持 `START_POLICY / STOP_POLICY / ZEROING / ESTOP`
- 支持 `START_WALK / START_STAND / START_FIX_STAND`

3. 多模型与可配置观测
- 主模型 + `sub_models` 融合
- `ObservationBuilder` 支持 `reference_motion` 与 `external_sensor`
- 兼容 AMP / BeyondMimic 类型部署

4. IMU 链路清理
- `imu_communication_yesense` 改为纯 DDS 发布 `/imu/yesense`
- 删除 IMU 共享内存写入与相关构建依赖

5. 冗余代码删除
- 删除 `shared_memory_robot_io.*`
- 删除 `RL_controller_bak.cpp`

## 关键兼容性说明

- `open_rl + seq + timestamp` 语义保持不变。
- 仅运输通道从 SHM 改为 DDS（上层链路）。
- 电机驱动底层接口不受影响（继续 SHM）。
