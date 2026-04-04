# DDS Sim2Real Deploy Guide (Motor SHM + Upper DDS)

## 1. 目标与边界

本次框架升级遵循一条硬约束：

- 仅保留电机闭环共享内存路径：
  - `RobotSolver::getMotorState()`
  - `RobotSolver::sendMotorCmd()`
- 其余部署通信全部迁移到 DDS（ROS2 中间件传输）。

这意味着：策略、遥控、状态回传、IMU 上层流全部走 DDS；电机底层驱动闭环仍保持你现有 SHM 方案。

## 2. 最终运行架构

### 2.1 控制器侧（`RL_controller`）

入口：`sim2real_rl_controller.cpp`

- I/O 后端：`DdsRobotIO`
- 订阅：
  - `/humanoid/rl/state`（机器人状态）
  - `/humanoid/rl/teleop`（速度命令）
  - `/humanoid/rl/walk_mode`（模式/状态机控制字）
- 发布：
  - `/humanoid/rl/command`（策略输出）

### 2.2 求解器侧（`RL_solver`）

- 电机环：共享内存（不变）
  - 目标下发：`target_handle`
  - 反馈读取：`feedback_handle`
- DDS 桥：`SolverDdsBridge`
  - 订阅 `/humanoid/rl/command`
  - 订阅 `/imu/yesense`
  - 发布 `/humanoid/rl/state`

### 2.3 IMU 侧（`imu_communication_yesense`）

- 仅发布 `sensor_msgs/msg/Imu` 到 `/imu/yesense`
- 已移除 IMU 共享内存写入逻辑

### 2.4 人机输入侧（`joyLaunch.py`）

- 发布 `/humanoid/rl/teleop`（`Twist`）
- 发布 `/humanoid/rl/walk_mode`（`Int32`）
- 不再写共享内存命令段

### 2.5 Sim2Sim 侧（`mujoco_sim2sim`，新增）

- 包路径：`src/humanoid_sim2sim/mujoco_sim2sim`
- 节点：`mujoco_sim_bridge`
  - 订阅 `/humanoid/rl/command`
  - 在 MuJoCo 中执行关节控制
  - 发布 `/humanoid/rl/state`

这样 `RL_controller` 不需要修改，只是把对端从 `RL_solver` 换成 MuJoCo 仿真桥。

## 3. DDS Topic 协议约定

### 3.1 `/humanoid/rl/command`

- 类型：`std_msgs/msg/Float32MultiArray`
- 方向：`RL_controller -> RL_solver`
- 数据布局：
  - `[q, dq, tau] * 12`
  - `open_rl`
  - `seq`
  - `stamp_sec`

说明：保留了旧协议里的 `open_rl + seq + timestamp` 语义，便于继续沿用 watchdog 与兼容逻辑。
当前建议语义：

- `open_rl=0`: hold（不推理，solver 回退 CSP 持位）
- `open_rl=10`: policy torque mode（推理控制，solver 使用 CST/R1 组合）
- `open_rl=20`: command stream mode（非推理命令流，solver 使用 CSP 位置跟踪，如 zeroing）

### 3.2 `/humanoid/rl/state`

- 类型：`std_msgs/msg/Float32MultiArray`
- 方向：`RL_solver -> RL_controller`
- 数据布局：
  - `[q, dq, tau] * 12`
  - `base_ang_vel(3)`
  - `base_quat(4)`（`x,y,z,w`）
  - `base_rpy(3)`

### 3.3 `/humanoid/rl/teleop`

- 类型：`geometry_msgs/msg/Twist`
- 方向：joystick/navigation -> controller
- 字段映射：
  - `linear.x -> vx`
  - `linear.y -> vy`
  - `angular.z -> dyaw`

### 3.4 `/humanoid/rl/walk_mode`

- 类型：`std_msgs/msg/Int32`
- 方向：joystick/navigation -> controller

控制字（泛化）：

- `[0..999]`: 直接设置 `mode_id`
- `1000 + mode_id`: 设置 `mode_id` 并 `START_POLICY`
- `2000 + mode_id`: 仅设置 `mode_id`（不触发生命周期变更）
- `10/11/12/13`: `START_POLICY / STOP_POLICY / ZEROING / ESTOP`

说明：

- `fix_stand` 不再作为“策略模式”硬编码。
- 若需要固定姿态保持（CSP 持位），应使用 `STOP_POLICY(11)` 进入 hold。

## 4. 代码模块划分（规范化与模块化）

- 传输协议：
  - `include/rl_master/dds_protocol.h`
  - `dds_protocol.cpp`
- 控制器 I/O：
  - `include/rl_master/robot_io.h`
  - `include/rl_master/dds_robot_io.h`
  - `dds_robot_io.cpp`
- 求解器桥接：
  - `include/rl_master/solver_dds_bridge.h`
  - `solver_dds_bridge.cpp`
- 求解器电机与主循环（模块化后）：
  - `include/rl_master/solver/motor_shm_io.h`
  - `solver/motor_shm_io.cpp`
  - `include/rl_master/solver/robot_solver.h`
  - `solver/robot_solver.cpp`
  - `rl_solver.cpp`（仅保留进程入口、实时优先级与信号处理）
- 状态机：
  - `include/rl_master/deploy_state_machine.h`
  - `deploy_state_machine.cpp`
- 观测构建与扩展：
  - `observation_builder.*`
  - `reference_motion_provider.*`
  - `external_observation_provider.*`
- 运行时与工程化组件：
  - `include/rl_master/runtime/realtime_utils.h` + `runtime/realtime_utils.cpp`
  - `include/rl_master/filters/moving_average_filter.h` + `filters/moving_average_filter.cpp`
  - `include/rl_master/logging/structured_logger.h` + `logging/structured_logger.cpp`

已删除的冗余旧路径：

- `shared_memory_robot_io.*`
- `RL_controller_bak.cpp`
- IMU 包中的共享内存写入与相关构建依赖

## 5. 多模型与观测可配置能力

支持项（通过 `rl_cfg.yaml` + observation manifest）：

- AMP 风格观测
- BeyondMimic 风格观测（含 `reference_motion` / `reference_joint_pos` / `reference_joint_vel` /
  `motion_anchor_pos_b` / `motion_anchor_ori_b` / `motion_body_pos_b` / `motion_body_ori_b` /
  `robot_body_pos` / `robot_body_ori`）
- 视觉/雷达等外部观测占位接口（`external_observations`）
- 主模型 + 多个 `sub_models` 的融合推理
- AMP 判别器可选推理（`amp_discriminator`），用于在线质量监测/日志记录，不影响主控制输出
- `deploy_mode_profiles` 驱动的 `mode_id -> policy config section` 动态映射
- 部署状态机（启动、停止、回零、急停）
- 参考动作多来源融合（`reference_motion_source=auto/file/policy_outputs`）与加载安全检查

建议清单：

- AMP：`config/observation_manifest_amp.yaml`
- BeyondMimic：`config/observation_manifest_beyondmimic.yaml`

`rl_cfg.yaml` 中可通过 `deploy_mode_profiles` 扩展模式映射，例如：

```yaml
deploy_mode_profiles:
  - mode_id: 0
    config_section: sim2real
    tag: walk
  - mode_id: 1
    config_section: stand_sim2real
    tag: stand
  - mode_id: 3
    config_section: stair_sim2real
    tag: stair
```

## 6. 编译依赖

### 6.1 `rl_master`

- `rclcpp`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `SharedMemory`（仅 RL_solver 电机闭环需要）

### 6.2 `imu_communication_yesense`

- `rclcpp`
- `sensor_msgs`
- 其余 ROS 依赖保持不变
- 不再链接 `SharedMemory`

## 7. 启动顺序（推荐）

1. 启动底层电机驱动栈（不在本仓库，保持原方案）。
2. 启动 IMU 节点：发布 `/imu/yesense`。
3. 启动 `RL_solver`（SHM 电机环 + DDS bridge）。
4. 启动 `RL_controller`（DDS RobotIO + 策略推理）。
5. 启动 `joyLaunch.py`（DDS 遥控与控制字）。

## 8. 迁移对照

- 原先 `RL_controller <-> RL_solver` 的 SHM 命令/状态段：已替换为 DDS topic。
- 原先 IMU 额外 SHM 通道：已删除，统一 DDS。
- 电机底层闭环 SHM：保留，不改动你的驱动接口。

## 9. 常见问题定位

- `RL_controller` 一直报等待状态流：
  - 检查 `RL_solver` 是否已启动并发布 `/humanoid/rl/state`。
- 姿态不更新：
  - 检查 `/imu/yesense` 是否有 `sensor_msgs/msg/Imu` 数据。
- 模式切换无效：
  - 检查 `/humanoid/rl/walk_mode` 是否发送了约定控制字。
- 策略无输出：
  - 检查 `walk_mode` 是否处于 `START_POLICY` 之后的运行状态。

## 10. 一键自检脚本

仓库内置脚本：`script/dds_selfcheck.sh`

- 默认只读检查：topic/type/端点连通/基础频率
- 可选发布冒烟：teleop 零值 + `STOP_POLICY(11)`
- 可选按序发布状态机控制字

示例：

```bash
cd script
sudo ./dds_selfcheck.sh
sudo ./dds_selfcheck.sh --publish-smoke
sudo ./dds_selfcheck.sh --publish-sequence "11,12,1000"
```

## 11. 结构化数据记录与分析

运行时开启 `save_data_flag: true` 后，`RL_solver` 与 `RL_controller` 会输出：

- `*_solver_metadata.json`
- `*_solver_records.jsonl`
- `*_controller_metadata.json`
- `*_controller_records.jsonl`

元数据包含模型路径、观测维度、控制频率、关节顺序、PD 参数、子模型列表等，用于跨模型/跨参数对齐分析。

分析工具：

- `tools/analysis/analyze_structured_logs.py`

完整运行清单、命名规范和分析流程见：

- `docs/runbooks/runtime_checklist.md`
- 观测构建流程图与维度偏移示意：
  - `docs/observation_pipeline_diagram.md`
