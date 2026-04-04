# MuJoCo Sim2Sim Deploy Guide

## 1. 目标

在不破坏现有 `sim2real` 部署链路的前提下，新增一条 `sim2sim` 路径：

- `sim2real` 保持原样：`RL_controller` -> DDS -> `RL_solver` -> SHM -> 电机
- `sim2sim` 新增：`RL_controller` -> DDS -> `mujoco_sim_bridge` -> MuJoCo

这样可以复用同一套策略部署代码（观测构建、状态机、多模型推理、日志等），只替换部署对象。

## 2. 架构设计（对齐主流做法）

`mujoco_sim2sim` 采用“控制器与仿真器解耦”模式：

- `RL_controller` 不改，继续发布/订阅原有 DDS topic。
- `mujoco_sim_bridge` 作为适配层：
  - 订阅 `/humanoid/rl/command`
  - 在 MuJoCo 中按关节映射执行控制
  - 发布 `/humanoid/rl/state`

这和很多主流项目的 sim2sim 组织方式一致：策略运行与物理仿真通过统一中间件接口耦合。

## 3. 新增包位置

- `src/humanoid_sim2sim/mujoco_sim2sim`

关键文件：

- `src/mujoco_sim_bridge.cpp`
- `include/mujoco_sim2sim/mujoco_sim_bridge.h`
- `launch/sim2sim_mujoco.launch.py`
- `config/mujoco_sim2sim.yaml`

## 4. 依赖与编译

## 4.1 MuJoCo 库路径

`mujoco_sim2sim` 通过 CMake 自动查找：

- 头文件：`mujoco/mujoco.h`
- 动态库：`libmujoco.so`（或对应平台库名）

推荐设置：

```bash
export MUJOCO_ROOT=/path/to/mujoco
```

其中应包含：

- `$MUJOCO_ROOT/include/mujoco/mujoco.h`
- `$MUJOCO_ROOT/lib/libmujoco.so`

若未找到 MuJoCo，包会被安装但不会生成 `mujoco_sim_bridge` 可执行文件。

## 4.2 编译命令

在工作空间根目录执行：

```bash
colcon build --symlink-install --packages-up-to rl_master mujoco_sim2sim
source install/setup.bash
```

编译顺序说明：

- `mujoco_sim2sim` 依赖 `rl_master` 的协议头文件。
- 用 `--packages-up-to rl_master mujoco_sim2sim` 即可自动满足顺序。

## 5. 运行

## 5.1 一键启动（推荐）

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml
```

默认会同时拉起：

- `rl_master/RL_controller`
- `mujoco_sim2sim/mujoco_sim_bridge`

## 5.2 只启仿真桥（你自己手动起控制器）

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  start_rl_controller:=false
```

## 5.3 参数文件

默认参数文件：

- `src/humanoid_sim2sim/mujoco_sim2sim/config/mujoco_sim2sim.yaml`

可通过 launch 参数覆盖：

```bash
ros2 launch mujoco_sim2sim sim2sim_mujoco.launch.py \
  model_path:=/abs/path/to/robot.xml \
  bridge_config:=/abs/path/to/your_sim2sim.yaml
```

## 6. 参数说明（核心）

- `model_path`：MuJoCo 模型路径（必填）
- `joint_names`：RL 12 关节顺序到 MuJoCo 关节名映射
- `actuator_names`：MuJoCo actuator 映射（默认与 `joint_names` 同名）
- `control_hz`：桥接控制频率（通常与 `RL_control_f` 对齐）
- `sim_dt`：MuJoCo 内部积分步长
- `kp/kd/torque_limit`：仿真侧关节控制参数
- `command_timeout_sec`：策略命令超时保护
- `use_command_torque_ff`：是否叠加策略下发扭矩前馈
- `base_body_name/base_free_joint_name`：基座姿态/角速度提取配置

## 7. Topic 协议（与现有系统保持一致）

- 订阅：
  - `/humanoid/rl/command` (`Float32MultiArray`)
- 发布：
  - `/humanoid/rl/state` (`Float32MultiArray`)

数据布局完全沿用 `rl_master/dds_protocol.h`，不引入新协议，保证复用性。

## 8. 与 sim2real 的边界

`mujoco_sim2sim` 不会修改或替换以下路径：

- `RL_solver` 电机闭环共享内存逻辑
- `sendMotorCmd()` / `getMotorState()`
- 现有 real deploy 启动流程

你可以把它理解为“新增一个仿真侧 `RobotIO` 对端”。

## 9. 调试建议

1. 先确认 MuJoCo 模型能被正确加载（启动日志会打印 `nq/nv/nu`）。
2. 再确认关节/执行器映射（命名不匹配会直接报错退出）。
3. 观察 topic：
   - `ros2 topic hz /humanoid/rl/state`
   - `ros2 topic echo /humanoid/rl/command --once`
4. 若策略不启动，检查 `rl_cfg.yaml` 中 `auto_start_policy` 是否开启，或发送状态机控制字。
