# 运行时全链路函数与数据流文档（sim2sim + sim2real）

本文目标：把这个仓库在**实际运行时**的主流程函数、调用顺序、数据传输路径全部串起来，方便你做策略接入、调试和问题定位。  
说明：这里聚焦“运行主链”，不是仓库里每一个辅助函数。

---

## 1. 总体进程与通信拓扑

核心 DDS Topic（`rl_master/include/rl_master/dds_protocol.h`）：

- `/humanoid/rl/state`
  - 发布者：`mujoco_sim_bridge`（sim2sim）或 `rl_solver`（sim2real）
  - 订阅者：`RL_controller`
- `/humanoid/rl/command`
  - 发布者：`RL_controller`
  - 订阅者：`mujoco_sim_bridge`（sim2sim）或 `rl_solver`（sim2real）
- `/humanoid/rl/teleop`
  - 发布者：遥控/上位机
  - 订阅者：`RL_controller`
- `/humanoid/rl/walk_mode`
  - 发布者：状态机控制端
  - 订阅者：`RL_controller`

两条部署路径：

- sim2sim：`RL_controller` + `mujoco_sim_bridge`（C++ 或 Python interactive backend）
- sim2real：`RL_controller` + `rl_solver`（`RobotSolver`）

---

## 2. 数据结构与编码顺序

定义在 `rl_master/include/rl_master/robot_types.h`、`rl_master/include/rl_master/rl_protocol.h`、`rl_master/dds_protocol.cpp`。

### 2.1 RobotStateData（state）

- 关节：`joint_q[12]`, `joint_dq[12]`, `joint_tau[12]`
- 基座：`base_ang_vel[3]`, `base_quat[4](xyzw)`, `base_rpy[3]`

编码到 `Float32MultiArray` 的顺序：

1. 每关节 3 个值（q,dq,tau）共 36
2. `base_ang_vel` 3
3. `base_quat` 4
4. `base_rpy` 3

总长度：`36 + 3 + 4 + 3 = 46`

### 2.2 RobotCommandData（command）

- `joint_target_q[12]`
- `joint_target_dq[12]`
- `joint_target_tau[12]`
- `open_rl`

编码顺序：

1. 每关节 3 个值（target_q,target_dq,target_tau）共 36
2. `open_rl`
3. `sequence`
4. `stamp_sec`

总长度：`36 + 1 + 2 = 39`

### 2.3 open_rl 模式码

`rl_master/include/rl_master/rl_protocol.h`：

- `0`：disabled（hold）
- `10`：policy
- `20`：command_stream（位置流）
- `30`：test_csp
- `40`：test_cst
- `50`：test_r1

统一解释器：`resolveCommandRuntimeMode(...)`（`command_runtime_mode.h`）。

---

## 3. 启动入口函数（按进程）

### 3.1 RL_controller 进程

文件：`rl_master/sim2real_rl_controller.cpp`

1. `main()`
2. `RL_controller::create()`
3. `DdsRobotIO` 构造
4. `run_sim2real_rl_controller(controller, robot_io)`
5. `controller->RL_controller_Init()`
6. `robot_io->connect()`
7. 循环：
   - `robot_io->read_state(...)`
   - `robot_io->read_control_command(...)`
   - `robot_io->read_mode_command(...)`
   - `controller->step(...)`
   - `robot_io->write_command(...)`

### 3.2 sim2real solver 进程

文件：`rl_master/rl_solver.cpp`

1. `main()`
2. `RobotSolver::create()`
3. `solver->initialize()`
4. 新线程 `solver->run()`
5. SIGINT 时 `solver->requestStop()`

### 3.3 sim2sim C++ bridge 进程

文件：`mujoco_sim2sim/src/main.cpp`

1. `main()`
2. `std::make_shared<MujocoSimBridge>()`
3. `rclcpp::spin(node)`

`MujocoSimBridge` 构造顺序：

1. `loadParameters()`
2. `loadModel()`
3. `resolveModelMappings()`
4. `initializeState()`
5. `initializeViewer()`
6. `setupRosInterfaces()`

### 3.4 sim2sim Python interactive backend 进程

文件：`mujoco_sim2sim/scripts/mujoco_sim_interactive_backend.py`

1. `main()`
2. `rclpy.init()`
3. `node = MujocoInteractiveBackend()`
4. `node.run()`

`MujocoInteractiveBackend.__init__` 顺序：

1. `_declare_parameters()`
2. `_load_parameters()`
3. `_load_model()`
4. `_resolve_mappings()`
5. `_initialize_base_lock_if_enabled()`
6. `_initialize_last_targets()`
7. `_log_startup_diagnostics()`
8. 创建 pub/sub

---

## 4. RL_controller 初始化函数链

入口：`RL_controller::RL_controller_Init()`

顺序：

1. `robot->initialize_buffers()`
2. `initModeProfiles()`
   - `loadModeProfileSpecsFromYaml()` 读取 `deploy_mode_profiles`
   - 每个 profile：
     - `cfg.loadFromYAML(...)`
     - `buildDefaultAnglesFromCfg(...)`
     - `buildActionIndexMap(...)`
     - `buildObsIndexMap(...)`
     - `ObservationManifest::loadFromYAML(...)`
     - `ObservationBuilder(...)`
     - `initPolicyGroup(...)`（主 ONNX + 可选 sub_models）
     - `initAmpDiscriminatorRunner(...)`
     - `initReferenceMotionProvider(...)`
3. `refreshPolicyMode(default_mode_id_, true)`
4. `handlePolicySwitch()`
   - 清 obs/action 缓冲
   - 可选 reset ONNX runner
   - `deploy_state_machine_.configure(cfg)`
   - `deploy_state_machine_.setZeroPose(...)`
5. `initDataLogger()`

---

## 5. RL_controller 单步主链（最关键）

入口：`RL_controller::step(state, command, mode_command, phase_t)`

每周期严格顺序：

1. `updateStateFromIO(state)`  
   把 DDS state 写入 `robot->joint_q/dq/tau/base_*`
2. `updateCommandFromIO(command)`  
   把 teleop 写入 `cmd.vx/vy/dyaw`
3. 状态机初始化（首次）：
   - `deploy_state_machine_.configure(activePolicyCfg())`
   - `deploy_state_machine_.initialize(robot->joint_q, activeZeroPose(), active_mode_id_)`
4. `deploy_output = deploy_state_machine_.update(mode_command, now_s, robot->joint_q)`
5. `refreshPolicyMode(deploy_output.locomotion_mode, true)`
6. `handlePolicySwitch()`（若 mode 切换）
7. 按生命周期输出分支：

- `deploy_output.enable_policy == true`（RUNNING）：
  1. `get_robot_observation(local_phase_t)`
     - `buildObservationFeatureContext(...)`
     - `activeObservationBuilder().build(...)`
  2. `update_obs_deque(current_obs)`
  3. `run_policy()`
     - 拼 stacked obs
     - `runPolicyGroup(...)` -> 每个 `OnnxPolicyRunner::forward(...)`
     - `runAmpDiscriminator(...)`（可选）
     - `clip + action_filter`
  4. `get_joint_target_q(policy_action)`（action 顺序映射 + default_angle + scale）
  5. `get_joint_target_torque(target_q)`（PD + 限幅）
  6. `robot->open_rl = kOpenRlPolicyEnabled (10)`

- `deploy_output.enable_command_stream == true`（ZEROING 等）：
  1. 直接采用 `deploy_output.target_q`
  2. `get_joint_target_torque(target_q)`
  3. `robot->open_rl = kOpenRlCommandStream (20)`

- 否则（HOLD/ESTOP）：
  1. `joint_target_q = joint_q`
  2. `joint_target_tau = 0`
  3. `robot->open_rl = kOpenRlDisabled (0)`

8. 组装 `RobotCommandData out_cmd`
   - 当前代码里写出：`joint_target_q` 有效，`joint_target_dq=0`，`joint_target_tau=0`（控制侧用 q 为主）
9. `logStepRecord(...)`
10. 返回 `out_cmd`

---

## 6. 状态机函数链（walk_mode -> lifecycle）

文件：`deploy_state_machine.cpp`

入口：`DeployStateMachine::update(control_word, now_s, current_q)`

1. `decodeControlWord(...)`
2. 更新 `active_locomotion_mode`
3. 根据控制词触发：
   - `start`
   - `stop`
   - `zeroing`（`startZeroing(...)`）
   - `estop`
4. 输出 `DeployStateOutput`
   - `state`
   - `locomotion_mode`
   - `enable_policy`
   - `enable_command_stream`
   - `target_q`

生命周期枚举：

- `kInitializing`
- `kHold`
- `kZeroing`
- `kRunning`
- `kEstop`

---

## 7. 观测构建函数链（manifest 驱动）

### 7.1 manifest 解析

`ObservationManifest::loadFromYAML(path)`：

1. 读取 `observation_manifest.terms`
2. 每个 term 解析 `name/type`, `count`, `components`, `source`, `enabled`
3. 存入 `terms_`

### 7.2 构建器初始化

`ObservationBuilder(manifest)`：

1. `registry()` 拿到 term->provider 映射
2. 校验 term 是否支持 components/count
3. 计算每个 term 的 `offset/dim`
4. 生成 `layout_description_`
5. 计算 `expected_dim_`

### 7.3 每周期 build

`ObservationBuilder::build(...)`：

1. 按 `resolved_terms_` 顺序调用 gather 函数
2. 每项校验 `added_dim == resolved_term.dim`
3. 全量 clip 到 `[-clip_observations, clip_observations]`
4. 校验总维度 `obs.size == expected_dim_`

### 7.4 外部观测

`ExternalObservationProvider::collect(specs)`：

1. 对每个 spec 查缓存
2. 缺失时用全 0 填充目标维度
3. 存在时 `fitDim(...)` 截断/补零

注意：`required=true` 当前不会硬报错，只会在离线校验脚本里给 warning。

---

## 8. ONNX 推理函数链

文件：`onnx_policy_runner.cpp`

### 8.1 初始化

`OnnxPolicyRunner::init()`：

1. 创建 ORT session
2. 读取输入输出名
3. 定位：
   - `obs_input_index`
   - `timestep_input_index`
   - `action_output_index`
4. 处理 `extra_output_names`
5. 处理未知输入（默认补零；`strict_model_io=true` 时报错）
6. `validateModelMetadata()`（开启时）
7. `reset()`

### 8.2 前向

`OnnxPolicyRunner::forward(observation)`：

1. 组装输入 tensor（obs / 可选 timestep / 其他零填充）
2. session `Run(...)`
3. 提取 action output
4. 按 `action_dim` 裁剪并校验最小维度
5. 采集 extra outputs
6. 如有 timestep 输入则 `time_step_++`

### 8.3 metadata 校验

`validateModelMetadata()`：

1. 读取 custom metadata map
2. 校验 `required_metadata_keys`
3. 校验 `expected_metadata`
4. 若 metadata 中存在这些键，则交叉校验：
   - `obs_dim`
   - `action_dim`
   - `obs_stack_n / obs_stack_N`
   - `obs_input_name`
   - `action_output_name`

`metadata_check_strict=true` 时任一不匹配直接抛异常。

---

## 9. sim2sim C++ bridge 控制循环函数链

每个 timer tick：`MujocoSimBridge::controlLoopTick()`

1. `has_fresh_command = commandFresh(now)`
2. `enforceBaseLock()`
3. `updateControlInput(now)`
   - 读取最新 command cache
   - `resolveCommandRuntimeMode(...)`
   - 根据模式选择 `q_des / dq_des / tau`
   - 根据 actuator 类型：
     - 位置执行器：`data_->ctrl = q_des`
     - 力矩执行器：PD 或 direct torque，最终限幅后写 `data_->ctrl`
   - no-command 行为：
     - `hold_position`
     - `hold_last`
     - `zero_torque`
4. 物理步进：
   - fresh command 或不暂停时：`mj_step`（若干 substeps）+ `mj_forward`
   - 否则仅 `mj_forward`
5. `publishRobotState()`
6. `renderViewerFrame()`

命令接收：`commandCallback(...)`  
状态发布：`publishRobotState(...)`

---

## 10. sim2sim Python backend 控制循环函数链

每周期 `_step_once(now_sec)`：

1. `_command_fresh(now_sec)`
2. 若 `pause_when_no_command` 且无新命令：
   - 仅 `_publish_robot_state()`，不步进
3. 否则：
   - `_update_control_input(now_sec)`（逻辑与 C++ bridge 对齐）
   - `mujoco.mj_step` x substeps
   - `mujoco.mj_forward`
   - `_publish_robot_state()`

主循环：`run()`

- viewer 模式：`launch_passive(...)` + `viewer.sync()`
- headless 模式：固定周期 sleep

---

## 11. sim2real RobotSolver 控制循环函数链

文件：`solver/robot_solver.cpp`

`RobotSolver::run()` 主循环顺序：

1. `dds_bridge_.spinOnce()`
2. `getRLCmd()`
   - `readLatestPolicyCommand(...)`
   - watchdog 新鲜度判定（seq + stamp）
   - `resolveCommandRuntimeMode(...)`
   - 根据模式设置 `joint_cmd_`
3. 若 active mode：
   - `getMotorState()`
   - `sendRLState()`
   - `sendMotorCmd()`
4. 否则 hold：
   - `getMotorState()`
   - 首次锁存 `hold_target_q_`
   - 持续 `RUN_MODE_CSP` 保持
   - `sendMotorCmd()`
   - `sendRLState()`
5. 定时 sleep（500Hz）

`getRLCmd()` 模式分支：

- `policy`：读目标 q/dq/tau，当前实现通常转为 CST 并在 solver 侧做 PD
- `command_stream/test_csp`：CSP 位置流
- `test_cst`：直接力矩流（限幅）
- `test_r1`：q/dq/tau 混合流

---

## 12. launch 到运行函数顺序（sim2sim）

文件：`mujoco_sim2sim/launch/sim2sim_mujoco.launch.py`

关键逻辑：

1. `backend:=cpp` -> 启动 `mujoco_sim_bridge`（C++）
2. `backend:=python_interactive` -> 启动 `mujoco_sim_interactive_backend.py`
3. `start_rl_controller:=true` -> 额外启动 `rl_master/RL_controller`
4. `start_rl_controller:=false` -> 只跑 bridge/backend，不会启动策略推理控制器

---

## 13. 一次完整闭环（时序摘要）

以 sim2sim + start_rl_controller=true 为例：

1. bridge 发布 `/humanoid/rl/state`
2. `RL_controller` 读 state + teleop + walk_mode
3. `RL_controller::step`：
   - 状态机
   - 观测构建
   - ONNX 推理
   - action->target_q 映射
   - 输出 `/humanoid/rl/command`
4. bridge 读 command
5. bridge 按 `open_rl` 模式把目标转换为 MuJoCo `ctrl`
6. MuJoCo 步进后再发布新 state
7. 回到第 1 步

---

## 14. 调试时最常看的函数（建议）

- 控制总入口：`RL_controller::step`
- 生命周期切换：`DeployStateMachine::update`
- 观测维度问题：`ObservationBuilder::build`、`ObservationManifest::loadFromYAML`
- ONNX 输入输出问题：`OnnxPolicyRunner::init`、`OnnxPolicyRunner::forward`
- 模式行为问题：`resolveCommandRuntimeMode`
- sim2sim 执行端：`MujocoSimBridge::updateControlInput` / Python `_update_control_input`
- sim2real 执行端：`RobotSolver::getRLCmd` + `RobotSolver::run`

---

## 15. 补充：离线配置体检脚本对应入口

文件：`rl_master/tools/analysis/validate_deploy_config.py`

主入口：`main()` -> `validate_profile(...)`

它会做：

1. `deploy_mode_profiles` 解析
2. manifest 维度与 `obs_dim` 一致性
3. `action_joint_order / obs_joint_order` 规则检查
4. ONNX I/O 合约检查
5. metadata 合约检查（如果配置开启）

常用命令：

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py \
  --rl-cfg src/humanoid_rl_controller/rl_master/config/rl_cfg.yaml
```

