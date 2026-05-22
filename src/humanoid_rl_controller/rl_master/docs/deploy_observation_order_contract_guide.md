# 部署侧观测顺序与默认值规范

## 1. 背景与适用范围

本文档用于说明当前仓库在部署强化学习策略时，如何解释源数据顺序、如何排列最终观测向量，以及当 IMU / reference motion / 外部传感器缺失时的默认行为。

本文档覆盖以下链路：

- `sim2sim`
- `sim2real`
- reference motion 文件与策略额外输出
- external observations
- ONNX 输入拼接

本文档的目标不是证明“部署侧已经和训练侧对齐”。在未提供训练模型、训练代码、训练时观测导出信息的前提下，部署侧只能做到：

- 以配置文件为唯一真源
- 按配置解释源数据
- 按配置构造观测
- 对关键顺序字段做静态校验
- 在缺少部分输入时保持运行时向量维度合法

对应实现可参考：

- [`rl_cfg.h`](../include/rl_master/rl_cfg.h)
- [`solver_dds_bridge.cpp`](../solver_dds_bridge.cpp)
- [`mujoco_sim_bridge.cpp`](../../../humanoid_sim2sim/mujoco_sim2sim/src/mujoco_sim_bridge.cpp)
- [`observation_builder.cpp`](../observation_builder.cpp)
- [`external_observation_provider.cpp`](../external_observation_provider.cpp)
- [`RL_controller.cpp`](../RL_controller.cpp)
- [`onnx_policy_runner.cpp`](../onnx_policy_runner.cpp)

## 2. 内部统一表示

当前部署链路内部有一套统一表示，外部数据在进入观测构造前会先被整理到这套表示上。

- internal base quaternion 统一为 `xyzw`
- `base_rpy` 统一由内部 `base_quat` 推导
- 当前默认欧拉角顺序为 `roll, pitch, yaw`

这意味着：

- `sim2sim` 和 `sim2real` 可以有不同的源数据格式
- 但进入观测构造阶段后，姿态字段已经被规整到同一套内部语义

其中：

- `sim2sim` 的 MuJoCo free joint 源四元数默认来自 `wxyz`
- `sim2real` 的 IMU 可以按配置解释为 `euler_compat` 或 `quaternion`
- `base_rpy` 在观测中不是直接相信外部输入，而是从内部 `base_quat` 再计算一次

## 3. 哪些顺序放在 `rl_cfg_jc01.yaml`

在当前多文件结构里，应理解为：

- 根 `rl_cfg_jc01.yaml` 负责索引 profile
- 具体策略的 `source_contract` 放在对应 profile 文件里

也就是说，这一节提到的顺序字段，逻辑上属于“部署侧 `rl_cfg` 体系”，物理上通常写在某个 profile 文件中。

这些字段负责定义“源数据解释规则”，即部署程序如何理解外部输入或文件中的原始排列。

应放在 `source_contract` 的字段包括：

- `source_contract.imu_input.euler_order`
- `source_contract.imu_input.quat_order`
- `source_contract.imu_input.ang_vel_order`
- `source_contract.sim_base.quat_source_order`
- `source_contract.reference_file.body_quat_order`
- `source_contract.policy_extra_outputs.body_quat_order`

这些字段的语义如下：

- `imu_input.euler_order`
  说明 IMU 在 `euler_compat` 模式下，`orientation.x/y/z` 分别表示什么轴的欧拉角。
- `imu_input.quat_order`
  说明 IMU 在 `quaternion` 模式下，四元数原始顺序是 `xyzw` 还是 `wxyz`。
- `imu_input.ang_vel_order`
  说明 IMU 原始角速度三个分量的轴顺序。
- `sim_base.quat_source_order`
  说明 `sim2sim` 后端从仿真器读取到的 base quaternion 原始顺序。
- `reference_file.body_quat_order`
  说明 reference motion 文件内 `body_quat_w` 的原始顺序。
- `policy_extra_outputs.body_quat_order`
  说明策略额外输出里的 body quaternion 原始顺序。

这里要特别强调：

- 这些字段定义的是“源数据顺序”
- 它们不直接定义最终 observation vector 的排列顺序
- 最终 observation vector 的排列顺序由 observation manifest 决定

## 4. 哪些顺序放在 observation manifest

observation manifest 负责定义“最终送进策略的观测排列方式”。

可以在 manifest 中控制顺序的典型项包括：

- `base_ang_vel.components`
- `base_euler.components`
- `base_quat.components`
- `reference_joint_pos` / `reference_joint_vel` 对应的 `reference_joint_order`

建议按以下规则理解：

- `base_ang_vel.components`
  决定最终观测里 base angular velocity 的输出顺序
- `base_euler.components`
  决定最终观测里欧拉角的输出顺序
- `base_quat.components`
  决定最终观测里四元数分量的输出顺序
- `reference_joint_order`
  决定 reference joint features 如何从 reference source 重排到运行时 canonical joint order

需要注意的当前实现限制：

- `base_rpy` 是固定顺序项
- 它当前等价于固定输出 `[roll, pitch, yaw]`
- 如果需要“可配置欧拉顺序”，不要继续使用 `base_rpy`
- 应改用 `base_euler` 并显式配置 `components`

## 5. 当前仓库默认行为

当前主 profile `engineai_walk` 与 `stand_sim2real` 使用相同的默认源数据合同：

- `imu_input.payload: euler_compat`
- `imu_input.euler_order: [roll, pitch, yaw]`
- `imu_input.euler_unit: rad`
- `imu_input.quat_order: xyzw`
- `imu_input.ang_vel_order: [x, y, z]`
- `sim_base.quat_source_order: wxyz`
- `reference_file.body_quat_order: wxyz`
- `policy_extra_outputs.body_quat_order: wxyz`

当前两个主策略 manifest 都是 47 维，实际只包含：

- `phase`
- `command`
- `joint_pos`
- `joint_vel`
- `last_action`
- `base_ang_vel`
- `base_rpy`

这意味着当前默认主策略：

- 实际使用 `base_ang_vel + base_rpy`
- 不直接把 IMU quaternion 作为主观测输入
- 没有把 external observations 直接并入这两个默认主策略的 observation manifest

另外：

- `reference_motion` 在这两个默认 profile 中处于关闭状态
- 即使配置里保留了 `reference_motion_dim` 等字段，默认主链路也不会启用 reference motion 特征

## 6. 传感器缺失时的默认值

部署侧会尽量保证“观测向量形状合法”，但这不代表策略语义一定正确。

### 6.1 IMU 未收到时

在 `sim2real` 路径中，如果尚未收到 IMU 消息，内部缓存默认值为：

- `base_ang_vel = [0, 0, 0]`
- `base_quat = [0, 0, 0, 1]`
- `base_rpy = [0, 0, 0]`

这保证了 observation builder 仍然可以构造完整观测。

### 6.2 external observations 缺失时

如果某个 external observation 没有被上游设置：

- runtime 会按照配置的 `dim` 直接补零
- 缺失的 external feature 不会导致 observation 拼接失败

即使配置里写了 `required: true`，当前 runtime 也不会因此强制报错；它仍然会按维度补零。

### 6.3 reference motion 缺失时

当前 reference-related 特征的回退行为是：

- 如果 `reference_motion` 缺失，但 `reference_joint_pos` 与 `reference_joint_vel` 存在，则会尝试打包成 `reference_motion`
- 如果 `reference_motion_dim > 0` 但最终没有拿到 reference motion，则按该维度补零
- `reference_joint_pos` / `reference_joint_vel` 缺失时，最终插入观测时也会按目标维度补零

### 6.4 ONNX 输入缺失时

对于 ONNX 额外输入，要区分两种情况：

- 默认未显式配置 `onnx_inputs` 时
  - 未绑定的模型输入会被作为 constant zero 输入补零
- 显式配置 `onnx_inputs` 且某个输入来源是 `feature` 时
  - 如果 feature 缺失，runtime 会报错

因此，结论应表述为：

- 部署侧通常会保证“能拼出向量并输入 ONNX”
- 但“能运行”不等于“与训练语义一致”

## 7. 推荐配置规范

推荐将部署侧配置职责固定为两层：

- `rl_cfg_jc01.yaml` 负责“源数据解释”
- observation manifest 负责“最终 obs 排列”

建议遵循以下规则：

1. 所有外部源格式差异都放进 `source_contract`
2. 所有最终观测排列差异都放进 manifest
3. 不要把“源顺序”和“最终观测顺序”混在同一个字段里表达
4. 当训练侧要求欧拉角顺序可配置时，使用 `base_euler`
5. 不要把 `base_rpy` 当成可配置欧拉顺序入口

### 7.1 标准 `roll, pitch, yaw` 例子

```yaml
source_contract:
  imu_input:
    payload: "euler_compat"
    euler_order: ["roll", "pitch", "yaw"]
    euler_unit: "rad"
    ang_vel_order: ["x", "y", "z"]

observation_manifest:
  terms:
    - name: base_ang_vel
      enabled: true
      components: [wx, wy, wz]
    - name: base_euler
      enabled: true
      components: [roll, pitch, yaw]
```

### 7.2 自定义欧拉排列例子

```yaml
source_contract:
  imu_input:
    payload: "euler_compat"
    euler_order: ["roll", "pitch", "yaw"]
    euler_unit: "rad"
    ang_vel_order: ["x", "y", "z"]

observation_manifest:
  terms:
    - name: base_euler
      enabled: true
      components: [pitch, roll, yaw]
```

这个例子表达的是：

- 原始 IMU 仍按 `roll, pitch, yaw` 被解释
- 但最终送进策略的欧拉角顺序被排列成 `pitch, roll, yaw`

## 8. 检查清单

当没有训练侧模型时，建议至少做部署侧自洽检查。

### 8.1 Jingchu01 IsaacLab locomotion 特例

`hust_lab` 训练的 Jingchu01 locomotion policy 中，`joint_pos` / `joint_vel` 与 `last_action` / `action` 的关节顺序不同。

`joint_pos` / `joint_vel` 必须按 IsaacLab runtime articulation joint order：

```text
left_hip_roll
right_hip_roll
left_hip_yaw
right_hip_yaw
left_hip_pitch
right_hip_pitch
left_knee_pitch
right_knee_pitch
left_ankle_pitch
right_ankle_pitch
left_ankle_roll
right_ankle_roll
```

`last_action` 和 ONNX 输出 `action` 必须按训练时 action order：

```text
left_hip_roll
left_hip_yaw
left_hip_pitch
left_knee_pitch
left_ankle_pitch
left_ankle_roll
right_hip_roll
right_hip_yaw
right_hip_pitch
right_knee_pitch
right_ankle_pitch
right_ankle_roll
```

部署时不要把 `obs_joint_order` 复制成 `action_joint_order`。这两个顺序混用会让策略输出明显异常。

### 8.2 配置静态检查

重点检查以下几类字段：

- `action_joint_order`
- `obs_joint_order`
- `reference_joint_order`
- `source_contract.*`
- observation manifest 中各项 `count` / `components`
- `policy_io.onnx_inputs`

### 8.3 预检查命令

当前仓库可直接使用以下命令：

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py \
  --rl-cfg src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml \
  --config-section engineai_walk \
  --skip-onnx
```

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py \
  --rl-cfg src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml \
  --config-section stand_sim2real \
  --skip-onnx
```

### 8.4 手工核对项

建议手工按以下顺序检查：

1. joint order
   - `action_joint_order`
   - `obs_joint_order`
   - `reference_joint_order`
2. source contract
   - IMU 欧拉角 / 四元数 / 角速度顺序
   - sim base quaternion 原始顺序
   - reference file / policy extra outputs quaternion 原始顺序
3. manifest layout
   - 观测项顺序
   - `components`
   - `count`
4. ONNX input binding
   - 默认 obs 输入名
   - 是否启用额外 `onnx_inputs`
   - `feature` 类型输入在缺失时是否允许补零

## 9. 已知限制

当前实现存在以下边界：

- 未提供训练模型、训练代码、训练导出 metadata 时，部署侧无法证明与训练侧语义一致
- `base_rpy` 当前是固定顺序项，不是可配置顺序项
- `required=true` 的 external observation 当前不会在 runtime 强制报错，仍会补零
- 观测维度合法并不保证策略输出合理
- `sim2sim` 当前 MuJoCo base quaternion 路径虽然配置允许写 `xyzw` / `wxyz`，但实现实际要求 MuJoCo free-joint 源顺序为 `wxyz`

## 10. 结论

在当前仓库中，最稳妥的部署原则应固定为：

- 配置文件是部署侧顺序合同
- `source_contract` 负责解释源数据
- observation manifest 负责定义最终观测排列
- runtime 负责按配置转换、校验、补齐

在未提供训练侧模型时，部署侧不应声称“已经与训练完全对齐”，而应只声称：

- 当前部署侧按配置解释输入
- 当前部署侧内部表示一致
- 当前部署侧在缺失输入时有明确默认行为
- 当前部署侧可以通过静态检查和运行时输入绑定检查确认自洽
