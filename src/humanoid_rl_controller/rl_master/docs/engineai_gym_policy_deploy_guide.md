# EngineAI Gym Policy Deploy Guide

This guide describes how to take a policy trained in an EngineAI Gym-style RL framework and deploy it through this repository's fused runtime.

## 0. Mode / Policy Group / Config 的关系

这一节专门回答一个最容易混淆的问题：`mode`、`config_section`、`ModeProfile`、`policy_group`、`policy_family` 在当前代码里分别代表什么。

### 0.1 核心结论

可以先记住这一句话：

- `mode` 是实际可切换、可启动的单个部署策略实例
- `policy_group` 是某个 `mode` 内部持有的一组推理运行时组件
- 多个 `mode` 可以属于同一种策略形态，也可以拥有相同的 `policy_family`
- 但当前实现里，每个 `mode` 仍然会构造自己的 `ModeProfile` 和自己的 `policy_group`

也就是说，当前仓库里并不存在一个“独立的、可被多个 mode 共享实例”的顶层 `policy_group` 配置层。

### 0.2 概念映射

#### `deploy_mode_profiles`

它负责把：

- `mode_id`
- `config_section`
- `tag`

这三者对应起来。

其中最重要的是：

- `mode_id` 是状态机和外部控制字真正切换的对象
- `config_section` 是 `rl_cfg.yaml` 中某一段具体策略配置

所以从运行时角度看，一个 `mode` 最终就是“通过 `mode_id` 选中某个 `config_section`，然后把它构造成运行时 profile”。

#### `config_section`

`config_section` 就是 `rl_cfg.yaml` 中的一段完整部署配置，例如某个 walk policy、stand policy、mimic policy 或 full-body policy。

这一段里会定义：

- policy 路径
- observation/action 维度
- `action_joint_order`
- `obs_joint_order`
- observation manifest
- 是否启用 `sub_models`
- 是否启用 `amp_discriminator`
- 是否声明 `external_observations`
- 是否启用 `reference_motion`

所以新增一个策略，通常不是只改一个字段，而是新增一个新的 `config_section`。

#### `ModeProfile`

`ModeProfile` 是每个 `mode` 在内存中的完整运行时载体。

它不是 YAML 里的原始段落，而是代码在启动时根据 `config_section` 解析、构造出来的运行时对象。当前它至少包含：

- 该 mode 对应的 `cfg`
- `joint_names`
- `default_angle`
- action/observation/reference 的 joint 映射
- `observation_manifest`
- `observation_builder`
- `policy_group`
- `amp_discriminator`
- `reference_motion`

所以更准确地说：

- `config_section` 是静态配置定义
- `ModeProfile` 是这段配置在运行时的实例化结果

#### `policy_group`

`policy_group` 只表示“这个 mode 运行时要一起参与推理的模型集合”。

当前代码里，它至少包含：

- 1 个主策略 ONNX
- 0 到多个 `sub_models`

也就是说，`policy_group` 更接近“一个 mode 的推理执行组”，而不是“仓库中独立的一类配置对象”。

请特别注意两点：

- 它不是 `rl_cfg.yaml` 里的顶层独立层级
- 它不是跨多个 mode 共享的运行时实例

如果两个 mode 都是“主策略 + 2 个子模型”的结构，那么它们可以说“结构相似”，但在当前实现里仍然会各自创建自己的 `policy_group`。

#### `policy_family`

`policy_family` 更像是策略家族或语义标签，用来表达这个策略大致属于哪一类，比如：

- `amp`
- `beyondmimic`
- `velloco`
- `custom`

它的作用主要是表达“这是什么类型的策略”，而不是自动决定整个运行时结构。

当前真正决定运行时结构的，仍然是该 `mode` 配置里是否启用了这些能力：

- `sub_models`
- `amp_discriminator`
- `external_observations`
- `reference_motion`
- 对应的 observation manifest

所以不要把 `policy_family` 理解成“结构开关总控”。

### 0.3 推荐理解方式 / 非推荐理解方式

推荐理解方式：

- 一个 `mode` = 一个具体可部署、可切换的单策略配置实例
- 多个 `mode` 可以属于同一种策略形态
- 多个 `mode` 也可以拥有相同的 `policy_family`
- 这些 mode 可以有相似的 `policy_group` 结构

可以半抽象地理解为：

- 一类策略形态可能共享相同结构，例如：
  - 单主策略
  - 主策略 + 子模型
  - 带外部观测输入
  - 带 reference motion

但不推荐这样理解：

- “一个 `policy_group` 是仓库里的独立配置层级”
- “多个 mode 真正复用同一个 `policy_group` 实例”

当前更准确的说法是：

- 多个 mode 可能拥有相同的 group 结构
- 但代码里仍然是每个 mode 各自构造自己的 `ModeProfile` 和自己的 `policy_group`

## 1. Target Runtime

### Sim2Sim

Use the fused C++ MuJoCo runtime:

```bash
./script/sim2sim_engineai.sh --model-path /abs/path/to/robot.xml --mode-id 0
```

### Sim2Real

Use the fused real-robot runtime:

```bash
./script/sim2real_engineai.sh --mode-id 0
```

## 2. Prepare the Policy

Place the exported ONNX file under:

```text
src/humanoid_rl_controller/rl_master/policies/
```

Example:

```text
src/humanoid_rl_controller/rl_master/policies/engineai_walk.onnx
```

## 3. Prepare the Config

Edit `src/humanoid_rl_controller/rl_master/config/rl_cfg.yaml`.

### 3.1 以后新增策略时，推荐按这个顺序写配置

1. 先确定是否需要一个新的 `mode_id`
2. 为这个策略新增一个新的 `config_section`
3. 在 `deploy_mode_profiles` 中注册 `mode_id -> config_section -> tag`
4. 在该 section 中补齐该 mode 的完整部署信息
5. 为该 mode 配套 observation manifest

其中第 4 步至少要关注这些字段：

- 顶层 `robot_global_joint_order`（必配，用来显式固定整个 runtime 的全局 joint 空间）
- `policy_file` 或 `policy_path`
- `obs_dim`
- `action_dim`
- `action_joint_order`
- `obs_joint_order`
- `observation_manifest_file` / `observation_manifest_path`
- `policy_io.obs_input_name`
- `policy_io.action_output_name`
- 是否需要 `time_step` 输入
- `robot.default_joint_angles`
- `robot.zero_joint_angles`
- 是否有 `sub_models`
- 是否启用 `amp_discriminator`
- 是否声明 `external_observations`
- 是否启用 `reference_motion`

如果只是“同类策略的新权重、新观测排列、新 action 顺序”，当前仍然建议：

- 新建一个新的 `config_section`
- 再给它分配一个新的 `mode_id`

而不是试图在运行时复用旧 section 再做临时魔改。

### 3.2 为什么建议“一个 mode 对应一个 section”

因为当前运行时是围绕 `ModeProfile` 组织的，而 `ModeProfile` 是按 `mode_id` 逐个构造的。

这意味着一个 mode 实际上不只是一个 ONNX 文件路径，它还绑定了：

- joint 顺序
- obs/action 维度
- observation manifest
- 外部观测声明
- reference motion 设定
- 子模型组合
- 默认姿态与关节映射

所以最稳妥的工程化做法，就是把“一个可部署策略实例”收口成“一个独立 section + 一个独立 mode_id”。

### 3.3 `default_angle` 和 `zero_pose` 现在如何配置

当前推荐配置方式是：

- 顶层 `robot_global_joint_order`（必配）
- `robot.default_joint_angles`
- `robot.zero_joint_angles`

两者都按 joint name 写，不再使用旧的顶层顺序向量 `zero_pose: [ ... ]`。

语义区别是：

- `default_angle`：策略工作的参考姿态
- `zero_pose`：状态机执行 zeroing 时的目标姿态

对应当前代码行为：

- runtime 会使用顶层 `robot_global_joint_order` 作为全局 joint 顺序
- 如果没有配置 `robot.zero_joint_angles`，zeroing 会回退到 `default_angle`
- 如果配置了 `robot.zero_joint_angles`，就必须完整覆盖该 mode 的运行时 joint 集合

这样做的目的，是避免全身模式下只写下半身时，把未写关节错误补成 `0.0`。

### 3.4 典型设计示例

#### 例子 A: `mode A`，纯 locomotion 主策略

特点：

- 只有一个主策略 ONNX
- 没有 `sub_models`
- 没有 `external_observations`
- 没有 `reference_motion`

这类 mode 最简单，适合最基础的 locomotion policy。

#### 例子 B: `mode B`，与 A 同 family，但 joint/order 不同

特点：

- `policy_family` 和 A 一样
- 仍然只有一个主策略
- 但 `action_joint_order`、`obs_joint_order`、`obs_dim`、`action_dim` 可能不同

这种情况下，B 仍然应该是一个独立 mode，而不是 A 的“变体开关”。

换句话说：

- “同 family” 不代表“同 mode”

#### 例子 C: `mode C`，与 A 同 family，但额外启用子模型或外部观测

特点：

- `policy_family` 可以仍然和 A 相同
- 但在该 section 中新增 `sub_models`
- 或增加 `external_observations`
- 或启用 `reference_motion`

这种 mode 和 A 可以属于同一类策略家族，但它已经是另一个独立部署实例。

所以也应该：

- 新增一个新的 `config_section`
- 在 `deploy_mode_profiles` 中挂一个新的 `mode_id`

这也意味着：

- “同结构” 不代表“共享同一个 `policy_group` 实例”
- “新增策略” 的标准做法仍然是“新增 `config_section` + 注册新的 `mode_id`”

For the target mode profile, confirm:

- `policy_file` or `policy_path`
- `obs_dim`
- `action_dim`
- `action_joint_order`
- `obs_joint_order`
- `observation_manifest_file` / `observation_manifest_path`
- `policy_io.obs_input_name`
- `policy_io.action_output_name`
- any required `time_step` input settings

## 4. Prepare the Observation Manifest

Create or update the manifest under:

```text
src/humanoid_rl_controller/rl_master/config/
```

Typical pattern:

```text
observation_manifest_engineai_walk.yaml
```

Then point the mode config to that manifest.

## 5. Run the Offline Precheck

```bash
python3 src/humanoid_rl_controller/rl_master/tools/analysis/validate_deploy_config.py --mode-id 0
```

This checks at least:

- profile selection
- manifest parsing
- observation dimension consistency
- joint-order consistency
- ONNX presence and basic I/O compatibility
- optional metadata checks if enabled

## 6. Sim2Sim Workflow

1. confirm the XML model path
2. run precheck for the mode
3. launch fused sim2sim runtime
4. publish `walk_mode` start word or use `--auto-start-mode`
5. inspect viewer motion, joint behavior, and `/humanoid/rl/state`

Example:

```bash
./script/sim2sim_engineai.sh \
  --model-path /abs/path/to/robot.xml \
  --mode-id 0 \
  --enable-viewer true \
  --auto-start-mode
```

## 7. Sim2Real Workflow

1. bring up motor driver
2. bring up IMU DDS publisher
3. run fused real runtime
4. publish `walk_mode` start word
5. use joystick / teleop DDS as needed

Example:

```bash
sudo ./script/driver.sh
sudo ./script/imu.sh
./script/sim2real_engineai.sh --mode-id 0 --auto-start-mode
sudo python3 ./script/joyLaunch.py
```

## 8. How Sim2Sim and Sim2Real Stay Aligned

The two paths now share the same embedded controller runtime:

- same `RL_controller::step(...)`
- same deploy state machine
- same observation manifest parsing
- same policy switching logic
- same ONNX inference stack

The environment-specific part is only:

- where state comes from
- where commands are applied

On the real-robot fused path, deploy-mode config is loaded once through a shared `ModeProfileRegistry`, then reused by both solver-side execution logic and controller-side policy logic.

On the current MuJoCo fused path, the same mode/profile definitions are still used, but the registry is lazily created inside controller initialization if it was not injected earlier.

That means sim2sim is now a much closer validation target for the real deploy path.

## 9. Choosing Between C++ and Python Sim2Sim Backends

Use `sim2sim_engineai.sh` when you want the closest validation target to the real fused runtime.

Use `sim2sim_engineai_python.sh` when you want the friendlier Python MuJoCo GUI while still keeping the fused C++ control/physics runtime:

```text
C++ fused backend (physics + controller) -> ROS2 frame stream -> Python MuJoCo viewer frontend
```

Use `sim2sim_engineai_python_legacy.sh` only when you need the historical split runtime for comparison:

```text
RL_controller (legacy standalone process) <-> DDS <-> python_interactive backend
```

## 10. 配置这份文档时，建议记住的最小心智模型

如果你后面继续给仓库加新策略，最稳妥的最小心智模型可以概括成下面 5 句话：

- 一个 `mode_id` 对应一个具体可切换的部署策略实例
- 一个 `config_section` 定义这个策略实例的静态配置
- 一个 `ModeProfile` 是这段配置在运行时的实例化结果
- 一个 `policy_group` 只是这个 mode 内部实际参与推理的模型集合
- 一个 `policy_family` 只是语义分类标签，不是完整结构开关

按这个模型去写 `rl_cfg.yaml`，一般就不会再把 `mode`、`group`、`family` 这几层混淆。
