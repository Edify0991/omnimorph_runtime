# Policy Runtime Architecture

这份文档专门说明最近在当前部署仓库中新增的运行时设计框架，以及为什么我们把架构继续往“共享核心 + 显式合同 + 可扩展策略适配层”这个方向推进。

它重点回答四个问题：

1. 为什么不继续为每一类算法单独堆一套部署状态机
2. 当前 `rl_master` 和 `mujoco_sim2sim` 各自新增了什么分层
3. 现在已经支持到什么程度，合同边界在哪里
4. 后续如果接 diffusion / flow-matching / chunked policy，应该接到哪一层

## 1. 设计目标

当前仓库服务的不是单一策略，而是一个逐步扩展的部署框架。随着策略类型增加，最容易失控的地方通常有三个：

- 不同算法各自复制一套 deploy 流程，状态机和执行链越来越分叉
- joint 顺序、action 顺序、reference 顺序在不同模块里各自“猜测”或隐式 fallback
- 为了兼容短期需求，不断在运行时里插入特判，后面排错越来越困难

这次扩增后的设计目标可以概括为三条：

- 用共享运行时核心承接大多数策略，而不是按算法家族复制整条部署链
- 用显式配置合同替代隐式 fallback，让 joint 空间、输入输出空间都可检查
- 把“模型后端”和“执行策略”拆开，让未来新增生成式策略时尽量复用已有控制主链

## 2. 核心原则

### 2.1 唯一真值优先

在机器人部署里，最容易出错的是“同一个含义在多个地方各自定义一份”。当前我们继续强化的方向是：

- `robot_global_joint_order` / `ModeProfileRegistry::jointOrder()` 是 installed joints 的唯一真值集合
- `action_joint_order`、`obs_joint_order`、`reference_joint_order` 都是这个全集上的显式子空间或重排
- sim2sim 后端也尽量从共享 registry / 显式 joint names 读取，不再依赖隐式默认顺序

换句话说，运行时允许“不同模块使用不同顺序”，但不允许“不同模块各自发明一个 joint 集合”。

### 2.2 显式合同优先

现在的方向不是“让系统尽量自己猜”，而是“如果合同不成立就尽早报错”。

这体现在：

- mode profile 必须能从 `deploy_mode_profiles` 正确解析
- `policy_adapter`、`inference_strategy`、`action_output_layout` 都有显式校验
- chunked policy 的输出维度合同必须与 `action_chunk_steps` 对应
- sim2sim 中越来越多旧的默认 joint fallback 被清掉，避免带着错误配置继续往下运行

### 2.3 算法形态与运行时机制解耦

“算法是什么”与“部署时怎么执行它”并不总是一回事。

例如：

- 一个策略可以仍然是 ONNX 后端，但输出是单步 action
- 另一个策略也可以是 ONNX 后端，但输出是一段 action chunk
- 再往后，diffusion / flow-matching 可能仍然走相同 observation 管线，但推理调度完全不同

所以当前拆出了两层抽象：

- `PolicyAdapter`：负责“怎么和模型后端打交道”
- `PolicyInferenceStrategy`：负责“怎么在每个控制 tick 使用模型输出”

这就是后续扩展的主轴。

## 3. 当前总体分层

可以把当前架构理解为下面四层：

```text
Mode / Config Registry
    -> ModeProfile / Observation / Reference mapping
        -> PolicyAdapter
            -> PolicyInferenceStrategy
                -> Robot command assembly / backend execution
```

### 3.1 配置与 mode 层

这一层仍然由以下对象承接：

- `ModeProfileRegistry`
- `DeployModeProfileSpec`
- `Sim2realCfg`
- `ModeProfile`

它负责回答：

- 当前有哪些可切换 mode
- 每个 mode 对应哪一段 `config_section`
- 这个 mode 的 obs/action/reference/joint 合同是什么
- 该 mode 需要什么 observation manifest、reference source、policy file

这一层的重点不是“做推理”，而是把部署合同先固化下来。

### 3.2 观测与特征层

这一层依然是：

- `ObservationBuilder`
- `ObservationFeatureContext`
- reference / external observation / motion anchor 等特征准备逻辑

这一层的职责是：

- 根据当前 `ModeProfile` 把 runtime state 变成 observation
- 维护 observation term 的拼接顺序合同
- 为 policy 输入和 extra outputs 提供统一 feature 容器

这层并不关心“后面是 ONNX、diffusion 还是别的后端”，它只关心把当前 mode 需要的输入数据准备对。

### 3.3 `PolicyAdapter` 层

新增的核心抽象是：

- `PolicyAdapter`
- `OnnxPolicyAdapter`

它的职责是把“具体模型后端”包装成统一接口：

- `init()`
- `reset()`
- `infer(request)`
- `prefetchExtraOutputs(...)`
- `summary()`

当前已经落地的实现只有：

- `OnnxPolicyAdapter`

它内部仍复用：

- `OnnxPolicyRunner`

但对上层 `RL_controller` 来说，已经不再需要直接依赖某个具体 runner 的细节。

这一步的意义是：

- 如果后续还是 ONNX，只是输入输出合同变化，不一定要动 controller 主流程
- 如果后续换成别的模型执行后端，也可以优先在 adapter 层做隔离

### 3.4 `PolicyInferenceStrategy` 层

第二个新增抽象是：

- `PolicyInferenceStrategy`

它负责定义“这个 policy group 在每个控制周期怎么执行”。

当前已有两个实现：

- `SyncWeightedInferenceStrategy`
- `ChunkedRecedingInferenceStrategy`

它们和 adapter 的关系可以理解成：

- adapter 决定“如何拿到一次模型输出”
- strategy 决定“拿到输出后，这一拍具体如何消费它”

这比“把所有策略类型都揉进 `RL_controller::run_policy()` 里写分支”更稳定，也更容易继续加新形态。

## 4. 当前已经落地的策略运行时能力

### 4.1 单步同步策略

`SyncWeightedInferenceStrategy` 对应当前最常见的部署形态：

- 每个控制 tick 推一次主策略
- 如果有 sub-model，可按权重融合 action
- 主策略 extra outputs 继续走现有链路

这基本对应原来的单步 RL/imitiation policy 形态，只是现在被显式抽成了 strategy。

### 4.2 Chunked receding 策略

`ChunkedRecedingInferenceStrategy` 是这次扩增中最关键的新能力之一。

它面向的不是“单步输出 action”的策略，而是“一次输出多个未来 action”的策略，例如：

- action chunk policy
- 某些生成式策略的短时 rollout 输出
- 未来可能的 flow-matching / diffusion 采样结果切片

当前合同如下：

- `inference_strategy: chunked_receding`
- `action_output_layout: chunk_flat`
- `action_chunk_steps > 1`
- `action_chunk_execute_steps > 0`
- `action_chunk_replan_interval > 0`

当前实现假设：

- 模型输出是一维 flatten 的 chunk
- 排布方式是 step-major
- 每次推理后缓存若干 step 的 pending actions
- runtime 只执行前 `action_chunk_execute_steps` 步
- 达到 `action_chunk_replan_interval` 后重新推理并覆盖后续执行

这不是最终形态，但已经把“单步 policy”和“多步 chunk policy”在运行时层面正式区分开了。

### 4.3 `last_action` 输入源

`OnnxInputSpec.source` 现在新增支持：

- `last_action`

这意味着后续遇到依赖前一拍 action 的策略时，不需要在 controller 里再手写一套私有输入拼接逻辑，而是可以继续沿用统一的 ONNX 输入绑定合同。

### 4.4 Policy prefetch 仍然保留

当前 strategy 接口同时保留了：

- `prefetchPrimaryExtraOutputs(...)`

这是为了兼容那些需要先拿 reference / extra outputs，再参与当前拍 observation 构造的策略路径。

目前这块仍然建立在现有 ONNX extra outputs 合同之上，但已经从“controller 直接知道 ONNX runner 细节”转成“controller 通过 strategy / adapter 请求需要的 extra outputs”。

## 5. Sim2sim 执行层这次同步收敛的设计

除了 `rl_master` 里的 policy runtime 抽象，这次在 `mujoco_sim2sim` 侧也做了一轮重要收敛，核心是把执行语义重新围绕 canonical joints 组织。

### 5.1 hold 参数现在覆盖全部 canonical joints

当前方向是：

- `hold_kp`
- `hold_kd`
- `hold_torque_limit`
- `hold_joint_target_q`

这些量描述的是“所有 installed canonical joints 在 hold 语义下应该如何处理”。

它们不再依赖一套单独的 `hold_joint_names_ / hold_actuator_names_` 语义层。

### 5.2 RUNNING 与 HOLD 的职责更清楚

当前语义收敛为：

- 在 `RUNNING` 中，policy action joints 走策略输出
- 在 `RUNNING` 中，非 action joints 走 `hold_*`
- 在 `HOLD` 或 inactive 中，全部 canonical joints 走 `hold_*`

也就是说，`hold_*` 不再只是“单独一组备用关节”的参数，而是后端执行层对非策略控制部分和 hold 场景的统一兜底执行合同。

### 5.3 mixed actuator 的语义也更显式

这次还把 mixed 模式从旧的 hold 语义中拆开，改成用：

- `position_controlled_joint_names`

来显式表示哪些 joint 在 mixed 执行中使用位置型通道。

这样后端不再通过旧的 hold 相关字段“间接猜测” actuator 语义，排错会更直接。

## 6. 为什么不推荐“每个算法一套部署状态机”

这是这次设计里最关键的取舍之一。

短期看，为某个新算法单独复制一套部署状态机似乎更快；但一旦策略类型增多，它会带来几个直接问题：

- 观测拼接和 joint 映射逻辑会复制多份，后面修 bug 很难保证全修到
- mode 切换、hold、startup、reference 预取等共性逻辑会逐渐分叉
- sim2real 和 sim2sim 两侧更容易在行为上漂移
- 新增日志、调试点、保护机制时，需要在多条分支上同步维护

相比之下，当前更推荐的方向是：

- 用一个共享 deploy state machine
- 用一个共享 observation / command 主链
- 用 adapter 表示“模型后端差异”
- 用 strategy 表示“执行调度差异”

这样大多数算法差异都能落在局部扩展点，而不是污染整条部署主流程。

## 7. 后续新增策略时，优先接到哪一层

可以按下面这个判断顺序来决定。

### 7.1 只是新权重 / 新 manifest / 新 joint order

这种情况通常只需要：

- 新增 profile
- 新增或调整 observation manifest
- 校对 obs/action/reference joint order

一般不需要新增 adapter，也不需要新增 strategy。

### 7.2 还是 ONNX，但输出语义不同

例如：

- 输出 action chunk
- 额外依赖 `last_action`
- 需要特殊 extra outputs 合同

优先做法是：

- 先看是否只需要扩展 `Sim2realCfg` 合同
- 必要时扩 `OnnxPolicyAdapter` / `OnnxPolicyRunner`
- 尽量不要直接把分支塞回 `RL_controller`

### 7.3 模型后端不同，但控制调度相似

例如未来如果不是 ONNX，而是另一个模型执行器，但仍然是一拍一拍地产生 action，那么优先新增：

- 新的 `PolicyAdapter`

而不是先复制一套 controller 主流程。

### 7.4 控制调度本身不同

例如：

- diffusion 每拍要做多次 denoise
- flow-matching 需要短时 rollout / refine
- MPC 风格要周期性重规划

这种情况优先新增：

- 新的 `PolicyInferenceStrategy`

必要时再配合新的 adapter。

## 8. 当前还没做完的部分

这次只是把框架骨架先搭稳，还不是所有形态都已经完成。

当前仍然明确未完成或仅做了第一版的包括：

- 真正面向 diffusion / flow-matching 的 adapter
- 更严格的 chunk layout 元数据合同
- async / dual-rate inference strategy
- 更丰富的 strategy 级 runtime logging
- 更系统的 policy metadata 自检与 profile 一致性验证

所以当前可以把它理解为：

- 主运行时的扩展点已经明确
- 第一批单步和 chunked 策略合同已经落地
- 但生成式策略的最终部署形态还需要继续沿这套抽象补实现

## 9. 与现有文档的关系

建议配合下面几份文档一起看：

- [runtime_end_to_end_function_flow.md](${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/docs/runtime_end_to_end_function_flow.md)
  讲当前 runtime 调用链和 mode 切换时序
- [policy_deploy_and_config_guide.md](./policy_deploy_and_config_guide.md)
  讲 mode / config_section / ModeProfile / policy_group 的配置映射关系
- [deploy_observation_order_contract_guide.md](${OMNIMORPH_RUNTIME_ROOT}/src/omnimorph_rl_controller/rl_master/docs/deploy_observation_order_contract_guide.md)
  讲 observation 顺序合同与 manifest 校对方法

如果只想先抓住这次扩增后的主思路，可以优先记住一句话：

- 共享主链不动，把差异尽量收敛到“显式配置合同 + adapter + strategy”这三层里。
