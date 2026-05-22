# 多文件 `rl_cfg` 配置指南

## 1. 这次拆分后的核心原则

当前仓库已经把原来的单文件 `rl_cfg_jc01.yaml` 拆成了：

- 一个根索引文件：[`config/rl_cfg_jc01.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml)
- 多个按策略拆分的 profile 文件：[`config/profiles/engineai_walk.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/profiles/engineai_walk.yaml)、[`config/profiles/stand_sim2real.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/profiles/stand_sim2real.yaml) 等

设计目标很简单：

- 根文件只放全局索引和运行时公共配置
- 每个 `config_section` 单独落到一个 profile 文件
- 运行时仍然只从 `RL_CFG_PATH` 启动，但会继续按 `config_files` 找到具体 profile

## 2. 根文件和 profile 文件各自负责什么

### 2.1 根 `rl_cfg_jc01.yaml`

根文件负责：

- `humanoid_rl_root_dir`
- `legged_gym_root_dir`
- `robot_global_joint_order`
- `joint_groups`
- `deploy_mode_profiles`
- `config_files`
- `runtime_process`
- `logging`

可以把它理解成“根索引 + 全局合同”。

其中最关键的两块是：

- `deploy_mode_profiles`
  它定义 `mode_id -> config_section -> tag`
- `config_files`
  它定义 `config_section -> profile 文件路径`

现在还新增了一层关节组合同：

- `robot_global_joint_order`
  它不仅是全局 runtime joint 顺序，也同时作为 solver installed joints 顺序和 SHM 槽位顺序
- `joint_groups.leg / arm / waist`
  它们是在这个全局顺序里切出来的显式子集合同，供 `KinConv` 和 solver 做分组转换

当前实现里：

- `joint_groups.leg` 必须完整给出当前 12 个下肢关节，并保持既定局部顺序
- `joint_groups.arm` 和 `joint_groups.waist` 可以为空
- arm / waist 当前只是接口预留，默认走直通

### 2.2 每个 profile 文件

每个 profile 文件只负责一个具体策略实例的静态配置，例如：

- policy 路径
- observation/action 维度
- joint order
- observation manifest
- `policy_io`
- `source_contract`
- `sub_models`
- `external_observations`
- `reference_motion`
- `startup_completion_action`
- `robot`
- `scales`

每个 profile 文件必须满足两条规则：

1. 只包含一个顶层 section
2. 顶层 section 名必须和 `config_section` 完全一致

## 3. `mode / config_section / ModeProfile / policy_group` 怎么理解

建议固定成下面这套理解：

- 一个 `mode` 是一个具体可切换、可启动的部署策略实例
- 一个 `config_section` 是这个策略实例的静态配置名
- 一个 `ModeProfile` 是运行时把这个 `config_section` 解析后的实例对象
- 一个 `policy_group` 是这个 `mode` 内部实际参与推理的模型集合

重点是下面这件事：

- 多个 `mode` 可以结构相似
- 多个 `mode` 也可以拥有相同的 `policy_family`
- 但只有同 `policy_family` 且运行时合同兼容的 mode，才允许热切
- 但它们不会共享同一个 runtime `policy_group` 实例

换句话说：

- “同一种策略形态” 只是结构上相似
- 不等于“共用同一份运行时 group 对象”

当前实现里，仍然是每个 `mode` 各自构造自己的 `ModeProfile`、自己的 `policy_group`。

另外要特别注意，`policy_family` 现在不再只是描述性标签：

- 它同时也是热切兼容分组
- 同 `policy_family` 只是“有资格热切”的第一层条件
- 真正是否允许热切，还要继续经过 `action_joint_order`、`installed_joint_run_modes`、`obs_joint_order`、`observation_manifest_path` 等合同一致性检查

## 4. 目录结构示例

```text
config/
├── rl_cfg_jc01.yaml
└── profiles/
    ├── engineai_walk.yaml
    ├── stand_sim2real.yaml
    └── engineai_full_body_example.yaml
```

推荐继续沿用“一个 profile 文件对应一个 `config_section`”的粒度，不要再往下拆成更细碎的 `policy_io.yaml`、`robot.yaml`、`source_contract.yaml`。

当前仓库里的 [`engineai_full_body_example.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/profiles/engineai_full_body_example.yaml) 更适合作为扩展示例：

- 它已经按多文件结构拆出
- 但如果要真正激活成可部署 mode，还需要同步扩展 `robot_global_joint_order` 等全局合同

## 5. 新增一个 mode 的标准步骤

1. 在 `config/profiles/` 下新增一个 profile 文件。
2. 在这个文件里写一个新的顶层 `config_section`。
3. 在根 [`rl_cfg_jc01.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml) 的 `config_files` 中注册这个 `config_section`。
4. 在根 [`rl_cfg_jc01.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml) 的 `deploy_mode_profiles` 中注册新的 `mode_id`。
5. 补齐这个 profile 的 observation manifest、policy 文件、joint order、source contract 等字段。

最常见的写法是：

```yaml
deploy_mode_profiles:
  - mode_id: 2
    config_section: walk_with_vision
    tag: walk_vision

config_files:
  walk_with_vision: profiles/walk_with_vision.yaml
```

然后在 `profiles/walk_with_vision.yaml` 中写：

```yaml
walk_with_vision:
  policy_name: my_policy
  obs_dim: 128
  action_dim: 12
  ...
```

## 6. 如果两个 mode 结构很像，该怎么写

如果两个 mode 都是“主策略 + 若干子模型 + 同样的外部传感输入”：

- 可以让它们保持相同的结构风格
- 也可以让它们拥有相同的 `policy_family`
- 但仍然建议写成两个独立 profile 文件

如果这两个 mode 希望支持运行中直接热切，还应尽量保持下面这些合同一致：

- `action_joint_order`
- `installed_joint_run_modes`
- `obs_joint_order`
- `reference_joint_order`
- `observation_manifest_file` / `observation_manifest_path`
- `control_mode`

推荐这样做的原因是：

- 两个 mode 往往会逐渐分叉出不同的 joint order、manifest、权重、默认姿态或 source contract
- 运行时本来就是按 `mode_id` 各自构造 profile
- 把每个 mode 保持为独立 profile，排查问题时最清晰

所以工程上最稳妥的规则仍然是：

- 一个可部署 mode
- 对应一个独立 `config_section`
- 对应一个独立 profile 文件

## 7. 当前加载语义

当前运行时加载顺序是：

1. 先读取根 [`rl_cfg_jc01.yaml`](/home/edify/Code/jc01_deploy/src/humanoid_rl_controller/rl_master/config/rl_cfg_jc01.yaml)
2. 读取 `deploy_mode_profiles`
3. 读取 `config_files`
4. 按 `config_section` 找到对应 profile 文件
5. 解析这个 profile 文件中的唯一同名 section

对于路径解析：

- `config_files` 的相对路径，相对根 `rl_cfg_jc01.yaml` 所在目录
- profile 文件内部的 `policy_file`、`observation_manifest_file`、`reference_motion_file` 等相对路径，仍然相对 `humanoid_rl_root_dir`

## 8. 常见错误

预检查和运行时都会重点报这几类错误：

- `config_files` 缺失某个 `config_section` 的映射
- profile 文件不存在
- 一个 profile 文件里包含多个顶层 section
- profile 文件的顶层 section 名和 `config_section` 不一致
- `deploy_mode_profiles` 中 `mode_id` 重复

## 9. 预检查命令

建议在新增或拆分配置后先跑：

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

如果只想点查一个未激活的 profile，也可以直接传 `--config-section <name>`，前提是它已经在 `config_files` 里注册。
