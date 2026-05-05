# Base Linear Velocity Kalman Filter Guide

本文档总结当前 `rl_master` 框架中用于估计机器人基座线速度 `base_lin_vel` 的卡尔曼滤波器设计、使用方式、参数含义、调参流程，以及在真实机器人上验证结果是否正确的方法。

## 1. 目标

很多 IsaacLab / BeyondMimic / AMP 策略的 observation 中包含：

```yaml
- name: base_lin_vel
  count: 3
  components: [x, y, z]
```

训练中这个量通常来自仿真的 base linear velocity，真实机器人上不能直接得到完全可靠的基座线速度。因此当前框架提供了一个轻量的 3 轴速度卡尔曼滤波器，把以下信息融合成最终喂给策略的 `robot.base_lin_vel`：

- IMU 线加速度积分预测
- 外部输入的线速度测量，例如 GLIM / odom
- 静止检测触发的 zero velocity update
- 如果关闭滤波器，则直接使用输入状态中的 `base_lin_vel`

最终 observation builder 读取的是 `robot.base_lin_vel`，见 `observation_builder.cpp` 中的 `base_lin_vel` term。

## 2. 代码位置

核心实现：

- `include/rl_master/filters/base_velocity_kalman_filter.h`
- `filters/base_velocity_kalman_filter.cpp`

控制器接入：

- `RL_controller::estimateBaseLinearVelocity`
- `RL_controller::updateStateFromIO`

配置定义：

- `include/rl_master/rl_cfg.h`
- profile YAML 中的 `source_contract.base_velocity_estimator`

ROS odom 接入：

- `solver_dds_bridge.cpp`
- 当 `use_odom_velocity_measurement: true` 时订阅 `odom_topic`

## 3. 滤波器模型

当前实现是一个 3 维解耦的一阶速度滤波器。状态只有世界系基座线速度：

```text
x = [vx, vy, vz]
```

每个轴独立维护一个方差：

```text
P = [Px, Py, Pz]
```

### 3.1 Prediction

当 IMU 线加速度有效并启用 `use_imu_prediction` 时：

```text
v_k = v_{k-1} + a_world * dt
P_k = P_{k-1} + process_noise * dt + accel_noise * dt^2
```

如果 IMU 加速度来自 body frame：

```text
a_world = R_world_body * a_body
```

如果 IMU 加速度包含重力：

```text
a_world.z -= gravity_mps2
```

如果 IMU 预测关闭或加速度无效，则使用零加速度做 prediction，只增长协方差，不积分速度。

### 3.2 Velocity Measurement Update

当有速度测量时，做标准一维 Kalman update：

```text
K = P / (P + R)
v = v + K * (z - v)
P = (1 - K) * P
```

其中 `R` 对应配置中的 measurement noise。

当前支持两类速度测量：

- `input_velocity_measurement`: 输入状态 `state.base_lin_vel`
- `odom_velocity_measurement`: DDS bridge 从 ROS `nav_msgs/msg/Odometry` 读到的 twist linear velocity，然后写入 `state.base_lin_vel`

注意：`RL_controller` 中目前统一使用 `state.base_lin_vel` 作为速度测量，并使用 `input_velocity_measurement_noise` 更新；`odom_velocity_measurement_noise` 当前主要用于配置合同和 DDS bridge 语义，尚未在控制器 update 中单独区分 odom 与其他输入速度源。

### 3.3 Zero Velocity Update

当机器人被判定为静止时，滤波器额外融合一个零速度测量：

```text
z = [0, 0, 0]
R = zero_velocity_measurement_noise
```

静止条件为：

```text
max(abs(joint_dq)) <= stationary_joint_velocity_threshold
norm(base_ang_vel) <= stationary_ang_vel_threshold
abs(norm(base_lin_acc) - expected_acc_norm) <= stationary_accel_norm_tolerance
```

其中：

```text
expected_acc_norm = gravity_mps2    # imu_accel_includes_gravity=true
expected_acc_norm = 0               # imu_accel_includes_gravity=false
```

这个机制用于抑制 IMU 积分漂移，尤其是机器人站立或刚启动时。

## 4. 数据流

### 4.1 Sim2sim

MuJoCo backend 会填充：

```text
RobotStateData.base_lin_vel
RobotStateData.base_lin_vel_valid
```

来源由 profile 的：

```yaml
source_contract:
  sim_base:
    velocity_source: body_object_velocity_local
```

控制，可选值包括：

- `freejoint_qvel`
- `body_object_velocity_local`
- `body_cvel`

如果 `base_velocity_estimator.enabled: false`，策略 observation 直接使用 MuJoCo 提供的速度。sim2sim 中通常不需要开启滤波器，除非要模拟实机观测链路。

### 4.2 Sim2real / Real Robot

实机路径通常是：

```text
robot sensors / DDS / ROS odom
  -> SolverDdsBridge fills RobotStateData
  -> RL_controller::estimateBaseLinearVelocity
  -> robot.base_lin_vel
  -> ObservationBuilder base_lin_vel term
  -> policy obs
```

如果启用 odom：

```yaml
use_odom_velocity_measurement: true
odom_topic: /glim/odom
odom_velocity_frame: world
```

`SolverDdsBridge` 会订阅该 topic，把 `msg->twist.twist.linear` 写入 `state.base_lin_vel`，并置 `base_lin_vel_valid = true`。

如果 `odom_velocity_frame: body`，bridge 会使用 odom quaternion 或最近 IMU quaternion 把速度旋到 world frame。

## 5. 如何启用

在对应 profile YAML 的 `source_contract` 下加入：

```yaml
source_contract:
  base_velocity_estimator:
    enabled: true
    use_imu_prediction: true
    imu_accel_frame: body
    imu_accel_includes_gravity: true
    gravity_mps2: 9.80665

    use_input_velocity_measurement: true
    input_velocity_measurement_noise: 0.02

    use_odom_velocity_measurement: true
    odom_topic: /glim/odom
    odom_velocity_frame: world
    odom_velocity_measurement_noise: 0.03

    zero_velocity_update: true
    zero_velocity_measurement_noise: 0.01
    stationary_joint_velocity_threshold: 0.03
    stationary_ang_vel_threshold: 0.12
    stationary_accel_norm_tolerance: 0.5

    initial_variance: 0.25
    process_noise: 0.05
    accel_noise: 0.2
    min_dt: 0.0001
    max_dt: 0.05
    reset_on_mode_switch: true
```

如果当前 profile 中这段是注释状态，需要取消注释后重新加载配置。

## 6. 参数含义

### enable / source

`enabled`

是否启用滤波器。关闭时，`robot.base_lin_vel = state.base_lin_vel`。

`use_imu_prediction`

是否用 IMU 线加速度积分预测速度。没有可靠线速度测量时必须开启；如果 IMU 线加速度质量差，可以先关闭，只用 odom / zero update。

`imu_accel_frame`

IMU 线加速度所在坐标系：

- `body`: 机器人基座/body frame
- `world`: world frame

填错会导致前进速度、横向速度或 z 速度方向异常。

`imu_accel_includes_gravity`

IMU 加速度是否包含重力。常见 IMU raw acceleration 包含重力，应设为 `true`。如果上游已经输出 linear acceleration without gravity，应设为 `false`。

`gravity_mps2`

重力常数，默认 `9.80665`。

### velocity measurement

`use_input_velocity_measurement`

是否融合 `state.base_lin_vel`。在实机中，如果 DDS bridge 已经把 odom 写入 `state.base_lin_vel`，这里应开启。

`input_velocity_measurement_noise`

速度测量噪声。数值越小，滤波器越相信外部速度；数值越大，越相信 IMU prediction。

经验：

- 外部 odom 很稳定：`0.01 ~ 0.03`
- odom 有抖动但趋势可靠：`0.03 ~ 0.08`
- odom 经常跳变：`0.08` 以上，或暂时关闭外部测量

`use_odom_velocity_measurement`

是否让 DDS bridge 订阅 ROS odom topic。

`odom_topic`

ROS odom topic 名称，例如 `/glim/odom`。

`odom_velocity_frame`

odom twist linear velocity 的坐标系：

- `world`: 已经是 world frame
- `body`: body frame，需要旋到 world frame

### zero velocity update

`zero_velocity_update`

是否启用静止零速约束。实机强烈建议开启。

`zero_velocity_measurement_noise`

零速测量噪声。越小，静止时越快把速度拉回 0。

经验：

- 站立速度仍漂移：减小到 `0.005 ~ 0.01`
- 慢速起步时被“吸住”：增大到 `0.02 ~ 0.05`，或放宽/关闭静止判定

`stationary_joint_velocity_threshold`

所有关节速度都低于该阈值时，才可能认为机器人静止。

`stationary_ang_vel_threshold`

基座角速度模长低于该阈值时，才可能认为静止。

`stationary_accel_norm_tolerance`

加速度模长接近期望值的容差。IMU 噪声较大时要适当增大。

### covariance / dt

`initial_variance`

滤波器初始速度不确定性。越大，首次测量收敛越快。

`process_noise`

基础过程噪声，随 `dt` 线性增加。增大后滤波器更容易跟随测量变化。

`accel_noise`

加速度积分噪声，随 `dt^2` 增加。IMU 加速度噪声大时增大它。

`min_dt`

小于等于该值的 prediction 会被跳过，避免异常小时间步造成无意义更新。

`max_dt`

prediction 的最大时间步，防止控制循环卡顿后一次积分过大。

`reset_on_mode_switch`

切换 mode 时是否重置滤波器。不同策略/模式之间 observation 合同不同，建议保持 `true`。

## 7. 推荐调参流程

### Step 1: 先确认坐标系

让机器人静止，查看 IMU 加速度：

- 如果 `imu_accel_includes_gravity: true`，静止时 `norm(base_lin_acc)` 应接近 `9.81`
- 如果设为 `false`，静止时应接近 `0`

让机器人向前平移或人工推动，确认：

- `base_lin_vel.x` 对应机器人前向
- `base_lin_vel.y` 对应机器人左/右向，符号符合训练定义
- yaw 转动不应产生很大的 x/y 线速度偏置

如果方向错，优先检查：

- `imu_accel_frame`
- `odom_velocity_frame`
- IMU quaternion 顺序和 frame alignment

### Step 2: 只开 odom measurement

先临时关闭 IMU prediction：

```yaml
use_imu_prediction: false
use_input_velocity_measurement: true
use_odom_velocity_measurement: true
zero_velocity_update: true
```

观察策略输入里的 `base_lin_vel` 是否和 `/glim/odom` 的速度趋势一致。

### Step 3: 加回 IMU prediction

开启：

```yaml
use_imu_prediction: true
```

如果速度变得发散或 z 速度明显漂移，优先检查：

- 重力是否扣了两次或没有扣
- body/world frame 是否填反
- base quaternion 是否有效

### Step 4: 调 measurement noise

现象与处理：

- 速度太抖：增大 `input_velocity_measurement_noise`
- 速度滞后：减小 `input_velocity_measurement_noise` 或增大 `process_noise`
- 静止时不归零：减小 `zero_velocity_measurement_noise`
- 慢速走动被误判静止：降低 `stationary_joint_velocity_threshold`，降低 `stationary_ang_vel_threshold`，或增大 `zero_velocity_measurement_noise`

### Step 5: 固化参数

实机可先从以下组合开始：

```yaml
input_velocity_measurement_noise: 0.02
zero_velocity_measurement_noise: 0.01
initial_variance: 0.25
process_noise: 0.05
accel_noise: 0.2
stationary_joint_velocity_threshold: 0.03
stationary_ang_vel_threshold: 0.12
stationary_accel_norm_tolerance: 0.5
```

然后根据真实日志调小或调大。

## 8. 如何验证实机结果正确

### 8.1 离地静止检查

机器人上电、保持站立不动，记录 10 到 30 秒：

期望：

- `base_lin_vel.x/y/z` 收敛到接近 0
- 不应出现持续单方向漂移
- `z` 速度不应随时间持续变大或变小

建议阈值：

```text
abs(vx) < 0.05 m/s
abs(vy) < 0.05 m/s
abs(vz) < 0.05 m/s
```

如果静止时速度不归零，检查 zero velocity update 是否触发。常见原因是关节速度噪声或 IMU 角速度噪声超过静止阈值。

### 8.2 手推直线检查

人工缓慢向前/向后推动机器人，或使用安全的低速指令：

期望：

- 向前时 `vx` 为正，向后时 `vx` 为负
- `vy` 均值接近 0
- 停止后速度回到 0

如果 `vx/vy` 互换或符号反了，检查 odom frame、IMU frame 和训练坐标定义。

### 8.3 横向检查

人工横向移动或在安全条件下给横向指令：

期望：

- 横移主要反映在 `vy`
- `vx` 不应出现同量级响应

### 8.4 转向检查

原地 yaw 转动：

期望：

- `base_ang_vel.z` 有明显响应
- `base_lin_vel.x/y` 不应因为纯 yaw 长时间偏离 0

若纯旋转导致线速度很大，通常是 odom twist frame 或 IMU orientation frame 处理错误。

### 8.5 与外部 ground truth 对比

如果有 motion capture / SLAM / lidar odom：

- 对比滤波后的 `base_lin_vel` 与外部速度
- 看趋势、峰值、相位延迟
- 停止阶段重点看归零速度和残余漂移

接受标准可以按策略敏感性设置，常见目标：

```text
低速行走均方误差 < 0.05 ~ 0.10 m/s
停止后 0.5 s 内回到 abs(v) < 0.05 m/s
无持续 z 速度漂移
```

## 9. 日志与观察方法

运行时 MCAP 日志的 `runtime/tick` 会记录：

- `observation`
- `joint_q / joint_dq / joint_tau`
- `policy_action`
- named features
- policy source samples

如果当前 observation manifest 中包含 `base_lin_vel`，可以从 `observation` 对应维度查看策略实际看到的速度。

对 BeyondMimic full-body manifest，`base_lin_vel` 位于：

```text
ref_joint_pos(28)
+ ref_joint_vel(28)
+ motion_anchor_pos_b(3)
+ motion_anchor_ori_b(6)
= offset 65
base_lin_vel: observation[65:68]
```

对 BeyondMimic leg12 manifest，按 manifest 中各 term 的 count 累加得到 offset。

验证时建议同时记录：

- raw odom twist
- IMU acceleration / angular velocity
- policy observation
- robot joint velocities

这样可以区分是上游传感器问题、frame 问题，还是滤波参数问题。

## 10. 常见问题

### 速度长期漂移

优先检查：

- `zero_velocity_update` 是否开启
- 静止阈值是否过严导致不触发
- `imu_accel_includes_gravity` 是否正确
- IMU frame 是否正确

### z 速度异常

大概率是重力处理错误：

- raw IMU 包含重力，但配置成 `imu_accel_includes_gravity: false`
- 上游已经去重力，但配置成 `true`
- base quaternion 错误导致重力方向旋转错误

### 速度方向或符号错

检查：

- `odom_velocity_frame`
- `imu_accel_frame`
- odom quaternion 是否有效
- IMU `frame_alignment_rpy`
- 机器人训练坐标系约定

### 策略表现比不用滤波更差

先在实机中临时关闭 IMU prediction，只用外部 odom：

```yaml
use_imu_prediction: false
use_input_velocity_measurement: true
zero_velocity_update: true
```

如果表现恢复，说明 IMU 加速度积分链路需要重新检查 frame / gravity / noise。

### sim2sim 中结果和训练不一致

sim2sim 通常应直接使用仿真速度，不必启用该滤波器。若启用滤波器，会引入实机观测链路的延迟和噪声特性，可能导致与训练中的 privileged velocity 有差异。

## 11. 当前实现限制

当前滤波器是轻量速度滤波器，不是完整惯导：

- 状态只包含速度，不估计位置、姿态、IMU bias
- 三个轴独立更新，不维护完整 3x3 协方差
- `odom_velocity_measurement_noise` 当前没有在 `RL_controller` 中与 `input_velocity_measurement_noise` 分离使用
- 没有显式接触约束或足端里程计约束

它的设计目标是给 RL policy 提供稳定、低延迟、不会明显漂移的 `base_lin_vel` observation，而不是替代完整状态估计器。

如果后续需要更高精度，可以扩展为：

- IMU bias 状态
- full covariance
- 足端接触零速/运动学约束
- odom 与 input velocity 分 source 独立测量噪声
- 输出滤波状态和创新量到 runtime log
