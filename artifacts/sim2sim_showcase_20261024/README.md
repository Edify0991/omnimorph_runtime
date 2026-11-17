# Sim2Sim 汇报视频

4 段视频均为 **15 秒、1920×1080、30 fps、H.264 MP4、无音轨**。摄像头跟随机器人平移并保持固定观察方向，画面没有控制台和调试标记，可直接插入汇报 PPT。

| 视频 | 内容 | 实际运行配置 |
| --- | --- | --- |
| `01_JC01_AMP_walk_run.mp4` | JC01 走路逐渐加速到跑步 | `amp_mjlab_jc01_walkrun_full_body`，mode 1 |
| `02_JC01_BeyondMimic_taichi.mp4` | JC01 太极 | `beyondmimic_jc01_taichi_full_body_history4_wo_state_estimation`，mode 8 |
| `03_JC01_BeyondMimic_dance.mp4` | JC01 舞蹈 | `beyondmimic_jc01_dance_full_body_wo_state_estimation`，mode 7 |
| `04_G1_OmniXtreme_acrobatics.mp4` | G1 倒立、翻转等高难度动作 | `omnixtreme_g1_full_body`，mode 2 |

## 录制方法

实际运行项目已安装的 C++ `mujoco_sim_bridge`，由 ONNX 策略闭环控制 MuJoCo 自由基座机器人。以约 50–60 Hz 发布的 qpos/qvel/ctrl 连续仿真状态为依据，使用相同 XML 模型重新渲染为 30 fps 视频；采样之间插值并归一化根部四元数。输出按物理仿真时间播放。

本机原生异步录屏触发 GLX 上下文线程错误，因此本次采用状态采集、单线程渲染流程。视频中的运动来自实际策略控制的物理仿真。BeyondMimic / OmniXtreme 只在启动时对齐参考初始关节姿态，运行中保持自由基座闭环控制。

AMP 指令前 5 秒为 0.6 m/s，之后以 0.4 m/s² 加速至 1.8 m/s。JC01 三段均截取 RUNNING 后 0.5–15.5 秒；G1 截取 RUNNING 后 16.5–31.5 秒，包含倒立和翻转动作。

## 配置说明

原始“舞蹈”profile 的 `policy_name` 为舞蹈，但 `policy_file` 指向 `beyondmimic_jingchu01_taichi1.onnx`。为展示舞蹈，本次在配置副本中使用项目自带的 `beyondmimic_jc01_dance_wo_state_estimation.onnx`。

录制脚本、使用的配置副本、仿真状态、MCAP 日志及关键帧预览均保存在 `work/`。各段的 `*_metadata.json` 记录摄像头、裁剪区间等参数。项目原始配置和源码没有因本次录制而修改。

## 复现

在项目根目录执行，使用系统 Python 和 ROS Humble：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PATH=/usr/bin:/bin:$PATH
export ROS_DOMAIN_ID=81
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOG_DIR=/tmp/jc01_showcase_ros
export ROBOT_ASSETS_DIR=/home/edify/Code/jc01-model

# key 可选 amp / taichi / dance / omnixtreme，每次运行一个
/usr/bin/python3 artifacts/sim2sim_showcase_20261024/work/record_showcase.py amp --duration 17
/usr/bin/python3 artifacts/sim2sim_showcase_20261024/work/render_showcase.py amp

# G1 先采集至少 32 秒，再导出所选动作段
/usr/bin/python3 artifacts/sim2sim_showcase_20261024/work/record_showcase.py omnixtreme --duration 33
/usr/bin/python3 artifacts/sim2sim_showcase_20261024/work/render_showcase.py omnixtreme --start 16.5
```

渲染需要可用的本机 X11/OpenGL 环境。模型及策略路径保留为这台机器的绝对路径。
