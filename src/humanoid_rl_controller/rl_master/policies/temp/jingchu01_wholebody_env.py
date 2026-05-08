# Copyright (c) 2021-2024, The RSL-RL Project Developers.
# All rights reserved.
# Original code is licensed under the BSD-3-Clause license.
#
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The Legged Lab Project Developers.
# All rights reserved.
#
# Copyright (c) 2025-2026, The TienKung-Lab Project Developers.
# All rights reserved.
# Modifications are licensed under the BSD-3-Clause license.
#
# This file contains code derived from the RSL-RL, Isaac Lab, and Legged Lab Projects,
# with additional modifications by the TienKung-Lab Project,
# and is distributed under the BSD-3-Clause license.

"""
Jingchu01 双足机器人强化学习环境

该模块实现了 Jingchu01 人形机器人的 Isaac Gym 仿真环境，支持：
- 行走/奔跑任务训练
- AMP (Adversarial Motion Prior) 对抗性运动先验
- 领域随机化 (Domain Randomization)
- 步态周期参数化
- 多模态感知 (高度扫描、深度相机、LiDAR)

环境类 Jc01Env 继承自 VecEnv，提供与 RSL-RL 兼容的接口。
"""

# ==================== 导入标准库 ====================
import isaaclab.sim as sim_utils
import isaacsim.core.utils.torch as torch_utils  # type: ignore
import numpy as np
import torch
from scipy.spatial.transform import Rotation

# ==================== Isaac Lab 核心模块 ====================
# 资产与场景管理
from isaaclab.assets.articulation import Articulation
from isaaclab.scene import InteractiveScene
from isaaclab.sensors import ContactSensor, RayCaster
from isaaclab.sensors.camera import TiledCamera
from isaaclab.sim import PhysxCfg, SimulationContext

# 命令生成器 - 用于生成期望速度命令
from isaaclab.envs.mdp.commands import UniformVelocityCommand, UniformVelocityCommandCfg

# 管理器 - 奖励和事件管理
from isaaclab.managers import EventManager, RewardManager
from isaaclab.managers.scene_entity_cfg import SceneEntityCfg

# 工具函数 - 缓冲区、延迟、四元数运算
from isaaclab.utils.buffers import CircularBuffer, DelayBuffer
from isaaclab.utils.math import quat_apply, quat_conjugate, quat_rotate

# ==================== Legged Lab 内部模块 ====================
from legged_lab.utils.env_utils.scene import SceneCfg
from legged_lab.mdp import rewards as mdp

# 环境配置类 - 支持多种配置变体
from legged_lab.envs.tienkung.run_cfg import TienKungRunFlatEnvCfg
from legged_lab.envs.tienkung.run_with_sensor_cfg import TienKungRunWithSensorFlatEnvCfg
from legged_lab.envs.tienkung.walk_cfg import TienKungWalkFlatEnvCfg
from legged_lab.envs.tienkung.walk_with_sensor_cfg import TienKungWalkWithSensorFlatEnvCfg
# ==================== RSL-RL 接口 ====================
from rsl_rl.env import VecEnv
from rsl_rl.utils.motion_loader_for_jingchu01_display import AMPLoaderDisplayJingchu01


# ================================================================================
#                                Jc01Env 类定义
# ================================================================================

class Jingchu01WholeBodyEnv(VecEnv):
    """
    Jingchu01 双足机器人强化学习环境
    
    继承自 VecEnv，实现与 RSL-RL 训练框架的兼容接口。
    支持并行模拟多个机器人环境 (默认 4096 个)。
    
    Attributes:
        cfg: 环境配置对象，包含场景、奖励、命令等所有参数
        robot: 机器人 articulation 对象
        contact_sensor: 接触力传感器
        height_scanner: 高度扫描传感器 (可选)
        command_generator: 速度命令生成器
        reward_manager: 奖励计算管理器
        gait_phase: 当前步态相位 [num_envs, 2] (左/右腿)
    
    Note:
        Jingchu01 机器人规格：
        - 12 个主动关节 (每条腿 6 个: hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll)
        - 末端执行器: 踝关节 (ankle_pitch, ankle_roll)
        - 传感器: 接触力、高度扫描、深度相机、LiDAR
        [TODO: 后续添加手臂支持后更新为 20 个关节]"""

    # =========================================================================
    # 手臂关节配置 (暂时禁用 - URDF 只有腿部)
    # 如需启用，需加载完整 URDF 并取消下面注释
    # =========================================================================
    LEFT_ARM_JOINTS = [
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_shoulder_yaw",
        "left_elbow_pitch",
    ]
    RIGHT_ARM_JOINTS = [
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow_pitch",
    ]
    
    def __init__(
        self,
        cfg: (
            TienKungRunFlatEnvCfg
            | TienKungWalkFlatEnvCfg
            | TienKungWalkWithSensorFlatEnvCfg
            | TienKungRunWithSensorFlatEnvCfg
        ),
        headless: bool,
    ):
        """
        初始化仿真环境
        
        Args:
            cfg: 环境配置对象，支持多种配置类型
            headless: 是否无头模式运行 (无渲染)
        """
        # =========================================================================
        # 基础配置存储
        # =========================================================================
        self.cfg = cfg
        self.headless = headless
        self.device = self.cfg.device
        
        # 仿真时间步参数
        self.physics_dt = self.cfg.sim.dt              # 物理引擎时间步 (默认 0.005s)
        self.step_dt = self.cfg.sim.decimation * self.cfg.sim.dt  # 环境步长 (默认 0.02s)
        
        self.num_envs = self.cfg.scene.num_envs        # 并行环境数量
        self.seed(cfg.scene.seed)                      # 设置随机种子
        
        # =========================================================================
        # 仿真上下文初始化
        # =========================================================================
        sim_cfg = sim_utils.SimulationCfg(
            device=cfg.device,
            dt=cfg.sim.dt,
            render_interval=cfg.sim.decimation,
            physx=PhysxCfg(gpu_max_rigid_patch_count=cfg.sim.physx.gpu_max_rigid_patch_count),
            # 物理材质配置 - 使用乘积模式混合摩擦系数
            physics_material=sim_utils.RigidBodyMaterialCfg(
                friction_combine_mode="multiply",
                restitution_combine_mode="multiply",
                static_friction=1.0,
                dynamic_friction=1.0,
            ),
        )
        self.sim = SimulationContext(sim_cfg)
        
        # =========================================================================
        # 交互式场景构建
        # =========================================================================
        scene_cfg = SceneCfg(config=cfg.scene, physics_dt=self.physics_dt, step_dt=self.step_dt)
        self.scene = InteractiveScene(scene_cfg)
        self.sim.reset()
        
        # =========================================================================
        # 传感器和资产引用
        # =========================================================================
        self.robot: Articulation = self.scene["robot"]                    # 机器人本体
        self.contact_sensor: ContactSensor = self.scene.sensors["contact_sensor"]  # 接触力传感器
        
        # 高度扫描传感器 (可选)
        if self.cfg.scene.height_scanner.enable_height_scan:
            self.height_scanner: RayCaster = self.scene.sensors["height_scanner"]
        
        # LiDAR 传感器 (可选)
        if self.cfg.scene.lidar.enable_lidar:
            self.lidar: RayCaster = self.scene.sensors["lidar"]
        
        # 深度相机 (可选)
        if self.cfg.scene.depth_camera.enable_depth_camera:
            self.depth_camera: TiledCamera = self.scene.sensors["depth_camera"]
        
        # =========================================================================
        # 命令生成器 - 生成随机速度命令
        # =========================================================================
        command_cfg = UniformVelocityCommandCfg(
            asset_name="robot",
            resampling_time_range=self.cfg.commands.resampling_time_range,
            rel_standing_envs=self.cfg.commands.rel_standing_envs,
            rel_heading_envs=self.cfg.commands.rel_heading_envs,
            heading_command=self.cfg.commands.heading_command,
            heading_control_stiffness=self.cfg.commands.heading_control_stiffness,
            debug_vis=self.cfg.commands.debug_vis,
            ranges=self.cfg.commands.ranges,
        )
        self.command_generator = UniformVelocityCommand(cfg=command_cfg, env=self)
        
        # =========================================================================
        # 奖励管理器
        # =========================================================================
        self.reward_manager = RewardManager(self.cfg.reward, self)
        
        # =========================================================================
        # 缓冲区和变量初始化
        # =========================================================================
        self.init_buffers()
        
        # =========================================================================
        # 事件管理器 - 领域随机化
        # =========================================================================
        env_ids = torch.arange(self.num_envs, device=self.device)
        self.event_manager = EventManager(self.cfg.domain_rand.events, self)
        
        # 应用 startup 事件 (环境初始化时的随机化)
        if "startup" in self.event_manager.available_modes:
            self.event_manager.apply(mode="startup")
        
        # 重置所有环境
        self.reset(env_ids)
        
        # =========================================================================
        # AMP 运动可视化加载器 (用于可视化参考运动)
        # =========================================================================
        self.amp_loader_display = AMPLoaderDisplayJingchu01(
            motion_files=self.cfg.amp_motion_files_display,
            device=self.device,
            time_between_frames=self.physics_dt
        )
        self.motion_len = self.amp_loader_display.trajectory_num_frames[0]

    def init_buffers(self):
        """
        初始化所有运行时缓冲区和配置
        
        该方法在 __init__ 中调用，用于分配张量缓冲区、解析关节/刚体 ID、
        初始化步态参数等。
        """
        # =========================================================================
        # 基础参数配置
        # =========================================================================
        self.extras = {}  # 日志额外信息
        
        # 回合最大长度配置
        self.max_episode_length_s = self.cfg.scene.max_episode_length_s
        self.max_episode_length = np.ceil(self.max_episode_length_s / self.step_dt)
        
        # 动作和观测裁剪范围
        self.num_actions = self.robot.data.default_joint_pos.shape[1]
        self.clip_actions = self.cfg.normalization.clip_actions
        self.clip_obs = self.cfg.normalization.clip_observations
        
        # 动作缩放因子
        self.action_scale = self.cfg.robot.action_scale
        
        # =========================================================================
        # 终止条件持续帧计数器 (用于减少误触发)
        # =========================================================================
        # 终止条件必须连续满足这么多帧才会触发重置
        self.termination_consecutive_frames = 10  # 连续10帧触发才重置 (约0.2秒 @ 50Hz)
        self.termination_counter = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        
        # =========================================================================
        # 动作延迟缓冲区 (用于模拟动作延迟)
        # =========================================================================
        self.action_buffer = DelayBuffer(
            self.cfg.domain_rand.action_delay.params["max_delay"],
            self.num_envs,
            device=self.device
        )
        self.action_buffer.compute(
            torch.zeros(self.num_envs, self.num_actions, dtype=torch.float, device=self.device, requires_grad=False)
        )
        
        # 如果启用动作延迟，为每个环境随机设置延迟时间
        if self.cfg.domain_rand.action_delay.enable:
            time_lags = torch.randint(
                low=self.cfg.domain_rand.action_delay.params["min_delay"],
                high=self.cfg.domain_rand.action_delay.params["max_delay"] + 1,
                size=(self.num_envs,),
                dtype=torch.int,
                device=self.device,
            )
            self.action_buffer.set_time_lag(time_lags, torch.arange(self.num_envs, device=self.device))

        # =========================================================================
        # 场景实体配置解析 - 查找关节和刚体 ID
        # =========================================================================
        # 机器人本体配置
        self.robot_cfg = SceneEntityCfg(name="robot")
        self.robot_cfg.resolve(self.scene)
        
        # 终止接触配置 - 检测这些部位的接触力来判断是否跌倒
        self.termination_contact_cfg = SceneEntityCfg(
            name="contact_sensor",
            body_names=self.cfg.robot.terminate_contacts_body_names
        )
        self.termination_contact_cfg.resolve(self.scene)
        
        # 脚部配置 - 用于脚部相关奖励计算
        self.feet_cfg = SceneEntityCfg(name="contact_sensor", body_names=self.cfg.robot.feet_body_names)
        self.feet_cfg.resolve(self.scene)
        
        # =========================================================================
        # 刚体 ID 查找 - 用于获取特定部位的状态
        # =========================================================================
        # 脚部刚体 (踝关节)
        self.feet_body_ids, _ = self.robot.find_bodies(
            name_keys=["left_ankle_roll", "right_ankle_roll"],
            preserve_order=True
        )
        
        # 肘部刚体 (用于手部位置估计)
        self.elbow_body_ids, _ = self.robot.find_bodies(
            name_keys=["left_elbow_pitch", "right_elbow_pitch"],
            preserve_order=True
        )
        
        # =========================================================================
        # 关节 ID 查找 - 按功能分组
        # =========================================================================
        # 左腿关节: hip_roll, hip_yaw, hip_pitch, knee, ankle_pitch, ankle_roll
        self.left_leg_ids, _ = self.robot.find_joints(
            name_keys=[
                "left_hip_roll",
                "left_hip_yaw",
                "left_hip_pitch",
                "left_knee_pitch",
                "left_ankle_pitch",
                "left_ankle_roll",
            ],
            preserve_order=True,
        )

        # 右腿关节 (同上)
        self.right_leg_ids, _ = self.robot.find_joints(
            name_keys=[
                "right_hip_roll",
                "right_hip_yaw",
                "right_hip_pitch",
                "right_knee_pitch",
                "right_ankle_pitch",
                "right_ankle_roll",
            ],
            preserve_order=True,
        )
        
        # 左臂关节: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
        self.left_arm_ids, _ = self.robot.find_joints(
            name_keys=[
                "left_shoulder_pitch",
                "left_shoulder_roll",
                "left_shoulder_yaw",
                "left_elbow_pitch",
                "left_elbow_yaw",
                "left_wrist_pitch",
                "left_wrist_roll",
            ],
            preserve_order=True,
        )
        
        # 右臂关节
        self.right_arm_ids, _ = self.robot.find_joints(
            name_keys=[
                "right_shoulder_pitch",
                "right_shoulder_roll",
                "right_shoulder_yaw",
                "right_elbow_pitch",
                "right_elbow_yaw",
                "right_wrist_pitch",
                "right_wrist_roll",
            ],
            preserve_order=True,
        )
        
        # 踝关节 (用于踝关节力矩惩罚)
        self.ankle_joint_ids, _ = self.robot.find_joints(
            name_keys=[
                "left_ankle_pitch",
                "right_ankle_pitch",
                "left_ankle_roll",
                "right_ankle_roll"
            ],
            preserve_order=True,
        )

        # 腰部关节 (用于全身支持)
        self.waist_ids, _ = self.robot.find_joints(
            name_keys=[
                "waist_roll",
                "waist_yaw",
            ],
            preserve_order=True,
        )

        # =========================================================================
        # 观测和噪声配置
        # =========================================================================
        self.obs_scales = self.cfg.normalization.obs_scales
        self.add_noise = self.cfg.noise.add_noise
        
        # =========================================================================
        # 环境状态缓冲区
        # =========================================================================
        self.episode_length_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.long)
        self.sim_step_counter = 0
        self.time_out_buf = torch.zeros(self.num_envs, device=self.device, dtype=torch.bool)
        
        # [TODO: 启用手臂支持后取消注释 - 手部本地位置偏移 (相对于肘部)]
        self.left_arm_local_vec = torch.tensor([0.0, 0.0, -0.3], device=self.device).repeat((self.num_envs, 1))
        self.right_arm_local_vec = torch.tensor([0.0, 0.0, -0.3], device=self.device).repeat((self.num_envs, 1))
        
        # =========================================================================
        # 步态周期参数
        # =========================================================================
        # 步态相位 [num_envs, 2] - 0: 左腿, 1: 右腿
        self.gait_phase = torch.zeros(self.num_envs, 2, dtype=torch.float, device=self.device, requires_grad=False)
        
        # 步态周期 (秒) - 控制步态频率
        self.gait_cycle = torch.full(
            (self.num_envs,),
            self.cfg.gait.gait_cycle,
            dtype=torch.float,
            device=self.device,
            requires_grad=False
        )
        
        # 相位抬腿比例 [num_envs, 2] - 0: 左腿, 1: 右腿
        self.phase_ratio = torch.tensor(
            [self.cfg.gait.gait_air_ratio_l, self.cfg.gait.gait_air_ratio_r],
            dtype=torch.float,
            device=self.device
        ).repeat(self.num_envs, 1)
        
        # 相位偏移 [num_envs, 2] - 控制在周期中的偏移
        self.phase_offset = torch.tensor(
            [self.cfg.gait.gait_phase_offset_l, self.cfg.gait.gait_phase_offset_r],
            dtype=torch.float,
            device=self.device,
        ).repeat(self.num_envs, 1)
        
        # 当前动作缓冲区
        self.action = torch.zeros(
            self.num_envs, self.num_actions,
            dtype=torch.float,
            device=self.device,
            requires_grad=False
        )
        
        # 脚部力和速度累积 (用于周期性奖励)
        self.avg_feet_force_per_step = torch.zeros(
            self.num_envs,
            len(self.feet_cfg.body_ids),
            dtype=torch.float,
            device=self.device,
            requires_grad=False
        )
        self.avg_feet_speed_per_step = torch.zeros(
            self.num_envs,
            len(self.feet_cfg.body_ids),
            dtype=torch.float,
            device=self.device,
            requires_grad=False
        )
        
        # =========================================================================
        # 观测缓冲区初始化
        # =========================================================================
        self.init_obs_buffer()

    def visualize_motion(self, time: float) -> torch.Tensor:
        """
        根据 AMP 运动捕捉数据更新机器人仿真状态
        
        该方法用于可视化参考运动轨迹，将 AMP 运动帧应用到仿真环境中。
        
        Args:
            time: 时间戳 (秒)，指定获取哪一帧运动数据
            
        Returns:
            AMP 训练格式张量 [num_envs, 68]:
            [dof_pos(28), dof_vel(28), left_hand(3), right_hand(3), left_foot(3), right_foot(3)]
        """
        # 获取指定时间的 AMP 运动帧 (从完整数据中获取，避免混合截断)
        device = self.device
        traj_idx = 0
        p = float(time) / self.amp_loader_display.trajectory_lens[traj_idx]
        n = self.amp_loader_display.trajectories_full[traj_idx].shape[0]
        idx = int(np.floor(p * n))
        idx = min(idx, n - 1)
        visual_motion_frame = self.amp_loader_display.trajectories_full[traj_idx][idx]
        frame_len = len(visual_motion_frame)

        # 初始化关节位置和速度缓冲区
        dof_pos = torch.zeros((self.num_envs, self.robot.num_joints), device=device)
        dof_vel = torch.zeros((self.num_envs, self.robot.num_joints), device=device)

        # =================================================================
        # 解析运动帧数据并分配到对应关节
        # 支持两种格式:
        #   - 68维: [root_pos(3), euler(3), dof_pos(28), lin_vel(3), ang_vel(3), dof_vel(28)]
        #   - 其他: 尽量从数据中读取, 不足补零
        # PKL 顺序: [左腿0:6, 右腿6:12, 腰部12:14, 左臂14:21, 右臂21:28]
        # =================================================================
        if frame_len >= 68:
            # 68维格式: 提取关节数据（跳过 root 状态）
            motion_dof_pos = visual_motion_frame[6:34]    # 28维, PKL 顺序
            motion_dof_vel = visual_motion_frame[40:68]   # 28维, PKL 顺序
        else:
            # 未知格式: 尽量从数据中读取, 不足的部分补零
            motion_dof_pos = torch.zeros(28, device=device)
            motion_dof_vel = torch.zeros(28, device=device)
            n_pos = min(28, frame_len)
            motion_dof_pos[:n_pos] = visual_motion_frame[:n_pos]
            if frame_len >= 56:
                motion_dof_vel[:28] = visual_motion_frame[28:56]

        # 断言: 确保切片后维度正确
        assert len(motion_dof_pos) >= 28, f"motion_dof_pos length {len(motion_dof_pos)} < 28, frame_len={frame_len}"
        assert len(motion_dof_vel) >= 28, f"motion_dof_vel length {len(motion_dof_vel)} < 28, frame_len={frame_len}"

        # 逐关节显式赋值: PKL顺序 → URDF顺序
        # 利用 broadcasting: [28] → [num_envs, 28]
        # 左腿: PKL[0:6] → URDF left_leg_ids
        dof_pos[:, self.left_leg_ids] = motion_dof_pos[0:6]
        dof_vel[:, self.left_leg_ids] = motion_dof_vel[0:6]
        # 右腿: PKL[6:12] → URDF right_leg_ids
        dof_pos[:, self.right_leg_ids] = motion_dof_pos[6:12]
        dof_vel[:, self.right_leg_ids] = motion_dof_vel[6:12]
        # waist: PKL[12:14] → URDF waist_ids
        if hasattr(self, 'waist_ids') and len(self.waist_ids) > 0:
            dof_pos[:, self.waist_ids] = motion_dof_pos[12:14]
            dof_vel[:, self.waist_ids] = motion_dof_vel[12:14]
        # 左臂: PKL[14:21] → URDF left_arm_ids
        if hasattr(self, 'left_arm_ids') and len(self.left_arm_ids) > 0:
            n_left_arm = len(self.left_arm_ids)
            dof_pos[:, self.left_arm_ids] = motion_dof_pos[14:14+n_left_arm]
            dof_vel[:, self.left_arm_ids] = motion_dof_vel[14:14+n_left_arm]
        # 右臂: PKL[21:28] → URDF right_arm_ids
        if hasattr(self, 'right_arm_ids') and len(self.right_arm_ids) > 0:
            n_right_arm = len(self.right_arm_ids)
            dof_pos[:, self.right_arm_ids] = motion_dof_pos[21:21+n_right_arm]
            dof_vel[:, self.right_arm_ids] = motion_dof_vel[21:21+n_right_arm]
        
        # 将关节状态写入仿真器
        self.robot.write_joint_position_to_sim(dof_pos)
        self.robot.write_joint_velocity_to_sim(dof_vel)
        
        env_ids = torch.arange(self.num_envs, device=device)
        
        # =================================================================
        # 计算根状态
        # 68维格式: [root_pos(3), euler(3), dof_pos(28), lin_vel(3), ang_vel(3), dof_vel(28)]
        # 其他格式: 使用默认站立姿态
        # =================================================================
        if frame_len >= 68:
            root_pos = visual_motion_frame[0:3]
            euler_angles = visual_motion_frame[3:6]   # roll, pitch, yaw
            lin_vel = visual_motion_frame[34:37]
            ang_vel = visual_motion_frame[37:40]
        else:
            root_pos = torch.tensor([0.0, 0.0, 0.85], device=device)
            euler_angles = torch.zeros(3, device=device)
            lin_vel = torch.zeros(3, device=device)
            ang_vel = torch.zeros(3, device=device)

        # 将欧拉角 (XYZ顺序) 转换为四元数 (wxyz)
        from scipy.spatial.transform import Rotation as R
        quat_xyzw = R.from_euler('XYZ', euler_angles.cpu().numpy()).as_quat()
        quat_wxyz = torch.tensor([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                                     dtype=torch.float32, device=device)
        # 构建根状态张量: [x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz]
        root_state = torch.zeros((self.num_envs, 13), device=device)
        root_state[:, 0:3] = root_pos.unsqueeze(0).repeat(self.num_envs, 1)
        root_state[:, 3:7] = quat_wxyz.unsqueeze(0).repeat(self.num_envs, 1)
        root_state[:, 7:10] = lin_vel.unsqueeze(0).repeat(self.num_envs, 1)
        root_state[:, 10:13] = ang_vel.unsqueeze(0).repeat(self.num_envs, 1)
        
        self.robot.write_root_state_to_sim(root_state, env_ids)
        
        # =================================================================
        # 仿真一步
        # =================================================================
        self.sim.render()
        self.sim.step()
        self.scene.update(dt=self.step_dt)
        
        # =================================================================
        # 计算脚部位置 (相对于机器人根部的局部坐标系)
        # =================================================================
        # 脚部位置 (直接在根部坐标系)
        left_foot_pos = (
            self.robot.data.body_state_w[:, self.feet_body_ids[0], :3]
            - self.robot.data.root_state_w[:, 0:3]
        )
        right_foot_pos = (
            self.robot.data.body_state_w[:, self.feet_body_ids[1], :3]
            - self.robot.data.root_state_w[:, 0:3]
        )
        left_foot_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), left_foot_pos)
        right_foot_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), right_foot_pos)
        
        # 手部位置: 肘部 + 偏移
        left_hand_pos = (
            self.robot.data.body_state_w[:, self.elbow_body_ids[0], :3]
            - self.robot.data.root_state_w[:, 0:3]
            + quat_rotate(self.robot.data.body_state_w[:, self.elbow_body_ids[0], 3:7], self.left_arm_local_vec)
        )
        right_hand_pos = (
            self.robot.data.body_state_w[:, self.elbow_body_ids[1], :3]
            - self.robot.data.root_state_w[:, 0:3]
            + quat_rotate(self.robot.data.body_state_w[:, self.elbow_body_ids[1], 3:7], self.right_arm_local_vec)
        )
        # 转换到根局部坐标系
        left_hand_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), left_hand_pos)
        right_hand_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), right_hand_pos)
        
        # =================================================================
        # 保存当前关节状态
        # =================================================================
        self.left_leg_dof_pos = dof_pos[:, self.left_leg_ids]
        self.right_leg_dof_pos = dof_pos[:, self.right_leg_ids]
        self.left_leg_dof_vel = dof_vel[:, self.left_leg_ids]
        self.right_leg_dof_vel = dof_vel[:, self.right_leg_ids]
        self.left_arm_dof_pos = dof_pos[:, self.left_arm_ids]
        self.right_arm_dof_pos = dof_pos[:, self.right_arm_ids]
        self.left_arm_dof_vel = dof_vel[:, self.left_arm_ids]
        self.right_arm_dof_vel = dof_vel[:, self.right_arm_ids]
        
        # =================================================================
        # 拼接所有数据返回 (68 维 AMP 训练格式, 与 get_amp_obs_for_expert_trans 一致)
        # [0:28]   dof_pos (28, PKL 顺序)
        # [28:56]  dof_vel (28, PKL 顺序)
        # [56:59]  left_hand_pos
        # [59:62]  right_hand_pos
        # [62:65]  left_foot_pos
        # [65:68]  right_foot_pos
        # =================================================================
        return torch.cat(
            (
                motion_dof_pos.unsqueeze(0).repeat(self.num_envs, 1),   # 28
                motion_dof_vel.unsqueeze(0).repeat(self.num_envs, 1),   # 28
                left_hand_pos,    # 3
                right_hand_pos,   # 3
                left_foot_pos,    # 3
                right_foot_pos,   # 3
            ),
            dim=-1,
        )


    def compute_current_observations(self) -> tuple[torch.Tensor, torch.Tensor]:
        """
        计算当前时刻的 Actor 和 Critic 观测值
        
        Actor 观测 (策略网络输入):
            - ang_vel (角速度, 3维)                 [0:3]
            - projected_gravity (重力投影, 3维)     [3:6]
            - command (速度命令, 3维)               [6:9]
            - joint_pos 偏差 (num_actions=28维)     [9:9+N]
            - joint_vel 偏差 (num_actions=28维)     [9+N:9+2N]
            - action (上一动作, num_actions=28维)   [9+2N:9+3N]
            - sin(2π * gait_phase) (步态相位sin, 2维) [9+3N:11+3N]
            - cos(2π * gait_phase) (步态相位cos, 2维) [11+3N:13+3N]
            - phase_ratio (抬腿比例, 2维)           [13+3N:15+3N]
            共计: 3 + 3 + 3 + 28 + 28 + 28 + 2 + 2 + 2 = 99 维 (num_actions=28)
        
        Critic 观测 (价值网络输入):
            - Actor 观测全部 (99维)
            - root_lin_vel (线速度, 3维)
            - feet_contact (脚部接触标志, 2维)
            共计: 99 + 3 + 2 = 104 维 (num_actions=28)
        
        Returns:
            actor_obs: Actor 网络观测 [num_envs, actor_obs_dim]
            critic_obs: Critic 网络观测 [num_envs, critic_obs_dim]
        """
        robot = self.robot
        net_contact_forces = self.contact_sensor.data.net_forces_w_history
        
        # =================================================================
        # 提取基础感知数据
        # =================================================================
        ang_vel = robot.data.root_ang_vel_b                     # 根部角速度 (Body坐标系)
        projected_gravity = robot.data.projected_gravity_b      # 重力在Body坐标系的投影
        command = self.command_generator.command                  # 当前速度命令
        joint_pos = robot.data.joint_pos - robot.data.default_joint_pos  # 关节位置偏差
        joint_vel = robot.data.joint_vel - robot.data.default_joint_vel  # 关节速度偏差
        action = self.action_buffer._circular_buffer.buffer[:, -1, :]      # 上一时刻动作
        root_lin_vel = robot.data.root_lin_vel_b                 # 根部线速度
        
        # 脚部接触状态 (任一接触力 > 0.5N 视为接触)
        feet_contact = torch.max(
            torch.norm(net_contact_forces[:, :, self.feet_cfg.body_ids], dim=-1),
            dim=1
        )[0] > 0.5
        
        # =================================================================
        # 构建 Actor 观测
        # =================================================================
        current_actor_obs = torch.cat(
            [
                ang_vel * self.obs_scales.ang_vel,              # [3] 角速度
                projected_gravity * self.obs_scales.projected_gravity,  # [3] 重力投影
                command * self.obs_scales.commands,             # [3] 速度命令
                joint_pos * self.obs_scales.joint_pos,          # [N] 关节位置 (N=num_actions=28, 由 URDF 自动推导)
                joint_vel * self.obs_scales.joint_vel,          # [N] 关节速度 (N=num_actions=28)
                action * self.obs_scales.actions,                # [N] 上一动作 (N=num_actions=28)
                torch.sin(2 * torch.pi * self.gait_phase),      # [2] 相位 sin
                torch.cos(2 * torch.pi * self.gait_phase),      # [2] 相位 cos
                self.phase_ratio,                                # [2] 抬腿比例
            ],
            dim=-1,
        )
        
        # =================================================================
        # 构建 Critic 观测 (包含额外信息)
        # =================================================================
        current_critic_obs = torch.cat(
            [current_actor_obs, root_lin_vel * self.obs_scales.lin_vel, feet_contact],
            dim=-1
        )
        
        return current_actor_obs, current_critic_obs

    def compute_observations(self) -> tuple[torch.Tensor, torch.Tensor]:
        """
        计算历史观测序列
        
        在当前观测基础上:
        1. 添加噪声 (如果启用)
        2. 拼接历史观测形成序列
        3. 添加高度扫描数据 (如果启用)
        4. 添加深度相机数据 (如果启用)
        5. 裁剪到 [-clip_obs, clip_obs] 范围
        
        Returns:
            actor_obs: 展平的历史 Actor 观测
            critic_obs: 展平的历史 Critic 观测
        """
        current_actor_obs, current_critic_obs = self.compute_current_observations()
        
        # 添加观测噪声
        if self.add_noise:
            current_actor_obs += (2 * torch.rand_like(current_actor_obs) - 1) * self.noise_scale_vec
        
        # 添加到历史缓冲区
        self.actor_obs_buffer.append(current_actor_obs)
        self.critic_obs_buffer.append(current_critic_obs)
        
        # 展平历史观测 [history_len, num_envs, dim] -> [num_envs, history_len * dim]
        actor_obs = self.actor_obs_buffer.buffer.reshape(self.num_envs, -1)
        critic_obs = self.critic_obs_buffer.buffer.reshape(self.num_envs, -1)
        
        # =================================================================
        # 添加高度扫描数据 (可选)
        # =================================================================
        if self.cfg.scene.height_scanner.enable_height_scan:
            # 计算高度差: 传感器高度 - 射线命中点高度 - 偏移
            height_scan = (
                self.height_scanner.data.pos_w[:, 2].unsqueeze(1)
                - self.height_scanner.data.ray_hits_w[..., 2]
                - self.cfg.normalization.height_scan_offset
            ) * self.obs_scales.height_scan
            
            # 添加到 Critic 观测 (用于地形适应)
            critic_obs = torch.cat([critic_obs, height_scan], dim=-1)
            
            # 添加噪声
            if self.add_noise:
                height_scan += (2 * torch.rand_like(height_scan) - 1) * self.height_scan_noise_vec
            
            # 添加到 Actor 观测
            actor_obs = torch.cat([actor_obs, height_scan], dim=-1)
        
        # =================================================================
        # 添加深度相机数据 (可选)
        # =================================================================
        if self.cfg.scene.depth_camera.enable_depth_camera:
            # 获取深度图像 [num_envs, height, width, 1]
            depth_image = self.depth_camera.data.output["distance_to_image_plane"]
            
            # 展平为 [num_envs, height * width]
            flattened_depth = depth_image.view(self.num_envs, -1)
            
            # 拼接深度数据
            actor_obs = torch.cat([actor_obs, flattened_depth], dim=-1)
            critic_obs = torch.cat([critic_obs, flattened_depth], dim=-1)
        
        # =================================================================
        # 裁剪观测值
        # =================================================================
        actor_obs = torch.clip(actor_obs, -self.clip_obs, self.clip_obs)
        critic_obs = torch.clip(critic_obs, -self.clip_obs, self.clip_obs)
        
        return actor_obs, critic_obs

    def reset(self, env_ids: torch.Tensor) -> None:
        """
        重置指定环境
        
        执行以下操作:
        1. 清零脚部力/速度累积
        2. 更新地形关卡 (如果启用课程学习)
        3. 重置场景和事件
        4. 重置奖励管理器
        5. 重置命令生成器
        6. 重置缓冲区
        
        Args:
            env_ids: 需要重置的环境索引张量
        """
        if len(env_ids) == 0:
            return
        
        # =================================================================
        # 重置脚部累积值
        # =================================================================
        self.avg_feet_force_per_step[env_ids] = 0.0
        self.avg_feet_speed_per_step[env_ids] = 0.0
        
        # =================================================================
        # 日志初始化
        # =================================================================
        self.extras["log"] = dict()
        
        # =================================================================
        # 地形课程学习更新 (如果启用)
        # =================================================================
        if self.cfg.scene.terrain_generator is not None:
            if self.cfg.scene.terrain_generator.curriculum:
                terrain_levels = self.update_terrain_levels(env_ids)
                self.extras["log"].update(terrain_levels)
        
        # =================================================================
        # 重置场景和事件
        # =================================================================
        self.scene.reset(env_ids)
        if "reset" in self.event_manager.available_modes:
            self.event_manager.apply(
                mode="reset",
                env_ids=env_ids,
                dt=self.step_dt,
                global_env_step_count=self.sim_step_counter // self.cfg.sim.decimation,
            )
        
        # =================================================================
        # 重置奖励管理器
        # =================================================================
        reward_extras = self.reward_manager.reset(env_ids)
        self.extras["log"].update(reward_extras)
        self.extras["time_outs"] = self.time_out_buf
        
        # =================================================================
        # 重置各个缓冲区
        # =================================================================
        self.command_generator.reset(env_ids)
        self.actor_obs_buffer.reset(env_ids)
        self.critic_obs_buffer.reset(env_ids)
        self.action_buffer.reset(env_ids)
        self.episode_length_buf[env_ids] = 0
        
        # 重置终止计数器
        self.termination_counter[env_ids] = 0
        
        # =================================================================
        # 同步仿真数据
        # =================================================================
        self.scene.write_data_to_sim()
        self.sim.forward()

    def step(self, actions: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, dict]:
        """
        执行一个环境步长
        
        核心仿真循环:
        1. 应用动作延迟
        2. 多步物理仿真 (decimation 次)
        3. 累积脚部力和速度
        4. 检查终止条件
        5. 计算奖励
        6. 重置终止的环境
        7. 计算观测
        
        Args:
            actions: 策略网络输出的动作 [num_envs, num_actions]
            
        Returns:
            actor_obs: Actor 观测
            reward_buf: 奖励值
            reset_buf: 终止标志
            extras: 额外信息 (日志、 Critic 观测等)
        """
        # =================================================================
        # 动作处理
        # =================================================================
        # 应用动作延迟
        delayed_actions = self.action_buffer.compute(actions)
        
        # 裁剪并转换到设备
        self.action = torch.clip(delayed_actions, -self.clip_actions, self.clip_actions).to(self.device)
        
        # 缩放动作并加上默认位置 -> 目标关节位置
        processed_actions = self.action * self.action_scale + self.robot.data.default_joint_pos
        
        # =================================================================
        # 多步物理仿真
        # =================================================================
        # 初始化脚部力/速度累积
        self.avg_feet_force_per_step = torch.zeros(
            self.num_envs,
            len(self.feet_cfg.body_ids),
            dtype=torch.float,
            device=self.device,
            requires_grad=False
        )
        self.avg_feet_speed_per_step = torch.zeros(
            self.num_envs,
            len(self.feet_cfg.body_ids),
            dtype=torch.float,
            device=self.device,
            requires_grad=False
        )
        
        # 执行 decimation 次物理步骤
        for _ in range(self.cfg.sim.decimation):
            self.sim_step_counter += 1
            
            # 设置关节位置目标
            self.robot.set_joint_position_target(processed_actions)
            
            # 写入仿真器数据
            self.scene.write_data_to_sim()
            
            # 物理仿真一步
            self.sim.step(render=False)
            self.scene.update(dt=self.physics_dt)
            
            # 累积脚部力 (世界坐标系)
            self.avg_feet_force_per_step += torch.norm(
                self.contact_sensor.data.net_forces_w[:, self.feet_cfg.body_ids, :3],
                dim=-1
            )
            
            # 累积脚部速度 (世界坐标系)
            self.avg_feet_speed_per_step += torch.norm(
                self.robot.data.body_lin_vel_w[:, self.feet_body_ids, :],
                dim=-1
            )
        
        # 计算平均值
        self.avg_feet_force_per_step /= self.cfg.sim.decimation
        self.avg_feet_speed_per_step /= self.cfg.sim.decimation
        
        # =================================================================
        # 渲染 (如果非无头模式)
        # =================================================================
        if not self.headless:
            self.sim.render()
        
        # =================================================================
        # 更新回合计数器
        # =================================================================
        self.episode_length_buf += 1
        self._calculate_gait_para()
        
        # =================================================================
        # 命令和事件处理
        # =================================================================
        self.command_generator.compute(self.step_dt)
        if "interval" in self.event_manager.available_modes:
            self.event_manager.apply(mode="interval", dt=self.step_dt)
        
        # =================================================================
        # 终止检查和奖励计算
        # =================================================================
        self.reset_buf, self.time_out_buf = self.check_reset()
        reward_buf = self.reward_manager.compute(self.step_dt)
        self.reset_env_ids = self.reset_buf.nonzero(as_tuple=False).flatten()
        
        # 重置终止的环境
        self.reset(self.reset_env_ids)
        
        # =================================================================
        # 计算观测
        # =================================================================
        actor_obs, critic_obs = self.compute_observations()
        self.extras["observations"] = {"critic": critic_obs}
        
        return actor_obs, reward_buf, self.reset_buf, self.extras

    def check_reset(self) -> tuple[torch.Tensor, torch.Tensor]:
        """
        检查环境是否需要重置
        
        终止条件:
        1. 指定部位的接触力 > 阈值 (如膝盖、肩部、骨盆接触地面)
        2. 回合长度达到最大值 (正常超时)
        3. 基座高度过低 (倒地)
        4. 基座姿态倾斜过大 (侧翻/前翻)
        
        使用持续帧检测机制：终止条件必须连续满足 N 帧才会触发重置，
        以减少误触发导致的频繁重置。
        
        Returns:
            reset_buf: 需要重置的环境标志 [num_envs]
            time_out_buf: 超时环境标志 [num_envs]
        """
        # =================================================================
        # 检查接触力终止 - 使用 mdp.undesired_contacts
        # =================================================================
        contact_term = mdp.undesired_contacts(
            self, threshold=1.0, sensor_cfg=self.termination_contact_cfg
        ) > 0  # [num_envs]
        
        # =================================================================
        # 检查基座高度终止 (倒地检测)
        # =================================================================
        # 直接使用 root_pos_w[:, 2] 获取世界坐标系中的基座高度
        height_term = self.robot.data.root_pos_w[:, 2] < self.cfg.robot.base_height_threshold  # [num_envs]
        
        # =================================================================
        # 检查基座姿态终止 (倾斜检测)
        # =================================================================
        # 使用 projected_gravity_b 计算与垂直方向的夹角
        # 重力向量在基座坐标系投影，机器直立时为 (0, 0, -1)
        # torch.acos(-projected_gravity_b[:, 2]) 得到倾斜角度 (弧度)
        base_orientation_threshold = self.cfg.robot.base_orientation_threshold
        tilt_angle = torch.acos(torch.clamp(-self.robot.data.projected_gravity_b[:, 2], -1.0, 1.0))
        orientation_term = tilt_angle > base_orientation_threshold  # [num_envs]
        
        # =================================================================
        # 合并所有立即终止条件 (如倒地高度为0时直接终止)
        # =================================================================
        immediate_term = height_term  # 高度过低立即终止
        
        # 需要持续检测的条件
        delayed_term = contact_term | orientation_term  # 接触力和姿态需要持续触发
        
        # =================================================================
        # 持续帧检测机制
        # =================================================================
        # 持续触发计数 +1，未触发则清零
        self.termination_counter[delayed_term] += 1
        self.termination_counter[~delayed_term] = 0
        
        # 只有连续 N 帧都触发才终止
        delayed_reset = self.termination_counter >= self.termination_consecutive_frames
        
        # 合并立即终止和持续检测终止
        reset_buf = immediate_term | delayed_reset
        
        # =================================================================
        # 检查超时终止
        # =================================================================
        time_out_buf = self.episode_length_buf >= self.max_episode_length
        
        # 调试打印
        # if self.sim_step_counter % 100 == 0:
        #     print(f"[DEBUG] step={self.sim_step_counter}, reset={reset_buf.sum().item()}/{len(reset_buf)}, "
        #           f"contact={contact_term.sum().item()}, height={height_term.sum().item()}, "
        #           f"orient={orientation_term.sum().item()}, base_h={self.robot.data.root_pos_w[:, 2].mean().item():.3f}")
        
        reset_buf = reset_buf | time_out_buf
        
        return reset_buf, time_out_buf

    def init_obs_buffer(self) -> None:
        """
        初始化观测噪声配置和历史缓冲区
        
        创建 CircularBuffer 用于存储历史观测序列。
        配置各观测维度的噪声缩放因子。
        """
        if self.add_noise:
            # 获取观测维度以创建噪声向量
            actor_obs, _ = self.compute_current_observations()
            noise_vec = torch.zeros_like(actor_obs[0])
            
            noise_scales = self.cfg.noise.noise_scales
            
            # =================================================================
            # 为各观测维度设置噪声缩放
            # 噪声 = (2 * rand - 1) * noise_scale * obs_scale
            # 范围: [-noise_scale * obs_scale, +noise_scale * obs_scale]
            # =================================================================
            
            # 注意: 噪声索引必须与 compute_current_observations 的拼接顺序完全一致
            # 观测布局: [ang_vel(3), gravity(3), command(3), joint_pos(N), joint_vel(N), action(N), gait(6)]
            
            # [0:3] 角速度
            noise_vec[0:3] = noise_scales.ang_vel * self.obs_scales.ang_vel
            
            # [3:6] 重力投影
            noise_vec[3:6] = noise_scales.projected_gravity * self.obs_scales.projected_gravity
            
            # [6:9] 速度命令
            noise_vec[6:9] = 0.0
            
            N = self.num_actions  # 28
            pos_start = 9  # 关节数据起始索引 (3+3+3)
            
            # [9:9+N] 关节位置
            noise_vec[pos_start : pos_start + N] = noise_scales.joint_pos * self.obs_scales.joint_pos
            
            # [9+N:9+2N] 关节速度
            noise_vec[pos_start + N : pos_start + 2 * N] = noise_scales.joint_vel * self.obs_scales.joint_vel
            
            # [9+2N:9+3N] 上一动作 (不清零)
            noise_vec[pos_start + 2 * N : pos_start + 3 * N] = 0.0
            
            # [9+3N:9+3N+6] 步态相位 sin/cos/ratio (不清零)
            gait_start = pos_start + 3 * N
            noise_vec[gait_start : gait_start + 6] = 0.0
            
            self.noise_scale_vec = noise_vec
            
            # =================================================================
            # 高度扫描噪声配置 (如果启用)
            # =================================================================
            if self.cfg.scene.height_scanner.enable_height_scan:
                height_scan = (
                    self.height_scanner.data.pos_w[:, 2].unsqueeze(1)
                    - self.height_scanner.data.ray_hits_w[..., 2]
                    - self.cfg.normalization.height_scan_offset
                )
                height_scan_noise_vec = torch.zeros_like(height_scan[0])
                height_scan_noise_vec[:] = noise_scales.height_scan * self.obs_scales.height_scan
                self.height_scan_noise_vec = height_scan_noise_vec
        
        # =================================================================
        # 创建历史观测缓冲区
        # =================================================================
        # Actor 观测历史缓冲区
        self.actor_obs_buffer = CircularBuffer(
            max_len=self.cfg.robot.actor_obs_history_length,
            batch_size=self.num_envs,
            device=self.device
        )
        
        # Critic 观测历史缓冲区
        self.critic_obs_buffer = CircularBuffer(
            max_len=self.cfg.robot.critic_obs_history_length,
            batch_size=self.num_envs,
            device=self.device
        )

    def update_terrain_levels(self, env_ids: torch.Tensor) -> dict:
        """
        更新地形难度等级 (课程学习)
        
        根据机器人在地形上的位置动态调整地形难度:
        - 移动到地形边界外 -> 升级 (更难的地形)
        - 移动到地形中心且完成目标 -> 降级 (更容易的地形)
        
        Args:
            env_ids: 需要检查的环境索引
            
        Returns:
            包含当前地形等级的日志字典
        """
        # 计算环境原点到机器人根部的水平距离
        distance = torch.norm(
            self.robot.data.root_pos_w[env_ids, :2] - self.scene.env_origins[env_ids, :2],
            dim=1
        )
        
        # 判断是否移动到地形边界外 -> 升级
        move_up = distance > self.scene.terrain.cfg.terrain_generator.size[0] / 2
        
        # 判断是否在中心且速度慢 -> 降级
        move_down = (
            distance < torch.norm(self.command_generator.command[env_ids, :2], dim=1)
            * self.max_episode_length_s
            * 0.5
        )
        
        # 互斥条件
        move_down *= ~move_up
        
        # 更新地形原点
        self.scene.terrain.update_env_origins(env_ids, move_up, move_down)
        
        # 返回日志
        extras = {}
        extras["Curriculum/terrain_levels"] = torch.mean(self.scene.terrain.terrain_levels.float())
        return extras

    def get_observations(self) -> tuple[torch.Tensor, dict]:
        """
        获取当前观测 (VecEnv 接口)
        
        Returns:
            actor_obs: Actor 观测
            extras: 包含 Critic 观测的字典
        """
        actor_obs, critic_obs = self.compute_observations()
        self.extras["observations"] = {"critic": critic_obs}
        return actor_obs, self.extras

    def get_amp_obs_for_expert_trans(self) -> torch.Tensor:
        """
        获取 AMP 专家观测 (用于 AMP 判别器)
        
        与 compute_current_observations 类似，但输出格式适配 AMP:
        - 关节位置/速度
        - 手部/脚部相对位置 (相对于根部)
        
        Returns:
            AMP 观测张量 [num_envs, amp_obs_dim]
        """
        # =================================================================
        # 计算手部位置 (相对于根部)
        # =================================================================
        # 左手位置
        left_hand_pos = (
            self.robot.data.body_state_w[:, self.elbow_body_ids[0], :3]
            - self.robot.data.root_state_w[:, 0:3]
            + quat_rotate(self.robot.data.body_state_w[:, self.elbow_body_ids[0], 3:7], self.left_arm_local_vec)
        )
        right_hand_pos = (
            self.robot.data.body_state_w[:, self.elbow_body_ids[1], :3]
            - self.robot.data.root_state_w[:, 0:3]
            + quat_rotate(self.robot.data.body_state_w[:, self.elbow_body_ids[1], 3:7], self.right_arm_local_vec)
        )
        
        # 转换到根部局部坐标系
        left_hand_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), left_hand_pos)
        right_hand_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), right_hand_pos)
        
        # =================================================================
        # 计算脚部位置 (相对于根部)
        # =================================================================
        left_foot_pos = (
            self.robot.data.body_state_w[:, self.feet_body_ids[0], :3]
            - self.robot.data.root_state_w[:, 0:3]
        )
        right_foot_pos = (
            self.robot.data.body_state_w[:, self.feet_body_ids[1], :3]
            - self.robot.data.root_state_w[:, 0:3]
        )
        left_foot_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), left_foot_pos)
        right_foot_pos = quat_apply(quat_conjugate(self.robot.data.root_state_w[:, 3:7]), right_foot_pos)
        
        # =================================================================
        # 计算关节位置/速度 (用于 AMP 68维格式)
        # 格式顺序与 Jingchu01AMPLoader 一致:
        # [0:6]   左腿: hip_roll, hip_yaw, hip_pitch, knee_pitch, ankle_pitch, ankle_roll
        # [6:12]  右腿: hip_roll, hip_yaw, hip_pitch, knee_pitch, ankle_pitch, ankle_roll
        # [12:14] 腰部: waist_roll, waist_yaw
        # [14:21] 左臂: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
        # [21:28] 右臂: shoulder_pitch, shoulder_roll, shoulder_yaw, elbow_pitch, elbow_yaw, wrist_pitch, wrist_roll
        # =================================================================
        # 获取实际关节数据
        left_leg_pos = self.robot.data.joint_pos[:, self.left_leg_ids]
        right_leg_pos = self.robot.data.joint_pos[:, self.right_leg_ids]
        left_leg_vel = self.robot.data.joint_vel[:, self.left_leg_ids]
        right_leg_vel = self.robot.data.joint_vel[:, self.right_leg_ids]
        
        waist_pos = self.robot.data.joint_pos[:, self.waist_ids]
        waist_vel = self.robot.data.joint_vel[:, self.waist_ids]
        
        left_arm_pos = self.robot.data.joint_pos[:, self.left_arm_ids]
        right_arm_pos = self.robot.data.joint_pos[:, self.right_arm_ids]
        left_arm_vel = self.robot.data.joint_vel[:, self.left_arm_ids]
        right_arm_vel = self.robot.data.joint_vel[:, self.right_arm_ids]
        
        # =================================================================
        # 拼接所有数据 (68 维 AMP 格式)
        # 顺序: [左腿位置, 右腿位置, 腰部位置, 左臂位置, 右臂位置,
        #        左腿速度, 右腿速度, 腰部速度, 左臂速度, 右臂速度,
        #        左手位置, 右手位置, 左脚位置, 右脚位置]
        # =================================================================
        return torch.cat(
            (
                left_leg_pos,          # 6
                right_leg_pos,         # 6
                waist_pos,             # 2
                left_arm_pos,          # 7
                right_arm_pos,         # 7
                left_leg_vel,          # 6
                right_leg_vel,         # 6
                waist_vel,             # 2
                left_arm_vel,          # 7
                right_arm_vel,         # 7
                left_hand_pos,         # 3
                right_hand_pos,        # 3
                left_foot_pos,         # 3
                right_foot_pos         # 3
            ),
            dim=-1,
        )
         # 格式: [右腿关节, 左腿关节, 右腿速度, 左腿速度, 左脚位置, 右脚位置]
        # return torch.cat(
        #     (
        #         self.right_leg_dof_pos,
        #         self.left_leg_dof_pos,
        #         self.right_leg_dof_vel,
        #         self.left_leg_dof_vel,
        #         left_foot_pos,
        #         right_foot_pos
        #     ),
        #     dim=-1,
        # )

    @staticmethod
    def seed(seed: int = -1) -> int:
        """
        设置随机种子
        
        尝试同时设置 Omni 和 PyTorch 的随机种子。
        
        Args:
            seed: 随机种子值，-1 表示使用默认种子
            
        Returns:
            最终使用的随机种子值
        """
        try:
            import omni.replicator.core as rep  # type: ignore
            rep.set_global_seed(seed)
        except ModuleNotFoundError:
            pass
        return torch_utils.set_seed(seed)

    def _calculate_gait_para(self) -> None:
        """
        更新步态相位参数
        
        根据仿真时间和步态周期计算各环境的步态相位。
        相位范围: [0, 1)，0 表示周期开始，1 表示周期结束。
        
        公式: phase = (t / cycle + offset) % 1.0
        
        其中:
        - t = episode_step * step_dt
        - offset = 用户配置的相位偏移
        """
        # 计算归一化时间 t = 仿真时间 / 步态周期
        t = self.episode_length_buf * self.step_dt / self.gait_cycle
        
        # 计算左腿相位 (带偏移)
        self.gait_phase[:, 0] = (t + self.phase_offset[:, 0]) % 1.0
        
        # 计算右腿相位 (带偏移)
        self.gait_phase[:, 1] = (t + self.phase_offset[:, 1]) % 1.0
