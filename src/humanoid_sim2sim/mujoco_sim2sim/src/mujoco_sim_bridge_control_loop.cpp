#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::enforceBaseLock()
{
    if (!shouldEnforceBaseLock() || !fixed_base_pose_initialized_ || base_free_qpos_adr_ < 0 || base_free_qvel_adr_ < 0)
    {
        return;
    }
    if ((base_free_qpos_adr_ + 6) >= model_->nq || (base_free_qvel_adr_ + 5) >= model_->nv)
    {
        return;
    }

    for (size_t i = 0; i < fixed_base_qpos_.size(); ++i)
    {
        data_->qpos[base_free_qpos_adr_ + static_cast<int>(i)] = fixed_base_qpos_[i];
    }
    for (int i = 0; i < 6; ++i)
    {
        data_->qvel[base_free_qvel_adr_ + i] = 0.0;
    }
}

int MujocoSimBridge::prepareModeControlWordForTick(int raw_control_word)
{
    const rl_master::DecodedControlWord decoded =
        rl_master::DeployStateMachine::decodeControlWord(
            raw_control_word,
            controller_state_initialized_ ? last_controller_mode_id_ : startup_mode_id_);

    if (decoded.request_estop)
    {
        release_settle_ticks_remaining_ = 0;
        zeroing_injection_pending_ = false;
        return raw_control_word;
    }

    if (decoded.request_zero && enable_fixed_base_zeroing_)
    {
        activateDynamicBaseLock(BaseLockReason::kExplicitZeroing, enable_prepose_snap_);
        release_settle_ticks_remaining_ = 0;
        return raw_control_word;
    }

    if (zeroing_injection_pending_ && enable_fixed_base_zeroing_)
    {
        activateDynamicBaseLock(BaseLockReason::kIncompatibleSwitchZeroing, enable_prepose_snap_);
        zeroing_injection_pending_ = false;
        release_settle_ticks_remaining_ = 0;
        return rl_master::kCtrlWordZeroing;
    }

    if (!controller_state_initialized_)
    {
        return raw_control_word;
    }

    const bool active_mode_needs_zeroing =
        last_completed_zeroing_mode_id_ !=
        (decoded.request_start ? decoded.locomotion_mode : last_controller_mode_id_);
    const bool current_state_is_hold =
        last_controller_deploy_state_ == rl_master::DeployLifecycleState::kHold;
    const bool current_state_is_zeroing =
        last_controller_deploy_state_ == rl_master::DeployLifecycleState::kZeroing;

    if (enable_fixed_base_zeroing_ &&
        decoded.request_start &&
        current_state_is_hold &&
        active_mode_needs_zeroing)
    {
        activateDynamicBaseLock(BaseLockReason::kIncompatibleSwitchZeroing, enable_prepose_snap_);
        zeroing_injection_pending_ = true;
        release_settle_ticks_remaining_ = 0;
        return rl_master::kCtrlWordSetModeBase + decoded.locomotion_mode;
    }

    if (decoded.request_start &&
        dynamic_base_lock_active_ &&
        current_state_is_zeroing)
    {
        return rl_master::kCtrlWordStopPolicy;
    }

    if (decoded.request_start &&
        hold_settle_ticks_remaining_ > 0)
    {
        return rl_master::kCtrlWordStopPolicy;
    }

    if (decoded.request_start &&
        enable_release_before_running_ &&
        dynamic_base_lock_active_ &&
        dynamic_base_lock_reason_ == BaseLockReason::kPreRunHold &&
        current_state_is_hold)
    {
        if (release_settle_ticks_remaining_ <= 0)
        {
            deactivateDynamicBaseLock("pre-running release");
            release_settle_ticks_remaining_ = post_release_settle_ticks_;
        }
        if (release_settle_ticks_remaining_ > 0)
        {
            return rl_master::kCtrlWordStopPolicy;
        }
    }

    if (release_settle_ticks_remaining_ > 0 && decoded.request_start)
    {
        return rl_master::kCtrlWordStopPolicy;
    }

    return raw_control_word;
}

void MujocoSimBridge::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    latest_teleop_command_.vx = static_cast<float>(msg->linear.x);
    latest_teleop_command_.vy = static_cast<float>(msg->linear.y);
    latest_teleop_command_.dyaw = static_cast<float>(msg->angular.z);
}

void MujocoSimBridge::modeControlCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }
    if (!rl_master::DeployStateMachine::isValidControlWord(msg->data))
    {
        if ((this->now() - last_mode_warn_).seconds() > 1.0)
        {
            RCLCPP_WARN(this->get_logger(), "Ignore invalid mode control word: %d", msg->data);
            last_mode_warn_ = this->now();
        }
        return;
    }
    mode_command_cache_.store(msg->data);
}

void MujocoSimBridge::controlLoopTick()
{
    const auto loop_begin = std::chrono::steady_clock::now();
    const rclcpp::Time now = this->now();
    const bool should_step = !viewer_paused_ || viewer_step_once_;
    const int speed_substeps = std::max(1, static_cast<int>(std::lround(substeps_per_control_ * sim_speed_scale_)));
    const int raw_mode_control_word = mode_command_cache_.load();
    const int effective_mode_control_word = prepareModeControlWordForTick(raw_mode_control_word);

    const rl_master::RobotStateData state = buildRobotState();
    const rl_master::TeleopCommand teleop_command = latestTeleopCommand();
    const rl_master::RobotCommandData command =
        controller_runtime_.step(state, teleop_command, effective_mode_control_word);
    const auto &controller_snapshot = controller_runtime_.controller().latestLogSnapshot();
    emitDerivedRuntimeEvents(controller_snapshot);
    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(true, command.open_rl);
    const bool control_active = runtime_mode.open_rl_active;
    const auto controller_state = static_cast<rl_master::DeployLifecycleState>(controller_snapshot.deploy_state);
    const int controller_mode_id = controller_snapshot.active_mode_id;

    if (controller_state == rl_master::DeployLifecycleState::kZeroing &&
        enable_fixed_base_zeroing_)
    {
        if (!dynamic_base_lock_active_)
        {
            activateDynamicBaseLock(BaseLockReason::kExplicitZeroing, false);
        }
    }

    if (controller_state_initialized_)
    {
        if (last_controller_deploy_state_ == rl_master::DeployLifecycleState::kZeroing &&
            controller_state != rl_master::DeployLifecycleState::kZeroing)
        {
            last_completed_zeroing_mode_id_ = controller_mode_id;
            if (controller_state == rl_master::DeployLifecycleState::kHold &&
                enable_fixed_base_hold_after_zeroing_)
            {
                dynamic_base_lock_active_ = true;
                dynamic_base_lock_reason_ = BaseLockReason::kPreRunHold;
            }
            if (controller_state == rl_master::DeployLifecycleState::kHold)
            {
                hold_settle_ticks_remaining_ = post_zeroing_hold_settle_ticks_;
                hold_target_latched_ = false;
                if (hold_settle_ticks_remaining_ > 0)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Post-zeroing hold settle active for %d ticks before latching HOLD target.",
                        hold_settle_ticks_remaining_);
                }
                if (!enable_fixed_base_hold_after_zeroing_ && !fix_base_)
                {
                    deactivateDynamicBaseLock("zeroing completed into hold without fixed-base hold");
                }
            }
            else if (controller_state == rl_master::DeployLifecycleState::kRunning &&
                     !fix_base_)
            {
                deactivateDynamicBaseLock("zeroing completed into running");
            }
        }

        if (last_controller_deploy_state_ == rl_master::DeployLifecycleState::kRunning &&
            controller_state == rl_master::DeployLifecycleState::kHold &&
            controller_mode_id != last_controller_mode_id_ &&
            enable_fixed_base_zeroing_)
        {
            activateDynamicBaseLock(BaseLockReason::kIncompatibleSwitchZeroing, false);
            zeroing_injection_pending_ = true;
        }

        if (last_controller_deploy_state_ != rl_master::DeployLifecycleState::kRunning &&
            controller_state == rl_master::DeployLifecycleState::kRunning)
        {
            running_start_reference_sync_pending_ = sim_sync_running_start_to_reference_;
        }
    }
    if (controller_state != rl_master::DeployLifecycleState::kRunning)
    {
        running_start_reference_sync_pending_ = false;
    }
    if (controller_state != rl_master::DeployLifecycleState::kHold &&
        hold_settle_ticks_remaining_ > 0)
    {
        hold_settle_ticks_remaining_ = 0;
    }

    const bool hold_settle_active = (hold_settle_ticks_remaining_ > 0);
    if (!control_active)
    {
        if (!hold_settle_active && !hold_target_latched_)
        {
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                const int qpos_adr = qpos_addrs_[i];
                if (qpos_adr >= 0 && qpos_adr < model_->nq)
                {
                    last_target_q_[i] = static_cast<float>(data_->qpos[qpos_adr]);
                }
            }
            for (size_t i = 0; i < hold_qpos_addrs_.size(); ++i)
            {
                const int qpos_adr = hold_qpos_addrs_[i];
                if (qpos_adr >= 0 && qpos_adr < model_->nq)
                {
                    latched_hold_target_q_[i] = data_->qpos[qpos_adr];
                }
            }
            hold_target_latched_ = true;
            RCLCPP_INFO(this->get_logger(), "Controller inactive, latch current pose for hold behavior.");
        }
    }
    else
    {
        hold_target_latched_ = false;
    }

    (void)maybeApplyRunningStartReferenceSync(controller_snapshot);
    enforceBaseLock();
    updateControlInput(command, control_active, now);

    const bool allow_inactive_step_for_release = (release_settle_ticks_remaining_ > 0);
    const bool allow_inactive_step_for_hold_settle = (hold_settle_ticks_remaining_ > 0);
    if (should_step &&
        (control_active || !pause_when_no_command_ || allow_inactive_step_for_release ||
         allow_inactive_step_for_hold_settle))
    {
        for (int i = 0; i < speed_substeps; ++i)
        {
            enforceBaseLock();
            mj_step(model_, data_);
            enforceBaseLock();
        }
        mj_forward(model_, data_);
        viewer_step_once_ = false;
        if (release_settle_ticks_remaining_ > 0)
        {
            --release_settle_ticks_remaining_;
        }
        if (hold_settle_ticks_remaining_ > 0)
        {
            --hold_settle_ticks_remaining_;
        }
    }
    else
    {
        enforceBaseLock();
        mj_forward(model_, data_);
    }

    const rl_master::RobotStateData post_state = buildRobotState();
    updateMirroredState(post_state);
    emitBaseImuSourceSample(post_state, rl_master::monotonicTimeSec());
    logLoopData(state, post_state, command, controller_snapshot, runtime_mode, control_active);
    updateViewerFrameMirror();
    updateViewerInspectorMirror(post_state, command, runtime_mode);
    renderViewerFrame();

    const auto loop_end = std::chrono::steady_clock::now();
    const auto loop_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_begin).count();
    const double budget_us = 1.0e6 / std::max(1.0, control_hz_);
    if (static_cast<double>(loop_elapsed_us) > budget_us)
    {
        ++sim_loop_overrun_count_;
    }

    if (controller_snapshot.valid)
    {
        last_controller_mode_id_ = controller_mode_id;
        last_controller_deploy_state_ = controller_state;
        controller_state_initialized_ = true;
    }
}

rl_master::RobotStateData MujocoSimBridge::buildRobotState() const
{
    const Sim2realCfg &runtime_cfg = controller_runtime_.runtimeCfg();
    std::string quat_source_order = runtime_cfg.source_contract.sim_base.quat_source_order;
    std::transform(quat_source_order.begin(), quat_source_order.end(), quat_source_order.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::string velocity_source = runtime_cfg.source_contract.sim_base.velocity_source;
    std::transform(velocity_source.begin(), velocity_source.end(), velocity_source.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (quat_source_order != "wxyz")
    {
        throw std::runtime_error(
            "MuJoCo sim base quaternion source order must be 'wxyz', got '" +
            runtime_cfg.source_contract.sim_base.quat_source_order + "'");
    }
    if (velocity_source != "freejoint_qvel" &&
        velocity_source != "body_object_velocity_local" &&
        velocity_source != "body_cvel")
    {
        throw std::runtime_error(
            "MuJoCo sim base velocity source must be 'freejoint_qvel', "
            "'body_object_velocity_local', or 'body_cvel', got '" +
            runtime_cfg.source_contract.sim_base.velocity_source + "'");
    }

    rl_master::RobotStateData state;
    state.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    state.active_joint_count = static_cast<int>(joint_names_.size());
    state.joint_q.assign(joint_names_.size(), 0.0f);
    state.joint_dq.assign(joint_names_.size(), 0.0f);
    state.joint_tau.assign(joint_names_.size(), 0.0f);

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        const int qvel_adr = qvel_addrs_[i];

        if (qpos_adr >= 0 && qpos_adr < model_->nq)
        {
            state.joint_q[i] = static_cast<float>(data_->qpos[qpos_adr]);
        }
        if (qvel_adr >= 0 && qvel_adr < model_->nv)
        {
            state.joint_dq[i] = static_cast<float>(data_->qvel[qvel_adr]);
        }
        state.joint_tau[i] = applied_tau_[i];
    }

    std::array<float, 4> base_quat_xyzw{0.0f, 0.0f, 0.0f, 1.0f};

    if (velocity_source == "freejoint_qvel" &&
        base_free_qvel_adr_ >= 0 && (base_free_qvel_adr_ + 5) < model_->nv)
    {
        state.base_lin_vel[0] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 0]);
        state.base_lin_vel[1] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 1]);
        state.base_lin_vel[2] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 2]);
        state.base_ang_vel[0] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 3]);
        state.base_ang_vel[1] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 4]);
        state.base_ang_vel[2] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 5]);
    }
    else if (velocity_source == "body_object_velocity_local" &&
             base_body_id_ >= 0 && base_body_id_ < model_->nbody)
    {
        mjtNum vel6_local[6] = {0, 0, 0, 0, 0, 0};
        mj_objectVelocity(model_, data_, mjOBJ_BODY, base_body_id_, vel6_local, 1);
        state.base_ang_vel[0] = static_cast<float>(vel6_local[0]);
        state.base_ang_vel[1] = static_cast<float>(vel6_local[1]);
        state.base_ang_vel[2] = static_cast<float>(vel6_local[2]);
        state.base_lin_vel[0] = static_cast<float>(vel6_local[3]);
        state.base_lin_vel[1] = static_cast<float>(vel6_local[4]);
        state.base_lin_vel[2] = static_cast<float>(vel6_local[5]);
    }
    else if (velocity_source == "body_cvel" &&
             base_body_id_ >= 0 && base_body_id_ < model_->nbody && data_->cvel)
    {
        const mjtNum *cvel = data_->cvel + 6 * base_body_id_;
        state.base_ang_vel[0] = static_cast<float>(cvel[0]);
        state.base_ang_vel[1] = static_cast<float>(cvel[1]);
        state.base_ang_vel[2] = static_cast<float>(cvel[2]);
        state.base_lin_vel[0] = static_cast<float>(cvel[3]);
        state.base_lin_vel[1] = static_cast<float>(cvel[4]);
        state.base_lin_vel[2] = static_cast<float>(cvel[5]);
    }
    else if (base_free_qvel_adr_ >= 0 && (base_free_qvel_adr_ + 5) < model_->nv)
    {
        state.base_lin_vel[0] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 0]);
        state.base_lin_vel[1] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 1]);
        state.base_lin_vel[2] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 2]);
        state.base_ang_vel[0] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 3]);
        state.base_ang_vel[1] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 4]);
        state.base_ang_vel[2] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 5]);
    }
    else if (base_body_id_ >= 0 && base_body_id_ < model_->nbody)
    {
        mjtNum vel6_local[6] = {0, 0, 0, 0, 0, 0};
        mj_objectVelocity(model_, data_, mjOBJ_BODY, base_body_id_, vel6_local, 1);
        state.base_ang_vel[0] = static_cast<float>(vel6_local[0]);
        state.base_ang_vel[1] = static_cast<float>(vel6_local[1]);
        state.base_ang_vel[2] = static_cast<float>(vel6_local[2]);
        state.base_lin_vel[0] = static_cast<float>(vel6_local[3]);
        state.base_lin_vel[1] = static_cast<float>(vel6_local[4]);
        state.base_lin_vel[2] = static_cast<float>(vel6_local[5]);
    }
    else if (base_body_id_ >= 0 && base_body_id_ < model_->nbody && data_->cvel)
    {
        const mjtNum *cvel = data_->cvel + 6 * base_body_id_;
        state.base_ang_vel[0] = static_cast<float>(cvel[0]);
        state.base_ang_vel[1] = static_cast<float>(cvel[1]);
        state.base_ang_vel[2] = static_cast<float>(cvel[2]);
        state.base_lin_vel[0] = static_cast<float>(cvel[3]);
        state.base_lin_vel[1] = static_cast<float>(cvel[4]);
        state.base_lin_vel[2] = static_cast<float>(cvel[5]);
    }

    if (base_free_qpos_adr_ >= 0 && (base_free_qpos_adr_ + 6) < model_->nq)
    {
        state.base_pos_w[0] = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 0]);
        state.base_pos_w[1] = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 1]);
        state.base_pos_w[2] = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 2]);
        const float qw = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 3]);
        const float qx = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 4]);
        const float qy = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 5]);
        const float qz = static_cast<float>(data_->qpos[base_free_qpos_adr_ + 6]);
        base_quat_xyzw = {qx, qy, qz, qw};
    }
    else if (base_body_id_ >= 0 && base_body_id_ < model_->nbody && data_->xquat)
    {
        const mjtNum *q = data_->xquat + 4 * base_body_id_;
        const float qw = static_cast<float>(q[0]);
        const float qx = static_cast<float>(q[1]);
        const float qy = static_cast<float>(q[2]);
        const float qz = static_cast<float>(q[3]);
        base_quat_xyzw = {qx, qy, qz, qw};
    }

    state.base_quat = base_quat_xyzw;
    state.base_rpy = quatXyzwToRpy(base_quat_xyzw);
    return state;
}

void MujocoSimBridge::updateControlInput(
    const rl_master::RobotCommandData &command,
    bool control_active,
    rclcpp::Time now)
{
    const Sim2realCfg &active_cfg = controller_runtime_.runtimeCfg();
    resolvePerJointControlConfig(controller_runtime_.activeModeId());
    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(true, command.open_rl);
    const bool mode_policy = runtime_mode.mode == rl_master::CommandRuntimeMode::kPolicy;

    auto commandQAt = [&](size_t idx) -> double {
        return idx < command.joint_target_q.size() ? static_cast<double>(command.joint_target_q[idx]) : 0.0;
    };
    auto commandDqAt = [&](size_t idx) -> double {
        return idx < command.joint_target_dq.size() ? static_cast<double>(command.joint_target_dq[idx]) : 0.0;
    };
    auto commandTauAt = [&](size_t idx) -> double {
        return idx < command.joint_target_tau.size() ? static_cast<double>(command.joint_target_tau[idx]) : 0.0;
    };
    auto isPolicyControlledJoint = [&](size_t idx) -> bool {
        if (idx >= joint_names_.size())
        {
            return false;
        }
        const std::string &joint_name = joint_names_[idx];

        if (active_cfg.action_joint_order.empty())
        {
            return false;
        }
        return std::find(
                   active_cfg.action_joint_order.begin(),
                   active_cfg.action_joint_order.end(),
                   joint_name) != active_cfg.action_joint_order.end();
    };

    if (runtime_mode.unknown_open_rl_mode &&
        (now - last_mode_warn_).seconds() > 1.0)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Unknown open_rl mode %.2f in fused sim bridge, fallback to inactive hold behavior.",
            static_cast<double>(command.open_rl));
        last_mode_warn_ = now;
    }

    const bool inactive_hold_position = !control_active && (no_command_behavior_ == "hold_position");
    const bool inactive_zero_torque = !control_active && (no_command_behavior_ == "zero_torque");
    const bool allow_hold_latch_targets = !control_active && (hold_settle_ticks_remaining_ <= 0);
    if (inactive_hold_position &&
        !use_mixed_actuator_control_ &&
        !use_position_actuator_control_ &&
        !warned_idle_position_fallback_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "inactive behavior hold_position requested, but actuator mode is torque. "
            "fallback to torque PD hold-last.");
        warned_idle_position_fallback_ = true;
    }

    auto streamModeToSimMode = [&](rl_master::CommandRuntimeMode mode) {
        if (mode == rl_master::CommandRuntimeMode::kTestCst)
        {
            return SimJointRuntimeMode::kCst;
        }
        if (mode == rl_master::CommandRuntimeMode::kTestR1)
        {
            return SimJointRuntimeMode::kR1;
        }
        return SimJointRuntimeMode::kCsp;
    };

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        const int qvel_adr = qvel_addrs_[i];
        const int actuator_id = actuator_ids_[i];
        if (qpos_adr < 0 || qvel_adr < 0 || actuator_id < 0)
        {
            continue;
        }
        if (qpos_adr >= model_->nq || qvel_adr >= model_->nv || actuator_id >= model_->nu)
        {
            continue;
        }

        const double q = data_->qpos[qpos_adr];
        const double dq = data_->qvel[qvel_adr];
        const bool policy_controlled_joint =
            i < joint_is_policy_controlled_.size() ? joint_is_policy_controlled_[i] : isPolicyControlledJoint(i);
        const int hold_cfg_idx =
            (i < joint_hold_config_indices_.size()) ? joint_hold_config_indices_[i] : -1;
        const ActuatorBackend actuator_backend =
            i < joint_actuator_backends_.size() ? joint_actuator_backends_[i]
                                                : (use_position_actuator_control_ ? ActuatorBackend::kPosition
                                                                                  : ActuatorBackend::kTorque);

        double q_des = static_cast<double>(last_target_q_[i]);
        double dq_des = 0.0;
        double tau_ff = 0.0;
        double tau_cmd = 0.0;
        SimJointRuntimeMode joint_mode = SimJointRuntimeMode::kCsp;
        bool forced_policy_csp = false;

        if (!control_active)
        {
            joint_mode = SimJointRuntimeMode::kCsp;
        }
        else if (mode_policy)
        {
            joint_mode =
                (i < resolved_joint_runtime_modes_.size()) ? resolved_joint_runtime_modes_[i] : SimJointRuntimeMode::kCsp;
            if (sim_only_force_policy_csp_ &&
                policy_controlled_joint &&
                joint_mode == SimJointRuntimeMode::kCst)
            {
                joint_mode = SimJointRuntimeMode::kCsp;
                forced_policy_csp = true;
            }
            if (policy_controlled_joint)
            {
                q_des = commandQAt(i);
                if (joint_mode == SimJointRuntimeMode::kR1)
                {
                    dq_des = commandDqAt(i);
                    tau_ff = use_command_torque_ff_ ? commandTauAt(i) : 0.0;
                    tau_cmd = commandTauAt(i);
                }
                else if (joint_mode == SimJointRuntimeMode::kCst)
                {
                    tau_cmd = commandTauAt(i);
                }
            }
            else
            {
                if (hold_cfg_idx >= 0 && i < resolved_hold_target_q_.size())
                {
                    q_des = static_cast<double>(resolved_hold_target_q_[i]);
                }
            }
        }
        else
        {
            joint_mode = streamModeToSimMode(runtime_mode.mode);
            if (joint_mode != SimJointRuntimeMode::kCst)
            {
                q_des = commandQAt(i);
            }
            if (joint_mode == SimJointRuntimeMode::kR1)
            {
                dq_des = commandDqAt(i);
                tau_ff = use_command_torque_ff_ ? commandTauAt(i) : 0.0;
                tau_cmd = commandTauAt(i);
            }
            else if (joint_mode == SimJointRuntimeMode::kCst)
            {
                tau_cmd = commandTauAt(i);
            }
        }

        if (control_active)
        {
            last_target_q_[i] = static_cast<float>(q_des);
        }
        joint_cmd_q_[i] = static_cast<float>(q_des);
        joint_cmd_dq_[i] = static_cast<float>(dq_des);
        joint_cmd_tau_[i] = static_cast<float>(tau_cmd);
        joint_cmd_mode_[i] = static_cast<float>(joint_mode);

        if (inactive_zero_torque)
        {
            data_->ctrl[actuator_id] = (actuator_backend == ActuatorBackend::kPosition) ? q : 0.0;
            applied_tau_[i] = 0.0f;
            continue;
        }

        if (actuator_backend == ActuatorBackend::kPosition)
        {
            if (joint_mode == SimJointRuntimeMode::kCst)
            {
                q_des = q;
            }
            data_->ctrl[actuator_id] = q_des;
            applied_tau_[i] = 0.0f;
        }
        else
        {
            double kp = kp_[i];
            double kd = kd_[i];
            double torque_limit = torque_limit_[i];
            if (forced_policy_csp &&
                i < resolved_policy_profile_kp_.size() &&
                i < resolved_policy_profile_kd_.size() &&
                i < resolved_policy_profile_torque_limit_.size())
            {
                kp = resolved_policy_profile_kp_[i];
                kd = resolved_policy_profile_kd_[i];
                torque_limit = resolved_policy_profile_torque_limit_[i];
            }

            double tau = 0.0;
            if (control_active && joint_mode == SimJointRuntimeMode::kCst)
            {
                tau = tau_cmd;
            }
            else
            {
                tau = kp * (q_des - q) + kd * (dq_des - dq);
                if (control_active &&
                    joint_mode == SimJointRuntimeMode::kR1 &&
                    policy_controlled_joint)
                {
                    tau += tau_ff;
                }
            }
            const double limit = std::max(1e-6, std::abs(torque_limit));
            tau = std::clamp(tau, -limit, limit);

            data_->ctrl[actuator_id] = tau;
            applied_tau_[i] = static_cast<float>(tau);
        }
    }

    for (size_t i = 0; i < hold_qpos_addrs_.size(); ++i)
    {
        const int qpos_adr = hold_qpos_addrs_[i];
        const int qvel_adr = hold_qvel_addrs_[i];
        const int actuator_id = hold_actuator_ids_[i];
        if (qpos_adr < 0 || qvel_adr < 0 || actuator_id < 0)
        {
            continue;
        }
        if (qpos_adr >= model_->nq || qvel_adr >= model_->nv || actuator_id >= model_->nu)
        {
            continue;
        }
        if (i >= hold_target_q_.size())
        {
            continue;
        }

        const double q = data_->qpos[qpos_adr];
        const double dq = data_->qvel[qvel_adr];
        double q_des =
            (i < resolved_hold_config_target_q_.size() &&
             hold_target_source_ != HoldTargetSource::kExplicit)
                ? resolved_hold_config_target_q_[i]
                : hold_target_q_[i];
        if (allow_hold_latch_targets && i < latched_hold_target_q_.size())
        {
            q_des = latched_hold_target_q_[i];
        }
        const ActuatorBackend actuator_backend =
            i < hold_actuator_backends_.size() ? hold_actuator_backends_[i]
                                               : (use_position_actuator_control_ ? ActuatorBackend::kPosition
                                                                                 : ActuatorBackend::kTorque);

        if (actuator_backend == ActuatorBackend::kPosition)
        {
            data_->ctrl[actuator_id] = q_des;
            if (i < hold_applied_tau_.size())
            {
                hold_applied_tau_[i] = 0.0f;
            }
        }
        else
        {
            double tau = hold_kp_[i] * (q_des - q) + hold_kd_[i] * (-dq);
            const double limit = std::max(1e-6, std::abs(hold_torque_limit_[i]));
            tau = std::clamp(tau, -limit, limit);
            data_->ctrl[actuator_id] = tau;
            if (i < hold_applied_tau_.size())
            {
                hold_applied_tau_[i] = static_cast<float>(tau);
            }
        }
    }
}

} // namespace mujoco_sim2sim
