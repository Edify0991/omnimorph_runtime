#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

namespace
{
constexpr double kRuntimeCommandFreshnessSec = 0.25;

bool parseTaggedModeId(
    const std::map<std::string, std::string> &tags,
    const char *key,
    int *mode_id)
{
    if (!key || !mode_id)
    {
        return false;
    }
    const auto it = tags.find(key);
    if (it == tags.end())
    {
        return false;
    }
    try
    {
        *mode_id = std::stoi(it->second);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

std::optional<int> suggestedLatchedModeCommandFromSnapshot(
    const rl_master::logging::ControllerLogSnapshot &snapshot)
{
    if (!snapshot.valid)
    {
        return std::nullopt;
    }

    int target_mode_id = -1;
    if (snapshot.runtime_warning_type == "reference_end_auto_mode_switch_scheduled" &&
        parseTaggedModeId(snapshot.runtime_warning_tags, "to_mode_id", &target_mode_id))
    {
        return rl_master::kCtrlWordSetModeBase + target_mode_id;
    }
    if (snapshot.runtime_warning_type == "reference_end_auto_mode_switch_applied" &&
        parseTaggedModeId(snapshot.runtime_warning_tags, "target_mode_id", &target_mode_id))
    {
        return rl_master::kCtrlWordSetModeBase + target_mode_id;
    }
    return std::nullopt;
}

int demoteLatchedStartModeCommand(
    int raw_control_word,
    const rl_master::logging::ControllerLogSnapshot &snapshot)
{
    if (!snapshot.valid)
    {
        return raw_control_word;
    }
    if (static_cast<rl_master::DeployLifecycleState>(snapshot.deploy_state) !=
        rl_master::DeployLifecycleState::kRunning)
    {
        return raw_control_word;
    }
    if (raw_control_word < rl_master::kCtrlWordStartModeBase ||
        raw_control_word >= (rl_master::kCtrlWordStartModeBase + rl_master::kCtrlWordModeRange))
    {
        return raw_control_word;
    }

    const rl_master::DecodedControlWord decoded =
        rl_master::DeployStateMachine::decodeControlWord(
            raw_control_word,
            snapshot.active_mode_id);
    if (!decoded.request_start || decoded.locomotion_mode != snapshot.active_mode_id)
    {
        return raw_control_word;
    }
    return rl_master::kCtrlWordSetModeBase + decoded.locomotion_mode;
}
}

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
    zeroLockedPreRunJointVelocities();
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
    const double sim_phase_t = data_ != nullptr ? data_->time : 0.0;
    rl_master::RobotCommandData command =
        controller_runtime_.step(state, teleop_command, effective_mode_control_word, sim_phase_t);
    const auto &controller_snapshot = controller_runtime_.controller().latestLogSnapshot();
    if (const auto rewritten_mode_command =
            suggestedLatchedModeCommandFromSnapshot(controller_snapshot))
    {
        mode_command_cache_.store(*rewritten_mode_command);
    }
    else
    {
        const int demoted_mode_command =
            demoteLatchedStartModeCommand(raw_mode_control_word, controller_snapshot);
        if (demoted_mode_command != raw_mode_control_word)
        {
            mode_command_cache_.store(demoted_mode_command);
        }
    }
    emitDerivedRuntimeEvents(controller_snapshot);
    bool runtime_command_fresh = true;
    bool has_external_runtime_command = false;
    {
        std::lock_guard<std::mutex> lock(runtime_command_mutex_);
        if (has_runtime_command_)
        {
            has_external_runtime_command = true;
            latest_runtime_command_fresh_ =
                (rl_master::monotonicTimeSec() - latest_runtime_command_stamp_sec_) <= kRuntimeCommandFreshnessSec;
            command = latest_runtime_command_;
            runtime_command_fresh = latest_runtime_command_fresh_;
        }
    }
    if (controller_runtime_.runtimeCfg().external_command_only && !has_external_runtime_command)
    {
        // Match the real solver: an external-command-only profile without a
        // runner command is inactive and enters the bridge hold behavior.
        command = rl_master::RobotCommandData{};
        command.active_joint_count = static_cast<int>(joint_names_.size());
        command.open_rl = rl_master::kOpenRlDisabled;
        runtime_command_fresh = true;
    }
    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(runtime_command_fresh, command.open_rl);
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
            hold_target_latched_ = true;
            RCLCPP_INFO(this->get_logger(), "Controller inactive, latch current pose for hold behavior.");
        }
    }
    else
    {
        hold_target_latched_ = false;
    }

    const bool reference_pose_replay_applied = applyReferencePoseReplayFrame(controller_snapshot);
    if (!reference_pose_replay_applied)
    {
        (void)maybeApplyRunningStartReferenceSync(controller_snapshot);
        enforceBaseLock();
        updateControlInput(command, control_active, now);
    }

    const bool allow_inactive_step_for_release = (release_settle_ticks_remaining_ > 0);
    const bool allow_inactive_step_for_hold_settle = (hold_settle_ticks_remaining_ > 0);
    if (reference_pose_replay_applied)
    {
        recordVideoFrameIfDue();
        viewer_step_once_ = false;
    }
    else if (should_step &&
             (control_active || !pause_when_no_command_ || allow_inactive_step_for_release ||
              allow_inactive_step_for_hold_settle))
    {
        for (int i = 0; i < speed_substeps; ++i)
        {
            std::vector<double> pre_step_q(joint_names_.size(), 0.0);
            for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx)
            {
                const int qpos_adr = (joint_idx < qpos_addrs_.size()) ? qpos_addrs_[joint_idx] : -1;
                if (qpos_adr >= 0 && qpos_adr < model_->nq)
                {
                    pre_step_q[joint_idx] = data_->qpos[qpos_adr];
                }
            }

            enforceBaseLock();
            mj_step(model_, data_);
            const double step_dt = std::max(1.0e-9, static_cast<double>(model_->opt.timestep));
            if (control_active)
            {
                for (size_t joint_idx = 0; joint_idx < joint_names_.size(); ++joint_idx)
                {
                    if (joint_idx >= resolved_dc_motor_velocity_limit_.size() ||
                        resolved_dc_motor_velocity_limit_[joint_idx] <= 0.0)
                    {
                        continue;
                    }
                    const int qpos_adr = (joint_idx < qpos_addrs_.size()) ? qpos_addrs_[joint_idx] : -1;
                    const int qvel_adr = (joint_idx < qvel_addrs_.size()) ? qvel_addrs_[joint_idx] : -1;
                    if (qpos_adr < 0 || qpos_adr >= model_->nq || qvel_adr < 0 || qvel_adr >= model_->nv)
                    {
                        continue;
                    }
                    const double max_delta = resolved_dc_motor_velocity_limit_[joint_idx] * step_dt;
                    const double delta = data_->qpos[qpos_adr] - pre_step_q[joint_idx];
                    if (delta > max_delta)
                    {
                        data_->qpos[qpos_adr] = pre_step_q[joint_idx] + max_delta;
                        data_->qvel[qvel_adr] = resolved_dc_motor_velocity_limit_[joint_idx];
                    }
                    else if (delta < -max_delta)
                    {
                        data_->qpos[qpos_adr] = pre_step_q[joint_idx] - max_delta;
                        data_->qvel[qvel_adr] = -resolved_dc_motor_velocity_limit_[joint_idx];
                    }
                }
            }
            enforceBaseLock();
            recordVideoFrameIfDue();
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
        recordVideoFrameIfDue();
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

} // namespace mujoco_sim2sim
