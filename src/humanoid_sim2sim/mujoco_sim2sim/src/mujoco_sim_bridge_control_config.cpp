#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::initializeState()
{
    resolvePerJointControlConfig(controller_runtime_.activeModeId());

    captureBaseLockPoseFromModel();

    if (enable_fixed_base_zeroing_)
    {
        activateDynamicBaseLock(BaseLockReason::kStartupZeroing, enable_prepose_snap_);
    }
    else if (fix_base_ && fixed_base_pose_initialized_)
    {
        enforceBaseLock();
        mj_forward(model_, data_);
        RCLCPP_INFO(
            this->get_logger(),
            "Base lock enabled. fixed xyz=(%.4f, %.4f, %.4f)",
            fixed_base_qpos_[0],
            fixed_base_qpos_[1],
            fixed_base_qpos_[2]);
    }
    else if (fix_base_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "fix_base=true but free base joint is unavailable; base lock disabled.");
        fix_base_ = false;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        if (qpos_adr >= 0 && qpos_adr < model_->nq)
        {
            last_target_q_[i] = static_cast<float>(data_->qpos[qpos_adr]);
        }
    }

    if (hold_target_source_ == HoldTargetSource::kExplicit &&
        hold_target_q_.empty() &&
        !hold_qpos_addrs_.empty())
    {
        hold_target_q_.assign(hold_qpos_addrs_.size(), 0.0);
        for (size_t i = 0; i < hold_qpos_addrs_.size(); ++i)
        {
            const int qpos_adr = hold_qpos_addrs_[i];
            if (qpos_adr >= 0 && qpos_adr < model_->nq)
            {
                hold_target_q_[i] = data_->qpos[qpos_adr];
            }
        }
        RCLCPP_INFO(
            this->get_logger(),
            "hold_joint_target_q not configured, latch %zu extra hold joints from model initial qpos.",
            hold_target_q_.size());
    }
}

MujocoSimBridge::SimJointRuntimeMode MujocoSimBridge::parseSimJointRuntimeMode(
    const std::string &raw_mode,
    const std::string &context)
{
    const std::string mode = toLowerCopy(trimCopy(raw_mode));
    if (mode == "csp")
    {
        return SimJointRuntimeMode::kCsp;
    }
    if (mode == "cst")
    {
        return SimJointRuntimeMode::kCst;
    }
    if (mode == "r1")
    {
        return SimJointRuntimeMode::kR1;
    }
    throw std::runtime_error(
        context + " must be one of: csp, cst, r1. got='" + raw_mode + "'");
}

const char *MujocoSimBridge::simJointRuntimeModeName(SimJointRuntimeMode mode)
{
    switch (mode)
    {
    case SimJointRuntimeMode::kCsp:
        return "csp";
    case SimJointRuntimeMode::kCst:
        return "cst";
    case SimJointRuntimeMode::kR1:
        return "r1";
    default:
        return "unknown";
    }
}

MujocoSimBridge::HoldTargetSource MujocoSimBridge::parseHoldTargetSource(const std::string &raw_source)
{
    const std::string source = toLowerCopy(trimCopy(raw_source));
    if (source == "zero_joint_angles")
    {
        return HoldTargetSource::kZeroJointAngles;
    }
    if (source == "default_joint_angles")
    {
        return HoldTargetSource::kDefaultJointAngles;
    }
    if (source == "explicit")
    {
        return HoldTargetSource::kExplicit;
    }
    throw std::runtime_error(
        "hold_target_source must be one of: zero_joint_angles, default_joint_angles, explicit. got='" +
        raw_source + "'");
}

const char *MujocoSimBridge::holdTargetSourceName(HoldTargetSource source)
{
    switch (source)
    {
    case HoldTargetSource::kZeroJointAngles:
        return "zero_joint_angles";
    case HoldTargetSource::kDefaultJointAngles:
        return "default_joint_angles";
    case HoldTargetSource::kExplicit:
        return "explicit";
    default:
        return "unknown";
    }
}

MujocoSimBridge::ActuatorBackend MujocoSimBridge::classifyModelActuatorBackend(
    const mjModel_ *model,
    int actuator_id)
{
    if (!model || actuator_id < 0 || actuator_id >= model->nu)
    {
        throw std::runtime_error("invalid actuator id for backend classification");
    }

    return model->actuator_biastype[actuator_id] == mjBIAS_NONE
               ? ActuatorBackend::kTorque
               : ActuatorBackend::kPosition;
}

const char *MujocoSimBridge::actuatorBackendName(ActuatorBackend backend)
{
    switch (backend)
    {
    case ActuatorBackend::kTorque:
        return "torque";
    case ActuatorBackend::kPosition:
        return "position";
    default:
        return "unknown";
    }
}

void MujocoSimBridge::resolvePerJointControlConfig(int active_mode_id)
{
    if (resolved_control_mode_id_ == active_mode_id &&
        resolved_joint_runtime_modes_.size() == joint_names_.size() &&
        joint_is_policy_controlled_.size() == joint_names_.size() &&
        resolved_hold_target_q_.size() == joint_names_.size())
    {
        return;
    }

    const Sim2realCfg &active_cfg = controller_runtime_.runtimeCfg();

    std::map<std::string, SimJointRuntimeMode> override_modes;
    for (size_t i = 0; i < joint_runtime_mode_override_entries_.size(); ++i)
    {
        const std::string raw_entry = trimCopy(joint_runtime_mode_override_entries_[i]);
        if (raw_entry.empty())
        {
            continue;
        }

        const size_t sep = raw_entry.find_first_of("=:");
        if (sep == std::string::npos)
        {
            throw std::runtime_error(
                "joint_runtime_mode_overrides[" + std::to_string(i) +
                "] must have format 'joint=mode' or 'joint:mode'");
        }

        const std::string joint_name = trimCopy(raw_entry.substr(0, sep));
        const std::string raw_mode = trimCopy(raw_entry.substr(sep + 1));
        if (joint_name.empty() || raw_mode.empty())
        {
            throw std::runtime_error(
                "joint_runtime_mode_overrides[" + std::to_string(i) +
                "] must have non-empty joint and mode");
        }
        override_modes[joint_name] = parseSimJointRuntimeMode(
            raw_mode,
            "joint_runtime_mode_overrides[" + std::to_string(i) + "]");
    }

    std::map<std::string, float> hold_source_targets;
    const auto &source_angles =
        (hold_target_source_ == HoldTargetSource::kDefaultJointAngles)
            ? active_cfg.robotCfg.default_joint_angles
            : active_cfg.robotCfg.zero_joint_angles;
    for (const auto &entry : source_angles)
    {
        hold_source_targets[entry.first] = entry.second;
    }

    resolved_joint_runtime_modes_.assign(joint_names_.size(), SimJointRuntimeMode::kCsp);
    joint_is_policy_controlled_.assign(joint_names_.size(), false);
    resolved_policy_profile_kp_.assign(joint_names_.size(), 0.0);
    resolved_policy_profile_kd_.assign(joint_names_.size(), 0.0);
    resolved_policy_profile_torque_limit_.assign(joint_names_.size(), 0.0);
    resolved_hold_target_q_.assign(joint_names_.size(), 0.0f);
    joint_cmd_q_.assign(joint_names_.size(), 0.0f);
    joint_cmd_dq_.assign(joint_names_.size(), 0.0f);
    joint_cmd_tau_.assign(joint_names_.size(), 0.0f);
    joint_cmd_mode_.assign(joint_names_.size(), 0.0f);
    resolved_hold_config_target_q_.assign(hold_joint_names_.size(), 0.0);

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const std::string &joint_name = joint_names_[i];
        joint_is_policy_controlled_[i] =
            std::find(active_cfg.action_joint_order.begin(), active_cfg.action_joint_order.end(), joint_name) !=
            active_cfg.action_joint_order.end();
        if (joint_is_policy_controlled_[i])
        {
            const auto action_it =
                std::find(active_cfg.action_joint_order.begin(), active_cfg.action_joint_order.end(), joint_name);
            if (action_it != active_cfg.action_joint_order.end())
            {
                const size_t policy_idx =
                    static_cast<size_t>(std::distance(active_cfg.action_joint_order.begin(), action_it));
                resolved_policy_profile_kp_[i] =
                    policy_idx < active_cfg.kps.size() ? static_cast<double>(active_cfg.kps[policy_idx]) : kp_[i];
                resolved_policy_profile_kd_[i] =
                    policy_idx < active_cfg.kds.size() ? static_cast<double>(active_cfg.kds[policy_idx]) : kd_[i];
                resolved_policy_profile_torque_limit_[i] =
                    policy_idx < active_cfg.tau_limit.size()
                        ? std::abs(static_cast<double>(active_cfg.tau_limit[policy_idx]))
                        : std::abs(torque_limit_[i]);
            }
            else
            {
                resolved_policy_profile_kp_[i] = kp_[i];
                resolved_policy_profile_kd_[i] = kd_[i];
                resolved_policy_profile_torque_limit_[i] = std::abs(torque_limit_[i]);
            }
        }
        if (joint_is_policy_controlled_[i] &&
            i < joint_actuator_backends_.size() &&
            joint_actuator_backends_[i] == ActuatorBackend::kPosition)
        {
            throw std::runtime_error(
                "mixed actuator config is invalid: policy-controlled joint '" + joint_name +
                "' cannot use a position actuator");
        }

        auto override_it = override_modes.find(joint_name);
        if (override_it != override_modes.end())
        {
            resolved_joint_runtime_modes_[i] = override_it->second;
        }
        else
        {
            const auto profile_it = active_cfg.installed_joint_run_modes.find(joint_name);
            if (profile_it != active_cfg.installed_joint_run_modes.end())
            {
                resolved_joint_runtime_modes_[i] = parseSimJointRuntimeMode(
                    profile_it->second,
                    "installed_joint_run_modes['" + joint_name + "']");
            }
        }

        const int hold_cfg_idx = (i < joint_hold_config_indices_.size()) ? joint_hold_config_indices_[i] : -1;
        if (hold_cfg_idx < 0)
        {
            continue;
        }

        double hold_q = 0.0;
        if (hold_target_source_ == HoldTargetSource::kExplicit)
        {
            if (static_cast<size_t>(hold_cfg_idx) >= hold_target_q_.size())
            {
                throw std::runtime_error(
                    "hold_joint_target_q is missing explicit value for hold_joint_names[" +
                    std::to_string(hold_cfg_idx) + "]='" + hold_joint_names_[static_cast<size_t>(hold_cfg_idx)] + "'");
            }
            hold_q = hold_target_q_[static_cast<size_t>(hold_cfg_idx)];
        }
        else
        {
            const auto target_it = hold_source_targets.find(joint_name);
            if (target_it == hold_source_targets.end())
            {
                throw std::runtime_error(
                    std::string("hold target source '") + holdTargetSourceName(hold_target_source_) +
                    "' is missing joint '" + joint_name + "'");
            }
            hold_q = target_it->second;
        }

        resolved_hold_target_q_[i] = static_cast<float>(hold_q);
        resolved_hold_config_target_q_[static_cast<size_t>(hold_cfg_idx)] = hold_q;
    }

    for (size_t i = 0; i < hold_joint_names_.size(); ++i)
    {
        if (hold_main_joint_indices_[i] >= 0)
        {
            continue;
        }

        double hold_q = 0.0;
        if (hold_target_source_ == HoldTargetSource::kExplicit)
        {
            if (i >= hold_target_q_.size())
            {
                throw std::runtime_error(
                    "hold_joint_target_q is missing explicit value for extra hold joint '" +
                    hold_joint_names_[i] + "'");
            }
            hold_q = hold_target_q_[i];
        }
        else
        {
            const auto target_it = hold_source_targets.find(hold_joint_names_[i]);
            if (target_it == hold_source_targets.end())
            {
                throw std::runtime_error(
                    std::string("hold target source '") + holdTargetSourceName(hold_target_source_) +
                    "' is missing extra hold joint '" + hold_joint_names_[i] + "'");
            }
            hold_q = target_it->second;
        }
        resolved_hold_config_target_q_[i] = hold_q;
    }

    resolved_control_mode_id_ = active_mode_id;
}

bool MujocoSimBridge::shouldEnforceBaseLock() const
{
    return fix_base_ || dynamic_base_lock_active_;
}

void MujocoSimBridge::captureBaseLockPoseFromModel()
{
    if (base_free_qpos_adr_ < 0 || (base_free_qpos_adr_ + 6) >= model_->nq)
    {
        fixed_base_pose_initialized_ = false;
        return;
    }

    for (size_t i = 0; i < fixed_base_qpos_.size(); ++i)
    {
        fixed_base_qpos_[i] = data_->qpos[base_free_qpos_adr_ + static_cast<int>(i)];
    }
    if (fixed_base_height_ >= 0.0)
    {
        fixed_base_qpos_[2] = fixed_base_height_;
    }
    fixed_base_pose_initialized_ = true;
}

void MujocoSimBridge::applyPreposeSnap()
{
    if (prepose_joint_q_.empty())
    {
        return;
    }

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        const int qvel_adr = qvel_addrs_[i];
        if (qpos_adr < 0 || qpos_adr >= model_->nq)
        {
            continue;
        }
        const double target_q =
            (prepose_joint_q_.size() == 1)
                ? prepose_joint_q_.front()
                : prepose_joint_q_[i];
        data_->qpos[qpos_adr] = target_q;
        if (qvel_adr >= 0 && qvel_adr < model_->nv)
        {
            data_->qvel[qvel_adr] = 0.0;
        }
        last_target_q_[i] = static_cast<float>(target_q);
    }

    mj_forward(model_, data_);
    RCLCPP_INFO(this->get_logger(), "Applied sim prepose snap before fixed-base zeroing.");
}

bool MujocoSimBridge::maybeApplyRunningStartReferenceSync(
    const rl_master::logging::ControllerLogSnapshot &controller_snapshot)
{
    if (!sim_sync_running_start_to_reference_ || !running_start_reference_sync_pending_)
    {
        return false;
    }
    if (!controller_snapshot.valid)
    {
        return false;
    }
    if (static_cast<rl_master::DeployLifecycleState>(controller_snapshot.deploy_state) !=
        rl_master::DeployLifecycleState::kRunning)
    {
        return false;
    }

    const auto find_feature =
        [&](const std::string &name) -> const std::vector<float> * {
        const auto it = controller_snapshot.named_features.find(name);
        if (it == controller_snapshot.named_features.end())
        {
            return nullptr;
        }
        return &it->second;
    };

    const auto *reference_joint_pos = find_feature("reference_joint_pos");
    if (!reference_joint_pos || reference_joint_pos->empty())
    {
        return false;
    }

    const auto *reference_joint_vel = find_feature("reference_joint_vel");
    const auto *reference_body_quat_w = find_feature("reference_body_quat_w");
    const auto *reference_body_lin_vel_w = find_feature("reference_body_lin_vel_w");
    const auto *reference_body_ang_vel_w = find_feature("reference_body_ang_vel_w");
    const Sim2realCfg &active_cfg = controller_runtime_.runtimeCfg();

    const std::string reference_source = toLowerCopy(trimCopy(active_cfg.reference_motion_source));
    const bool policy_reference_preserves_order =
        reference_source != "file" &&
        active_cfg.source_contract.policy_extra_outputs.preserve_reference_joint_order &&
        active_cfg.reference_joint_order.size() == reference_joint_pos->size();
    const std::vector<std::string> &reference_joint_source_order =
        policy_reference_preserves_order ? active_cfg.reference_joint_order : joint_names_;
    const auto referenceJointValue = [&](const std::vector<float> *values,
                                         const std::string &joint_name,
                                         size_t fallback_index,
                                         float fallback_value) -> float {
        if (!values || values->empty())
        {
            return fallback_value;
        }
        if (reference_joint_source_order.size() == values->size())
        {
            const auto it = std::find(
                reference_joint_source_order.begin(),
                reference_joint_source_order.end(),
                joint_name);
            if (it != reference_joint_source_order.end())
            {
                const size_t source_index =
                    static_cast<size_t>(std::distance(reference_joint_source_order.begin(), it));
                if (source_index < values->size())
                {
                    return (*values)[source_index];
                }
            }
        }
        if (fallback_index < values->size())
        {
            return (*values)[fallback_index];
        }
        return fallback_value;
    };

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        const int qvel_adr = qvel_addrs_[i];
        const std::string &joint_name = joint_names_[i];
        if (qpos_adr >= 0 && qpos_adr < model_->nq)
        {
            const float q =
                referenceJointValue(reference_joint_pos, joint_name, i, static_cast<float>(data_->qpos[qpos_adr]));
            data_->qpos[qpos_adr] = static_cast<double>(q);
            last_target_q_[i] = q;
        }
        if (qvel_adr >= 0 && qvel_adr < model_->nv)
        {
            const float dq = referenceJointValue(reference_joint_vel, joint_name, i, 0.0f);
            data_->qvel[qvel_adr] = static_cast<double>(dq);
        }
    }

    if (base_free_qpos_adr_ >= 0 &&
        (base_free_qpos_adr_ + 6) < model_->nq &&
        base_free_qvel_adr_ >= 0 &&
        (base_free_qvel_adr_ + 5) < model_->nv)
    {
        const std::vector<std::string> &body_names = active_cfg.reference_body_names;
        const auto anchor_it = std::find(body_names.begin(), body_names.end(), active_cfg.reference_anchor_body);
        if (anchor_it != body_names.end())
        {
            const size_t anchor_index = static_cast<size_t>(std::distance(body_names.begin(), anchor_it));
            if (reference_body_quat_w)
            {
                const size_t quat_offset = anchor_index * 4;
                if (quat_offset + 3 < reference_body_quat_w->size())
                {
                    const std::array<double, 4> current_quat_wxyz = normalizeQuatWxyz({
                        data_->qpos[base_free_qpos_adr_ + 3],
                        data_->qpos[base_free_qpos_adr_ + 4],
                        data_->qpos[base_free_qpos_adr_ + 5],
                        data_->qpos[base_free_qpos_adr_ + 6],
                    });
                    const std::array<double, 4> reference_quat_wxyz = normalizeQuatWxyz({
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 3]),
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 0]),
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 1]),
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 2]),
                    });
                    const std::array<double, 4> reference_roll_pitch =
                        multiplyQuatWxyz(inverseQuatWxyz(yawQuatWxyz(reference_quat_wxyz)), reference_quat_wxyz);
                    const std::array<double, 4> synced_quat =
                        multiplyQuatWxyz(yawQuatWxyz(current_quat_wxyz), reference_roll_pitch);
                    data_->qpos[base_free_qpos_adr_ + 3] = synced_quat[0];
                    data_->qpos[base_free_qpos_adr_ + 4] = synced_quat[1];
                    data_->qpos[base_free_qpos_adr_ + 5] = synced_quat[2];
                    data_->qpos[base_free_qpos_adr_ + 6] = synced_quat[3];
                }
            }
            for (int i = 0; i < 6; ++i)
            {
                data_->qvel[base_free_qvel_adr_ + i] = 0.0;
            }
            if (reference_body_lin_vel_w)
            {
                const size_t vel_offset = anchor_index * 3;
                if (vel_offset + 2 < reference_body_lin_vel_w->size())
                {
                    data_->qvel[base_free_qvel_adr_ + 0] =
                        static_cast<double>((*reference_body_lin_vel_w)[vel_offset + 0]);
                    data_->qvel[base_free_qvel_adr_ + 1] =
                        static_cast<double>((*reference_body_lin_vel_w)[vel_offset + 1]);
                    data_->qvel[base_free_qvel_adr_ + 2] =
                        static_cast<double>((*reference_body_lin_vel_w)[vel_offset + 2]);
                }
            }
            if (reference_body_ang_vel_w)
            {
                const size_t vel_offset = anchor_index * 3;
                if (vel_offset + 2 < reference_body_ang_vel_w->size())
                {
                    data_->qvel[base_free_qvel_adr_ + 3] =
                        static_cast<double>((*reference_body_ang_vel_w)[vel_offset + 0]);
                    data_->qvel[base_free_qvel_adr_ + 4] =
                        static_cast<double>((*reference_body_ang_vel_w)[vel_offset + 1]);
                    data_->qvel[base_free_qvel_adr_ + 5] =
                        static_cast<double>((*reference_body_ang_vel_w)[vel_offset + 2]);
                }
            }
        }
    }

    mj_forward(model_, data_);
    running_start_reference_sync_pending_ = false;
    RCLCPP_INFO(
        this->get_logger(),
        "Applied sim-only startup reference sync at RUNNING entry without changing base world pose.");
    return true;
}

void MujocoSimBridge::activateDynamicBaseLock(BaseLockReason reason, bool apply_prepose)
{
    const bool was_active = dynamic_base_lock_active_;
    const BaseLockReason previous_reason = dynamic_base_lock_reason_;
    if (!fixed_base_pose_initialized_)
    {
        captureBaseLockPoseFromModel();
    }
    if (!fixed_base_pose_initialized_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "Dynamic base lock requested for %s, but base free joint pose is unavailable.",
            baseLockReasonName(reason));
        return;
    }

    dynamic_base_lock_active_ = true;
    dynamic_base_lock_reason_ = reason;

    if (apply_prepose)
    {
        applyPreposeSnap();
    }

    enforceBaseLock();
    mj_forward(model_, data_);

    if (!was_active || previous_reason != reason)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Dynamic base lock active: reason=%s, fixed xyz=(%.4f, %.4f, %.4f)",
            baseLockReasonName(reason),
            fixed_base_qpos_[0],
            fixed_base_qpos_[1],
            fixed_base_qpos_[2]);
    }
}

void MujocoSimBridge::deactivateDynamicBaseLock(const char *reason)
{
    if (!dynamic_base_lock_active_)
    {
        return;
    }
    dynamic_base_lock_active_ = false;
    dynamic_base_lock_reason_ = BaseLockReason::kNone;
    zeroing_injection_pending_ = false;
    RCLCPP_INFO(this->get_logger(), "Dynamic base lock released: %s", reason ? reason : "unspecified");
}

} // namespace mujoco_sim2sim
