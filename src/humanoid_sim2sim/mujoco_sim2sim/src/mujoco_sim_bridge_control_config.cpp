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
    for (const auto &entry : active_cfg.robotCfg.zero_joint_angles)
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
                if (policy_idx >= active_cfg.kps.size() ||
                    policy_idx >= active_cfg.kds.size() ||
                    policy_idx >= active_cfg.tau_limit.size())
                {
                    throw std::runtime_error(
                        "active mode profile does not fully cover action joint '" + joint_name +
                        "' in kps/kds/tau_limit");
                }
                // active_cfg.{kps,kds,tau_limit} are stored in action_joint_order.
                // The backend loop executes in canonical joint order, so we materialize a
                // per-canonical-joint view here without changing each joint's runtime mode.
                resolved_policy_profile_kp_[i] = static_cast<double>(active_cfg.kps[policy_idx]);
                resolved_policy_profile_kd_[i] = static_cast<double>(active_cfg.kds[policy_idx]);
                resolved_policy_profile_torque_limit_[i] =
                    std::abs(static_cast<double>(active_cfg.tau_limit[policy_idx]));
            }
            else
            {
                throw std::runtime_error(
                    "policy-controlled canonical joint '" + joint_name +
                    "' is missing from active_cfg.action_joint_order");
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

        const auto target_it = hold_source_targets.find(joint_name);
        if (target_it == hold_source_targets.end())
        {
            throw std::runtime_error(
                "robot.zero_joint_angles is missing canonical joint '" + joint_name +
                "' required by sim2sim hold targets");
        }
        resolved_hold_target_q_[i] = static_cast<float>(target_it->second);
    }

    resolved_control_mode_id_ = active_mode_id;
}

} // namespace mujoco_sim2sim
