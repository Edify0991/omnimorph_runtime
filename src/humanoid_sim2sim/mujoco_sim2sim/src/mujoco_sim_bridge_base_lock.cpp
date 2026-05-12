#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

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
    const bool full_pose_sync = sim_sync_running_start_to_reference_;
    const bool dynamics_only_seed = sim_seed_running_start_reference_dynamics_;
    if ((!full_pose_sync && !dynamics_only_seed) || !running_start_reference_sync_pending_)
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
        if (full_pose_sync && qpos_adr >= 0 && qpos_adr < model_->nq)
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
            if (full_pose_sync && reference_body_quat_w)
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
    if (full_pose_sync)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Applied sim-only startup reference sync at RUNNING entry without changing base world pose.");
    }
    else
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Applied sim-only startup reference dynamics seed at RUNNING entry (joint/base velocities only).");
    }
    return true;
}

bool MujocoSimBridge::applyReferencePoseReplayFrame(
    const rl_master::logging::ControllerLogSnapshot &controller_snapshot)
{
    if (!enable_reference_pose_replay_test_ || !controller_snapshot.valid)
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
    const auto *reference_body_pos_w = find_feature("reference_body_pos_w");
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
            if (reference_body_pos_w)
            {
                const size_t pos_offset = anchor_index * 3;
                if (pos_offset + 2 < reference_body_pos_w->size())
                {
                    data_->qpos[base_free_qpos_adr_ + 0] =
                        static_cast<double>((*reference_body_pos_w)[pos_offset + 0]);
                    data_->qpos[base_free_qpos_adr_ + 1] =
                        static_cast<double>((*reference_body_pos_w)[pos_offset + 1]);
                    data_->qpos[base_free_qpos_adr_ + 2] =
                        static_cast<double>((*reference_body_pos_w)[pos_offset + 2]);
                }
            }
            if (reference_body_quat_w)
            {
                const size_t quat_offset = anchor_index * 4;
                if (quat_offset + 3 < reference_body_quat_w->size())
                {
                    const std::array<double, 4> reference_quat_wxyz = normalizeQuatWxyz({
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 3]),
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 0]),
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 1]),
                        static_cast<double>((*reference_body_quat_w)[quat_offset + 2]),
                    });
                    data_->qpos[base_free_qpos_adr_ + 3] = reference_quat_wxyz[0];
                    data_->qpos[base_free_qpos_adr_ + 4] = reference_quat_wxyz[1];
                    data_->qpos[base_free_qpos_adr_ + 5] = reference_quat_wxyz[2];
                    data_->qpos[base_free_qpos_adr_ + 6] = reference_quat_wxyz[3];
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

    if (model_->nu > 0)
    {
        mju_zero(data_->ctrl, model_->nu);
    }
    std::fill(applied_tau_.begin(), applied_tau_.end(), 0.0f);
    std::fill(joint_cmd_tau_.begin(), joint_cmd_tau_.end(), 0.0f);
    std::fill(joint_cmd_dq_.begin(), joint_cmd_dq_.end(), 0.0f);
    std::fill(joint_cmd_mode_.begin(), joint_cmd_mode_.end(), 0.0f);
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        if (qpos_adr >= 0 && qpos_adr < model_->nq)
        {
            joint_cmd_q_[i] = static_cast<float>(data_->qpos[qpos_adr]);
        }
    }

    mj_forward(model_, data_);
    running_start_reference_sync_pending_ = false;
    if (!reference_pose_replay_test_logged_)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Reference pose replay test enabled: each RUNNING control tick now writes reference qpos/qvel/base pose directly.");
        reference_pose_replay_test_logged_ = true;
    }
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
