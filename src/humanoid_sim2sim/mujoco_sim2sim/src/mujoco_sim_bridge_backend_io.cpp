#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{

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
        const ActuatorBackend actuator_backend =
            i < joint_actuator_backends_.size() ? joint_actuator_backends_[i]
                                                : (use_position_actuator_control_ ? ActuatorBackend::kPosition
                                                                                  : ActuatorBackend::kTorque);

        double q_des = static_cast<double>(last_target_q_[i]);
        double dq_des = 0.0;
        double tau_ff = 0.0;
        double tau_cmd = 0.0;
        SimJointRuntimeMode joint_mode = SimJointRuntimeMode::kCsp;

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
                if (i < resolved_hold_target_q_.size())
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
            const bool use_hold_gains = (!control_active) || (mode_policy && !policy_controlled_joint);
            if (use_hold_gains &&
                i < hold_kp_.size() &&
                i < hold_kd_.size() &&
                i < hold_torque_limit_.size())
            {
                double tau = hold_kp_[i] * (q_des - q) + hold_kd_[i] * (dq_des - dq);
                const double limit = std::max(1e-6, std::abs(hold_torque_limit_[i]));
                tau = std::clamp(tau, -limit, limit);
                data_->ctrl[actuator_id] = tau;
                applied_tau_[i] = static_cast<float>(tau);
                continue;
            }

            double tau = 0.0;
            if (control_active &&
                mode_policy &&
                policy_controlled_joint)
            {
                if (joint_mode == SimJointRuntimeMode::kCst)
                {
                    // Policy CST is already fully resolved by RL_controller.
                    tau = tau_cmd;
                }
                else
                {
                    if (i >= resolved_policy_profile_kp_.size() ||
                        i >= resolved_policy_profile_kd_.size() ||
                        i >= resolved_policy_profile_torque_limit_.size())
                    {
                        throw std::runtime_error(
                            "resolved policy profile gains/limits size mismatch for canonical joint '" +
                            joint_names_[i] + "'");
                    }
                    tau =
                        resolved_policy_profile_kp_[i] * (q_des - q) +
                        resolved_policy_profile_kd_[i] * (dq_des - dq);
                    if (joint_mode == SimJointRuntimeMode::kR1)
                    {
                        tau += tau_ff;
                    }
                    const double limit = std::max(1e-6, std::abs(resolved_policy_profile_torque_limit_[i]));
                    tau = std::clamp(tau, -limit, limit);
                }
            }
            else
            {
                if (control_active && joint_mode == SimJointRuntimeMode::kCst)
                {
                    tau = tau_cmd;
                }
                else
                {
                    if (i >= hold_kp_.size() ||
                        i >= hold_kd_.size() ||
                        i >= hold_torque_limit_.size())
                    {
                        throw std::runtime_error(
                            "hold gains/limits size mismatch for canonical joint '" +
                            joint_names_[i] + "'");
                    }
                    tau = hold_kp_[i] * (q_des - q) + hold_kd_[i] * (dq_des - dq);
                    if (control_active && joint_mode == SimJointRuntimeMode::kR1)
                    {
                        tau += tau_ff;
                    }
                    const double limit = std::max(1e-6, std::abs(hold_torque_limit_[i]));
                    tau = std::clamp(tau, -limit, limit);
                }
            }

            data_->ctrl[actuator_id] = tau;
            applied_tau_[i] = static_cast<float>(tau);
        }
    }
}

} // namespace mujoco_sim2sim
