#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::loadModel()
{
    char error[1024] = {0};
    if (endsWith(model_path_, ".mjb"))
    {
        model_ = mj_loadModel(model_path_.c_str(), nullptr);
    }
    else
    {
        model_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    }
    if (!model_)
    {
        const std::string reason = (error[0] != '\0') ? std::string(error) : "unknown error";
        throw std::runtime_error("Failed to load MuJoCo model: " + reason);
    }

    model_->opt.timestep = sim_dt_;
    data_ = mj_makeData(model_);
    if (!data_)
    {
        throw std::runtime_error("Failed to create MuJoCo data");
    }

    mj_forward(model_, data_);

    sim_dt_ = std::max(1e-6, static_cast<double>(model_->opt.timestep));
    const double control_period = 1.0 / control_hz_;
    substeps_per_control_ = std::max(1, static_cast<int>(std::lround(control_period / sim_dt_)));

    RCLCPP_INFO(
        this->get_logger(),
        "Loaded MuJoCo model. nq=%d, nv=%d, nu=%d, nbody=%d",
        model_->nq,
        model_->nv,
        model_->nu,
        model_->nbody);
}

void MujocoSimBridge::resolveModelMappings()
{
    joint_actuator_backends_.assign(joint_names_.size(), ActuatorBackend::kTorque);
    default_dof_armature_.assign(joint_names_.size(), 0.0);
    default_dof_frictionloss_.assign(joint_names_.size(), 0.0);
    default_dof_damping_.assign(joint_names_.size(), 0.0);
    position_controlled_joint_names_.clear();
    position_actuator_joint_indices_.clear();
    applied_position_actuator_kp_.clear();
    applied_position_actuator_kv_.clear();
    applied_position_actuator_forcerange_.clear();

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, joint_names_[i].c_str());
        if (joint_id < 0)
        {
            throw std::runtime_error("Joint name not found in MuJoCo model: " + joint_names_[i]);
        }

        const int joint_type = model_->jnt_type[joint_id];
        if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE)
        {
            throw std::runtime_error(
                "Controlled joint must be hinge or slide: " + joint_names_[i]);
        }

        joint_ids_[i] = joint_id;
        qpos_addrs_[i] = model_->jnt_qposadr[joint_id];
        qvel_addrs_[i] = model_->jnt_dofadr[joint_id];
        default_dof_armature_[i] = model_->dof_armature[qvel_addrs_[i]];
        default_dof_frictionloss_[i] = model_->dof_frictionloss[qvel_addrs_[i]];
        default_dof_damping_[i] = model_->dof_damping[qvel_addrs_[i]];

        int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, actuator_names_[i].c_str());
        if (actuator_id < 0)
        {
            throw std::runtime_error(
                "Actuator name not found in MuJoCo model: " + actuator_names_[i]);
        }

        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            throw std::runtime_error("Resolved actuator index out of range for " + actuator_names_[i]);
        }
        actuator_ids_[i] = actuator_id;

        const ActuatorBackend actual_backend = classifyModelActuatorBackend(model_, actuator_id);
        joint_actuator_backends_[i] = actual_backend;
        if (actual_backend == ActuatorBackend::kPosition)
        {
            if (model_->actuator_gaintype[actuator_id] != mjGAIN_FIXED ||
                model_->actuator_biastype[actuator_id] != mjBIAS_AFFINE)
            {
                throw std::runtime_error(
                    "position actuator backend expects MuJoCo position shortcut layout "
                    "(gaintype=fixed, biastype=affine) for joint '" +
                    joint_names_[i] + "'");
            }
            position_controlled_joint_names_.push_back(joint_names_[i]);
            position_actuator_joint_indices_.push_back(static_cast<int>(i));
        }
    }

    applied_position_actuator_kp_.assign(position_actuator_joint_indices_.size(), std::numeric_limits<double>::quiet_NaN());
    applied_position_actuator_kv_.assign(position_actuator_joint_indices_.size(), std::numeric_limits<double>::quiet_NaN());
    applied_position_actuator_forcerange_.assign(position_actuator_joint_indices_.size(), std::numeric_limits<double>::quiet_NaN());

    size_t resolved_torque_joint_count = 0;
    size_t resolved_position_joint_count = 0;
    for (const ActuatorBackend backend : joint_actuator_backends_)
    {
        if (backend == ActuatorBackend::kPosition)
        {
            ++resolved_position_joint_count;
        }
        else
        {
            ++resolved_torque_joint_count;
        }
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Resolved actuator backends from model (canonical_joints=%zu, torque_joints=%zu, position_joints=%zu)",
        joint_names_.size(),
        resolved_torque_joint_count,
        resolved_position_joint_count);

    base_body_id_ = mj_name2id(model_, mjOBJ_BODY, base_body_name_.c_str());
    if (base_body_id_ < 0)
    {
        base_body_id_ = (model_->nbody > 1) ? 1 : 0;
        RCLCPP_WARN(
            this->get_logger(),
            "Base body '%s' not found. Fallback to body id %d.",
            base_body_name_.c_str(),
            base_body_id_);
    }

    if (!base_free_joint_name_.empty())
    {
        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, base_free_joint_name_.c_str());
        if (joint_id >= 0 && model_->jnt_type[joint_id] == mjJNT_FREE)
        {
            base_free_joint_id_ = joint_id;
        }
    }

    if (base_free_joint_id_ < 0)
    {
        for (int jid = 0; jid < model_->njnt; ++jid)
        {
            if (model_->jnt_type[jid] == mjJNT_FREE &&
                (base_body_id_ < 0 || model_->jnt_bodyid[jid] == base_body_id_))
            {
                base_free_joint_id_ = jid;
                break;
            }
        }
    }

    if (base_free_joint_id_ >= 0)
    {
        base_free_qpos_adr_ = model_->jnt_qposadr[base_free_joint_id_];
        base_free_qvel_adr_ = model_->jnt_dofadr[base_free_joint_id_];
    }

    const bool requires_free_joint =
        fix_base_ ||
        enable_fixed_base_zeroing_ ||
        enable_fixed_base_hold_after_zeroing_ ||
        enable_release_before_running_;
    if (requires_free_joint && base_free_joint_id_ < 0)
    {
        std::ostringstream oss;
        oss << "MuJoCo model must provide a free joint for base lock features. "
            << "base_body_name='" << base_body_name_ << "'";
        if (!base_free_joint_name_.empty())
        {
            oss << ", requested base_free_joint_name='" << base_free_joint_name_ << "'";
        }
        oss << ", enabled features={fix_base=" << (fix_base_ ? "true" : "false")
            << ", enable_fixed_base_zeroing=" << (enable_fixed_base_zeroing_ ? "true" : "false")
            << ", enable_fixed_base_hold_after_zeroing=" << (enable_fixed_base_hold_after_zeroing_ ? "true" : "false")
            << ", enable_release_before_running=" << (enable_release_before_running_ ? "true" : "false")
            << "}. Add a free joint to the base body or disable these features.";
        throw std::runtime_error(oss.str());
    }
}

void MujocoSimBridge::refreshPositionActuatorTuning(bool control_active)
{
    if (!model_ || position_actuator_joint_indices_.empty())
    {
        return;
    }

    size_t updated_joint_count = 0;
    std::ostringstream summary;
    summary << "Refreshed MuJoCo position actuator tuning from active joint role source:";

    for (size_t i = 0; i < position_actuator_joint_indices_.size(); ++i)
    {
        const int joint_index = position_actuator_joint_indices_[i];
        if (joint_index < 0 || joint_index >= static_cast<int>(joint_names_.size()))
        {
            throw std::runtime_error(
                "position actuator tuning resolved invalid canonical joint index");
        }

        const int actuator_id = actuator_ids_[static_cast<size_t>(joint_index)];
        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            throw std::runtime_error(
                "position actuator tuning resolved invalid actuator id for '" +
                joint_names_[static_cast<size_t>(joint_index)] + "'");
        }
        if (joint_actuator_backends_[static_cast<size_t>(joint_index)] != ActuatorBackend::kPosition)
        {
            throw std::runtime_error(
                "position actuator tuning requires a MuJoCo position actuator for joint '" +
                joint_names_[static_cast<size_t>(joint_index)] + "'");
        }
        if (model_->actuator_gaintype[actuator_id] != mjGAIN_FIXED ||
            model_->actuator_biastype[actuator_id] != mjBIAS_AFFINE)
        {
            throw std::runtime_error(
                "position actuator tuning expects MuJoCo position shortcut layout "
                "(gaintype=fixed, biastype=affine) for joint '" +
                joint_names_[static_cast<size_t>(joint_index)] + "'");
        }

        const bool use_policy_profile =
            control_active &&
            static_cast<size_t>(joint_index) < joint_is_policy_controlled_.size() &&
            joint_is_policy_controlled_[static_cast<size_t>(joint_index)];
        const std::vector<double> &kp_source = use_policy_profile ? resolved_policy_profile_kp_ : hold_kp_;
        const std::vector<double> &kd_source = use_policy_profile ? resolved_policy_profile_kd_ : hold_kd_;
        const std::vector<double> &limit_source =
            use_policy_profile ? resolved_policy_profile_torque_limit_ : hold_torque_limit_;

        if (static_cast<size_t>(joint_index) >= kp_source.size() ||
            static_cast<size_t>(joint_index) >= kd_source.size() ||
            static_cast<size_t>(joint_index) >= limit_source.size())
        {
            throw std::runtime_error(
                "position actuator tuning source size mismatch for canonical joint '" +
                joint_names_[static_cast<size_t>(joint_index)] + "'");
        }

        const double kp = kp_source[static_cast<size_t>(joint_index)];
        const double kv = kd_source[static_cast<size_t>(joint_index)];
        const double force_limit = std::abs(limit_source[static_cast<size_t>(joint_index)]);
        const bool unchanged =
            std::abs(applied_position_actuator_kp_[i] - kp) < 1e-9 &&
            std::abs(applied_position_actuator_kv_[i] - kv) < 1e-9 &&
            std::abs(applied_position_actuator_forcerange_[i] - force_limit) < 1e-9;
        if (unchanged)
        {
            continue;
        }

        model_->actuator_gainprm[actuator_id * mjNGAIN + 0] = kp;
        model_->actuator_biasprm[actuator_id * mjNBIAS + 1] = -kp;
        model_->actuator_biasprm[actuator_id * mjNBIAS + 2] = -kv;
        model_->actuator_forcelimited[actuator_id] = 1;
        model_->actuator_forcerange[2 * actuator_id + 0] = -force_limit;
        model_->actuator_forcerange[2 * actuator_id + 1] = force_limit;
        applied_position_actuator_kp_[i] = kp;
        applied_position_actuator_kv_[i] = kv;
        applied_position_actuator_forcerange_[i] = force_limit;
        ++updated_joint_count;

        summary << " " << joint_names_[static_cast<size_t>(joint_index)]
                << "(" << (use_policy_profile ? "policy" : "hold")
                << ", kp=" << kp
                << ", kv=" << kv
                << ", force=" << force_limit << ")";
        if (i + 1 < position_actuator_joint_indices_.size())
        {
            summary << ",";
        }
    }

    if (updated_joint_count > 0)
    {
        RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
    }
}

} // namespace mujoco_sim2sim
