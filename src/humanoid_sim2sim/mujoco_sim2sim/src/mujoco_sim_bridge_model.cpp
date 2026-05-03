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
    position_controlled_joint_names_.clear();

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
            position_controlled_joint_names_.push_back(joint_names_[i]);
        }
    }

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

void MujocoSimBridge::applyPositionActuatorTuning()
{
    if (!model_ || position_controlled_joint_names_.empty())
    {
        return;
    }

    auto loadRequiredNamedJointParams =
        [this](
            const std::string &prefix,
            const std::vector<std::string> &joint_names,
            bool absolute_value) -> std::vector<double> {
            std::vector<double> values(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                const std::string param_name = prefix + "." + joint_names[i];
                this->declare_parameter<double>(
                    param_name,
                    std::numeric_limits<double>::quiet_NaN());
                const double raw_value = this->get_parameter(param_name).as_double();
                if (!std::isfinite(raw_value))
                {
                    throw std::runtime_error(
                        "parameter set '" + prefix +
                        "' must be configured using per-joint name/value entries; missing joint '" +
                        joint_names[i] + "'");
                }
                values[i] = absolute_value ? std::abs(raw_value) : raw_value;
            }
            return values;
        };

    position_actuator_kp_ =
        loadRequiredNamedJointParams("position_actuator_kp", position_controlled_joint_names_, false);
    position_actuator_kv_ =
        loadRequiredNamedJointParams("position_actuator_kv", position_controlled_joint_names_, false);
    position_actuator_forcerange_ =
        loadRequiredNamedJointParams("position_actuator_forcerange", position_controlled_joint_names_, true);

    for (size_t i = 0; i < position_controlled_joint_names_.size(); ++i)
    {
        const auto joint_it = std::find(joint_names_.begin(), joint_names_.end(), position_controlled_joint_names_[i]);
        if (joint_it == joint_names_.end())
        {
            throw std::runtime_error(
                "position actuator tuning in mixed mode requires position_controlled_joint_names to reference only canonical controlled joints; got '" +
                position_controlled_joint_names_[i] + "'");
        }
        const size_t joint_index = static_cast<size_t>(std::distance(joint_names_.begin(), joint_it));
        if (joint_index >= actuator_ids_.size())
        {
            throw std::runtime_error(
                "position actuator tuning resolved invalid joint index for '" +
                position_controlled_joint_names_[i] + "'");
        }

        const int actuator_id = actuator_ids_[joint_index];
        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            throw std::runtime_error(
                "position actuator tuning resolved invalid actuator id for '" +
                position_controlled_joint_names_[i] + "'");
        }
        if (joint_actuator_backends_[joint_index] != ActuatorBackend::kPosition)
        {
            throw std::runtime_error(
                "position actuator tuning requires a MuJoCo position actuator for joint '" +
                position_controlled_joint_names_[i] + "'");
        }
        if (model_->actuator_gaintype[actuator_id] != mjGAIN_FIXED ||
            model_->actuator_biastype[actuator_id] != mjBIAS_AFFINE)
        {
            throw std::runtime_error(
                "position actuator tuning expects MuJoCo position shortcut layout "
                "(gaintype=fixed, biastype=affine) for joint '" +
                position_controlled_joint_names_[i] + "'");
        }

        const double kp = position_actuator_kp_[i];
        const double kv = position_actuator_kv_[i];
        const double force_limit = std::abs(position_actuator_forcerange_[i]);
        model_->actuator_gainprm[actuator_id * mjNGAIN + 0] = kp;
        model_->actuator_biasprm[actuator_id * mjNBIAS + 1] = -kp;
        model_->actuator_biasprm[actuator_id * mjNBIAS + 2] = -kv;
        model_->actuator_forcelimited[actuator_id] = 1;
        model_->actuator_forcerange[2 * actuator_id + 0] = -force_limit;
        model_->actuator_forcerange[2 * actuator_id + 1] = force_limit;
    }

    std::ostringstream summary;
    summary << "Applied position actuator tuning:";
    for (size_t i = 0; i < position_controlled_joint_names_.size(); ++i)
    {
        summary << " " << position_controlled_joint_names_[i]
                << "(kp=" << position_actuator_kp_[i]
                << ", kv=" << position_actuator_kv_[i]
                << ", force=" << position_actuator_forcerange_[i] << ")";
        if (i + 1 < position_controlled_joint_names_.size())
        {
            summary << ",";
        }
    }
    RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
}

} // namespace mujoco_sim2sim
