#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::loadParameters()
{
    std::vector<std::string> canonical_names = defaultJointNames();
    if (mode_registry_ && !mode_registry_->jointOrder().empty())
    {
        canonical_names = mode_registry_->jointOrder();
    }
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("base_body_name", "base_link");
    this->declare_parameter<std::string>("base_free_joint_name", "");
    this->declare_parameter<std::vector<std::string>>("joint_names", canonical_names);
    this->declare_parameter<std::vector<std::string>>("actuator_names", canonical_names);
    this->declare_parameter<std::vector<std::string>>("position_actuator_joint_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("hold_joint_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("hold_actuator_names", std::vector<std::string>{});
    this->declare_parameter<int>("startup_mode_id", rl_master::kModeCodeMin);
    this->declare_parameter<double>("control_hz", 100.0);
    this->declare_parameter<double>("sim_dt", 0.001);
    this->declare_parameter<bool>("use_command_torque_ff", false);
    this->declare_parameter<bool>("pause_when_no_command", false);
    this->declare_parameter<std::string>("no_command_behavior", "hold_position");
    this->declare_parameter<bool>("fix_base", false);
    this->declare_parameter<double>("fixed_base_height", -1.0);
    this->declare_parameter<bool>("enable_fixed_base_zeroing", false);
    this->declare_parameter<bool>("enable_fixed_base_hold_after_zeroing", false);
    this->declare_parameter<bool>("enable_release_before_running", false);
    this->declare_parameter<int>("post_release_settle_ticks", 0);
    this->declare_parameter<int>("post_zeroing_hold_settle_ticks", 0);
    this->declare_parameter<bool>("enable_prepose_snap", false);
    this->declare_parameter<bool>("sim_sync_running_start_to_reference", false);
    this->declare_parameter<std::vector<double>>("prepose_joint_q", std::vector<double>{});
    this->declare_parameter<bool>("sim_only_force_policy_csp", false);
    this->declare_parameter<std::string>("actuator_control_mode", "auto");
    this->declare_parameter<std::vector<std::string>>("joint_runtime_mode_overrides", std::vector<std::string>{});
    this->declare_parameter<std::string>("hold_target_source", "zero_joint_angles");
    this->declare_parameter<bool>("enable_viewer", false);
    this->declare_parameter<bool>("enable_python_viewer_stream", false);
    this->declare_parameter<std::string>("viewer_frame_topic", kDefaultViewerFrameTopic);
    this->declare_parameter<bool>("enable_python_viewer_inspector", false);
    this->declare_parameter<std::string>("viewer_inspector_topic", "/humanoid/sim2sim/mujoco_viewer_inspector");
    this->declare_parameter<double>("viewer_fps", 60.0);
    this->declare_parameter<double>("viewer_inspector_hz", 10.0);
    this->declare_parameter<int>("viewer_width", 1280);
    this->declare_parameter<int>("viewer_height", 720);
    this->declare_parameter<std::string>("viewer_title", "MuJoCo Sim2Sim Viewer");
    this->declare_parameter<bool>("enable_state_telemetry", true);
    this->declare_parameter<double>("state_telemetry_hz", 50.0);
    this->declare_parameter<std::vector<double>>("kp", std::vector<double>(canonical_names.size(), 80.0));
    this->declare_parameter<std::vector<double>>("kd", std::vector<double>(canonical_names.size(), 2.0));
    this->declare_parameter<std::vector<double>>("torque_limit", std::vector<double>(canonical_names.size(), 120.0));
    this->declare_parameter<std::vector<double>>("hold_kp", std::vector<double>{80.0});
    this->declare_parameter<std::vector<double>>("hold_kd", std::vector<double>{2.0});
    this->declare_parameter<std::vector<double>>("hold_torque_limit", std::vector<double>{120.0});
    this->declare_parameter<std::vector<double>>("hold_joint_target_q", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("position_actuator_kp", std::vector<double>{80.0});
    this->declare_parameter<std::vector<double>>("position_actuator_kv", std::vector<double>{2.0});
    this->declare_parameter<std::vector<double>>("position_actuator_forcerange", std::vector<double>{120.0});

    model_path_ = this->get_parameter("model_path").as_string();
    base_body_name_ = this->get_parameter("base_body_name").as_string();
    base_free_joint_name_ = this->get_parameter("base_free_joint_name").as_string();
    startup_mode_id_ = static_cast<int>(this->get_parameter("startup_mode_id").as_int());
    control_hz_ = std::max(1.0, this->get_parameter("control_hz").as_double());
    sim_dt_ = std::max(1e-5, this->get_parameter("sim_dt").as_double());
    use_command_torque_ff_ = this->get_parameter("use_command_torque_ff").as_bool();
    pause_when_no_command_ = this->get_parameter("pause_when_no_command").as_bool();
    no_command_behavior_ = normalizeNoCommandBehavior(this->get_parameter("no_command_behavior").as_string());
    fix_base_ = this->get_parameter("fix_base").as_bool();
    fixed_base_height_ = this->get_parameter("fixed_base_height").as_double();
    enable_fixed_base_zeroing_ = this->get_parameter("enable_fixed_base_zeroing").as_bool();
    enable_fixed_base_hold_after_zeroing_ = this->get_parameter("enable_fixed_base_hold_after_zeroing").as_bool();
    enable_release_before_running_ = this->get_parameter("enable_release_before_running").as_bool();
    post_release_settle_ticks_ = std::max<int>(0, static_cast<int>(this->get_parameter("post_release_settle_ticks").as_int()));
    post_zeroing_hold_settle_ticks_ = std::max<int>(
        0,
        static_cast<int>(this->get_parameter("post_zeroing_hold_settle_ticks").as_int()));
    enable_prepose_snap_ = this->get_parameter("enable_prepose_snap").as_bool();
    sim_sync_running_start_to_reference_ = this->get_parameter("sim_sync_running_start_to_reference").as_bool();
    prepose_joint_q_ = this->get_parameter("prepose_joint_q").as_double_array();
    sim_only_force_policy_csp_ = this->get_parameter("sim_only_force_policy_csp").as_bool();
    actuator_control_mode_ = this->get_parameter("actuator_control_mode").as_string();
    joint_runtime_mode_override_entries_ = this->get_parameter("joint_runtime_mode_overrides").as_string_array();
    hold_target_source_ = parseHoldTargetSource(this->get_parameter("hold_target_source").as_string());
    enable_viewer_ = this->get_parameter("enable_viewer").as_bool();
    enable_python_viewer_stream_ = this->get_parameter("enable_python_viewer_stream").as_bool();
    viewer_frame_topic_ = this->get_parameter("viewer_frame_topic").as_string();
    if (viewer_frame_topic_.empty())
    {
        viewer_frame_topic_ = kDefaultViewerFrameTopic;
    }
    enable_python_viewer_inspector_ = this->get_parameter("enable_python_viewer_inspector").as_bool();
    viewer_inspector_topic_ = this->get_parameter("viewer_inspector_topic").as_string();
    if (viewer_inspector_topic_.empty())
    {
        viewer_inspector_topic_ = "/humanoid/sim2sim/mujoco_viewer_inspector";
    }
    viewer_fps_ = std::max(1.0, this->get_parameter("viewer_fps").as_double());
    viewer_inspector_hz_ = std::max(0.1, this->get_parameter("viewer_inspector_hz").as_double());
    const int64_t viewer_width_param = this->get_parameter("viewer_width").as_int();
    const int64_t viewer_height_param = this->get_parameter("viewer_height").as_int();
    viewer_width_ = static_cast<int>(std::clamp<int64_t>(viewer_width_param, 320, 8192));
    viewer_height_ = static_cast<int>(std::clamp<int64_t>(viewer_height_param, 240, 8192));
    viewer_title_ = this->get_parameter("viewer_title").as_string();
    enable_state_telemetry_ = this->get_parameter("enable_state_telemetry").as_bool();
    state_telemetry_hz_ = std::max(0.0, this->get_parameter("state_telemetry_hz").as_double());

    std::vector<std::string> joint_names_param = canonical_names;
    const auto joint_names_param_obj = this->get_parameter("joint_names");
    if (joint_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        joint_names_param = joint_names_param_obj.as_string_array();
    }
    joint_names_ = normalizeNameParam(joint_names_param, canonical_names);

    std::vector<std::string> actuator_names_param = joint_names_;
    const auto actuator_names_param_obj = this->get_parameter("actuator_names");
    if (actuator_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        actuator_names_param = actuator_names_param_obj.as_string_array();
    }
    actuator_names_ = normalizeNameParam(actuator_names_param, joint_names_);

    position_actuator_joint_names_.clear();
    const auto position_joint_names_param_obj = this->get_parameter("position_actuator_joint_names");
    if (position_joint_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        position_actuator_joint_names_ = position_joint_names_param_obj.as_string_array();
    }
    const size_t position_actuator_count = position_actuator_joint_names_.size();
    position_actuator_kp_ = normalizeGainParam(
        this->get_parameter("position_actuator_kp").as_double_array(),
        80.0,
        position_actuator_count);
    position_actuator_kv_ = normalizeGainParam(
        this->get_parameter("position_actuator_kv").as_double_array(),
        2.0,
        position_actuator_count);
    position_actuator_forcerange_ = normalizeGainParam(
        this->get_parameter("position_actuator_forcerange").as_double_array(),
        120.0,
        position_actuator_count);
    for (size_t i = 0; i < position_actuator_forcerange_.size(); ++i)
    {
        position_actuator_forcerange_[i] = std::abs(position_actuator_forcerange_[i]);
    }

    auto overrideFromNamedPositionActuatorParams =
        [this, &position_actuator_count](
            const std::string &prefix,
            std::vector<double> *values,
            bool absolute_value) {
            if (!values)
            {
                return;
            }
            std::vector<double> named_values(position_actuator_count, std::numeric_limits<double>::quiet_NaN());
            bool any_named_override = false;
            for (size_t i = 0; i < position_actuator_joint_names_.size(); ++i)
            {
                const std::string param_name = prefix + "." + position_actuator_joint_names_[i];
                this->declare_parameter<double>(
                    param_name,
                    std::numeric_limits<double>::quiet_NaN());
                const double raw_value = this->get_parameter(param_name).as_double();
                if (std::isfinite(raw_value))
                {
                    named_values[i] = absolute_value ? std::abs(raw_value) : raw_value;
                    any_named_override = true;
                }
            }

            if (!any_named_override)
            {
                return;
            }

            for (size_t i = 0; i < position_actuator_joint_names_.size(); ++i)
            {
                if (!std::isfinite(named_values[i]))
                {
                    throw std::runtime_error(
                        "named position actuator parameter set '" + prefix +
                        "' is partial; missing joint '" + position_actuator_joint_names_[i] + "'");
                }
            }
            *values = std::move(named_values);
        };

    overrideFromNamedPositionActuatorParams("position_actuator_kp", &position_actuator_kp_, false);
    overrideFromNamedPositionActuatorParams("position_actuator_kv", &position_actuator_kv_, false);
    overrideFromNamedPositionActuatorParams("position_actuator_forcerange", &position_actuator_forcerange_, true);

    hold_joint_names_.clear();
    const auto hold_joint_names_param_obj = this->get_parameter("hold_joint_names");
    if (hold_joint_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        hold_joint_names_ = hold_joint_names_param_obj.as_string_array();
    }
    hold_actuator_names_.clear();
    const auto hold_actuator_names_param_obj = this->get_parameter("hold_actuator_names");
    if (hold_actuator_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        hold_actuator_names_ = hold_actuator_names_param_obj.as_string_array();
    }
    if (hold_actuator_names_.empty())
    {
        hold_actuator_names_ = hold_joint_names_;
    }
    if (!hold_actuator_names_.empty() && hold_actuator_names_.size() != hold_joint_names_.size())
    {
        throw std::runtime_error("hold_actuator_names size must match hold_joint_names");
    }

    if (!prepose_joint_q_.empty() &&
        prepose_joint_q_.size() != 1 &&
        prepose_joint_q_.size() != joint_names_.size())
    {
        throw std::runtime_error("prepose_joint_q size must be 1 or match joint_names");
    }

    auto overrideFromNamedMainJointParams =
        [this](
            const std::string &prefix,
            const std::vector<std::string> &joint_names,
            std::vector<double> *values,
            bool absolute_value) {
            if (!values)
            {
                return;
            }

            std::vector<double> named_values(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
            bool any_named_override = false;
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                const std::string param_name = prefix + "." + joint_names[i];
                this->declare_parameter<double>(
                    param_name,
                    std::numeric_limits<double>::quiet_NaN());
                const double raw_value = this->get_parameter(param_name).as_double();
                if (std::isfinite(raw_value))
                {
                    named_values[i] = absolute_value ? std::abs(raw_value) : raw_value;
                    any_named_override = true;
                }
            }

            if (!any_named_override)
            {
                return;
            }

            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                if (!std::isfinite(named_values[i]))
                {
                    throw std::runtime_error(
                        "named joint parameter set '" + prefix +
                        "' is partial; missing joint '" + joint_names[i] + "'");
                }
            }
            *values = std::move(named_values);
        };

    kp_ = normalizeGainParam(this->get_parameter("kp").as_double_array(), 80.0, joint_names_.size());
    kd_ = normalizeGainParam(this->get_parameter("kd").as_double_array(), 2.0, joint_names_.size());
    torque_limit_ = normalizeGainParam(this->get_parameter("torque_limit").as_double_array(), 120.0, joint_names_.size());
    overrideFromNamedMainJointParams("kp", joint_names_, &kp_, false);
    overrideFromNamedMainJointParams("kd", joint_names_, &kd_, false);
    overrideFromNamedMainJointParams("torque_limit", joint_names_, &torque_limit_, true);
    joint_ids_.assign(joint_names_.size(), -1);
    qpos_addrs_.assign(joint_names_.size(), -1);
    qvel_addrs_.assign(joint_names_.size(), -1);
    actuator_ids_.assign(joint_names_.size(), -1);
    applied_tau_.assign(joint_names_.size(), 0.0f);
    last_target_q_.assign(joint_names_.size(), 0.0f);
    const size_t hold_count = hold_joint_names_.size();
    hold_kp_ = normalizeGainParam(this->get_parameter("hold_kp").as_double_array(), 80.0, hold_count);
    hold_kd_ = normalizeGainParam(this->get_parameter("hold_kd").as_double_array(), 2.0, hold_count);
    hold_torque_limit_ = normalizeGainParam(this->get_parameter("hold_torque_limit").as_double_array(), 120.0, hold_count);
    const auto hold_target_raw = this->get_parameter("hold_joint_target_q").as_double_array();
    hold_target_q_.clear();
    if (!hold_target_raw.empty())
    {
        if (hold_target_raw.size() == 1 && hold_count > 0)
        {
            hold_target_q_.assign(hold_count, hold_target_raw.front());
        }
        else if (hold_target_raw.size() == hold_count)
        {
            hold_target_q_ = hold_target_raw;
        }
        else
        {
            throw std::runtime_error("hold_joint_target_q size must be 1 or match hold_joint_names");
        }
    }
    if (hold_target_source_ == HoldTargetSource::kExplicit &&
        hold_count > 0 &&
        hold_target_q_.empty())
    {
        throw std::runtime_error(
            "hold_joint_target_q must be provided when hold_target_source=explicit and hold_joint_names is non-empty");
    }
    if (model_path_.empty())
    {
        throw std::runtime_error(
            "Parameter 'model_path' is empty. Please set a MuJoCo xml/mjb file path.");
    }
}

} // namespace mujoco_sim2sim
