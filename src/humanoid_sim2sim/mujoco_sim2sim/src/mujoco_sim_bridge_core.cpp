#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

MujocoSimBridge::MujocoSimBridge()
    : rclcpp::Node("mujoco_sim_bridge")
{
    fixed_base_qpos_.fill(0.0);
    mode_registry_ = rl_master::ModeProfileRegistry::loadFromYaml(RL_CFG_PATH, "engineai_walk");

    loadParameters();
    loadModel();
    resolveModelMappings();
    applyPositionActuatorTuning();
    controller_runtime_.initialize(startup_mode_id_);
    initializeState();
    initRuntimeRecorder();
    mode_command_cache_.store(rl_master::kCtrlWordSetModeBase + startup_mode_id_);
    initializeViewer();
    setupRosInterfaces();
    startInputExecutor();
    startStateTelemetry();
    startViewerTelemetry();

    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo sim2sim fused runtime ready. model='%s', control_hz=%.1f, sim_dt=%.6f, substeps=%d, startup_mode_id=%d, viewer=%s, python_viewer_stream=%s, python_viewer_inspector=%s, inactive_behavior=%s, state_telemetry=%s@%.1fHz, sim_base_quat_source_order=%s, sim_base_velocity_source=%s, fixed_base_zeroing=%s, fixed_base_hold=%s, release_before_running=%s, release_settle_ticks=%d, hold_settle_ticks=%d, prepose_snap=%s, sim_only_force_policy_csp=%s",
        model_path_.c_str(),
        control_hz_,
        sim_dt_,
        substeps_per_control_,
        startup_mode_id_,
        enable_viewer_ ? "on" : "off",
        enable_python_viewer_stream_ ? viewer_frame_topic_.c_str() : "off",
        enable_python_viewer_inspector_ ? viewer_inspector_topic_.c_str() : "off",
        no_command_behavior_.c_str(),
        enable_state_telemetry_ ? "on" : "off",
        state_telemetry_hz_,
        controller_runtime_.runtimeCfg().source_contract.sim_base.quat_source_order.c_str(),
        controller_runtime_.runtimeCfg().source_contract.sim_base.velocity_source.c_str(),
        enable_fixed_base_zeroing_ ? "on" : "off",
        enable_fixed_base_hold_after_zeroing_ ? "on" : "off",
        enable_release_before_running_ ? "on" : "off",
        post_release_settle_ticks_,
        post_zeroing_hold_settle_ticks_,
        enable_prepose_snap_ ? "on" : "off",
        sim_only_force_policy_csp_ ? "on" : "off");
}

MujocoSimBridge::~MujocoSimBridge()
{
    control_timer_.reset();
    stopViewerTelemetry();
    stopStateTelemetry();
    stopInputExecutor();
    mode_control_sub_.reset();
    teleop_sub_.reset();
    state_pub_.reset();
    controller_runtime_.estop();
    runtime_recorder_.flush();
    runtime_recorder_.close();
    shutdownViewer();

    if (data_)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}

std::shared_ptr<rclcpp::Node> createMujocoSimBridgeNode()
{
    return std::make_shared<MujocoSimBridge>();
}

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
    int model_position_like_actuator_count = 0;
    joint_actuator_backends_.assign(joint_names_.size(), ActuatorBackend::kTorque);
    hold_actuator_backends_.assign(hold_joint_names_.size(), ActuatorBackend::kTorque);

    const std::string mode_lower = [&]() {
        std::string out = actuator_control_mode_;
        std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return out;
    }();
    use_mixed_actuator_control_ = (mode_lower == "mixed");

    std::vector<bool> joint_expected_position(joint_names_.size(), false);
    if (use_mixed_actuator_control_)
    {
        if (position_actuator_joint_names_.empty())
        {
            throw std::runtime_error(
                "actuator_control_mode=mixed requires non-empty position_actuator_joint_names");
        }
        if (position_actuator_joint_names_ != hold_joint_names_)
        {
            throw std::runtime_error(
                "actuator_control_mode=mixed requires hold_joint_names to exactly match position_actuator_joint_names");
        }
        for (const auto &joint_name : position_actuator_joint_names_)
        {
            const auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
            if (it == joint_names_.end())
            {
                throw std::runtime_error(
                    "position_actuator_joint_names contains unknown joint '" + joint_name + "'");
            }
            const size_t joint_index = static_cast<size_t>(std::distance(joint_names_.begin(), it));
            if (joint_expected_position[joint_index])
            {
                throw std::runtime_error(
                    "position_actuator_joint_names contains duplicate joint '" + joint_name + "'");
            }
            joint_expected_position[joint_index] = true;
        }
    }

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
            if (model_->nu == static_cast<int>(joint_names_.size()))
            {
                actuator_id = static_cast<int>(i);
                RCLCPP_WARN(
                    this->get_logger(),
                    "Actuator '%s' not found, fallback to actuator index %d.",
                    actuator_names_[i].c_str(),
                    actuator_id);
            }
            else
            {
                throw std::runtime_error(
                    "Actuator name not found in MuJoCo model: " + actuator_names_[i]);
            }
        }

        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            throw std::runtime_error("Resolved actuator index out of range for " + actuator_names_[i]);
        }
        actuator_ids_[i] = actuator_id;

        const ActuatorBackend actual_backend = classifyModelActuatorBackend(model_, actuator_id);
        if (actual_backend == ActuatorBackend::kPosition)
        {
            ++model_position_like_actuator_count;
        }

        if (use_mixed_actuator_control_)
        {
            const ActuatorBackend expected_backend =
                joint_expected_position[i] ? ActuatorBackend::kPosition : ActuatorBackend::kTorque;
            if (actual_backend != expected_backend)
            {
                throw std::runtime_error(
                    "mixed actuator validation failed for joint '" + joint_names_[i] +
                    "' actuator '" + actuator_names_[i] + "': expected " +
                    actuatorBackendName(expected_backend) + ", model provides " +
                    actuatorBackendName(actual_backend));
            }
            joint_actuator_backends_[i] = actual_backend;
        }
    }

    hold_joint_ids_.assign(hold_joint_names_.size(), -1);
    hold_qpos_addrs_.assign(hold_joint_names_.size(), -1);
    hold_qvel_addrs_.assign(hold_joint_names_.size(), -1);
    hold_actuator_ids_.assign(hold_joint_names_.size(), -1);
    hold_main_joint_indices_.assign(hold_joint_names_.size(), -1);
    hold_applied_tau_.assign(hold_joint_names_.size(), 0.0f);
    latched_hold_target_q_.assign(hold_joint_names_.size(), 0.0);
    joint_hold_config_indices_.assign(joint_names_.size(), -1);

    size_t hold_extra_joint_count = 0;
    size_t hold_main_joint_count = 0;
    for (size_t i = 0; i < hold_joint_names_.size(); ++i)
    {
        const auto existing_it = std::find(joint_names_.begin(), joint_names_.end(), hold_joint_names_[i]);
        if (existing_it != joint_names_.end())
        {
            const size_t existing_idx = static_cast<size_t>(std::distance(joint_names_.begin(), existing_it));
            if (joint_hold_config_indices_[existing_idx] >= 0)
            {
                throw std::runtime_error(
                    "hold_joint_names contains duplicate main-joint entry: " + hold_joint_names_[i]);
            }
            hold_main_joint_indices_[i] = static_cast<int>(existing_idx);
            joint_hold_config_indices_[existing_idx] = static_cast<int>(i);
            ++hold_main_joint_count;
            continue;
        }

        const int joint_id = mj_name2id(model_, mjOBJ_JOINT, hold_joint_names_[i].c_str());
        if (joint_id < 0)
        {
            throw std::runtime_error("hold joint name not found in MuJoCo model: " + hold_joint_names_[i]);
        }

        const int joint_type = model_->jnt_type[joint_id];
        if (joint_type != mjJNT_HINGE && joint_type != mjJNT_SLIDE)
        {
            throw std::runtime_error("hold controlled joint must be hinge or slide: " + hold_joint_names_[i]);
        }

        hold_joint_ids_[i] = joint_id;
        hold_qpos_addrs_[i] = model_->jnt_qposadr[joint_id];
        hold_qvel_addrs_[i] = model_->jnt_dofadr[joint_id];

        int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, hold_actuator_names_[i].c_str());
        if (actuator_id < 0)
        {
            throw std::runtime_error("hold actuator name not found in MuJoCo model: " + hold_actuator_names_[i]);
        }
        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            throw std::runtime_error("resolved hold actuator index out of range for " + hold_actuator_names_[i]);
        }
        hold_actuator_ids_[i] = actuator_id;
        hold_actuator_backends_[i] = classifyModelActuatorBackend(model_, actuator_id);
        ++hold_extra_joint_count;
    }

    size_t resolved_torque_joint_count = 0;
    size_t resolved_position_joint_count = 0;
    if (use_mixed_actuator_control_)
    {
        use_position_actuator_control_ = false;
        for (size_t i = 0; i < hold_main_joint_indices_.size(); ++i)
        {
            const int main_joint_index = hold_main_joint_indices_[i];
            if (main_joint_index >= 0)
            {
                hold_actuator_backends_[i] = joint_actuator_backends_[static_cast<size_t>(main_joint_index)];
            }
        }
    }
    else if (mode_lower == "position")
    {
        use_position_actuator_control_ = true;
    }
    else
    {
        use_position_actuator_control_ =
            (mode_lower == "auto") ? (model_position_like_actuator_count > static_cast<int>(joint_names_.size() / 2))
                                   : false;
    }

    if (!use_mixed_actuator_control_)
    {
        const ActuatorBackend global_backend =
            use_position_actuator_control_ ? ActuatorBackend::kPosition : ActuatorBackend::kTorque;
        std::fill(joint_actuator_backends_.begin(), joint_actuator_backends_.end(), global_backend);
        std::fill(hold_actuator_backends_.begin(), hold_actuator_backends_.end(), global_backend);
    }

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
        "Actuator control mode: %s (model_position_like=%d/%zu, resolved_torque_joints=%zu, resolved_position_joints=%zu), hold_main_joints=%zu, hold_extra_joints=%zu, hold_target_source=%s",
        use_mixed_actuator_control_ ? "mixed" : (use_position_actuator_control_ ? "position" : "torque"),
        model_position_like_actuator_count,
        joint_names_.size(),
        resolved_torque_joint_count,
        resolved_position_joint_count,
        hold_main_joint_count,
        hold_extra_joint_count,
        holdTargetSourceName(hold_target_source_));

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
    if (!model_ || position_actuator_joint_names_.empty())
    {
        return;
    }
    if (position_actuator_kp_.size() != position_actuator_joint_names_.size() ||
        position_actuator_kv_.size() != position_actuator_joint_names_.size() ||
        position_actuator_forcerange_.size() != position_actuator_joint_names_.size())
    {
        throw std::runtime_error(
            "position actuator tuning vectors must match position_actuator_joint_names");
    }

    for (size_t i = 0; i < position_actuator_joint_names_.size(); ++i)
    {
        const auto joint_it = std::find(joint_names_.begin(), joint_names_.end(), position_actuator_joint_names_[i]);
        if (joint_it == joint_names_.end())
        {
            throw std::runtime_error(
                "position actuator tuning references unknown joint '" +
                position_actuator_joint_names_[i] + "'");
        }
        const size_t joint_index = static_cast<size_t>(std::distance(joint_names_.begin(), joint_it));
        if (joint_index >= actuator_ids_.size())
        {
            throw std::runtime_error(
                "position actuator tuning resolved invalid joint index for '" +
                position_actuator_joint_names_[i] + "'");
        }

        const int actuator_id = actuator_ids_[joint_index];
        if (actuator_id < 0 || actuator_id >= model_->nu)
        {
            throw std::runtime_error(
                "position actuator tuning resolved invalid actuator id for '" +
                position_actuator_joint_names_[i] + "'");
        }
        if (joint_actuator_backends_[joint_index] != ActuatorBackend::kPosition)
        {
            throw std::runtime_error(
                "position actuator tuning requires a MuJoCo position actuator for joint '" +
                position_actuator_joint_names_[i] + "'");
        }
        if (model_->actuator_gaintype[actuator_id] != mjGAIN_FIXED ||
            model_->actuator_biastype[actuator_id] != mjBIAS_AFFINE)
        {
            throw std::runtime_error(
                "position actuator tuning expects MuJoCo position shortcut layout "
                "(gaintype=fixed, biastype=affine) for joint '" +
                position_actuator_joint_names_[i] + "'");
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
    for (size_t i = 0; i < position_actuator_joint_names_.size(); ++i)
    {
        summary << " " << position_actuator_joint_names_[i]
                << "(kp=" << position_actuator_kp_[i]
                << ", kv=" << position_actuator_kv_[i]
                << ", force=" << position_actuator_forcerange_[i] << ")";
        if (i + 1 < position_actuator_joint_names_.size())
        {
            summary << ",";
        }
    }
    RCLCPP_INFO(this->get_logger(), "%s", summary.str().c_str());
}

} // namespace mujoco_sim2sim
