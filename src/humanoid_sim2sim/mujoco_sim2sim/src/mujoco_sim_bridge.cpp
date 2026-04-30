#include "mujoco_sim2sim/mujoco_sim_bridge.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
#include <GLFW/glfw3.h>
#endif

#include "rl_master/dds_protocol.h"
#include "rl_master/command_runtime_mode.h"
#include "rl_master/deploy_state_machine.h"
#include "rl_master/RL_controller.h"
#include "rl_master/rl_protocol.h"

namespace
{
constexpr const char *kDefaultViewerFrameTopic = "/humanoid/sim2sim/mujoco_viewer_frame";
constexpr float kViewerFrameMagic = 260413.0f;
constexpr float kViewerFrameVersion = 1.0f;

std::vector<std::string> defaultJointNames()
{
    return {
        "right_hip_roll",
        "right_hip_yaw",
        "right_hip_pitch",
        "right_knee_pitch",
        "right_ankle_pitch",
        "right_ankle_roll",
        "left_hip_roll",
        "left_hip_yaw",
        "left_hip_pitch",
        "left_knee_pitch",
        "left_ankle_pitch",
        "left_ankle_roll"};
}

bool endsWith(const std::string &value, const std::string &suffix)
{
    if (value.size() < suffix.size())
    {
        return false;
    }
    return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string normalizeNoCommandBehavior(const std::string &raw)
{
    std::string value = raw;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "hold_position" || value == "position_hold" || value == "position" || value == "hold-pos")
    {
        return "hold_position";
    }
    if (value == "zero_torque" || value == "zero" || value == "torque_off" || value == "off")
    {
        return "zero_torque";
    }
    if (value == "hold_last" || value == "hold" || value == "last")
    {
        return "hold_last";
    }
    return "hold_position";
}

std::string trimCopy(const std::string &raw)
{
    size_t begin = 0;
    while (begin < raw.size() && std::isspace(static_cast<unsigned char>(raw[begin])) != 0)
    {
        ++begin;
    }

    size_t end = raw.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(raw[end - 1])) != 0)
    {
        --end;
    }
    return raw.substr(begin, end - begin);
}

std::string toLowerCopy(const std::string &raw)
{
    std::string out = raw;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

const char *baseLockReasonName(mujoco_sim2sim::MujocoSimBridge::BaseLockReason reason)
{
    switch (reason)
    {
    case mujoco_sim2sim::MujocoSimBridge::BaseLockReason::kStartupZeroing:
        return "startup_zeroing";
    case mujoco_sim2sim::MujocoSimBridge::BaseLockReason::kExplicitZeroing:
        return "explicit_zeroing";
    case mujoco_sim2sim::MujocoSimBridge::BaseLockReason::kIncompatibleSwitchZeroing:
        return "incompatible_switch_zeroing";
    case mujoco_sim2sim::MujocoSimBridge::BaseLockReason::kPreRunHold:
        return "pre_run_hold";
    case mujoco_sim2sim::MujocoSimBridge::BaseLockReason::kNone:
    default:
        return "none";
    }
}

std::array<double, 4> normalizeQuatWxyz(std::array<double, 4> q)
{
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm < 1.0e-9)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }
    for (double &v : q)
    {
        v /= norm;
    }
    return q;
}

std::array<double, 4> multiplyQuatWxyz(
    const std::array<double, 4> &a,
    const std::array<double, 4> &b)
{
    return normalizeQuatWxyz({
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    });
}

std::array<double, 4> inverseQuatWxyz(const std::array<double, 4> &q)
{
    const double norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!std::isfinite(norm_sq) || norm_sq < 1.0e-12)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq};
}

std::array<double, 4> yawQuatWxyz(const std::array<double, 4> &q)
{
    const auto nq = normalizeQuatWxyz(q);
    const double yaw = std::atan2(
        2.0 * (nq[0] * nq[3] + nq[1] * nq[2]),
        1.0 - 2.0 * (nq[2] * nq[2] + nq[3] * nq[3]));
    return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}

} // namespace

namespace mujoco_sim2sim
{

#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
struct MujocoSimBridge::ViewerState
{
    GLFWwindow *window = nullptr;
    mjvCamera camera;
    mjvOption option;
    mjvScene scene;
    mjrContext context;
    std::chrono::steady_clock::time_point last_render_time{};
    bool glfw_initialized = false;
    bool scene_initialized = false;
    bool context_initialized = false;

    ViewerState()
    {
        mjv_defaultCamera(&camera);
        mjv_defaultOption(&option);
        mjv_defaultScene(&scene);
        mjr_defaultContext(&context);
    }

    ~ViewerState()
    {
        if (context_initialized)
        {
            mjr_freeContext(&context);
        }
        if (scene_initialized)
        {
            mjv_freeScene(&scene);
        }
        if (window)
        {
            glfwDestroyWindow(window);
            window = nullptr;
        }
        if (glfw_initialized)
        {
            glfwTerminate();
        }
    }
};
#else
struct MujocoSimBridge::ViewerState
{
};
#endif

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

void MujocoSimBridge::setupRosInterfaces()
{
    state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    if (enable_python_viewer_stream_)
    {
        viewer_frame_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            viewer_frame_topic_,
            rclcpp::QoS(rclcpp::KeepLast(2)).best_effort());
    }
    if (enable_python_viewer_inspector_)
    {
        viewer_inspector_pub_ = this->create_publisher<std_msgs::msg::String>(
            viewer_inspector_topic_,
            rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
    }

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { this->controlLoopTick(); });
}

void MujocoSimBridge::startInputExecutor()
{
    stopInputExecutor();

    input_node_ = std::make_shared<rclcpp::Node>("mujoco_sim_bridge_io");
    input_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    teleop_sub_ = input_node_->create_subscription<geometry_msgs::msg::Twist>(
        rl_master::dds::kTopicTeleopCommand,
        rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->teleopCallback(msg);
        });

    mode_control_sub_ = input_node_->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicModeControl,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            this->modeControlCallback(msg);
        });

    io_stop_requested_.store(false);
    input_executor_->add_node(input_node_);
    input_executor_thread_ = std::thread([this]() {
        try
        {
            if (input_executor_)
            {
                input_executor_->spin();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo input executor exception: %s", e.what());
        }
    });
}

void MujocoSimBridge::stopInputExecutor()
{
    io_stop_requested_.store(true);
    telemetry_cv_.notify_all();

    if (input_executor_)
    {
        input_executor_->cancel();
    }
    if (input_executor_thread_.joinable())
    {
        input_executor_thread_.join();
    }
    if (input_executor_ && input_node_)
    {
        input_executor_->remove_node(input_node_);
    }
    mode_control_sub_.reset();
    teleop_sub_.reset();
    input_executor_.reset();
    input_node_.reset();
}

void MujocoSimBridge::startStateTelemetry()
{
    stopStateTelemetry();
    io_stop_requested_.store(false);
    state_telemetry_thread_ = std::thread([this]() {
        while (!io_stop_requested_.load())
        {
            rl_master::RobotStateData state;
            bool enabled = false;
            double publish_hz = 0.0;
            bool has_state = false;

            {
                std::unique_lock<std::mutex> lock(telemetry_mutex_);
                enabled = enable_state_telemetry_;
                publish_hz = state_telemetry_hz_;
                has_state = has_mirrored_state_;
                if (!enabled || publish_hz <= 0.0 || !has_state)
                {
                    telemetry_cv_.wait_for(
                        lock,
                        std::chrono::milliseconds(100),
                        [this]() {
                            return io_stop_requested_.load() ||
                                   (enable_state_telemetry_ && state_telemetry_hz_ > 0.0 && has_mirrored_state_);
                        });
                    continue;
                }
                state = latest_mirrored_state_;
            }

            publishRobotState(state);

            std::unique_lock<std::mutex> lock(telemetry_mutex_);
            telemetry_cv_.wait_for(
                lock,
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / publish_hz)),
                [this]() { return io_stop_requested_.load(); });
        }
    });
}

void MujocoSimBridge::stopStateTelemetry()
{
    io_stop_requested_.store(true);
    telemetry_cv_.notify_all();
    if (state_telemetry_thread_.joinable())
    {
        state_telemetry_thread_.join();
    }
}

void MujocoSimBridge::startViewerTelemetry()
{
    stopViewerTelemetry();
    io_stop_requested_.store(false);
    viewer_telemetry_thread_ = std::thread([this]() {
        auto last_frame_pub = std::chrono::steady_clock::time_point{};
        auto last_inspector_pub = std::chrono::steady_clock::time_point{};

        while (!io_stop_requested_.load())
        {
            std::vector<float> qpos;
            std::vector<float> qvel;
            std::vector<float> ctrl;
            float sim_time = 0.0f;
            std::string inspector_text;
            bool has_frame = false;
            bool has_inspector = false;

            {
                std::unique_lock<std::mutex> lock(viewer_telemetry_mutex_);
                if ((!enable_python_viewer_stream_ || !has_viewer_frame_) &&
                    (!enable_python_viewer_inspector_ || !has_viewer_inspector_))
                {
                    viewer_telemetry_cv_.wait_for(
                        lock,
                        std::chrono::milliseconds(100),
                        [this]() {
                            return io_stop_requested_.load() ||
                                   (enable_python_viewer_stream_ && has_viewer_frame_) ||
                                   (enable_python_viewer_inspector_ && has_viewer_inspector_);
                        });
                    continue;
                }

                has_frame = has_viewer_frame_;
                has_inspector = has_viewer_inspector_;
                if (has_frame)
                {
                    qpos = latest_viewer_qpos_;
                    qvel = latest_viewer_qvel_;
                    ctrl = latest_viewer_ctrl_;
                    sim_time = latest_viewer_sim_time_;
                }
                if (has_inspector)
                {
                    inspector_text = latest_viewer_inspector_text_;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (enable_python_viewer_stream_ && has_frame)
            {
                const auto frame_period = std::chrono::duration<double>(1.0 / std::max(1.0, viewer_fps_));
                if (last_frame_pub.time_since_epoch().count() == 0 ||
                    (now - last_frame_pub) >= frame_period)
                {
                    publishViewerFrameMirror(qpos, qvel, ctrl, sim_time);
                    last_frame_pub = now;
                }
            }

            if (enable_python_viewer_inspector_ && has_inspector)
            {
                const auto inspector_period = std::chrono::duration<double>(1.0 / std::max(0.1, viewer_inspector_hz_));
                if (last_inspector_pub.time_since_epoch().count() == 0 ||
                    (now - last_inspector_pub) >= inspector_period)
                {
                    publishViewerInspectorText(inspector_text);
                    last_inspector_pub = now;
                }
            }

            std::unique_lock<std::mutex> lock(viewer_telemetry_mutex_);
            viewer_telemetry_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() { return io_stop_requested_.load(); });
        }
    });
}

void MujocoSimBridge::stopViewerTelemetry()
{
    io_stop_requested_.store(true);
    viewer_telemetry_cv_.notify_all();
    if (viewer_telemetry_thread_.joinable())
    {
        viewer_telemetry_thread_.join();
    }
}

void MujocoSimBridge::updateMirroredState(const rl_master::RobotStateData &state)
{
    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        should_notify = !has_mirrored_state_;
        latest_mirrored_state_ = state;
        has_mirrored_state_ = true;
    }
    if (should_notify)
    {
        telemetry_cv_.notify_all();
    }
}

void MujocoSimBridge::initRuntimeRecorder()
{
    runtime_logging_enabled_ = false;
    const Sim2realCfg &runtime_cfg = controller_runtime_.runtimeCfg();
    if (!runtime_cfg.logging.enabled)
    {
        return;
    }

    std::ostringstream snapshot;
    snapshot << "{"
             << "\"backend\":\"sim2sim\","
             << "\"config_section\":\"" << controller_runtime_.activeConfigSection() << "\","
             << "\"mode_id\":" << controller_runtime_.activeModeId() << ","
             << "\"policy_name\":\"" << runtime_cfg.policy_name << "\","
             << "\"policy_family\":\"" << runtime_cfg.policy_family << "\","
             << "\"model_path\":\"" << model_path_ << "\","
             << "\"control_hz\":" << control_hz_ << ","
             << "\"policy_hz\":" << runtime_cfg.RL_control_f << ","
             << "\"enable_fixed_base_zeroing\":" << (enable_fixed_base_zeroing_ ? "true" : "false") << ","
             << "\"enable_fixed_base_hold_after_zeroing\":" << (enable_fixed_base_hold_after_zeroing_ ? "true" : "false") << ","
             << "\"enable_release_before_running\":" << (enable_release_before_running_ ? "true" : "false") << ","
             << "\"post_release_settle_ticks\":" << post_release_settle_ticks_ << ","
             << "\"post_zeroing_hold_settle_ticks\":" << post_zeroing_hold_settle_ticks_ << ","
             << "\"enable_prepose_snap\":" << (enable_prepose_snap_ ? "true" : "false") << ","
             << "\"sim_only_force_policy_csp\":" << (sim_only_force_policy_csp_ ? "true" : "false") << ","
             << "\"sim_dt\":" << sim_dt_
             << "}";

    std::map<std::string, std::string> session_metadata;
    session_metadata["backend"] = "sim2sim_mujoco";
    session_metadata["policy_name"] = runtime_cfg.policy_name;
    session_metadata["active_config_section"] = controller_runtime_.activeConfigSection();
    session_metadata["active_mode_id"] = std::to_string(controller_runtime_.activeModeId());
    session_metadata["model_path"] = model_path_;
    session_metadata["output_file_path"] = runtime_cfg.logging.output_file_path;

    if (!runtime_recorder_.open(runtime_cfg.logging, snapshot.str(), session_metadata))
    {
        RCLCPP_ERROR(this->get_logger(), "failed to open sim2sim runtime recorder");
        return;
    }

    runtime_logging_enabled_ = true;

    rl_master::logging::RuntimeEventRecord event;
    event.monotonic_time_sec = rl_master::monotonicTimeSec();
    event.event_type = "sim2sim_initialized";
    event.tags["model_path"] = model_path_;
    event.tags["mode_id"] = std::to_string(controller_runtime_.activeModeId());
    event.tags["config_section"] = controller_runtime_.activeConfigSection();
    event.tags["effective_compression"] = runtime_recorder_.effectiveCompression();
    runtime_recorder_.recordEvent(event);

    RCLCPP_INFO(
        this->get_logger(),
        "Runtime MCAP log: %s",
        runtime_recorder_.filePath().c_str());
}

void MujocoSimBridge::emitDerivedRuntimeEvents(const rl_master::logging::ControllerLogSnapshot &controller_snapshot)
{
    if (!runtime_logging_enabled_ || !runtime_recorder_.isOpen() || !controller_snapshot.valid)
    {
        return;
    }

    if (controller_snapshot.active_mode_id != last_logged_mode_id_)
    {
        rl_master::logging::RuntimeEventRecord event;
        event.monotonic_time_sec = controller_snapshot.monotonic_time_sec;
        event.event_type = "mode_switch";
        event.tags["mode_id"] = std::to_string(controller_snapshot.active_mode_id);
        event.tags["config_section"] = controller_snapshot.active_config_section;
        event.tags["policy_name"] = controller_snapshot.policy_name;
        runtime_recorder_.recordEvent(event);
        last_logged_mode_id_ = controller_snapshot.active_mode_id;
    }

    if (controller_snapshot.deploy_state != last_logged_deploy_state_)
    {
        rl_master::logging::RuntimeEventRecord event;
        event.monotonic_time_sec = controller_snapshot.monotonic_time_sec;
        event.event_type = "lifecycle_transition";
        event.tags["deploy_state"] = std::to_string(controller_snapshot.deploy_state);
        event.tags["mode_id"] = std::to_string(controller_snapshot.active_mode_id);
        runtime_recorder_.recordEvent(event);
        last_logged_deploy_state_ = controller_snapshot.deploy_state;
    }
}

void MujocoSimBridge::emitBaseImuSourceSample(const rl_master::RobotStateData &state, double monotonic_time_sec)
{
    const Sim2realCfg &runtime_cfg = controller_runtime_.runtimeCfg();
    if (!runtime_logging_enabled_ ||
        !runtime_recorder_.isOpen() ||
        !runtime_cfg.logging.source_samples.enabled ||
        !runtime_cfg.logging.source_samples.include_base_imu)
    {
        return;
    }

    rl_master::logging::RuntimeSourceSampleRecord sample;
    sample.monotonic_time_sec = monotonic_time_sec;
    sample.topic = "runtime/source/base_imu";
    sample.sample_name = "base_imu";
    sample.tags["backend"] = "sim2sim";
    sample.values["ang_vel"] = {
        state.base_ang_vel[0],
        state.base_ang_vel[1],
        state.base_ang_vel[2]};
    sample.values["quat_xyzw"] = {
        state.base_quat[0],
        state.base_quat[1],
        state.base_quat[2],
        state.base_quat[3]};
    sample.values["rpy"] = {
        state.base_rpy[0],
        state.base_rpy[1],
        state.base_rpy[2]};
    runtime_recorder_.recordSourceSample(sample);
}

void MujocoSimBridge::logLoopData(
    const rl_master::RobotStateData &state,
    const rl_master::RobotStateData &post_state,
    const rl_master::RobotCommandData &command,
    const rl_master::logging::ControllerLogSnapshot &controller_snapshot,
    const rl_master::CommandRuntimeDecision &runtime_mode,
    bool control_active)
{
    (void)state;
    (void)command;
    (void)runtime_mode;
    if (!runtime_logging_enabled_ || !runtime_recorder_.isOpen())
    {
        return;
    }

    rl_master::logging::RuntimeTickLogRecord record;
    record.frame_index = controller_snapshot.frame_index;
    record.monotonic_time_sec = controller_snapshot.valid
                                    ? controller_snapshot.monotonic_time_sec
                                    : rl_master::monotonicTimeSec();
    record.phase_t = controller_snapshot.phase_t;
    record.phase_t_global = controller_snapshot.phase_t_global;
    record.phase_origin_t = controller_snapshot.phase_origin_t;
    record.requested_mode_command = controller_snapshot.requested_mode_command;
    record.active_mode_id = controller_snapshot.active_mode_id;
    record.deploy_state = controller_snapshot.deploy_state;
    record.active_profile_index = controller_snapshot.active_profile_index;
    record.policy_step_index = controller_snapshot.policy_step_index;
    record.policy_ran_this_tick = controller_snapshot.policy_ran_this_tick;
    record.policy_sample_time_sec = controller_snapshot.policy_sample_time_sec;
    record.policy_sample_age_sec = controller_snapshot.policy_sample_age_sec;
    record.open_rl = controller_snapshot.open_rl;
    record.cmd_vx = controller_snapshot.cmd_vx;
    record.cmd_vy = controller_snapshot.cmd_vy;
    record.cmd_dyaw = controller_snapshot.cmd_dyaw;
    record.latest_cmd_fresh = control_active;
    record.loop_overrun_count = sim_loop_overrun_count_;
    record.active_tag = controller_snapshot.active_tag;
    record.active_config_section = controller_snapshot.active_config_section;
    record.policy_name = controller_snapshot.policy_name;
    record.joint_q = controller_snapshot.joint_q;
    record.joint_dq = controller_snapshot.joint_dq;
    record.joint_tau = controller_snapshot.joint_tau;
    record.joint_target_q = controller_snapshot.joint_target_q;
    record.joint_target_tau = controller_snapshot.joint_target_tau;
    record.observation = controller_snapshot.observation;
    record.policy_action = controller_snapshot.policy_action;
    record.named_features = controller_snapshot.named_features;
    record.external_feature_names = controller_snapshot.external_feature_names;
    record.amp_discriminator_score = controller_snapshot.amp_discriminator_score;
    record.has_amp_discriminator_score = controller_snapshot.has_amp_discriminator_score;
    record.amp_discriminator_score_mean = controller_snapshot.amp_discriminator_score_mean;

    record.joint_cmd_q = joint_cmd_q_;
    record.joint_cmd_dq = joint_cmd_dq_;
    record.joint_cmd_tau = joint_cmd_tau_;
    record.joint_state_q = post_state.joint_q;
    record.joint_state_dq = post_state.joint_dq;
    record.joint_state_tau = post_state.joint_tau;
    record.motor_cmd_q = joint_cmd_q_;
    record.motor_cmd_dq = joint_cmd_dq_;
    record.motor_cmd_tau = applied_tau_;
    record.motor_state_q = post_state.joint_q;
    record.motor_state_dq = post_state.joint_dq;
    record.motor_state_tau = applied_tau_;
    record.motor_cmd_mode = joint_cmd_mode_;

    runtime_recorder_.recordTick(record);

    if (controller_snapshot.policy_ran_this_tick && !controller_snapshot.policy_action.empty())
    {
        rl_master::logging::RuntimeSourceSampleRecord sample;
        sample.monotonic_time_sec = record.monotonic_time_sec;
        sample.topic = "runtime/source/policy_action";
        sample.sample_name = "policy_action";
        sample.tags["backend"] = "sim2sim";
        sample.tags["mode_id"] = std::to_string(record.active_mode_id);
        sample.values["action"] = controller_snapshot.policy_action;
        runtime_recorder_.recordSourceSample(sample);
    }

    if (controller_snapshot.policy_ran_this_tick && !controller_snapshot.observation.empty())
    {
        rl_master::logging::RuntimeSourceSampleRecord sample;
        sample.monotonic_time_sec = controller_snapshot.policy_sample_time_sec > 0.0
                                        ? controller_snapshot.policy_sample_time_sec
                                        : record.monotonic_time_sec;
        sample.topic = "runtime/source/policy_observation";
        sample.sample_name = "policy_observation";
        sample.tags["backend"] = "sim2sim";
        sample.tags["mode_id"] = std::to_string(record.active_mode_id);
        sample.tags["policy_step_index"] = std::to_string(record.policy_step_index);
        sample.values["observation"] = controller_snapshot.observation;
        runtime_recorder_.recordSourceSample(sample);
    }

    if (controller_runtime_.runtimeCfg().logging.source_samples.include_external_observations)
    {
        for (auto &sample : controller_runtime_.controller().drainExternalObservationSamplesForLogging())
        {
            sample.tags["backend"] = "sim2sim";
            runtime_recorder_.recordSourceSample(sample);
        }
    }
}

void MujocoSimBridge::updateViewerFrameMirror()
{
    if (!enable_python_viewer_stream_)
    {
        return;
    }

    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(viewer_telemetry_mutex_);
        should_notify = !has_viewer_frame_;
        latest_viewer_qpos_.assign(model_->nq, 0.0f);
        latest_viewer_qvel_.assign(model_->nv, 0.0f);
        latest_viewer_ctrl_.assign(model_->nu, 0.0f);
        for (int i = 0; i < model_->nq; ++i)
        {
            latest_viewer_qpos_[i] = static_cast<float>(data_->qpos[i]);
        }
        for (int i = 0; i < model_->nv; ++i)
        {
            latest_viewer_qvel_[i] = static_cast<float>(data_->qvel[i]);
        }
        for (int i = 0; i < model_->nu; ++i)
        {
            latest_viewer_ctrl_[i] = static_cast<float>(data_->ctrl[i]);
        }
        latest_viewer_sim_time_ = static_cast<float>(data_->time);
        has_viewer_frame_ = true;
    }
    if (should_notify)
    {
        viewer_telemetry_cv_.notify_all();
    }
}

void MujocoSimBridge::updateViewerInspectorMirror(
    const rl_master::RobotStateData &state,
    const rl_master::RobotCommandData &command,
    const rl_master::CommandRuntimeDecision &runtime_mode)
{
    if (!enable_python_viewer_inspector_)
    {
        return;
    }

    double mean_abs_joint_error = 0.0;
    double max_abs_joint_error = 0.0;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const double err = std::abs(static_cast<double>(last_target_q_[i]) - static_cast<double>(state.joint_q[i]));
        mean_abs_joint_error += err;
        max_abs_joint_error = std::max(max_abs_joint_error, err);
    }
    if (!joint_names_.empty())
    {
        mean_abs_joint_error /= static_cast<double>(joint_names_.size());
    }

    std::ostringstream oss;
    const rl_master::TeleopCommand teleop_command = latestTeleopCommand();
    oss << std::fixed << std::setprecision(3)
        << "mode_id=" << controller_runtime_.activeModeId()
        << " section=" << controller_runtime_.activeConfigSection()
        << " runtime=" << rl_master::commandRuntimeModeName(runtime_mode.mode)
        << " active=" << (runtime_mode.open_rl_active ? "1" : "0")
        << " open_rl=" << static_cast<double>(command.open_rl)
        << " sim_t=" << static_cast<double>(data_->time)
        << " teleop=("
        << static_cast<double>(teleop_command.vx) << ","
        << static_cast<double>(teleop_command.vy) << ","
        << static_cast<double>(teleop_command.dyaw) << ")"
        << " base_rpy=("
        << static_cast<double>(state.base_rpy[0]) << ","
        << static_cast<double>(state.base_rpy[1]) << ","
        << static_cast<double>(state.base_rpy[2]) << ")"
        << " qerr_mean=" << mean_abs_joint_error
        << " qerr_max=" << max_abs_joint_error
        << " paused=" << (viewer_paused_ ? "1" : "0");

    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(viewer_telemetry_mutex_);
        should_notify = !has_viewer_inspector_;
        latest_viewer_inspector_text_ = oss.str();
        has_viewer_inspector_ = true;
    }
    if (should_notify)
    {
        viewer_telemetry_cv_.notify_all();
    }
}

rl_master::TeleopCommand MujocoSimBridge::latestTeleopCommand() const
{
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    return latest_teleop_command_;
}

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

    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        const int qvel_adr = qvel_addrs_[i];
        if (qpos_adr >= 0 && qpos_adr < model_->nq &&
            i < reference_joint_pos->size())
        {
            data_->qpos[qpos_adr] = static_cast<double>((*reference_joint_pos)[i]);
            last_target_q_[i] = (*reference_joint_pos)[i];
        }
        if (qvel_adr >= 0 && qvel_adr < model_->nv)
        {
            const double dq =
                (reference_joint_vel && i < reference_joint_vel->size())
                    ? static_cast<double>((*reference_joint_vel)[i])
                    : 0.0;
            data_->qvel[qvel_adr] = dq;
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

void MujocoSimBridge::initializeViewer()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_)
    {
        return;
    }

    viewer_state_ = std::make_unique<ViewerState>();
    if (!glfwInit())
    {
        RCLCPP_WARN(this->get_logger(), "GLFW init failed. Disable MuJoCo viewer and continue headless.");
        enable_viewer_ = false;
        viewer_state_.reset();
        return;
    }
    viewer_state_->glfw_initialized = true;

    glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);
    viewer_state_->window = glfwCreateWindow(
        viewer_width_,
        viewer_height_,
        viewer_title_.c_str(),
        nullptr,
        nullptr);
    if (!viewer_state_->window)
    {
        RCLCPP_WARN(this->get_logger(), "GLFW window creation failed. Disable MuJoCo viewer and continue headless.");
        enable_viewer_ = false;
        shutdownViewer();
        return;
    }

    glfwMakeContextCurrent(viewer_state_->window);
    glfwSwapInterval(1);
    glfwSetWindowUserPointer(viewer_state_->window, this);
    glfwSetMouseButtonCallback(
        viewer_state_->window,
        [](GLFWwindow *window, int button, int action, int mods) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerMouseButton(button, action, mods);
            }
        });
    glfwSetCursorPosCallback(
        viewer_state_->window,
        [](GLFWwindow *window, double xpos, double ypos) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerMouseMove(xpos, ypos);
            }
        });
    glfwSetScrollCallback(
        viewer_state_->window,
        [](GLFWwindow *window, double, double yoffset) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerScroll(yoffset);
            }
        });
    glfwSetKeyCallback(
        viewer_state_->window,
        [](GLFWwindow *window, int key, int, int action, int mods) {
            auto *bridge = static_cast<MujocoSimBridge *>(glfwGetWindowUserPointer(window));
            if (bridge)
            {
                bridge->handleViewerKey(key, action, mods);
            }
        });

    mjv_makeScene(model_, &viewer_state_->scene, 4000);
    viewer_state_->scene_initialized = true;
    mjr_makeContext(model_, &viewer_state_->context, mjFONTSCALE_150);
    viewer_state_->context_initialized = true;
    viewer_state_->camera.type = mjCAMERA_FREE;
    viewer_state_->camera.azimuth = 90.0;
    viewer_state_->camera.elevation = -20.0;
    viewer_state_->camera.distance = 3.0;
    viewer_state_->last_render_time = std::chrono::steady_clock::now();
    viewer_state_->option.flags[mjVIS_CONTACTPOINT] = viewer_show_contact_ ? 1 : 0;
    viewer_state_->option.flags[mjVIS_CONTACTFORCE] = viewer_show_contact_ ? 1 : 0;

    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo viewer enabled: %dx%d @ %.1fHz",
        viewer_width_,
        viewer_height_,
        viewer_fps_);
#else
    if (enable_viewer_)
    {
        RCLCPP_WARN(this->get_logger(), "Viewer requested but mujoco_sim2sim was built without GLFW support.");
        enable_viewer_ = false;
    }
#endif
}

void MujocoSimBridge::shutdownViewer()
{
    viewer_state_.reset();
}

void MujocoSimBridge::renderViewerFrame()
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_ || !viewer_state_ || !viewer_state_->window)
    {
        return;
    }
    if (glfwWindowShouldClose(viewer_state_->window))
    {
        RCLCPP_INFO(this->get_logger(), "MuJoCo viewer window closed by user. Continue headless.");
        enable_viewer_ = false;
        shutdownViewer();
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double min_render_period = 1.0 / std::max(1.0, viewer_fps_);
    if (viewer_state_->last_render_time.time_since_epoch().count() != 0)
    {
        const double dt = std::chrono::duration<double>(now - viewer_state_->last_render_time).count();
        if (dt < min_render_period)
        {
            return;
        }
    }

    glfwMakeContextCurrent(viewer_state_->window);
    glfwPollEvents();

    int fb_w = 0;
    int fb_h = 0;
    glfwGetFramebufferSize(viewer_state_->window, &fb_w, &fb_h);
    if (fb_w <= 0 || fb_h <= 0)
    {
        return;
    }

    const mjrRect viewport{0, 0, fb_w, fb_h};
    mjv_updateScene(
        model_,
        data_,
        &viewer_state_->option,
        nullptr,
        &viewer_state_->camera,
        mjCAT_ALL,
        &viewer_state_->scene);
    mjr_render(viewport, &viewer_state_->scene, &viewer_state_->context);

    if (viewer_show_hud_)
    {
        std::ostringstream left;
        std::ostringstream right;
        left << "Space: pause/resume\n"
             << "Right: step once\n"
             << "[ / ]: speed -/+\n"
             << "C: toggle contacts\n"
             << "B: toggle base omega\n"
             << "H: toggle HUD\n"
             << "Ncon";
        right << (viewer_paused_ ? "paused" : "running") << "\n"
              << "step\n"
              << sim_speed_scale_ << "x\n"
              << (viewer_show_contact_ ? "on" : "off") << "\n"
              << (viewer_show_base_speed_ ? "on" : "off") << "\n"
              << "on\n"
              << data_->ncon;

        if (viewer_show_base_speed_)
        {
            double wx = 0.0;
            double wy = 0.0;
            double wz = 0.0;
            if (base_free_qvel_adr_ >= 0 && (base_free_qvel_adr_ + 5) < model_->nv)
            {
                wx = data_->qvel[base_free_qvel_adr_ + 3];
                wy = data_->qvel[base_free_qvel_adr_ + 4];
                wz = data_->qvel[base_free_qvel_adr_ + 5];
            }
            left << "\nBase omega";
            right << "\n[" << wx << ", " << wy << ", " << wz << "]";
        }

        mjr_overlay(
            mjFONT_NORMAL,
            mjGRID_TOPLEFT,
            viewport,
            left.str().c_str(),
            right.str().c_str(),
            &viewer_state_->context);
    }

    glfwSwapBuffers(viewer_state_->window);
    viewer_state_->last_render_time = now;
#endif
}

void MujocoSimBridge::handleViewerMouseButton(int button, int action, int)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (action == GLFW_PRESS)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            viewer_mouse_left_down_ = true;
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            viewer_mouse_middle_down_ = true;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            viewer_mouse_right_down_ = true;
        }
    }
    else if (action == GLFW_RELEASE)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
        {
            viewer_mouse_left_down_ = false;
        }
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        {
            viewer_mouse_middle_down_ = false;
        }
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
        {
            viewer_mouse_right_down_ = false;
        }
    }
#else
    (void)button;
    (void)action;
#endif
}

void MujocoSimBridge::handleViewerMouseMove(double xpos, double ypos)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_ || !viewer_state_ || !viewer_state_->window)
    {
        return;
    }

    const double dx = xpos - viewer_last_mouse_x_;
    const double dy = ypos - viewer_last_mouse_y_;
    viewer_last_mouse_x_ = xpos;
    viewer_last_mouse_y_ = ypos;

    if (!viewer_mouse_left_down_ && !viewer_mouse_middle_down_ && !viewer_mouse_right_down_)
    {
        return;
    }

    const int shift_pressed =
        (glfwGetKey(viewer_state_->window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
        (glfwGetKey(viewer_state_->window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    int action = mjMOUSE_ZOOM;
    if (viewer_mouse_right_down_)
    {
        action = shift_pressed ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (viewer_mouse_left_down_)
    {
        action = shift_pressed ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else if (viewer_mouse_middle_down_)
    {
        action = mjMOUSE_ZOOM;
    }

    int width = 0;
    int height = 0;
    glfwGetWindowSize(viewer_state_->window, &width, &height);
    const double norm = std::max(1, height);
    mjv_moveCamera(
        model_,
        action,
        dx / norm,
        dy / norm,
        &viewer_state_->scene,
        &viewer_state_->camera);
#else
    (void)xpos;
    (void)ypos;
#endif
}

void MujocoSimBridge::handleViewerScroll(double yoffset)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (!enable_viewer_ || !viewer_state_)
    {
        return;
    }
    mjv_moveCamera(
        model_,
        mjMOUSE_ZOOM,
        0.0,
        -0.05 * yoffset,
        &viewer_state_->scene,
        &viewer_state_->camera);
#else
    (void)yoffset;
#endif
}

void MujocoSimBridge::handleViewerKey(int key, int action, int)
{
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
    if (action != GLFW_PRESS || !viewer_state_)
    {
        return;
    }

    if (key == GLFW_KEY_SPACE)
    {
        viewer_paused_ = !viewer_paused_;
        return;
    }
    if (key == GLFW_KEY_RIGHT)
    {
        viewer_step_once_ = true;
        viewer_paused_ = true;
        return;
    }
    if (key == GLFW_KEY_LEFT_BRACKET)
    {
        sim_speed_scale_ = std::max(0.1, sim_speed_scale_ / 1.25);
        return;
    }
    if (key == GLFW_KEY_RIGHT_BRACKET)
    {
        sim_speed_scale_ = std::min(4.0, sim_speed_scale_ * 1.25);
        return;
    }
    if (key == GLFW_KEY_C)
    {
        viewer_show_contact_ = !viewer_show_contact_;
        viewer_state_->option.flags[mjVIS_CONTACTPOINT] = viewer_show_contact_ ? 1 : 0;
        viewer_state_->option.flags[mjVIS_CONTACTFORCE] = viewer_show_contact_ ? 1 : 0;
        return;
    }
    if (key == GLFW_KEY_B)
    {
        viewer_show_base_speed_ = !viewer_show_base_speed_;
        return;
    }
    if (key == GLFW_KEY_H)
    {
        viewer_show_hud_ = !viewer_show_hud_;
        return;
    }
#else
    (void)key;
    (void)action;
#endif
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

        if (control_active && joint_mode != SimJointRuntimeMode::kCst)
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

void MujocoSimBridge::publishRobotState(const rl_master::RobotStateData &state)
{
    if (!state_pub_)
    {
        return;
    }
    state_pub_->publish(rl_master::dds::encodeRobotState(state));
}

void MujocoSimBridge::publishViewerFrame()
{
    updateViewerFrameMirror();
}

void MujocoSimBridge::publishViewerFrameMirror(
    const std::vector<float> &qpos,
    const std::vector<float> &qvel,
    const std::vector<float> &ctrl,
    float sim_time)
{
    if (!enable_python_viewer_stream_ || !viewer_frame_pub_)
    {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(
        8 + static_cast<size_t>(model_->nq) + static_cast<size_t>(model_->nv) + static_cast<size_t>(model_->nu));

    msg.data.push_back(kViewerFrameMagic);
    msg.data.push_back(kViewerFrameVersion);
    msg.data.push_back(static_cast<float>(model_->nq));
    msg.data.push_back(static_cast<float>(model_->nv));
    msg.data.push_back(static_cast<float>(model_->nu));
    msg.data.push_back(sim_time);
    msg.data.push_back(control_hz_ > 0.0 ? static_cast<float>(1.0 / control_hz_) : 0.0f);
    msg.data.push_back(shouldEnforceBaseLock() ? 1.0f : 0.0f);

    for (size_t i = 0; i < qpos.size(); ++i)
    {
        msg.data.push_back(qpos[i]);
    }
    for (size_t i = 0; i < qvel.size(); ++i)
    {
        msg.data.push_back(qvel[i]);
    }
    for (size_t i = 0; i < ctrl.size(); ++i)
    {
        msg.data.push_back(ctrl[i]);
    }

    viewer_frame_pub_->publish(std::move(msg));
}

void MujocoSimBridge::publishViewerInspector(
    const rl_master::RobotStateData &state,
    const rl_master::RobotCommandData &command,
    const rl_master::CommandRuntimeDecision &runtime_mode)
{
    updateViewerInspectorMirror(state, command, runtime_mode);
}

void MujocoSimBridge::publishViewerInspectorText(const std::string &text)
{
    if (!enable_python_viewer_inspector_ || !viewer_inspector_pub_)
    {
        return;
    }

    std_msgs::msg::String msg;
    msg.data = text;
    viewer_inspector_pub_->publish(std::move(msg));
}

std::array<float, 3> MujocoSimBridge::quatXyzwToRpy(const std::array<float, 4> &quat_xyzw)
{
    const double x = static_cast<double>(quat_xyzw[0]);
    const double y = static_cast<double>(quat_xyzw[1]);
    const double z = static_cast<double>(quat_xyzw[2]);
    const double w = static_cast<double>(quat_xyzw[3]);

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    constexpr double kPi = 3.14159265358979323846;
    const double pitch = std::abs(sinp) >= 1.0
                             ? std::copysign(kPi / 2.0, sinp)
                             : std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {
        static_cast<float>(roll),
        static_cast<float>(pitch),
        static_cast<float>(yaw)};
}

std::vector<double> MujocoSimBridge::normalizeGainParam(
    const std::vector<double> &input,
    double fallback,
    size_t expected_count)
{
    std::vector<double> out(expected_count, fallback);
    if (input.empty())
    {
        return out;
    }
    if (input.size() == 1)
    {
        out.assign(expected_count, input.front());
        return out;
    }
    if (input.size() != expected_count)
    {
        throw std::runtime_error("Gain vector size mismatch. Expect 1 or " + std::to_string(expected_count));
    }
    out = input;
    return out;
}

std::vector<std::string> MujocoSimBridge::normalizeNameParam(
    const std::vector<std::string> &input,
    const std::vector<std::string> &fallback)
{
    if (input.empty())
    {
        return fallback;
    }
    if (!fallback.empty() && input.size() != fallback.size())
    {
        throw std::runtime_error("Name vector size mismatch. Expect " + std::to_string(fallback.size()));
    }
    return input;
}

} // namespace mujoco_sim2sim
