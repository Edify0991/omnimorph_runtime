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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
#include <GLFW/glfw3.h>
#endif

#include "rl_master/dds_protocol.h"
#include "rl_master/command_runtime_mode.h"
#include "rl_master/deploy_state_machine.h"
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
    joint_ids_.fill(-1);
    qpos_addrs_.fill(-1);
    qvel_addrs_.fill(-1);
    actuator_ids_.fill(-1);
    applied_tau_.fill(0.0f);
    last_target_q_.fill(0.0f);
    fixed_base_qpos_.fill(0.0);

    loadParameters();
    loadModel();
    resolveModelMappings();
    initializeState();
    controller_runtime_.initialize(startup_mode_id_);
    mode_command_cache_ = rl_master::kCtrlWordSetModeBase + startup_mode_id_;
    initializeViewer();
    setupRosInterfaces();

    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo sim2sim fused runtime ready. model='%s', control_hz=%.1f, sim_dt=%.6f, substeps=%d, startup_mode_id=%d, viewer=%s, python_viewer_stream=%s, python_viewer_inspector=%s, inactive_behavior=%s",
        model_path_.c_str(),
        control_hz_,
        sim_dt_,
        substeps_per_control_,
        startup_mode_id_,
        enable_viewer_ ? "on" : "off",
        enable_python_viewer_stream_ ? viewer_frame_topic_.c_str() : "off",
        enable_python_viewer_inspector_ ? viewer_inspector_topic_.c_str() : "off",
        no_command_behavior_.c_str());
}

MujocoSimBridge::~MujocoSimBridge()
{
    control_timer_.reset();
    walk_mode_sub_.reset();
    teleop_sub_.reset();
    state_pub_.reset();
    controller_runtime_.estop();
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
    const std::vector<std::string> canonical_names = defaultJointNames();
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("base_body_name", "base_link");
    this->declare_parameter<std::string>("base_free_joint_name", "");
    this->declare_parameter<std::vector<std::string>>("joint_names", canonical_names);
    this->declare_parameter<std::vector<std::string>>("actuator_names", canonical_names);
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
    this->declare_parameter<std::string>("actuator_control_mode", "auto");
    this->declare_parameter<bool>("enable_viewer", false);
    this->declare_parameter<bool>("enable_python_viewer_stream", false);
    this->declare_parameter<std::string>("viewer_frame_topic", kDefaultViewerFrameTopic);
    this->declare_parameter<bool>("enable_python_viewer_inspector", false);
    this->declare_parameter<std::string>("viewer_inspector_topic", "/humanoid/sim2sim/mujoco_viewer_inspector");
    this->declare_parameter<double>("viewer_fps", 60.0);
    this->declare_parameter<int>("viewer_width", 1280);
    this->declare_parameter<int>("viewer_height", 720);
    this->declare_parameter<std::string>("viewer_title", "MuJoCo Sim2Sim Viewer");
    this->declare_parameter<std::vector<double>>("kp", std::vector<double>(kJointCount, 80.0));
    this->declare_parameter<std::vector<double>>("kd", std::vector<double>(kJointCount, 2.0));
    this->declare_parameter<std::vector<double>>("torque_limit", std::vector<double>(kJointCount, 120.0));
    this->declare_parameter<std::vector<double>>("hold_kp", std::vector<double>{80.0});
    this->declare_parameter<std::vector<double>>("hold_kd", std::vector<double>{2.0});
    this->declare_parameter<std::vector<double>>("hold_torque_limit", std::vector<double>{120.0});
    this->declare_parameter<std::vector<double>>("hold_joint_target_q", std::vector<double>{});

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
    actuator_control_mode_ = this->get_parameter("actuator_control_mode").as_string();
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
    const int64_t viewer_width_param = this->get_parameter("viewer_width").as_int();
    const int64_t viewer_height_param = this->get_parameter("viewer_height").as_int();
    viewer_width_ = static_cast<int>(std::clamp<int64_t>(viewer_width_param, 320, 8192));
    viewer_height_ = static_cast<int>(std::clamp<int64_t>(viewer_height_param, 240, 8192));
    viewer_title_ = this->get_parameter("viewer_title").as_string();

    std::vector<std::string> joint_names_param = canonical_names;
    const auto joint_names_param_obj = this->get_parameter("joint_names");
    if (joint_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        joint_names_param = joint_names_param_obj.as_string_array();
    }
    joint_names_ = normalizeNameParam(joint_names_param, canonical_names, kJointCount);

    std::vector<std::string> actuator_names_param = joint_names_;
    const auto actuator_names_param_obj = this->get_parameter("actuator_names");
    if (actuator_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        actuator_names_param = actuator_names_param_obj.as_string_array();
    }
    actuator_names_ = normalizeNameParam(actuator_names_param, joint_names_, kJointCount);

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

    kp_ = normalizeGainParam(this->get_parameter("kp").as_double_array(), 80.0, kJointCount);
    kd_ = normalizeGainParam(this->get_parameter("kd").as_double_array(), 2.0, kJointCount);
    torque_limit_ = normalizeGainParam(this->get_parameter("torque_limit").as_double_array(), 120.0, kJointCount);

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
    int position_like_actuator_count = 0;

    for (size_t i = 0; i < kJointCount; ++i)
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
            if (model_->nu == static_cast<int>(kJointCount))
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

        if (model_->actuator_biastype[actuator_id] != mjBIAS_NONE)
        {
            ++position_like_actuator_count;
        }
    }

    hold_joint_ids_.assign(hold_joint_names_.size(), -1);
    hold_qpos_addrs_.assign(hold_joint_names_.size(), -1);
    hold_qvel_addrs_.assign(hold_joint_names_.size(), -1);
    hold_actuator_ids_.assign(hold_joint_names_.size(), -1);
    hold_applied_tau_.assign(hold_joint_names_.size(), 0.0f);

    for (size_t i = 0; i < hold_joint_names_.size(); ++i)
    {
        const auto existing_it = std::find(joint_names_.begin(), joint_names_.end(), hold_joint_names_[i]);
        if (existing_it != joint_names_.end())
        {
            const size_t existing_idx = static_cast<size_t>(std::distance(joint_names_.begin(), existing_it));
            RCLCPP_WARN(
                this->get_logger(),
                "hold_joint_names[%zu]='%s' overlaps policy-controlled joint index %zu, skip extra-hold mapping.",
                i,
                hold_joint_names_[i].c_str(),
                existing_idx);
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
    }

    const std::string mode_lower = [&]() {
        std::string out = actuator_control_mode_;
        std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
            return static_cast<char>(std::tolower(c));
        });
        return out;
    }();
    if (mode_lower == "position")
    {
        use_position_actuator_control_ = true;
    }
    else if (mode_lower == "torque")
    {
        use_position_actuator_control_ = false;
    }
    else
    {
        use_position_actuator_control_ = (position_like_actuator_count > static_cast<int>(kJointCount / 2));
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Actuator control mode: %s (position_like=%d/%zu), hold_extra_joints=%zu",
        use_position_actuator_control_ ? "position" : "torque",
        position_like_actuator_count,
        kJointCount,
        hold_joint_names_.size());

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

    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        rl_master::dds::kTopicTeleopCommand,
        rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->teleopCallback(msg);
        });

    walk_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicWalkMode,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            this->walkModeCallback(msg);
        });

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { this->controlLoopTick(); });
}

void MujocoSimBridge::initializeState()
{
    for (size_t i = 0; i < kJointCount; ++i)
    {
        const int qpos_adr = qpos_addrs_[i];
        if (qpos_adr >= 0 && qpos_adr < model_->nq)
        {
            last_target_q_[i] = static_cast<float>(data_->qpos[qpos_adr]);
        }
    }

    if (hold_target_q_.empty() && !hold_qpos_addrs_.empty())
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
            "hold_joint_target_q not configured, latch %zu hold joints from model initial qpos.",
            hold_target_q_.size());
    }

    if (fix_base_ && base_free_qpos_adr_ >= 0 && (base_free_qpos_adr_ + 6) < model_->nq)
    {
        for (size_t i = 0; i < fixed_base_qpos_.size(); ++i)
        {
            fixed_base_qpos_[i] = data_->qpos[base_free_qpos_adr_ + static_cast<int>(i)];
        }
        if (fixed_base_height_ >= 0.0)
        {
            fixed_base_qpos_[2] = fixed_base_height_;
        }
        fixed_base_pose_initialized_ = true;
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
    if (!fix_base_ || !fixed_base_pose_initialized_ || base_free_qpos_adr_ < 0 || base_free_qvel_adr_ < 0)
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

void MujocoSimBridge::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }
    latest_teleop_command_.vx = static_cast<float>(msg->linear.x);
    latest_teleop_command_.vy = static_cast<float>(msg->linear.y);
    latest_teleop_command_.dyaw = static_cast<float>(msg->angular.z);
}

void MujocoSimBridge::walkModeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }
    if (!rl_master::DeployStateMachine::isValidControlWord(msg->data))
    {
        if ((this->now() - last_mode_warn_).seconds() > 1.0)
        {
            RCLCPP_WARN(this->get_logger(), "Ignore invalid walk_mode control word: %d", msg->data);
            last_mode_warn_ = this->now();
        }
        return;
    }
    mode_command_cache_ = msg->data;
}

void MujocoSimBridge::controlLoopTick()
{
    const rclcpp::Time now = this->now();
    const bool should_step = !viewer_paused_ || viewer_step_once_;
    const int speed_substeps = std::max(1, static_cast<int>(std::lround(substeps_per_control_ * sim_speed_scale_)));

    const rl_master::RobotStateData state = buildRobotState();
    const rl_master::RobotCommandData command =
        controller_runtime_.step(state, latest_teleop_command_, mode_command_cache_);
    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(true, command.open_rl);
    const bool control_active = runtime_mode.open_rl_active;

    if (!control_active)
    {
        if (!hold_target_latched_)
        {
            for (size_t i = 0; i < kJointCount; ++i)
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

    enforceBaseLock();
    updateControlInput(command, control_active, now);

    if (should_step && (control_active || !pause_when_no_command_))
    {
        for (int i = 0; i < speed_substeps; ++i)
        {
            enforceBaseLock();
            mj_step(model_, data_);
            enforceBaseLock();
        }
        mj_forward(model_, data_);
        viewer_step_once_ = false;
    }
    else
    {
        enforceBaseLock();
        mj_forward(model_, data_);
    }

    const rl_master::RobotStateData post_state = buildRobotState();
    publishRobotState(post_state);
    publishViewerFrame();
    publishViewerInspector(post_state, command, runtime_mode);
    renderViewerFrame();
}

rl_master::RobotStateData MujocoSimBridge::buildRobotState() const
{
    rl_master::RobotStateData state;

    for (size_t i = 0; i < kJointCount; ++i)
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

    if (base_free_qvel_adr_ >= 0 && (base_free_qvel_adr_ + 5) < model_->nv)
    {
        state.base_ang_vel[0] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 3]);
        state.base_ang_vel[1] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 4]);
        state.base_ang_vel[2] = static_cast<float>(data_->qvel[base_free_qvel_adr_ + 5]);
    }
    else if (base_body_id_ >= 0 && base_body_id_ < model_->nbody && data_->cvel)
    {
        const mjtNum *cvel = data_->cvel + 6 * base_body_id_;
        state.base_ang_vel[0] = static_cast<float>(cvel[0]);
        state.base_ang_vel[1] = static_cast<float>(cvel[1]);
        state.base_ang_vel[2] = static_cast<float>(cvel[2]);
    }

    if (base_free_qpos_adr_ >= 0 && (base_free_qpos_adr_ + 6) < model_->nq)
    {
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
    state.syncDynamicFromLegacy();
    return state;
}

void MujocoSimBridge::updateControlInput(
    const rl_master::RobotCommandData &command,
    bool control_active,
    rclcpp::Time now)
{
    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(true, command.open_rl);
    const bool mode_policy = runtime_mode.mode == rl_master::CommandRuntimeMode::kPolicy;
    const bool mode_test_cst = runtime_mode.mode == rl_master::CommandRuntimeMode::kTestCst;
    const bool mode_test_r1 = runtime_mode.mode == rl_master::CommandRuntimeMode::kTestR1;

    auto commandQAt = [&](size_t idx) -> double {
        if (command.protocol_version >= rl_master::kProtocolVersionDynamicJointsV2 &&
            idx < command.joint_target_q_full.size())
        {
            return static_cast<double>(command.joint_target_q_full[idx]);
        }
        return idx < command.joint_target_q.size() ? static_cast<double>(command.joint_target_q[idx]) : 0.0;
    };
    auto commandDqAt = [&](size_t idx) -> double {
        if (command.protocol_version >= rl_master::kProtocolVersionDynamicJointsV2 &&
            idx < command.joint_target_dq_full.size())
        {
            return static_cast<double>(command.joint_target_dq_full[idx]);
        }
        return idx < command.joint_target_dq.size() ? static_cast<double>(command.joint_target_dq[idx]) : 0.0;
    };
    auto commandTauAt = [&](size_t idx) -> double {
        if (command.protocol_version >= rl_master::kProtocolVersionDynamicJointsV2 &&
            idx < command.joint_target_tau_full.size())
        {
            return static_cast<double>(command.joint_target_tau_full[idx]);
        }
        return idx < command.joint_target_tau.size() ? static_cast<double>(command.joint_target_tau[idx]) : 0.0;
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
    if (inactive_hold_position && !use_position_actuator_control_ && !warned_idle_position_fallback_)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "inactive behavior hold_position requested, but actuator mode is torque. "
            "fallback to torque PD hold-last.");
        warned_idle_position_fallback_ = true;
    }

    for (size_t i = 0; i < kJointCount; ++i)
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

        double q_des = static_cast<double>(last_target_q_[i]);
        double dq_des = 0.0;
        double tau_ff = 0.0;
        if (control_active && rl_master::modeUsesPositionTargets(runtime_mode.mode))
        {
            q_des = commandQAt(i);
            if (rl_master::modeUsesVelocityTargets(runtime_mode.mode))
            {
                dq_des = commandDqAt(i);
            }
            if (rl_master::modeUsesTorqueFeedForward(runtime_mode.mode))
            {
                tau_ff = commandTauAt(i);
            }
            last_target_q_[i] = static_cast<float>(q_des);
        }

        if (inactive_zero_torque)
        {
            data_->ctrl[actuator_id] = 0.0;
            applied_tau_[i] = 0.0f;
            continue;
        }

        if (use_position_actuator_control_)
        {
            if (control_active && mode_test_cst)
            {
                q_des = static_cast<double>(last_target_q_[i]);
            }
            data_->ctrl[actuator_id] = q_des;
            applied_tau_[i] = 0.0f;
        }
        else
        {
            double tau = 0.0;
            if (control_active && mode_test_cst)
            {
                tau = commandTauAt(i);
            }
            else
            {
                tau = kp_[i] * (q_des - q) + kd_[i] * (dq_des - dq);
                if (control_active && (mode_policy || mode_test_r1) && use_command_torque_ff_)
                {
                    tau += tau_ff;
                }
            }
            const double limit = std::max(1e-6, std::abs(torque_limit_[i]));
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
        const double q_des = hold_target_q_[i];

        if (use_position_actuator_control_)
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
    msg.data.push_back(static_cast<float>(data_->time));
    msg.data.push_back(control_hz_ > 0.0 ? static_cast<float>(1.0 / control_hz_) : 0.0f);
    msg.data.push_back(fix_base_ ? 1.0f : 0.0f);

    for (int i = 0; i < model_->nq; ++i)
    {
        msg.data.push_back(static_cast<float>(data_->qpos[i]));
    }
    for (int i = 0; i < model_->nv; ++i)
    {
        msg.data.push_back(static_cast<float>(data_->qvel[i]));
    }
    for (int i = 0; i < model_->nu; ++i)
    {
        msg.data.push_back(static_cast<float>(data_->ctrl[i]));
    }

    viewer_frame_pub_->publish(std::move(msg));
}

void MujocoSimBridge::publishViewerInspector(
    const rl_master::RobotStateData &state,
    const rl_master::RobotCommandData &command,
    const rl_master::CommandRuntimeDecision &runtime_mode)
{
    if (!enable_python_viewer_inspector_ || !viewer_inspector_pub_)
    {
        return;
    }

    double mean_abs_joint_error = 0.0;
    double max_abs_joint_error = 0.0;
    for (size_t i = 0; i < kJointCount; ++i)
    {
        const double err = std::abs(static_cast<double>(last_target_q_[i]) - static_cast<double>(state.joint_q[i]));
        mean_abs_joint_error += err;
        max_abs_joint_error = std::max(max_abs_joint_error, err);
    }
    if (kJointCount > 0)
    {
        mean_abs_joint_error /= static_cast<double>(kJointCount);
    }

    std_msgs::msg::String msg;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "mode_id=" << controller_runtime_.activeModeId()
        << " section=" << controller_runtime_.activeConfigSection()
        << " runtime=" << rl_master::commandRuntimeModeName(runtime_mode.mode)
        << " active=" << (runtime_mode.open_rl_active ? "1" : "0")
        << " open_rl=" << static_cast<double>(command.open_rl)
        << " sim_t=" << static_cast<double>(data_->time)
        << " teleop=("
        << static_cast<double>(latest_teleop_command_.vx) << ","
        << static_cast<double>(latest_teleop_command_.vy) << ","
        << static_cast<double>(latest_teleop_command_.dyaw) << ")"
        << " base_rpy=("
        << static_cast<double>(state.base_rpy[0]) << ","
        << static_cast<double>(state.base_rpy[1]) << ","
        << static_cast<double>(state.base_rpy[2]) << ")"
        << " qerr_mean=" << mean_abs_joint_error
        << " qerr_max=" << max_abs_joint_error
        << " paused=" << (viewer_paused_ ? "1" : "0");
    msg.data = oss.str();
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
    const std::vector<std::string> &fallback,
    size_t expected_count)
{
    if (fallback.size() != expected_count)
    {
        throw std::runtime_error("Fallback name vector size mismatch");
    }
    if (input.empty())
    {
        return fallback;
    }
    if (input.size() != expected_count)
    {
        throw std::runtime_error("Name vector size mismatch. Expect " + std::to_string(expected_count));
    }
    return input;
}

} // namespace mujoco_sim2sim
