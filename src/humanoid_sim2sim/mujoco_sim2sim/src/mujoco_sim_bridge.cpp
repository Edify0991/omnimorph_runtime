#include "mujoco_sim2sim/mujoco_sim_bridge.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>

#include "rl_master/dds_protocol.h"
#include "rl_master/rl_protocol.h"

namespace
{
std::vector<std::string> defaultJointNames()
{
    return {
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_hip_pitch_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_hip_pitch_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint"};
}

bool endsWith(const std::string &value, const std::string &suffix)
{
    if (value.size() < suffix.size())
    {
        return false;
    }
    return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
} // namespace

namespace mujoco_sim2sim
{

MujocoSimBridge::MujocoSimBridge()
    : rclcpp::Node("mujoco_sim_bridge")
{
    joint_ids_.fill(-1);
    qpos_addrs_.fill(-1);
    qvel_addrs_.fill(-1);
    actuator_ids_.fill(-1);
    applied_tau_.fill(0.0f);
    last_target_q_.fill(0.0f);

    loadParameters();
    loadModel();
    resolveModelMappings();
    initializeState();
    setupRosInterfaces();

    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo sim2sim bridge ready. model='%s', control_hz=%.1f, sim_dt=%.6f, substeps=%d",
        model_path_.c_str(),
        control_hz_,
        sim_dt_,
        substeps_per_control_);
}

MujocoSimBridge::~MujocoSimBridge()
{
    control_timer_.reset();
    command_sub_.reset();
    state_pub_.reset();

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
    this->declare_parameter<double>("control_hz", 100.0);
    this->declare_parameter<double>("sim_dt", 0.001);
    this->declare_parameter<double>("command_timeout_sec", 0.1);
    this->declare_parameter<double>("open_rl_enable_threshold", 1.0);
    this->declare_parameter<bool>("use_command_torque_ff", false);
    this->declare_parameter<bool>("pause_when_no_command", false);
    this->declare_parameter<std::vector<double>>("kp", std::vector<double>(kJointCount, 80.0));
    this->declare_parameter<std::vector<double>>("kd", std::vector<double>(kJointCount, 2.0));
    this->declare_parameter<std::vector<double>>("torque_limit", std::vector<double>(kJointCount, 120.0));

    model_path_ = this->get_parameter("model_path").as_string();
    base_body_name_ = this->get_parameter("base_body_name").as_string();
    base_free_joint_name_ = this->get_parameter("base_free_joint_name").as_string();
    control_hz_ = std::max(1.0, this->get_parameter("control_hz").as_double());
    sim_dt_ = std::max(1e-5, this->get_parameter("sim_dt").as_double());
    command_timeout_sec_ = std::max(0.01, this->get_parameter("command_timeout_sec").as_double());
    open_rl_enable_threshold_ = this->get_parameter("open_rl_enable_threshold").as_double();
    use_command_torque_ff_ = this->get_parameter("use_command_torque_ff").as_bool();
    pause_when_no_command_ = this->get_parameter("pause_when_no_command").as_bool();

    joint_names_ = normalizeNameParam(
        this->get_parameter("joint_names").as_string_array(),
        canonical_names,
        kJointCount);
    actuator_names_ = normalizeNameParam(
        this->get_parameter("actuator_names").as_string_array(),
        joint_names_,
        kJointCount);

    kp_ = normalizeGainParam(this->get_parameter("kp").as_double_array(), 80.0, kJointCount);
    kd_ = normalizeGainParam(this->get_parameter("kd").as_double_array(), 2.0, kJointCount);
    torque_limit_ = normalizeGainParam(this->get_parameter("torque_limit").as_double_array(), 120.0, kJointCount);

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
    }

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

    command_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicPolicyCommand,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->commandCallback(msg);
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
}

void MujocoSimBridge::commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!msg || msg->data.size() < rl_master::kJointCmdValueCount)
    {
        return;
    }

    CommandCache cache;
    for (size_t i = 0; i < kJointCount; ++i)
    {
        const size_t offset = i * 3;
        cache.command.joint_target_q[i] = msg->data[offset + 0];
        cache.command.joint_target_dq[i] = msg->data[offset + 1];
        cache.command.joint_target_tau[i] = msg->data[offset + 2];
    }
    cache.command.open_rl = msg->data[rl_master::kJointStateValueCount];
    cache.sequence = static_cast<uint32_t>(std::max(0.0f, msg->data[rl_master::kJointCmdSeqIndex]));
    cache.remote_stamp_sec = static_cast<double>(msg->data[rl_master::kJointCmdStampIndex]);
    cache.receive_time = this->now();
    cache.valid = true;

    std::lock_guard<std::mutex> lock(command_mutex_);
    latest_command_ = cache;
}

void MujocoSimBridge::controlLoopTick()
{
    const rclcpp::Time now = this->now();
    const bool has_fresh_command = commandFresh(now);

    updateControlInput(now);

    if (has_fresh_command || !pause_when_no_command_)
    {
        for (int i = 0; i < substeps_per_control_; ++i)
        {
            mj_step(model_, data_);
        }
    }
    else
    {
        mj_forward(model_, data_);
    }

    publishRobotState();
}

bool MujocoSimBridge::commandFresh(rclcpp::Time now) const
{
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!latest_command_.valid)
    {
        return false;
    }
    return (now - latest_command_.receive_time).seconds() <= command_timeout_sec_;
}

void MujocoSimBridge::updateControlInput(rclcpp::Time now)
{
    CommandCache command;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command = latest_command_;
    }

    const bool is_fresh = command.valid &&
                          ((now - command.receive_time).seconds() <= command_timeout_sec_);
    const bool enable_rl = is_fresh &&
                           (command.command.open_rl > static_cast<float>(open_rl_enable_threshold_));

    if (!is_fresh)
    {
        if ((now - last_timeout_warn_).seconds() > 1.0)
        {
            RCLCPP_WARN(this->get_logger(), "Policy command timed out. Entering hold behavior.");
            last_timeout_warn_ = now;
        }
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

        double q_des = q;
        double dq_des = 0.0;
        double tau_ff = 0.0;
        if (enable_rl)
        {
            q_des = static_cast<double>(command.command.joint_target_q[i]);
            dq_des = static_cast<double>(command.command.joint_target_dq[i]);
            tau_ff = static_cast<double>(command.command.joint_target_tau[i]);
            last_target_q_[i] = static_cast<float>(q_des);
        }

        double tau = kp_[i] * (q_des - q) + kd_[i] * (dq_des - dq);
        if (use_command_torque_ff_)
        {
            tau += tau_ff;
        }
        const double limit = std::max(1e-6, std::abs(torque_limit_[i]));
        tau = std::clamp(tau, -limit, limit);

        data_->ctrl[actuator_id] = tau;
        applied_tau_[i] = static_cast<float>(tau);
    }
}

void MujocoSimBridge::publishRobotState()
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

    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(rl_master::dds::kRobotStateValueCount, 0.0f);
    for (size_t i = 0; i < kJointCount; ++i)
    {
        const size_t offset = i * 3;
        msg.data[offset + 0] = state.joint_q[i];
        msg.data[offset + 1] = state.joint_dq[i];
        msg.data[offset + 2] = state.joint_tau[i];
    }

    size_t cursor = rl_master::kJointStateValueCount;
    for (size_t i = 0; i < 3; ++i)
    {
        msg.data[cursor++] = state.base_ang_vel[i];
    }
    for (size_t i = 0; i < 4; ++i)
    {
        msg.data[cursor++] = state.base_quat[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
        msg.data[cursor++] = state.base_rpy[i];
    }
    state_pub_->publish(msg);
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
