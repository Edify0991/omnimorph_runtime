#include "rl_master/solver_dds_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

#include "rl_master/KinConv.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;
}

SolverDdsBridge::~SolverDdsBridge()
{
    disconnect();
}

void SolverDdsBridge::connect()
{
    connect(StateTelemetryConfig{});
}

void SolverDdsBridge::connect(const StateTelemetryConfig &telemetry_config)
{
    disconnect();

    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("rl_solver_dds_bridge");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    state_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    teleop_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        rl_master::dds::kTopicTeleopCommand,
        rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            rl_master::TeleopCommand cmd;
            cmd.vx = static_cast<float>(msg->linear.x);
            cmd.vy = static_cast<float>(msg->linear.y);
            cmd.dyaw = static_cast<float>(msg->angular.z);
            std::lock_guard<std::mutex> lock(teleop_mutex_);
            latest_teleop_ = cmd;
            has_teleop_ = true;
        });

    walk_mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicWalkMode,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(walk_mode_mutex_);
            latest_walk_mode_control_word_ = msg->data;
            has_walk_mode_control_word_ = true;
        });

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/yesense",
        rclcpp::QoS(rclcpp::KeepLast(30)).best_effort(),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
            std::array<float, 3> rpy{};
            std::array<float, 4> quat{};

            const float ow = static_cast<float>(msg->orientation.w);
            const float ox = static_cast<float>(msg->orientation.x);
            const float oy = static_cast<float>(msg->orientation.y);
            const float oz = static_cast<float>(msg->orientation.z);

            // Compatibility with current IMU node:
            // orientation may carry roll/pitch/yaw with w==0.
            if (std::abs(ow) < 1e-6f &&
                std::abs(ox) <= kPi * 2.0f &&
                std::abs(oy) <= kPi * 2.0f &&
                std::abs(oz) <= kPi * 2.0f)
            {
                rpy = {ox, oy, oz};
                quat = rpyToQuat(rpy[0], rpy[1], rpy[2]);
            }
            else
            {
                quat = {ox, oy, oz, ow};
                const std::vector<float> rpy_vec = quaternion_to_euler_array({quat[0], quat[1], quat[2], quat[3]});
                if (rpy_vec.size() >= 3)
                {
                    rpy = {rpy_vec[0], rpy_vec[1], rpy_vec[2]};
                }
                else
                {
                    rpy = {0.0f, 0.0f, 0.0f};
                }
            }

            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_ang_vel_[0] = static_cast<float>(msg->angular_velocity.x);
            imu_ang_vel_[1] = static_cast<float>(msg->angular_velocity.y);
            imu_ang_vel_[2] = static_cast<float>(msg->angular_velocity.z);
            imu_quat_ = quat;
            imu_rpy_ = rpy;
        });

    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_config_ = telemetry_config;
        has_mirrored_state_ = false;
    }
    stop_requested_.store(false);

    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executorLoop(); });
    telemetry_thread_ = std::thread([this]() { telemetryLoop(); });
}

void SolverDdsBridge::disconnect()
{
    stop_requested_.store(true);
    telemetry_cv_.notify_all();

    if (executor_)
    {
        executor_->cancel();
    }

    if (telemetry_thread_.joinable())
    {
        telemetry_thread_.join();
    }
    if (executor_thread_.joinable())
    {
        executor_thread_.join();
    }

    if (executor_ && node_)
    {
        executor_->remove_node(node_);
    }

    imu_sub_.reset();
    walk_mode_sub_.reset();
    teleop_sub_.reset();
    state_pub_.reset();
    executor_.reset();
    node_.reset();
}

void SolverDdsBridge::updateStateTelemetryConfig(const StateTelemetryConfig &telemetry_config)
{
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        telemetry_config_ = telemetry_config;
    }
    telemetry_cv_.notify_all();
}

void SolverDdsBridge::executorLoop()
{
    try
    {
        if (executor_)
        {
            executor_->spin();
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[SolverDdsBridge] executor thread exception: " << e.what() << std::endl;
    }
}

void SolverDdsBridge::telemetryLoop()
{
    while (!stop_requested_.load())
    {
        rl_master::RobotStateData state;
        double publish_hz = 0.0;
        bool enabled = false;
        bool has_state = false;

        {
            std::unique_lock<std::mutex> lock(telemetry_mutex_);
            enabled = telemetry_config_.enabled;
            publish_hz = telemetry_config_.publish_hz;
            has_state = has_mirrored_state_;

            if (!enabled || publish_hz <= 0.0 || !has_state)
            {
                telemetry_cv_.wait_for(
                    lock,
                    std::chrono::milliseconds(100),
                    [this]() {
                        return stop_requested_.load() ||
                               (telemetry_config_.enabled && telemetry_config_.publish_hz > 0.0 && has_mirrored_state_);
                    });
                continue;
            }

            state = latest_mirrored_state_;
        }

        if (state_pub_)
        {
            state_pub_->publish(rl_master::dds::encodeRobotState(state));
        }

        std::unique_lock<std::mutex> lock(telemetry_mutex_);
        telemetry_cv_.wait_for(
            lock,
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / publish_hz)),
            [this]() { return stop_requested_.load(); });
    }
}

bool SolverDdsBridge::readLatestTeleopCommand(rl_master::TeleopCommand *command)
{
    if (!command)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    if (!has_teleop_)
    {
        return false;
    }
    *command = latest_teleop_;
    return true;
}

bool SolverDdsBridge::readLatestWalkModeControlWord(int *control_word)
{
    if (!control_word)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(walk_mode_mutex_);
    if (!has_walk_mode_control_word_)
    {
        return false;
    }
    *control_word = latest_walk_mode_control_word_;
    return true;
}

void SolverDdsBridge::buildRobotStateData(
    const std::vector<JointData> &joint_state,
    rl_master::RobotStateData *state)
{
    if (!state)
    {
        return;
    }

    *state = rl_master::RobotStateData{};
    const size_t n = std::min(joint_state.size(), static_cast<size_t>(rl_master::kLegJointCount));
    for (size_t i = 0; i < n; ++i)
    {
        state->joint_q[i] = joint_state[i].q;
        state->joint_dq[i] = joint_state[i].dq;
        state->joint_tau[i] = joint_state[i].tau;
    }
    state->syncDynamicFromLegacy();

    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        state->base_ang_vel = imu_ang_vel_;
        state->base_quat = imu_quat_;
        state->base_rpy = imu_rpy_;
    }
}

void SolverDdsBridge::mirrorRobotState(const std::vector<JointData> &joint_state)
{
    rl_master::RobotStateData state;
    buildRobotStateData(joint_state, &state);
    mirrorRobotState(state);
}

void SolverDdsBridge::mirrorRobotState(const rl_master::RobotStateData &state)
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

std::array<float, 4> SolverDdsBridge::rpyToQuat(float roll, float pitch, float yaw)
{
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);
    const float cy = std::cos(yaw * 0.5f);
    const float sy = std::sin(yaw * 0.5f);

    std::array<float, 4> q{};
    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
    return q;
}
