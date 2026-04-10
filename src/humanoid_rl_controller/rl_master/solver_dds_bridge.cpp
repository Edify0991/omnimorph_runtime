#include "rl_master/solver_dds_bridge.h"

#include <algorithm>
#include <cmath>

#include "rl_master/KinConv.h"

namespace
{
constexpr float kPi = 3.14159265358979323846f;
}

void SolverDdsBridge::connect()
{
    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("rl_solver_dds_bridge");

    state_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    command_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicPolicyCommand,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            rl_master::RobotCommandData cmd;
            uint32_t seq = 0;
            double stamp = 0.0;
            if (!rl_master::dds::decodePolicyCommand(*msg, &cmd, &seq, &stamp))
            {
                return;
            }
            std::lock_guard<std::mutex> lock(command_mutex_);
            latest_command_ = cmd;
            latest_command_seq_ = seq;
            latest_command_stamp_sec_ = stamp;
            has_command_ = true;
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
}

void SolverDdsBridge::spinOnce()
{
    if (!node_)
    {
        return;
    }
    rclcpp::spin_some(node_);
}

bool SolverDdsBridge::readLatestPolicyCommand(
    rl_master::RobotCommandData *command,
    uint32_t *sequence,
    double *stamp_sec)
{
    if (!command)
    {
        return false;
    }
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!has_command_)
    {
        return false;
    }
    *command = latest_command_;
    if (sequence)
    {
        *sequence = latest_command_seq_;
    }
    if (stamp_sec)
    {
        *stamp_sec = latest_command_stamp_sec_;
    }
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

void SolverDdsBridge::publishRobotState(const std::vector<JointData> &joint_state)
{
    if (!state_pub_)
    {
        return;
    }

    rl_master::RobotStateData state{};
    const size_t n = std::min(joint_state.size(), static_cast<size_t>(rl_master::kLegJointCount));
    for (size_t i = 0; i < n; ++i)
    {
        state.joint_q[i] = joint_state[i].q;
        state.joint_dq[i] = joint_state[i].dq;
        state.joint_tau[i] = joint_state[i].tau;
    }

    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        state.base_ang_vel = imu_ang_vel_;
        state.base_quat = imu_quat_;
        state.base_rpy = imu_rpy_;
    }

    const auto msg = rl_master::dds::encodeRobotState(state);
    state_pub_->publish(msg);
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
