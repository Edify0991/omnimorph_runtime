#include "rl_master/legacy/dds_robot_io.h"

#include <chrono>

#include "rl_master/deploy_state_machine.h"
#include "rl_master/rl_protocol.h"

DdsRobotIO::~DdsRobotIO()
{
    command_pub_.reset();
    state_sub_.reset();
    teleop_sub_.reset();
    mode_command_sub_.reset();
    node_.reset();

    if (owns_rclcpp_context_ && rclcpp::ok())
    {
        rclcpp::shutdown();
    }
}

void DdsRobotIO::connect()
{
    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
        owns_rclcpp_context_ = true;
    }

    node_ = std::make_shared<rclcpp::Node>("rl_controller_dds_io");

    command_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicPolicyCommand,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            rl_master::RobotStateData parsed_state;
            if (!rl_master::dds::decodeRobotState(*msg, &parsed_state))
            {
                return;
            }
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = parsed_state;
            has_state_ = true;
        });

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

    mode_command_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicWalkMode,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            if (!valid_mode_command(msg->data))
            {
                return;
            }
            std::lock_guard<std::mutex> lock(mode_command_mutex_);
            latest_mode_command_ = msg->data;
            has_mode_command_ = true;
        });

    connected_ = true;
}

bool DdsRobotIO::read_state(rl_master::RobotStateData &state)
{
    if (!connected_)
    {
        return false;
    }
    spin_some();

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_state_)
    {
        return false;
    }
    state = latest_state_;
    return true;
}

bool DdsRobotIO::read_control_command(rl_master::TeleopCommand &command)
{
    if (!connected_)
    {
        return false;
    }
    spin_some();

    std::lock_guard<std::mutex> lock(teleop_mutex_);
    if (!has_teleop_)
    {
        return false;
    }
    command = latest_teleop_;
    return true;
}

int DdsRobotIO::read_mode_command(int fallback_mode)
{
    if (!connected_)
    {
        return fallback_mode;
    }
    spin_some();

    std::lock_guard<std::mutex> lock(mode_command_mutex_);
    if (!has_mode_command_ || !valid_mode_command(latest_mode_command_))
    {
        return fallback_mode;
    }
    return latest_mode_command_;
}

bool DdsRobotIO::write_command(const rl_master::RobotCommandData &command)
{
    if (!connected_)
    {
        return false;
    }

    spin_some();
    const auto msg = rl_master::dds::encodePolicyCommand(
        command,
        ++cmd_sequence_,
        rl_master::monotonicTimeSec());
    command_pub_->publish(msg);
    return true;
}

void DdsRobotIO::estop()
{
    rl_master::RobotCommandData stop_cmd;
    stop_cmd.open_rl = rl_master::kOpenRlDisabled;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (has_state_)
        {
            stop_cmd.joint_target_q = latest_state_.joint_q;
        }
    }
    stop_cmd.joint_target_dq.fill(0.0f);
    stop_cmd.joint_target_tau.fill(0.0f);
    (void)write_command(stop_cmd);
}

void DdsRobotIO::spin_some()
{
    if (!node_)
    {
        return;
    }
    rclcpp::spin_some(node_);
}

bool DdsRobotIO::valid_mode_command(int mode)
{
    return rl_master::DeployStateMachine::isValidControlWord(mode);
}
