#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{

namespace
{
constexpr double kRuntimeCommandFreshnessSec = 0.25;
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

    runtime_command_sub_ = input_node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRuntimeCommand,
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->runtimeCommandCallback(msg);
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
    runtime_command_sub_.reset();
    input_executor_.reset();
    input_node_.reset();
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

void MujocoSimBridge::runtimeCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    rl_master::RobotCommandData command;
    uint32_t sequence = 0;
    double stamp_sec = 0.0;
    if (!rl_master::dds::decodeRuntimeCommand(*msg, &command, &sequence, &stamp_sec))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(runtime_command_mutex_);
    latest_runtime_command_ = std::move(command);
    latest_runtime_command_seq_ = sequence;
    latest_runtime_command_stamp_sec_ = stamp_sec;
    latest_runtime_command_fresh_ = (rl_master::monotonicTimeSec() - stamp_sec) <= kRuntimeCommandFreshnessSec;
    has_runtime_command_ = true;
}

rl_master::TeleopCommand MujocoSimBridge::latestTeleopCommand() const
{
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    return latest_teleop_command_;
}

} // namespace mujoco_sim2sim
