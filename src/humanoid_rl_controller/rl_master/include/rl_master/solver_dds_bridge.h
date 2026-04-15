#ifndef RL_MASTER_SOLVER_DDS_BRIDGE_H
#define RL_MASTER_SOLVER_DDS_BRIDGE_H

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "dds_protocol.h"
#include "math_tool.h"
#include "rl_protocol.h"
#include "robot_types.h"

struct JointData;

class SolverDdsBridge
{
public:
    struct StateTelemetryConfig
    {
        bool enabled = true;
        double publish_hz = 50.0;
    };

    SolverDdsBridge() = default;
    ~SolverDdsBridge();

    void connect();
    void connect(const StateTelemetryConfig &telemetry_config);
    void disconnect();
    void updateStateTelemetryConfig(const StateTelemetryConfig &telemetry_config);

    bool readLatestTeleopCommand(rl_master::TeleopCommand *command);
    bool readLatestWalkModeControlWord(int *control_word);

    void buildRobotStateData(
        const std::vector<JointData> &joint_state,
        rl_master::RobotStateData *state);
    void mirrorRobotState(const std::vector<JointData> &joint_state);
    void mirrorRobotState(const rl_master::RobotStateData &state);

private:
    static std::array<float, 4> rpyToQuat(float roll, float pitch, float yaw);
    void executorLoop();
    void telemetryLoop();

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr walk_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::mutex teleop_mutex_;
    rl_master::TeleopCommand latest_teleop_{};
    bool has_teleop_ = false;

    std::mutex walk_mode_mutex_;
    int latest_walk_mode_control_word_ = 0;
    bool has_walk_mode_control_word_ = false;

    std::mutex imu_mutex_;
    std::array<float, 3> imu_ang_vel_{0.0f, 0.0f, 0.0f};
    std::array<float, 4> imu_quat_{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> imu_rpy_{0.0f, 0.0f, 0.0f};

    std::atomic<bool> stop_requested_{false};
    std::thread executor_thread_;
    std::thread telemetry_thread_;

    std::mutex telemetry_mutex_;
    std::condition_variable telemetry_cv_;
    StateTelemetryConfig telemetry_config_{};
    rl_master::RobotStateData latest_mirrored_state_{};
    bool has_mirrored_state_ = false;
};

#endif // RL_MASTER_SOLVER_DDS_BRIDGE_H
