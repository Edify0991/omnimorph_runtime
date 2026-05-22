#ifndef RL_MASTER_SOLVER_DDS_BRIDGE_H
#define RL_MASTER_SOLVER_DDS_BRIDGE_H

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "dds_protocol.h"
#include "math_tool.h"
#include "rl_cfg.h"
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
    void updateSourceContract(const SourceContract &source_contract);
    void updateExternalObservationSpecs(const std::vector<ExternalObservationSpec> &specs);
    void setImuSampleCallback(
        std::function<void(
            const std::array<float, 3> &ang_vel,
            const std::array<float, 4> &quat,
            const std::array<float, 3> &rpy,
            double monotonic_time_sec)> callback);
    void setExternalObservationFeatureCallback(
        std::function<void(
            const std::string &name,
            const std::vector<float> &values,
            double monotonic_time_sec)> callback);

    bool readLatestTeleopCommand(rl_master::TeleopCommand *command);
    bool readLatestModeControlWord(int *control_word);
    bool readLatestRuntimeCommand(
        rl_master::RobotCommandData *command,
        bool *fresh,
        uint32_t *sequence = nullptr,
        double *stamp_sec = nullptr);

    void buildRobotStateData(
        const std::vector<JointData> &joint_state,
        rl_master::RobotStateData *state);
    void mirrorRobotState(const std::vector<JointData> &joint_state);
    void mirrorRobotState(const rl_master::RobotStateData &state);

private:
    static std::array<float, 4> rpyToQuat(float roll, float pitch, float yaw);
    void executorLoop();
    void telemetryLoop();
    void configureOdomSubscription();
    void configureExternalObservationSubscriptions();

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr runtime_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> external_observation_subs_;

    std::mutex teleop_mutex_;
    rl_master::TeleopCommand latest_teleop_{};
    bool has_teleop_ = false;

    std::mutex mode_control_mutex_;
    int latest_mode_control_word_ = 0;
    bool has_mode_control_word_ = false;

    std::mutex runtime_command_mutex_;
    rl_master::RobotCommandData latest_runtime_command_{};
    bool has_runtime_command_ = false;
    uint32_t latest_runtime_command_seq_ = 0;
    double latest_runtime_command_stamp_sec_ = 0.0;

    std::mutex imu_mutex_;
    std::array<float, 3> imu_ang_vel_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> imu_lin_acc_{0.0f, 0.0f, 0.0f};
    std::array<float, 4> imu_quat_{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> imu_rpy_{0.0f, 0.0f, 0.0f};
    bool has_imu_sample_ = false;
    std::array<float, 3> odom_pos_w_{0.0f, 0.0f, 0.0f};
    bool has_odom_pose_ = false;
    std::array<float, 3> odom_lin_vel_w_{0.0f, 0.0f, 0.0f};
    bool has_odom_lin_vel_ = false;
    std::string active_odom_topic_;
    SourceContract source_contract_{};
    std::mutex external_observation_mutex_;
    std::vector<ExternalObservationSpec> external_observation_specs_;
    std::function<void(
        const std::array<float, 3> &,
        const std::array<float, 4> &,
        const std::array<float, 3> &,
        double)> imu_sample_callback_;
    std::function<void(
        const std::string &,
        const std::vector<float> &,
        double)> external_observation_callback_;

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
