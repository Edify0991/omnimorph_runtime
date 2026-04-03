#ifndef RL_MASTER_SOLVER_DDS_BRIDGE_H
#define RL_MASTER_SOLVER_DDS_BRIDGE_H

#include <array>
#include <cstdint>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "dds_protocol.h"
#include "math_tool.h"
#include "rl_protocol.h"
#include "robot_types.h"

struct JointData;

class SolverDdsBridge
{
public:
    SolverDdsBridge() = default;
    ~SolverDdsBridge() = default;

    void connect();
    void spinOnce();

    bool readLatestPolicyCommand(
        rl_master::RobotCommandData *command,
        uint32_t *sequence,
        double *stamp_sec);

    void publishRobotState(const std::vector<JointData> &joint_state);

private:
    static std::array<float, 4> rpyToQuat(float roll, float pitch, float yaw);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::mutex command_mutex_;
    rl_master::RobotCommandData latest_command_{};
    uint32_t latest_command_seq_ = 0;
    double latest_command_stamp_sec_ = 0.0;
    bool has_command_ = false;

    std::mutex imu_mutex_;
    std::array<float, 3> imu_ang_vel_{0.0f, 0.0f, 0.0f};
    std::array<float, 4> imu_quat_{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> imu_rpy_{0.0f, 0.0f, 0.0f};
};

#endif // RL_MASTER_SOLVER_DDS_BRIDGE_H
