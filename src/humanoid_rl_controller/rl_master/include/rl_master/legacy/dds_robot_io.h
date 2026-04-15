#ifndef RL_MASTER_LEGACY_DDS_ROBOT_IO_H
#define RL_MASTER_LEGACY_DDS_ROBOT_IO_H

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rl_master/dds_protocol.h"
#include "rl_master/robot_io.h"

// Legacy standalone RobotIO implementation used only by the old
// two-process controller path:
//   RL_controller (process A) <-> DDS legacy command topic <-> simulator/solver (process B)
//
// The fused runtime path does not depend on this class anymore.
class DdsRobotIO final : public RobotIO
{
public:
    DdsRobotIO() = default;
    ~DdsRobotIO() override;

    void connect() override;

    bool read_state(rl_master::RobotStateData &state) override;
    bool read_control_command(rl_master::TeleopCommand &command) override;
    int read_mode_command(int fallback_mode) override;

    bool write_command(const rl_master::RobotCommandData &command) override;

    void estop() override;

private:
    void spin_some();
    static bool valid_mode_command(int mode);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_command_sub_;

    std::mutex state_mutex_;
    rl_master::RobotStateData latest_state_{};
    bool has_state_ = false;

    std::mutex teleop_mutex_;
    rl_master::TeleopCommand latest_teleop_{};
    bool has_teleop_ = false;

    std::mutex mode_command_mutex_;
    int latest_mode_command_ = 0;
    bool has_mode_command_ = false;

    bool connected_ = false;
    bool owns_rclcpp_context_ = false;
    uint32_t cmd_sequence_ = 0;
};

#endif // RL_MASTER_LEGACY_DDS_ROBOT_IO_H
