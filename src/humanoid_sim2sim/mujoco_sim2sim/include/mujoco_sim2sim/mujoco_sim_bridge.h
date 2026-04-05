#ifndef MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H
#define MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "rl_master/robot_types.h"

struct mjData_;
struct mjModel_;

namespace mujoco_sim2sim
{

class MujocoSimBridge final : public rclcpp::Node
{
public:
    MujocoSimBridge();
    ~MujocoSimBridge() override;

private:
    static constexpr size_t kJointCount = rl_master::kLegJointCount;
    struct ViewerState;

    struct CommandCache
    {
        rl_master::RobotCommandData command{};
        uint32_t sequence = 0;
        double remote_stamp_sec = 0.0;
        rclcpp::Time receive_time{0, 0, RCL_ROS_TIME};
        bool valid = false;
    };

    void loadParameters();
    void loadModel();
    void resolveModelMappings();
    void setupRosInterfaces();
    void initializeState();
    void initializeViewer();
    void shutdownViewer();
    void renderViewerFrame();

    void commandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void controlLoopTick();
    void enforceBaseLock();

    bool commandFresh(rclcpp::Time now) const;
    void updateControlInput(rclcpp::Time now);
    void publishRobotState();

    static std::array<float, 3> quatXyzwToRpy(const std::array<float, 4> &quat_xyzw);
    static std::vector<double> normalizeGainParam(
        const std::vector<double> &input,
        double fallback,
        size_t expected_count);
    static std::vector<std::string> normalizeNameParam(
        const std::vector<std::string> &input,
        const std::vector<std::string> &fallback,
        size_t expected_count);

    std::string model_path_;
    std::string base_body_name_;
    std::string base_free_joint_name_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> actuator_names_;

    double control_hz_ = 200.0;
    double sim_dt_ = 0.001;
    double command_timeout_sec_ = 0.1;
    double open_rl_enable_threshold_ = 1.0;
    bool use_command_torque_ff_ = false;
    bool pause_when_no_command_ = false;
    bool fix_base_ = false;
    double fixed_base_height_ = -1.0;
    bool enable_viewer_ = false;
    double viewer_fps_ = 60.0;
    int viewer_width_ = 1280;
    int viewer_height_ = 720;
    std::string viewer_title_ = "MuJoCo Sim2Sim Viewer";

    std::vector<double> kp_;
    std::vector<double> kd_;
    std::vector<double> torque_limit_;

    std::array<int, kJointCount> joint_ids_{};
    std::array<int, kJointCount> qpos_addrs_{};
    std::array<int, kJointCount> qvel_addrs_{};
    std::array<int, kJointCount> actuator_ids_{};
    std::array<float, kJointCount> applied_tau_{};
    std::array<float, kJointCount> last_target_q_{};
    std::array<double, 7> fixed_base_qpos_{};
    bool fixed_base_pose_initialized_ = false;

    int base_body_id_ = -1;
    int base_free_joint_id_ = -1;
    int base_free_qpos_adr_ = -1;
    int base_free_qvel_adr_ = -1;
    int substeps_per_control_ = 1;

    mjModel_ *model_ = nullptr;
    mjData_ *data_ = nullptr;
    std::unique_ptr<ViewerState> viewer_state_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr command_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    mutable std::mutex command_mutex_;
    CommandCache latest_command_;
    rclcpp::Time last_timeout_warn_{0, 0, RCL_ROS_TIME};
};

} // namespace mujoco_sim2sim

#endif // MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H
