#ifndef MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H
#define MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "rl_master/command_runtime_mode.h"
#include "rl_master/runtime/integrated_controller_runtime.h"
#include "rl_master/robot_types.h"

struct mjData_;
struct mjModel_;
struct GLFWwindow;

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

    void loadParameters();
    void loadModel();
    void resolveModelMappings();
    void setupRosInterfaces();
    void initializeState();
    void initializeViewer();
    void shutdownViewer();
    void renderViewerFrame();
    void handleViewerMouseButton(int button, int action, int mods);
    void handleViewerMouseMove(double xpos, double ypos);
    void handleViewerScroll(double yoffset);
    void handleViewerKey(int key, int action, int mods);

    void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void walkModeCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void controlLoopTick();
    void enforceBaseLock();

    rl_master::RobotStateData buildRobotState() const;
    void updateControlInput(
        const rl_master::RobotCommandData &command,
        bool control_active,
        rclcpp::Time now);
    void publishRobotState(const rl_master::RobotStateData &state);
    void publishViewerFrame();
    void publishViewerInspector(
        const rl_master::RobotStateData &state,
        const rl_master::RobotCommandData &command,
        const rl_master::CommandRuntimeDecision &runtime_mode);

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
    std::vector<std::string> hold_joint_names_;
    std::vector<std::string> hold_actuator_names_;

    double control_hz_ = 200.0;
    double sim_dt_ = 0.001;
    int startup_mode_id_ = rl_master::kModeCodeMin;
    bool use_command_torque_ff_ = false;
    bool pause_when_no_command_ = false;
    std::string no_command_behavior_ = "hold_position";
    bool fix_base_ = false;
    double fixed_base_height_ = -1.0;
    std::string actuator_control_mode_ = "auto";
    bool use_position_actuator_control_ = false;
    bool enable_viewer_ = false;
    bool enable_python_viewer_stream_ = false;
    std::string viewer_frame_topic_ = "/humanoid/sim2sim/mujoco_viewer_frame";
    bool enable_python_viewer_inspector_ = false;
    std::string viewer_inspector_topic_ = "/humanoid/sim2sim/mujoco_viewer_inspector";
    double viewer_fps_ = 60.0;
    int viewer_width_ = 1280;
    int viewer_height_ = 720;
    std::string viewer_title_ = "MuJoCo Sim2Sim Viewer";

    std::vector<double> kp_;
    std::vector<double> kd_;
    std::vector<double> torque_limit_;
    std::vector<double> hold_kp_;
    std::vector<double> hold_kd_;
    std::vector<double> hold_torque_limit_;
    std::vector<double> hold_target_q_;

    std::array<int, kJointCount> joint_ids_{};
    std::array<int, kJointCount> qpos_addrs_{};
    std::array<int, kJointCount> qvel_addrs_{};
    std::array<int, kJointCount> actuator_ids_{};
    std::array<float, kJointCount> applied_tau_{};
    std::array<float, kJointCount> last_target_q_{};

    std::vector<int> hold_joint_ids_;
    std::vector<int> hold_qpos_addrs_;
    std::vector<int> hold_qvel_addrs_;
    std::vector<int> hold_actuator_ids_;
    std::vector<float> hold_applied_tau_;

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
    bool viewer_mouse_left_down_ = false;
    bool viewer_mouse_middle_down_ = false;
    bool viewer_mouse_right_down_ = false;
    double viewer_last_mouse_x_ = 0.0;
    double viewer_last_mouse_y_ = 0.0;
    bool viewer_show_contact_ = false;
    bool viewer_show_hud_ = true;
    bool viewer_show_base_speed_ = true;
    bool viewer_paused_ = false;
    bool viewer_step_once_ = false;
    double sim_speed_scale_ = 1.0;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr viewer_frame_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr viewer_inspector_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr walk_mode_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rl_master::runtime::IntegratedControllerRuntime controller_runtime_;
    rl_master::TeleopCommand latest_teleop_command_{};
    int mode_command_cache_ = rl_master::kCtrlWordSetModeBase + rl_master::kModeCodeMin;
    bool hold_target_latched_ = false;
    rclcpp::Time last_mode_warn_{0, 0, RCL_ROS_TIME};
    bool warned_idle_position_fallback_ = false;
};

} // namespace mujoco_sim2sim

#endif // MUJOCO_SIM2SIM_MUJOCO_SIM_BRIDGE_H
