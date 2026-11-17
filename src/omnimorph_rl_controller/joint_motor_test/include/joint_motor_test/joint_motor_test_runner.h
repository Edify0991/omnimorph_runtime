#ifndef JOINT_MOTOR_TEST_RUNNER_H
#define JOINT_MOTOR_TEST_RUNNER_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include "rl_master/deploy_state_machine.h"
#include "rl_master/logging/structured_logger.h"
#include "rl_master/robot_types.h"
#include "joint_motor_test/septic_trajectory.h"

namespace joint_motor_test
{

enum class TrajectorySource
{
    kFile,
    kSine,
    kAcceptance,
};

enum class MotorControlMode
{
    kCsp,
    kCst,
    kR1,
};

enum class SineActivationMode
{
    kAll,
    kSequential,
};

struct SineTrajectoryConfig
{
    double duration_sec = 12.0;
    std::vector<float> offset;
    std::vector<float> amplitude;
    std::vector<float> period_sec;
    std::vector<float> phase_rad;
    SineActivationMode activation_mode = SineActivationMode::kAll;
    std::vector<int> sequential_joint_order;
    double sequential_segment_sec = 2.0;
    std::string export_reference_path;
};

struct AcceptanceJointConfig
{
    std::string name;
    size_t index = 0;
    double q_min = 0.0;
    double q_max = 0.0;
    double kp = 0.0;
    double kd = 0.0;
    double tau_limit = 0.0;
    SepticLimits limits;
    double required_actual_velocity = 0.0;
    double required_actual_range = 0.0;
    double actuator_mass_kg = 0.0;
    std::vector<std::string> coupled_cst_joints;
    std::vector<size_t> coupled_cst_indices;
};

struct AcceptanceTestConfig
{
    double csp_hold_sec = 2.0;
    double dwell_sec = 2.0;
    double state_timeout_sec = 0.10;
    double speed_abort_ratio = 1.15;
    double position_guard_margin = 0.035;
    double pd_gain_scale = 1.0;
    double torque_limit_scale = 1.0;
    std::vector<AcceptanceJointConfig> joints;
};

struct JointMotorTestConfig
{
    int test_mode_id = 90;
    double control_hz = 500.0;
    std::string deploy_config_path;
    std::vector<std::string> joint_names;

    TrajectorySource trajectory_source = TrajectorySource::kFile;
    MotorControlMode control_mode = MotorControlMode::kCsp;

    std::string trajectory_file;
    bool loop_trajectory = true;
    bool restart_trajectory_on_enter_running = true;

    std::string startup_completion_action = "hold";
    double zeroing_duration_s = 2.0;
    std::vector<float> zero_pose;

    std::vector<float> fallback_kp;
    std::vector<float> fallback_kd;
    std::vector<float> tau_limit;
    bool strict_safety_checks = true;
    float max_abs_q = 6.5f;
    float max_abs_dq = 40.0f;
    float max_abs_tau = 200.0f;

    bool save_data = true;
    std::string data_path;

    SineTrajectoryConfig sine;
    AcceptanceTestConfig acceptance;
};

struct TrajectoryFrame
{
    std::vector<float> q;
    std::vector<float> dq;
    std::vector<float> tau;
};

class JointMotorTestRunner
{
public:
    JointMotorTestRunner();
    void run();

private:
    void loadConfig();
    void resolveJointLayout();
    void loadTrajectory();
    void loadTrajectoryFromFile(const std::string &path);
    void generateSineTrajectory();
    void exportTrajectoryToCsv(const std::string &path) const;
    void validateTrajectory() const;
    void validateTrajectoryFrame(const TrajectoryFrame &frame, size_t frame_index) const;

    void initializeStateMachineIfNeeded();
    rl_master::DeployStateOutput updateStateMachine(double now_sec);

    rl_master::RobotCommandData buildPlaybackCommand();
    rl_master::RobotCommandData buildAcceptanceCommand(double now_sec);
    rl_master::RobotCommandData buildZeroingCommand(const std::vector<float> &target_q) const;
    rl_master::RobotCommandData buildDisabledCommand();
    rl_master::RobotCommandData buildCspHoldCommand(const std::vector<float> &hold_q) const;

    enum class AcceptancePhase
    {
        kCspHold,
        kMoveLower,
        kHoldLower,
        kMoveUpper,
        kHoldUpper,
        kMoveHome,
        kHoldHome,
        kComplete,
        kAborted,
    };
    void resetAcceptance(double now_sec);
    void startAcceptanceMotion(AcceptancePhase phase, double q_start, double q_end, double now_sec);
    void transitionAcceptance(AcceptancePhase phase, double now_sec);
    void abortAcceptance(const std::string &reason, double now_sec, const rl_master::RobotStateData &state);
    bool checkAcceptanceSafety(double now_sec, const rl_master::RobotStateData &state, std::string *reason);
    static const char *acceptancePhaseName(AcceptancePhase phase);

    void publishCommand(const rl_master::RobotCommandData &command, double now_sec);
    void logStep(
        const rl_master::DeployStateOutput &deploy_output,
        int mode_command,
        const rl_master::RobotCommandData &command);
    void initLogger();

    void onStateMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void onModeMsg(const std_msgs::msg::Int32::SharedPtr msg);

    static std::string toLower(std::string value);
    static std::vector<std::string> splitCsv(const std::string &line);
    static bool parseNumericRow(const std::string &line, std::vector<double> *values);

    static std::vector<float> normalizeJointVector(const std::vector<float> &input, size_t joint_count, float fallback);
    static std::vector<int> normalizeJointOrder(const std::vector<int> &input, size_t joint_count);

    static MotorControlMode parseControlMode(const std::string &raw);
    static TrajectorySource parseTrajectorySource(const std::string &raw);
    static SineActivationMode parseSineActivationMode(const std::string &raw);

    static float clampTorque(float value, float limit);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;

    JointMotorTestConfig config_;
    std::vector<std::string> joint_names_;
    size_t joint_count_ = 0;

    std::vector<TrajectoryFrame> trajectory_;
    bool trajectory_has_input_dq_ = false;
    bool trajectory_has_input_tau_ = false;
    size_t playback_index_ = 0;

    std::mutex state_mutex_;
    rl_master::RobotStateData latest_state_{};
    std::vector<float> latest_state_acceleration_;
    bool has_state_ = false;
    double latest_state_receive_time_sec_ = 0.0;

    std::mutex mode_mutex_;
    int latest_mode_command_ = -1;

    rl_master::DeployStateMachine state_machine_;
    bool state_machine_initialized_ = false;
    rl_master::DeployLifecycleState last_lifecycle_state_ = rl_master::DeployLifecycleState::kInitializing;

    uint32_t command_sequence_ = 0;
    uint64_t step_index_ = 0;

    rl_master::logging::StructuredLogger logger_;
    bool logger_enabled_ = false;

    bool acceptance_initialized_ = false;
    bool acceptance_complete_ = false;
    bool acceptance_aborted_ = false;
    std::string acceptance_abort_reason_;
    size_t acceptance_joint_cursor_ = 0;
    AcceptancePhase acceptance_phase_ = AcceptancePhase::kCspHold;
    double acceptance_phase_start_sec_ = 0.0;
    double acceptance_motion_duration_sec_ = 0.0;
    double acceptance_motion_q_start_ = 0.0;
    double acceptance_motion_q_end_ = 0.0;
    double acceptance_home_q_ = 0.0;
    SepticSample acceptance_sample_{};
    std::vector<float> acceptance_hold_q_;
    std::vector<float> acceptance_target_velocity_;
    std::vector<float> acceptance_target_acceleration_;
    std::vector<float> acceptance_target_jerk_;
};

} // namespace joint_motor_test

#endif // JOINT_MOTOR_TEST_RUNNER_H
