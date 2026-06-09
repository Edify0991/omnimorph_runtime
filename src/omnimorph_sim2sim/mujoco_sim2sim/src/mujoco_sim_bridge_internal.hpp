#pragma once

#include "mujoco_sim2sim/mujoco_sim_bridge.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#ifdef MUJOCO_SIM2SIM_WITH_VIEWER
#include <GLFW/glfw3.h>
#endif

#include "rl_master/RL_controller.h"
#include "rl_master/command_runtime_mode.h"
#include "rl_master/dds_protocol.h"
#include "rl_master/deploy_state_machine.h"
#include "rl_master/rl_protocol.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "rl_master/logging/runtime_recorder.h"
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

    enum class SimJointRuntimeMode
    {
        kCsp = 0,
        kCst,
        kR1,
    };

    enum class ActuatorBackend
    {
        kTorque = 0,
        kPosition,
    };

    enum class BaseLockReason
    {
        kNone = 0,
        kStartupZeroing,
        kExplicitZeroing,
        kIncompatibleSwitchZeroing,
        kPreRunHold,
    };

private:
    struct ViewerState;
    struct VideoRecorderState;
    struct VideoFrameSnapshot
    {
        std::vector<double> qpos;
        std::vector<double> qvel;
        std::vector<double> ctrl;
        double sim_time = 0.0;
    };

    // Core setup and MuJoCo model mapping.
    void loadParameters();
    void loadModel();
    void resolveModelMappings();
    void refreshPositionActuatorTuning(bool control_active);
    void setupRosInterfaces();
    void initializeState();

    // Native MuJoCo viewer.
    void initializeViewer();
    void shutdownViewer();
    void renderViewerFrame();
    void handleViewerMouseButton(int button, int action, int mods);
    void handleViewerMouseMove(double xpos, double ypos);
    void handleViewerScroll(double yoffset);
    void handleViewerKey(int key, int action, int mods);
    void initializeVideoRecorder();
    void shutdownVideoRecorder();
    void recordVideoFrameIfDue();
    bool writeVideoFrame(const VideoFrameSnapshot &snapshot);

    // Command input, lifecycle handling, and control-loop timing.
    void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void modeControlCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void runtimeCommandCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void controlLoopTick();
    int prepareModeControlWordForTick(int raw_control_word);
    void enforceBaseLock();
    bool shouldEnforceBaseLock() const;
    void captureBaseLockPoseFromModel();
    void activateDynamicBaseLock(BaseLockReason reason, bool apply_prepose);
    void deactivateDynamicBaseLock(const char *reason);
    void applyPreposeSnap();
    void zeroLockedPreRunJointVelocities();
    bool maybeApplyRunningStartReferenceSync(
        const rl_master::logging::ControllerLogSnapshot &controller_snapshot);
    bool applyReferencePoseReplayFrame(
        const rl_master::logging::ControllerLogSnapshot &controller_snapshot);

    // ROS IO, telemetry, and runtime logging.
    void startInputExecutor();
    void stopInputExecutor();
    void startStateTelemetry();
    void stopStateTelemetry();
    void updateMirroredState(const rl_master::RobotStateData &state);
    void initRuntimeRecorder();
    void emitDerivedRuntimeEvents(const rl_master::logging::ControllerLogSnapshot &controller_snapshot);
    void logLoopData(
        const rl_master::RobotStateData &state,
        const rl_master::RobotStateData &post_state,
        const rl_master::RobotCommandData &command,
        const rl_master::logging::ControllerLogSnapshot &controller_snapshot,
        const rl_master::CommandRuntimeDecision &runtime_mode,
        bool control_active);
    void emitBaseImuSourceSample(const rl_master::RobotStateData &state, double monotonic_time_sec);
    rl_master::TeleopCommand latestTeleopCommand() const;

    // Python viewer stream/inspector support.
    void startViewerTelemetry();
    void stopViewerTelemetry();
    void updateViewerFrameMirror();
    void updateViewerInspectorMirror(
        const rl_master::RobotStateData &state,
        const rl_master::RobotCommandData &command,
        const rl_master::CommandRuntimeDecision &runtime_mode);

    // Backend state/command conversion and publishing utilities.
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
    void publishViewerFrameMirror(const std::vector<float> &qpos, const std::vector<float> &qvel, const std::vector<float> &ctrl, float sim_time);
    void publishViewerInspectorText(const std::string &text);
    void resolvePerJointControlConfig(int active_mode_id);

    // Small parsing/math helpers shared by the split implementation files.
    static std::array<float, 3> quatXyzwToRpy(const std::array<float, 4> &quat_xyzw);
    static SimJointRuntimeMode parseSimJointRuntimeMode(
        const std::string &raw_mode,
        const std::string &context);
    static const char *simJointRuntimeModeName(SimJointRuntimeMode mode);
    static ActuatorBackend classifyModelActuatorBackend(const mjModel_ *model, int actuator_id);
    static const char *actuatorBackendName(ActuatorBackend backend);

    // Configured model/control names.
    std::string rl_cfg_path_;
    std::string model_path_;
    std::string base_body_name_;
    std::string base_free_joint_name_;
    std::vector<std::string> joint_names_;
    std::vector<std::string> actuator_names_;
    std::vector<std::string> position_controlled_joint_names_;

    // Runtime parameters loaded from ROS/YAML.
    double control_hz_ = 200.0;
    double sim_dt_ = 0.001;
    int startup_mode_id_ = rl_master::kModeCodeMin;
    bool use_command_torque_ff_ = false;
    bool pause_when_no_command_ = false;
    std::string no_command_behavior_ = "hold_position";
    bool fix_base_ = false;
    double fixed_base_height_ = -1.0;
    bool enable_fixed_base_zeroing_ = false;
    bool enable_fixed_base_hold_after_zeroing_ = false;
    bool enable_release_before_running_ = false;
    int post_release_settle_ticks_ = 0;
    bool enable_prepose_snap_ = false;
    bool sim_sync_running_start_to_reference_ = false;
    bool enable_reference_pose_replay_test_ = false;
    std::vector<double> prepose_joint_q_;
    bool sim_only_force_policy_csp_ = false;
    std::vector<std::string> joint_runtime_mode_override_entries_;
    bool enable_viewer_ = false;
    bool enable_python_viewer_stream_ = false;
    std::string viewer_frame_topic_ = "/omnimorph/sim2sim/mujoco_viewer_frame";
    bool enable_python_viewer_inspector_ = false;
    std::string viewer_inspector_topic_ = "/omnimorph/sim2sim/mujoco_viewer_inspector";
    double viewer_fps_ = 60.0;
    double viewer_inspector_hz_ = 10.0;
    int viewer_width_ = 1280;
    int viewer_height_ = 720;
    std::string viewer_title_ = "MuJoCo Sim2Sim Viewer";
    bool enable_video_recording_ = false;
    std::string video_output_dir_ = "/tmp/omnimorph_sim2sim_videos";
    std::string video_output_name_;
    std::string video_output_path_;
    std::string video_ffmpeg_path_ = "ffmpeg";
    double video_fps_ = 60.0;
    int video_width_ = 1280;
    int video_height_ = 720;
    bool video_follow_robot_ = true;
    double video_follow_distance_ = 3.0;
    double video_follow_azimuth_ = 90.0;
    double video_follow_elevation_ = -20.0;
    std::array<double, 3> video_follow_lookat_offset_{0.0, 0.0, 0.8};

    // Hold and policy-resolved gains/limits.
    std::vector<double> hold_kp_;
    std::vector<double> hold_kd_;
    std::vector<double> hold_torque_limit_;
    bool enable_state_telemetry_ = true;
    double state_telemetry_hz_ = 50.0;

    // MuJoCo joint/actuator mapping caches.
    std::vector<int> joint_ids_;
    std::vector<int> qpos_addrs_;
    std::vector<int> qvel_addrs_;
    std::vector<int> actuator_ids_;
    std::vector<ActuatorBackend> joint_actuator_backends_;
    std::vector<double> default_dof_armature_;
    std::vector<double> default_dof_frictionloss_;
    std::vector<double> default_dof_damping_;
    std::vector<int> position_actuator_joint_indices_;
    std::vector<double> applied_position_actuator_kp_;
    std::vector<double> applied_position_actuator_kv_;
    std::vector<double> applied_position_actuator_forcerange_;
    std::vector<float> applied_tau_;
    std::vector<float> last_target_q_;

    // Hold joints and per-mode resolved control buffers.
    std::vector<SimJointRuntimeMode> resolved_joint_runtime_modes_;
    std::vector<bool> joint_is_policy_controlled_;
    std::vector<double> resolved_policy_profile_kp_;
    std::vector<double> resolved_policy_profile_kd_;
    std::vector<double> resolved_policy_profile_torque_limit_;
    std::vector<double> resolved_dc_motor_velocity_limit_;
    std::vector<double> resolved_pace_encoder_bias_;
    std::vector<int> resolved_pace_torque_delay_ticks_;
    std::vector<std::deque<double>> pace_torque_delay_buffers_;
    std::vector<float> resolved_hold_target_q_;
    std::vector<float> joint_cmd_q_;
    std::vector<float> joint_cmd_dq_;
    std::vector<float> joint_cmd_tau_;
    std::vector<float> joint_cmd_mode_;
    int resolved_control_mode_id_ = std::numeric_limits<int>::min();

    // Fixed-base zeroing, base locking, and RUNNING-entry synchronization.
    std::array<double, 7> fixed_base_qpos_{};
    bool fixed_base_pose_initialized_ = false;
    bool dynamic_base_lock_active_ = false;
    BaseLockReason dynamic_base_lock_reason_ = BaseLockReason::kNone;
    bool zeroing_injection_pending_ = false;
    int release_settle_ticks_remaining_ = 0;
    int post_zeroing_hold_settle_ticks_ = 0;
    int hold_settle_ticks_remaining_ = 0;
    int last_completed_zeroing_mode_id_ = std::numeric_limits<int>::min();
    int last_controller_mode_id_ = std::numeric_limits<int>::min();
    rl_master::DeployLifecycleState last_controller_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;
    bool controller_state_initialized_ = false;
    bool running_start_reference_sync_pending_ = false;
    bool reference_pose_replay_test_logged_ = false;

    // MuJoCo runtime handles and base addresses.
    int base_body_id_ = -1;
    int base_free_joint_id_ = -1;
    int base_free_qpos_adr_ = -1;
    int base_free_qvel_adr_ = -1;
    int substeps_per_control_ = 1;

    mjModel_ *model_ = nullptr;
    mjData_ *data_ = nullptr;

    // Native viewer state.
    std::unique_ptr<ViewerState> viewer_state_;
    std::unique_ptr<VideoRecorderState> video_recorder_state_;
    double next_video_frame_time_ = std::numeric_limits<double>::quiet_NaN();
    uint64_t video_frame_count_ = 0;
    std::thread video_recorder_thread_;
    std::mutex video_recorder_mutex_;
    std::condition_variable video_recorder_cv_;
    std::deque<VideoFrameSnapshot> pending_video_frames_;
    std::atomic<bool> video_recorder_stop_requested_{false};
    std::atomic<bool> video_recorder_failed_{false};
    bool video_queue_overflow_warned_ = false;
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

    // ROS IO and telemetry mirrors.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr viewer_frame_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr viewer_inspector_pub_;
    rclcpp::Node::SharedPtr input_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr input_executor_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_control_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr runtime_command_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::thread input_executor_thread_;
    std::thread state_telemetry_thread_;
    std::thread viewer_telemetry_thread_;
    std::atomic<bool> io_stop_requested_{false};
    mutable std::mutex teleop_mutex_;
    std::atomic<int> mode_command_cache_{rl_master::kCtrlWordSetModeBase + rl_master::kModeCodeMin};
    mutable std::mutex runtime_command_mutex_;
    rl_master::RobotCommandData latest_runtime_command_{};
    bool has_runtime_command_ = false;
    bool latest_runtime_command_fresh_ = false;
    uint32_t latest_runtime_command_seq_ = 0;
    double latest_runtime_command_stamp_sec_ = 0.0;
    std::mutex telemetry_mutex_;
    std::condition_variable telemetry_cv_;
    rl_master::RobotStateData latest_mirrored_state_{};
    bool has_mirrored_state_ = false;
    std::mutex viewer_telemetry_mutex_;
    std::condition_variable viewer_telemetry_cv_;
    std::vector<float> latest_viewer_qpos_;
    std::vector<float> latest_viewer_qvel_;
    std::vector<float> latest_viewer_ctrl_;
    float latest_viewer_sim_time_ = 0.0f;
    bool has_viewer_frame_ = false;
    std::string latest_viewer_inspector_text_;
    bool has_viewer_inspector_ = false;

    // Shared controller runtime and logging state.
    rl_master::runtime::IntegratedControllerRuntime controller_runtime_;
    std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry_;
    rl_master::TeleopCommand latest_teleop_command_{};
    bool hold_target_latched_ = false;
    rclcpp::Time last_mode_warn_{0, 0, RCL_ROS_TIME};
    bool warned_idle_position_fallback_ = false;
    rl_master::logging::RuntimeRecorder runtime_recorder_;
    bool runtime_logging_enabled_ = false;
    int last_logged_mode_id_ = std::numeric_limits<int>::min();
    int last_logged_deploy_state_ = std::numeric_limits<int>::min();
    uint64_t last_logged_runtime_warning_seq_ = 0;
    uint64_t sim_loop_overrun_count_ = 0;
};

} // namespace mujoco_sim2sim

#include "mujoco_sim_bridge_internal_helpers.hpp"
#include "mujoco_sim_bridge_viewer_state.hpp"
