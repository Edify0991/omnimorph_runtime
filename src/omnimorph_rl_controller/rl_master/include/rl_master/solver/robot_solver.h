#ifndef RL_MASTER_SOLVER_ROBOT_SOLVER_H
#define RL_MASTER_SOLVER_ROBOT_SOLVER_H

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "rl_master/filters/moving_average_filter.h"
#include "rl_master/kinematics/robot_kinematics_adapter.h"
#include "rl_master/logging/runtime_recorder.h"
#include "rl_master/rl_cfg.h"
#include "rl_master/runtime/integrated_controller_runtime.h"
#include "rl_master/solver/motor_io_backend.h"
#include "rl_master/solver_dds_bridge.h"

namespace rl_master::solver
{

class RobotSolver
{
public:
    ~RobotSolver();
    static std::unique_ptr<RobotSolver> create(
        int startup_mode_id = 0,
        std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry = nullptr);

    bool initialize();
    void run();
    void requestStop();

private:
    RobotSolver() = default;

    void initializeBuffers();
    void initMotorTypes();

    void getMotorState();
    void sendMotorCmd();

    void initializeController();
    void syncRuntimeCfgFromController(bool force = false);
    void applyRuntimeCommand(const rl_master::RobotCommandData &command, bool command_fresh);
    int prepareModeControlWordForTick(int raw_control_word);
    void updateModeControlPreprocessState(const rl_master::logging::ControllerLogSnapshot &controller_snapshot);
    void sendRLState();

    std::map<std::string, std::vector<float>> getRobotStateBag() const;

    std::string buildRuntimeConfigSnapshotJson() const;
    void initRuntimeRecorder();
    void logLoopData(const rl_master::logging::ControllerLogSnapshot &controller_snapshot);
    void emitDerivedRuntimeEvents(const rl_master::logging::ControllerLogSnapshot &controller_snapshot);
    void emitBaseImuSourceSample(
        const std::array<float, 3> &ang_vel,
        const std::array<float, 4> &quat,
        const std::array<float, 3> &rpy,
        double monotonic_time_sec);

    void holdCurrentPose();
    void applyControlGainsFromCfg();
    void cacheInstalledZeroPoseFromCfg();
    void cacheInstalledJointRunModesFromCfg();
    void cacheInstalledJointTauLimitsFromCfg();
    void cacheInstalledMotorTorqueLimitsFromCfg();
    void initModeProfileMap();
    void initializeJointLayout();
    rl_master::RobotStateData buildControllerStateData();
    bool switchToModeConfig(int mode_id, bool allow_fallback_to_default);
    size_t installedJointCount() const;
    size_t motorSlotCount() const;
    bool jointBuffersInitialized() const;

    std::atomic<bool> run_flag_{true};

    std::vector<JointData> joint_state_;
    std::vector<JointData> joint_cmd_;
    std::vector<JointData> motor_state_;
    std::vector<JointData> motor_cmd_;

    std::vector<float> joint_cmd_q_;
    std::vector<float> joint_cmd_dq_;
    std::vector<float> joint_cmd_tau_;
    std::vector<float> joint_state_q_;
    std::vector<float> joint_state_dq_;
    std::vector<float> joint_state_tau_;
    std::vector<float> motor_cmd_q_;
    std::vector<float> motor_cmd_dq_;
    std::vector<float> motor_cmd_tau_;
    std::vector<float> motor_state_q_;
    std::vector<float> motor_state_dq_;
    std::vector<float> motor_state_tau_;
    std::vector<float> motor_cmd_mode_;
    std::vector<float> hold_target_q_;
    bool hold_target_latched_ = false;

    int open_rl_ = 0;
    int last_open_rl_ = 0;
    bool latest_cmd_fresh_ = true;
    uint32_t last_cmd_seq_ = 0;
    double last_stale_warn_time_s_ = 0.0;
    uint64_t loop_overrun_count_ = 0;

    std::array<MotorHandle, kMotorShmSlotCount> motor_feedback_all_{};
    std::array<MotorHandle, kMotorShmSlotCount> motor_target_all_{};
    std::array<uint8_t, kMotorShmSlotCount> motor_types_{};

    std::unique_ptr<rl_master::kinematics::RobotKinematicsAdapter> kinematics_adapter_;
    std::unique_ptr<MotorIoBackend> motor_io_backend_;
    SolverDdsBridge dds_bridge_;

    std::string active_config_section_ = "sim2real";
    Sim2realCfg sim2real_cfg_;
    const rl_master::ModeProfileJointLayout *active_joint_layout_ = nullptr;
    int active_mode_id_ = 0;
    std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry_;
    std::unordered_map<int, std::string> mode_to_config_section_;
    std::vector<DeployModeProfileSpec> mode_profile_specs_;
    std::vector<std::string> installed_joint_names_;
    std::unordered_map<std::string, size_t> installed_joint_index_;
    std::vector<std::string> installed_motor_names_;
    std::unordered_map<std::string, size_t> installed_motor_index_;
    std::vector<float> installed_zero_joint_q_;
    std::vector<float> installed_joint_tau_limits_;
    std::vector<float> installed_motor_torque_limits_;
    std::vector<MotorRunMode> installed_joint_configured_run_modes_;
    MotorRunMode zeroing_run_mode_ = RUN_MODE_CSP;
    int last_mode_reload_failure_id_ = std::numeric_limits<int>::min();

    std::vector<rl_master::filters::MovingAverageFilter> velocity_filters_;
    rl_master::runtime::IntegratedControllerRuntime controller_runtime_;

    rl_master::logging::RuntimeRecorder runtime_recorder_;
    bool runtime_logging_enabled_ = false;
    int last_logged_mode_id_ = std::numeric_limits<int>::min();
    int last_logged_deploy_state_ = std::numeric_limits<int>::min();
    uint64_t last_logged_runtime_warning_seq_ = 0;
    int mode_command_cache_ = rl_master::kCtrlWordSetModeBase + rl_master::kModeCodeMin;
    bool zeroing_injection_pending_ = false;
    int post_zeroing_hold_settle_ticks_ = 0;
    int hold_settle_ticks_remaining_ = 0;
    int last_completed_zeroing_mode_id_ = std::numeric_limits<int>::min();
    int last_controller_mode_id_ = std::numeric_limits<int>::min();
    rl_master::DeployLifecycleState last_controller_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;
    bool controller_state_initialized_ = false;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_ROBOT_SOLVER_H
