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

#include "rl_master/KinConv.h"
#include "rl_master/filters/moving_average_filter.h"
#include "rl_master/logging/structured_logger.h"
#include "rl_master/rl_cfg.h"
#include "rl_master/runtime/integrated_controller_runtime.h"
#include "rl_master/solver/motor_shm_io.h"
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

    std::vector<float> computePdControl(
        const std::vector<float> &target_q,
        const std::vector<float> &target_dq) const;

    void getMotorState();
    void sendMotorCmd();

    void initializeController();
    void syncRuntimeCfgFromController(bool force = false);
    void applyRuntimeCommand(const rl_master::RobotCommandData &command, bool command_fresh);
    void sendRLState();

    std::map<std::string, std::vector<float>> getRobotStateBag() const;

    rl_master::logging::LoggerMetadata buildLoggerMetadata() const;
    void initDataLogger();
    void logLoopData();

    void moveToPosition(const std::vector<float> &target_positions);
    void holdCurrentPose();
    void applyControlGainsFromCfg();
    void initModeProfileMap();
    void initializeJointLayout();
    rl_master::RobotStateData buildControllerStateData() const;
    bool switchToModeConfig(int mode_id, bool allow_fallback_to_default);

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

    std::array<MotorHandle, kMotorCountMax> motor_feedback_all_{};
    std::array<MotorHandle, kMotorCountMax> motor_target_all_{};
    std::array<uint8_t, kMotorCountMax> motor_types_{};

    KinConv kin_conv_;
    SolverDdsBridge dds_bridge_;
    MotorShmIo motor_shm_io_;

    std::string active_config_section_ = "sim2real";
    Sim2realCfg sim2real_cfg_;
    int active_mode_id_ = 0;
    std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry_;
    std::unordered_map<int, std::string> mode_to_config_section_;
    std::vector<DeployModeProfileSpec> mode_profile_specs_;
    std::vector<std::string> runtime_joint_names_;
    std::unordered_map<std::string, size_t> runtime_joint_index_;
    std::vector<std::string> installed_joint_names_;
    std::vector<int> installed_joint_global_indices_;
    int last_mode_reload_failure_id_ = std::numeric_limits<int>::min();

    std::array<rl_master::filters::MovingAverageFilter, kMotorCountMax> velocity_filters_;
    rl_master::runtime::IntegratedControllerRuntime controller_runtime_;

    rl_master::logging::StructuredLogger data_logger_;
    bool data_logging_enabled_ = false;
    uint64_t data_log_frame_index_ = 0;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_ROBOT_SOLVER_H
