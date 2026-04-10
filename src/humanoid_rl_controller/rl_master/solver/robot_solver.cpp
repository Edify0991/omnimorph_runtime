#include "rl_master/solver/robot_solver.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

#include <time.h>
#include <unistd.h>

#include "rl_master/command_runtime_mode.h"
#include "rl_master/rl_protocol.h"

namespace rl_master::solver
{
namespace
{
constexpr long kControlPeriodNs = 2'000'000; // 500 Hz
constexpr int kCtrlWordStartModeBase = 1000;
constexpr int kCtrlWordSetModeBase = 2000;
constexpr int kCtrlWordModeRange = 1000;

const std::vector<float> kHomePositions = {
    0.0f,
    0.0f,
    -0.24f,
    0.48f,
    -0.24f,
    0.0f,
    0.0f,
    0.0f,
    -0.24f,
    0.48f,
    -0.24f,
    0.0f,
};

std::vector<double> toDoubleVector(const std::vector<float> &values)
{
    std::vector<double> out(values.size(), 0.0);
    for (size_t i = 0; i < values.size(); ++i)
    {
        out[i] = static_cast<double>(values[i]);
    }
    return out;
}

int decodeLocomotionModeFromControlWord(int control_word, int fallback_mode)
{
    if (control_word >= kCtrlWordStartModeBase &&
        control_word < (kCtrlWordStartModeBase + kCtrlWordModeRange))
    {
        return control_word - kCtrlWordStartModeBase;
    }
    if (control_word >= kCtrlWordSetModeBase &&
        control_word < (kCtrlWordSetModeBase + kCtrlWordModeRange))
    {
        return control_word - kCtrlWordSetModeBase;
    }
    return fallback_mode;
}

} // namespace

std::unique_ptr<RobotSolver> RobotSolver::create(int startup_mode_id)
{
    auto solver = std::unique_ptr<RobotSolver>(new RobotSolver());
    solver->initModeProfileMap();
    if (!solver->switchToModeConfig(startup_mode_id, true))
    {
        std::cerr << "Failed to load solver config section for mode_id=" << startup_mode_id << std::endl;
        return nullptr;
    }
    std::cout << "Solver config loaded successfully: mode_id=" << solver->active_mode_id_
              << ", section=" << solver->active_config_section_ << std::endl;
    return solver;
}

void RobotSolver::initModeProfileMap()
{
    mode_profile_specs_.clear();
    try
    {
        mode_profile_specs_ = loadDeployModeProfilesFromYAML(RL_CFG_PATH);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RL_solver] failed to parse deploy_mode_profiles: " << e.what()
                  << ". fallback to default section resolution." << std::endl;
    }
    mode_to_config_section_.clear();
    for (const auto &spec : mode_profile_specs_)
    {
        if (spec.config_section.empty())
        {
            continue;
        }
        mode_to_config_section_[spec.mode_id] = spec.config_section;
    }
}

bool RobotSolver::switchToModeConfig(int mode_id, bool allow_fallback_to_default)
{
    std::string section;
    int resolved_mode_id = mode_id;
    auto it = mode_to_config_section_.find(mode_id);
    if (it != mode_to_config_section_.end())
    {
        section = it->second;
    }
    else if (allow_fallback_to_default)
    {
        section = resolveDeployConfigSectionForModeFromYAML(RL_CFG_PATH, mode_id, "sim2real");
        if (!mode_profile_specs_.empty())
        {
            resolved_mode_id = mode_profile_specs_.front().mode_id;
        }
    }
    else
    {
        return false;
    }

    if (section.empty())
    {
        return false;
    }

    Sim2realCfg loaded;
    if (!loaded.loadFromYAML(RL_CFG_PATH, section))
    {
        return false;
    }

    sim2real_cfg_ = loaded;
    active_config_section_ = section;
    active_mode_id_ = resolved_mode_id;
    applyControlGainsFromCfg();

    std::cout << "[RL_solver] mode config active: mode_id=" << active_mode_id_
              << ", section=" << active_config_section_
              << ", policy=" << sim2real_cfg_.policy_name << std::endl;
    if (data_logging_enabled_ && data_logger_.isOpen())
    {
        data_logger_.writeEvent(
            rl_master::monotonicTimeSec(),
            "solver_mode_config_switched",
            {{"mode_id", std::to_string(active_mode_id_)},
             {"config_section", active_config_section_},
             {"policy_name", sim2real_cfg_.policy_name}});
    }
    return true;
}

bool RobotSolver::initialize()
{
    initializeBuffers();

    try
    {
        motor_shm_io_.connect();
        dds_bridge_.connect();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RL_solver] initialization exception: " << e.what() << std::endl;
        return false;
    }

    initMotorTypes();
    for (auto &filter : velocity_filters_)
    {
        filter = rl_master::filters::MovingAverageFilter(5);
    }

    run_flag_.store(true);
    return true;
}

void RobotSolver::requestStop()
{
    run_flag_.store(false);
}

void RobotSolver::initializeBuffers()
{
    joint_state_ = std::vector<JointData>(kMotorCountMax, {0, 0, 0, RUN_MODE_CSP, 0, 0});
    joint_cmd_ = std::vector<JointData>(kMotorCountMax, {0, 0, 0, RUN_MODE_CSP, 0, 0});
    motor_state_ = std::vector<JointData>(kMotorCountMax, {0, 0, 0, RUN_MODE_CSP, 0, 0});
    motor_cmd_ = std::vector<JointData>(kMotorCountMax, {0, 0, 0, RUN_MODE_CSP, 0, 0});

    open_rl_ = 0;
    last_open_rl_ = 0;

    joint_cmd_q_ = std::vector<float>(kMotorCountMax, 0.0f);
    joint_cmd_dq_ = std::vector<float>(kMotorCountMax, 0.0f);
    joint_cmd_tau_ = std::vector<float>(kMotorCountMax, 0.0f);
    joint_state_q_ = std::vector<float>(kMotorCountMax, 0.0f);
    joint_state_dq_ = std::vector<float>(kMotorCountMax, 0.0f);
    joint_state_tau_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_cmd_q_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_cmd_dq_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_cmd_tau_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_state_q_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_state_dq_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_state_tau_ = std::vector<float>(kMotorCountMax, 0.0f);
    motor_cmd_mode_ = std::vector<float>(kMotorCountMax, 0.0f);
    hold_target_q_ = std::vector<float>(kInstalledMotorCount, 0.0f);
    hold_target_latched_ = false;

    applyControlGainsFromCfg();
}

void RobotSolver::applyControlGainsFromCfg()
{
    if (joint_cmd_.size() < kInstalledMotorCount ||
        joint_state_.size() < kInstalledMotorCount)
    {
        return;
    }

    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        const float kp = (i < sim2real_cfg_.kps.size()) ? sim2real_cfg_.kps[i] : 0.0f;
        const float kd = (i < sim2real_cfg_.kds.size()) ? sim2real_cfg_.kds[i] : 0.0f;
        joint_cmd_[i].kp = kp;
        joint_cmd_[i].kd = kd;
        joint_state_[i].kp = kp;
        joint_state_[i].kd = kd;
    }
}

void RobotSolver::initMotorTypes()
{
    motor_types_.fill(0);
    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        if (i == 3 || i == 9)
        {
            motor_types_[i] = 1; // linear motor
        }
        else
        {
            motor_types_[i] = 0; // rotary motor
        }
    }
}

std::vector<float> RobotSolver::computePdControl(
    const std::vector<float> &target_q,
    const std::vector<float> &target_dq) const
{
    std::vector<float> torque(kInstalledMotorCount, 0.0f);
    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        torque[i] = (target_q[i] - joint_state_[i].q) * static_cast<float>(joint_cmd_[i].kp) +
                    (target_dq[i] - joint_state_[i].dq) * static_cast<float>(joint_cmd_[i].kd);
    }
    return torque;
}

void RobotSolver::getMotorState()
{
    motor_shm_io_.readFeedback(&motor_feedback_all_);

    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        motor_state_[i].q = motor_feedback_all_[i].io.feedback.feedback_pos;
        motor_state_[i].dq = motor_feedback_all_[i].io.feedback.feedback_speed;
        motor_state_[i].tau = motor_feedback_all_[i].io.feedback.feedback_torque;

        motor_state_q_[i] = motor_state_[i].q;
        motor_state_dq_[i] = motor_state_[i].dq;
        motor_state_tau_[i] = motor_state_[i].tau;

        if (i == 2 || i == 8)
        {
            constexpr float kSpeedLimit = 2.7f;
            const float current_speed = std::fabs(motor_state_[i].dq);
            if (current_speed > kSpeedLimit)
            {
                std::cerr << "[RL_solver][WARN] motor #" << i << " speed exceeds limit. speed="
                          << motor_state_[i].dq << " rad/s, limit=" << kSpeedLimit << std::endl;
            }
        }

        if (i == 3 || i == 9)
        {
            constexpr float kQMinMm = -0.1f;
            constexpr float kQMaxMm = 60.0f;
            const float q_mm = motor_state_q_[i];
            if (q_mm < kQMinMm || q_mm > kQMaxMm)
            {
                std::cerr << "[RL_solver][WARN] knee motor #" << i << " out of range. q="
                          << q_mm << " mm, allowed=[" << kQMinMm << ", " << kQMaxMm << "]" << std::endl;
            }
        }
    }

    joint_state_ = kin_conv_.legMotorToJoint(motor_state_);
    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        joint_state_q_[i] = joint_state_[i].q;
        joint_state_dq_[i] = joint_state_[i].dq;
        joint_state_tau_[i] = joint_state_[i].tau;
    }
}

void RobotSolver::sendMotorCmd()
{
    motor_target_all_.fill(MotorHandle{});

    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        joint_cmd_q_[i] = joint_cmd_[i].q;
        joint_cmd_dq_[i] = joint_cmd_[i].dq;
        joint_cmd_tau_[i] = joint_cmd_[i].tau;
    }

    motor_cmd_ = kin_conv_.legJointToMotor(joint_state_, joint_cmd_);

    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        motor_target_all_[i].motor_type = motor_types_[i];
        motor_target_all_[i].io.target.target_speed = motor_cmd_[i].dq;
        motor_target_all_[i].io.target.target_pos = motor_cmd_[i].q;
        motor_target_all_[i].io.target.target_torque = motor_cmd_[i].tau;
        motor_target_all_[i].run_mode = static_cast<uint8_t>(joint_cmd_[i].mode);
        motor_target_all_[i].pd[0] = static_cast<uint8_t>(joint_cmd_[i].kp);
        motor_target_all_[i].pd[1] = static_cast<uint8_t>(joint_cmd_[i].kd);

        motor_cmd_q_[i] = motor_cmd_[i].q;
        motor_cmd_dq_[i] = motor_cmd_[i].dq;
        motor_cmd_tau_[i] = motor_cmd_[i].tau;
        motor_cmd_mode_[i] = static_cast<float>(joint_cmd_[i].mode);
    }

    motor_shm_io_.writeTarget(motor_target_all_);
}

void RobotSolver::getRLCmd()
{
    rl_master::RobotCommandData dds_cmd{};
    uint32_t cmd_seq = 0;
    double cmd_stamp_s = 0.0;
    if (!dds_bridge_.readLatestPolicyCommand(&dds_cmd, &cmd_seq, &cmd_stamp_s))
    {
        latest_cmd_fresh_ = false;
        last_open_rl_ = open_rl_;
        open_rl_ = static_cast<int>(rl_master::kOpenRlDisabled);
        return;
    }

    last_open_rl_ = open_rl_;
    open_rl_ = static_cast<int>(std::lround(dds_cmd.open_rl));

    const double now_s = rl_master::monotonicTimeSec();
    latest_cmd_fresh_ = true;
    if (sim2real_cfg_.enable_cmd_watchdog && cmd_stamp_s > 1e-6)
    {
        if (cmd_seq == last_cmd_seq_)
        {
            latest_cmd_fresh_ = false;
        }
        if ((now_s - cmd_stamp_s) > sim2real_cfg_.cmd_timeout_s)
        {
            latest_cmd_fresh_ = false;
        }
        if (!latest_cmd_fresh_ && (now_s - last_stale_warn_time_s_) > 1.0)
        {
            std::cerr << "[RL_solver] stale RL command detected. Switching to hold mode." << std::endl;
            last_stale_warn_time_s_ = now_s;
        }
        if (latest_cmd_fresh_)
        {
            last_cmd_seq_ = cmd_seq;
        }
    }

    if (!latest_cmd_fresh_)
    {
        return;
    }

    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(latest_cmd_fresh_, dds_cmd.open_rl);

    auto tauLimitAt = [this](size_t i) -> float {
        if (i < sim2real_cfg_.tau_limit.size())
        {
            return std::max(0.0f, std::abs(sim2real_cfg_.tau_limit[i]));
        }
        return 300.0f;
    };

    if (runtime_mode.unknown_open_rl_mode &&
        (now_s - last_stale_warn_time_s_) > 1.0)
    {
        std::cerr << "[RL_solver] unknown open_rl mode=" << dds_cmd.open_rl
                  << ", fallback to hold mode." << std::endl;
        last_stale_warn_time_s_ = now_s;
    }

    if (runtime_mode.mode == rl_master::CommandRuntimeMode::kPolicy)
    {
        std::vector<float> target_q(kInstalledMotorCount, 0.0f);
        std::vector<float> target_dq(kInstalledMotorCount, 0.0f);
        for (size_t i = 0; i < kInstalledMotorCount; ++i)
        {
            joint_cmd_[i].q = dds_cmd.joint_target_q[i];
            joint_cmd_[i].dq = dds_cmd.joint_target_dq[i];
            joint_cmd_[i].tau = dds_cmd.joint_target_tau[i];

            target_q[i] = joint_cmd_[i].q;
            target_dq[i] = joint_cmd_[i].dq;
            // joint_cmd_[i].mode = (i == 0 || i == 1 || i == 2 || i == 6 || i == 7 || i == 8) ? RUN_MODE_R1 : RUN_MODE_CST;
            joint_cmd_[i].mode = RUN_MODE_CST;
        }

        const std::vector<float> joint_tau = computePdControl(target_q, target_dq);
        constexpr float kPdScale = 1.0f;
        for (size_t i = 0; i < kInstalledMotorCount; ++i)
        {
            if (joint_cmd_[i].mode == RUN_MODE_CST)
            {
                joint_cmd_[i].tau = joint_tau[i] * kPdScale;
            }
        }
    }
    else if (runtime_mode.mode == rl_master::CommandRuntimeMode::kCommandStream ||
             runtime_mode.mode == rl_master::CommandRuntimeMode::kTestCsp)
    {
        // Position stream: keep joints in CSP and track commanded positions.
        for (size_t i = 0; i < kInstalledMotorCount; ++i)
        {
            joint_cmd_[i].q = dds_cmd.joint_target_q[i];
            joint_cmd_[i].dq = 0.0f;
            joint_cmd_[i].tau = 0.0f;
            joint_cmd_[i].mode = RUN_MODE_CSP;
        }
    }
    else if (runtime_mode.mode == rl_master::CommandRuntimeMode::kTestCst)
    {
        // Torque stream: use commanded joint torques directly in CST mode.
        for (size_t i = 0; i < kInstalledMotorCount; ++i)
        {
            const float tau_limit = tauLimitAt(i);
            joint_cmd_[i].q = joint_state_[i].q;
            joint_cmd_[i].dq = 0.0f;
            joint_cmd_[i].tau = std::clamp(dds_cmd.joint_target_tau[i], -tau_limit, tau_limit);
            joint_cmd_[i].mode = RUN_MODE_CST;
        }
    }
    else if (runtime_mode.mode == rl_master::CommandRuntimeMode::kTestR1)
    {
        // Mixed stream: forward target q/dq/tau with R1 run mode.
        for (size_t i = 0; i < kInstalledMotorCount; ++i)
        {
            const float tau_limit = tauLimitAt(i);
            joint_cmd_[i].q = dds_cmd.joint_target_q[i];
            joint_cmd_[i].dq = dds_cmd.joint_target_dq[i];
            joint_cmd_[i].tau = std::clamp(dds_cmd.joint_target_tau[i], -tau_limit, tau_limit);
            joint_cmd_[i].mode = RUN_MODE_R1;
        }
    }

    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        joint_cmd_q_[i] = joint_cmd_[i].q;
        joint_cmd_dq_[i] = joint_cmd_[i].dq;
        joint_cmd_tau_[i] = joint_cmd_[i].tau;
    }
}

void RobotSolver::sendRLState()
{
    dds_bridge_.publishRobotState(joint_state_);
}

std::map<std::string, std::vector<float>> RobotSolver::getRobotStateBag() const
{
    const auto now = std::chrono::high_resolution_clock::now();
    const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_).count();
    const float time_ms = static_cast<float>(elapsed_us / 1000.0);

    return {
        {"timestamp_ms", {time_ms}},
        {"joint_cmd_q", joint_cmd_q_},
        {"joint_cmd_dq", joint_cmd_dq_},
        {"joint_cmd_tau", joint_cmd_tau_},
        {"joint_state_q", joint_state_q_},
        {"joint_state_dq", joint_state_dq_},
        {"joint_state_tau", joint_state_tau_},
        {"motor_cmd_q", motor_cmd_q_},
        {"motor_cmd_dq", motor_cmd_dq_},
        {"motor_cmd_tau", motor_cmd_tau_},
        {"motor_state_q", motor_state_q_},
        {"motor_state_dq", motor_state_dq_},
        {"motor_state_tau", motor_state_tau_},
        {"motor_cmd_mode", motor_cmd_mode_},
    };
}

rl_master::logging::LoggerMetadata RobotSolver::buildLoggerMetadata() const
{
    rl_master::logging::LoggerMetadata metadata;
    metadata.string_fields["module"] = "RL_solver";
    metadata.string_fields["policy_name"] = sim2real_cfg_.policy_name;
    metadata.string_fields["policy_family"] = sim2real_cfg_.policy_family;
    metadata.string_fields["policy_path"] = sim2real_cfg_.policy_path;
    metadata.string_fields["observation_manifest_path"] = sim2real_cfg_.observation_manifest_path;
    metadata.string_fields["control_mode"] = sim2real_cfg_.control_mode;

    metadata.numeric_fields["obs_dim"] = static_cast<double>(sim2real_cfg_.obs_dim);
    metadata.numeric_fields["action_dim"] = static_cast<double>(sim2real_cfg_.action_dim);
    metadata.numeric_fields["obs_stack"] = static_cast<double>(sim2real_cfg_.obs_stack_N);
    metadata.numeric_fields["control_hz"] = static_cast<double>(sim2real_cfg_.RL_control_f);
    metadata.numeric_fields["command_timeout_s"] = sim2real_cfg_.cmd_timeout_s;

    metadata.vector_fields["kps"] = toDoubleVector(sim2real_cfg_.kps);
    metadata.vector_fields["kds"] = toDoubleVector(sim2real_cfg_.kds);
    metadata.vector_fields["tau_limit"] = toDoubleVector(sim2real_cfg_.tau_limit);

    metadata.string_list_fields["action_joint_order"] = sim2real_cfg_.action_joint_order;
    metadata.string_list_fields["obs_joint_order"] = sim2real_cfg_.obs_joint_order;
    for (const auto &sub : sim2real_cfg_.sub_models)
    {
        metadata.string_list_fields["sub_model_names"].push_back(sub.name);
        metadata.string_list_fields["sub_model_paths"].push_back(sub.policy_path);
    }

    return metadata;
}

void RobotSolver::initDataLogger()
{
    data_logging_enabled_ = false;
    if (!sim2real_cfg_.save_data_flag)
    {
        return;
    }

    if (!data_logger_.open(sim2real_cfg_.data_path, "solver", buildLoggerMetadata()))
    {
        std::cerr << "[RL_solver] failed to open structured data logger." << std::endl;
        return;
    }

    data_logger_.writeEvent(
        rl_master::monotonicTimeSec(),
        "solver_initialized",
        {{"session_base_path", sim2real_cfg_.data_path}});
    data_logging_enabled_ = true;
    std::cout << "RL Solver structured log: " << data_logger_.recordsPath() << std::endl;
}

void RobotSolver::logLoopData()
{
    if (!data_logging_enabled_ || !data_logger_.isOpen())
    {
        return;
    }

    std::map<std::string, double> scalars;
    scalars["frame_index"] = static_cast<double>(data_log_frame_index_++);
    scalars["open_rl"] = static_cast<double>(open_rl_);
    scalars["last_open_rl"] = static_cast<double>(last_open_rl_);
    scalars["latest_cmd_fresh"] = latest_cmd_fresh_ ? 1.0 : 0.0;
    scalars["loop_overrun_count"] = static_cast<double>(loop_overrun_count_);

    data_logger_.writeRecord(
        rl_master::monotonicTimeSec(),
        "solver_loop",
        scalars,
        getRobotStateBag());
}

void RobotSolver::moveToPosition(const std::vector<float> &target_positions)
{
    getMotorState();

    std::vector<float> current_positions(kInstalledMotorCount, 0.0f);
    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        current_positions[i] = joint_state_[i].q;
        std::cout << "Joint " << i << " current position: " << current_positions[i] << std::endl;
    }

    const double total_time = 5.0;
    const double target_period = 1.0 / 1000.0;
    const int total_steps = static_cast<int>(total_time / target_period);

    std::cout << "Starting linear interpolation to home position..." << std::endl;
    std::cout << "Total time: " << total_time << "s, Steps: " << total_steps << std::endl;

    std::vector<float> step_increments(kInstalledMotorCount, 0.0f);
    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        step_increments[i] = (target_positions[i] - current_positions[i]) / static_cast<float>(total_steps);
    }

    for (int step = 0; step <= total_steps; ++step)
    {
        const auto frame_start = std::chrono::high_resolution_clock::now();

        for (size_t i = 0; i < kInstalledMotorCount; ++i)
        {
            const float interpolated_pos = current_positions[i] + static_cast<float>(step) * step_increments[i];
            joint_cmd_[i].q = interpolated_pos;
            joint_cmd_[i].dq = 0.0;
            joint_cmd_[i].tau = 0.0;
            joint_cmd_[i].mode = RUN_MODE_CSP;
        }

        sendMotorCmd();
        getMotorState();

        const auto frame_end = std::chrono::high_resolution_clock::now();
        const double execution_time = std::chrono::duration<double>(frame_end - frame_start).count();
        const double sleep_time = target_period - execution_time;
        if (sleep_time > 0.0)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(sleep_time));
        }
    }

    std::cout << "Linear interpolation to home position completed." << std::endl;
}

void RobotSolver::holdCurrentPose()
{
    getMotorState();
    for (size_t i = 0; i < kInstalledMotorCount; ++i)
    {
        joint_cmd_[i].q = joint_state_[i].q;
        joint_cmd_[i].dq = 0.0f;
        joint_cmd_[i].tau = 0.0f;
        joint_cmd_[i].mode = RUN_MODE_CSP;
    }
    sendMotorCmd();
}

void RobotSolver::run()
{
    initDataLogger();

    getMotorState();
    sendRLState();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::cout << "Move to Home Position!" << std::endl;
    moveToPosition(kHomePositions);
    std::cout << "Move to Home Position Done!" << std::endl;
    sleep(2);
    std::cout << "Start RL Solver Loop!" << std::endl;

    getMotorState();

    try
    {
        struct timespec next;
        clock_gettime(CLOCK_MONOTONIC, &next);
        auto last_overrun_warn_time = std::chrono::steady_clock::now();

        while (run_flag_.load())
        {
            const auto loop_begin = std::chrono::steady_clock::now();
            dds_bridge_.spinOnce();

            int walk_mode_control_word = 0;
            if (dds_bridge_.readLatestWalkModeControlWord(&walk_mode_control_word))
            {
                const int requested_mode_id = decodeLocomotionModeFromControlWord(
                    walk_mode_control_word,
                    active_mode_id_);
                if (requested_mode_id != active_mode_id_)
                {
                    if (!switchToModeConfig(requested_mode_id, false))
                    {
                        if (last_mode_reload_failure_id_ != requested_mode_id)
                        {
                            std::cerr << "[RL_solver] failed to switch mode config for mode_id="
                                      << requested_mode_id
                                      << ", keep current mode_id=" << active_mode_id_
                                      << ", section=" << active_config_section_ << std::endl;
                            last_mode_reload_failure_id_ = requested_mode_id;
                        }
                    }
                    else
                    {
                        last_mode_reload_failure_id_ = std::numeric_limits<int>::min();
                        hold_target_latched_ = false;
                    }
                }
            }

            getRLCmd();
            const auto open_rl_mode = rl_master::resolveCommandRuntimeMode(true, static_cast<float>(open_rl_));
            const bool any_active_mode = open_rl_mode.open_rl_active;
            if (any_active_mode && !latest_cmd_fresh_)
            {
                open_rl_ = static_cast<int>(rl_master::kOpenRlDisabled);
            }

            const auto last_mode = rl_master::resolveCommandRuntimeMode(true, static_cast<float>(last_open_rl_));
            const auto current_mode = rl_master::resolveCommandRuntimeMode(true, static_cast<float>(open_rl_));
            const bool last_policy_mode_active = (last_mode.mode == rl_master::CommandRuntimeMode::kPolicy);
            const bool current_policy_mode_active = (current_mode.mode == rl_master::CommandRuntimeMode::kPolicy);
            const bool current_any_active_mode = current_mode.open_rl_active;

            if (current_policy_mode_active && !last_policy_mode_active)
            {
                start_time_ = std::chrono::high_resolution_clock::now();
            }

            if (current_any_active_mode)
            {
                hold_target_latched_ = false;
                getMotorState();
                sendRLState();
                sendMotorCmd();
                logLoopData();
            }
            else
            {
                getMotorState();
                if (!hold_target_latched_)
                {
                    std::cout << "[RL_solver] hold mode active" << std::endl;
                    for (size_t i = 0; i < kInstalledMotorCount; ++i)
                    {
                        hold_target_q_[i] = joint_state_[i].q;
                    }
                    hold_target_latched_ = true;
                }

                for (size_t i = 0; i < kInstalledMotorCount; ++i)
                {
                    joint_cmd_[i].q = hold_target_q_[i];
                    joint_cmd_[i].dq = 0.0;
                    joint_cmd_[i].tau = 0.0;
                    joint_cmd_[i].mode = RUN_MODE_CSP;
                }
                sendMotorCmd();
                sendRLState();
            }

            const auto loop_end = std::chrono::steady_clock::now();
            const auto loop_elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(loop_end - loop_begin).count();
            if (loop_elapsed_us > sim2real_cfg_.loop_overrun_warn_us)
            {
                ++loop_overrun_count_;
                if ((loop_end - last_overrun_warn_time) > std::chrono::seconds(1))
                {
                    std::cerr << "[RL_solver] loop overrun: " << loop_elapsed_us
                              << " us, total overruns: " << loop_overrun_count_ << std::endl;
                    last_overrun_warn_time = loop_end;
                }
            }

            next.tv_nsec += kControlPeriodNs;
            if (next.tv_nsec >= 1'000'000'000)
            {
                next.tv_nsec -= 1'000'000'000;
                next.tv_sec++;
            }

            const int ret = clock_nanosleep(
                CLOCK_MONOTONIC,
                TIMER_ABSTIME,
                &next,
                nullptr);
            if (ret != 0 && ret != EINTR)
            {
                perror("clock_nanosleep");
            }
        }
    }
    catch (const std::exception &)
    {
        std::cerr << "program is stopped, perform motor return to zero operation..." << std::endl;
        holdCurrentPose();
    }

    holdCurrentPose();
    data_logger_.flush();
}

} // namespace rl_master::solver
