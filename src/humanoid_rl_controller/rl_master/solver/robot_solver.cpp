#include "rl_master/solver/robot_solver.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <time.h>
#include <unistd.h>

#include "rl_master/RL_controller.h"
#include "rl_master/command_runtime_mode.h"
#include "rl_master/deploy_state_machine.h"
#include "rl_master/rl_protocol.h"

namespace rl_master::solver
{
namespace
{
constexpr long kNanosecondsPerSecond = 1'000'000'000L;

long resolveControlPeriodNs(const Sim2realCfg &cfg)
{
    const long solver_control_hz = std::max(1L, static_cast<long>(cfg.solver_control_hz));
    return std::max(1L, kNanosecondsPerSecond / solver_control_hz);
}

std::vector<JointData> extractJointGroup(
    const std::vector<JointData> &full,
    const std::vector<int> &indices)
{
    std::vector<JointData> out;
    out.reserve(indices.size());
    for (const int index : indices)
    {
        if (index < 0 || static_cast<size_t>(index) >= full.size())
        {
            throw std::runtime_error("joint group index out of range during extraction");
        }
        out.push_back(full[static_cast<size_t>(index)]);
    }
    return out;
}

void scatterJointGroup(
    const std::vector<JointData> &group,
    const std::vector<int> &indices,
    std::vector<JointData> *full)
{
    if (!full)
    {
        return;
    }
    if (group.size() != indices.size())
    {
        throw std::runtime_error("joint group scatter size mismatch");
    }
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const int index = indices[i];
        if (index < 0 || static_cast<size_t>(index) >= full->size())
        {
            throw std::runtime_error("joint group index out of range during scatter");
        }
        (*full)[static_cast<size_t>(index)] = group[i];
    }
}

bool jointNameInSet(
    const std::string &joint_name,
    const std::initializer_list<const char *> &names)
{
    for (const char *candidate : names)
    {
        if (joint_name == candidate)
        {
            return true;
        }
    }
    return false;
}

} // namespace

RobotSolver::~RobotSolver() = default;

size_t RobotSolver::installedJointCount() const
{
    return installed_joint_names_.size();
}

bool RobotSolver::jointBuffersInitialized() const
{
    const size_t installed_count = installedJointCount();
    return installed_count > 0 &&
           joint_cmd_.size() == installed_count &&
           joint_state_.size() == installed_count;
}

std::unique_ptr<RobotSolver> RobotSolver::create(
    int startup_mode_id,
    std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry)
{
    auto solver = std::unique_ptr<RobotSolver>(new RobotSolver());
    solver->mode_registry_ = std::move(mode_registry);
    if (!solver->mode_registry_)
    {
        solver->mode_registry_ = rl_master::ModeProfileRegistry::loadFromYaml(RL_CFG_PATH, "engineai_walk");
    }
    solver->initModeProfileMap();
    solver->initializeJointLayout();
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
    mode_to_config_section_.clear();
    if (!mode_registry_)
    {
        return;
    }

    try
    {
        mode_profile_specs_ = mode_registry_->specs();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RL_solver] failed to read mode registry specs: " << e.what() << std::endl;
    }
    for (const auto &spec : mode_profile_specs_)
    {
        if (spec.config_section.empty())
        {
            continue;
        }
        mode_to_config_section_[spec.mode_id] = spec.config_section;
    }
}

void RobotSolver::initializeJointLayout()
{
    installed_joint_names_.clear();
    installed_joint_index_.clear();
    if (mode_registry_)
    {
        installed_joint_names_ = mode_registry_->jointOrder();
    }
    if (installed_joint_names_.empty())
    {
        throw std::runtime_error("robot_global_joint_order must not be empty for RobotSolver");
    }
    if (installed_joint_names_.size() > kMotorShmSlotCount)
    {
        throw std::runtime_error(
            "robot_global_joint_order size exceeds SHM slot count: " +
            std::to_string(installed_joint_names_.size()) + " > " +
            std::to_string(kMotorShmSlotCount));
    }

    installed_zero_joint_q_.assign(installed_joint_names_.size(), 0.0f);
    installed_joint_tau_limits_.assign(installed_joint_names_.size(), 0.0f);
    installed_motor_torque_limits_.assign(installed_joint_names_.size(), 0.0f);
    installed_joint_configured_run_modes_.assign(installed_joint_names_.size(), RUN_MODE_CSP);

    installed_joint_index_.reserve(installed_joint_names_.size());
    for (size_t i = 0; i < installed_joint_names_.size(); ++i)
    {
        installed_joint_index_[installed_joint_names_[i]] = i;
    }

    if (mode_registry_)
    {
        kin_conv_.configureJointGroups(installed_joint_names_, mode_registry_->jointGroups());
    }
}

bool RobotSolver::switchToModeConfig(int mode_id, bool allow_fallback_to_default)
{
    if (!mode_registry_)
    {
        return false;
    }

    int resolved_mode_id = mode_id;
    std::string section;
    Sim2realCfg loaded;
    try
    {
        const auto &spec = mode_registry_->specForMode(mode_id, allow_fallback_to_default);
        resolved_mode_id = spec.mode_id;
        section = spec.config_section;
        loaded = mode_registry_->cfgForMode(mode_id, allow_fallback_to_default);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RL_solver] failed to resolve mode config for mode_id=" << mode_id
                  << ": " << e.what() << std::endl;
        return false;
    }

    sim2real_cfg_ = loaded;
    active_config_section_ = section;
    active_mode_id_ = resolved_mode_id;
    active_joint_layout_ = mode_registry_ ? &mode_registry_->layoutForSection(section) : nullptr;
    cacheInstalledZeroPoseFromCfg();
    cacheInstalledJointRunModesFromCfg();
    cacheInstalledJointTauLimitsFromCfg();
    cacheInstalledMotorTorqueLimitsFromCfg();
    if (jointBuffersInitialized())
    {
        applyControlGainsFromCfg();
    }

    std::cout << "[RL_solver] mode config active: mode_id=" << active_mode_id_
              << ", section=" << active_config_section_
              << ", policy=" << sim2real_cfg_.policy_name << std::endl;
    if (runtime_logging_enabled_ && runtime_recorder_.isOpen())
    {
        rl_master::logging::RuntimeEventRecord event;
        event.monotonic_time_sec = rl_master::monotonicTimeSec();
        event.event_type = "solver_mode_config_switched";
        event.tags["mode_id"] = std::to_string(active_mode_id_);
        event.tags["config_section"] = active_config_section_;
        event.tags["policy_name"] = sim2real_cfg_.policy_name;
        runtime_recorder_.recordEvent(event);
    }
    return true;
}

bool RobotSolver::initialize()
{
    initializeBuffers();

    try
    {
        motor_shm_io_.connect();
        dds_bridge_.connect({
            sim2real_cfg_.enable_state_telemetry,
            sim2real_cfg_.state_telemetry_hz,
        });
        dds_bridge_.updateSourceContract(sim2real_cfg_.source_contract);
        dds_bridge_.setImuSampleCallback(
            [this](
                const std::array<float, 3> &ang_vel,
                const std::array<float, 4> &quat,
                const std::array<float, 3> &rpy,
                double monotonic_time_sec) {
                this->emitBaseImuSourceSample(ang_vel, quat, rpy, monotonic_time_sec);
            });
        initializeController();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RL_solver] initialization exception: " << e.what() << std::endl;
        return false;
    }

    initMotorTypes();

    run_flag_.store(true);
    return true;
}

void RobotSolver::requestStop()
{
    run_flag_.store(false);
}

void RobotSolver::initializeBuffers()
{
    const size_t installed_count = installedJointCount();
    if (installed_count == 0)
    {
        throw std::runtime_error("RobotSolver::initializeBuffers requires installed joints to be initialized first.");
    }

    joint_state_ = std::vector<JointData>(installed_count, {0, 0, 0, RUN_MODE_CSP, 0, 0});
    joint_cmd_ = std::vector<JointData>(installed_count, {0, 0, 0, RUN_MODE_CSP, 0, 0});
    motor_state_ = std::vector<JointData>(installed_count, {0, 0, 0, RUN_MODE_CSP, 0, 0});
    motor_cmd_ = std::vector<JointData>(installed_count, {0, 0, 0, RUN_MODE_CSP, 0, 0});

    open_rl_ = 0;
    last_open_rl_ = 0;

    joint_cmd_q_ = std::vector<float>(installed_count, 0.0f);
    joint_cmd_dq_ = std::vector<float>(installed_count, 0.0f);
    joint_cmd_tau_ = std::vector<float>(installed_count, 0.0f);
    joint_state_q_ = std::vector<float>(installed_count, 0.0f);
    joint_state_dq_ = std::vector<float>(installed_count, 0.0f);
    joint_state_tau_ = std::vector<float>(installed_count, 0.0f);
    motor_cmd_q_ = std::vector<float>(installed_count, 0.0f);
    motor_cmd_dq_ = std::vector<float>(installed_count, 0.0f);
    motor_cmd_tau_ = std::vector<float>(installed_count, 0.0f);
    motor_state_q_ = std::vector<float>(installed_count, 0.0f);
    motor_state_dq_ = std::vector<float>(installed_count, 0.0f);
    motor_state_tau_ = std::vector<float>(installed_count, 0.0f);
    motor_cmd_mode_ = std::vector<float>(installed_count, 0.0f);
    hold_target_q_ = std::vector<float>(installed_count, 0.0f);
    velocity_filters_.assign(installed_count, rl_master::filters::MovingAverageFilter(5));
    hold_target_latched_ = false;
}

void RobotSolver::initializeController()
{
    controller_runtime_.setModeProfileRegistry(mode_registry_);
    controller_runtime_.initialize(active_mode_id_);
    syncRuntimeCfgFromController(true);
}

void RobotSolver::syncRuntimeCfgFromController(bool force)
{
    if (!controller_runtime_.initialized())
    {
        return;
    }

    const int controller_mode_id = controller_runtime_.activeModeId();
    const std::string controller_section = controller_runtime_.activeConfigSection();
    if (!force &&
        controller_mode_id == active_mode_id_ &&
        controller_section == active_config_section_)
    {
        return;
    }

    sim2real_cfg_ = controller_runtime_.runtimeCfg();
    active_mode_id_ = controller_mode_id;
    active_config_section_ = controller_section;
    active_joint_layout_ = mode_registry_ ? &mode_registry_->layoutForSection(active_config_section_) : nullptr;
    cacheInstalledZeroPoseFromCfg();
    cacheInstalledJointRunModesFromCfg();
    cacheInstalledJointTauLimitsFromCfg();
    cacheInstalledMotorTorqueLimitsFromCfg();
    applyControlGainsFromCfg();
    dds_bridge_.updateStateTelemetryConfig({
        sim2real_cfg_.enable_state_telemetry,
        sim2real_cfg_.state_telemetry_hz,
    });
    dds_bridge_.updateSourceContract(sim2real_cfg_.source_contract);

    std::cout << "[RL_solver] controller runtime synced: mode_id=" << active_mode_id_
              << ", section=" << active_config_section_
              << ", policy=" << sim2real_cfg_.policy_name << std::endl;
    if (runtime_logging_enabled_ && runtime_recorder_.isOpen())
    {
        rl_master::logging::RuntimeEventRecord event;
        event.monotonic_time_sec = rl_master::monotonicTimeSec();
        event.event_type = "solver_mode_config_switched";
        event.tags["mode_id"] = std::to_string(active_mode_id_);
        event.tags["config_section"] = active_config_section_;
        event.tags["policy_name"] = sim2real_cfg_.policy_name;
        runtime_recorder_.recordEvent(event);
    }
}

void RobotSolver::applyControlGainsFromCfg()
{
    if (!jointBuffersInitialized())
    {
        return;
    }

    const size_t installed_count = installedJointCount();
    for (size_t i = 0; i < installed_count; ++i)
    {
        joint_cmd_[i].kp = 0.0f;
        joint_cmd_[i].kd = 0.0f;
        joint_state_[i].kp = 0.0f;
        joint_state_[i].kd = 0.0f;
    }

    if (!active_joint_layout_)
    {
        throw std::runtime_error("RobotSolver::applyControlGainsFromCfg requires active joint layout");
    }

    const size_t policy_joint_count = active_joint_layout_->action_global_indices.size();
    for (size_t policy_idx = 0; policy_idx < policy_joint_count; ++policy_idx)
    {
        const int global_index = active_joint_layout_->action_global_indices[policy_idx];
        if (global_index < 0 || static_cast<size_t>(global_index) >= installed_count)
        {
            continue;
        }

        const size_t hardware_idx = static_cast<size_t>(global_index);
        const float kp = sim2real_cfg_.kps[policy_idx];
        const float kd = sim2real_cfg_.kds[policy_idx];
        joint_cmd_[hardware_idx].kp = kp;
        joint_cmd_[hardware_idx].kd = kd;
        joint_state_[hardware_idx].kp = kp;
        joint_state_[hardware_idx].kd = kd;
    }
}

void RobotSolver::cacheInstalledZeroPoseFromCfg()
{
    if (!active_joint_layout_)
    {
        throw std::runtime_error("RobotSolver::cacheInstalledZeroPoseFromCfg requires active joint layout");
    }
    if (active_joint_layout_->zero_pose.size() != installed_joint_names_.size())
    {
        throw std::runtime_error("active zero pose layout size does not match installed joints");
    }
    installed_zero_joint_q_ = active_joint_layout_->zero_pose;
}

void RobotSolver::cacheInstalledJointRunModesFromCfg()
{
    installed_joint_configured_run_modes_.assign(installed_joint_names_.size(), RUN_MODE_CSP);

    auto parseConfiguredRunMode = [](const std::string &joint_name, const std::string &raw_mode) -> MotorRunMode {
        if (raw_mode == "csp")
        {
            return RUN_MODE_CSP;
        }
        if (raw_mode == "cst")
        {
            return RUN_MODE_CST;
        }
        if (raw_mode == "r1")
        {
            return RUN_MODE_R1;
        }
        throw std::runtime_error(
            "installed_joint_run_modes for joint '" + joint_name +
            "' must be one of: csp, cst, r1");
    };

    for (size_t i = 0; i < installed_joint_names_.size(); ++i)
    {
        const std::string &joint_name = installed_joint_names_[i];
        const auto it = sim2real_cfg_.installed_joint_run_modes.find(joint_name);
        if (it == sim2real_cfg_.installed_joint_run_modes.end())
        {
            throw std::runtime_error(
                "installed_joint_run_modes missing installed joint: " + joint_name);
        }
        installed_joint_configured_run_modes_[i] = parseConfiguredRunMode(joint_name, it->second);
    }
}

void RobotSolver::cacheInstalledJointTauLimitsFromCfg()
{
    installed_joint_tau_limits_.assign(installed_joint_names_.size(), 0.0f);
    if (!active_joint_layout_)
    {
        throw std::runtime_error("RobotSolver::cacheInstalledJointTauLimitsFromCfg requires active joint layout");
    }

    for (size_t policy_idx = 0; policy_idx < active_joint_layout_->action_global_indices.size(); ++policy_idx)
    {
        const int global_index = active_joint_layout_->action_global_indices[policy_idx];
        if (global_index < 0 || static_cast<size_t>(global_index) >= installed_joint_tau_limits_.size())
        {
            continue;
        }
        if (policy_idx < sim2real_cfg_.tau_limit.size())
        {
            installed_joint_tau_limits_[static_cast<size_t>(global_index)] =
                std::max(0.0f, std::abs(sim2real_cfg_.tau_limit[policy_idx]));
        }
    }
}

void RobotSolver::cacheInstalledMotorTorqueLimitsFromCfg()
{
    installed_motor_torque_limits_.assign(installed_joint_names_.size(), 0.0f);

    for (size_t i = 0; i < installed_joint_names_.size(); ++i)
    {
        const std::string &joint_name = installed_joint_names_[i];
        const auto it = sim2real_cfg_.robotCfg.motor_torque_limit.find(joint_name);
        if (it == sim2real_cfg_.robotCfg.motor_torque_limit.end())
        {
            continue;
        }
        installed_motor_torque_limits_[i] = std::max(0.0f, std::abs(it->second));
    }
}

void RobotSolver::initMotorTypes()
{
    motor_types_.fill(0);
    const size_t installed_count = installedJointCount();
    for (size_t i = 0; i < installed_count; ++i)
    {
        if (jointNameInSet(installed_joint_names_[i], {"right_knee_pitch", "left_knee_pitch"}))
        {
            motor_types_[i] = 1; // linear motor
        }
        else
        {
            motor_types_[i] = 0; // rotary motor
        }
    }
}

void RobotSolver::getMotorState()
{
    motor_shm_io_.readFeedback(&motor_feedback_all_);

    const size_t installed_count = installedJointCount();
    for (size_t i = 0; i < installed_count; ++i)
    {
        motor_state_[i].q = motor_feedback_all_[i].io.feedback.feedback_pos;
        motor_state_[i].dq = motor_feedback_all_[i].io.feedback.feedback_speed;
        motor_state_[i].tau = motor_feedback_all_[i].io.feedback.feedback_torque;

        motor_state_q_[i] = motor_state_[i].q;
        motor_state_dq_[i] = motor_state_[i].dq;
        motor_state_tau_[i] = motor_state_[i].tau;

        if (jointNameInSet(installed_joint_names_[i], {"right_hip_pitch", "left_hip_pitch"}))
        {
            constexpr float kSpeedLimit = 2.7f;
            const float current_speed = std::fabs(motor_state_[i].dq);
            if (current_speed > kSpeedLimit)
            {
                std::cerr << "[RL_solver][WARN] motor #" << i << " speed exceeds limit. speed="
                          << motor_state_[i].dq << " rad/s, limit=" << kSpeedLimit << std::endl;
            }
        }

        if (jointNameInSet(installed_joint_names_[i], {"right_knee_pitch", "left_knee_pitch"}))
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

    joint_state_ = motor_state_;
    const auto &leg_indices = kin_conv_.legGlobalIndices();
    if (!leg_indices.empty())
    {
        scatterJointGroup(
            kin_conv_.legMotorToJoint(extractJointGroup(motor_state_, leg_indices)),
            leg_indices,
            &joint_state_);
    }
    const auto &arm_indices = kin_conv_.armGlobalIndices();
    if (!arm_indices.empty())
    {
        scatterJointGroup(
            kin_conv_.armMotorToJoint(extractJointGroup(motor_state_, arm_indices)),
            arm_indices,
            &joint_state_);
    }
    const auto &waist_indices = kin_conv_.waistGlobalIndices();
    if (!waist_indices.empty())
    {
        scatterJointGroup(
            kin_conv_.waistMotorToJoint(extractJointGroup(motor_state_, waist_indices)),
            waist_indices,
            &joint_state_);
    }
    for (size_t i = 0; i < installed_count; ++i)
    {
        joint_state_q_[i] = joint_state_[i].q;
        joint_state_dq_[i] = joint_state_[i].dq;
        joint_state_tau_[i] = joint_state_[i].tau;
    }
}

void RobotSolver::sendMotorCmd()
{
    motor_target_all_.fill(MotorHandle{});

    const size_t installed_count = installedJointCount();
    for (size_t i = 0; i < installed_count; ++i)
    {
        joint_cmd_q_[i] = joint_cmd_[i].q;
        joint_cmd_dq_[i] = joint_cmd_[i].dq;
        joint_cmd_tau_[i] = joint_cmd_[i].tau;
    }

    motor_cmd_ = joint_cmd_;
    const auto &leg_indices = kin_conv_.legGlobalIndices();
    if (!leg_indices.empty())
    {
        scatterJointGroup(
            kin_conv_.legJointToMotor(
                extractJointGroup(joint_state_, leg_indices),
                extractJointGroup(joint_cmd_, leg_indices)),
            leg_indices,
            &motor_cmd_);
    }
    const auto &arm_indices = kin_conv_.armGlobalIndices();
    if (!arm_indices.empty())
    {
        scatterJointGroup(
            kin_conv_.armJointToMotor(
                extractJointGroup(joint_state_, arm_indices),
                extractJointGroup(joint_cmd_, arm_indices)),
            arm_indices,
            &motor_cmd_);
    }
    const auto &waist_indices = kin_conv_.waistGlobalIndices();
    if (!waist_indices.empty())
    {
        scatterJointGroup(
            kin_conv_.waistJointToMotor(
                extractJointGroup(joint_state_, waist_indices),
                extractJointGroup(joint_cmd_, waist_indices)),
            waist_indices,
            &motor_cmd_);
    }

    for (size_t i = 0; i < installed_count; ++i)
    {
        if (i >= installed_motor_torque_limits_.size())
        {
            continue;
        }
        const float motor_limit = installed_motor_torque_limits_[i];
        if (motor_limit > 0.0f)
        {
            motor_cmd_[i].tau = std::clamp(motor_cmd_[i].tau, -motor_limit, motor_limit);
        }
    }

    for (size_t i = 0; i < installed_count; ++i)
    {
        motor_target_all_[i].motor_type = motor_types_[i];
        motor_target_all_[i].io.target.target_speed = motor_cmd_[i].dq;
        motor_target_all_[i].io.target.target_pos = motor_cmd_[i].q;
        motor_target_all_[i].io.target.target_torque = motor_cmd_[i].tau;
        motor_target_all_[i].run_mode = static_cast<uint8_t>(joint_cmd_[i].mode);
        if (joint_cmd_[i].mode == RUN_MODE_R1)
        {
            motor_target_all_[i].pd[0] = static_cast<uint8_t>(joint_cmd_[i].kp);
            motor_target_all_[i].pd[1] = static_cast<uint8_t>(joint_cmd_[i].kd);
        }
        else
        {
            motor_target_all_[i].pd[0] = 0;
            motor_target_all_[i].pd[1] = 0;
        }

        motor_cmd_q_[i] = motor_cmd_[i].q;
        motor_cmd_dq_[i] = motor_cmd_[i].dq;
        motor_cmd_tau_[i] = motor_cmd_[i].tau;
        motor_cmd_mode_[i] = static_cast<float>(joint_cmd_[i].mode);
    }

    motor_shm_io_.writeTarget(motor_target_all_);
}

void RobotSolver::applyRuntimeCommand(
    const rl_master::RobotCommandData &command,
    bool command_fresh)
{
    latest_cmd_fresh_ = command_fresh;
    last_open_rl_ = open_rl_;
    open_rl_ = static_cast<int>(std::lround(command.open_rl));

    const auto runtime_mode = rl_master::resolveCommandRuntimeMode(command_fresh, command.open_rl);

    auto tauLimitAt = [this](size_t i) -> float {
        if (i < installed_joint_tau_limits_.size())
        {
            return std::max(0.0f, std::abs(installed_joint_tau_limits_[i]));
        }
        return 0.0f;
    };
    auto commandQAt = [&](size_t hardware_idx) -> float {
        if (hardware_idx >= command.joint_target_q.size())
        {
            return joint_state_[hardware_idx].q;
        }
        return command.joint_target_q[hardware_idx];
    };
    auto commandDqAt = [&](size_t hardware_idx) -> float {
        if (hardware_idx >= command.joint_target_dq.size())
        {
            return 0.0f;
        }
        return command.joint_target_dq[hardware_idx];
    };
    auto commandTauAt = [&](size_t hardware_idx) -> float {
        if (hardware_idx >= command.joint_target_tau.size())
        {
            return 0.0f;
        }
        return command.joint_target_tau[hardware_idx];
    };
    auto zeroJointQAt = [&](size_t hardware_idx) -> float {
        if (hardware_idx >= installed_zero_joint_q_.size())
        {
            return 0.0f;
        }
        return installed_zero_joint_q_[hardware_idx];
    };
    auto isPolicyControlledHardwareJoint = [&](size_t hardware_idx) -> bool {
        if (hardware_idx >= installed_joint_names_.size())
        {
            return false;
        }
        const std::string &joint_name = installed_joint_names_[hardware_idx];
        return std::find(
                   sim2real_cfg_.action_joint_order.begin(),
                   sim2real_cfg_.action_joint_order.end(),
                   joint_name) != sim2real_cfg_.action_joint_order.end();
    };

    const double now_s = rl_master::monotonicTimeSec();
    if (runtime_mode.unknown_open_rl_mode &&
        (now_s - last_stale_warn_time_s_) > 1.0)
    {
        std::cerr << "[RL_solver] unknown open_rl mode=" << command.open_rl
                  << ", fallback to hold mode." << std::endl;
        last_stale_warn_time_s_ = now_s;
    }

    if (runtime_mode.mode == rl_master::CommandRuntimeMode::kPolicy)
    {
        const size_t installed_count = installedJointCount();
        for (size_t i = 0; i < installed_count; ++i)
        {
            const bool policy_controlled = isPolicyControlledHardwareJoint(i);
            const float target_q_i = commandQAt(i);
            const float target_dq_i = commandDqAt(i);
            const float target_tau_i = commandTauAt(i);
            const float hold_q_i = zeroJointQAt(i);

            if (policy_controlled)
            {
                joint_cmd_[i].q = target_q_i;
                joint_cmd_[i].dq = target_dq_i;
                joint_cmd_[i].tau = target_tau_i;
                joint_cmd_[i].mode = installed_joint_configured_run_modes_[i];
            }
            else
            {
                joint_cmd_[i].q = hold_q_i;
                joint_cmd_[i].dq = 0.0f;
                joint_cmd_[i].tau = 0.0f;
                joint_cmd_[i].mode = installed_joint_configured_run_modes_[i];
            }
        }

        for (size_t i = 0; i < installed_count; ++i)
        {
            if (joint_cmd_[i].mode == RUN_MODE_CST)
            {
                const float tau_limit = tauLimitAt(i);
                if (tau_limit > 0.0f)
                {
                    joint_cmd_[i].tau = std::clamp(joint_cmd_[i].tau, -tau_limit, tau_limit);
                }
            }
            else if (joint_cmd_[i].mode == RUN_MODE_R1)
            {
                const float tau_limit = tauLimitAt(i);
                if (tau_limit > 0.0f)
                {
                    joint_cmd_[i].tau = std::clamp(joint_cmd_[i].tau, -tau_limit, tau_limit);
                }
            }
            else if (joint_cmd_[i].mode == RUN_MODE_CSP)
            {
                joint_cmd_[i].tau = 0.0f;
            }
        }
    }
    else if (runtime_mode.mode == rl_master::CommandRuntimeMode::kCommandStream ||
             runtime_mode.mode == rl_master::CommandRuntimeMode::kTestCsp)
    {
        // Position stream: keep joints in CSP and track commanded positions.
        const size_t installed_count = installedJointCount();
        for (size_t i = 0; i < installed_count; ++i)
        {
            joint_cmd_[i].q = commandQAt(i);
            joint_cmd_[i].dq = 0.0f;
            joint_cmd_[i].tau = 0.0f;
            joint_cmd_[i].mode = RUN_MODE_CSP;
        }
    }
    else if (runtime_mode.mode == rl_master::CommandRuntimeMode::kTestCst)
    {
        // Torque stream: use commanded joint torques directly in CST mode.
        const size_t installed_count = installedJointCount();
        for (size_t i = 0; i < installed_count; ++i)
        {
            const float tau_limit = tauLimitAt(i);
            joint_cmd_[i].q = joint_state_[i].q;
            joint_cmd_[i].dq = 0.0f;
            joint_cmd_[i].tau = commandTauAt(i);
            if (tau_limit > 0.0f)
            {
                joint_cmd_[i].tau = std::clamp(joint_cmd_[i].tau, -tau_limit, tau_limit);
            }
            joint_cmd_[i].mode = RUN_MODE_CST;
        }
    }
    else if (runtime_mode.mode == rl_master::CommandRuntimeMode::kTestR1)
    {
        // Mixed stream: forward target q/dq/tau with R1 run mode.
        const size_t installed_count = installedJointCount();
        for (size_t i = 0; i < installed_count; ++i)
        {
            const float tau_limit = tauLimitAt(i);
            joint_cmd_[i].q = commandQAt(i);
            joint_cmd_[i].dq = commandDqAt(i);
            joint_cmd_[i].tau = commandTauAt(i);
            if (tau_limit > 0.0f)
            {
                joint_cmd_[i].tau = std::clamp(joint_cmd_[i].tau, -tau_limit, tau_limit);
            }
            joint_cmd_[i].mode = RUN_MODE_R1;
        }
    }

    const size_t installed_count = installedJointCount();
    for (size_t i = 0; i < installed_count; ++i)
    {
        joint_cmd_q_[i] = joint_cmd_[i].q;
        joint_cmd_dq_[i] = joint_cmd_[i].dq;
        joint_cmd_tau_[i] = joint_cmd_[i].tau;
    }
}

rl_master::RobotStateData RobotSolver::buildControllerStateData() const
{
    rl_master::RobotStateData state;
    state.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    state.active_joint_count = static_cast<int>(installed_joint_names_.size());
    state.joint_q.assign(installed_joint_names_.size(), 0.0f);
    state.joint_dq.assign(installed_joint_names_.size(), 0.0f);
    state.joint_tau.assign(installed_joint_names_.size(), 0.0f);
    for (size_t i = 0; i < installed_joint_names_.size() && i < joint_state_.size(); ++i)
    {
        state.joint_q[i] = joint_state_[i].q;
        state.joint_dq[i] = joint_state_[i].dq;
        state.joint_tau[i] = joint_state_[i].tau;
    }

    return state;
}

void RobotSolver::sendRLState()
{
    rl_master::RobotStateData io_state = buildControllerStateData();
    dds_bridge_.mirrorRobotState(io_state);
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

std::string RobotSolver::buildRuntimeConfigSnapshotJson() const
{
    auto appendQuoted = [](std::ostringstream &oss, const std::string &value) {
        oss << '\"';
        for (char ch : value)
        {
            switch (ch)
            {
            case '\"':
                oss << "\\\\\"";
                break;
            case '\\':
                oss << "\\\\\\\\";
                break;
            case '\n':
                oss << "\\\\n";
                break;
            case '\r':
                oss << "\\\\r";
                break;
            case '\t':
                oss << "\\\\t";
                break;
            default:
                oss << ch;
                break;
            }
        }
        oss << '\"';
    };

    auto appendStringVector = [&appendQuoted](std::ostringstream &oss, const std::vector<std::string> &values) {
        oss << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                oss << ",";
            }
            appendQuoted(oss, values[i]);
        }
        oss << "]";
    };

    auto appendFloatVector = [](std::ostringstream &oss, const std::vector<float> &values) {
        oss << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                oss << ",";
            }
            oss.precision(10);
            oss << static_cast<double>(values[i]);
        }
        oss << "]";
    };
    auto appendStringMap = [&](std::ostringstream &oss, const std::map<std::string, std::string> &values) {
        oss << "{";
        bool first = true;
        for (const auto &entry : values)
        {
            if (!first)
            {
                oss << ",";
            }
            first = false;
            appendQuoted(oss, entry.first);
            oss << ":";
            appendQuoted(oss, entry.second);
        }
        oss << "}";
    };

    std::ostringstream oss;
    oss << "{";
    oss << "\"schema_version\":1,";
    oss << "\"active_mode_id\":" << active_mode_id_ << ",";
    oss << "\"active_config_section\":";
    appendQuoted(oss, active_config_section_);
    oss << ",\"active_policy_name\":";
    appendQuoted(oss, sim2real_cfg_.policy_name);
    oss << ",\"runtime_joint_order\":";
    appendStringVector(oss, installed_joint_names_);
    oss << ",\"installed_joint_names\":";
    appendStringVector(oss, installed_joint_names_);
    oss << ",\"profiles\":[";
    for (size_t i = 0; i < mode_profile_specs_.size(); ++i)
    {
        const auto &spec = mode_profile_specs_[i];
        if (i > 0)
        {
            oss << ",";
        }
        const auto cfg = mode_registry_->cfgForMode(spec.mode_id, true);
        oss << "{";
        oss << "\"mode_id\":" << spec.mode_id << ",";
        oss << "\"tag\":";
        appendQuoted(oss, spec.tag);
        oss << ",\"config_section\":";
        appendQuoted(oss, spec.config_section);
        oss << ",\"policy_name\":";
        appendQuoted(oss, cfg.policy_name);
        oss << ",\"policy_family\":";
        appendQuoted(oss, cfg.policy_family);
        oss << ",\"policy_path\":";
        appendQuoted(oss, cfg.policy_path);
        oss << ",\"observation_manifest_path\":";
        appendQuoted(oss, cfg.observation_manifest_path);
        oss << ",\"obs_dim\":" << cfg.obs_dim << ",";
        oss << "\"action_dim\":" << cfg.action_dim << ",";
        oss << "\"obs_stack_n\":" << cfg.obs_stack_N << ",";
        oss << "\"policy_hz\":" << cfg.RL_control_f << ",";
        oss << "\"solver_control_hz\":" << cfg.solver_control_hz << ",";
        oss << "\"installed_joint_run_modes\":";
        appendStringMap(oss, cfg.installed_joint_run_modes);
        oss << ",";
        oss << "\"action_joint_order\":";
        appendStringVector(oss, cfg.action_joint_order);
        oss << ",\"obs_joint_order\":";
        appendStringVector(oss, cfg.obs_joint_order);
        oss << ",\"tau_limit\":";
        appendFloatVector(oss, cfg.tau_limit);
        oss << "}";
    }
    oss << "]}";
    return oss.str();
}

void RobotSolver::initRuntimeRecorder()
{
    runtime_logging_enabled_ = false;
    if (!sim2real_cfg_.logging.enabled)
    {
        return;
    }

    std::map<std::string, std::string> session_metadata;
    session_metadata["backend"] = sim2real_cfg_.logging.backend;
    session_metadata["policy_name"] = sim2real_cfg_.policy_name;
    session_metadata["active_config_section"] = active_config_section_;
    session_metadata["active_mode_id"] = std::to_string(active_mode_id_);
    session_metadata["output_file_path"] = sim2real_cfg_.logging.output_file_path;
    session_metadata["requested_compression"] = sim2real_cfg_.logging.writer.compression;

    if (!runtime_recorder_.open(
            sim2real_cfg_.logging,
            buildRuntimeConfigSnapshotJson(),
            session_metadata))
    {
        std::cerr << "[RL_solver] failed to open runtime recorder." << std::endl;
        return;
    }

    runtime_logging_enabled_ = true;
    session_metadata["effective_compression"] = runtime_recorder_.effectiveCompression();
    last_logged_mode_id_ = std::numeric_limits<int>::min();
    last_logged_deploy_state_ = std::numeric_limits<int>::min();

    rl_master::logging::RuntimeEventRecord solver_event;
    solver_event.monotonic_time_sec = rl_master::monotonicTimeSec();
    solver_event.event_type = "solver_initialized";
    solver_event.tags["session_base_path"] = sim2real_cfg_.logging.session_base_path;
    solver_event.tags["output_file_path"] = sim2real_cfg_.logging.output_file_path;
    solver_event.tags["requested_compression"] = sim2real_cfg_.logging.writer.compression;
    solver_event.tags["effective_compression"] = runtime_recorder_.effectiveCompression();
    runtime_recorder_.recordEvent(solver_event);

    rl_master::logging::RuntimeEventRecord controller_event;
    controller_event.monotonic_time_sec = rl_master::monotonicTimeSec();
    controller_event.event_type = "controller_initialized";
    controller_event.tags["mode_id"] = std::to_string(active_mode_id_);
    controller_event.tags["config_section"] = active_config_section_;
    runtime_recorder_.recordEvent(controller_event);

    std::cout << "Runtime MCAP log: " << runtime_recorder_.filePath() << std::endl;
}

void RobotSolver::emitBaseImuSourceSample(
    const std::array<float, 3> &ang_vel,
    const std::array<float, 4> &quat,
    const std::array<float, 3> &rpy,
    double monotonic_time_sec)
{
    if (!runtime_logging_enabled_ ||
        !runtime_recorder_.isOpen() ||
        !sim2real_cfg_.logging.source_samples.enabled ||
        !sim2real_cfg_.logging.source_samples.include_base_imu)
    {
        return;
    }

    rl_master::logging::RuntimeSourceSampleRecord sample;
    sample.monotonic_time_sec = monotonic_time_sec;
    sample.topic = "runtime/source/base_imu";
    sample.sample_name = "base_imu";
    sample.tags["backend"] = "sim2real";
    sample.values["ang_vel"] = {ang_vel[0], ang_vel[1], ang_vel[2]};
    sample.values["quat_xyzw"] = {quat[0], quat[1], quat[2], quat[3]};
    sample.values["rpy"] = {rpy[0], rpy[1], rpy[2]};
    runtime_recorder_.recordSourceSample(sample);
}

void RobotSolver::emitDerivedRuntimeEvents(const rl_master::logging::ControllerLogSnapshot &controller_snapshot)
{
    if (!runtime_logging_enabled_ || !runtime_recorder_.isOpen() || !controller_snapshot.valid)
    {
        return;
    }

    if (controller_snapshot.active_mode_id != last_logged_mode_id_)
    {
        rl_master::logging::RuntimeEventRecord event;
        event.monotonic_time_sec = controller_snapshot.monotonic_time_sec;
        event.event_type = "mode_switch";
        event.tags["mode_id"] = std::to_string(controller_snapshot.active_mode_id);
        event.tags["config_section"] = controller_snapshot.active_config_section;
        event.tags["policy_name"] = controller_snapshot.policy_name;
        runtime_recorder_.recordEvent(event);
        last_logged_mode_id_ = controller_snapshot.active_mode_id;
    }

    if (controller_snapshot.deploy_state != last_logged_deploy_state_)
    {
        rl_master::logging::RuntimeEventRecord event;
        event.monotonic_time_sec = controller_snapshot.monotonic_time_sec;
        event.event_type = "lifecycle_transition";
        event.tags["deploy_state"] = std::to_string(controller_snapshot.deploy_state);
        event.tags["mode_id"] = std::to_string(controller_snapshot.active_mode_id);
        runtime_recorder_.recordEvent(event);
        last_logged_deploy_state_ = controller_snapshot.deploy_state;
    }
}

void RobotSolver::logLoopData(const rl_master::logging::ControllerLogSnapshot &controller_snapshot)
{
    if (!runtime_logging_enabled_ || !runtime_recorder_.isOpen())
    {
        return;
    }

    rl_master::logging::RuntimeTickLogRecord record;
    record.frame_index = controller_snapshot.frame_index;
    record.monotonic_time_sec = controller_snapshot.valid
                                    ? controller_snapshot.monotonic_time_sec
                                    : rl_master::monotonicTimeSec();
    record.phase_t = controller_snapshot.phase_t;
    record.phase_t_global = controller_snapshot.phase_t_global;
    record.phase_origin_t = controller_snapshot.phase_origin_t;
    record.requested_mode_command = controller_snapshot.requested_mode_command;
    record.active_mode_id = controller_snapshot.active_mode_id;
    record.deploy_state = controller_snapshot.deploy_state;
    record.active_profile_index = controller_snapshot.active_profile_index;
    record.policy_step_index = controller_snapshot.policy_step_index;
    record.policy_ran_this_tick = controller_snapshot.policy_ran_this_tick;
    record.policy_sample_time_sec = controller_snapshot.policy_sample_time_sec;
    record.policy_sample_age_sec = controller_snapshot.policy_sample_age_sec;
    record.open_rl = controller_snapshot.open_rl;
    record.cmd_vx = controller_snapshot.cmd_vx;
    record.cmd_vy = controller_snapshot.cmd_vy;
    record.cmd_dyaw = controller_snapshot.cmd_dyaw;
    record.latest_cmd_fresh = latest_cmd_fresh_;
    record.loop_overrun_count = loop_overrun_count_;
    record.active_tag = controller_snapshot.active_tag;
    record.active_config_section = controller_snapshot.active_config_section;
    record.policy_name = controller_snapshot.policy_name;
    record.joint_q = controller_snapshot.joint_q;
    record.joint_dq = controller_snapshot.joint_dq;
    record.joint_tau = controller_snapshot.joint_tau;
    record.joint_target_q = controller_snapshot.joint_target_q;
    record.joint_target_tau = controller_snapshot.joint_target_tau;
    record.observation = controller_snapshot.observation;
    record.policy_action = controller_snapshot.policy_action;
    record.named_features = controller_snapshot.named_features;
    record.external_feature_names = controller_snapshot.external_feature_names;
    record.amp_discriminator_score = controller_snapshot.amp_discriminator_score;
    record.has_amp_discriminator_score = controller_snapshot.has_amp_discriminator_score;
    record.amp_discriminator_score_mean = controller_snapshot.amp_discriminator_score_mean;

    record.joint_cmd_q = joint_cmd_q_;
    record.joint_cmd_dq = joint_cmd_dq_;
    record.joint_cmd_tau = joint_cmd_tau_;
    record.joint_state_q = joint_state_q_;
    record.joint_state_dq = joint_state_dq_;
    record.joint_state_tau = joint_state_tau_;
    record.motor_cmd_q = motor_cmd_q_;
    record.motor_cmd_dq = motor_cmd_dq_;
    record.motor_cmd_tau = motor_cmd_tau_;
    record.motor_state_q = motor_state_q_;
    record.motor_state_dq = motor_state_dq_;
    record.motor_state_tau = motor_state_tau_;
    record.motor_cmd_mode = motor_cmd_mode_;

    runtime_recorder_.recordTick(record);

    if (controller_snapshot.policy_ran_this_tick && !controller_snapshot.policy_action.empty())
    {
        rl_master::logging::RuntimeSourceSampleRecord sample;
        sample.monotonic_time_sec = record.monotonic_time_sec;
        sample.topic = "runtime/source/policy_action";
        sample.sample_name = "policy_action";
        sample.tags["backend"] = "sim2real";
        sample.tags["mode_id"] = std::to_string(record.active_mode_id);
        sample.values["action"] = controller_snapshot.policy_action;
        runtime_recorder_.recordSourceSample(sample);
    }

    if (controller_snapshot.policy_ran_this_tick && !controller_snapshot.observation.empty())
    {
        rl_master::logging::RuntimeSourceSampleRecord sample;
        sample.monotonic_time_sec = controller_snapshot.policy_sample_time_sec > 0.0
                                        ? controller_snapshot.policy_sample_time_sec
                                        : record.monotonic_time_sec;
        sample.topic = "runtime/source/policy_observation";
        sample.sample_name = "policy_observation";
        sample.tags["backend"] = "sim2real";
        sample.tags["mode_id"] = std::to_string(record.active_mode_id);
        sample.tags["policy_step_index"] = std::to_string(record.policy_step_index);
        sample.values["observation"] = controller_snapshot.observation;
        runtime_recorder_.recordSourceSample(sample);
    }

    if (sim2real_cfg_.logging.source_samples.include_external_observations)
    {
        for (auto &sample : controller_runtime_.controller().drainExternalObservationSamplesForLogging())
        {
            sample.tags["backend"] = "sim2real";
            runtime_recorder_.recordSourceSample(sample);
        }
    }
}

void RobotSolver::holdCurrentPose()
{
    getMotorState();
    const size_t installed_count = installedJointCount();
    for (size_t i = 0; i < installed_count; ++i)
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
    initRuntimeRecorder();

    getMotorState();
    sendRLState();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

            int mode_control_word_value = 0;
            std::optional<int> mode_control_word;
            if (dds_bridge_.readLatestModeControlWord(&mode_control_word_value))
            {
                mode_control_word = mode_control_word_value;
            }

            rl_master::TeleopCommand teleop_command{};
            std::optional<rl_master::TeleopCommand> teleop_sample;
            if (dds_bridge_.readLatestTeleopCommand(&teleop_command))
            {
                teleop_sample = teleop_command;
            }

            getMotorState();

            rl_master::RobotStateData io_state = buildControllerStateData();
            const rl_master::RobotCommandData controller_command =
                controller_runtime_.step(io_state, teleop_sample, mode_control_word);
            syncRuntimeCfgFromController();
            const long control_period_ns = resolveControlPeriodNs(sim2real_cfg_);
            const auto &controller_snapshot = controller_runtime_.controller().latestLogSnapshot();
            emitDerivedRuntimeEvents(controller_snapshot);
            applyRuntimeCommand(controller_command, true);

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
                dds_bridge_.mirrorRobotState(io_state);
                sendMotorCmd();
                logLoopData(controller_snapshot);
            }
            else
            {
                if (!hold_target_latched_)
                {
                    std::cout << "[RL_solver] hold mode active" << std::endl;
                    const size_t installed_count = installedJointCount();
                    for (size_t i = 0; i < installed_count; ++i)
                    {
                        hold_target_q_[i] = joint_state_[i].q;
                    }
                    hold_target_latched_ = true;
                }

                const size_t installed_count = installedJointCount();
                for (size_t i = 0; i < installed_count; ++i)
                {
                    joint_cmd_[i].q = hold_target_q_[i];
                    joint_cmd_[i].dq = 0.0;
                    joint_cmd_[i].tau = 0.0;
                    joint_cmd_[i].mode = RUN_MODE_CSP;
                }
                sendMotorCmd();
                dds_bridge_.mirrorRobotState(io_state);
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

            next.tv_nsec += control_period_ns;
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
    runtime_recorder_.flush();
    runtime_recorder_.close();
}

} // namespace rl_master::solver
