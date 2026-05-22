#include "joint_motor_test/joint_motor_test_runner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <ctime>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "rl_master/dds_protocol.h"
#include "rl_master/mode_profile_registry.h"
#include "rl_master/rl_cfg.h"
#include "rl_master/rl_protocol.h"

namespace joint_motor_test
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string nowTag()
{
    const auto t = std::time(nullptr);
    const auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    return oss.str();
}

std::vector<float> toStateQ(const rl_master::RobotStateData &state)
{
    return std::vector<float>(state.joint_q.begin(), state.joint_q.end());
}

std::vector<float> toStateDq(const rl_master::RobotStateData &state)
{
    return std::vector<float>(state.joint_dq.begin(), state.joint_dq.end());
}

float safeRead(const std::vector<float> &values, size_t index, float fallback = 0.0f)
{
    return index < values.size() ? values[index] : fallback;
}

std::string sourceName(TrajectorySource src)
{
    switch (src)
    {
    case TrajectorySource::kFile:
        return "file";
    case TrajectorySource::kSine:
        return "sine";
    default:
        return "unknown";
    }
}

std::string controlModeName(MotorControlMode mode)
{
    switch (mode)
    {
    case MotorControlMode::kCsp:
        return "csp";
    case MotorControlMode::kCst:
        return "cst";
    case MotorControlMode::kR1:
        return "r1";
    default:
        return "unknown";
    }
}

} // namespace

JointMotorTestRunner::JointMotorTestRunner()
{
    node_ = rclcpp::Node::make_shared("joint_motor_test_runner");

    loadConfig();
    loadTrajectory();

    command_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRuntimeCommand,
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    state_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
            this->onStateMsg(msg);
        });

    mode_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicModeControl,
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            this->onModeMsg(msg);
        });

    initLogger();

    RCLCPP_INFO(
        node_->get_logger(),
        "joint_motor_test ready: mode_id=%d source=%s control_mode=%s hz=%.2f joints=%zu frames=%zu",
        config_.test_mode_id,
        sourceName(config_.trajectory_source).c_str(),
        controlModeName(config_.control_mode).c_str(),
        config_.control_hz,
        joint_count_,
        trajectory_.size());
}

void JointMotorTestRunner::run()
{
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, config_.control_hz));
    auto next_tick = std::chrono::steady_clock::now();
    auto last_no_state_warn = std::chrono::steady_clock::time_point{};

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node_);

        if (!has_state_)
        {
            const auto now = std::chrono::steady_clock::now();
            if (last_no_state_warn.time_since_epoch().count() == 0 ||
                (now - last_no_state_warn) > std::chrono::seconds(1))
            {
                RCLCPP_WARN(node_->get_logger(), "waiting DDS robot state on %s", rl_master::dds::kTopicRobotState);
                last_no_state_warn = now;
            }

            const double now_sec = rl_master::monotonicTimeSec();
            publishCommand(buildDisabledCommand(), now_sec);

            next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        initializeStateMachineIfNeeded();

        const double now_sec = rl_master::monotonicTimeSec();
        const rl_master::DeployStateOutput deploy_output = updateStateMachine(now_sec);

        int mode_command_snapshot = -1;
        {
            std::lock_guard<std::mutex> lock(mode_mutex_);
            mode_command_snapshot = latest_mode_command_;
        }

        const bool entered_running =
            (deploy_output.state == rl_master::DeployLifecycleState::kRunning) &&
            (last_lifecycle_state_ != rl_master::DeployLifecycleState::kRunning);
        if (entered_running && config_.restart_trajectory_on_enter_running)
        {
            playback_index_ = 0;
        }

        const bool running_for_test_mode = deploy_output.enable_policy &&
                                           (deploy_output.locomotion_mode == config_.test_mode_id);

        rl_master::RobotCommandData command;
        if (running_for_test_mode)
        {
            command = buildPlaybackCommand();
        }
        else if (deploy_output.enable_command_stream)
        {
            command = buildZeroingCommand(deploy_output.target_q);
        }
        else
        {
            command = buildDisabledCommand();
        }

        if (deploy_output.state != last_lifecycle_state_)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "lifecycle -> %s (mode_id=%d)",
                rl_master::DeployStateMachine::stateName(deploy_output.state),
                deploy_output.locomotion_mode);
            if (logger_enabled_)
            {
                logger_.writeEvent(
                    now_sec,
                    "lifecycle_transition",
                    {
                        {"state", rl_master::DeployStateMachine::stateName(deploy_output.state)},
                        {"mode_id", std::to_string(deploy_output.locomotion_mode)},
                    });
            }
            last_lifecycle_state_ = deploy_output.state;
        }

        publishCommand(command, now_sec);
        logStep(deploy_output, mode_command_snapshot, command);
        ++step_index_;

        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        const auto now = std::chrono::steady_clock::now();
        if (now > next_tick)
        {
            next_tick = now;
        }
        std::this_thread::sleep_until(next_tick);
    }

    logger_.flush();
}

void JointMotorTestRunner::loadConfig()
{
    std::string default_config_path;
    try
    {
        default_config_path =
            ament_index_cpp::get_package_share_directory("joint_motor_test") + "/config/joint_motor_test.yaml";
    }
    catch (const std::exception &)
    {
        default_config_path = "joint_motor_test.yaml";
    }

    const std::string config_path = node_->declare_parameter<std::string>("config_path", default_config_path);

    YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node cfg = (root["joint_motor_test"] ? root["joint_motor_test"] : root);

    auto readString = [&cfg](const char *key, const std::string &fallback) {
        return cfg[key] ? cfg[key].as<std::string>() : fallback;
    };
    auto readBool = [&cfg](const char *key, bool fallback) {
        return cfg[key] ? cfg[key].as<bool>() : fallback;
    };
    auto readInt = [&cfg](const char *key, int fallback) {
        return cfg[key] ? cfg[key].as<int>() : fallback;
    };
    auto readDouble = [&cfg](const char *key, double fallback) {
        return cfg[key] ? cfg[key].as<double>() : fallback;
    };

    config_.test_mode_id = readInt("test_mode_id", config_.test_mode_id);
    config_.control_hz = std::max(1.0, readDouble("control_hz", config_.control_hz));
    config_.deploy_config_path = readString("deploy_config_path", "");
    if (config_.deploy_config_path.empty())
    {
        config_.deploy_config_path = RL_CFG_PATH;
    }
    if (cfg["joint_names"])
    {
        config_.joint_names = cfg["joint_names"].as<std::vector<std::string>>();
    }

    resolveJointLayout();

    config_.trajectory_source = parseTrajectorySource(readString("trajectory_source", "file"));
    config_.control_mode = parseControlMode(readString("control_mode", "csp"));

    config_.trajectory_file = readString("trajectory_file", "");
    config_.loop_trajectory = readBool("loop_trajectory", true);
    config_.restart_trajectory_on_enter_running = readBool("restart_trajectory_on_enter_running", true);

    config_.startup_completion_action = readString("startup_completion_action", "hold");
    std::transform(
        config_.startup_completion_action.begin(),
        config_.startup_completion_action.end(),
        config_.startup_completion_action.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (cfg["auto_start_policy"] && !cfg["startup_completion_action"])
    {
        config_.startup_completion_action = readBool("auto_start_policy", false) ? "running" : "hold";
    }
    if (config_.startup_completion_action != "hold" &&
        config_.startup_completion_action != "running")
    {
        throw std::runtime_error("startup_completion_action must be 'hold' or 'running'");
    }
    config_.zeroing_duration_s = std::max(0.05, readDouble("zeroing_duration_s", 2.0));

    if (cfg["zero_pose"])
    {
        config_.zero_pose = cfg["zero_pose"].as<std::vector<float>>();
    }
    config_.zero_pose = normalizeJointVector(config_.zero_pose, joint_count_, 0.0f);

    if (cfg["fallback_kp"])
    {
        config_.fallback_kp = cfg["fallback_kp"].as<std::vector<float>>();
    }
    config_.fallback_kp = normalizeJointVector(config_.fallback_kp, joint_count_, 60.0f);

    if (cfg["fallback_kd"])
    {
        config_.fallback_kd = cfg["fallback_kd"].as<std::vector<float>>();
    }
    config_.fallback_kd = normalizeJointVector(config_.fallback_kd, joint_count_, 2.0f);

    if (cfg["tau_limit"])
    {
        config_.tau_limit = cfg["tau_limit"].as<std::vector<float>>();
    }
    config_.tau_limit = normalizeJointVector(config_.tau_limit, joint_count_, 120.0f);
    config_.strict_safety_checks = readBool("strict_safety_checks", true);
    config_.max_abs_q = static_cast<float>(std::max(0.1, readDouble("max_abs_q", 6.5)));
    config_.max_abs_dq = static_cast<float>(std::max(0.1, readDouble("max_abs_dq", 40.0)));
    config_.max_abs_tau = static_cast<float>(std::max(0.1, readDouble("max_abs_tau", 200.0)));

    config_.save_data = readBool("save_data", true);
    config_.data_path = readString("data_path", "");
    if (config_.data_path.empty())
    {
        config_.data_path = (std::filesystem::current_path() / "data" / ("joint_motor_test_" + nowTag())).string();
    }

    const YAML::Node sine = cfg["sine"];
    if (sine)
    {
        config_.sine.duration_sec = sine["duration_sec"] ? sine["duration_sec"].as<double>() : config_.sine.duration_sec;
        if (sine["offset"])
        {
            config_.sine.offset = sine["offset"].as<std::vector<float>>();
        }
        if (sine["amplitude"])
        {
            config_.sine.amplitude = sine["amplitude"].as<std::vector<float>>();
        }
        if (sine["period_sec"])
        {
            config_.sine.period_sec = sine["period_sec"].as<std::vector<float>>();
        }
        if (sine["phase_rad"])
        {
            config_.sine.phase_rad = sine["phase_rad"].as<std::vector<float>>();
        }
        config_.sine.activation_mode = parseSineActivationMode(
            sine["activation_mode"] ? sine["activation_mode"].as<std::string>() : "all");
        if (sine["sequential_joint_order"])
        {
            config_.sine.sequential_joint_order = sine["sequential_joint_order"].as<std::vector<int>>();
        }
        config_.sine.sequential_segment_sec = sine["sequential_segment_sec"]
                                                  ? sine["sequential_segment_sec"].as<double>()
                                                  : config_.sine.sequential_segment_sec;
        config_.sine.export_reference_path = sine["export_reference_path"]
                                                 ? sine["export_reference_path"].as<std::string>()
                                                 : "";
    }

    config_.sine.offset = normalizeJointVector(config_.sine.offset, joint_count_, 0.0f);
    config_.sine.amplitude = normalizeJointVector(config_.sine.amplitude, joint_count_, 0.1f);
    config_.sine.period_sec = normalizeJointVector(config_.sine.period_sec, joint_count_, 2.0f);
    config_.sine.phase_rad = normalizeJointVector(config_.sine.phase_rad, joint_count_, 0.0f);
    config_.sine.sequential_joint_order = normalizeJointOrder(config_.sine.sequential_joint_order, joint_count_);
    config_.sine.duration_sec = std::max(0.2, config_.sine.duration_sec);
    config_.sine.sequential_segment_sec = std::max(0.1, config_.sine.sequential_segment_sec);
}

void JointMotorTestRunner::resolveJointLayout()
{
    joint_names_.clear();
    if (!config_.joint_names.empty())
    {
        joint_names_ = config_.joint_names;
    }
    else
    {
        auto registry = rl_master::ModeProfileRegistry::loadFromYaml(config_.deploy_config_path, "engineai_walk");
        const auto &cfg = registry->cfgForMode(config_.test_mode_id, false);
        joint_names_ = cfg.action_joint_order;
    }

    if (joint_names_.empty())
    {
        throw std::runtime_error(
            "joint_motor_test failed to resolve joint layout from config_path=" +
            config_.deploy_config_path +
            ", mode_id=" +
            std::to_string(config_.test_mode_id));
    }
    joint_count_ = joint_names_.size();
}

void JointMotorTestRunner::loadTrajectory()
{
    trajectory_.clear();
    playback_index_ = 0;

    if (config_.trajectory_source == TrajectorySource::kFile)
    {
        if (config_.trajectory_file.empty())
        {
            throw std::runtime_error("trajectory_source=file but trajectory_file is empty");
        }
        loadTrajectoryFromFile(config_.trajectory_file);
    }
    else
    {
        generateSineTrajectory();
        if (!config_.sine.export_reference_path.empty())
        {
            exportTrajectoryToCsv(config_.sine.export_reference_path);
        }
    }

    if (trajectory_.empty())
    {
        throw std::runtime_error("trajectory is empty");
    }

    validateTrajectory();
}

void JointMotorTestRunner::loadTrajectoryFromFile(const std::string &path)
{
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        throw std::runtime_error("failed to open trajectory file: " + path);
    }

    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(ifs, line))
    {
        if (line.empty())
        {
            continue;
        }
        const char first_char = line[0];
        if (first_char == '#')
        {
            continue;
        }

        std::vector<double> numeric;
        if (!parseNumericRow(line, &numeric))
        {
            if (rows.empty())
            {
                continue;
            }
            throw std::runtime_error("invalid numeric row in trajectory file: " + line);
        }
        rows.push_back(std::move(numeric));
    }

    if (rows.empty())
    {
        throw std::runtime_error("trajectory file has no valid numeric rows: " + path);
    }

    const size_t col_count = rows.front().size();
    bool has_time = false;
    bool has_dq = false;
    bool has_tau = false;

    if (col_count == joint_count_)
    {
        has_time = false;
        has_dq = false;
        has_tau = false;
    }
    else if (col_count == joint_count_ + 1)
    {
        has_time = true;
        has_dq = false;
        has_tau = false;
    }
    else if (col_count == joint_count_ * 2)
    {
        has_time = false;
        has_dq = true;
        has_tau = false;
    }
    else if (col_count == joint_count_ * 2 + 1)
    {
        has_time = true;
        has_dq = true;
        has_tau = false;
    }
    else if (col_count == joint_count_ * 3)
    {
        has_time = false;
        has_dq = true;
        has_tau = true;
    }
    else if (col_count == joint_count_ * 3 + 1)
    {
        has_time = true;
        has_dq = true;
        has_tau = true;
    }
    else
    {
        throw std::runtime_error(
            "unsupported trajectory column count: " + std::to_string(col_count) +
            " for joint_count=" + std::to_string(joint_count_) +
            " (expect N/N+1/2N/2N+1/3N/3N+1)");
    }

    trajectory_.reserve(rows.size());
    for (const auto &row : rows)
    {
        if (row.size() != col_count)
        {
            throw std::runtime_error("trajectory row column mismatch");
        }

        size_t cursor = has_time ? 1 : 0;

        TrajectoryFrame frame;
        frame.q.assign(joint_count_, 0.0f);
        frame.dq.assign(joint_count_, 0.0f);
        frame.tau.assign(joint_count_, 0.0f);
        for (size_t i = 0; i < joint_count_; ++i)
        {
            frame.q[i] = static_cast<float>(row[cursor + i]);
        }
        cursor += joint_count_;

        if (has_dq)
        {
            for (size_t i = 0; i < joint_count_; ++i)
            {
                frame.dq[i] = static_cast<float>(row[cursor + i]);
            }
            cursor += joint_count_;
        }

        if (has_tau)
        {
            for (size_t i = 0; i < joint_count_; ++i)
            {
                frame.tau[i] = static_cast<float>(row[cursor + i]);
            }
        }

        trajectory_.push_back(frame);
    }

    if (!has_dq)
    {
        const float dt = static_cast<float>(1.0 / std::max(1.0, config_.control_hz));
        for (size_t i = 1; i < trajectory_.size(); ++i)
        {
            for (size_t j = 0; j < joint_count_; ++j)
            {
                trajectory_[i - 1].dq[j] = (trajectory_[i].q[j] - trajectory_[i - 1].q[j]) / dt;
            }
        }
        if (trajectory_.size() > 1)
        {
            trajectory_.back().dq = trajectory_[trajectory_.size() - 2].dq;
        }
    }

    if (!has_tau)
    {
        for (auto &frame : trajectory_)
        {
            std::fill(frame.tau.begin(), frame.tau.end(), 0.0f);
        }
    }

    trajectory_has_input_dq_ = has_dq;
    trajectory_has_input_tau_ = has_tau;
}

void JointMotorTestRunner::generateSineTrajectory()
{
    const size_t total_steps = std::max<size_t>(1, static_cast<size_t>(std::lround(config_.sine.duration_sec * config_.control_hz)));
    trajectory_.reserve(total_steps);

    for (size_t step = 0; step < total_steps; ++step)
    {
        const double t = static_cast<double>(step) / std::max(1.0, config_.control_hz);

        int active_joint = -1;
        if (config_.sine.activation_mode == SineActivationMode::kSequential)
        {
            const size_t segment_index = static_cast<size_t>(std::floor(t / config_.sine.sequential_segment_sec));
            if (!config_.sine.sequential_joint_order.empty())
            {
                active_joint = config_.sine.sequential_joint_order[segment_index % config_.sine.sequential_joint_order.size()];
            }
        }

        TrajectoryFrame frame;
        frame.q.assign(joint_count_, 0.0f);
        frame.dq.assign(joint_count_, 0.0f);
        frame.tau.assign(joint_count_, 0.0f);
        for (size_t i = 0; i < joint_count_; ++i)
        {
            const bool joint_active = (config_.sine.activation_mode == SineActivationMode::kAll) ||
                                      (static_cast<int>(i) == active_joint);
            if (joint_active)
            {
                const double period = std::max(1e-4, static_cast<double>(config_.sine.period_sec[i]));
                const double omega = 2.0 * kPi / period;
                const double phase = static_cast<double>(config_.sine.phase_rad[i]);
                const double amp = static_cast<double>(config_.sine.amplitude[i]);
                const double offset = static_cast<double>(config_.sine.offset[i]);

                frame.q[i] = static_cast<float>(offset + amp * std::sin(omega * t + phase));
                frame.dq[i] = static_cast<float>(amp * omega * std::cos(omega * t + phase));
            }
            else
            {
                frame.q[i] = config_.sine.offset[i];
                frame.dq[i] = 0.0f;
            }
            frame.tau[i] = 0.0f;
        }

        trajectory_.push_back(frame);
    }

    trajectory_has_input_dq_ = true;
    trajectory_has_input_tau_ = false;
}

void JointMotorTestRunner::exportTrajectoryToCsv(const std::string &path) const
{
    if (trajectory_.empty())
    {
        return;
    }

    const std::filesystem::path out_path(path);
    if (!out_path.parent_path().empty())
    {
        std::filesystem::create_directories(out_path.parent_path());
    }

    std::ofstream ofs(path, std::ios::out | std::ios::trunc);
    if (!ofs.is_open())
    {
        throw std::runtime_error("failed to export generated trajectory: " + path);
    }

    ofs << "time_s";
    for (size_t i = 0; i < joint_count_; ++i)
    {
        ofs << ",q:" << (i < joint_names_.size() ? joint_names_[i] : std::to_string(i));
    }
    for (size_t i = 0; i < joint_count_; ++i)
    {
        ofs << ",dq:" << (i < joint_names_.size() ? joint_names_[i] : std::to_string(i));
    }
    for (size_t i = 0; i < joint_count_; ++i)
    {
        ofs << ",tau:" << (i < joint_names_.size() ? joint_names_[i] : std::to_string(i));
    }
    ofs << "\n";

    const double dt = 1.0 / std::max(1.0, config_.control_hz);
    for (size_t step = 0; step < trajectory_.size(); ++step)
    {
        ofs << std::fixed << std::setprecision(6) << (step * dt);
        for (size_t i = 0; i < joint_count_; ++i)
        {
            ofs << "," << std::setprecision(9) << trajectory_[step].q[i];
        }
        for (size_t i = 0; i < joint_count_; ++i)
        {
            ofs << "," << std::setprecision(9) << trajectory_[step].dq[i];
        }
        for (size_t i = 0; i < joint_count_; ++i)
        {
            ofs << "," << std::setprecision(9) << trajectory_[step].tau[i];
        }
        ofs << "\n";
    }
}

void JointMotorTestRunner::validateTrajectory() const
{
    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
        validateTrajectoryFrame(trajectory_[i], i);
    }
}

void JointMotorTestRunner::validateTrajectoryFrame(const TrajectoryFrame &frame, size_t frame_index) const
{
    auto checkFinite = [frame_index](float value, const char *field, size_t joint_idx) {
        if (!std::isfinite(value))
        {
            throw std::runtime_error(
                "trajectory contains non-finite value at frame=" + std::to_string(frame_index) +
                ", joint=" + std::to_string(joint_idx) + ", field=" + field);
        }
    };

    for (size_t i = 0; i < joint_count_; ++i)
    {
        checkFinite(frame.q[i], "q", i);
        checkFinite(frame.dq[i], "dq", i);
        checkFinite(frame.tau[i], "tau", i);

        const float q_abs = std::abs(frame.q[i]);
        const float dq_abs = std::abs(frame.dq[i]);
        const float tau_abs = std::abs(frame.tau[i]);
        const float tau_limit = std::min(std::abs(config_.tau_limit[i]), config_.max_abs_tau);

        if (config_.strict_safety_checks)
        {
            if (q_abs > config_.max_abs_q)
            {
                throw std::runtime_error(
                    "trajectory q exceeds max_abs_q at frame=" + std::to_string(frame_index) +
                    ", joint=" + std::to_string(i));
            }
            if (dq_abs > config_.max_abs_dq)
            {
                throw std::runtime_error(
                    "trajectory dq exceeds max_abs_dq at frame=" + std::to_string(frame_index) +
                    ", joint=" + std::to_string(i));
            }
            if (tau_abs > tau_limit)
            {
                throw std::runtime_error(
                    "trajectory tau exceeds limit at frame=" + std::to_string(frame_index) +
                    ", joint=" + std::to_string(i));
            }
        }
    }
}

void JointMotorTestRunner::initializeStateMachineIfNeeded()
{
    if (state_machine_initialized_)
    {
        return;
    }

    rl_master::RobotStateData state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!has_state_)
        {
            return;
        }
        state = latest_state_;
    }

    Sim2realCfg cfg;
    cfg.startup_completion_action = config_.startup_completion_action;
    cfg.zeroing_duration_s = config_.zeroing_duration_s;

    state_machine_.configure(cfg);
    state_machine_.initialize(toStateQ(state), config_.zero_pose, config_.test_mode_id);
    state_machine_.setZeroPose(config_.zero_pose);
    state_machine_initialized_ = true;
    last_lifecycle_state_ = state_machine_.state();
}

rl_master::DeployStateOutput JointMotorTestRunner::updateStateMachine(double now_sec)
{
    rl_master::RobotStateData state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = latest_state_;
    }

    int mode_command = -1;
    {
        std::lock_guard<std::mutex> lock(mode_mutex_);
        mode_command = latest_mode_command_;
    }

    return state_machine_.update(
        mode_command,
        now_sec,
        toStateQ(state),
        toStateDq(state));
}

rl_master::RobotCommandData JointMotorTestRunner::buildPlaybackCommand()
{
    rl_master::RobotStateData state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = latest_state_;
    }

    if (trajectory_.empty())
    {
        return buildDisabledCommand();
    }

    if (playback_index_ >= trajectory_.size())
    {
        playback_index_ = config_.loop_trajectory ? 0 : (trajectory_.size() - 1);
    }

    const TrajectoryFrame &frame = trajectory_[playback_index_];
    if (playback_index_ + 1 < trajectory_.size())
    {
        ++playback_index_;
    }
    else if (config_.loop_trajectory)
    {
        playback_index_ = 0;
    }

    rl_master::RobotCommandData command;
    command.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    command.active_joint_count = static_cast<int>(joint_count_);
    command.joint_target_q.assign(joint_count_, 0.0f);
    command.joint_target_dq.assign(joint_count_, 0.0f);
    command.joint_target_tau.assign(joint_count_, 0.0f);
    auto clipAbs = [](float value, float limit) {
        const float safe_limit = std::max(0.0f, std::abs(limit));
        if (safe_limit <= 1e-6f)
        {
            return 0.0f;
        }
        return std::clamp(value, -safe_limit, safe_limit);
    };

    if (config_.control_mode == MotorControlMode::kCsp)
    {
        command.open_rl = rl_master::kOpenRlTestCspStream;
        for (size_t i = 0; i < joint_count_; ++i)
        {
            command.joint_target_q[i] = clipAbs(frame.q[i], config_.max_abs_q);
            command.joint_target_dq[i] = clipAbs(frame.dq[i], config_.max_abs_dq);
        }
        return command;
    }

    for (size_t i = 0; i < joint_count_; ++i)
    {
        command.joint_target_q[i] = clipAbs(frame.q[i], config_.max_abs_q);
        command.joint_target_dq[i] = clipAbs(frame.dq[i], config_.max_abs_dq);
    }

    for (size_t i = 0; i < joint_count_; ++i)
    {
        float tau_cmd = frame.tau[i];
        if (!trajectory_has_input_tau_)
        {
            tau_cmd = (frame.q[i] - safeRead(state.joint_q, i, 0.0f)) * config_.fallback_kp[i] +
                      (frame.dq[i] - safeRead(state.joint_dq, i, 0.0f)) * config_.fallback_kd[i];
        }
        const float safe_tau_limit = std::min(std::abs(config_.tau_limit[i]), config_.max_abs_tau);
        command.joint_target_tau[i] = clampTorque(tau_cmd, safe_tau_limit);
    }

    if (config_.control_mode == MotorControlMode::kCst)
    {
        command.open_rl = rl_master::kOpenRlTestCstStream;
    }
    else
    {
        command.open_rl = rl_master::kOpenRlTestR1Stream;
    }
    return command;
}

rl_master::RobotCommandData JointMotorTestRunner::buildZeroingCommand(const std::vector<float> &target_q) const
{
    rl_master::RobotCommandData command;
    command.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    command.active_joint_count = static_cast<int>(joint_count_);
    command.joint_target_q.assign(joint_count_, 0.0f);
    command.joint_target_dq.assign(joint_count_, 0.0f);
    command.joint_target_tau.assign(joint_count_, 0.0f);
    command.open_rl = rl_master::kOpenRlCommandStream;

    const size_t copy_n = std::min(target_q.size(), joint_count_);
    for (size_t i = 0; i < copy_n; ++i)
    {
        command.joint_target_q[i] = target_q[i];
    }
    return command;
}

rl_master::RobotCommandData JointMotorTestRunner::buildDisabledCommand()
{
    rl_master::RobotCommandData command;
    command.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    command.open_rl = rl_master::kOpenRlDisabled;

    rl_master::RobotStateData state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = latest_state_;
    }
    command.active_joint_count = static_cast<int>(std::max(joint_count_, state.joint_q.size()));
    command.joint_target_q = state.joint_q;
    if (command.joint_target_q.size() < joint_count_)
    {
        command.joint_target_q.resize(joint_count_, 0.0f);
    }
    command.joint_target_dq.assign(command.joint_target_q.size(), 0.0f);
    command.joint_target_tau.assign(command.joint_target_q.size(), 0.0f);
    return command;
}

void JointMotorTestRunner::publishCommand(const rl_master::RobotCommandData &command, double now_sec)
{
    const auto msg = rl_master::dds::encodeRuntimeCommand(command, ++command_sequence_, now_sec);
    command_pub_->publish(msg);
}

void JointMotorTestRunner::logStep(
    const rl_master::DeployStateOutput &deploy_output,
    int mode_command,
    const rl_master::RobotCommandData &command)
{
    if (!logger_enabled_)
    {
        return;
    }

    rl_master::RobotStateData state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = latest_state_;
    }

    std::map<std::string, double> scalars;
    scalars["step_index"] = static_cast<double>(step_index_);
    scalars["mode_command"] = static_cast<double>(mode_command);
    scalars["lifecycle_state"] = static_cast<double>(static_cast<int>(deploy_output.state));
    scalars["locomotion_mode"] = static_cast<double>(deploy_output.locomotion_mode);
    scalars["open_rl"] = static_cast<double>(command.open_rl);
    scalars["playback_index"] = static_cast<double>(playback_index_);

    std::vector<float> q_err(joint_count_, 0.0f);
    double rmse = 0.0;
    for (size_t i = 0; i < joint_count_; ++i)
    {
        q_err[i] = safeRead(command.joint_target_q, i, 0.0f) - safeRead(state.joint_q, i, 0.0f);
        rmse += static_cast<double>(q_err[i] * q_err[i]);
    }
    rmse = joint_count_ > 0 ? std::sqrt(rmse / static_cast<double>(joint_count_)) : 0.0;
    scalars["joint_q_tracking_rmse"] = rmse;

    std::map<std::string, std::vector<float>> vectors;
    vectors["state_q"] = state.joint_q;
    vectors["state_dq"] = state.joint_dq;
    vectors["state_tau"] = state.joint_tau;
    vectors["cmd_q"] = command.joint_target_q;
    vectors["cmd_dq"] = command.joint_target_dq;
    vectors["cmd_tau"] = command.joint_target_tau;
    vectors["q_error"] = q_err;

    logger_.writeRecord(rl_master::monotonicTimeSec(), "joint_motor_test", scalars, vectors);
}

void JointMotorTestRunner::initLogger()
{
    if (!config_.save_data)
    {
        logger_enabled_ = false;
        return;
    }

    rl_master::logging::LoggerMetadata metadata;
    metadata.string_fields["module"] = "joint_motor_test";
    metadata.string_fields["trajectory_source"] = sourceName(config_.trajectory_source);
    metadata.string_fields["control_mode"] = controlModeName(config_.control_mode);
    metadata.string_fields["trajectory_file"] = config_.trajectory_file;
    metadata.string_fields["sine_activation_mode"] =
        (config_.sine.activation_mode == SineActivationMode::kAll) ? "all" : "sequential";

    metadata.numeric_fields["test_mode_id"] = static_cast<double>(config_.test_mode_id);
    metadata.numeric_fields["control_hz"] = config_.control_hz;
    metadata.numeric_fields["joint_count"] = static_cast<double>(joint_count_);
    metadata.numeric_fields["frame_count"] = static_cast<double>(trajectory_.size());
    metadata.numeric_fields["loop_trajectory"] = config_.loop_trajectory ? 1.0 : 0.0;
    metadata.numeric_fields["zeroing_duration_s"] = config_.zeroing_duration_s;
    metadata.numeric_fields["strict_safety_checks"] = config_.strict_safety_checks ? 1.0 : 0.0;
    metadata.numeric_fields["max_abs_q"] = static_cast<double>(config_.max_abs_q);
    metadata.numeric_fields["max_abs_dq"] = static_cast<double>(config_.max_abs_dq);
    metadata.numeric_fields["max_abs_tau"] = static_cast<double>(config_.max_abs_tau);

    metadata.vector_fields["fallback_kp"] = std::vector<double>(config_.fallback_kp.begin(), config_.fallback_kp.end());
    metadata.vector_fields["fallback_kd"] = std::vector<double>(config_.fallback_kd.begin(), config_.fallback_kd.end());
    metadata.vector_fields["tau_limit"] = std::vector<double>(config_.tau_limit.begin(), config_.tau_limit.end());

    std::vector<std::string> order;
    order.reserve(config_.sine.sequential_joint_order.size());
    for (int idx : config_.sine.sequential_joint_order)
    {
        order.push_back(std::to_string(idx));
    }
    metadata.string_list_fields["sine_sequential_joint_order"] = order;
    metadata.string_list_fields["joint_names"] = joint_names_;

    if (!logger_.open(config_.data_path, "joint_motor_test", metadata))
    {
        RCLCPP_ERROR(node_->get_logger(), "failed to open structured logger at %s", config_.data_path.c_str());
        logger_enabled_ = false;
        return;
    }

    logger_enabled_ = true;
    logger_.writeEvent(
        rl_master::monotonicTimeSec(),
        "joint_motor_test_started",
        {
            {"mode_id", std::to_string(config_.test_mode_id)},
            {"source", sourceName(config_.trajectory_source)},
            {"control_mode", controlModeName(config_.control_mode)},
        });

    RCLCPP_INFO(node_->get_logger(), "joint_motor_test structured log: %s", logger_.recordsPath().c_str());
}

void JointMotorTestRunner::onStateMsg(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    rl_master::RobotStateData parsed;
    if (!rl_master::dds::decodeRobotState(*msg, &parsed))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = parsed;
    has_state_ = true;
}

void JointMotorTestRunner::onModeMsg(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (!rl_master::DeployStateMachine::isValidControlWord(msg->data))
    {
        return;
    }
    std::lock_guard<std::mutex> lock(mode_mutex_);
    latest_mode_command_ = msg->data;
}

std::string JointMotorTestRunner::toLower(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::vector<std::string> JointMotorTestRunner::splitCsv(const std::string &line)
{
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, ','))
    {
        out.push_back(token);
    }
    return out;
}

bool JointMotorTestRunner::parseNumericRow(const std::string &line, std::vector<double> *values)
{
    if (!values)
    {
        return false;
    }
    values->clear();

    auto parseToken = [values](const std::string &token) -> bool {
        std::string trimmed = token;
        trimmed.erase(trimmed.begin(), std::find_if(trimmed.begin(), trimmed.end(), [](unsigned char c) {
                          return !std::isspace(c);
                      }));
        trimmed.erase(std::find_if(trimmed.rbegin(), trimmed.rend(), [](unsigned char c) {
                          return !std::isspace(c);
                      }).base(),
                      trimmed.end());
        if (trimmed.empty())
        {
            return true;
        }
        try
        {
            values->push_back(std::stod(trimmed));
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    };

    if (line.find(',') != std::string::npos)
    {
        const auto tokens = splitCsv(line);
        for (const auto &token : tokens)
        {
            if (!parseToken(token))
            {
                return false;
            }
        }
    }
    else
    {
        std::stringstream ss(line);
        std::string token;
        while (ss >> token)
        {
            if (!parseToken(token))
            {
                return false;
            }
        }
    }

    return !values->empty();
}

std::vector<float> JointMotorTestRunner::normalizeJointVector(const std::vector<float> &input, size_t joint_count, float fallback)
{
    if (joint_count == 0)
    {
        return {};
    }
    if (input.empty())
    {
        return std::vector<float>(joint_count, fallback);
    }
    if (input.size() == 1)
    {
        return std::vector<float>(joint_count, input.front());
    }
    if (input.size() != joint_count)
    {
        throw std::runtime_error(
            "joint vector size must be 1 or " + std::to_string(joint_count));
    }
    return input;
}

std::vector<int> JointMotorTestRunner::normalizeJointOrder(const std::vector<int> &input, size_t joint_count)
{
    std::vector<int> out;
    std::vector<bool> used(joint_count, false);

    if (input.empty())
    {
        out.reserve(joint_count);
        for (size_t i = 0; i < joint_count; ++i)
        {
            out.push_back(static_cast<int>(i));
        }
        return out;
    }

    for (int idx : input)
    {
        if (idx < 0 || static_cast<size_t>(idx) >= joint_count)
        {
            continue;
        }
        if (used[static_cast<size_t>(idx)])
        {
            continue;
        }
        used[static_cast<size_t>(idx)] = true;
        out.push_back(idx);
    }

    if (out.empty())
    {
        for (size_t i = 0; i < joint_count; ++i)
        {
            out.push_back(static_cast<int>(i));
        }
    }
    return out;
}

MotorControlMode JointMotorTestRunner::parseControlMode(const std::string &raw)
{
    const std::string lower = toLower(raw);
    if (lower == "csp")
    {
        return MotorControlMode::kCsp;
    }
    if (lower == "cst")
    {
        return MotorControlMode::kCst;
    }
    if (lower == "r1")
    {
        return MotorControlMode::kR1;
    }
    throw std::runtime_error("unsupported control_mode: " + raw);
}

TrajectorySource JointMotorTestRunner::parseTrajectorySource(const std::string &raw)
{
    const std::string lower = toLower(raw);
    if (lower == "file")
    {
        return TrajectorySource::kFile;
    }
    if (lower == "sine")
    {
        return TrajectorySource::kSine;
    }
    throw std::runtime_error("unsupported trajectory_source: " + raw);
}

SineActivationMode JointMotorTestRunner::parseSineActivationMode(const std::string &raw)
{
    const std::string lower = toLower(raw);
    if (lower == "all")
    {
        return SineActivationMode::kAll;
    }
    if (lower == "sequential")
    {
        return SineActivationMode::kSequential;
    }
    throw std::runtime_error("unsupported sine.activation_mode: " + raw);
}

float JointMotorTestRunner::clampTorque(float value, float limit)
{
    const float safe_limit = std::max(0.0f, std::abs(limit));
    if (safe_limit <= 1e-6f)
    {
        return 0.0f;
    }
    return std::clamp(value, -safe_limit, safe_limit);
}

} // namespace joint_motor_test
