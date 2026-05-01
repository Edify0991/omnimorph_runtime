#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::setupRosInterfaces()
{
    state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        rl_master::dds::kTopicRobotState,
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    if (enable_python_viewer_stream_)
    {
        viewer_frame_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            viewer_frame_topic_,
            rclcpp::QoS(rclcpp::KeepLast(2)).best_effort());
    }
    if (enable_python_viewer_inspector_)
    {
        viewer_inspector_pub_ = this->create_publisher<std_msgs::msg::String>(
            viewer_inspector_topic_,
            rclcpp::QoS(rclcpp::KeepLast(5)).best_effort());
    }

    const auto period = std::chrono::duration<double>(1.0 / control_hz_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() { this->controlLoopTick(); });
}

void MujocoSimBridge::startInputExecutor()
{
    stopInputExecutor();

    input_node_ = std::make_shared<rclcpp::Node>("mujoco_sim_bridge_io");
    input_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    teleop_sub_ = input_node_->create_subscription<geometry_msgs::msg::Twist>(
        rl_master::dds::kTopicTeleopCommand,
        rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->teleopCallback(msg);
        });

    mode_control_sub_ = input_node_->create_subscription<std_msgs::msg::Int32>(
        rl_master::dds::kTopicModeControl,
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            this->modeControlCallback(msg);
        });

    io_stop_requested_.store(false);
    input_executor_->add_node(input_node_);
    input_executor_thread_ = std::thread([this]() {
        try
        {
            if (input_executor_)
            {
                input_executor_->spin();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "MuJoCo input executor exception: %s", e.what());
        }
    });
}

void MujocoSimBridge::stopInputExecutor()
{
    io_stop_requested_.store(true);
    telemetry_cv_.notify_all();

    if (input_executor_)
    {
        input_executor_->cancel();
    }
    if (input_executor_thread_.joinable())
    {
        input_executor_thread_.join();
    }
    if (input_executor_ && input_node_)
    {
        input_executor_->remove_node(input_node_);
    }
    mode_control_sub_.reset();
    teleop_sub_.reset();
    input_executor_.reset();
    input_node_.reset();
}

void MujocoSimBridge::startStateTelemetry()
{
    stopStateTelemetry();
    io_stop_requested_.store(false);
    state_telemetry_thread_ = std::thread([this]() {
        while (!io_stop_requested_.load())
        {
            rl_master::RobotStateData state;
            bool enabled = false;
            double publish_hz = 0.0;
            bool has_state = false;

            {
                std::unique_lock<std::mutex> lock(telemetry_mutex_);
                enabled = enable_state_telemetry_;
                publish_hz = state_telemetry_hz_;
                has_state = has_mirrored_state_;
                if (!enabled || publish_hz <= 0.0 || !has_state)
                {
                    telemetry_cv_.wait_for(
                        lock,
                        std::chrono::milliseconds(100),
                        [this]() {
                            return io_stop_requested_.load() ||
                                   (enable_state_telemetry_ && state_telemetry_hz_ > 0.0 && has_mirrored_state_);
                        });
                    continue;
                }
                state = latest_mirrored_state_;
            }

            publishRobotState(state);

            std::unique_lock<std::mutex> lock(telemetry_mutex_);
            telemetry_cv_.wait_for(
                lock,
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / publish_hz)),
                [this]() { return io_stop_requested_.load(); });
        }
    });
}

void MujocoSimBridge::stopStateTelemetry()
{
    io_stop_requested_.store(true);
    telemetry_cv_.notify_all();
    if (state_telemetry_thread_.joinable())
    {
        state_telemetry_thread_.join();
    }
}

void MujocoSimBridge::startViewerTelemetry()
{
    stopViewerTelemetry();
    io_stop_requested_.store(false);
    viewer_telemetry_thread_ = std::thread([this]() {
        auto last_frame_pub = std::chrono::steady_clock::time_point{};
        auto last_inspector_pub = std::chrono::steady_clock::time_point{};

        while (!io_stop_requested_.load())
        {
            std::vector<float> qpos;
            std::vector<float> qvel;
            std::vector<float> ctrl;
            float sim_time = 0.0f;
            std::string inspector_text;
            bool has_frame = false;
            bool has_inspector = false;

            {
                std::unique_lock<std::mutex> lock(viewer_telemetry_mutex_);
                if ((!enable_python_viewer_stream_ || !has_viewer_frame_) &&
                    (!enable_python_viewer_inspector_ || !has_viewer_inspector_))
                {
                    viewer_telemetry_cv_.wait_for(
                        lock,
                        std::chrono::milliseconds(100),
                        [this]() {
                            return io_stop_requested_.load() ||
                                   (enable_python_viewer_stream_ && has_viewer_frame_) ||
                                   (enable_python_viewer_inspector_ && has_viewer_inspector_);
                        });
                    continue;
                }

                has_frame = has_viewer_frame_;
                has_inspector = has_viewer_inspector_;
                if (has_frame)
                {
                    qpos = latest_viewer_qpos_;
                    qvel = latest_viewer_qvel_;
                    ctrl = latest_viewer_ctrl_;
                    sim_time = latest_viewer_sim_time_;
                }
                if (has_inspector)
                {
                    inspector_text = latest_viewer_inspector_text_;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (enable_python_viewer_stream_ && has_frame)
            {
                const auto frame_period = std::chrono::duration<double>(1.0 / std::max(1.0, viewer_fps_));
                if (last_frame_pub.time_since_epoch().count() == 0 ||
                    (now - last_frame_pub) >= frame_period)
                {
                    publishViewerFrameMirror(qpos, qvel, ctrl, sim_time);
                    last_frame_pub = now;
                }
            }

            if (enable_python_viewer_inspector_ && has_inspector)
            {
                const auto inspector_period = std::chrono::duration<double>(1.0 / std::max(0.1, viewer_inspector_hz_));
                if (last_inspector_pub.time_since_epoch().count() == 0 ||
                    (now - last_inspector_pub) >= inspector_period)
                {
                    publishViewerInspectorText(inspector_text);
                    last_inspector_pub = now;
                }
            }

            std::unique_lock<std::mutex> lock(viewer_telemetry_mutex_);
            viewer_telemetry_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() { return io_stop_requested_.load(); });
        }
    });
}

void MujocoSimBridge::stopViewerTelemetry()
{
    io_stop_requested_.store(true);
    viewer_telemetry_cv_.notify_all();
    if (viewer_telemetry_thread_.joinable())
    {
        viewer_telemetry_thread_.join();
    }
}

void MujocoSimBridge::updateMirroredState(const rl_master::RobotStateData &state)
{
    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(telemetry_mutex_);
        should_notify = !has_mirrored_state_;
        latest_mirrored_state_ = state;
        has_mirrored_state_ = true;
    }
    if (should_notify)
    {
        telemetry_cv_.notify_all();
    }
}

void MujocoSimBridge::initRuntimeRecorder()
{
    runtime_logging_enabled_ = false;
    const Sim2realCfg &runtime_cfg = controller_runtime_.runtimeCfg();
    if (!runtime_cfg.logging.enabled)
    {
        return;
    }

    std::ostringstream snapshot;
    snapshot << "{"
             << "\"backend\":\"sim2sim\","
             << "\"config_section\":\"" << controller_runtime_.activeConfigSection() << "\","
             << "\"mode_id\":" << controller_runtime_.activeModeId() << ","
             << "\"policy_name\":\"" << runtime_cfg.policy_name << "\","
             << "\"policy_family\":\"" << runtime_cfg.policy_family << "\","
             << "\"model_path\":\"" << model_path_ << "\","
             << "\"control_hz\":" << control_hz_ << ","
             << "\"policy_hz\":" << runtime_cfg.RL_control_f << ","
             << "\"enable_fixed_base_zeroing\":" << (enable_fixed_base_zeroing_ ? "true" : "false") << ","
             << "\"enable_fixed_base_hold_after_zeroing\":" << (enable_fixed_base_hold_after_zeroing_ ? "true" : "false") << ","
             << "\"enable_release_before_running\":" << (enable_release_before_running_ ? "true" : "false") << ","
             << "\"post_release_settle_ticks\":" << post_release_settle_ticks_ << ","
             << "\"post_zeroing_hold_settle_ticks\":" << post_zeroing_hold_settle_ticks_ << ","
             << "\"enable_prepose_snap\":" << (enable_prepose_snap_ ? "true" : "false") << ","
             << "\"sim_only_force_policy_csp\":" << (sim_only_force_policy_csp_ ? "true" : "false") << ","
             << "\"sim_dt\":" << sim_dt_
             << "}";

    std::map<std::string, std::string> session_metadata;
    session_metadata["backend"] = "sim2sim_mujoco";
    session_metadata["policy_name"] = runtime_cfg.policy_name;
    session_metadata["active_config_section"] = controller_runtime_.activeConfigSection();
    session_metadata["active_mode_id"] = std::to_string(controller_runtime_.activeModeId());
    session_metadata["model_path"] = model_path_;
    session_metadata["output_file_path"] = runtime_cfg.logging.output_file_path;

    if (!runtime_recorder_.open(runtime_cfg.logging, snapshot.str(), session_metadata))
    {
        RCLCPP_ERROR(this->get_logger(), "failed to open sim2sim runtime recorder");
        return;
    }

    runtime_logging_enabled_ = true;

    rl_master::logging::RuntimeEventRecord event;
    event.monotonic_time_sec = rl_master::monotonicTimeSec();
    event.event_type = "sim2sim_initialized";
    event.tags["model_path"] = model_path_;
    event.tags["mode_id"] = std::to_string(controller_runtime_.activeModeId());
    event.tags["config_section"] = controller_runtime_.activeConfigSection();
    event.tags["effective_compression"] = runtime_recorder_.effectiveCompression();
    runtime_recorder_.recordEvent(event);

    RCLCPP_INFO(
        this->get_logger(),
        "Runtime MCAP log: %s",
        runtime_recorder_.filePath().c_str());
}

void MujocoSimBridge::emitDerivedRuntimeEvents(const rl_master::logging::ControllerLogSnapshot &controller_snapshot)
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

void MujocoSimBridge::emitBaseImuSourceSample(const rl_master::RobotStateData &state, double monotonic_time_sec)
{
    const Sim2realCfg &runtime_cfg = controller_runtime_.runtimeCfg();
    if (!runtime_logging_enabled_ ||
        !runtime_recorder_.isOpen() ||
        !runtime_cfg.logging.source_samples.enabled ||
        !runtime_cfg.logging.source_samples.include_base_imu)
    {
        return;
    }

    rl_master::logging::RuntimeSourceSampleRecord sample;
    sample.monotonic_time_sec = monotonic_time_sec;
    sample.topic = "runtime/source/base_imu";
    sample.sample_name = "base_imu";
    sample.tags["backend"] = "sim2sim";
    sample.values["ang_vel"] = {
        state.base_ang_vel[0],
        state.base_ang_vel[1],
        state.base_ang_vel[2]};
    sample.values["quat_xyzw"] = {
        state.base_quat[0],
        state.base_quat[1],
        state.base_quat[2],
        state.base_quat[3]};
    sample.values["rpy"] = {
        state.base_rpy[0],
        state.base_rpy[1],
        state.base_rpy[2]};
    runtime_recorder_.recordSourceSample(sample);
}

void MujocoSimBridge::logLoopData(
    const rl_master::RobotStateData &state,
    const rl_master::RobotStateData &post_state,
    const rl_master::RobotCommandData &command,
    const rl_master::logging::ControllerLogSnapshot &controller_snapshot,
    const rl_master::CommandRuntimeDecision &runtime_mode,
    bool control_active)
{
    (void)state;
    (void)command;
    (void)runtime_mode;
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
    record.latest_cmd_fresh = control_active;
    record.loop_overrun_count = sim_loop_overrun_count_;
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

    record.joint_cmd_q = joint_cmd_q_;
    record.joint_cmd_dq = joint_cmd_dq_;
    record.joint_cmd_tau = joint_cmd_tau_;
    record.joint_state_q = post_state.joint_q;
    record.joint_state_dq = post_state.joint_dq;
    record.joint_state_tau = post_state.joint_tau;
    record.motor_cmd_q = joint_cmd_q_;
    record.motor_cmd_dq = joint_cmd_dq_;
    record.motor_cmd_tau = applied_tau_;
    record.motor_state_q = post_state.joint_q;
    record.motor_state_dq = post_state.joint_dq;
    record.motor_state_tau = applied_tau_;
    record.motor_cmd_mode = joint_cmd_mode_;

    runtime_recorder_.recordTick(record);

    if (controller_snapshot.policy_ran_this_tick && !controller_snapshot.policy_action.empty())
    {
        rl_master::logging::RuntimeSourceSampleRecord sample;
        sample.monotonic_time_sec = record.monotonic_time_sec;
        sample.topic = "runtime/source/policy_action";
        sample.sample_name = "policy_action";
        sample.tags["backend"] = "sim2sim";
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
        sample.tags["backend"] = "sim2sim";
        sample.tags["mode_id"] = std::to_string(record.active_mode_id);
        sample.tags["policy_step_index"] = std::to_string(record.policy_step_index);
        sample.values["observation"] = controller_snapshot.observation;
        runtime_recorder_.recordSourceSample(sample);
    }

    if (controller_runtime_.runtimeCfg().logging.source_samples.include_external_observations)
    {
        for (auto &sample : controller_runtime_.controller().drainExternalObservationSamplesForLogging())
        {
            sample.tags["backend"] = "sim2sim";
            runtime_recorder_.recordSourceSample(sample);
        }
    }
}

void MujocoSimBridge::updateViewerFrameMirror()
{
    if (!enable_python_viewer_stream_)
    {
        return;
    }

    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(viewer_telemetry_mutex_);
        should_notify = !has_viewer_frame_;
        latest_viewer_qpos_.assign(model_->nq, 0.0f);
        latest_viewer_qvel_.assign(model_->nv, 0.0f);
        latest_viewer_ctrl_.assign(model_->nu, 0.0f);
        for (int i = 0; i < model_->nq; ++i)
        {
            latest_viewer_qpos_[i] = static_cast<float>(data_->qpos[i]);
        }
        for (int i = 0; i < model_->nv; ++i)
        {
            latest_viewer_qvel_[i] = static_cast<float>(data_->qvel[i]);
        }
        for (int i = 0; i < model_->nu; ++i)
        {
            latest_viewer_ctrl_[i] = static_cast<float>(data_->ctrl[i]);
        }
        latest_viewer_sim_time_ = static_cast<float>(data_->time);
        has_viewer_frame_ = true;
    }
    if (should_notify)
    {
        viewer_telemetry_cv_.notify_all();
    }
}

void MujocoSimBridge::updateViewerInspectorMirror(
    const rl_master::RobotStateData &state,
    const rl_master::RobotCommandData &command,
    const rl_master::CommandRuntimeDecision &runtime_mode)
{
    if (!enable_python_viewer_inspector_)
    {
        return;
    }

    double mean_abs_joint_error = 0.0;
    double max_abs_joint_error = 0.0;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
        const double err = std::abs(static_cast<double>(last_target_q_[i]) - static_cast<double>(state.joint_q[i]));
        mean_abs_joint_error += err;
        max_abs_joint_error = std::max(max_abs_joint_error, err);
    }
    if (!joint_names_.empty())
    {
        mean_abs_joint_error /= static_cast<double>(joint_names_.size());
    }

    std::ostringstream oss;
    const rl_master::TeleopCommand teleop_command = latestTeleopCommand();
    oss << std::fixed << std::setprecision(3)
        << "mode_id=" << controller_runtime_.activeModeId()
        << " section=" << controller_runtime_.activeConfigSection()
        << " runtime=" << rl_master::commandRuntimeModeName(runtime_mode.mode)
        << " active=" << (runtime_mode.open_rl_active ? "1" : "0")
        << " open_rl=" << static_cast<double>(command.open_rl)
        << " sim_t=" << static_cast<double>(data_->time)
        << " teleop=("
        << static_cast<double>(teleop_command.vx) << ","
        << static_cast<double>(teleop_command.vy) << ","
        << static_cast<double>(teleop_command.dyaw) << ")"
        << " base_rpy=("
        << static_cast<double>(state.base_rpy[0]) << ","
        << static_cast<double>(state.base_rpy[1]) << ","
        << static_cast<double>(state.base_rpy[2]) << ")"
        << " qerr_mean=" << mean_abs_joint_error
        << " qerr_max=" << max_abs_joint_error
        << " paused=" << (viewer_paused_ ? "1" : "0");

    bool should_notify = false;
    {
        std::lock_guard<std::mutex> lock(viewer_telemetry_mutex_);
        should_notify = !has_viewer_inspector_;
        latest_viewer_inspector_text_ = oss.str();
        has_viewer_inspector_ = true;
    }
    if (should_notify)
    {
        viewer_telemetry_cv_.notify_all();
    }
}

rl_master::TeleopCommand MujocoSimBridge::latestTeleopCommand() const
{
    std::lock_guard<std::mutex> lock(teleop_mutex_);
    return latest_teleop_command_;
}

} // namespace mujoco_sim2sim
