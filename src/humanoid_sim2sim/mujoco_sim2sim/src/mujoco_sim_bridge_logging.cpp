#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{

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

} // namespace mujoco_sim2sim
