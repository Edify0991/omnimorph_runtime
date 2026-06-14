#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

MujocoSimBridge::MujocoSimBridge()
    : rclcpp::Node("mujoco_sim_bridge")
{
    fixed_base_qpos_.fill(0.0);
    this->declare_parameter<std::string>("rl_cfg_path", RL_CFG_PATH);
    rl_cfg_path_ = trimCopy(this->get_parameter("rl_cfg_path").as_string());
    if (rl_cfg_path_.empty())
    {
        rl_cfg_path_ = RL_CFG_PATH;
    }

    mode_registry_ = rl_master::ModeProfileRegistry::loadFromYaml(rl_cfg_path_, "engineai_walk");

    loadParameters();
    loadModel();
    resolveModelMappings();
    controller_runtime_.setModeProfileRegistry(mode_registry_);
    controller_runtime_.initialize(startup_mode_id_);
    initializeComSupportVisualization();
    initializeState();
    initRuntimeRecorder();
    mode_command_cache_.store(rl_master::kCtrlWordSetModeBase + startup_mode_id_);
    initializeViewer();
    initializeVideoRecorder();
    setupRosInterfaces();
    startInputExecutor();
    startStateTelemetry();
    startViewerTelemetry();

    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo sim2sim fused runtime ready. model='%s', control_hz=%.1f, sim_dt=%.6f, substeps=%d, startup_mode_id=%d, viewer=%s, python_viewer_stream=%s, python_viewer_inspector=%s, video_recording=%s, inactive_behavior=%s, state_telemetry=%s@%.1fHz, sim_base_quat_source_order=%s, sim_base_velocity_source=%s, fixed_base_zeroing=%s, fixed_base_hold=%s, release_before_running=%s, release_settle_ticks=%d, hold_settle_ticks=%d, prepose_snap=%s, running_start_ref_sync=%s, sim_only_force_policy_csp=%s",
        model_path_.c_str(),
        control_hz_,
        sim_dt_,
        substeps_per_control_,
        startup_mode_id_,
        enable_viewer_ ? "on" : "off",
        enable_python_viewer_stream_ ? viewer_frame_topic_.c_str() : "off",
        enable_python_viewer_inspector_ ? viewer_inspector_topic_.c_str() : "off",
        enable_video_recording_ ? video_output_path_.c_str() : "off",
        no_command_behavior_.c_str(),
        enable_state_telemetry_ ? "on" : "off",
        state_telemetry_hz_,
        controller_runtime_.runtimeCfg().source_contract.sim_base.quat_source_order.c_str(),
        controller_runtime_.runtimeCfg().source_contract.sim_base.velocity_source.c_str(),
        enable_fixed_base_zeroing_ ? "on" : "off",
        enable_fixed_base_hold_after_zeroing_ ? "on" : "off",
        enable_release_before_running_ ? "on" : "off",
        post_release_settle_ticks_,
        post_zeroing_hold_settle_ticks_,
        enable_prepose_snap_ ? "on" : "off",
        sim_sync_running_start_to_reference_ ? "on" : "off",
        sim_only_force_policy_csp_ ? "on" : "off");
    RCLCPP_INFO(
        this->get_logger(),
        "MuJoCo sim2sim RL root config: %s",
        rl_cfg_path_.c_str());
}

MujocoSimBridge::~MujocoSimBridge()
{
    control_timer_.reset();
    stopViewerTelemetry();
    stopStateTelemetry();
    stopInputExecutor();
    mode_control_sub_.reset();
    teleop_sub_.reset();
    state_pub_.reset();
    controller_runtime_.estop();
    runtime_recorder_.flush();
    runtime_recorder_.close();
    shutdownVideoRecorder();
    shutdownViewer();

    if (data_)
    {
        mj_deleteData(data_);
        data_ = nullptr;
    }
    if (model_)
    {
        mj_deleteModel(model_);
        model_ = nullptr;
    }
}

std::shared_ptr<rclcpp::Node> createMujocoSimBridgeNode()
{
    return std::make_shared<MujocoSimBridge>();
}

} // namespace mujoco_sim2sim
