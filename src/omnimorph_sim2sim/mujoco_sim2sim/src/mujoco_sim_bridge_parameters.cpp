#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::loadParameters()
{
    if (!mode_registry_)
    {
        throw std::runtime_error(
            "MuJoCo sim2sim requires a valid mode registry. "
            "Failed to load ModeProfileRegistry before parameter initialization.");
    }

    const std::vector<std::string> canonical_names = mode_registry_->jointOrder();
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<std::string>("base_body_name", "base_link");
    this->declare_parameter<std::string>("base_free_joint_name", "");
    this->declare_parameter<std::vector<std::string>>("joint_names", canonical_names);
    this->declare_parameter<std::vector<std::string>>("actuator_names", std::vector<std::string>{});
    this->declare_parameter<int>("startup_mode_id", rl_master::kModeCodeMin);
    this->declare_parameter<double>("control_hz", 50.0);
    this->declare_parameter<double>("sim_dt", 0.001);
    this->declare_parameter<bool>("use_command_torque_ff", false);
    this->declare_parameter<bool>("pause_when_no_command", false);
    this->declare_parameter<std::string>("no_command_behavior", "hold_position");
    this->declare_parameter<bool>("fix_base", false);
    this->declare_parameter<double>("fixed_base_height", -1.0);
    this->declare_parameter<bool>("enable_fixed_base_zeroing", false);
    this->declare_parameter<bool>("enable_fixed_base_hold_after_zeroing", false);
    this->declare_parameter<bool>("enable_release_before_running", false);
    this->declare_parameter<int>("post_release_settle_ticks", 0);
    this->declare_parameter<int>("post_zeroing_hold_settle_ticks", 0);
    this->declare_parameter<bool>("enable_prepose_snap", false);
    this->declare_parameter<bool>("sim_sync_running_start_to_reference", false);
    this->declare_parameter<bool>("enable_reference_pose_replay_test", false);
    this->declare_parameter<std::vector<double>>("prepose_joint_q", std::vector<double>{});
    this->declare_parameter<bool>("sim_only_force_policy_csp", false);
    this->declare_parameter<std::vector<std::string>>("joint_runtime_mode_overrides", std::vector<std::string>{});
    this->declare_parameter<bool>("enable_viewer", false);
    this->declare_parameter<bool>("enable_python_viewer_stream", false);
    this->declare_parameter<std::string>("viewer_frame_topic", kDefaultViewerFrameTopic);
    this->declare_parameter<bool>("enable_python_viewer_inspector", false);
    this->declare_parameter<std::string>("viewer_inspector_topic", "/omnimorph/sim2sim/mujoco_viewer_inspector");
    this->declare_parameter<double>("viewer_fps", 60.0);
    this->declare_parameter<double>("viewer_inspector_hz", 10.0);
    this->declare_parameter<int>("viewer_width", 1280);
    this->declare_parameter<int>("viewer_height", 720);
    this->declare_parameter<std::string>("viewer_title", "MuJoCo Sim2Sim Viewer");
    this->declare_parameter<bool>("enable_video_recording", false);
    this->declare_parameter<std::string>("video_output_dir", "/tmp/omnimorph_sim2sim_videos");
    this->declare_parameter<std::string>("video_output_name", "");
    this->declare_parameter<std::string>("video_ffmpeg_path", "ffmpeg");
    this->declare_parameter<double>("video_fps", 60.0);
    this->declare_parameter<int>("video_width", 1280);
    this->declare_parameter<int>("video_height", 720);
    this->declare_parameter<bool>("video_follow_robot", true);
    this->declare_parameter<double>("video_follow_distance", 3.0);
    this->declare_parameter<double>("video_follow_azimuth", 90.0);
    this->declare_parameter<double>("video_follow_elevation", -20.0);
    this->declare_parameter<std::vector<double>>("video_follow_lookat_offset", std::vector<double>{0.0, 0.0, 0.8});
    this->declare_parameter<bool>("enable_com_support_visualization", false);
    this->declare_parameter<std::string>("com_support_pinocchio_urdf_path", "");
    this->declare_parameter<std::vector<std::string>>(
        "support_foot_site_names",
        std::vector<std::string>{"right_foot_site", "left_foot_site"});
    this->declare_parameter<double>("support_foot_half_length", 0.11);
    this->declare_parameter<double>("support_foot_half_width", 0.055);
    this->declare_parameter<double>("support_contact_height_threshold", 0.05);
    this->declare_parameter<double>("com_marker_radius", 0.035);
    this->declare_parameter<double>("com_projection_marker_radius", 0.025);
    this->declare_parameter<double>("cop_marker_radius", 0.025);
    this->declare_parameter<bool>("enable_state_telemetry", true);
    this->declare_parameter<double>("state_telemetry_hz", 50.0);

    model_path_ = this->get_parameter("model_path").as_string();
    base_body_name_ = this->get_parameter("base_body_name").as_string();
    base_free_joint_name_ = this->get_parameter("base_free_joint_name").as_string();
    startup_mode_id_ = static_cast<int>(this->get_parameter("startup_mode_id").as_int());
    control_hz_ = std::max(1.0, this->get_parameter("control_hz").as_double());
    sim_dt_ = std::max(1e-5, this->get_parameter("sim_dt").as_double());
    use_command_torque_ff_ = this->get_parameter("use_command_torque_ff").as_bool();
    pause_when_no_command_ = this->get_parameter("pause_when_no_command").as_bool();
    no_command_behavior_ = normalizeNoCommandBehavior(this->get_parameter("no_command_behavior").as_string());
    fix_base_ = this->get_parameter("fix_base").as_bool();
    fixed_base_height_ = this->get_parameter("fixed_base_height").as_double();
    enable_fixed_base_zeroing_ = this->get_parameter("enable_fixed_base_zeroing").as_bool();
    enable_fixed_base_hold_after_zeroing_ = this->get_parameter("enable_fixed_base_hold_after_zeroing").as_bool();
    enable_release_before_running_ = this->get_parameter("enable_release_before_running").as_bool();
    post_release_settle_ticks_ = std::max<int>(0, static_cast<int>(this->get_parameter("post_release_settle_ticks").as_int()));
    post_zeroing_hold_settle_ticks_ = std::max<int>(
        0,
        static_cast<int>(this->get_parameter("post_zeroing_hold_settle_ticks").as_int()));
    enable_prepose_snap_ = this->get_parameter("enable_prepose_snap").as_bool();
    sim_sync_running_start_to_reference_ = this->get_parameter("sim_sync_running_start_to_reference").as_bool();
    enable_reference_pose_replay_test_ = this->get_parameter("enable_reference_pose_replay_test").as_bool();
    prepose_joint_q_ = this->get_parameter("prepose_joint_q").as_double_array();
    sim_only_force_policy_csp_ = this->get_parameter("sim_only_force_policy_csp").as_bool();
    joint_runtime_mode_override_entries_ = this->get_parameter("joint_runtime_mode_overrides").as_string_array();
    enable_viewer_ = this->get_parameter("enable_viewer").as_bool();
    enable_python_viewer_stream_ = this->get_parameter("enable_python_viewer_stream").as_bool();
    viewer_frame_topic_ = this->get_parameter("viewer_frame_topic").as_string();
    if (viewer_frame_topic_.empty())
    {
        viewer_frame_topic_ = kDefaultViewerFrameTopic;
    }
    enable_python_viewer_inspector_ = this->get_parameter("enable_python_viewer_inspector").as_bool();
    viewer_inspector_topic_ = this->get_parameter("viewer_inspector_topic").as_string();
    if (viewer_inspector_topic_.empty())
    {
        viewer_inspector_topic_ = "/omnimorph/sim2sim/mujoco_viewer_inspector";
    }
    viewer_fps_ = std::max(1.0, this->get_parameter("viewer_fps").as_double());
    viewer_inspector_hz_ = std::max(0.1, this->get_parameter("viewer_inspector_hz").as_double());
    const int64_t viewer_width_param = this->get_parameter("viewer_width").as_int();
    const int64_t viewer_height_param = this->get_parameter("viewer_height").as_int();
    viewer_width_ = static_cast<int>(std::clamp<int64_t>(viewer_width_param, 320, 8192));
    viewer_height_ = static_cast<int>(std::clamp<int64_t>(viewer_height_param, 240, 8192));
    viewer_title_ = this->get_parameter("viewer_title").as_string();
    enable_video_recording_ = this->get_parameter("enable_video_recording").as_bool();
    video_output_dir_ = trimCopy(this->get_parameter("video_output_dir").as_string());
    if (video_output_dir_.empty())
    {
        video_output_dir_ = "/tmp/omnimorph_sim2sim_videos";
    }
    video_output_name_ = trimCopy(this->get_parameter("video_output_name").as_string());
    video_ffmpeg_path_ = trimCopy(this->get_parameter("video_ffmpeg_path").as_string());
    if (video_ffmpeg_path_.empty())
    {
        video_ffmpeg_path_ = "ffmpeg";
    }
    video_fps_ = std::max(1.0, this->get_parameter("video_fps").as_double());
    const int64_t video_width_param = this->get_parameter("video_width").as_int();
    const int64_t video_height_param = this->get_parameter("video_height").as_int();
    video_width_ = static_cast<int>(std::clamp<int64_t>(video_width_param, 64, 8192));
    video_height_ = static_cast<int>(std::clamp<int64_t>(video_height_param, 64, 8192));
    video_width_ = std::max(64, video_width_ - (video_width_ % 2));
    video_height_ = std::max(64, video_height_ - (video_height_ % 2));
    video_follow_robot_ = this->get_parameter("video_follow_robot").as_bool();
    video_follow_distance_ = std::max(0.1, this->get_parameter("video_follow_distance").as_double());
    video_follow_azimuth_ = this->get_parameter("video_follow_azimuth").as_double();
    video_follow_elevation_ = this->get_parameter("video_follow_elevation").as_double();
    const std::vector<double> video_lookat_offset =
        this->get_parameter("video_follow_lookat_offset").as_double_array();
    if (video_lookat_offset.size() >= 3)
    {
        video_follow_lookat_offset_ = {video_lookat_offset[0], video_lookat_offset[1], video_lookat_offset[2]};
    }
    enable_com_support_visualization_ = this->get_parameter("enable_com_support_visualization").as_bool();
    com_support_pinocchio_urdf_path_ =
        trimCopy(this->get_parameter("com_support_pinocchio_urdf_path").as_string());
    support_foot_site_names_ = this->get_parameter("support_foot_site_names").as_string_array();
    if (support_foot_site_names_.empty())
    {
        support_foot_site_names_ = {"right_foot_site", "left_foot_site"};
    }
    support_foot_half_length_ = std::max(0.001, this->get_parameter("support_foot_half_length").as_double());
    support_foot_half_width_ = std::max(0.001, this->get_parameter("support_foot_half_width").as_double());
    support_contact_height_threshold_ =
        std::max(0.0, this->get_parameter("support_contact_height_threshold").as_double());
    com_marker_radius_ = std::max(0.001, this->get_parameter("com_marker_radius").as_double());
    com_projection_marker_radius_ =
        std::max(0.001, this->get_parameter("com_projection_marker_radius").as_double());
    cop_marker_radius_ = std::max(0.001, this->get_parameter("cop_marker_radius").as_double());
    enable_state_telemetry_ = this->get_parameter("enable_state_telemetry").as_bool();
    state_telemetry_hz_ = std::max(0.0, this->get_parameter("state_telemetry_hz").as_double());

    if (!model_path_.empty())
    {
        try
        {
            const RootConfigDocument root_doc = loadRootConfigDocument(rl_cfg_path_);
            const std::map<std::string, std::string> path_variables =
                loadPathVariablesFromRootDocument(root_doc);
            const std::string expanded_model_path =
                expandPathVariables(model_path_, path_variables, true);
            std::filesystem::path resolved_model_path = std::filesystem::path(expanded_model_path);
            if (resolved_model_path.is_relative())
            {
                const std::filesystem::path resolved_root_dir =
                    resolveConfiguredOmnimorphRootDir(root_doc.root, root_doc.root_dir);
                resolved_model_path = resolved_root_dir / resolved_model_path;
            }
            model_path_ = std::filesystem::absolute(resolved_model_path).lexically_normal().string();
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error(
                "failed to resolve MuJoCo model_path '" + model_path_ +
                "' using rl_cfg '" + rl_cfg_path_ + "': " + e.what());
        }
    }

    joint_names_ = canonical_names;

    std::vector<std::string> actuator_names_param;
    const auto actuator_names_param_obj = this->get_parameter("actuator_names");
    if (actuator_names_param_obj.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
    {
        actuator_names_param = actuator_names_param_obj.as_string_array();
    }
    if (actuator_names_param.empty())
    {
        throw std::runtime_error(
            "actuator_names must be configured explicitly. "
            "Implicit fallback from actuator_names to joint_names has been disabled for strict debugging.");
    }
    if (actuator_names_param.size() != joint_names_.size())
    {
        throw std::runtime_error("actuator_names vector size mismatch. Expect " + std::to_string(joint_names_.size()));
    }
    actuator_names_ = actuator_names_param;

    if (!prepose_joint_q_.empty() &&
        prepose_joint_q_.size() != 1 &&
        prepose_joint_q_.size() != joint_names_.size())
    {
        throw std::runtime_error("prepose_joint_q size must be 1 or match joint_names");
    }

    auto loadRequiredNamedJointParams =
        [this](
            const std::string &prefix,
            const std::vector<std::string> &joint_names,
            bool absolute_value) -> std::vector<double> {
            std::vector<double> values(joint_names.size(), std::numeric_limits<double>::quiet_NaN());
            for (size_t i = 0; i < joint_names.size(); ++i)
            {
                const std::string param_name = prefix + "." + joint_names[i];
                this->declare_parameter<double>(
                    param_name,
                    std::numeric_limits<double>::quiet_NaN());
                const double raw_value = this->get_parameter(param_name).as_double();
                if (!std::isfinite(raw_value))
                {
                    throw std::runtime_error(
                        "parameter set '" + prefix +
                        "' must be configured using per-joint name/value entries; missing joint '" +
                        joint_names[i] + "'");
                }
                values[i] = absolute_value ? std::abs(raw_value) : raw_value;
            }
            return values;
        };

    position_controlled_joint_names_.clear();
    position_actuator_joint_indices_.clear();
    applied_position_actuator_kp_.clear();
    applied_position_actuator_kv_.clear();
    applied_position_actuator_forcerange_.clear();

    joint_ids_.assign(joint_names_.size(), -1);
    qpos_addrs_.assign(joint_names_.size(), -1);
    qvel_addrs_.assign(joint_names_.size(), -1);
    actuator_ids_.assign(joint_names_.size(), -1);
    applied_tau_.assign(joint_names_.size(), 0.0f);
    last_target_q_.assign(joint_names_.size(), 0.0f);
    hold_kp_ = loadRequiredNamedJointParams("hold_kp", joint_names_, false);
    hold_kd_ = loadRequiredNamedJointParams("hold_kd", joint_names_, false);
    hold_torque_limit_ = loadRequiredNamedJointParams("hold_torque_limit", joint_names_, true);
    if (model_path_.empty())
    {
        throw std::runtime_error(
            "Parameter 'model_path' is empty. Please set a MuJoCo xml/mjb file path.");
    }
}

} // namespace mujoco_sim2sim
