#ifndef RL_CFG_H
#define RL_CFG_H

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>
#include "rl_master/runtime/realtime_utils.h"

template <typename T>
inline T yamlReadOr(const YAML::Node &node, const char *key, const T &default_value)
{
    if (!node || !node[key])
    {
        return default_value;
    }
    return node[key].as<T>();
}

inline std::map<std::string, std::string> yamlReadStringMapOr(
    const YAML::Node &node,
    const char *key)
{
    std::map<std::string, std::string> out;
    if (!node || !node[key])
    {
        return out;
    }
    const YAML::Node map_node = node[key];
    if (!map_node.IsMap())
    {
        throw std::runtime_error(std::string("yaml key is not a map: ") + key);
    }
    for (auto it = map_node.begin(); it != map_node.end(); ++it)
    {
        out[it->first.as<std::string>()] = it->second.as<std::string>();
    }
    return out;
}

inline std::map<std::string, float> yamlReadFloatMapOr(
    const YAML::Node &node,
    const char *key)
{
    std::map<std::string, float> out;
    if (!node || !node[key])
    {
        return out;
    }
    const YAML::Node map_node = node[key];
    if (!map_node.IsMap())
    {
        throw std::runtime_error(std::string("yaml key is not a map: ") + key);
    }
    for (auto it = map_node.begin(); it != map_node.end(); ++it)
    {
        out[it->first.as<std::string>()] = it->second.as<float>();
    }
    return out;
}

inline std::string getCurrentTime()
{
    const auto t = std::time(nullptr);
    const auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%b%d_%H-%M-%S");
    return oss.str();
}

inline rl_master::runtime::RealtimeConfig parseRealtimeConfigNode(
    const YAML::Node &node,
    const rl_master::runtime::RealtimeConfig &defaults = {})
{
    rl_master::runtime::RealtimeConfig config = defaults;
    if (!node)
    {
        return config;
    }
    config.enabled = yamlReadOr<bool>(node, "enabled", config.enabled);
    config.lock_memory = yamlReadOr<bool>(node, "lock_memory", config.lock_memory);
    config.set_affinity = yamlReadOr<bool>(node, "set_affinity", config.set_affinity);
    config.cpu_id = yamlReadOr<int>(node, "cpu_id", config.cpu_id);
    config.use_fifo_scheduler = yamlReadOr<bool>(node, "use_fifo", config.use_fifo_scheduler);
    config.fifo_priority = yamlReadOr<int>(node, "fifo_priority", config.fifo_priority);
    return config;
}

struct DeployModeProfileSpec
{
    int mode_id = 0;
    std::string config_section;
    std::string tag;
};

struct JointGroupsConfig
{
    std::vector<std::string> leg;
    std::vector<std::string> arm;
    std::vector<std::string> waist;
};

struct RootConfigDocument
{
    std::filesystem::path root_path;
    std::filesystem::path root_dir;
    YAML::Node root;
};

struct ProfileConfigDocument
{
    RootConfigDocument root_doc;
    std::filesystem::path profile_path;
    YAML::Node profile_root;
    YAML::Node section;
};

inline RootConfigDocument loadRootConfigDocument(const std::string &yaml_file)
{
    RootConfigDocument out;
    out.root_path = std::filesystem::absolute(std::filesystem::path(yaml_file));
    out.root_dir = out.root_path.parent_path();
    out.root = YAML::LoadFile(out.root_path.string());
    if (!out.root || !out.root.IsMap())
    {
        throw std::runtime_error("top-level YAML must be a map: " + out.root_path.string());
    }
    return out;
}

inline std::map<std::string, std::string> loadConfigFileMapFromRoot(const YAML::Node &root)
{
    if (!root || !root["config_files"])
    {
        throw std::runtime_error("config_files is required in root rl config");
    }
    const YAML::Node config_files = root["config_files"];
    if (!config_files.IsMap())
    {
        throw std::runtime_error("config_files must be a map");
    }

    std::map<std::string, std::string> out;
    for (auto it = config_files.begin(); it != config_files.end(); ++it)
    {
        const std::string section = it->first.as<std::string>();
        const std::string raw_path = it->second.as<std::string>();
        if (section.empty())
        {
            throw std::runtime_error("config_files contains empty config_section key");
        }
        if (raw_path.empty())
        {
            throw std::runtime_error("config_files entry has empty path for section: " + section);
        }
        out[section] = raw_path;
    }
    return out;
}

inline std::filesystem::path resolveConfigSectionPath(
    const RootConfigDocument &root_doc,
    const std::string &config_section)
{
    const auto config_files = loadConfigFileMapFromRoot(root_doc.root);
    std::map<std::string, std::string> resolved_path_to_section;
    std::filesystem::path profile_path;
    bool found = false;
    for (const auto &entry : config_files)
    {
        std::filesystem::path resolved_path = std::filesystem::path(entry.second);
        if (resolved_path.is_relative())
        {
            resolved_path = root_doc.root_dir / resolved_path;
        }
        resolved_path = std::filesystem::absolute(resolved_path);

        const std::string resolved_key = resolved_path.lexically_normal().string();
        const auto duplicate_it = resolved_path_to_section.find(resolved_key);
        if (duplicate_it != resolved_path_to_section.end() && duplicate_it->second != entry.first)
        {
            throw std::runtime_error(
                "config_files maps both '" + duplicate_it->second + "' and '" + entry.first +
                "' to the same profile file: " + resolved_key);
        }
        resolved_path_to_section[resolved_key] = entry.first;

        if (entry.first == config_section)
        {
            profile_path = resolved_path;
            found = true;
        }
    }
    if (!found)
    {
        throw std::runtime_error("config_files is missing mapping for config section: " + config_section);
    }
    if (!std::filesystem::exists(profile_path))
    {
        throw std::runtime_error(
            "profile file not found for config section '" + config_section + "': " +
            profile_path.string());
    }
    return profile_path;
}

inline ProfileConfigDocument loadProfileConfigDocument(
    const std::string &yaml_file,
    const std::string &config_section)
{
    ProfileConfigDocument out;
    out.root_doc = loadRootConfigDocument(yaml_file);
    out.profile_path = resolveConfigSectionPath(out.root_doc, config_section);
    out.profile_root = YAML::LoadFile(out.profile_path.string());
    if (!out.profile_root || !out.profile_root.IsMap())
    {
        throw std::runtime_error("profile YAML must be a map: " + out.profile_path.string());
    }
    if (out.profile_root.size() != 1 || !out.profile_root[config_section])
    {
        throw std::runtime_error(
            "profile file must contain exactly one top-level section named '" +
            config_section + "': " + out.profile_path.string());
    }
    out.section = out.profile_root[config_section];
    if (!out.section || !out.section.IsMap())
    {
        throw std::runtime_error(
            "profile section must be a map for config section '" + config_section +
            "': " + out.profile_path.string());
    }
    return out;
}

inline bool loadProcessRealtimeConfigFromYAML(
    const std::string &yaml_file,
    const std::string &process_name,
    rl_master::runtime::RealtimeConfig *out,
    const std::string &group_name = "runtime_process")
{
    if (!out || process_name.empty())
    {
        return false;
    }

    try
    {
        const YAML::Node config = loadRootConfigDocument(yaml_file).root;
        const YAML::Node process_node = config[group_name] ? config[group_name][process_name] : YAML::Node();
        if (!process_node)
        {
            return false;
        }
        *out = parseRealtimeConfigNode(process_node, *out);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

inline std::vector<DeployModeProfileSpec> loadDeployModeProfilesFromYAML(
    const std::string &yaml_file)
{
    std::vector<DeployModeProfileSpec> specs;
    const YAML::Node root = loadRootConfigDocument(yaml_file).root;
    const YAML::Node profiles = root["deploy_mode_profiles"];
    if (!profiles || !profiles.IsSequence())
    {
        return specs;
    }

    specs.reserve(profiles.size());
    for (size_t i = 0; i < profiles.size(); ++i)
    {
        const YAML::Node node = profiles[i];
        if (!node["mode_id"] || !node["config_section"])
        {
            throw std::runtime_error(
                "deploy_mode_profiles[" + std::to_string(i) + "] requires mode_id and config_section");
        }
        DeployModeProfileSpec spec;
        spec.mode_id = node["mode_id"].as<int>();
        spec.config_section = node["config_section"].as<std::string>();
        spec.tag = yamlReadOr<std::string>(node, "tag", spec.config_section);
        specs.push_back(spec);
    }
    return specs;
}

inline std::vector<std::string> loadRobotGlobalJointOrderFromYAML(
    const std::string &yaml_file)
{
    const YAML::Node root = loadRootConfigDocument(yaml_file).root;
    const YAML::Node joint_order = root["robot_global_joint_order"];
    if (!joint_order)
    {
        throw std::runtime_error("robot_global_joint_order is required");
    }
    if (!joint_order.IsSequence())
    {
        throw std::runtime_error("robot_global_joint_order must be a sequence");
    }

    std::vector<std::string> out;
    std::unordered_set<std::string> seen;
    out.reserve(joint_order.size());
    seen.reserve(joint_order.size());
    for (size_t i = 0; i < joint_order.size(); ++i)
    {
        const std::string name = joint_order[i].as<std::string>();
        if (name.empty())
        {
            throw std::runtime_error("robot_global_joint_order contains empty joint name");
        }
        if (!seen.insert(name).second)
        {
            throw std::runtime_error("robot_global_joint_order contains duplicate joint: " + name);
        }
        out.push_back(name);
    }
    return out;
}

inline std::vector<std::string> loadJointGroupNamesFromRoot(
    const YAML::Node &joint_groups,
    const char *group_name,
    bool required)
{
    if (!group_name)
    {
        return {};
    }
    const YAML::Node group_node = joint_groups ? joint_groups[group_name] : YAML::Node();
    if (!group_node)
    {
        if (required)
        {
            throw std::runtime_error(std::string("joint_groups.") + group_name + " is required");
        }
        return {};
    }
    if (!group_node.IsSequence())
    {
        throw std::runtime_error(std::string("joint_groups.") + group_name + " must be a sequence");
    }

    std::vector<std::string> out;
    std::unordered_set<std::string> seen;
    out.reserve(group_node.size());
    seen.reserve(group_node.size());
    for (size_t i = 0; i < group_node.size(); ++i)
    {
        const std::string name = group_node[i].as<std::string>();
        if (name.empty())
        {
            throw std::runtime_error(
                std::string("joint_groups.") + group_name + " contains empty joint name");
        }
        if (!seen.insert(name).second)
        {
            throw std::runtime_error(
                std::string("joint_groups.") + group_name + " contains duplicate joint: " + name);
        }
        out.push_back(name);
    }
    return out;
}

inline JointGroupsConfig loadJointGroupsFromYAML(
    const std::string &yaml_file)
{
    const YAML::Node root = loadRootConfigDocument(yaml_file).root;
    const YAML::Node joint_groups = root["joint_groups"];
    if (!joint_groups)
    {
        throw std::runtime_error("joint_groups is required");
    }
    if (!joint_groups.IsMap())
    {
        throw std::runtime_error("joint_groups must be a map");
    }

    JointGroupsConfig out;
    out.leg = loadJointGroupNamesFromRoot(joint_groups, "leg", true);
    out.arm = loadJointGroupNamesFromRoot(joint_groups, "arm", false);
    out.waist = loadJointGroupNamesFromRoot(joint_groups, "waist", false);
    return out;
}

inline std::string resolveDeployConfigSectionForModeFromYAML(
    const std::string &yaml_file,
    int mode_id,
    const std::string &fallback_section = "engineai_walk")
{
    std::vector<DeployModeProfileSpec> specs;
    try
    {
        specs = loadDeployModeProfilesFromYAML(yaml_file);
    }
    catch (const std::exception &)
    {
        specs.clear();
    }
    for (const auto &spec : specs)
    {
        if (spec.mode_id == mode_id && !spec.config_section.empty())
        {
            return spec.config_section;
        }
    }
    if (!specs.empty() && !specs.front().config_section.empty())
    {
        return specs.front().config_section;
    }
    return fallback_section;
}

inline int readDeployModeIdFromEnv(
    const char *env_key,
    int fallback_mode_id)
{
    if (!env_key)
    {
        return fallback_mode_id;
    }
    const char *raw = std::getenv(env_key);
    if (!raw || raw[0] == '\0')
    {
        return fallback_mode_id;
    }
    try
    {
        return std::stoi(std::string(raw));
    }
    catch (const std::exception &)
    {
        return fallback_mode_id;
    }
}

#define RL_CFG_PATH RL_MASTER_ROOT_DIR "/config/rl_cfg.yaml"
#define OBS_MANIFEST_PATH RL_MASTER_ROOT_DIR "/config/observation_manifest.yaml"

struct ExternalObservationSpec
{
    std::string name;
    int dim = 0;
    bool required = false;
};

struct OnnxInputSpec
{
    std::string name;
    std::string source = "stacked_observation"; // stacked_observation / observation / time_step / feature / constant
    std::string feature_name;
    std::vector<int64_t> shape;
    std::string fill_policy = "error"; // error / zero
    std::vector<float> constant;
};

struct SourceContractImuInput
{
    std::string payload = "euler_compat"; // euler_compat / quaternion
    std::vector<std::string> euler_order{"roll", "pitch", "yaw"};
    std::string euler_unit = "rad";       // rad / deg
    std::string quat_order = "xyzw";      // xyzw / wxyz
    std::vector<std::string> ang_vel_order{"x", "y", "z"};
    std::vector<float> frame_alignment_rpy{0.0f, 0.0f, 0.0f};
};

struct SourceContractSimBase
{
    std::string quat_source_order = "wxyz"; // MuJoCo free-joint qpos order
};

struct SourceContractReferenceFile
{
    std::string reference_motion_key = "reference_motion";
    std::string reference_joint_pos_key = "joint_pos";
    std::string reference_joint_vel_key = "joint_vel";
    std::string reference_body_pos_w_key = "body_pos_w";
    std::string reference_body_quat_w_key = "body_quat_w";
    std::string body_quat_order = "wxyz";
    std::string body_quat_representation = "quat";
    std::string body_quat_frame = "world";
};

struct SourceContractPolicyExtraOutputs
{
    std::string reference_motion_key = "reference_motion";
    std::string reference_joint_pos_key = "joint_pos";
    std::string reference_joint_vel_key = "joint_vel";
    std::string reference_body_pos_w_key = "body_pos_w";
    std::string reference_body_quat_w_key = "body_quat_w";
    std::string body_quat_order = "wxyz";
    std::string body_quat_representation = "quat";
    std::string body_quat_frame = "world";
};

struct ObservationCanonicalContract
{
    std::string joint_order = "robot_global_joint_order";
    std::string quat_order = "xyzw";
    std::string quat_representation = "quat";
    std::string body_quat_frame = "world";
    std::string body_orientation_representation = "rot6";
};

struct SourceContract
{
    SourceContractImuInput imu_input;
    SourceContractSimBase sim_base;
    SourceContractReferenceFile reference_file;
    SourceContractPolicyExtraOutputs policy_extra_outputs;
};

struct PolicySubModelCfg
{
    std::string name;
    bool enabled = true;
    float weight = 1.0f;
    int action_dim = -1;

    std::string policy_file;
    std::string policy_path;

    std::string obs_input_name = "obs";
    std::string action_output_name = "actions";
    std::string time_step_input_name = "time_step";
    int64_t time_step_start = 0;
    bool enable_time_step_input = false;
    bool strict_model_io = false;
    std::vector<std::string> extra_output_names;
    std::vector<OnnxInputSpec> onnx_inputs;
    bool enable_metadata_check = false;
    bool metadata_check_strict = true;
    std::vector<std::string> required_metadata_keys;
    std::map<std::string, std::string> expected_metadata;
};

struct AmpDiscriminatorCfg
{
    bool enabled = false;
    std::string policy_file;
    std::string policy_path;

    // stacked_observation / observation
    std::string input_source = "stacked_observation";

    std::string obs_input_name = "obs";
    std::string score_output_name = "disc_score";
    std::string time_step_input_name = "time_step";
    int64_t time_step_start = 0;
    bool enable_time_step_input = false;
    bool strict_model_io = false;
    std::vector<std::string> extra_output_names;
    std::vector<OnnxInputSpec> onnx_inputs;
    bool enable_metadata_check = false;
    bool metadata_check_strict = true;
    std::vector<std::string> required_metadata_keys;
    std::map<std::string, std::string> expected_metadata;

    // Disabled when set to a very negative number (default).
    float warn_below = -1.0e9f;
};

struct RuntimeLogWriterConfig
{
    int queue_capacity = 256;
    int flush_period_ms = 250;
    std::string compression = "none";
    int chunk_size_kb = 1024;
};

struct RuntimeLogTickConfig
{
    bool enabled = true;
    int decimation = 1;
    bool include_observation = true;
    bool include_policy_action = true;
    bool include_motor_io = true;
    bool include_joint_targets = true;
    bool include_external_observations = false;
};

struct RuntimeLogEventsConfig
{
    bool enabled = true;
};

struct RuntimeLogReferenceMotionConfig
{
    bool enabled = false;
};

struct RuntimeLogAmpConfig
{
    bool enabled = false;
};

struct RuntimeLogSourceSamplesConfig
{
    bool enabled = true;
    bool include_base_imu = true;
    bool include_external_observations = false;
};

struct RuntimeLogExportConfig
{
    std::string default_format = "none";
};

struct RuntimeLoggingConfig
{
    bool enabled = false;
    std::string backend = "mcap";
    std::string output_dir;
    std::string session_name_policy = "timestamp_policy";
    std::string custom_session_name;
    RuntimeLogWriterConfig writer;
    RuntimeLogTickConfig tick;
    RuntimeLogEventsConfig events;
    RuntimeLogReferenceMotionConfig reference_motion;
    RuntimeLogAmpConfig amp;
    RuntimeLogSourceSamplesConfig source_samples;
    RuntimeLogExportConfig export_config;
    std::string session_base_path;
    std::string output_file_path;
};

class Sim2realCfg
{
public:
    std::string humanoid_rl_root_dir;
    std::string policy_name;
    std::string policy_family = "amp"; // amp / beyondmimic / custom
    int obs_stack_N = 1;
    double cycle_time = 1.0;

    std::vector<float> kps;
    std::vector<float> kds;
    std::vector<float> tau_limit;
    std::map<std::string, float> named_kps;
    std::map<std::string, float> named_kds;
    std::map<std::string, float> named_tau_limit;

    float clip_observations = 100.0f;
    float clip_actions = 100.0f;
    float action_scale = 1.0f;

    std::string policy_path;
    int device_id = 0;

    int obs_dim = 0;
    int action_dim = 0;
    int motor_N = 0;
    int RL_control_f = 100;
    int solver_control_hz = 500;
    std::vector<std::string> action_joint_order;
    std::map<std::string, std::string> installed_joint_run_modes;
    std::vector<std::string> obs_joint_order;
    std::string observation_manifest_file;
    std::string observation_manifest_path;

    int onnx_intra_threads = 1;
    int onnx_inter_threads = 1;
    std::string obs_input_name = "obs";
    std::string action_output_name = "actions";
    std::string time_step_input_name = "time_step";
    int64_t time_step_start = 0;
    bool enable_time_step_input = false;
    bool strict_model_io = false;
    bool reset_policy_on_mode_switch = true;
    std::vector<std::string> extra_output_names;
    std::vector<OnnxInputSpec> onnx_inputs;
    bool enable_metadata_check = false;
    bool metadata_check_strict = true;
    std::vector<std::string> required_metadata_keys;
    std::map<std::string, std::string> expected_metadata;
    std::vector<PolicySubModelCfg> sub_models;
    AmpDiscriminatorCfg amp_discriminator;

    bool enable_reference_motion = false;
    int reference_motion_dim = 0;
    std::string reference_motion_file;
    std::string reference_motion_path;
    std::string reference_motion_sampling = "phase"; // phase / step
    std::string reference_motion_source = "auto";    // auto / file / policy_outputs
    std::string reference_anchor_body = "base";
    std::vector<std::string> reference_body_names;
    std::vector<std::string> reference_joint_order;
    std::vector<ExternalObservationSpec> external_observations;
    SourceContract source_contract;
    ObservationCanonicalContract observation_canonical_contract;

    std::string startup_completion_action = "hold";
    double zeroing_duration_s = 2.0;
    double zeroing_position_tolerance = 0.05;
    double zeroing_velocity_tolerance = 0.2;

    bool enable_cmd_watchdog = true;
    double cmd_timeout_s = 0.12;
    int loop_overrun_warn_us = 2000;
    bool enable_state_telemetry = true;
    double state_telemetry_hz = 50.0;

    RuntimeLoggingConfig logging;
    std::string data_path;
    std::string control_mode;
    float action_filter = 0.0f;
    rl_master::runtime::RealtimeConfig realtime;

    class RobotCfg
    {
    public:
        std::vector<std::pair<std::string, float>> default_joint_angles;
        std::vector<std::pair<std::string, float>> zero_joint_angles;
        std::map<std::string, std::vector<float>> joint_limit_range;
        std::map<std::string, float> motor_torque_limit;
    };

    class Scales
    {
    public:
        float lin_vel = 1.0f;
        float ang_vel = 1.0f;
        float dof_pos = 1.0f;
        float dof_vel = 1.0f;
        float quat = 1.0f;
        float height_measurements = 1.0f;
    };

    RobotCfg robotCfg;
    Scales scales;

    bool loadFromYAML(const std::string &yaml_file, const std::string &config_type = "engineai_walk")
    {
        try
        {
            const ProfileConfigDocument config_doc = loadProfileConfigDocument(yaml_file, config_type);
            const YAML::Node config = config_doc.root_doc.root;
            const YAML::Node cfg = config_doc.section;

            robotCfg.default_joint_angles.clear();
            robotCfg.zero_joint_angles.clear();
            robotCfg.joint_limit_range.clear();
            robotCfg.motor_torque_limit.clear();
            sub_models.clear();
            external_observations.clear();
            amp_discriminator = AmpDiscriminatorCfg{};
            onnx_inputs.clear();
            reference_joint_order.clear();
            named_kps.clear();
            named_kds.clear();
            named_tau_limit.clear();
            source_contract = SourceContract{};
            observation_canonical_contract = ObservationCanonicalContract{};
            logging = RuntimeLoggingConfig{};

            const std::string configured_root_raw = yamlReadOr<std::string>(config, "humanoid_rl_root_dir", "");
            const std::filesystem::path cfg_parent_dir = config_doc.root_doc.root_dir;
            const std::filesystem::path default_root_path = std::filesystem::path(RL_MASTER_ROOT_DIR);
            std::filesystem::path resolved_root_path = default_root_path;
            if (!configured_root_raw.empty())
            {
                std::filesystem::path candidate = std::filesystem::path(configured_root_raw);
                if (candidate.is_relative())
                {
                    candidate = cfg_parent_dir / candidate;
                }
                if (std::filesystem::exists(candidate))
                {
                    resolved_root_path = candidate;
                }
                else
                {
                    std::cerr << "[Sim2realCfg] warning: configured humanoid_rl_root_dir does not exist: "
                              << candidate
                              << ", fallback to RL_MASTER_ROOT_DIR: "
                              << default_root_path << std::endl;
                }
            }
            else
            {
                std::cerr << "[Sim2realCfg] warning: missing humanoid_rl_root_dir, fallback to RL_MASTER_ROOT_DIR: "
                          << default_root_path << std::endl;
            }
            humanoid_rl_root_dir = resolved_root_path.string();

            auto resolvePath = [&](const std::string &raw) -> std::string {
                if (raw.empty())
                {
                    return "";
                }
                const std::filesystem::path p(raw);
                if (p.is_absolute())
                {
                    return raw;
                }
                return (std::filesystem::path(humanoid_rl_root_dir) / p).string();
            };

            const YAML::Node logging_cfg = config["logging"];
            logging.enabled = yamlReadOr<bool>(logging_cfg, "enabled", false);
            logging.backend = yamlReadOr<std::string>(logging_cfg, "backend", "mcap");
            logging.output_dir = yamlReadOr<std::string>(logging_cfg, "output_dir", "data");
            logging.session_name_policy = yamlReadOr<std::string>(logging_cfg, "session_name_policy", "timestamp_policy");
            logging.custom_session_name = yamlReadOr<std::string>(logging_cfg, "custom_session_name", "");
            if (!logging.output_dir.empty())
            {
                logging.output_dir = resolvePath(logging.output_dir);
            }
            else
            {
                logging.output_dir = (std::filesystem::path(humanoid_rl_root_dir) / "data").string();
            }

            const YAML::Node logging_writer_cfg = logging_cfg["writer"];
            logging.writer.queue_capacity = yamlReadOr<int>(logging_writer_cfg, "queue_capacity", 256);
            logging.writer.flush_period_ms = yamlReadOr<int>(logging_writer_cfg, "flush_period_ms", 250);
            logging.writer.compression = yamlReadOr<std::string>(logging_writer_cfg, "compression", "none");
            logging.writer.chunk_size_kb = yamlReadOr<int>(logging_writer_cfg, "chunk_size_kb", 1024);

            const YAML::Node logging_tick_cfg = logging_cfg["tick"];
            logging.tick.enabled = yamlReadOr<bool>(logging_tick_cfg, "enabled", true);
            logging.tick.decimation = yamlReadOr<int>(logging_tick_cfg, "decimation", 1);
            logging.tick.include_observation = yamlReadOr<bool>(logging_tick_cfg, "include_observation", true);
            logging.tick.include_policy_action = yamlReadOr<bool>(logging_tick_cfg, "include_policy_action", true);
            logging.tick.include_motor_io = yamlReadOr<bool>(logging_tick_cfg, "include_motor_io", true);
            logging.tick.include_joint_targets = yamlReadOr<bool>(logging_tick_cfg, "include_joint_targets", true);
            logging.tick.include_external_observations = yamlReadOr<bool>(logging_tick_cfg, "include_external_observations", false);

            const YAML::Node logging_events_cfg = logging_cfg["events"];
            logging.events.enabled = yamlReadOr<bool>(logging_events_cfg, "enabled", true);

            const YAML::Node logging_reference_cfg = logging_cfg["reference_motion"];
            logging.reference_motion.enabled = yamlReadOr<bool>(logging_reference_cfg, "enabled", false);

            const YAML::Node logging_amp_cfg = logging_cfg["amp"];
            logging.amp.enabled = yamlReadOr<bool>(logging_amp_cfg, "enabled", false);

            const YAML::Node logging_sources_cfg = logging_cfg["source_samples"];
            logging.source_samples.enabled = yamlReadOr<bool>(logging_sources_cfg, "enabled", true);
            logging.source_samples.include_base_imu = yamlReadOr<bool>(logging_sources_cfg, "include_base_imu", true);
            logging.source_samples.include_external_observations = yamlReadOr<bool>(logging_sources_cfg, "include_external_observations", false);

            const YAML::Node logging_export_cfg = logging_cfg["export"];
            logging.export_config.default_format = yamlReadOr<std::string>(logging_export_cfg, "default_format", "none");

            auto parseOnnxInputs = [](const YAML::Node &node) -> std::vector<OnnxInputSpec> {
                std::vector<OnnxInputSpec> specs;
                if (!node || !node.IsSequence())
                {
                    return specs;
                }
                specs.reserve(node.size());
                for (size_t i = 0; i < node.size(); ++i)
                {
                    const YAML::Node item = node[i];
                    if (!item || !item.IsMap())
                    {
                        continue;
                    }
                    OnnxInputSpec spec;
                    spec.name = yamlReadOr<std::string>(item, "name", "");
                    spec.source = yamlReadOr<std::string>(item, "source", "stacked_observation");
                    spec.feature_name = yamlReadOr<std::string>(item, "feature_name", "");
                    spec.fill_policy = yamlReadOr<std::string>(item, "fill_policy", "error");
                    if (item["shape"])
                    {
                        spec.shape = item["shape"].as<std::vector<int64_t>>();
                    }
                    if (item["constant"])
                    {
                        spec.constant = item["constant"].as<std::vector<float>>();
                    }
                    specs.push_back(std::move(spec));
                }
                return specs;
            };

            auto validatePermutation = [](
                                           const std::vector<std::string> &values,
                                           const std::vector<std::string> &expected,
                                           const std::string &field_name) {
                if (values.size() != expected.size())
                {
                    throw std::runtime_error(field_name + " must contain exactly " + std::to_string(expected.size()) + " items");
                }
                std::unordered_set<std::string> seen;
                seen.reserve(values.size());
                for (const auto &value : values)
                {
                    if (!seen.insert(value).second)
                    {
                        throw std::runtime_error(field_name + " contains duplicates");
                    }
                }
                for (const auto &expected_value : expected)
                {
                    if (seen.find(expected_value) == seen.end())
                    {
                        throw std::runtime_error(field_name + " must be a permutation of the expected tokens");
                    }
                }
            };

            auto validateOnnxInputs = [](const std::vector<OnnxInputSpec> &specs, const std::string &owner_name) {
                std::unordered_set<std::string> seen_names;
                seen_names.reserve(specs.size());
                for (size_t i = 0; i < specs.size(); ++i)
                {
                    const auto &spec = specs[i];
                    const std::string item_name = owner_name + ".onnx_inputs[" + std::to_string(i) + "]";
                    if (spec.name.empty())
                    {
                        throw std::runtime_error(item_name + " missing name");
                    }
                    if (!seen_names.insert(spec.name).second)
                    {
                        throw std::runtime_error(item_name + " duplicate name: " + spec.name);
                    }
                    if (spec.source != "stacked_observation" &&
                        spec.source != "observation" &&
                        spec.source != "time_step" &&
                        spec.source != "feature" &&
                        spec.source != "constant")
                    {
                        throw std::runtime_error(item_name + " unsupported source: " + spec.source);
                    }
                    if (spec.fill_policy != "error" && spec.fill_policy != "zero")
                    {
                        throw std::runtime_error(item_name + " unsupported fill_policy: " + spec.fill_policy);
                    }
                    if (spec.source == "feature" && spec.feature_name.empty())
                    {
                        throw std::runtime_error(item_name + " requires feature_name when source=feature");
                    }
                    if (spec.source == "constant" && spec.constant.empty() && spec.fill_policy != "zero")
                    {
                        throw std::runtime_error(item_name + " constant source requires values or fill_policy=zero");
                    }
                    for (size_t dim_idx = 0; dim_idx < spec.shape.size(); ++dim_idx)
                    {
                        if (spec.shape[dim_idx] <= 0)
                        {
                            throw std::runtime_error(
                                item_name + " shape[" + std::to_string(dim_idx) + "] must be > 0");
                        }
                    }
                }
            };

            policy_name = yamlReadOr<std::string>(cfg, "policy_name", "");
            policy_family = yamlReadOr<std::string>(cfg, "policy_family", "amp");
            obs_stack_N = cfg["obs_stack_N"].as<int>();
            cycle_time = cfg["cycle_time"].as<double>();

            clip_observations = cfg["clip_observations"].as<float>();
            clip_actions = cfg["clip_actions"].as<float>();
            action_scale = cfg["action_scale"].as<float>();

            device_id = cfg["device_id"].as<int>();
            obs_dim = cfg["obs_dim"].as<int>();
            action_dim = cfg["action_dim"].as<int>();
            motor_N = cfg["motor_N"].as<int>();
            RL_control_f = cfg["RL_control_f"].as<int>();
            solver_control_hz = yamlReadOr<int>(cfg, "solver_control_hz", 500);
            action_joint_order = yamlReadOr<std::vector<std::string>>(cfg, "action_joint_order", {});
            installed_joint_run_modes = yamlReadStringMapOr(cfg, "installed_joint_run_modes");
            obs_joint_order = yamlReadOr<std::vector<std::string>>(cfg, "obs_joint_order", {});
            if (solver_control_hz <= 0)
            {
                throw std::runtime_error("solver_control_hz must be > 0");
            }
            if (obs_joint_order.empty())
            {
                obs_joint_order = action_joint_order;
            }
            if (action_joint_order.empty())
            {
                throw std::runtime_error("action_joint_order must not be empty");
            }
            if (action_joint_order.size() != static_cast<size_t>(action_dim))
            {
                throw std::runtime_error("action_joint_order length must equal action_dim");
            }
            for (auto &entry : installed_joint_run_modes)
            {
                std::string &mode = entry.second;
                std::transform(
                    mode.begin(),
                    mode.end(),
                    mode.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
                if (mode != "csp" && mode != "cst" && mode != "r1")
                {
                    throw std::runtime_error("installed_joint_run_modes items must be 'csp', 'cst' or 'r1'");
                }
            }
            {
                std::unordered_set<std::string> seen_action_joint_names;
                seen_action_joint_names.reserve(action_joint_order.size());
                for (const auto &name : action_joint_order)
                {
                    if (!seen_action_joint_names.insert(name).second)
                    {
                        throw std::runtime_error("action_joint_order contains duplicate joint: " + name);
                    }
                }
            }
            if (obs_joint_order.empty())
            {
                throw std::runtime_error("obs_joint_order must not be empty");
            }
            if (obs_joint_order.size() != static_cast<size_t>(motor_N))
            {
                throw std::runtime_error("obs_joint_order length must equal motor_N");
            }
            {
                std::unordered_set<std::string> seen_obs_joint_names;
                seen_obs_joint_names.reserve(obs_joint_order.size());
                for (const auto &name : obs_joint_order)
                {
                    if (!seen_obs_joint_names.insert(name).second)
                    {
                        throw std::runtime_error("obs_joint_order contains duplicate joint: " + name);
                    }
                }
            }
            named_kps = yamlReadFloatMapOr(cfg, "kps");
            named_kds = yamlReadFloatMapOr(cfg, "kds");
            if (named_kps.empty())
            {
                throw std::runtime_error("kps must be a non-empty joint-name map");
            }
            if (named_kds.empty())
            {
                throw std::runtime_error("kds must be a non-empty joint-name map");
            }
            auto validateNamedActionJointValueMap = [&](const std::map<std::string, float> &value_map, const std::string &field_name) {
                std::unordered_set<std::string> action_joint_names(
                    action_joint_order.begin(),
                    action_joint_order.end());
                std::unordered_set<std::string> obs_joint_names(
                    obs_joint_order.begin(),
                    obs_joint_order.end());
                for (const auto &entry : value_map)
                {
                    const std::string &joint_name = entry.first;
                    const bool in_action = action_joint_names.find(joint_name) != action_joint_names.end();
                    const bool in_obs = obs_joint_names.find(joint_name) != obs_joint_names.end();
                    if (!in_action && !in_obs)
                    {
                        throw std::runtime_error(
                            field_name + " contains joint not present in action_joint_order or obs_joint_order: " +
                            joint_name);
                    }
                    if (!in_action)
                    {
                        throw std::runtime_error(
                            field_name + " must match action_joint_order exactly; unexpected joint: " +
                            joint_name);
                    }
                }
                for (const auto &joint_name : action_joint_order)
                {
                    if (value_map.find(joint_name) == value_map.end())
                    {
                        throw std::runtime_error(
                            field_name + " missing action joint from action_joint_order: " +
                            joint_name);
                    }
                }
            };
            validateNamedActionJointValueMap(named_kps, "kps");
            validateNamedActionJointValueMap(named_kds, "kds");
            named_tau_limit = yamlReadFloatMapOr(cfg, "tau_limit");
            if (named_tau_limit.empty())
            {
                throw std::runtime_error("tau_limit must be a non-empty joint-name map");
            }
            validateNamedActionJointValueMap(named_tau_limit, "tau_limit");
            kps.clear();
            kds.clear();
            tau_limit.clear();
            kps.reserve(action_joint_order.size());
            kds.reserve(action_joint_order.size());
            tau_limit.reserve(action_joint_order.size());
            for (const auto &joint_name : action_joint_order)
            {
                kps.push_back(named_kps.at(joint_name));
                kds.push_back(named_kds.at(joint_name));
                tau_limit.push_back(named_tau_limit.at(joint_name));
            }

            const std::string policy_file = yamlReadOr<std::string>(cfg, "policy_file", "");
            const std::string policy_path_raw = yamlReadOr<std::string>(cfg, "policy_path", "");
            if (!policy_path_raw.empty())
            {
                policy_path = resolvePath(policy_path_raw);
            }
            else if (!policy_file.empty())
            {
                policy_path = resolvePath(policy_file);
            }
            else
            {
                policy_path = (std::filesystem::path(humanoid_rl_root_dir) / "policies" / (policy_name + ".onnx")).string();
            }
            if (policy_name.empty() && !policy_path.empty())
            {
                policy_name = std::filesystem::path(policy_path).stem().string();
            }

            observation_manifest_file = yamlReadOr<std::string>(cfg, "observation_manifest_file", "observation_manifest.yaml");
            const std::string observation_manifest_path_raw = yamlReadOr<std::string>(cfg, "observation_manifest_path", "");
            if (!observation_manifest_path_raw.empty())
            {
                observation_manifest_path = resolvePath(observation_manifest_path_raw);
            }
            else
            {
                observation_manifest_path = (std::filesystem::path(humanoid_rl_root_dir) / "config" / observation_manifest_file).string();
            }
            if (!std::filesystem::exists(observation_manifest_path))
            {
                observation_manifest_path = OBS_MANIFEST_PATH;
            }

            onnx_intra_threads = yamlReadOr<int>(cfg, "onnx_intra_threads", 1);
            onnx_inter_threads = yamlReadOr<int>(cfg, "onnx_inter_threads", 1);

            const YAML::Node policy_io_cfg = cfg["policy_io"] ? cfg["policy_io"] : cfg;
            obs_input_name = yamlReadOr<std::string>(policy_io_cfg, "obs_input_name", "obs");
            action_output_name = yamlReadOr<std::string>(policy_io_cfg, "action_output_name", "actions");
            time_step_input_name = yamlReadOr<std::string>(policy_io_cfg, "time_step_input_name", "time_step");
            time_step_start = yamlReadOr<int64_t>(policy_io_cfg, "time_step_start", 0);
            enable_time_step_input = yamlReadOr<bool>(policy_io_cfg, "enable_time_step_input", false);
            strict_model_io = yamlReadOr<bool>(policy_io_cfg, "strict_model_io", false);
            reset_policy_on_mode_switch = yamlReadOr<bool>(policy_io_cfg, "reset_policy_on_mode_switch", true);
            extra_output_names = yamlReadOr<std::vector<std::string>>(policy_io_cfg, "extra_output_names", {});
            onnx_inputs = parseOnnxInputs(policy_io_cfg["onnx_inputs"]);
            validateOnnxInputs(onnx_inputs, config_type + ".policy_io");
            enable_metadata_check = yamlReadOr<bool>(policy_io_cfg, "enable_metadata_check", false);
            metadata_check_strict = yamlReadOr<bool>(policy_io_cfg, "metadata_check_strict", true);
            required_metadata_keys =
                yamlReadOr<std::vector<std::string>>(policy_io_cfg, "required_metadata_keys", {});
            expected_metadata = yamlReadStringMapOr(policy_io_cfg, "expected_metadata");

            if (cfg["sub_models"])
            {
                int sub_index = 0;
                for (const auto &node : cfg["sub_models"])
                {
                    PolicySubModelCfg sub;
                    sub.name = yamlReadOr<std::string>(node, "name", "sub_model_" + std::to_string(sub_index));
                    sub.enabled = yamlReadOr<bool>(node, "enabled", true);
                    sub.weight = yamlReadOr<float>(node, "weight", 1.0f);
                    sub.action_dim = yamlReadOr<int>(node, "action_dim", -1);
                    sub.policy_file = yamlReadOr<std::string>(node, "policy_file", "");

                    const std::string sub_path_raw = yamlReadOr<std::string>(node, "policy_path", "");
                    if (!sub_path_raw.empty())
                    {
                        sub.policy_path = resolvePath(sub_path_raw);
                    }
                    else if (!sub.policy_file.empty())
                    {
                        sub.policy_path = resolvePath(sub.policy_file);
                    }
                    else
                    {
                        throw std::runtime_error("sub_models[" + std::to_string(sub_index) + "] missing policy_file/policy_path");
                    }

                    const YAML::Node sub_io_cfg = node["policy_io"] ? node["policy_io"] : node;
                    sub.obs_input_name = yamlReadOr<std::string>(sub_io_cfg, "obs_input_name", obs_input_name);
                    sub.action_output_name = yamlReadOr<std::string>(sub_io_cfg, "action_output_name", action_output_name);
                    sub.time_step_input_name = yamlReadOr<std::string>(sub_io_cfg, "time_step_input_name", time_step_input_name);
                    sub.time_step_start = yamlReadOr<int64_t>(sub_io_cfg, "time_step_start", time_step_start);
                    sub.enable_time_step_input = yamlReadOr<bool>(sub_io_cfg, "enable_time_step_input", enable_time_step_input);
                    sub.strict_model_io = yamlReadOr<bool>(sub_io_cfg, "strict_model_io", strict_model_io);
                    sub.extra_output_names = yamlReadOr<std::vector<std::string>>(sub_io_cfg, "extra_output_names", {});
                    sub.onnx_inputs = parseOnnxInputs(sub_io_cfg["onnx_inputs"]);
                    validateOnnxInputs(sub.onnx_inputs, config_type + ".sub_models[" + std::to_string(sub_index) + "].policy_io");
                    sub.enable_metadata_check = yamlReadOr<bool>(sub_io_cfg, "enable_metadata_check", enable_metadata_check);
                    sub.metadata_check_strict = yamlReadOr<bool>(sub_io_cfg, "metadata_check_strict", metadata_check_strict);
                    sub.required_metadata_keys = yamlReadOr<std::vector<std::string>>(
                        sub_io_cfg,
                        "required_metadata_keys",
                        required_metadata_keys);
                    sub.expected_metadata = expected_metadata;
                    if (sub_io_cfg && sub_io_cfg["expected_metadata"])
                    {
                        sub.expected_metadata = yamlReadStringMapOr(sub_io_cfg, "expected_metadata");
                    }

                    sub_models.push_back(sub);
                    ++sub_index;
                }
            }

            if (cfg["amp_discriminator"])
            {
                const YAML::Node disc = cfg["amp_discriminator"];
                amp_discriminator.enabled = yamlReadOr<bool>(disc, "enabled", false);
                amp_discriminator.policy_file = yamlReadOr<std::string>(disc, "policy_file", "");
                amp_discriminator.input_source = yamlReadOr<std::string>(disc, "input_source", "stacked_observation");

                const std::string disc_path_raw = yamlReadOr<std::string>(disc, "policy_path", "");
                if (!disc_path_raw.empty())
                {
                    amp_discriminator.policy_path = resolvePath(disc_path_raw);
                }
                else if (!amp_discriminator.policy_file.empty())
                {
                    amp_discriminator.policy_path = resolvePath(amp_discriminator.policy_file);
                }
                if (amp_discriminator.enabled && amp_discriminator.policy_path.empty())
                {
                    throw std::runtime_error("amp_discriminator enabled but policy_file/policy_path is empty");
                }

                const YAML::Node disc_io_cfg = disc["policy_io"] ? disc["policy_io"] : disc;
                amp_discriminator.obs_input_name = yamlReadOr<std::string>(disc_io_cfg, "obs_input_name", "obs");
                amp_discriminator.score_output_name = yamlReadOr<std::string>(disc_io_cfg, "score_output_name", "disc_score");
                amp_discriminator.time_step_input_name = yamlReadOr<std::string>(disc_io_cfg, "time_step_input_name", "time_step");
                amp_discriminator.time_step_start = yamlReadOr<int64_t>(disc_io_cfg, "time_step_start", 0);
                amp_discriminator.enable_time_step_input = yamlReadOr<bool>(disc_io_cfg, "enable_time_step_input", false);
                amp_discriminator.strict_model_io = yamlReadOr<bool>(disc_io_cfg, "strict_model_io", false);
                amp_discriminator.extra_output_names = yamlReadOr<std::vector<std::string>>(disc_io_cfg, "extra_output_names", {});
                amp_discriminator.onnx_inputs = parseOnnxInputs(disc_io_cfg["onnx_inputs"]);
                validateOnnxInputs(amp_discriminator.onnx_inputs, config_type + ".amp_discriminator.policy_io");
                amp_discriminator.enable_metadata_check = yamlReadOr<bool>(disc_io_cfg, "enable_metadata_check", false);
                amp_discriminator.metadata_check_strict = yamlReadOr<bool>(disc_io_cfg, "metadata_check_strict", true);
                amp_discriminator.required_metadata_keys =
                    yamlReadOr<std::vector<std::string>>(disc_io_cfg, "required_metadata_keys", {});
                amp_discriminator.expected_metadata = yamlReadStringMapOr(disc_io_cfg, "expected_metadata");
                amp_discriminator.warn_below = yamlReadOr<float>(disc_io_cfg, "warn_below", -1.0e9f);
            }

            enable_reference_motion = yamlReadOr<bool>(cfg, "enable_reference_motion", false);
            reference_motion_dim = yamlReadOr<int>(cfg, "reference_motion_dim", 0);
            reference_motion_file = yamlReadOr<std::string>(cfg, "reference_motion_file", "");
            reference_motion_sampling = yamlReadOr<std::string>(cfg, "reference_motion_sampling", "phase");
            reference_motion_source = yamlReadOr<std::string>(cfg, "reference_motion_source", "auto");
            reference_anchor_body = yamlReadOr<std::string>(cfg, "reference_anchor_body", "base");
            reference_body_names = yamlReadOr<std::vector<std::string>>(cfg, "reference_body_names", {});
            reference_joint_order = yamlReadOr<std::vector<std::string>>(cfg, "reference_joint_order", {});
            const std::string reference_motion_path_raw = yamlReadOr<std::string>(cfg, "reference_motion_path", "");
            if (!reference_motion_path_raw.empty())
            {
                reference_motion_path = resolvePath(reference_motion_path_raw);
            }
            else
            {
                reference_motion_path = resolvePath(reference_motion_file);
            }

            if (reference_joint_order.empty())
            {
                reference_joint_order = action_joint_order;
            }
            if (!reference_joint_order.empty() && reference_joint_order.size() != static_cast<size_t>(motor_N))
            {
                throw std::runtime_error("reference_joint_order length must equal motor_N");
            }
            {
                std::unordered_set<std::string> seen_reference_joint_names;
                seen_reference_joint_names.reserve(reference_joint_order.size());
                for (const auto &name : reference_joint_order)
                {
                    if (!seen_reference_joint_names.insert(name).second)
                    {
                        throw std::runtime_error("reference_joint_order contains duplicate joint: " + name);
                    }
                }
            }

            const YAML::Node source_contract_cfg = cfg["source_contract"];
            const YAML::Node imu_contract_cfg = source_contract_cfg["imu_input"];
            source_contract.imu_input.payload = yamlReadOr<std::string>(
                imu_contract_cfg, "payload", source_contract.imu_input.payload);
            source_contract.imu_input.euler_order = yamlReadOr<std::vector<std::string>>(
                imu_contract_cfg, "euler_order", source_contract.imu_input.euler_order);
            source_contract.imu_input.euler_unit = yamlReadOr<std::string>(
                imu_contract_cfg, "euler_unit", source_contract.imu_input.euler_unit);
            source_contract.imu_input.quat_order = yamlReadOr<std::string>(
                imu_contract_cfg, "quat_order", source_contract.imu_input.quat_order);
            source_contract.imu_input.ang_vel_order = yamlReadOr<std::vector<std::string>>(
                imu_contract_cfg, "ang_vel_order", source_contract.imu_input.ang_vel_order);
            source_contract.imu_input.frame_alignment_rpy = yamlReadOr<std::vector<float>>(
                imu_contract_cfg, "frame_alignment_rpy", source_contract.imu_input.frame_alignment_rpy);

            const YAML::Node sim_base_contract_cfg = source_contract_cfg["sim_base"];
            source_contract.sim_base.quat_source_order = yamlReadOr<std::string>(
                sim_base_contract_cfg, "quat_source_order", source_contract.sim_base.quat_source_order);

            const YAML::Node reference_file_contract_cfg = source_contract_cfg["reference_file"];
            source_contract.reference_file.reference_motion_key = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "reference_motion_key",
                source_contract.reference_file.reference_motion_key);
            source_contract.reference_file.reference_joint_pos_key = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "reference_joint_pos_key",
                source_contract.reference_file.reference_joint_pos_key);
            source_contract.reference_file.reference_joint_vel_key = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "reference_joint_vel_key",
                source_contract.reference_file.reference_joint_vel_key);
            source_contract.reference_file.reference_body_pos_w_key = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "reference_body_pos_w_key",
                source_contract.reference_file.reference_body_pos_w_key);
            source_contract.reference_file.reference_body_quat_w_key = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "reference_body_quat_w_key",
                source_contract.reference_file.reference_body_quat_w_key);
            source_contract.reference_file.body_quat_order = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "body_quat_order",
                source_contract.reference_file.body_quat_order);
            source_contract.reference_file.body_quat_representation = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "body_quat_representation",
                source_contract.reference_file.body_quat_representation);
            source_contract.reference_file.body_quat_frame = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "body_quat_frame",
                source_contract.reference_file.body_quat_frame);

            const YAML::Node policy_extra_outputs_contract_cfg = source_contract_cfg["policy_extra_outputs"];
            source_contract.policy_extra_outputs.reference_motion_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_motion_key",
                source_contract.policy_extra_outputs.reference_motion_key);
            source_contract.policy_extra_outputs.reference_joint_pos_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_joint_pos_key",
                source_contract.policy_extra_outputs.reference_joint_pos_key);
            source_contract.policy_extra_outputs.reference_joint_vel_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_joint_vel_key",
                source_contract.policy_extra_outputs.reference_joint_vel_key);
            source_contract.policy_extra_outputs.reference_body_pos_w_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_body_pos_w_key",
                source_contract.policy_extra_outputs.reference_body_pos_w_key);
            source_contract.policy_extra_outputs.reference_body_quat_w_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_body_quat_w_key",
                source_contract.policy_extra_outputs.reference_body_quat_w_key);
            source_contract.policy_extra_outputs.body_quat_order = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "body_quat_order",
                source_contract.policy_extra_outputs.body_quat_order);
            source_contract.policy_extra_outputs.body_quat_representation = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "body_quat_representation",
                source_contract.policy_extra_outputs.body_quat_representation);
            source_contract.policy_extra_outputs.body_quat_frame = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "body_quat_frame",
                source_contract.policy_extra_outputs.body_quat_frame);

            const YAML::Node observation_contract_cfg = cfg["observation_contract"];
            const YAML::Node canonical_contract_cfg = observation_contract_cfg["canonical"];
            observation_canonical_contract.joint_order = yamlReadOr<std::string>(
                canonical_contract_cfg,
                "joint_order",
                observation_canonical_contract.joint_order);
            observation_canonical_contract.quat_order = yamlReadOr<std::string>(
                canonical_contract_cfg,
                "quat_order",
                observation_canonical_contract.quat_order);
            observation_canonical_contract.quat_representation = yamlReadOr<std::string>(
                canonical_contract_cfg,
                "quat_representation",
                observation_canonical_contract.quat_representation);
            observation_canonical_contract.body_quat_frame = yamlReadOr<std::string>(
                canonical_contract_cfg,
                "body_quat_frame",
                observation_canonical_contract.body_quat_frame);
            observation_canonical_contract.body_orientation_representation = yamlReadOr<std::string>(
                canonical_contract_cfg,
                "body_orientation_representation",
                observation_canonical_contract.body_orientation_representation);

            if (source_contract.imu_input.euler_order.size() != 3)
            {
                throw std::runtime_error("source_contract.imu_input.euler_order must contain exactly 3 items");
            }
            if (source_contract.imu_input.ang_vel_order.size() != 3)
            {
                throw std::runtime_error("source_contract.imu_input.ang_vel_order must contain exactly 3 items");
            }
            if (source_contract.imu_input.frame_alignment_rpy.size() != 3)
            {
                throw std::runtime_error("source_contract.imu_input.frame_alignment_rpy must contain exactly 3 items");
            }
            validatePermutation(
                source_contract.imu_input.euler_order,
                {"roll", "pitch", "yaw"},
                "source_contract.imu_input.euler_order");
            validatePermutation(
                source_contract.imu_input.ang_vel_order,
                {"x", "y", "z"},
                "source_contract.imu_input.ang_vel_order");
            if (source_contract.imu_input.payload != "euler_compat" &&
                source_contract.imu_input.payload != "quaternion")
            {
                throw std::runtime_error("source_contract.imu_input.payload must be 'euler_compat' or 'quaternion'");
            }
            if (source_contract.imu_input.euler_unit != "rad" &&
                source_contract.imu_input.euler_unit != "deg")
            {
                throw std::runtime_error("source_contract.imu_input.euler_unit must be 'rad' or 'deg'");
            }
            if (source_contract.imu_input.quat_order != "xyzw" &&
                source_contract.imu_input.quat_order != "wxyz")
            {
                throw std::runtime_error("source_contract.imu_input.quat_order must be 'xyzw' or 'wxyz'");
            }
            if (source_contract.sim_base.quat_source_order != "wxyz" &&
                source_contract.sim_base.quat_source_order != "xyzw")
            {
                throw std::runtime_error("source_contract.sim_base.quat_source_order must be 'wxyz' or 'xyzw'");
            }
            auto normalizeLower = [](std::string text) {
                std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
                    return static_cast<char>(std::tolower(c));
                });
                return text;
            };
            const std::string normalized_reference_motion_source = normalizeLower(reference_motion_source);
            const bool reference_file_contract_active =
                enable_reference_motion && normalized_reference_motion_source != "policy_outputs";
            if (reference_file_contract_active)
            {
                if (source_contract.reference_file.body_quat_order != "xyzw" &&
                    source_contract.reference_file.body_quat_order != "wxyz")
                {
                    throw std::runtime_error("source_contract.reference_file.body_quat_order must be 'xyzw' or 'wxyz'");
                }
                if (source_contract.reference_file.body_quat_representation != "quat")
                {
                    throw std::runtime_error("source_contract.reference_file.body_quat_representation must be 'quat'");
                }
                if (source_contract.reference_file.body_quat_frame != "world")
                {
                    throw std::runtime_error("source_contract.reference_file.body_quat_frame must be 'world'");
                }
            }
            const bool policy_extra_reference_contract_active =
                enable_reference_motion && normalized_reference_motion_source != "file";
            if (policy_extra_reference_contract_active)
            {
                if (source_contract.policy_extra_outputs.body_quat_order != "xyzw" &&
                    source_contract.policy_extra_outputs.body_quat_order != "wxyz")
                {
                    throw std::runtime_error("source_contract.policy_extra_outputs.body_quat_order must be 'xyzw' or 'wxyz'");
                }
                if (source_contract.policy_extra_outputs.body_quat_representation != "quat")
                {
                    throw std::runtime_error("source_contract.policy_extra_outputs.body_quat_representation must be 'quat'");
                }
                if (source_contract.policy_extra_outputs.body_quat_frame != "world")
                {
                    throw std::runtime_error("source_contract.policy_extra_outputs.body_quat_frame must be 'world'");
                }
            }
            if (observation_canonical_contract.joint_order != "robot_global_joint_order")
            {
                throw std::runtime_error(
                    "observation_contract.canonical.joint_order must be 'robot_global_joint_order'");
            }
            if (observation_canonical_contract.quat_order != "xyzw")
            {
                throw std::runtime_error(
                    "observation_contract.canonical.quat_order must be 'xyzw' in the current implementation");
            }
            if (observation_canonical_contract.quat_representation != "quat")
            {
                throw std::runtime_error(
                    "observation_contract.canonical.quat_representation must be 'quat'");
            }
            if (observation_canonical_contract.body_orientation_representation != "rot6")
            {
                throw std::runtime_error(
                    "observation_contract.canonical.body_orientation_representation must be 'rot6'");
            }

            if (cfg["external_observations"])
            {
                for (const auto &node : cfg["external_observations"])
                {
                    ExternalObservationSpec spec;
                    spec.name = yamlReadOr<std::string>(node, "name", "");
                    spec.dim = yamlReadOr<int>(node, "dim", 0);
                    spec.required = yamlReadOr<bool>(node, "required", false);
                    if (spec.name.empty())
                    {
                        continue;
                    }
                    external_observations.push_back(spec);
                }
            }

            startup_completion_action = yamlReadOr<std::string>(cfg, "startup_completion_action", "hold");
            std::transform(
                startup_completion_action.begin(),
                startup_completion_action.end(),
                startup_completion_action.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (startup_completion_action != "hold" &&
                startup_completion_action != "running")
            {
                throw std::runtime_error(
                    "startup_completion_action must be 'hold' or 'running'");
            }
            zeroing_duration_s = yamlReadOr<double>(cfg, "zeroing_duration_s", 2.0);
            zeroing_position_tolerance = yamlReadOr<double>(cfg, "zeroing_position_tolerance", 0.05);
            if (zeroing_position_tolerance < 0.0)
            {
                throw std::runtime_error("zeroing_position_tolerance must be >= 0");
            }
            zeroing_velocity_tolerance = yamlReadOr<double>(cfg, "zeroing_velocity_tolerance", 0.2);
            if (zeroing_velocity_tolerance < 0.0)
            {
                throw std::runtime_error("zeroing_velocity_tolerance must be >= 0");
            }
            if (cfg["zero_pose"])
            {
                throw std::runtime_error(
                    "legacy top-level zero_pose vector is no longer supported; "
                    "use robot.zero_joint_angles instead");
            }

            enable_cmd_watchdog = yamlReadOr<bool>(cfg, "enable_cmd_watchdog", true);
            cmd_timeout_s = yamlReadOr<double>(cfg, "cmd_timeout_s", 0.12);
            loop_overrun_warn_us = yamlReadOr<int>(cfg, "loop_overrun_warn_us", 2000);
            enable_state_telemetry = yamlReadOr<bool>(cfg, "enable_state_telemetry", true);
            state_telemetry_hz = yamlReadOr<double>(cfg, "state_telemetry_hz", 50.0);

            realtime = parseRealtimeConfigNode(cfg["realtime"]);
            if (!cfg["realtime"])
            {
                realtime.enabled = yamlReadOr<bool>(cfg, "realtime_enabled", realtime.enabled);
                realtime.lock_memory = yamlReadOr<bool>(cfg, "realtime_lock_memory", realtime.lock_memory);
                realtime.set_affinity = yamlReadOr<bool>(cfg, "realtime_set_affinity", realtime.set_affinity);
                realtime.cpu_id = yamlReadOr<int>(cfg, "realtime_cpu_id", realtime.cpu_id);
                realtime.use_fifo_scheduler = yamlReadOr<bool>(cfg, "realtime_use_fifo", realtime.use_fifo_scheduler);
                realtime.fifo_priority = yamlReadOr<int>(cfg, "realtime_fifo_priority", realtime.fifo_priority);
            }

            control_mode = cfg["control_mode"].as<std::string>();
            action_filter = cfg["action_filter"].as<float>();
            if (cfg["save_data_flag"])
            {
                throw std::runtime_error("legacy save_data_flag is no longer supported; use top-level logging.enabled");
            }
            if (logging.backend != "mcap")
            {
                throw std::runtime_error("logging.backend currently only supports 'mcap'");
            }
            if (logging.writer.queue_capacity <= 0)
            {
                throw std::runtime_error("logging.writer.queue_capacity must be > 0");
            }
            if (logging.writer.flush_period_ms <= 0)
            {
                throw std::runtime_error("logging.writer.flush_period_ms must be > 0");
            }
            if (logging.tick.decimation <= 0)
            {
                throw std::runtime_error("logging.tick.decimation must be > 0");
            }

            std::string session_name;
            if (logging.session_name_policy == "custom")
            {
                session_name = logging.custom_session_name;
            }
            else if (logging.session_name_policy == "timestamp_mode")
            {
                session_name = getCurrentTime() + "_" + config_type;
            }
            else
            {
                session_name = getCurrentTime() + "_" + policy_name;
            }
            if (session_name.empty())
            {
                session_name = getCurrentTime() + "_" + config_type;
            }

            data_path = (std::filesystem::path(logging.output_dir) / session_name).string();
            logging.session_base_path = data_path;
            logging.output_file_path = data_path + ".mcap";

            const YAML::Node robot = cfg["robot"];
            if (!robot)
            {
                throw std::runtime_error("missing robot config in " + config_type);
            }

            const YAML::Node angles = robot["default_joint_angles"];
            for (auto it = angles.begin(); it != angles.end(); ++it)
            {
                robotCfg.default_joint_angles.push_back({
                    it->first.as<std::string>(),
                    it->second.as<float>()});
            }

            if (!robot["zero_joint_angles"])
            {
                throw std::runtime_error(
                    "robot.zero_joint_angles is required in " + config_type +
                    "; it must cover robot_global_joint_order");
            }
            {
                const YAML::Node zero_angles = robot["zero_joint_angles"];
                if (!zero_angles.IsMap())
                {
                    throw std::runtime_error("robot.zero_joint_angles must be a map in " + config_type);
                }
                for (auto it = zero_angles.begin(); it != zero_angles.end(); ++it)
                {
                    robotCfg.zero_joint_angles.push_back({
                        it->first.as<std::string>(),
                        it->second.as<float>()});
                }
                if (robotCfg.zero_joint_angles.empty())
                {
                    throw std::runtime_error(
                        "robot.zero_joint_angles must not be empty in " + config_type);
                }
            }

            const YAML::Node limits = robot["joint_limit_range"];
            for (auto it = limits.begin(); it != limits.end(); ++it)
            {
                robotCfg.joint_limit_range[it->first.as<std::string>()] =
                    it->second.as<std::vector<float>>();
            }

            const YAML::Node torques = robot["motor_torque_limit"];
            for (auto it = torques.begin(); it != torques.end(); ++it)
            {
                robotCfg.motor_torque_limit[it->first.as<std::string>()] =
                    it->second.as<float>();
            }

            const YAML::Node scale = cfg["scales"];
            scales.lin_vel = scale["lin_vel"].as<float>();
            scales.ang_vel = scale["ang_vel"].as<float>();
            scales.dof_pos = scale["dof_pos"].as<float>();
            scales.dof_vel = scale["dof_vel"].as<float>();
            scales.quat = scale["quat"].as<float>();
            scales.height_measurements = scale["height_measurements"].as<float>();

            return true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error loading YAML config: " << e.what() << std::endl;
            return false;
        }
    }

    void printConfig() const
    {
        std::cout << "=== Sim2Real Configuration ===" << std::endl;
        std::cout << "Policy Name: " << policy_name << std::endl;
        std::cout << "Policy Family: " << policy_family << std::endl;
        std::cout << "Policy Path: " << policy_path << std::endl;
        std::cout << "Obs Stack N: " << obs_stack_N << std::endl;
        std::cout << "Action Scale: " << action_scale << std::endl;
        std::cout << "Zero Joint Angles: " << robotCfg.zero_joint_angles.size() << std::endl;
        std::cout << "Control Mode: " << control_mode << std::endl;
        std::cout << "Installed Joint Run Modes: " << installed_joint_run_modes.size() << std::endl;
        std::cout << "Startup Completion Action: " << startup_completion_action << std::endl;
        std::cout << "Policy Frequency: " << RL_control_f << std::endl;
        std::cout << "Solver Control Frequency: " << solver_control_hz << std::endl;
        std::cout << "Sub Models: " << sub_models.size() << std::endl;
        std::cout << "AMP Discriminator Enabled: " << (amp_discriminator.enabled ? "true" : "false") << std::endl;
        std::cout << "AMP Discriminator Path: " << amp_discriminator.policy_path << std::endl;
        std::cout << "Reference Motion Source: " << reference_motion_source
                  << ", enabled=" << (enable_reference_motion ? "true" : "false")
                  << std::endl;
        std::cout << "Reference Motion Path: " << reference_motion_path << std::endl;
        std::cout << "Reference Joint Order: " << reference_joint_order.size() << std::endl;
        std::cout << "Source Contract IMU: payload=" << source_contract.imu_input.payload
                  << ", euler_unit=" << source_contract.imu_input.euler_unit
                  << ", quat_order=" << source_contract.imu_input.quat_order
                  << ", sim_base_quat_source_order=" << source_contract.sim_base.quat_source_order
                  << ", canonical_quat_order=" << observation_canonical_contract.quat_order
                  << ", canonical_body_orientation_representation=" << observation_canonical_contract.body_orientation_representation
                  << std::endl;
        if (enable_reference_motion)
        {
            std::cout << "Reference Source Contract: "
                      << "reference_file_body_quat_order=" << source_contract.reference_file.body_quat_order
                      << ", reference_file_body_quat_key=" << source_contract.reference_file.reference_body_quat_w_key
                      << ", policy_extra_body_quat_order=" << source_contract.policy_extra_outputs.body_quat_order
                      << ", policy_extra_body_quat_key=" << source_contract.policy_extra_outputs.reference_body_quat_w_key
                      << std::endl;
        }
        std::cout << "External Obs Inputs: " << external_observations.size() << std::endl;
        std::cout << "ONNX Inputs: " << onnx_inputs.size() << std::endl;
        std::cout << "Logging: enabled=" << (logging.enabled ? "true" : "false")
                  << ", backend=" << logging.backend
                  << ", output_file=" << logging.output_file_path
                  << ", tick_decimation=" << logging.tick.decimation
                  << std::endl;
        std::cout << "Realtime: enabled=" << (realtime.enabled ? "true" : "false")
                  << ", lock_memory=" << (realtime.lock_memory ? "true" : "false")
                  << ", set_affinity=" << (realtime.set_affinity ? "true" : "false")
                  << ", cpu_id=" << realtime.cpu_id
                  << ", use_fifo=" << (realtime.use_fifo_scheduler ? "true" : "false")
                  << ", fifo_priority=" << realtime.fifo_priority
                  << std::endl;
        std::cout << "State Telemetry: enabled=" << (enable_state_telemetry ? "true" : "false")
                  << ", hz=" << state_telemetry_hz
                  << std::endl;
        std::cout << "=============================" << std::endl;
    }
};

#endif // RL_CFG_H
