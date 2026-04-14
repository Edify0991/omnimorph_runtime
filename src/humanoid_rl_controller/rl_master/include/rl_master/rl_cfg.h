#ifndef RL_CFG_H
#define RL_CFG_H

#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
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
        const YAML::Node config = YAML::LoadFile(yaml_file);
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

struct DeployModeProfileSpec
{
    int mode_id = 0;
    std::string config_section;
    std::string tag;
};

inline std::vector<DeployModeProfileSpec> loadDeployModeProfilesFromYAML(
    const std::string &yaml_file)
{
    std::vector<DeployModeProfileSpec> specs;
    const YAML::Node root = YAML::LoadFile(yaml_file);
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
    bool enable_metadata_check = false;
    bool metadata_check_strict = true;
    std::vector<std::string> required_metadata_keys;
    std::map<std::string, std::string> expected_metadata;

    // Disabled when set to a very negative number (default).
    float warn_below = -1.0e9f;
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

    float clip_observations = 100.0f;
    float clip_actions = 100.0f;
    float action_scale = 1.0f;

    std::string policy_path;
    int device_id = 0;

    int obs_dim = 0;
    int action_dim = 0;
    int motor_N = 0;
    int RL_control_f = 100;
    std::vector<std::string> action_joint_order;
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
    std::vector<ExternalObservationSpec> external_observations;

    bool auto_start_policy = true;
    double zeroing_duration_s = 2.0;
    std::vector<float> zero_pose;

    bool enable_cmd_watchdog = true;
    double cmd_timeout_s = 0.12;
    int loop_overrun_warn_us = 2000;

    std::vector<float> tau_limit;

    std::string data_path;
    bool save_data_flag = false;
    std::string control_mode;
    float action_filter = 0.0f;
    rl_master::runtime::RealtimeConfig realtime;

    class RobotCfg
    {
    public:
        std::vector<std::pair<std::string, float>> default_joint_angles;
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
            const YAML::Node config = YAML::LoadFile(yaml_file);
            if (!config[config_type])
            {
                throw std::runtime_error("missing config section: " + config_type);
            }

            robotCfg.default_joint_angles.clear();
            robotCfg.joint_limit_range.clear();
            robotCfg.motor_torque_limit.clear();
            sub_models.clear();
            external_observations.clear();
            amp_discriminator = AmpDiscriminatorCfg{};

            const std::string configured_root_raw = yamlReadOr<std::string>(config, "humanoid_rl_root_dir", "");
            const std::filesystem::path cfg_parent_dir = std::filesystem::path(yaml_file).parent_path();
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
            const YAML::Node cfg = config[config_type];

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

            policy_name = yamlReadOr<std::string>(cfg, "policy_name", "");
            policy_family = yamlReadOr<std::string>(cfg, "policy_family", "amp");
            obs_stack_N = cfg["obs_stack_N"].as<int>();
            cycle_time = cfg["cycle_time"].as<double>();

            kps = cfg["kps"].as<std::vector<float>>();
            kds = cfg["kds"].as<std::vector<float>>();

            clip_observations = cfg["clip_observations"].as<float>();
            clip_actions = cfg["clip_actions"].as<float>();
            action_scale = cfg["action_scale"].as<float>();

            device_id = cfg["device_id"].as<int>();
            obs_dim = cfg["obs_dim"].as<int>();
            action_dim = cfg["action_dim"].as<int>();
            motor_N = cfg["motor_N"].as<int>();
            RL_control_f = cfg["RL_control_f"].as<int>();
            action_joint_order = yamlReadOr<std::vector<std::string>>(cfg, "action_joint_order", {});
            obs_joint_order = yamlReadOr<std::vector<std::string>>(cfg, "obs_joint_order", {});
            if (obs_joint_order.empty())
            {
                obs_joint_order = action_joint_order;
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
            const std::string reference_motion_path_raw = yamlReadOr<std::string>(cfg, "reference_motion_path", "");
            if (!reference_motion_path_raw.empty())
            {
                reference_motion_path = resolvePath(reference_motion_path_raw);
            }
            else
            {
                reference_motion_path = resolvePath(reference_motion_file);
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

            auto_start_policy = yamlReadOr<bool>(cfg, "auto_start_policy", true);
            zeroing_duration_s = yamlReadOr<double>(cfg, "zeroing_duration_s", 2.0);
            zero_pose = yamlReadOr<std::vector<float>>(cfg, "zero_pose", {});

            enable_cmd_watchdog = yamlReadOr<bool>(cfg, "enable_cmd_watchdog", true);
            cmd_timeout_s = yamlReadOr<double>(cfg, "cmd_timeout_s", 0.12);
            loop_overrun_warn_us = yamlReadOr<int>(cfg, "loop_overrun_warn_us", 2000);

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

            tau_limit = cfg["tau_limit"].as<std::vector<float>>();
            save_data_flag = cfg["save_data_flag"].as<bool>();
            control_mode = cfg["control_mode"].as<std::string>();
            action_filter = cfg["action_filter"].as<float>();

            data_path = (std::filesystem::path(humanoid_rl_root_dir) / "data" / (getCurrentTime() + "_" + policy_name)).string();

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
        std::cout << "Control Mode: " << control_mode << std::endl;
        std::cout << "RL Control Frequency: " << RL_control_f << std::endl;
        std::cout << "Sub Models: " << sub_models.size() << std::endl;
        std::cout << "AMP Discriminator Enabled: " << (amp_discriminator.enabled ? "true" : "false") << std::endl;
        std::cout << "AMP Discriminator Path: " << amp_discriminator.policy_path << std::endl;
        std::cout << "Reference Motion Source: " << reference_motion_source << std::endl;
        std::cout << "Reference Motion Path: " << reference_motion_path << std::endl;
        std::cout << "External Obs Inputs: " << external_observations.size() << std::endl;
        std::cout << "Realtime: enabled=" << (realtime.enabled ? "true" : "false")
                  << ", lock_memory=" << (realtime.lock_memory ? "true" : "false")
                  << ", set_affinity=" << (realtime.set_affinity ? "true" : "false")
                  << ", cpu_id=" << realtime.cpu_id
                  << ", use_fifo=" << (realtime.use_fifo_scheduler ? "true" : "false")
                  << ", fifo_priority=" << realtime.fifo_priority
                  << std::endl;
        std::cout << "=============================" << std::endl;
    }
};

#endif // RL_CFG_H
