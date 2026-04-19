#ifndef RL_CFG_H
#define RL_CFG_H

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

inline std::vector<std::string> loadRobotGlobalJointOrderFromYAML(
    const std::string &yaml_file)
{
    const YAML::Node root = YAML::LoadFile(yaml_file);
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

inline void appendUniqueName(
    std::vector<std::string> *out,
    std::unordered_set<std::string> *seen,
    const std::string &name)
{
    if (!out || !seen || name.empty())
    {
        return;
    }
    if (seen->insert(name).second)
    {
        out->push_back(name);
    }
}

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
    std::string body_quat_order = "wxyz";
};

struct SourceContractPolicyExtraOutputs
{
    std::string body_quat_order = "wxyz";
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

    bool auto_start_policy = true;
    double zeroing_duration_s = 2.0;

    bool enable_cmd_watchdog = true;
    double cmd_timeout_s = 0.12;
    int loop_overrun_warn_us = 2000;
    bool enable_state_telemetry = true;
    double state_telemetry_hz = 50.0;

    std::vector<float> tau_limit;
    std::vector<std::string> robot_joint_order;

    std::string data_path;
    bool save_data_flag = false;
    std::string control_mode;
    float action_filter = 0.0f;
    rl_master::runtime::RealtimeConfig realtime;

    class RobotCfg
    {
    public:
        std::vector<std::string> joint_order;
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
            const YAML::Node config = YAML::LoadFile(yaml_file);
            if (!config[config_type])
            {
                throw std::runtime_error("missing config section: " + config_type);
            }

            robotCfg.default_joint_angles.clear();
            robotCfg.zero_joint_angles.clear();
            robotCfg.joint_order.clear();
            robotCfg.joint_limit_range.clear();
            robotCfg.motor_torque_limit.clear();
            sub_models.clear();
            external_observations.clear();
            amp_discriminator = AmpDiscriminatorCfg{};
            onnx_inputs.clear();
            reference_joint_order.clear();
            source_contract = SourceContract{};

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
            source_contract.reference_file.body_quat_order = yamlReadOr<std::string>(
                reference_file_contract_cfg,
                "body_quat_order",
                source_contract.reference_file.body_quat_order);

            const YAML::Node policy_extra_outputs_contract_cfg = source_contract_cfg["policy_extra_outputs"];
            source_contract.policy_extra_outputs.body_quat_order = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "body_quat_order",
                source_contract.policy_extra_outputs.body_quat_order);

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
            if (source_contract.reference_file.body_quat_order != "xyzw" &&
                source_contract.reference_file.body_quat_order != "wxyz")
            {
                throw std::runtime_error("source_contract.reference_file.body_quat_order must be 'xyzw' or 'wxyz'");
            }
            if (source_contract.policy_extra_outputs.body_quat_order != "xyzw" &&
                source_contract.policy_extra_outputs.body_quat_order != "wxyz")
            {
                throw std::runtime_error("source_contract.policy_extra_outputs.body_quat_order must be 'xyzw' or 'wxyz'");
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

            if (robot["zero_joint_angles"])
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
            }

            if (robot["joint_order"])
            {
                robotCfg.joint_order = robot["joint_order"].as<std::vector<std::string>>();
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

            robot_joint_order.clear();
            std::unordered_set<std::string> seen_joint_names;
            seen_joint_names.reserve(
                robotCfg.joint_order.size() +
                robotCfg.default_joint_angles.size() +
                action_joint_order.size() +
                obs_joint_order.size() +
                reference_joint_order.size());

            for (const auto &name : robotCfg.joint_order)
            {
                appendUniqueName(&robot_joint_order, &seen_joint_names, name);
            }
            for (const auto &entry : robotCfg.default_joint_angles)
            {
                appendUniqueName(&robot_joint_order, &seen_joint_names, entry.first);
            }
            for (const auto &name : action_joint_order)
            {
                appendUniqueName(&robot_joint_order, &seen_joint_names, name);
            }
            for (const auto &name : obs_joint_order)
            {
                appendUniqueName(&robot_joint_order, &seen_joint_names, name);
            }
            for (const auto &name : reference_joint_order)
            {
                appendUniqueName(&robot_joint_order, &seen_joint_names, name);
            }
            if (robot_joint_order.empty())
            {
                throw std::runtime_error("failed to resolve robot joint order from config: " + config_type);
            }

            if (!robotCfg.zero_joint_angles.empty())
            {
                std::unordered_set<std::string> known_joint_names;
                known_joint_names.reserve(robot_joint_order.size());
                for (const auto &name : robot_joint_order)
                {
                    known_joint_names.insert(name);
                }

                std::unordered_set<std::string> zero_joint_names;
                zero_joint_names.reserve(robotCfg.zero_joint_angles.size());
                std::vector<std::string> unknown_zero_joints;
                for (const auto &entry : robotCfg.zero_joint_angles)
                {
                    const std::string &name = entry.first;
                    if (!zero_joint_names.insert(name).second)
                    {
                        throw std::runtime_error("robot.zero_joint_angles contains duplicate joint: " + name);
                    }
                    if (known_joint_names.find(name) == known_joint_names.end())
                    {
                        unknown_zero_joints.push_back(name);
                    }
                }

                if (!unknown_zero_joints.empty())
                {
                    std::ostringstream oss;
                    for (size_t i = 0; i < unknown_zero_joints.size(); ++i)
                    {
                        if (i > 0)
                        {
                            oss << ", ";
                        }
                        oss << unknown_zero_joints[i];
                    }
                    throw std::runtime_error(
                        "robot.zero_joint_angles contains unknown joints in " + config_type + ": " + oss.str());
                }

                std::vector<std::string> missing_zero_joints;
                for (const auto &name : robot_joint_order)
                {
                    if (zero_joint_names.find(name) == zero_joint_names.end())
                    {
                        missing_zero_joints.push_back(name);
                    }
                }
                if (!missing_zero_joints.empty())
                {
                    std::ostringstream oss;
                    for (size_t i = 0; i < missing_zero_joints.size(); ++i)
                    {
                        if (i > 0)
                        {
                            oss << ", ";
                        }
                        oss << missing_zero_joints[i];
                    }
                    throw std::runtime_error(
                        "robot.zero_joint_angles missing joints in " + config_type + ": " + oss.str());
                }
            }

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
        std::cout << "RL Control Frequency: " << RL_control_f << std::endl;
        std::cout << "Sub Models: " << sub_models.size() << std::endl;
        std::cout << "AMP Discriminator Enabled: " << (amp_discriminator.enabled ? "true" : "false") << std::endl;
        std::cout << "AMP Discriminator Path: " << amp_discriminator.policy_path << std::endl;
        std::cout << "Reference Motion Source: " << reference_motion_source << std::endl;
        std::cout << "Reference Motion Path: " << reference_motion_path << std::endl;
        std::cout << "Reference Joint Order: " << reference_joint_order.size() << std::endl;
        std::cout << "Source Contract IMU: payload=" << source_contract.imu_input.payload
                  << ", euler_unit=" << source_contract.imu_input.euler_unit
                  << ", quat_order=" << source_contract.imu_input.quat_order
                  << ", sim_base_quat_source_order=" << source_contract.sim_base.quat_source_order
                  << ", reference_file_body_quat_order=" << source_contract.reference_file.body_quat_order
                  << ", policy_extra_body_quat_order=" << source_contract.policy_extra_outputs.body_quat_order
                  << std::endl;
        std::cout << "External Obs Inputs: " << external_observations.size() << std::endl;
        std::cout << "ONNX Inputs: " << onnx_inputs.size() << std::endl;
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
