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

inline bool containsPathVariablePlaceholder(const std::string &raw)
{
    return raw.find("${") != std::string::npos;
}

inline std::vector<std::string> collectPathVariablePlaceholders(const std::string &raw)
{
    std::vector<std::string> keys;
    size_t search_pos = 0;
    while (true)
    {
        const size_t open = raw.find("${", search_pos);
        if (open == std::string::npos)
        {
            break;
        }
        const size_t close = raw.find('}', open + 2);
        if (close == std::string::npos)
        {
            break;
        }
        keys.push_back(raw.substr(open + 2, close - (open + 2)));
        search_pos = close + 1;
    }
    return keys;
}

inline std::string expandPathVariables(
    const std::string &raw,
    const std::map<std::string, std::string> &variables,
    bool strict = true)
{
    std::string out = raw;
    size_t search_pos = 0;
    while (true)
    {
        const size_t open = out.find("${", search_pos);
        if (open == std::string::npos)
        {
            break;
        }
        const size_t close = out.find('}', open + 2);
        if (close == std::string::npos)
        {
            if (strict)
            {
                throw std::runtime_error("unterminated path variable placeholder in: " + raw);
            }
            break;
        }
        const std::string key = out.substr(open + 2, close - (open + 2));
        const auto it = variables.find(key);
        if (it == variables.end())
        {
            if (strict)
            {
                throw std::runtime_error("unknown path variable '${" + key + "}' in: " + raw);
            }
            search_pos = close + 1;
            continue;
        }
        out.replace(open, close - open + 1, it->second);
        search_pos = open + it->second.size();
    }
    return out;
}

inline std::filesystem::path resolveConfiguredOmnimorphRootDir(
    const YAML::Node &root,
    const std::filesystem::path &cfg_parent_dir)
{
    const std::string configured_root_raw =
        yamlReadOr<std::string>(
            root,
            "omnimorph_root_dir",
            yamlReadOr<std::string>(root, "humanoid_rl_root_dir", ""));
    const std::filesystem::path default_root_path = std::filesystem::path(RL_MASTER_ROOT_DIR);
    if (configured_root_raw.empty())
    {
        return default_root_path;
    }

    std::filesystem::path candidate = std::filesystem::path(configured_root_raw);
    if (candidate.is_relative())
    {
        candidate = cfg_parent_dir / candidate;
    }
    std::error_code candidate_exists_ec;
    if (std::filesystem::exists(candidate, candidate_exists_ec))
    {
        return std::filesystem::absolute(candidate);
    }
    return default_root_path;
}

inline std::filesystem::path resolveConfiguredHumanoidRlRootDir(
    const YAML::Node &root,
    const std::filesystem::path &cfg_parent_dir)
{
    return resolveConfiguredOmnimorphRootDir(root, cfg_parent_dir);
}

inline std::map<std::string, std::string> loadPathVariablesFromRootDocument(
    const RootConfigDocument &root_doc)
{
    std::map<std::string, std::string> variables;
    const std::filesystem::path resolved_root =
        resolveConfiguredOmnimorphRootDir(root_doc.root, root_doc.root_dir);
    variables["omnimorph_root_dir"] = resolved_root.string();
    variables["humanoid_rl_root_dir"] = resolved_root.string();
    variables["rl_cfg_dir"] = root_doc.root_dir.string();

    const YAML::Node path_variables_node = root_doc.root["path_variables"];
    if (!path_variables_node)
    {
        return variables;
    }
    if (!path_variables_node.IsMap())
    {
        throw std::runtime_error("path_variables must be a map");
    }

    std::map<std::string, std::string> pending;
    for (auto it = path_variables_node.begin(); it != path_variables_node.end(); ++it)
    {
        const std::string key = it->first.as<std::string>();
        const std::string value = it->second.as<std::string>();
        if (key.empty())
        {
            throw std::runtime_error("path_variables contains an empty key");
        }
        if (value.empty())
        {
            throw std::runtime_error("path_variables['" + key + "'] must not be empty");
        }
        pending[key] = value;
    }

    for (const auto &entry : pending)
    {
        for (const std::string &placeholder_key : collectPathVariablePlaceholders(entry.second))
        {
            if (variables.find(placeholder_key) != variables.end())
            {
                continue;
            }
            if (const char *env_value = std::getenv(placeholder_key.c_str()))
            {
                if (env_value[0] != '\0')
                {
                    variables[placeholder_key] = env_value;
                }
            }
        }
    }

    for (size_t pass = 0; pass < pending.size() + 2 && !pending.empty(); ++pass)
    {
        bool progress = false;
        for (auto it = pending.begin(); it != pending.end();)
        {
            const std::string expanded = expandPathVariables(it->second, variables, false);
            if (containsPathVariablePlaceholder(expanded))
            {
                ++it;
                continue;
            }

            std::filesystem::path value_path = std::filesystem::path(expanded);
            if (value_path.is_relative())
            {
                value_path = root_doc.root_dir / value_path;
            }
            variables[it->first] = value_path.lexically_normal().string();
            it = pending.erase(it);
            progress = true;
        }
        if (!progress)
        {
            break;
        }
    }

    if (!pending.empty())
    {
        auto first = pending.begin();
        throw std::runtime_error(
            "failed to resolve path_variables entry '" + first->first +
            "': unresolved placeholder chain in '" + first->second + "'");
    }

    return variables;
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
    std::error_code profile_exists_ec;
    if (!std::filesystem::exists(profile_path, profile_exists_ec))
    {
        const std::string access_suffix = profile_exists_ec
                                              ? " (" + profile_exists_ec.message() + ")"
                                              : "";
        throw std::runtime_error(
            "profile file not found for config section '" + config_section + "': " +
            profile_path.string() + access_suffix);
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

inline std::vector<std::string> loadRobotGlobalMotorOrderFromYAML(
    const std::string &yaml_file)
{
    const RootConfigDocument doc = loadRootConfigDocument(yaml_file);
    const YAML::Node joint_order = doc.root["robot_global_joint_order"];
    if (!joint_order || !joint_order.IsSequence())
    {
        throw std::runtime_error("robot_global_joint_order is required and must be a sequence");
    }

    const YAML::Node motor_order = doc.root["robot_global_motor_order"];
    if (!motor_order)
    {
        return loadRobotGlobalJointOrderFromYAML(yaml_file);
    }
    if (!motor_order.IsSequence())
    {
        throw std::runtime_error("robot_global_motor_order must be a sequence");
    }

    std::vector<std::string> out;
    std::unordered_set<std::string> seen;
    out.reserve(motor_order.size());
    seen.reserve(motor_order.size());
    for (size_t i = 0; i < motor_order.size(); ++i)
    {
        const std::string name = motor_order[i].as<std::string>();
        if (name.empty())
        {
            throw std::runtime_error("robot_global_motor_order contains empty joint name");
        }
        if (!seen.insert(name).second)
        {
            throw std::runtime_error("robot_global_motor_order contains duplicate joint: " + name);
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
    out.leg = loadJointGroupNamesFromRoot(joint_groups, "leg", false);
    out.arm = loadJointGroupNamesFromRoot(joint_groups, "arm", false);
    out.waist = loadJointGroupNamesFromRoot(joint_groups, "waist", false);
    return out;
}

inline std::string loadRobotKinematicsAdapterFromYAML(
    const std::string &yaml_file)
{
    const YAML::Node root = loadRootConfigDocument(yaml_file).root;
    const YAML::Node robot_identity = root["robot_identity"];
    if (robot_identity && robot_identity["kinematics_adapter"])
    {
        return robot_identity["kinematics_adapter"].as<std::string>();
    }
    return yamlReadOr<std::string>(root, "robot_kinematics_adapter", "jc01");
}

inline std::string resolveDeployConfigSectionForModeFromYAML(
    const std::string &yaml_file,
    int mode_id)
{
    const std::vector<DeployModeProfileSpec> specs = loadDeployModeProfilesFromYAML(yaml_file);
    for (const auto &spec : specs)
    {
        if (spec.mode_id == mode_id && !spec.config_section.empty())
        {
            return spec.config_section;
        }
    }
    throw std::runtime_error(
        "failed to resolve deploy config section for mode_id=" + std::to_string(mode_id));
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
        throw std::runtime_error(
            std::string("invalid integer in environment variable '") + env_key + "'");
    }
}

#define RL_CFG_PATH RL_MASTER_ROOT_DIR "/config/rl_cfg_jc01.yaml"
#define OBS_MANIFEST_PATH RL_MASTER_ROOT_DIR "/config/observation_manifest.yaml"

struct ExternalObservationSpec
{
    std::string name;
    int dim = 0;
    bool required = false;
    std::string topic;
    std::string message_type = "float32_multi_array";
};

struct OnnxInputSpec
{
    std::string name;
    std::string source = "stacked_observation"; // stacked_observation / observation / last_action / time_step / feature / constant
    std::string feature_name;
    std::vector<int64_t> shape;
    std::vector<float> constant;
};

struct SourceContractImuInput
{
    std::string source_type = "sensor_msgs_imu"; // sensor_msgs_imu / unitree_hg_lowstate / unitree_hg_imu_state / unitree_sdk2_lowstate / unitree_sdk2_imu_state
    std::string topic = "/imu/yesense";
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
    std::string velocity_source = "freejoint_qvel"; // freejoint_qvel / body_object_velocity_local / body_object_velocity_root_local / body_cvel
};

struct GaitConfig
{
    float gait_air_ratio_l = 0.0f;
    float gait_air_ratio_r = 0.0f;
    float gait_phase_offset_l = 0.0f;
    float gait_phase_offset_r = 0.0f;
    double gait_cycle = 1.0;
};

struct SourceContractBaseVelocityEstimator
{
    bool enabled = false;
    bool use_imu_prediction = true;
    std::string imu_accel_frame = "body"; // body / world
    bool imu_accel_includes_gravity = true;
    float gravity_mps2 = 9.80665f;
    bool use_input_velocity_measurement = true;
    float input_velocity_measurement_noise = 0.02f;
    bool use_odom_velocity_measurement = false;
    std::string odom_topic = "/glim/odom";
    std::string odom_velocity_frame = "world"; // world / body
    float odom_velocity_measurement_noise = 0.02f;
    bool zero_velocity_update = true;
    float zero_velocity_measurement_noise = 0.01f;
    float stationary_joint_velocity_threshold = 0.03f;
    float stationary_ang_vel_threshold = 0.12f;
    float stationary_accel_norm_tolerance = 0.5f;
    float initial_variance = 0.25f;
    float process_noise = 0.05f;
    float accel_noise = 0.2f;
    float min_dt = 1.0e-4f;
    float max_dt = 0.05f;
    bool reset_on_mode_switch = true;
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
    std::string reference_joint_order_metadata_key = "command_joint_names";
    bool preserve_reference_joint_order = false;
    std::string reference_body_pos_w_key = "body_pos_w";
    std::string reference_body_quat_w_key = "body_quat_w";
    std::string reference_body_lin_vel_w_key = "body_lin_vel_w";
    std::string reference_body_ang_vel_w_key = "body_ang_vel_w";
    std::string body_quat_order = "wxyz";
    std::string body_quat_representation = "quat";
    std::string body_quat_frame = "world";
};

struct SourceContractUnitreeSdk2
{
    int domain_id = 0;
    std::string network_interface;
    std::string lowcmd_topic = "rt/lowcmd";
    std::string lowstate_topic = "rt/lowstate";
    std::string imu_topic = "rt/secondary_imu";
    int queue_len = 1;
    int writer_period_us = 2000;
    int mode_pr = 0;
    float lowstate_timeout_sec = 0.1f;
    float default_lower_kp = 100.0f;
    float default_lower_kd = 1.0f;
    float default_upper_kp = 50.0f;
    float default_upper_kd = 1.0f;
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
    SourceContractBaseVelocityEstimator base_velocity_estimator;
    SourceContractReferenceFile reference_file;
    SourceContractPolicyExtraOutputs policy_extra_outputs;
    SourceContractUnitreeSdk2 unitree_sdk2;
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
    RuntimeLogSourceSamplesConfig source_samples;
    RuntimeLogExportConfig export_config;
    std::string session_base_path;
    std::string output_file_path;
};

class Sim2realCfg
{
public:
    class SimPaceMotorCfg
    {
    public:
        bool enabled = false;
        int max_delay = 0;
        double delay_step_dt_s = 0.005;
        std::map<std::string, float> armature;
        std::map<std::string, float> frictionloss;
        std::map<std::string, float> encoder_bias;
        std::map<std::string, int> delay;
    };

    class SimDcMotorCfg
    {
    public:
        bool enabled = false;
        float saturation_effort = 0.0f;
        float effort_limit = 0.0f;
        std::map<std::string, float> velocity_limit;
    };

    class CommandLimitsCfg
    {
    public:
        bool enabled = false;
        float vx_min = 0.0f;
        float vx_max = 0.0f;
        float vy_min = 0.0f;
        float vy_max = 0.0f;
        float dyaw_min = 0.0f;
        float dyaw_max = 0.0f;
    };

    class AutoSwitchOnReferenceEndCfg
    {
    public:
        bool enabled = false;
        int target_mode_id = -1;
        int total_steps = -1;
        std::vector<std::string> metadata_keys = {
            "motion_num_frames",
            "reference_frame_count",
            "reference_frames",
            "motion_frame_count",
            "frame_count",
            "num_frames",
            "sequence_length",
            "total_steps",
        };
    };

    std::string omnimorph_root_dir;
    std::string humanoid_rl_root_dir;
    std::string robot_id;
    std::string robot_morphology;
    std::string robot_vendor;
    std::string robot_kinematics_adapter = "jc01";
    std::string motor_io_backend = "shm";
    std::string external_hardware_bridge;
    std::string policy_name;
    std::string policy_family = "amp"; // amp / beyondmimic / custom
    std::string policy_adapter = "onnx";
    std::string inference_strategy = "sync_step"; // sync_step / chunked_receding
    std::string action_output_layout = "step_flat"; // step_flat / chunk_flat
    int action_chunk_steps = 1;
    int action_chunk_execute_steps = 1;
    int action_chunk_replan_interval = 1;
    int obs_stack_N = 1;
    std::string observation_stack_layout = "frame_major";
    double cycle_time = 1.0;

    std::vector<float> kps;
    std::vector<float> kds;
    std::vector<float> tau_limit;
    std::map<std::string, float> named_kps;
    std::map<std::string, float> named_kds;
    std::map<std::string, float> named_tau_limit;
    std::vector<float> action_scales;
    std::map<std::string, float> named_action_scales;

    float clip_observations = 100.0f;
    float clip_actions = 100.0f;
    float action_scale = 1.0f;
    std::string action_clip_stage = "raw_action"; // raw_action / target_delta / target_q
    float target_delta_clip = 0.0f;
    float target_q_clip = 0.0f;
    bool clamp_target_q_to_joint_limits = false;
    float target_q_joint_limit_margin = 0.0f;

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
    bool advance_time_step_on_reference_prefetch = false;
    std::vector<std::string> extra_output_names;
    std::vector<OnnxInputSpec> onnx_inputs;
    bool enable_metadata_check = false;
    bool metadata_check_strict = true;
    std::vector<std::string> required_metadata_keys;
    std::map<std::string, std::string> expected_metadata;
    std::vector<PolicySubModelCfg> sub_models;

    bool enable_reference_motion = false;
    int reference_motion_dim = 0;
    std::string reference_motion_file;
    std::string reference_motion_path;
    std::string reference_motion_sampling = "phase"; // phase / step
    std::string reference_motion_source = "auto";    // auto / file / policy_outputs
    std::string reference_anchor_body = "base";
    std::string motion_reference_alignment = "current_robot_anchor"; // current_robot_anchor / startup_anchor_pos_yaw
    std::vector<std::string> reference_body_names;
    std::vector<std::string> reference_joint_order;
    std::string pinocchio_urdf_file;
    std::string pinocchio_urdf_path;
    std::vector<ExternalObservationSpec> external_observations;
    SourceContract source_contract;
    ObservationCanonicalContract observation_canonical_contract;
    SimPaceMotorCfg sim_pace_motor;
    SimDcMotorCfg sim_dc_motor;
    CommandLimitsCfg command_limits;
    AutoSwitchOnReferenceEndCfg auto_switch_on_reference_end;
    GaitConfig gait;

    std::string startup_completion_action = "hold";
    int policy_startup_warmup_steps = 0;
    bool prefill_observation_history_on_running_start = false;
    bool seed_running_start_observation_from_reference = false;
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
        float command_lin_vel = 1.0f;
        float command_ang_vel = 1.0f;
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
            onnx_inputs.clear();
            reference_joint_order.clear();
            named_kps.clear();
            named_kds.clear();
            named_tau_limit.clear();
            source_contract = SourceContract{};
            observation_canonical_contract = ObservationCanonicalContract{};
            sim_pace_motor = SimPaceMotorCfg{};
            sim_dc_motor = SimDcMotorCfg{};
            command_limits = CommandLimitsCfg{};
            gait = GaitConfig{};
            logging = RuntimeLoggingConfig{};

            const std::filesystem::path cfg_parent_dir = config_doc.root_doc.root_dir;
            const std::filesystem::path default_root_path = std::filesystem::path(RL_MASTER_ROOT_DIR);
            const std::filesystem::path resolved_root_path =
                resolveConfiguredOmnimorphRootDir(config_doc.root_doc.root, cfg_parent_dir);
            if (!config["omnimorph_root_dir"] && !config["humanoid_rl_root_dir"])
            {
                std::cerr << "[Sim2realCfg] warning: missing omnimorph_root_dir, fallback to RL_MASTER_ROOT_DIR: "
                          << default_root_path << std::endl;
            }
            else if (resolved_root_path == default_root_path)
            {
                std::cerr << "[Sim2realCfg] warning: configured omnimorph_root_dir is invalid or inaccessible, "
                          << "fallback to RL_MASTER_ROOT_DIR: " << default_root_path << std::endl;
            }
            omnimorph_root_dir = resolved_root_path.string();
            humanoid_rl_root_dir = omnimorph_root_dir;
            const YAML::Node robot_identity = config["robot_identity"];
            robot_id = yamlReadOr<std::string>(robot_identity, "id", "");
            robot_morphology = yamlReadOr<std::string>(robot_identity, "morphology", "");
            robot_vendor = yamlReadOr<std::string>(robot_identity, "vendor", "");
            robot_kinematics_adapter = yamlReadOr<std::string>(
                robot_identity,
                "kinematics_adapter",
                yamlReadOr<std::string>(config, "robot_kinematics_adapter", "jc01"));
            external_hardware_bridge = yamlReadOr<std::string>(
                robot_identity,
                "external_hardware_bridge",
                "");
            motor_io_backend = yamlReadOr<std::string>(
                robot_identity,
                "motor_io_backend",
                yamlReadOr<std::string>(config, "motor_io_backend", ""));
            if (motor_io_backend.empty())
            {
                if (external_hardware_bridge == "unitree_sdk_bridge")
                {
                    motor_io_backend = "unitree_g1_sdk2";
                }
                else
                {
                    motor_io_backend = "shm";
                }
            }
            const std::map<std::string, std::string> path_variables =
                loadPathVariablesFromRootDocument(config_doc.root_doc);

            auto resolvePath = [&](const std::string &raw) -> std::string {
                if (raw.empty())
                {
                    return "";
                }
                const std::string expanded = expandPathVariables(raw, path_variables, true);
                const std::filesystem::path p(expanded);
                if (p.is_absolute())
                {
                    return p.lexically_normal().string();
                }
                return (std::filesystem::path(omnimorph_root_dir) / p).lexically_normal().string();
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
                logging.output_dir = (std::filesystem::path(omnimorph_root_dir) / "data").string();
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
                        spec.source != "last_action" &&
                        spec.source != "time_step" &&
                        spec.source != "feature" &&
                        spec.source != "constant")
                    {
                        throw std::runtime_error(item_name + " unsupported source: " + spec.source);
                    }
                    if (spec.source == "feature" && spec.feature_name.empty())
                    {
                        throw std::runtime_error(item_name + " requires feature_name when source=feature");
                    }
                    if (spec.source == "constant" && spec.constant.empty())
                    {
                        throw std::runtime_error(item_name + " constant source requires values");
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
            policy_adapter = yamlReadOr<std::string>(cfg, "policy_adapter", "onnx");
            inference_strategy = yamlReadOr<std::string>(cfg, "inference_strategy", "sync_step");
            action_output_layout = yamlReadOr<std::string>(cfg, "action_output_layout", "step_flat");
            action_chunk_steps = yamlReadOr<int>(cfg, "action_chunk_steps", 1);
            action_chunk_execute_steps = yamlReadOr<int>(cfg, "action_chunk_execute_steps", 1);
            action_chunk_replan_interval = yamlReadOr<int>(
                cfg,
                "action_chunk_replan_interval",
                action_chunk_execute_steps);
            obs_stack_N = cfg["obs_stack_N"].as<int>();
            observation_stack_layout = yamlReadOr<std::string>(
                cfg,
                "observation_stack_layout",
                "frame_major");
            std::transform(
                observation_stack_layout.begin(),
                observation_stack_layout.end(),
                observation_stack_layout.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (observation_stack_layout != "frame_major" &&
                observation_stack_layout != "term_major")
            {
                throw std::runtime_error(
                    "observation_stack_layout must be one of: frame_major, term_major");
            }
            cycle_time = cfg["cycle_time"].as<double>();

            clip_observations = cfg["clip_observations"].as<float>();
            clip_actions = cfg["clip_actions"].as<float>();
            const bool has_scalar_action_scale = static_cast<bool>(cfg["action_scale"]);
            action_scale = yamlReadOr<float>(cfg, "action_scale", 1.0f);
            action_clip_stage = yamlReadOr<std::string>(cfg, "action_clip_stage", "raw_action");
            std::transform(
                action_clip_stage.begin(),
                action_clip_stage.end(),
                action_clip_stage.begin(),
                [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (action_clip_stage != "raw_action" &&
                action_clip_stage != "target_delta" &&
                action_clip_stage != "target_q")
            {
                throw std::runtime_error("action_clip_stage must be one of: raw_action, target_delta, target_q");
            }
            target_delta_clip = yamlReadOr<float>(cfg, "target_delta_clip", 0.0f);
            if (target_delta_clip < 0.0f)
            {
                throw std::runtime_error("target_delta_clip must be >= 0");
            }
            if (action_clip_stage == "target_delta" && target_delta_clip <= 0.0f)
            {
                throw std::runtime_error("target_delta_clip must be > 0 when action_clip_stage is target_delta");
            }
            target_q_clip = yamlReadOr<float>(cfg, "target_q_clip", 0.0f);
            if (target_q_clip < 0.0f)
            {
                throw std::runtime_error("target_q_clip must be >= 0");
            }
            if (action_clip_stage == "target_q" && target_q_clip <= 0.0f)
            {
                throw std::runtime_error("target_q_clip must be > 0 when action_clip_stage is target_q");
            }
            clamp_target_q_to_joint_limits = yamlReadOr<bool>(cfg, "clamp_target_q_to_joint_limits", false);
            target_q_joint_limit_margin = yamlReadOr<float>(cfg, "target_q_joint_limit_margin", 0.0f);
            if (target_q_joint_limit_margin < 0.0f)
            {
                throw std::runtime_error("target_q_joint_limit_margin must be >= 0");
            }
            const YAML::Node command_limits_cfg = cfg["command_limits"];
            if (command_limits_cfg)
            {
                if (!command_limits_cfg.IsMap())
                {
                    throw std::runtime_error("command_limits must be a map when provided");
                }
                command_limits.enabled = yamlReadOr<bool>(command_limits_cfg, "enabled", false);
                auto readRange = [&](const char *key, float *min_value, float *max_value) {
                    const YAML::Node range_node = command_limits_cfg[key];
                    if (!range_node)
                    {
                        if (command_limits.enabled)
                        {
                            throw std::runtime_error(std::string("command_limits.") + key + " is required when command_limits.enabled is true");
                        }
                        return;
                    }
                    if (!range_node.IsSequence() || range_node.size() != 2)
                    {
                        throw std::runtime_error(std::string("command_limits.") + key + " must be a [min, max] sequence");
                    }
                    *min_value = range_node[0].as<float>();
                    *max_value = range_node[1].as<float>();
                    if (*min_value > *max_value)
                    {
                        throw std::runtime_error(std::string("command_limits.") + key + " min must be <= max");
                    }
                };
                readRange("vx", &command_limits.vx_min, &command_limits.vx_max);
                readRange("vy", &command_limits.vy_min, &command_limits.vy_max);
                readRange("dyaw", &command_limits.dyaw_min, &command_limits.dyaw_max);
            }

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
            if (policy_adapter != "onnx")
            {
                throw std::runtime_error("policy_adapter currently only supports 'onnx'");
            }
            if (inference_strategy != "sync_step" &&
                inference_strategy != "chunked_receding")
            {
                throw std::runtime_error(
                    "inference_strategy must be one of: sync_step, chunked_receding");
            }
            if (action_output_layout != "step_flat" &&
                action_output_layout != "chunk_flat")
            {
                throw std::runtime_error(
                    "action_output_layout must be one of: step_flat, chunk_flat");
            }
            if (action_chunk_steps <= 0)
            {
                throw std::runtime_error("action_chunk_steps must be > 0");
            }
            if (action_chunk_execute_steps <= 0)
            {
                throw std::runtime_error("action_chunk_execute_steps must be > 0");
            }
            if (action_chunk_execute_steps > action_chunk_steps)
            {
                throw std::runtime_error(
                    "action_chunk_execute_steps must be <= action_chunk_steps");
            }
            if (action_chunk_replan_interval <= 0)
            {
                throw std::runtime_error("action_chunk_replan_interval must be > 0");
            }
            if (action_chunk_replan_interval > action_chunk_execute_steps)
            {
                throw std::runtime_error(
                    "action_chunk_replan_interval must be <= action_chunk_execute_steps");
            }
            if (inference_strategy == "chunked_receding" && action_chunk_steps <= 1)
            {
                throw std::runtime_error(
                    "chunked_receding inference_strategy requires action_chunk_steps > 1");
            }
            if (inference_strategy == "chunked_receding" &&
                action_output_layout != "chunk_flat")
            {
                throw std::runtime_error(
                    "chunked_receding inference_strategy requires action_output_layout=chunk_flat");
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
                for (const auto &entry : value_map)
                {
                    const std::string &joint_name = entry.first;
                    const bool in_action = action_joint_names.find(joint_name) != action_joint_names.end();
                    if (!in_action)
                    {
                        throw std::runtime_error(
                            field_name + " contains joint not present in action_joint_order: " +
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
            named_action_scales = yamlReadFloatMapOr(cfg, "action_scales");
            if (named_action_scales.empty() && !has_scalar_action_scale)
            {
                throw std::runtime_error(
                    "either action_scale or action_scales must be provided");
            }
            kps.clear();
            kds.clear();
            tau_limit.clear();
            action_scales.clear();
            kps.reserve(action_joint_order.size());
            kds.reserve(action_joint_order.size());
            tau_limit.reserve(action_joint_order.size());
            action_scales.reserve(action_joint_order.size());
            if (!named_action_scales.empty())
            {
                validateNamedActionJointValueMap(named_action_scales, "action_scales");
            }
            for (const auto &joint_name : action_joint_order)
            {
                kps.push_back(named_kps.at(joint_name));
                kds.push_back(named_kds.at(joint_name));
                tau_limit.push_back(named_tau_limit.at(joint_name));
                action_scales.push_back(
                    named_action_scales.empty() ? action_scale : named_action_scales.at(joint_name));
            }

            const YAML::Node sim_pace_cfg = cfg["sim_pace_motor"];
            if (sim_pace_cfg)
            {
                if (!sim_pace_cfg.IsMap())
                {
                    throw std::runtime_error("sim_pace_motor must be a map when provided");
                }
                sim_pace_motor.enabled = yamlReadOr<bool>(sim_pace_cfg, "enabled", false);
                sim_pace_motor.max_delay = yamlReadOr<int>(sim_pace_cfg, "max_delay", 0);
                sim_pace_motor.delay_step_dt_s = yamlReadOr<double>(sim_pace_cfg, "delay_step_dt_s", 0.005);
                if (sim_pace_motor.max_delay < 0)
                {
                    throw std::runtime_error("sim_pace_motor.max_delay must be >= 0");
                }
                if (sim_pace_motor.delay_step_dt_s <= 0.0)
                {
                    throw std::runtime_error("sim_pace_motor.delay_step_dt_s must be > 0");
                }
                sim_pace_motor.armature = yamlReadFloatMapOr(sim_pace_cfg, "armature");
                sim_pace_motor.frictionloss = yamlReadFloatMapOr(sim_pace_cfg, "frictionloss");
                sim_pace_motor.encoder_bias = yamlReadFloatMapOr(sim_pace_cfg, "encoder_bias");
                const YAML::Node delay_node = sim_pace_cfg["delay"];
                if (delay_node)
                {
                    if (!delay_node.IsMap())
                    {
                        throw std::runtime_error("sim_pace_motor.delay must be a joint-name map");
                    }
                    for (auto it = delay_node.begin(); it != delay_node.end(); ++it)
                    {
                        const std::string joint_name = it->first.as<std::string>();
                        const int delay_steps = it->second.as<int>();
                        if (delay_steps < 0 || delay_steps > sim_pace_motor.max_delay)
                        {
                            throw std::runtime_error(
                                "sim_pace_motor.delay for joint '" + joint_name +
                                "' must be in [0, max_delay]");
                        }
                        sim_pace_motor.delay[joint_name] = delay_steps;
                    }
                }
                auto validateOptionalActionJointMap = [&](const auto &value_map, const std::string &field_name) {
                    std::unordered_set<std::string> action_joint_names(
                        action_joint_order.begin(),
                        action_joint_order.end());
                    for (const auto &entry : value_map)
                    {
                        if (action_joint_names.find(entry.first) == action_joint_names.end())
                        {
                            throw std::runtime_error(
                                "sim_pace_motor." + field_name +
                                " contains joint not present in action_joint_order: " + entry.first);
                        }
                    }
                };
                validateOptionalActionJointMap(sim_pace_motor.armature, "armature");
                validateOptionalActionJointMap(sim_pace_motor.frictionloss, "frictionloss");
                validateOptionalActionJointMap(sim_pace_motor.encoder_bias, "encoder_bias");
                validateOptionalActionJointMap(sim_pace_motor.delay, "delay");
            }

            const YAML::Node sim_dc_cfg = cfg["sim_dc_motor"];
            if (sim_dc_cfg)
            {
                if (!sim_dc_cfg.IsMap())
                {
                    throw std::runtime_error("sim_dc_motor must be a map when provided");
                }
                sim_dc_motor.enabled = yamlReadOr<bool>(sim_dc_cfg, "enabled", false);
                sim_dc_motor.saturation_effort = yamlReadOr<float>(sim_dc_cfg, "saturation_effort", 0.0f);
                sim_dc_motor.effort_limit = yamlReadOr<float>(sim_dc_cfg, "effort_limit", 0.0f);
                sim_dc_motor.velocity_limit = yamlReadFloatMapOr(sim_dc_cfg, "velocity_limit");
                if (sim_dc_motor.enabled)
                {
                    if (sim_dc_motor.saturation_effort <= 0.0f)
                    {
                        throw std::runtime_error("sim_dc_motor.saturation_effort must be > 0 when enabled");
                    }
                    if (sim_dc_motor.effort_limit <= 0.0f)
                    {
                        throw std::runtime_error("sim_dc_motor.effort_limit must be > 0 when enabled");
                    }
                    validateNamedActionJointValueMap(sim_dc_motor.velocity_limit, "sim_dc_motor.velocity_limit");
                    for (const auto &entry : sim_dc_motor.velocity_limit)
                    {
                        if (entry.second <= 0.0f)
                        {
                            throw std::runtime_error(
                                "sim_dc_motor.velocity_limit for joint '" + entry.first + "' must be > 0");
                        }
                    }
                }
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
                policy_path = (std::filesystem::path(omnimorph_root_dir) / "policies" / (policy_name + ".onnx")).string();
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
                observation_manifest_path = (std::filesystem::path(omnimorph_root_dir) / "config" / observation_manifest_file).string();
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
            advance_time_step_on_reference_prefetch =
                yamlReadOr<bool>(policy_io_cfg, "advance_time_step_on_reference_prefetch", false);
            extra_output_names = yamlReadOr<std::vector<std::string>>(policy_io_cfg, "extra_output_names", {});
            onnx_inputs = parseOnnxInputs(policy_io_cfg["onnx_inputs"]);
            validateOnnxInputs(onnx_inputs, config_type + ".policy_io");
            enable_metadata_check = yamlReadOr<bool>(policy_io_cfg, "enable_metadata_check", false);
            metadata_check_strict = yamlReadOr<bool>(policy_io_cfg, "metadata_check_strict", true);
            required_metadata_keys =
                yamlReadOr<std::vector<std::string>>(policy_io_cfg, "required_metadata_keys", {});
            expected_metadata = yamlReadStringMapOr(policy_io_cfg, "expected_metadata");
            if (cfg["auto_switch_on_reference_end"])
            {
                const YAML::Node auto_switch_cfg = cfg["auto_switch_on_reference_end"];
                auto_switch_on_reference_end.enabled =
                    yamlReadOr<bool>(auto_switch_cfg, "enabled", false);
                auto_switch_on_reference_end.target_mode_id =
                    yamlReadOr<int>(auto_switch_cfg, "target_mode_id", -1);
                auto_switch_on_reference_end.total_steps =
                    yamlReadOr<int>(auto_switch_cfg, "total_steps", -1);
                auto_switch_on_reference_end.metadata_keys =
                    yamlReadOr<std::vector<std::string>>(
                        auto_switch_cfg,
                        "metadata_keys",
                        auto_switch_on_reference_end.metadata_keys);
            }

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

            enable_reference_motion = yamlReadOr<bool>(cfg, "enable_reference_motion", false);
            reference_motion_dim = yamlReadOr<int>(cfg, "reference_motion_dim", 0);
            reference_motion_file = yamlReadOr<std::string>(cfg, "reference_motion_file", "");
            reference_motion_sampling = yamlReadOr<std::string>(cfg, "reference_motion_sampling", "phase");
            reference_motion_source = yamlReadOr<std::string>(cfg, "reference_motion_source", "auto");
            reference_anchor_body = yamlReadOr<std::string>(cfg, "reference_anchor_body", "base");
            motion_reference_alignment = yamlReadOr<std::string>(
                cfg,
                "motion_reference_alignment",
                motion_reference_alignment);
            reference_body_names = yamlReadOr<std::vector<std::string>>(cfg, "reference_body_names", {});
            reference_joint_order = yamlReadOr<std::vector<std::string>>(cfg, "reference_joint_order", {});
            pinocchio_urdf_file = yamlReadOr<std::string>(cfg, "pinocchio_urdf_file", "");
            const std::string pinocchio_urdf_path_raw =
                yamlReadOr<std::string>(cfg, "pinocchio_urdf_path", "");
            const std::string reference_motion_path_raw = yamlReadOr<std::string>(cfg, "reference_motion_path", "");
            if (!reference_motion_path_raw.empty())
            {
                reference_motion_path = resolvePath(reference_motion_path_raw);
            }
            else
            {
                reference_motion_path = resolvePath(reference_motion_file);
            }
            if (!pinocchio_urdf_path_raw.empty())
            {
                pinocchio_urdf_path = resolvePath(pinocchio_urdf_path_raw);
            }
            else if (!pinocchio_urdf_file.empty())
            {
                pinocchio_urdf_path = resolvePath(pinocchio_urdf_file);
            }

            if (reference_joint_order.empty())
            {
                reference_joint_order = action_joint_order;
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
            source_contract.imu_input.source_type = yamlReadOr<std::string>(
                imu_contract_cfg, "source_type", source_contract.imu_input.source_type);
            source_contract.imu_input.topic = yamlReadOr<std::string>(
                imu_contract_cfg, "topic", source_contract.imu_input.topic);
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

            const YAML::Node unitree_sdk2_cfg = source_contract_cfg["unitree_sdk2"];
            source_contract.unitree_sdk2.domain_id = yamlReadOr<int>(
                unitree_sdk2_cfg, "domain_id", source_contract.unitree_sdk2.domain_id);
            source_contract.unitree_sdk2.network_interface = yamlReadOr<std::string>(
                unitree_sdk2_cfg, "network_interface", source_contract.unitree_sdk2.network_interface);
            source_contract.unitree_sdk2.lowcmd_topic = yamlReadOr<std::string>(
                unitree_sdk2_cfg, "lowcmd_topic", source_contract.unitree_sdk2.lowcmd_topic);
            source_contract.unitree_sdk2.lowstate_topic = yamlReadOr<std::string>(
                unitree_sdk2_cfg, "lowstate_topic", source_contract.unitree_sdk2.lowstate_topic);
            source_contract.unitree_sdk2.imu_topic = yamlReadOr<std::string>(
                unitree_sdk2_cfg, "imu_topic", source_contract.unitree_sdk2.imu_topic);
            source_contract.unitree_sdk2.queue_len = yamlReadOr<int>(
                unitree_sdk2_cfg, "queue_len", source_contract.unitree_sdk2.queue_len);
            source_contract.unitree_sdk2.writer_period_us = yamlReadOr<int>(
                unitree_sdk2_cfg, "writer_period_us", source_contract.unitree_sdk2.writer_period_us);
            source_contract.unitree_sdk2.mode_pr = yamlReadOr<int>(
                unitree_sdk2_cfg, "mode_pr", source_contract.unitree_sdk2.mode_pr);
            source_contract.unitree_sdk2.lowstate_timeout_sec = yamlReadOr<float>(
                unitree_sdk2_cfg, "lowstate_timeout_sec", source_contract.unitree_sdk2.lowstate_timeout_sec);
            source_contract.unitree_sdk2.default_lower_kp = yamlReadOr<float>(
                unitree_sdk2_cfg, "default_lower_kp", source_contract.unitree_sdk2.default_lower_kp);
            source_contract.unitree_sdk2.default_lower_kd = yamlReadOr<float>(
                unitree_sdk2_cfg, "default_lower_kd", source_contract.unitree_sdk2.default_lower_kd);
            source_contract.unitree_sdk2.default_upper_kp = yamlReadOr<float>(
                unitree_sdk2_cfg, "default_upper_kp", source_contract.unitree_sdk2.default_upper_kp);
            source_contract.unitree_sdk2.default_upper_kd = yamlReadOr<float>(
                unitree_sdk2_cfg, "default_upper_kd", source_contract.unitree_sdk2.default_upper_kd);

            const YAML::Node sim_base_contract_cfg = source_contract_cfg["sim_base"];
            source_contract.sim_base.quat_source_order = yamlReadOr<std::string>(
                sim_base_contract_cfg, "quat_source_order", source_contract.sim_base.quat_source_order);
            source_contract.sim_base.velocity_source = yamlReadOr<std::string>(
                sim_base_contract_cfg, "velocity_source", source_contract.sim_base.velocity_source);

            const YAML::Node base_velocity_estimator_cfg = source_contract_cfg["base_velocity_estimator"];
            source_contract.base_velocity_estimator.enabled = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "enabled", source_contract.base_velocity_estimator.enabled);
            source_contract.base_velocity_estimator.use_imu_prediction = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "use_imu_prediction", source_contract.base_velocity_estimator.use_imu_prediction);
            source_contract.base_velocity_estimator.imu_accel_frame = yamlReadOr<std::string>(
                base_velocity_estimator_cfg, "imu_accel_frame", source_contract.base_velocity_estimator.imu_accel_frame);
            source_contract.base_velocity_estimator.imu_accel_includes_gravity = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "imu_accel_includes_gravity", source_contract.base_velocity_estimator.imu_accel_includes_gravity);
            source_contract.base_velocity_estimator.gravity_mps2 = yamlReadOr<float>(
                base_velocity_estimator_cfg, "gravity_mps2", source_contract.base_velocity_estimator.gravity_mps2);
            source_contract.base_velocity_estimator.use_input_velocity_measurement = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "use_input_velocity_measurement", source_contract.base_velocity_estimator.use_input_velocity_measurement);
            source_contract.base_velocity_estimator.input_velocity_measurement_noise = yamlReadOr<float>(
                base_velocity_estimator_cfg, "input_velocity_measurement_noise", source_contract.base_velocity_estimator.input_velocity_measurement_noise);
            source_contract.base_velocity_estimator.use_odom_velocity_measurement = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "use_odom_velocity_measurement", source_contract.base_velocity_estimator.use_odom_velocity_measurement);
            source_contract.base_velocity_estimator.odom_topic = yamlReadOr<std::string>(
                base_velocity_estimator_cfg, "odom_topic", source_contract.base_velocity_estimator.odom_topic);
            source_contract.base_velocity_estimator.odom_velocity_frame = yamlReadOr<std::string>(
                base_velocity_estimator_cfg, "odom_velocity_frame", source_contract.base_velocity_estimator.odom_velocity_frame);
            source_contract.base_velocity_estimator.odom_velocity_measurement_noise = yamlReadOr<float>(
                base_velocity_estimator_cfg, "odom_velocity_measurement_noise", source_contract.base_velocity_estimator.odom_velocity_measurement_noise);
            source_contract.base_velocity_estimator.zero_velocity_update = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "zero_velocity_update", source_contract.base_velocity_estimator.zero_velocity_update);
            source_contract.base_velocity_estimator.zero_velocity_measurement_noise = yamlReadOr<float>(
                base_velocity_estimator_cfg, "zero_velocity_measurement_noise", source_contract.base_velocity_estimator.zero_velocity_measurement_noise);
            source_contract.base_velocity_estimator.stationary_joint_velocity_threshold = yamlReadOr<float>(
                base_velocity_estimator_cfg, "stationary_joint_velocity_threshold", source_contract.base_velocity_estimator.stationary_joint_velocity_threshold);
            source_contract.base_velocity_estimator.stationary_ang_vel_threshold = yamlReadOr<float>(
                base_velocity_estimator_cfg, "stationary_ang_vel_threshold", source_contract.base_velocity_estimator.stationary_ang_vel_threshold);
            source_contract.base_velocity_estimator.stationary_accel_norm_tolerance = yamlReadOr<float>(
                base_velocity_estimator_cfg, "stationary_accel_norm_tolerance", source_contract.base_velocity_estimator.stationary_accel_norm_tolerance);
            source_contract.base_velocity_estimator.initial_variance = yamlReadOr<float>(
                base_velocity_estimator_cfg, "initial_variance", source_contract.base_velocity_estimator.initial_variance);
            source_contract.base_velocity_estimator.process_noise = yamlReadOr<float>(
                base_velocity_estimator_cfg, "process_noise", source_contract.base_velocity_estimator.process_noise);
            source_contract.base_velocity_estimator.accel_noise = yamlReadOr<float>(
                base_velocity_estimator_cfg, "accel_noise", source_contract.base_velocity_estimator.accel_noise);
            source_contract.base_velocity_estimator.min_dt = yamlReadOr<float>(
                base_velocity_estimator_cfg, "min_dt", source_contract.base_velocity_estimator.min_dt);
            source_contract.base_velocity_estimator.max_dt = yamlReadOr<float>(
                base_velocity_estimator_cfg, "max_dt", source_contract.base_velocity_estimator.max_dt);
            source_contract.base_velocity_estimator.reset_on_mode_switch = yamlReadOr<bool>(
                base_velocity_estimator_cfg, "reset_on_mode_switch", source_contract.base_velocity_estimator.reset_on_mode_switch);

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
            source_contract.policy_extra_outputs.reference_joint_order_metadata_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_joint_order_metadata_key",
                source_contract.policy_extra_outputs.reference_joint_order_metadata_key);
            source_contract.policy_extra_outputs.preserve_reference_joint_order = yamlReadOr<bool>(
                policy_extra_outputs_contract_cfg,
                "preserve_reference_joint_order",
                source_contract.policy_extra_outputs.preserve_reference_joint_order);
            source_contract.policy_extra_outputs.reference_body_pos_w_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_body_pos_w_key",
                source_contract.policy_extra_outputs.reference_body_pos_w_key);
            source_contract.policy_extra_outputs.reference_body_quat_w_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_body_quat_w_key",
                source_contract.policy_extra_outputs.reference_body_quat_w_key);
            source_contract.policy_extra_outputs.reference_body_lin_vel_w_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_body_lin_vel_w_key",
                source_contract.policy_extra_outputs.reference_body_lin_vel_w_key);
            source_contract.policy_extra_outputs.reference_body_ang_vel_w_key = yamlReadOr<std::string>(
                policy_extra_outputs_contract_cfg,
                "reference_body_ang_vel_w_key",
                source_contract.policy_extra_outputs.reference_body_ang_vel_w_key);
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
            if (source_contract.imu_input.source_type != "sensor_msgs_imu" &&
                source_contract.imu_input.source_type != "unitree_hg_lowstate" &&
                source_contract.imu_input.source_type != "unitree_hg_imu_state" &&
                source_contract.imu_input.source_type != "unitree_sdk2_lowstate" &&
                source_contract.imu_input.source_type != "unitree_sdk2_imu_state")
            {
                throw std::runtime_error(
                    "source_contract.imu_input.source_type must be "
                    "'sensor_msgs_imu', 'unitree_hg_lowstate', 'unitree_hg_imu_state', "
                    "'unitree_sdk2_lowstate', or 'unitree_sdk2_imu_state'");
            }
            if (source_contract.imu_input.topic.empty())
            {
                throw std::runtime_error("source_contract.imu_input.topic must not be empty");
            }
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
            if (source_contract.sim_base.velocity_source != "freejoint_qvel" &&
                source_contract.sim_base.velocity_source != "body_object_velocity_local" &&
                source_contract.sim_base.velocity_source != "body_object_velocity_root_local" &&
                source_contract.sim_base.velocity_source != "body_cvel")
            {
                throw std::runtime_error(
                    "source_contract.sim_base.velocity_source must be "
                    "'freejoint_qvel', 'body_object_velocity_local', "
                    "'body_object_velocity_root_local', or 'body_cvel'");
            }
            if (source_contract.unitree_sdk2.domain_id < 0)
            {
                throw std::runtime_error("source_contract.unitree_sdk2.domain_id must be >= 0");
            }
            if (source_contract.unitree_sdk2.lowcmd_topic.empty())
            {
                throw std::runtime_error("source_contract.unitree_sdk2.lowcmd_topic must not be empty");
            }
            if (source_contract.unitree_sdk2.lowstate_topic.empty())
            {
                throw std::runtime_error("source_contract.unitree_sdk2.lowstate_topic must not be empty");
            }
            if (source_contract.unitree_sdk2.imu_topic.empty())
            {
                throw std::runtime_error("source_contract.unitree_sdk2.imu_topic must not be empty");
            }
            if (source_contract.unitree_sdk2.queue_len < 0)
            {
                throw std::runtime_error("source_contract.unitree_sdk2.queue_len must be >= 0");
            }
            if (source_contract.unitree_sdk2.writer_period_us <= 0)
            {
                throw std::runtime_error("source_contract.unitree_sdk2.writer_period_us must be > 0");
            }
            if (source_contract.unitree_sdk2.lowstate_timeout_sec <= 0.0f)
            {
                throw std::runtime_error("source_contract.unitree_sdk2.lowstate_timeout_sec must be > 0");
            }
            if (source_contract.unitree_sdk2.default_lower_kp < 0.0f ||
                source_contract.unitree_sdk2.default_lower_kd < 0.0f ||
                source_contract.unitree_sdk2.default_upper_kp < 0.0f ||
                source_contract.unitree_sdk2.default_upper_kd < 0.0f)
            {
                throw std::runtime_error("source_contract.unitree_sdk2 default gains must be >= 0");
            }
            if (source_contract.base_velocity_estimator.imu_accel_frame != "body" &&
                source_contract.base_velocity_estimator.imu_accel_frame != "world")
            {
                throw std::runtime_error("source_contract.base_velocity_estimator.imu_accel_frame must be 'body' or 'world'");
            }
            if (source_contract.base_velocity_estimator.odom_velocity_frame != "body" &&
                source_contract.base_velocity_estimator.odom_velocity_frame != "world")
            {
                throw std::runtime_error("source_contract.base_velocity_estimator.odom_velocity_frame must be 'body' or 'world'");
            }
            if (source_contract.base_velocity_estimator.gravity_mps2 <= 0.0f)
            {
                throw std::runtime_error("source_contract.base_velocity_estimator.gravity_mps2 must be > 0");
            }
            if (source_contract.base_velocity_estimator.input_velocity_measurement_noise <= 0.0f ||
                source_contract.base_velocity_estimator.odom_velocity_measurement_noise <= 0.0f ||
                source_contract.base_velocity_estimator.zero_velocity_measurement_noise <= 0.0f)
            {
                throw std::runtime_error("source_contract.base_velocity_estimator measurement noise values must be > 0");
            }
            if (source_contract.base_velocity_estimator.initial_variance <= 0.0f ||
                source_contract.base_velocity_estimator.min_dt < 0.0f ||
                source_contract.base_velocity_estimator.max_dt < source_contract.base_velocity_estimator.min_dt)
            {
                throw std::runtime_error("source_contract.base_velocity_estimator variance/dt settings are invalid");
            }
            auto normalizeLower = [](std::string text) {
                std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
                    return static_cast<char>(std::tolower(c));
                });
                return text;
            };
            const std::string normalized_reference_motion_source = normalizeLower(reference_motion_source);
            motion_reference_alignment = normalizeLower(motion_reference_alignment);
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
            if (motion_reference_alignment != "current_robot_anchor" &&
                motion_reference_alignment != "startup_anchor_pos_yaw")
            {
                throw std::runtime_error(
                    "motion_reference_alignment must be 'current_robot_anchor' or 'startup_anchor_pos_yaw'");
            }

            if (cfg["external_observations"])
            {
                for (const auto &node : cfg["external_observations"])
                {
                    ExternalObservationSpec spec;
                    spec.name = yamlReadOr<std::string>(node, "name", "");
                    spec.dim = yamlReadOr<int>(node, "dim", 0);
                    spec.required = yamlReadOr<bool>(node, "required", false);
                    spec.topic = yamlReadOr<std::string>(node, "topic", "");
                    spec.message_type = yamlReadOr<std::string>(node, "message_type", "float32_multi_array");
                    if (spec.name.empty())
                    {
                        continue;
                    }
                    if (spec.topic.empty())
                    {
                        spec.topic = "/omnimorph/external/" + spec.name;
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
            if (auto_switch_on_reference_end.enabled)
            {
                constexpr int kSupportedModeCodeMin = 0;
                constexpr int kSupportedModeCodeMax = 999;
                if (auto_switch_on_reference_end.target_mode_id < kSupportedModeCodeMin ||
                    auto_switch_on_reference_end.target_mode_id > kSupportedModeCodeMax)
                {
                    throw std::runtime_error(
                        "auto_switch_on_reference_end.target_mode_id is out of supported mode range");
                }
                if (auto_switch_on_reference_end.total_steps == 0 ||
                    auto_switch_on_reference_end.total_steps < -1)
                {
                    throw std::runtime_error(
                        "auto_switch_on_reference_end.total_steps must be -1 or > 0");
                }
            }
            policy_startup_warmup_steps = yamlReadOr<int>(cfg, "policy_startup_warmup_steps", 0);
            if (policy_startup_warmup_steps < 0)
            {
                throw std::runtime_error("policy_startup_warmup_steps must be >= 0");
            }
            prefill_observation_history_on_running_start =
                yamlReadOr<bool>(cfg, "prefill_observation_history_on_running_start", false);
            seed_running_start_observation_from_reference =
                yamlReadOr<bool>(cfg, "seed_running_start_observation_from_reference", false);
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
            scales.command_lin_vel = yamlReadOr<float>(scale, "command_lin_vel", scales.lin_vel);
            scales.command_ang_vel = yamlReadOr<float>(scale, "command_ang_vel", scales.ang_vel);
            scales.dof_pos = scale["dof_pos"].as<float>();
            scales.dof_vel = scale["dof_vel"].as<float>();
            scales.quat = scale["quat"].as<float>();
            scales.height_measurements = scale["height_measurements"].as<float>();

            const YAML::Node gait_cfg = cfg["gait"];
            gait.gait_air_ratio_l = yamlReadOr<float>(gait_cfg, "gait_air_ratio_l", gait.gait_air_ratio_l);
            gait.gait_air_ratio_r = yamlReadOr<float>(gait_cfg, "gait_air_ratio_r", gait.gait_air_ratio_r);
            gait.gait_phase_offset_l = yamlReadOr<float>(gait_cfg, "gait_phase_offset_l", gait.gait_phase_offset_l);
            gait.gait_phase_offset_r = yamlReadOr<float>(gait_cfg, "gait_phase_offset_r", gait.gait_phase_offset_r);
            gait.gait_cycle = yamlReadOr<double>(gait_cfg, "gait_cycle", gait.gait_cycle);

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
        std::cout << "Policy Adapter: " << policy_adapter << std::endl;
        std::cout << "Inference Strategy: " << inference_strategy << std::endl;
        std::cout << "Action Output Layout: " << action_output_layout << std::endl;
        std::cout << "Policy Path: " << policy_path << std::endl;
        std::cout << "Obs Stack N: " << obs_stack_N << std::endl;
        std::cout << "Action Chunk Steps: " << action_chunk_steps << std::endl;
        std::cout << "Action Chunk Execute Steps: " << action_chunk_execute_steps << std::endl;
        std::cout << "Action Chunk Replan Interval: " << action_chunk_replan_interval << std::endl;
        std::cout << "Action Scale: " << action_scale << std::endl;
        std::cout << "Zero Joint Angles: " << robotCfg.zero_joint_angles.size() << std::endl;
        std::cout << "Control Mode: " << control_mode << std::endl;
        std::cout << "Installed Joint Run Modes: " << installed_joint_run_modes.size() << std::endl;
        std::cout << "Startup Completion Action: " << startup_completion_action << std::endl;
        std::cout << "Policy Frequency: " << RL_control_f << std::endl;
        std::cout << "Solver Control Frequency: " << solver_control_hz << std::endl;
        std::cout << "Sub Models: " << sub_models.size() << std::endl;
        std::cout << "Reference Motion Source: " << reference_motion_source
                  << ", enabled=" << (enable_reference_motion ? "true" : "false")
                  << std::endl;
        std::cout << "Reference Motion Path: " << reference_motion_path << std::endl;
        std::cout << "Motion Reference Alignment: " << motion_reference_alignment << std::endl;
        std::cout << "Pinocchio URDF Path: " << pinocchio_urdf_path << std::endl;
        std::cout << "Reference Joint Order: " << reference_joint_order.size() << std::endl;
        std::cout << "Source Contract IMU: source_type=" << source_contract.imu_input.source_type
                  << ", topic=" << source_contract.imu_input.topic
                  << ", payload=" << source_contract.imu_input.payload
                  << ", euler_unit=" << source_contract.imu_input.euler_unit
                  << ", quat_order=" << source_contract.imu_input.quat_order
                  << ", sim_base_quat_source_order=" << source_contract.sim_base.quat_source_order
                  << ", sim_base_velocity_source=" << source_contract.sim_base.velocity_source
                  << ", base_velocity_estimator=" << (source_contract.base_velocity_estimator.enabled ? "enabled" : "disabled")
                  << ", base_velocity_odom_topic=" << source_contract.base_velocity_estimator.odom_topic
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
