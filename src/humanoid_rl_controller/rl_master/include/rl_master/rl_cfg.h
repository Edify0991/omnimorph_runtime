#ifndef RL_CFG_H
#define RL_CFG_H

#include <cstdint>
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

template <typename T>
inline T yamlReadOr(const YAML::Node &node, const char *key, const T &default_value)
{
    if (!node || !node[key])
    {
        return default_value;
    }
    return node[key].as<T>();
}

inline std::string getCurrentTime()
{
    const auto t = std::time(nullptr);
    const auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%b%d_%H-%M-%S");
    return oss.str();
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
    std::vector<PolicySubModelCfg> sub_models;

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

    bool loadFromYAML(const std::string &yaml_file, const std::string &config_type = "sim2real")
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

            humanoid_rl_root_dir = config["humanoid_rl_root_dir"].as<std::string>();
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
        std::cout << "Reference Motion Source: " << reference_motion_source << std::endl;
        std::cout << "Reference Motion Path: " << reference_motion_path << std::endl;
        std::cout << "External Obs Inputs: " << external_observations.size() << std::endl;
        std::cout << "=============================" << std::endl;
    }
};

using StandSim2RealCfg = Sim2realCfg;

#endif // RL_CFG_H
