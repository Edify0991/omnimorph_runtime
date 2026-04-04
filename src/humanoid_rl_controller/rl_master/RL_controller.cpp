/*
onnxruntime:
https://blog.csdn.net/yangyu0515/article/details/142093965
https://blog.csdn.net/m0_57254760/article/details/138304321
*/
#include "rl_master/RL_controller.h"

#include <algorithm>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "rl_master/rl_protocol.h"

namespace
{
std::vector<double> toDoubleVector(const std::vector<float> &values)
{
    std::vector<double> out(values.size(), 0.0);
    for (size_t i = 0; i < values.size(); ++i)
    {
        out[i] = static_cast<double>(values[i]);
    }
    return out;
}
} // namespace

RL_controller::RL_controller()
    : onnx_env_(ORT_LOGGING_LEVEL_WARNING, "RL_controller")
{
    robot = RobotState::create();
    if (!robot)
    {
        throw std::runtime_error("Failed to create RobotState object!");
    }
}

RL_controller::~RL_controller()
{
    if (data_logger_)
    {
        data_logger_->close();
    }
}

std::unique_ptr<RL_controller> RL_controller::create()
{
    return std::make_unique<RL_controller>();
}

const std::array<std::string, rl_master::kLegJointCount> &RL_controller::canonicalJointOrder()
{
    static const std::array<std::string, rl_master::kLegJointCount> kJointOrder = {
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_hip_pitch_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_hip_pitch_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint"};
    return kJointOrder;
}

std::vector<int> RL_controller::buildActionIndexMap(const Sim2realCfg &cfg, const std::string &cfg_name) const
{
    if (cfg.action_dim <= 0)
    {
        throw std::runtime_error(cfg_name + ": action_dim must be > 0");
    }
    if (cfg.action_dim > static_cast<int>(rl_master::kLegJointCount))
    {
        throw std::runtime_error(cfg_name + ": action_dim exceeds robot joint count");
    }

    std::vector<int> map_robot_idx_to_policy_idx(static_cast<size_t>(cfg.action_dim), 0);
    if (cfg.action_joint_order.empty())
    {
        for (int i = 0; i < cfg.action_dim; ++i)
        {
            map_robot_idx_to_policy_idx[static_cast<size_t>(i)] = i;
        }
        return map_robot_idx_to_policy_idx;
    }

    if (cfg.action_joint_order.size() != static_cast<size_t>(cfg.action_dim))
    {
        throw std::runtime_error(cfg_name + ": action_joint_order length must equal action_dim");
    }

    std::unordered_map<std::string, int> policy_order_index;
    policy_order_index.reserve(cfg.action_joint_order.size());
    for (size_t i = 0; i < cfg.action_joint_order.size(); ++i)
    {
        const auto &name = cfg.action_joint_order[i];
        if (policy_order_index.find(name) != policy_order_index.end())
        {
            throw std::runtime_error(cfg_name + ": duplicate joint name in action_joint_order: " + name);
        }
        policy_order_index[name] = static_cast<int>(i);
    }

    const auto &canonical = canonicalJointOrder();
    for (int robot_idx = 0; robot_idx < cfg.action_dim; ++robot_idx)
    {
        const auto &joint_name = canonical[static_cast<size_t>(robot_idx)];
        const auto it = policy_order_index.find(joint_name);
        if (it == policy_order_index.end())
        {
            throw std::runtime_error(cfg_name + ": action_joint_order missing joint " + joint_name);
        }
        map_robot_idx_to_policy_idx[static_cast<size_t>(robot_idx)] = it->second;
    }

    return map_robot_idx_to_policy_idx;
}

std::vector<int> RL_controller::buildObsIndexMap(const Sim2realCfg &cfg, const std::string &cfg_name) const
{
    if (cfg.motor_N <= 0)
    {
        throw std::runtime_error(cfg_name + ": motor_N must be > 0");
    }
    if (cfg.motor_N > static_cast<int>(rl_master::kLegJointCount))
    {
        throw std::runtime_error(cfg_name + ": motor_N exceeds robot joint count");
    }

    std::vector<int> map_policy_idx_to_robot_idx(static_cast<size_t>(cfg.motor_N), 0);
    if (cfg.obs_joint_order.empty())
    {
        for (int i = 0; i < cfg.motor_N; ++i)
        {
            map_policy_idx_to_robot_idx[static_cast<size_t>(i)] = i;
        }
        return map_policy_idx_to_robot_idx;
    }

    if (cfg.obs_joint_order.size() != static_cast<size_t>(cfg.motor_N))
    {
        throw std::runtime_error(cfg_name + ": obs_joint_order length must equal motor_N");
    }

    std::unordered_map<std::string, int> canonical_index;
    canonical_index.reserve(rl_master::kLegJointCount);
    const auto &canonical = canonicalJointOrder();
    for (size_t i = 0; i < canonical.size(); ++i)
    {
        canonical_index[canonical[i]] = static_cast<int>(i);
    }

    for (size_t policy_idx = 0; policy_idx < cfg.obs_joint_order.size(); ++policy_idx)
    {
        const auto &name = cfg.obs_joint_order[policy_idx];
        const auto it = canonical_index.find(name);
        if (it == canonical_index.end())
        {
            throw std::runtime_error(cfg_name + ": unknown joint in obs_joint_order: " + name);
        }
        map_policy_idx_to_robot_idx[policy_idx] = it->second;
    }
    return map_policy_idx_to_robot_idx;
}

const std::vector<int> &RL_controller::currentActionIndexMap() const
{
    return activeModeProfile().action_index_map;
}

const std::vector<int> &RL_controller::currentObsIndexMap() const
{
    return activeModeProfile().obs_index_map;
}

const Sim2realCfg &RL_controller::activePolicyCfg() const
{
    return activeModeProfile().cfg;
}

const ObservationBuilder &RL_controller::activeObservationBuilder() const
{
    if (!activeModeProfile().observation_builder)
    {
        throw std::runtime_error("Active observation builder is null");
    }
    return *activeModeProfile().observation_builder;
}

RL_controller::PolicyRuntimeGroup &RL_controller::activePolicyGroup()
{
    return activeModeProfile().policy_group;
}

const ReferenceMotionProvider &RL_controller::activeReferenceMotionProvider() const
{
    return activeModeProfile().reference_motion;
}

std::vector<float> RL_controller::activeZeroPose() const
{
    const auto &cfg = activePolicyCfg();
    if (!cfg.zero_pose.empty())
    {
        return cfg.zero_pose;
    }
    return robot->default_angle;
}

void RL_controller::refreshPolicyMode(int requested_mode, bool sanitize_invalid_mode)
{
    if (mode_profiles_.empty())
    {
        throw std::runtime_error("No mode profile is loaded");
    }

    const size_t profile_index = profileIndexForMode(requested_mode, sanitize_invalid_mode);
    active_profile_index_ = profile_index;
    active_mode_id_ = mode_profiles_[profile_index].mode_id;
    robot->default_angle = mode_profiles_[profile_index].default_angle;
}

void RL_controller::handlePolicySwitch()
{
    if (last_active_mode_id_ == active_mode_id_)
    {
        return;
    }

    const auto &cfg = activePolicyCfg();
    if (cfg.obs_dim <= 0 || cfg.obs_stack_N <= 0)
    {
        throw std::runtime_error("Invalid active policy dimensions");
    }

    obs.assign(static_cast<size_t>(cfg.obs_dim), 0.0f);
    obs_deque.clear();
    for (int i = 0; i < cfg.obs_stack_N; ++i)
    {
        obs_deque.push_back(std::vector<float>(static_cast<size_t>(cfg.obs_dim), 0.0f));
    }
    stacked_obs_buffer_.assign(static_cast<size_t>(cfg.obs_dim * cfg.obs_stack_N), 0.0f);
    action.assign(static_cast<size_t>(cfg.action_dim), 0.0f);
    joint_target_q.assign(rl_master::kLegJointCount, 0.0f);
    joint_target_torque.assign(rl_master::kLegJointCount, 0.0f);
    latest_policy_extra_outputs_.clear();
    deploy_step_counter_ = 0;

    if (cfg.reset_policy_on_mode_switch)
    {
        auto &group = activePolicyGroup();
        for (auto &node : group.runners)
        {
            if (node.runner)
            {
                node.runner->reset();
            }
        }
    }

    deploy_state_machine_.configure(cfg);
    deploy_state_machine_.setZeroPose(activeZeroPose());

    last_active_mode_id_ = active_mode_id_;
    std::cout << "[RL_controller] switch policy to "
              << activeModeProfile().tag
              << ", mode_id=" << active_mode_id_ << std::endl;
}

const Sim2realCfg &RL_controller::runtimeCfg() const
{
    return runtimeModeProfile().cfg;
}

RL_controller::ModeProfile &RL_controller::activeModeProfile()
{
    if (mode_profiles_.empty() || active_profile_index_ >= mode_profiles_.size())
    {
        throw std::runtime_error("Active mode profile index is invalid");
    }
    return mode_profiles_[active_profile_index_];
}

const RL_controller::ModeProfile &RL_controller::activeModeProfile() const
{
    if (mode_profiles_.empty() || active_profile_index_ >= mode_profiles_.size())
    {
        throw std::runtime_error("Active mode profile index is invalid");
    }
    return mode_profiles_[active_profile_index_];
}

const RL_controller::ModeProfile &RL_controller::runtimeModeProfile() const
{
    if (mode_profiles_.empty())
    {
        throw std::runtime_error("Runtime mode profile is unavailable");
    }
    const auto it = mode_to_profile_index_.find(default_mode_id_);
    if (it != mode_to_profile_index_.end())
    {
        return mode_profiles_[it->second];
    }
    return mode_profiles_.front();
}

size_t RL_controller::profileIndexForMode(int mode_id, bool sanitize_invalid_mode) const
{
    const auto it = mode_to_profile_index_.find(mode_id);
    if (it != mode_to_profile_index_.end())
    {
        return it->second;
    }

    if (!sanitize_invalid_mode)
    {
        throw std::runtime_error("Unknown mode id: " + std::to_string(mode_id));
    }

    const auto fallback_it = mode_to_profile_index_.find(default_mode_id_);
    if (fallback_it != mode_to_profile_index_.end())
    {
        return fallback_it->second;
    }
    return 0;
}

std::vector<float> RL_controller::buildDefaultAnglesFromCfg(const Sim2realCfg::RobotCfg &robot_cfg) const
{
    std::vector<float> out(rl_master::kLegJointCount, 0.0f);
    std::unordered_map<std::string, float> default_angle_map;
    default_angle_map.reserve(robot_cfg.default_joint_angles.size());
    for (const auto &entry : robot_cfg.default_joint_angles)
    {
        default_angle_map[entry.first] = entry.second;
    }

    const auto &order = canonicalJointOrder();
    for (size_t i = 0; i < order.size(); ++i)
    {
        const auto it = default_angle_map.find(order[i]);
        if (it != default_angle_map.end())
        {
            out[i] = it->second;
        }
    }
    return out;
}

std::vector<RL_controller::ModeProfileSpec> RL_controller::loadModeProfileSpecsFromYaml() const
{
    std::vector<ModeProfileSpec> specs = {
        {rl_master::kWalkModeCode, "sim2real", "walk"},
        {rl_master::kStandModeCode, "stand_sim2real", "stand"},
        {rl_master::kFixStandModeCode, "stand_sim2real", "fix_stand"},
    };

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(RL_CFG_PATH);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[RL_controller] failed to parse RL cfg for mode profile map: " << e.what()
                  << ". Use default mode profile mapping." << std::endl;
        return specs;
    }

    const YAML::Node profile_nodes = root["deploy_mode_profiles"];
    if (!profile_nodes || !profile_nodes.IsSequence() || profile_nodes.size() == 0)
    {
        return specs;
    }

    std::vector<ModeProfileSpec> parsed;
    for (size_t i = 0; i < profile_nodes.size(); ++i)
    {
        const YAML::Node node = profile_nodes[i];
        if (!node["mode_id"] || !node["config_section"])
        {
            throw std::runtime_error(
                "deploy_mode_profiles[" + std::to_string(i) + "] requires mode_id and config_section");
        }

        ModeProfileSpec spec;
        spec.mode_id = node["mode_id"].as<int>();
        spec.config_section = node["config_section"].as<std::string>();
        spec.tag = yamlReadOr<std::string>(node, "tag", spec.config_section);
        parsed.push_back(spec);
    }

    return parsed;
}

void RL_controller::initModeProfiles()
{
    mode_profiles_.clear();
    mode_to_profile_index_.clear();

    const std::vector<ModeProfileSpec> specs = loadModeProfileSpecsFromYaml();
    if (specs.empty())
    {
        throw std::runtime_error("No deploy mode profile spec is configured");
    }

    std::unordered_map<std::string, Sim2realCfg> cfg_cache;
    for (const auto &spec : specs)
    {
        if (mode_to_profile_index_.find(spec.mode_id) != mode_to_profile_index_.end())
        {
            throw std::runtime_error("Duplicate mode_id in deploy_mode_profiles: " + std::to_string(spec.mode_id));
        }
        if (spec.mode_id < rl_master::kModeCodeMin || spec.mode_id > rl_master::kModeCodeMax)
        {
            throw std::runtime_error(
                "mode_id out of supported range [" +
                std::to_string(rl_master::kModeCodeMin) +
                ", " +
                std::to_string(rl_master::kModeCodeMax) +
                "]: " +
                std::to_string(spec.mode_id));
        }

        Sim2realCfg cfg;
        const auto cache_it = cfg_cache.find(spec.config_section);
        if (cache_it != cfg_cache.end())
        {
            cfg = cache_it->second;
        }
        else
        {
            if (!cfg.loadFromYAML(RL_CFG_PATH, spec.config_section))
            {
                throw std::runtime_error("Failed to load config section: " + spec.config_section);
            }
            cfg_cache[spec.config_section] = cfg;
        }

        ModeProfile profile;
        profile.mode_id = spec.mode_id;
        profile.config_section = spec.config_section;
        profile.tag = spec.tag;
        profile.cfg = cfg;
        profile.default_angle = buildDefaultAnglesFromCfg(profile.cfg.robotCfg);
        profile.action_index_map = buildActionIndexMap(profile.cfg, profile.config_section);
        profile.obs_index_map = buildObsIndexMap(profile.cfg, profile.config_section);
        profile.observation_manifest = ObservationManifest::loadFromYAML(profile.cfg.observation_manifest_path);
        profile.observation_builder = std::make_unique<ObservationBuilder>(profile.observation_manifest);
        if (profile.observation_builder->expectedDim() != static_cast<size_t>(profile.cfg.obs_dim))
        {
            throw std::runtime_error(
                profile.tag + " observation manifest dim (" +
                std::to_string(profile.observation_builder->expectedDim()) +
                ") does not match cfg obs_dim (" + std::to_string(profile.cfg.obs_dim) + ")");
        }
        initPolicyGroup(profile.cfg, profile.tag, &profile.policy_group);
        initReferenceMotionProvider(profile.cfg, &profile.reference_motion, profile.tag);

        mode_to_profile_index_[profile.mode_id] = mode_profiles_.size();
        mode_profiles_.push_back(std::move(profile));
    }

    default_mode_id_ = mode_profiles_.front().mode_id;
    if (mode_to_profile_index_.find(rl_master::kWalkModeCode) != mode_to_profile_index_.end())
    {
        default_mode_id_ = rl_master::kWalkModeCode;
    }
    active_mode_id_ = default_mode_id_;
    active_profile_index_ = profileIndexForMode(default_mode_id_, true);

    // Keep backward compatibility fields for existing components.
    robot->sim2realCfg = mode_profiles_[active_profile_index_].cfg;
    robot->default_angle_walk = mode_profiles_[active_profile_index_].default_angle;
    robot->default_angle = mode_profiles_[active_profile_index_].default_angle;

    const auto stand_it = mode_to_profile_index_.find(rl_master::kStandModeCode);
    if (stand_it != mode_to_profile_index_.end())
    {
        robot->standSim2RealCfg = mode_profiles_[stand_it->second].cfg;
        robot->default_angle_stand = mode_profiles_[stand_it->second].default_angle;
    }
    else
    {
        robot->standSim2RealCfg = mode_profiles_[active_profile_index_].cfg;
        robot->default_angle_stand = mode_profiles_[active_profile_index_].default_angle;
    }

    std::cout << "[RL_controller] mode profiles loaded: " << mode_profiles_.size() << std::endl;
    for (const auto &profile : mode_profiles_)
    {
        std::cout << "  - mode_id=" << profile.mode_id
                  << ", tag=" << profile.tag
                  << ", section=" << profile.config_section
                  << ", policy=" << profile.cfg.policy_name << std::endl;
    }
}

void RL_controller::updateStateFromIO(const rl_master::RobotStateData &state)
{
    for (size_t i = 0; i < rl_master::kLegJointCount; ++i)
    {
        robot->joint_q[i] = state.joint_q[i];
        robot->joint_dq[i] = state.joint_dq[i];
        robot->joint_tau[i] = state.joint_tau[i];
    }

    for (size_t i = 0; i < 3; ++i)
    {
        robot->base_ang_vel[i] = state.base_ang_vel[i];
        robot->base_rpy[i] = state.base_rpy[i];
    }

    for (size_t i = 0; i < 4; ++i)
    {
        robot->base_quat[i] = state.base_quat[i];
    }
}

void RL_controller::updateCommandFromIO(const rl_master::TeleopCommand &command)
{
    cmd.vx = command.vx;
    cmd.vy = command.vy;
    cmd.dyaw = command.dyaw;
}

void RL_controller::initPolicyGroup(const Sim2realCfg &cfg, const std::string &tag, PolicyRuntimeGroup *group)
{
    if (!group)
    {
        throw std::runtime_error("initPolicyGroup: group is null");
    }

    group->runners.clear();

    PolicyRunnerNode primary;
    primary.name = tag + "/main";
    primary.weight = 1.0f;
    primary.runner = std::make_unique<OnnxPolicyRunner>(
        onnx_env_,
        cfg.policy_path,
        cfg,
        primary.name);
    primary.runner->init();
    group->runners.push_back(std::move(primary));

    for (const auto &sub : cfg.sub_models)
    {
        if (!sub.enabled)
        {
            continue;
        }

        Sim2realCfg sub_cfg = cfg;
        sub_cfg.policy_path = sub.policy_path;
        if (sub.action_dim > 0)
        {
            sub_cfg.action_dim = sub.action_dim;
        }
        sub_cfg.obs_input_name = sub.obs_input_name;
        sub_cfg.action_output_name = sub.action_output_name;
        sub_cfg.time_step_input_name = sub.time_step_input_name;
        sub_cfg.time_step_start = sub.time_step_start;
        sub_cfg.enable_time_step_input = sub.enable_time_step_input;
        sub_cfg.strict_model_io = sub.strict_model_io;
        sub_cfg.extra_output_names = sub.extra_output_names;

        PolicyRunnerNode node;
        node.name = tag + "/" + sub.name;
        node.weight = std::max(0.0f, sub.weight);
        node.runner = std::make_unique<OnnxPolicyRunner>(
            onnx_env_,
            sub_cfg.policy_path,
            sub_cfg,
            node.name);
        node.runner->init();
        group->runners.push_back(std::move(node));
    }
}

RL_controller::PolicyRunOutput RL_controller::runPolicyGroup(PolicyRuntimeGroup *group, const std::vector<float> &stacked_obs)
{
    if (!group || group->runners.empty())
    {
        throw std::runtime_error("runPolicyGroup: empty policy group");
    }

    const size_t expected_dim = static_cast<size_t>(std::max(0, activePolicyCfg().action_dim));
    PolicyRunOutput output;
    output.action.assign(expected_dim, 0.0f);

    float total_weight = 0.0f;
    bool primary_done = false;
    for (auto &node : group->runners)
    {
        if (!node.runner)
        {
            continue;
        }

        PolicyInferenceResult result = node.runner->forward(stacked_obs);
        const float weight = primary_done ? node.weight : 1.0f;
        const size_t dim = std::min(output.action.size(), result.action.size());
        for (size_t i = 0; i < dim; ++i)
        {
            output.action[i] += weight * result.action[i];
        }
        total_weight += weight;

        for (auto &kv : result.extra_outputs)
        {
            output.extra_outputs[node.name + "/" + kv.first] = std::move(kv.second);
        }

        primary_done = true;
    }

    if (total_weight > 1e-6f)
    {
        for (auto &v : output.action)
        {
            v /= total_weight;
        }
    }
    return output;
}

void RL_controller::initReferenceMotionProvider(const Sim2realCfg &cfg, ReferenceMotionProvider *provider, const std::string &tag)
{
    if (!provider)
    {
        return;
    }

    provider->clear();
    if (!cfg.enable_reference_motion)
    {
        return;
    }
    if (cfg.reference_motion_dim <= 0 || cfg.reference_motion_path.empty())
    {
        std::cerr << "[RL_controller][" << tag << "] reference motion config invalid. dim="
                  << cfg.reference_motion_dim << ", path=" << cfg.reference_motion_path << std::endl;
        return;
    }

    if (!provider->load(cfg.reference_motion_path, cfg.reference_motion_dim))
    {
        std::cerr << "[RL_controller][" << tag << "] failed to load reference motion file: "
                  << cfg.reference_motion_path << std::endl;
        return;
    }

    std::cout << "[RL_controller][" << tag << "] reference motion loaded. frames="
              << provider->frameCount() << ", dim=" << provider->dim() << std::endl;
}

ObservationFeatureContext RL_controller::buildObservationFeatureContext(const Sim2realCfg &cfg, double phase_t)
{
    ObservationFeatureContext feature_context;

    if (cfg.enable_reference_motion && cfg.reference_motion_dim > 0)
    {
        const auto &provider = activeReferenceMotionProvider();
        if (cfg.reference_motion_sampling == "step")
        {
            feature_context.named_features["reference_motion"] =
                provider.sampleByStep(deploy_step_counter_, cfg.reference_motion_dim);
        }
        else
        {
            feature_context.named_features["reference_motion"] =
                provider.sampleByPhase(phase_t, cfg.cycle_time, cfg.reference_motion_dim);
        }
    }

    auto external = external_observation_provider_.collect(cfg.external_observations);
    for (auto &kv : external)
    {
        feature_context.named_features.emplace(std::move(kv.first), std::move(kv.second));
    }

    return feature_context;
}

void RL_controller::initDataLogger()
{
    const auto &runtime_cfg = runtimeModeProfile().cfg;
    if (!runtime_cfg.save_data_flag)
    {
        return;
    }

    data_logger_ = std::make_unique<rl_master::logging::StructuredLogger>();
    rl_master::logging::LoggerMetadata metadata;
    metadata.string_fields["module"] = "RL_controller";
    metadata.numeric_fields["profile_count"] = static_cast<double>(mode_profiles_.size());

    auto collectSubModelNames = [](const Sim2realCfg &cfg) {
        std::vector<std::string> names;
        for (const auto &sub : cfg.sub_models)
        {
            names.push_back(sub.name);
        }
        return names;
    };
    auto collectSubModelPaths = [](const Sim2realCfg &cfg) {
        std::vector<std::string> paths;
        for (const auto &sub : cfg.sub_models)
        {
            paths.push_back(sub.policy_path);
        }
        return paths;
    };

    for (size_t i = 0; i < mode_profiles_.size(); ++i)
    {
        const auto &profile = mode_profiles_[i];
        const std::string prefix = "profile_" + std::to_string(i) + "_";

        metadata.numeric_fields[prefix + "mode_id"] = static_cast<double>(profile.mode_id);
        metadata.numeric_fields[prefix + "obs_dim"] = static_cast<double>(profile.cfg.obs_dim);
        metadata.numeric_fields[prefix + "action_dim"] = static_cast<double>(profile.cfg.action_dim);
        metadata.numeric_fields[prefix + "obs_stack"] = static_cast<double>(profile.cfg.obs_stack_N);
        metadata.numeric_fields[prefix + "control_hz"] = static_cast<double>(profile.cfg.RL_control_f);

        metadata.string_fields[prefix + "tag"] = profile.tag;
        metadata.string_fields[prefix + "config_section"] = profile.config_section;
        metadata.string_fields[prefix + "policy_name"] = profile.cfg.policy_name;
        metadata.string_fields[prefix + "policy_family"] = profile.cfg.policy_family;
        metadata.string_fields[prefix + "policy_path"] = profile.cfg.policy_path;
        metadata.string_fields[prefix + "observation_manifest"] = profile.cfg.observation_manifest_path;

        metadata.vector_fields[prefix + "kps"] = toDoubleVector(profile.cfg.kps);
        metadata.vector_fields[prefix + "kds"] = toDoubleVector(profile.cfg.kds);
        metadata.vector_fields[prefix + "tau_limit"] = toDoubleVector(profile.cfg.tau_limit);

        metadata.string_list_fields[prefix + "action_joint_order"] = profile.cfg.action_joint_order;
        metadata.string_list_fields[prefix + "obs_joint_order"] = profile.cfg.obs_joint_order;
        metadata.string_list_fields[prefix + "sub_model_names"] = collectSubModelNames(profile.cfg);
        metadata.string_list_fields[prefix + "sub_model_paths"] = collectSubModelPaths(profile.cfg);
    }

    if (!data_logger_->open(runtime_cfg.data_path, "controller", metadata))
    {
        std::cerr << "[RL_controller] failed to open structured data logger." << std::endl;
        data_logger_.reset();
        return;
    }

    data_logger_->writeEvent(
        rl_master::monotonicTimeSec(),
        "controller_initialized",
        {{"session_base_path", runtime_cfg.data_path}});
    std::cout << "RL Controller structured log: " << data_logger_->recordsPath() << std::endl;
}

void RL_controller::logStepRecord(
    double phase_t,
    int requested_mode_command,
    const rl_master::DeployStateOutput &deploy_output)
{
    if (!data_logger_ || !data_logger_->isOpen())
    {
        return;
    }

    std::map<std::string, double> scalars;
    scalars["frame_index"] = static_cast<double>(data_log_frame_index_++);
    scalars["phase_t"] = phase_t;
    scalars["requested_mode_command"] = static_cast<double>(requested_mode_command);
    scalars["active_mode_id"] = static_cast<double>(deploy_output.locomotion_mode);
    scalars["deploy_state"] = static_cast<double>(static_cast<int>(deploy_output.state));
    scalars["active_profile_index"] = static_cast<double>(active_profile_index_);
    scalars["open_rl"] = static_cast<double>(robot->open_rl);
    scalars["cmd_vx"] = static_cast<double>(cmd.vx);
    scalars["cmd_vy"] = static_cast<double>(cmd.vy);
    scalars["cmd_dyaw"] = static_cast<double>(cmd.dyaw);

    std::map<std::string, std::vector<float>> vectors;
    vectors["joint_q"] = robot->joint_q;
    vectors["joint_dq"] = robot->joint_dq;
    vectors["joint_tau"] = robot->joint_tau;
    vectors["joint_target_q"] = robot->joint_target_q;
    vectors["joint_target_tau"] = robot->joint_target_tau;
    vectors["observation"] = obs;
    vectors["policy_action"] = action;

    data_logger_->writeRecord(rl_master::monotonicTimeSec(), "controller_step", scalars, vectors);
}

void RL_controller::RL_controller_Init()
{
    robot->initialize_buffers();
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.dyaw = 0.0f;

    initModeProfiles();

    refreshPolicyMode(default_mode_id_, true);
    handlePolicySwitch();
    deploy_state_machine_initialized_ = false;
    last_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;

    start_time = std::chrono::high_resolution_clock::now();
    initDataLogger();
}

rl_master::RobotCommandData RL_controller::step(
    const rl_master::RobotStateData &state,
    const rl_master::TeleopCommand &command,
    int mode_command,
    double phase_t)
{
    updateStateFromIO(state);
    updateCommandFromIO(command);

    if (!deploy_state_machine_initialized_)
    {
        refreshPolicyMode(default_mode_id_, true);
        handlePolicySwitch();
        deploy_state_machine_.configure(activePolicyCfg());
        deploy_state_machine_.initialize(robot->joint_q, activeZeroPose(), active_mode_id_);
        deploy_state_machine_initialized_ = true;
        last_deploy_state_ = deploy_state_machine_.state();
    }

    const double now_s = rl_master::monotonicTimeSec();
    const auto deploy_output = deploy_state_machine_.update(mode_command, now_s, robot->joint_q);

    refreshPolicyMode(deploy_output.locomotion_mode, true);
    handlePolicySwitch();

    if (deploy_output.state != last_deploy_state_)
    {
        std::cout << "[RL_controller] lifecycle -> "
                  << rl_master::DeployStateMachine::stateName(deploy_output.state)
                  << std::endl;
        if (data_logger_ && data_logger_->isOpen())
        {
            data_logger_->writeEvent(
                rl_master::monotonicTimeSec(),
                "lifecycle_transition",
                {{"state", rl_master::DeployStateMachine::stateName(deploy_output.state)}});
        }
        last_deploy_state_ = deploy_output.state;
    }

    if (deploy_output.enable_policy)
    {
        std::vector<float> current_obs = get_robot_observation(phase_t);
        update_obs_deque(current_obs);

        const std::vector<float> policy_action = run_policy();
        robot->joint_target_q = get_joint_target_q(policy_action);
        robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
        robot->open_rl = rl_master::kOpenRlEnabled;
    }
    else if (deploy_output.enable_command_stream)
    {
        robot->joint_target_q.assign(rl_master::kLegJointCount, 0.0f);
        const size_t copy_n = std::min(robot->joint_target_q.size(), deploy_output.target_q.size());
        for (size_t i = 0; i < copy_n; ++i)
        {
            robot->joint_target_q[i] = deploy_output.target_q[i];
        }
        robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
        robot->open_rl = rl_master::kOpenRlEnabled;
        latest_policy_extra_outputs_.clear();
    }
    else
    {
        robot->joint_target_q = robot->joint_q;
        robot->joint_target_tau.assign(rl_master::kLegJointCount, 0.0f);
        robot->open_rl = rl_master::kOpenRlDisabled;
        latest_policy_extra_outputs_.clear();
    }

    rl_master::RobotCommandData out_cmd;
    out_cmd.open_rl = robot->open_rl;
    for (size_t i = 0; i < rl_master::kLegJointCount; ++i)
    {
        out_cmd.joint_target_q[i] = robot->joint_target_q[i];
        out_cmd.joint_target_dq[i] = 0.0f;
        out_cmd.joint_target_tau[i] = 0.0f;
    }

    ++deploy_step_counter_;
    logStepRecord(phase_t, mode_command, deploy_output);
    return out_cmd;
}

void RL_controller::estop()
{
    robot->open_rl = rl_master::kOpenRlDisabled;
}

std::vector<float> RL_controller::get_robot_observation(double phase_t)
{
    const auto &active_cfg = activePolicyCfg();

    const ObservationFeatureContext feature_context = buildObservationFeatureContext(active_cfg, phase_t);
    obs = activeObservationBuilder().build(*robot, cmd, action, phase_t, active_cfg, currentObsIndexMap(), feature_context);
    return obs;
}

std::vector<float> RL_controller::run_policy(std::deque<std::vector<float>> *obs_deque_ptr)
{
    if (!obs_deque_ptr)
    {
        obs_deque_ptr = &obs_deque;
    }

    const auto &active_cfg = activePolicyCfg();
    const size_t expected_obs_size = static_cast<size_t>(active_cfg.obs_dim) * obs_deque_ptr->size();
    if (stacked_obs_buffer_.size() != expected_obs_size)
    {
        stacked_obs_buffer_.assign(expected_obs_size, 0.0f);
    }

    size_t offset = 0;
    for (const auto &frame_obs : *obs_deque_ptr)
    {
        if (frame_obs.size() != static_cast<size_t>(active_cfg.obs_dim))
        {
            throw std::runtime_error(
                "Stacked observation frame dim mismatch. got=" + std::to_string(frame_obs.size()) +
                ", expected=" + std::to_string(active_cfg.obs_dim));
        }
        std::copy(frame_obs.begin(), frame_obs.end(), stacked_obs_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
        offset += frame_obs.size();
    }

    PolicyRunOutput policy_output = runPolicyGroup(&activePolicyGroup(), stacked_obs_buffer_);
    std::vector<float> target_action = std::move(policy_output.action);
    latest_policy_extra_outputs_ = std::move(policy_output.extra_outputs);

    for (auto &value : target_action)
    {
        value = std::clamp(value, -active_cfg.clip_actions, active_cfg.clip_actions);
    }

    if (action.size() != target_action.size())
    {
        action.assign(target_action.size(), 0.0f);
    }
    for (size_t i = 0; i < target_action.size(); ++i)
    {
        target_action[i] = (1.0f - active_cfg.action_filter) * target_action[i] + active_cfg.action_filter * action[i];
    }

    action = target_action;
    return target_action;
}

std::deque<std::vector<float>> RL_controller::update_obs_deque(const std::vector<float> &new_obs)
{
    const auto &active_cfg = activePolicyCfg();
    if (new_obs.size() != static_cast<size_t>(active_cfg.obs_dim))
    {
        throw std::runtime_error(
            "Observation dim mismatch in deque update. got=" + std::to_string(new_obs.size()) +
            ", expected=" + std::to_string(active_cfg.obs_dim));
    }

    const size_t expected_stack = static_cast<size_t>(active_cfg.obs_stack_N);
    if (obs_deque.size() != expected_stack)
    {
        obs_deque.clear();
        for (size_t i = 0; i < expected_stack; ++i)
        {
            obs_deque.push_back(std::vector<float>(new_obs.size(), 0.0f));
        }
    }

    obs_deque.push_back(new_obs);
    while (obs_deque.size() > expected_stack)
    {
        obs_deque.pop_front();
    }
    return obs_deque;
}

std::vector<float> RL_controller::pd_control(
    const std::vector<float> &target_q,
    const std::vector<float> &kp,
    const std::vector<float> &target_dq,
    const std::vector<float> &kd)
{
    const std::vector<float> &q = robot->joint_q;
    const std::vector<float> &dq = robot->joint_dq;
    std::vector<float> torque(target_q.size(), 0.0f);

    const size_t n = std::min(
        std::min(target_q.size(), target_dq.size()),
        std::min(std::min(kp.size(), kd.size()), std::min(q.size(), dq.size())));

    for (size_t i = 0; i < n; ++i)
    {
        torque[i] = (target_q[i] - q[i]) * kp[i] + (target_dq[i] - dq[i]) * kd[i];
    }
    return torque;
}

std::vector<float> RL_controller::get_joint_target_torque(const std::vector<float> &target_q)
{
    const auto &active_cfg = activePolicyCfg();
    std::vector<float> target_dq(target_q.size(), 0.0f);
    std::vector<float> target_tau = pd_control(target_q, active_cfg.kps, target_dq, active_cfg.kds);

    auto limit_or = [&](const std::string &name, float fallback) {
        const auto it = active_cfg.robotCfg.motor_torque_limit.find(name);
        if (it == active_cfg.robotCfg.motor_torque_limit.end())
        {
            return fallback;
        }
        return it->second;
    };

    const float hip_roll_limit = limit_or("hip_roll_joint", 330.0f);
    const float hip_yaw_limit = limit_or("hip_yaw_joint", 150.0f);
    const float ankle_limit = limit_or("ankle_left_motor", 90.0f);
    const float knee_limit = limit_or("knee_joint", 8000.0f);

    auto apply_joint_limits = [&](int index, int ref_index) {
        if (ref_index == 0 || ref_index == 2)
        {
            target_tau[static_cast<size_t>(index)] = std::clamp(target_tau[static_cast<size_t>(index)], -hip_roll_limit, hip_roll_limit);
        }
        else if (ref_index == 1)
        {
            target_tau[static_cast<size_t>(index)] = std::clamp(target_tau[static_cast<size_t>(index)], -hip_yaw_limit, hip_yaw_limit);
        }
        else if (ref_index == 3)
        {
            target_tau[static_cast<size_t>(index)] = std::clamp(target_tau[static_cast<size_t>(index)], -knee_limit, knee_limit);
        }
        else if (ref_index == 4 || ref_index == 5)
        {
            target_tau[static_cast<size_t>(index)] = std::clamp(target_tau[static_cast<size_t>(index)], -ankle_limit, ankle_limit);
        }
    };

    for (size_t i = 0; i < target_tau.size(); ++i)
    {
        apply_joint_limits(static_cast<int>(i), static_cast<int>(i % 6));
    }

    joint_target_torque = target_tau;
    return target_tau;
}

std::vector<float> RL_controller::get_joint_target_q(const std::vector<float> &policy_action)
{
    const Sim2realCfg &active_cfg = activePolicyCfg();
    const std::vector<int> &index_map = currentActionIndexMap();
    std::vector<float> target_q(rl_master::kLegJointCount, 0.0f);

    for (size_t robot_idx = 0; robot_idx < target_q.size(); ++robot_idx)
    {
        float action_value = 0.0f;
        if (robot_idx < static_cast<size_t>(active_cfg.action_dim))
        {
            int policy_idx = static_cast<int>(robot_idx);
            if (robot_idx < index_map.size())
            {
                policy_idx = index_map[robot_idx];
            }

            if (policy_idx >= 0 && static_cast<size_t>(policy_idx) < policy_action.size())
            {
                action_value = policy_action[static_cast<size_t>(policy_idx)];
            }
        }

        target_q[robot_idx] = robot->default_angle[robot_idx] + action_value * active_cfg.action_scale;
    }

    joint_target_q = target_q;
    return target_q;
}
