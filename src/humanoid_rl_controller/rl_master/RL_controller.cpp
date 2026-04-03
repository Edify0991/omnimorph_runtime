/*
onnxruntime:
https://blog.csdn.net/yangyu0515/article/details/142093965
https://blog.csdn.net/m0_57254760/article/details/138304321
*/
#include "rl_master/RL_controller.h"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "rl_master/rl_protocol.h"

std::ofstream rl_debug_file;

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
    if (rl_debug_file.is_open())
    {
        rl_debug_file.flush();
        rl_debug_file.close();
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
    if (policy_index == 0)
    {
        return stand_action_index_map_;
    }
    return walk_action_index_map_;
}

const std::vector<int> &RL_controller::currentObsIndexMap() const
{
    if (policy_index == 0)
    {
        return stand_obs_index_map_;
    }
    return walk_obs_index_map_;
}

const Sim2realCfg &RL_controller::activePolicyCfg() const
{
    if (policy_index == 0)
    {
        return robot->standSim2RealCfg;
    }
    return robot->sim2realCfg;
}

const ObservationBuilder &RL_controller::activeObservationBuilder() const
{
    if (policy_index == 0)
    {
        return *stand_observation_builder_;
    }
    return *walk_observation_builder_;
}

RL_controller::PolicyRuntimeGroup &RL_controller::activePolicyGroup()
{
    if (policy_index == 0)
    {
        return stand_policy_group_;
    }
    return walk_policy_group_;
}

const ReferenceMotionProvider &RL_controller::activeReferenceMotionProvider() const
{
    if (policy_index == 0)
    {
        return stand_reference_motion_;
    }
    return walk_reference_motion_;
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
    walk_mode = requested_mode;

    if (sanitize_invalid_mode && walk_mode != WALK && walk_mode != STAND && walk_mode != FIX_STAND)
    {
        walk_mode = WALK;
    }

    if (walk_mode == STAND || walk_mode == FIX_STAND)
    {
        policy_index = 0;
        robot->default_angle = robot->default_angle_stand;
    }
    else
    {
        policy_index = 1;
        robot->default_angle = robot->default_angle_walk;
    }
}

void RL_controller::handlePolicySwitch()
{
    if (last_policy_index_ == policy_index)
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

    last_policy_index_ = policy_index;
    std::cout << "[RL_controller] switch policy to "
              << (policy_index == 0 ? "stand" : "walk")
              << ", mode=" << walk_mode << std::endl;
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

void RL_controller::init_onnx_session()
{
    initPolicyGroup(robot->sim2realCfg, "walk", &walk_policy_group_);
    initPolicyGroup(robot->standSim2RealCfg, "stand", &stand_policy_group_);

    initReferenceMotionProvider(robot->sim2realCfg, &walk_reference_motion_, "walk");
    initReferenceMotionProvider(robot->standSim2RealCfg, &stand_reference_motion_, "stand");
}

void RL_controller::RL_controller_Init()
{
    robot->initialize_buffers();
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.dyaw = 0.0f;
    stand_flag = std::vector<float>(1, 0.0f);

    walk_action_index_map_ = buildActionIndexMap(robot->sim2realCfg, "sim2real");
    stand_action_index_map_ = buildActionIndexMap(robot->standSim2RealCfg, "stand_sim2real");
    walk_obs_index_map_ = buildObsIndexMap(robot->sim2realCfg, "sim2real");
    stand_obs_index_map_ = buildObsIndexMap(robot->standSim2RealCfg, "stand_sim2real");

    walk_observation_manifest_ = ObservationManifest::loadFromYAML(robot->sim2realCfg.observation_manifest_path);
    walk_observation_builder_ = std::make_unique<ObservationBuilder>(walk_observation_manifest_);
    if (walk_observation_builder_->expectedDim() != static_cast<size_t>(robot->sim2realCfg.obs_dim))
    {
        throw std::runtime_error(
            "walk observation manifest dim (" + std::to_string(walk_observation_builder_->expectedDim()) +
            ") does not match cfg obs_dim (" + std::to_string(robot->sim2realCfg.obs_dim) + ")");
    }
    std::cout << "Walk observation manifest: " << robot->sim2realCfg.observation_manifest_path << std::endl;
    for (const auto &line : walk_observation_builder_->layoutDescription())
    {
        std::cout << "  - " << line << std::endl;
    }

    stand_observation_manifest_ = ObservationManifest::loadFromYAML(robot->standSim2RealCfg.observation_manifest_path);
    stand_observation_builder_ = std::make_unique<ObservationBuilder>(stand_observation_manifest_);
    if (stand_observation_builder_->expectedDim() != static_cast<size_t>(robot->standSim2RealCfg.obs_dim))
    {
        throw std::runtime_error(
            "stand observation manifest dim (" + std::to_string(stand_observation_builder_->expectedDim()) +
            ") does not match cfg obs_dim (" + std::to_string(robot->standSim2RealCfg.obs_dim) + ")");
    }
    std::cout << "Stand observation manifest: " << robot->standSim2RealCfg.observation_manifest_path << std::endl;
    for (const auto &line : stand_observation_builder_->layoutDescription())
    {
        std::cout << "  - " << line << std::endl;
    }
    std::cout << "Action/Observation order remap initialized for walk and stand policy." << std::endl;

    init_onnx_session();

    refreshPolicyMode(WALK, true);
    handlePolicySwitch();
    deploy_state_machine_initialized_ = false;
    last_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;

    start_time = std::chrono::high_resolution_clock::now();
    if (robot->sim2realCfg.save_data_flag)
    {
        rl_debug_file.open(robot->sim2realCfg.data_path + "_rl_controller_data.txt");
        std::cout << "RL Controller debug file: " << robot->sim2realCfg.data_path + "_rl_controller_data.txt" << std::endl;
    }
}

rl_master::RobotCommandData RL_controller::step(
    const rl_master::RobotStateData &state,
    const rl_master::TeleopCommand &command,
    int requested_walk_mode,
    double phase_t)
{
    updateStateFromIO(state);
    updateCommandFromIO(command);

    if (!deploy_state_machine_initialized_)
    {
        refreshPolicyMode(WALK, true);
        handlePolicySwitch();
        deploy_state_machine_.configure(activePolicyCfg());
        deploy_state_machine_.initialize(robot->joint_q, activeZeroPose());
        deploy_state_machine_initialized_ = true;
        last_deploy_state_ = deploy_state_machine_.state();
    }

    const double now_s = rl_master::monotonicTimeSec();
    const auto deploy_output = deploy_state_machine_.update(requested_walk_mode, now_s, robot->joint_q);

    refreshPolicyMode(deploy_output.locomotion_mode, true);
    handlePolicySwitch();

    if (deploy_output.state != last_deploy_state_)
    {
        std::cout << "[RL_controller] lifecycle -> "
                  << rl_master::DeployStateMachine::stateName(deploy_output.state)
                  << std::endl;
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
    return out_cmd;
}

void RL_controller::estop()
{
    robot->open_rl = rl_master::kOpenRlDisabled;
}

std::vector<float> RL_controller::get_robot_observation(double phase_t)
{
    const auto &active_cfg = activePolicyCfg();

    if (active_cfg.save_data_flag && rl_debug_file.is_open())
    {
        rl_debug_file << "w: " << robot->base_ang_vel[0] << ", " << robot->base_ang_vel[1] << ", " << robot->base_ang_vel[2] << std::endl;
    }

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

    if (rl_debug_file.is_open() && policy_action.size() > 9)
    {
        rl_debug_file << "action: " << policy_action[3] << ", " << policy_action[9] << std::endl;
    }

    joint_target_q = target_q;
    return target_q;
}
