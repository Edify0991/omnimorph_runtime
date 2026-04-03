#ifndef RL_CONTROLLER_H
#define RL_CONTROLLER_H

#include <onnxruntime_cxx_api.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>
#include "cmd.h"
#include "deploy_state_machine.h"
#include "external_observation_provider.h"
#include "math_tool.h"
#include "onnx_policy_runner.h"
#include "observation_builder.h"
#include "reference_motion_provider.h"
#include "rl_cfg.h"
#include "robot_types.h"
#include "robot_state.h"

enum RobotWalkMode
{
    WALK = 0,
    STAND = 1,
    FIX_STAND = 2
};

class RL_controller
{
public:
    RL_controller();
    ~RL_controller();

    static std::unique_ptr<RL_controller> create();
    void RL_controller_Init();
    rl_master::RobotCommandData step(
        const rl_master::RobotStateData &state,
        const rl_master::TeleopCommand &command,
        int walk_mode,
        double phase_t);
    void estop();
    std::vector<float> get_robot_observation(double phase_t);
    std::vector<float> run_policy(std::deque<std::vector<float>> *obs_deque = nullptr);
    std::deque<std::vector<float>> update_obs_deque(const std::vector<float> &obs);
    std::vector<float> pd_control(const std::vector<float> &target_q, const std::vector<float> &kp, const std::vector<float> &target_dq, const std::vector<float> &kd);
    std::vector<float> get_joint_target_torque(const std::vector<float> &target_q);
    std::vector<float> get_joint_target_q(const std::vector<float> &action);

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::unique_ptr<RobotState> robot;
    const Sim2realCfg &runtimeCfg() const { return robot->sim2realCfg; }

private:
    struct PolicyRunnerNode
    {
        std::string name;
        float weight = 1.0f;
        std::unique_ptr<OnnxPolicyRunner> runner;
    };

    struct PolicyRuntimeGroup
    {
        std::vector<PolicyRunnerNode> runners;
    };

    struct PolicyRunOutput
    {
        std::vector<float> action;
        std::unordered_map<std::string, std::vector<float>> extra_outputs;
    };

    static const std::array<std::string, rl_master::kLegJointCount> &canonicalJointOrder();
    std::vector<int> buildActionIndexMap(const Sim2realCfg &cfg, const std::string &cfg_name) const;
    std::vector<int> buildObsIndexMap(const Sim2realCfg &cfg, const std::string &cfg_name) const;
    const std::vector<int> &currentActionIndexMap() const;
    const std::vector<int> &currentObsIndexMap() const;
    const Sim2realCfg &activePolicyCfg() const;
    const ObservationBuilder &activeObservationBuilder() const;
    PolicyRuntimeGroup &activePolicyGroup();
    const ReferenceMotionProvider &activeReferenceMotionProvider() const;
    std::vector<float> activeZeroPose() const;
    void refreshPolicyMode(int requested_mode, bool sanitize_invalid_mode = true);
    void handlePolicySwitch();
    void updateStateFromIO(const rl_master::RobotStateData &state);
    void updateCommandFromIO(const rl_master::TeleopCommand &command);
    void initPolicyGroup(const Sim2realCfg &cfg, const std::string &tag, PolicyRuntimeGroup *group);
    PolicyRunOutput runPolicyGroup(PolicyRuntimeGroup *group, const std::vector<float> &stacked_obs);
    void initReferenceMotionProvider(const Sim2realCfg &cfg, ReferenceMotionProvider *provider, const std::string &tag);
    ObservationFeatureContext buildObservationFeatureContext(const Sim2realCfg &cfg, double phase_t);

    void init_onnx_session();

    Ort::Env onnx_env_;
    PolicyRuntimeGroup walk_policy_group_;
    PolicyRuntimeGroup stand_policy_group_;
    ReferenceMotionProvider walk_reference_motion_;
    ReferenceMotionProvider stand_reference_motion_;
    ExternalObservationProvider external_observation_provider_;
    rl_master::DeployStateMachine deploy_state_machine_;
    rl_master::DeployLifecycleState last_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;
    bool deploy_state_machine_initialized_ = false;
    size_t deploy_step_counter_ = 0;

    std::vector<float> action;
    std::vector<float> joint_target_q;
    std::vector<float> joint_target_torque;
    std::unordered_map<std::string, std::vector<float>> latest_policy_extra_outputs_;

    std::vector<float> obs;
    std::deque<std::vector<float>> obs_deque;
    std::vector<float> stacked_obs_buffer_;
    std::vector<int> walk_action_index_map_;
    std::vector<int> stand_action_index_map_;
    std::vector<int> walk_obs_index_map_;
    std::vector<int> stand_obs_index_map_;
    ObservationManifest walk_observation_manifest_;
    ObservationManifest stand_observation_manifest_;
    std::unique_ptr<ObservationBuilder> walk_observation_builder_;
    std::unique_ptr<ObservationBuilder> stand_observation_builder_;

    int filter_window = 5;
    std::deque<float> left_knee_target_filter;
    std::deque<float> right_knee_target_filter;
    float left_knee_target_now = 0.0f;
    float right_knee_target_now = 0.0f;

    float filter_range = 0.01f;
    bool use_filter_range = true;
    float vel_threshold = 0.01f;
    Cmd cmd;
    int walk_mode = WALK;
    int policy_index = 0;
    int last_policy_index_ = -1;
    std::vector<float> stand_flag;
};

#endif // RL_CONTROLLER_H
