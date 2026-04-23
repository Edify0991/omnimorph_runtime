#ifndef RL_CONTROLLER_H
#define RL_CONTROLLER_H

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "cmd.h"
#include "deploy_state_machine.h"
#include "external_observation_provider.h"
#include "logging/runtime_log_types.h"
#include "math_tool.h"
#include "mode_profile_registry.h"
#include "onnx_policy_runner.h"
#include "observation_builder.h"
#include "reference_motion_provider.h"
#include "rl_cfg.h"
#include "robot_types.h"
#include "robot_state.h"

class RL_controller
{
public:
    RL_controller();
    explicit RL_controller(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry);
    ~RL_controller();

    static std::unique_ptr<RL_controller> create(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry = nullptr);
    void RL_controller_Init(int startup_mode_id = rl_master::kModeCodeMin);
    rl_master::RobotCommandData step(
        const rl_master::RobotStateData &state,
        const rl_master::TeleopCommand &command,
        int mode_command,
        double phase_t);
    void estop();
    std::vector<float> get_robot_observation(double phase_t);
    std::vector<float> run_policy(std::deque<std::vector<float>> *obs_deque = nullptr);
    std::deque<std::vector<float>> update_obs_deque(const std::vector<float> &obs);
    std::vector<float> get_joint_target_torque(const std::vector<float> &target_q);
    std::vector<float> get_joint_target_q(const std::vector<float> &action);

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::unique_ptr<RobotState> robot;
    const Sim2realCfg &runtimeCfg() const;
    int activeModeId() const;
    const std::string &activeConfigSection() const;
    const rl_master::logging::ControllerLogSnapshot &latestLogSnapshot() const;
    void setExternalObservationFeature(const std::string &name, const std::vector<float> &values);
    void setExternalObservationFeature(const std::string &name, const std::vector<float> &values, double monotonic_time_sec);
    std::vector<rl_master::logging::RuntimeSourceSampleRecord> drainExternalObservationSamplesForLogging();

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

    struct ModeProfileSpec
    {
        int mode_id = rl_master::kModeCodeMin;
        std::string config_section;
        std::string tag;
    };

    struct ModeProfile
    {
        int mode_id = rl_master::kModeCodeMin;
        std::string config_section;
        std::string tag;
        Sim2realCfg cfg;
        std::vector<std::string> joint_names;
        std::vector<float> default_angle;
        std::vector<float> zero_pose;
        std::vector<int> action_robot_indices;
        std::vector<int> obs_index_map;
        std::vector<int> reference_index_map;
        ObservationManifest observation_manifest;
        std::unique_ptr<ObservationBuilder> observation_builder;
        PolicyRuntimeGroup policy_group;
        std::unique_ptr<OnnxPolicyRunner> amp_discriminator;
        ReferenceMotionProvider reference_motion;
        bool reference_alignment_initialized = false;
        std::vector<float> reference_anchor_init_pos_w;
        std::vector<float> reference_anchor_init_quat_w;
        std::vector<float> robot_base_init_quat_w;
    };

    const std::vector<int> &currentActionIndexMap() const;
    const std::vector<int> &currentObsIndexMap() const;
    const std::vector<int> &currentReferenceIndexMap() const;
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
    void initAmpDiscriminatorRunner(const Sim2realCfg &cfg, const std::string &tag, std::unique_ptr<OnnxPolicyRunner> *runner);
    void runAmpDiscriminator(
        const Sim2realCfg &cfg,
        const std::string &tag,
        OnnxPolicyRunner *runner,
        const std::vector<float> &current_observation,
        const std::vector<float> &stacked_observation);
    void resetPolicyScheduler();
    PolicyRunOutput runPolicyGroup(
        PolicyRuntimeGroup *group,
        const std::vector<float> &stacked_obs,
        const std::vector<float> &current_observation,
        const ObservationFeatureContext &feature_context);
    void initReferenceMotionProvider(const Sim2realCfg &cfg, ReferenceMotionProvider *provider, const std::string &tag);
    ObservationFeatureContext buildObservationFeatureContext(const Sim2realCfg &cfg, double phase_t);

    void initModeProfiles();
    std::vector<ModeProfileSpec> loadModeProfileSpecsFromYaml() const;
    size_t profileIndexForMode(int mode_id, bool sanitize_invalid_mode) const;
    ModeProfile &activeModeProfile();
    const ModeProfile &activeModeProfile() const;
    void syncActiveProfileToRobotState();

    Ort::Env onnx_env_;
    std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry_;
    std::vector<std::string> joint_order_;
    std::vector<ModeProfile> mode_profiles_;
    std::unordered_map<int, size_t> mode_to_profile_index_;
    int default_mode_id_ = rl_master::kModeCodeMin;
    size_t active_profile_index_ = 0;
    int active_mode_id_ = rl_master::kModeCodeMin;
    int last_active_mode_id_ = std::numeric_limits<int>::min();

    ExternalObservationProvider external_observation_provider_;
    rl_master::DeployStateMachine deploy_state_machine_;
    rl_master::DeployLifecycleState last_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;
    bool deploy_state_machine_initialized_ = false;
    size_t policy_step_counter_ = 0;
    bool policy_schedule_initialized_ = false;
    double next_policy_phase_t_ = 0.0;
    double last_policy_sample_time_sec_ = 0.0;
    double last_policy_sample_phase_t_ = 0.0;
    double phase_origin_t_ = 0.0;
    bool phase_origin_initialized_ = false;
    bool phase_reset_pending_ = true;

    std::vector<float> action;
    std::vector<float> joint_target_q;
    std::vector<float> joint_target_torque;
    std::unordered_map<std::string, std::vector<float>> latest_policy_extra_outputs_;
    ObservationFeatureContext latest_observation_feature_context_;

    std::vector<float> obs;
    std::deque<std::vector<float>> obs_deque;
    std::vector<float> stacked_obs_buffer_;

    Cmd cmd;
    rl_master::logging::ControllerLogSnapshot latest_log_snapshot_;
    uint64_t log_frame_index_ = 0;
};

#endif // RL_CONTROLLER_H
