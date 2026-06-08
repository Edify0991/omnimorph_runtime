#ifndef RL_MASTER_POLICY_RUNTIME_H
#define RL_MASTER_POLICY_RUNTIME_H

#include <deque>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "onnx_policy_runner.h"
#include "rl_cfg.h"

struct PolicyExecutionRequest
{
    const std::vector<float> *stacked_observation = nullptr;
    const std::vector<float> *current_observation = nullptr;
    const std::vector<float> *last_action = nullptr;
    const std::unordered_map<std::string, std::vector<float>> *features = nullptr;
    bool advance_time_step = true;
    uint64_t policy_step_index = 0;
};

struct PolicyGroupExecutionResult
{
    std::vector<float> action;
    std::unordered_map<std::string, std::vector<float>> extra_outputs;
};

struct PolicyInferenceConfig
{
    std::string strategy_name = "sync_step";
    size_t expected_action_dim = 0;
    int action_chunk_steps = 1;
    int action_chunk_execute_steps = 1;
    int action_chunk_replan_interval = 1;
};

class PolicyAdapter
{
public:
    virtual ~PolicyAdapter() = default;

    virtual void init() = 0;
    virtual void reset() = 0;
    virtual PolicyInferenceResult infer(const PolicyExecutionRequest &request) = 0;
    virtual std::unordered_map<std::string, std::vector<float>> prefetchExtraOutputs(
        const std::vector<std::string> &extra_output_names,
        const PolicyExecutionRequest &request) = 0;
    virtual std::vector<std::string> effectiveExecutionProviders() const { return {}; }
    virtual std::string summary() const = 0;
};

class OnnxPolicyAdapter final : public PolicyAdapter
{
public:
    OnnxPolicyAdapter(
        Ort::Env &env,
        const std::string &model_path,
        const Sim2realCfg &cfg,
        std::string policy_tag);

    void init() override;
    void reset() override;
    PolicyInferenceResult infer(const PolicyExecutionRequest &request) override;
    std::unordered_map<std::string, std::vector<float>> prefetchExtraOutputs(
        const std::vector<std::string> &extra_output_names,
        const PolicyExecutionRequest &request) override;
    const std::unordered_map<std::string, std::string> &customMetadata() const;
    std::vector<std::string> effectiveExecutionProviders() const override;
    std::string summary() const override;

private:
    std::unique_ptr<OnnxPolicyRunner> runner_;
};

struct PolicyAdapterNodeView
{
    std::string name;
    float weight = 1.0f;
    std::vector<int> primary_action_indices;
    PolicyAdapter *adapter = nullptr;
};

class PolicyInferenceStrategy
{
public:
    virtual ~PolicyInferenceStrategy() = default;

    virtual void configure(const PolicyInferenceConfig &config) = 0;
    virtual void reset() = 0;

    virtual PolicyGroupExecutionResult execute(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const PolicyExecutionRequest &request) = 0;

    virtual std::unordered_map<std::string, std::vector<float>> prefetchPrimaryExtraOutputs(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const std::vector<std::string> &extra_output_names,
        const PolicyExecutionRequest &request) = 0;
};

class SyncWeightedInferenceStrategy final : public PolicyInferenceStrategy
{
public:
    void configure(const PolicyInferenceConfig &config) override;
    void reset() override;

    PolicyGroupExecutionResult execute(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const PolicyExecutionRequest &request) override;

    std::unordered_map<std::string, std::vector<float>> prefetchPrimaryExtraOutputs(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const std::vector<std::string> &extra_output_names,
        const PolicyExecutionRequest &request) override;

private:
    PolicyInferenceConfig config_;
};

class ChunkedRecedingInferenceStrategy final : public PolicyInferenceStrategy
{
public:
    void configure(const PolicyInferenceConfig &config) override;
    void reset() override;

    PolicyGroupExecutionResult execute(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const PolicyExecutionRequest &request) override;

    std::unordered_map<std::string, std::vector<float>> prefetchPrimaryExtraOutputs(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const std::vector<std::string> &extra_output_names,
        const PolicyExecutionRequest &request) override;

private:
    PolicyGroupExecutionResult runChunkInference(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const PolicyExecutionRequest &request);

    PolicyInferenceConfig config_;
    std::deque<std::vector<float>> pending_actions_;
    std::unordered_map<std::string, std::vector<float>> cached_extra_outputs_;
    int ticks_since_last_inference_ = 0;
};

class ResidualAdditiveInferenceStrategy final : public PolicyInferenceStrategy
{
public:
    void configure(const PolicyInferenceConfig &config) override;
    void reset() override;

    PolicyGroupExecutionResult execute(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const PolicyExecutionRequest &request) override;

    std::unordered_map<std::string, std::vector<float>> prefetchPrimaryExtraOutputs(
        const std::vector<PolicyAdapterNodeView> &nodes,
        const std::vector<std::string> &extra_output_names,
        const PolicyExecutionRequest &request) override;

private:
    PolicyInferenceConfig config_;
};

#endif // RL_MASTER_POLICY_RUNTIME_H
