#include "rl_master/policy_runtime.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace
{
const std::vector<float> &requireVector(const std::vector<float> *ptr, const char *name)
{
    if (!ptr)
    {
        throw std::runtime_error(std::string("PolicyExecutionRequest missing ") + name);
    }
    return *ptr;
}

const std::unordered_map<std::string, std::vector<float>> &requireFeatures(
    const std::unordered_map<std::string, std::vector<float>> *ptr)
{
    static const std::unordered_map<std::string, std::vector<float>> kEmpty;
    if (!ptr)
    {
        return kEmpty;
    }
    return *ptr;
}
} // namespace

OnnxPolicyAdapter::OnnxPolicyAdapter(
    Ort::Env &env,
    const std::string &model_path,
    const Sim2realCfg &cfg,
    std::string policy_tag)
    : runner_(std::make_unique<OnnxPolicyRunner>(
          env,
          model_path,
          cfg,
          std::move(policy_tag)))
{
}

void OnnxPolicyAdapter::init()
{
    runner_->init();
}

void OnnxPolicyAdapter::reset()
{
    runner_->reset();
}

PolicyInferenceResult OnnxPolicyAdapter::infer(const PolicyExecutionRequest &request)
{
    return runner_->forward(
        requireVector(request.stacked_observation, "stacked_observation"),
        requireVector(request.current_observation, "current_observation"),
        requireFeatures(request.features),
        request.advance_time_step);
}

std::unordered_map<std::string, std::vector<float>> OnnxPolicyAdapter::prefetchExtraOutputs(
    const std::vector<std::string> &extra_output_names,
    const PolicyExecutionRequest &request)
{
    return runner_->prefetchExtraOutputs(
        extra_output_names,
        requireVector(request.stacked_observation, "stacked_observation"),
        requireVector(request.current_observation, "current_observation"),
        requireFeatures(request.features),
        request.advance_time_step);
}

std::string OnnxPolicyAdapter::summary() const
{
    return runner_->summary();
}

void SyncWeightedInferenceStrategy::configure(const PolicyInferenceConfig &config)
{
    config_ = config;
}

void SyncWeightedInferenceStrategy::reset()
{
}

PolicyGroupExecutionResult SyncWeightedInferenceStrategy::execute(
    const std::vector<PolicyAdapterNodeView> &nodes,
    const PolicyExecutionRequest &request)
{
    if (nodes.empty())
    {
        throw std::runtime_error("SyncWeightedInferenceStrategy::execute got empty node list");
    }

    PolicyGroupExecutionResult output;
    output.action.assign(config_.expected_action_dim, 0.0f);

    float total_weight = 0.0f;
    bool primary_done = false;
    for (const auto &node : nodes)
    {
        if (!node.adapter)
        {
            continue;
        }

        PolicyInferenceResult result = node.adapter->infer(request);
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

std::unordered_map<std::string, std::vector<float>> SyncWeightedInferenceStrategy::prefetchPrimaryExtraOutputs(
    const std::vector<PolicyAdapterNodeView> &nodes,
    const std::vector<std::string> &extra_output_names,
    const PolicyExecutionRequest &request)
{
    if (nodes.empty() || !nodes.front().adapter)
    {
        return {};
    }
    return nodes.front().adapter->prefetchExtraOutputs(extra_output_names, request);
}

void ChunkedRecedingInferenceStrategy::configure(const PolicyInferenceConfig &config)
{
    config_ = config;
    if (config_.action_chunk_steps <= 0)
    {
        config_.action_chunk_steps = 1;
    }
    if (config_.action_chunk_execute_steps <= 0)
    {
        config_.action_chunk_execute_steps = 1;
    }
    if (config_.action_chunk_replan_interval <= 0)
    {
        config_.action_chunk_replan_interval = config_.action_chunk_execute_steps;
    }
}

void ChunkedRecedingInferenceStrategy::reset()
{
    pending_actions_.clear();
    cached_extra_outputs_.clear();
    ticks_since_last_inference_ = 0;
}

PolicyGroupExecutionResult ChunkedRecedingInferenceStrategy::execute(
    const std::vector<PolicyAdapterNodeView> &nodes,
    const PolicyExecutionRequest &request)
{
    if (nodes.empty())
    {
        throw std::runtime_error("ChunkedRecedingInferenceStrategy::execute got empty node list");
    }

    const bool need_replan =
        pending_actions_.empty() ||
        ticks_since_last_inference_ >= config_.action_chunk_replan_interval;
    if (need_replan)
    {
        PolicyGroupExecutionResult inferred = runChunkInference(nodes, request);
        cached_extra_outputs_ = inferred.extra_outputs;
        ticks_since_last_inference_ = 0;
    }

    if (pending_actions_.empty())
    {
        throw std::runtime_error(
            "ChunkedRecedingInferenceStrategy has no pending action after inference");
    }

    PolicyGroupExecutionResult output;
    output.action = std::move(pending_actions_.front());
    pending_actions_.pop_front();
    output.extra_outputs = cached_extra_outputs_;
    ++ticks_since_last_inference_;
    return output;
}

std::unordered_map<std::string, std::vector<float>> ChunkedRecedingInferenceStrategy::prefetchPrimaryExtraOutputs(
    const std::vector<PolicyAdapterNodeView> &nodes,
    const std::vector<std::string> &extra_output_names,
    const PolicyExecutionRequest &request)
{
    if (nodes.empty() || !nodes.front().adapter)
    {
        return {};
    }
    return nodes.front().adapter->prefetchExtraOutputs(extra_output_names, request);
}

PolicyGroupExecutionResult ChunkedRecedingInferenceStrategy::runChunkInference(
    const std::vector<PolicyAdapterNodeView> &nodes,
    const PolicyExecutionRequest &request)
{
    PolicyGroupExecutionResult blended;
    blended.action.assign(config_.expected_action_dim * static_cast<size_t>(config_.action_chunk_steps), 0.0f);

    float total_weight = 0.0f;
    bool primary_done = false;
    for (const auto &node : nodes)
    {
        if (!node.adapter)
        {
            continue;
        }

        PolicyInferenceResult result = node.adapter->infer(request);
        const size_t expected_flat_dim =
            config_.expected_action_dim * static_cast<size_t>(config_.action_chunk_steps);
        if (result.action.size() < expected_flat_dim)
        {
            throw std::runtime_error(
                "ChunkedRecedingInferenceStrategy expects flattened chunk action dim >= " +
                std::to_string(expected_flat_dim) + ", got " + std::to_string(result.action.size()));
        }

        const float weight = primary_done ? node.weight : 1.0f;
        for (size_t i = 0; i < expected_flat_dim; ++i)
        {
            blended.action[i] += weight * result.action[i];
        }
        total_weight += weight;
        for (auto &kv : result.extra_outputs)
        {
            blended.extra_outputs[node.name + "/" + kv.first] = std::move(kv.second);
        }
        primary_done = true;
    }

    if (total_weight > 1e-6f)
    {
        for (auto &value : blended.action)
        {
            value /= total_weight;
        }
    }

    pending_actions_.clear();
    const size_t step_dim = config_.expected_action_dim;
    const size_t execute_steps = static_cast<size_t>(config_.action_chunk_execute_steps);
    for (size_t step = 0; step < execute_steps; ++step)
    {
        const size_t offset = step * step_dim;
        pending_actions_.emplace_back(
            blended.action.begin() + static_cast<std::ptrdiff_t>(offset),
            blended.action.begin() + static_cast<std::ptrdiff_t>(offset + step_dim));
    }
    return blended;
}
