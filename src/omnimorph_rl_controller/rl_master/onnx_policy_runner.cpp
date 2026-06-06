#include "rl_master/onnx_policy_runner.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace
{
std::string trimCopy(const std::string &input)
{
    size_t begin = 0;
    while (begin < input.size() &&
           (input[begin] == ' ' || input[begin] == '\t' || input[begin] == '\n' || input[begin] == '\r'))
    {
        ++begin;
    }
    size_t end = input.size();
    while (end > begin &&
           (input[end - 1] == ' ' || input[end - 1] == '\t' || input[end - 1] == '\n' || input[end - 1] == '\r'))
    {
        --end;
    }
    return input.substr(begin, end - begin);
}

std::vector<std::string> parseCsvList(const std::string &input)
{
    std::vector<std::string> items;
    std::stringstream ss(input);
    std::string item;
    while (std::getline(ss, item, ','))
    {
        const std::string trimmed = trimCopy(item);
        if (!trimmed.empty())
        {
            items.push_back(trimmed);
        }
    }
    return items;
}

std::string joinStrings(const std::vector<std::string> &items)
{
    std::ostringstream oss;
    for (size_t i = 0; i < items.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << items[i];
    }
    return oss.str();
}

bool parseInt64Value(const std::string &text, int64_t *out)
{
    if (!out)
    {
        return false;
    }
    const std::string normalized = trimCopy(text);
    if (normalized.empty())
    {
        return false;
    }
    errno = 0;
    char *end_ptr = nullptr;
    const long long value = std::strtoll(normalized.c_str(), &end_ptr, 10);
    if (end_ptr == normalized.c_str() || *end_ptr != '\0' || errno != 0)
    {
        return false;
    }
    *out = static_cast<int64_t>(value);
    return true;
}
} // namespace

OnnxPolicyRunner::OnnxPolicyRunner(
    Ort::Env &env,
    const std::string &model_path,
    const Sim2realCfg &cfg,
    std::string policy_tag)
    : env_(env),
      model_path_(model_path),
      cfg_(cfg),
      policy_tag_(std::move(policy_tag))
{
}

void OnnxPolicyRunner::init()
{
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_options_.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    session_options_.SetIntraOpNumThreads(std::max(1, cfg_.onnx_intra_threads));
    session_options_.SetInterOpNumThreads(std::max(1, cfg_.onnx_inter_threads));

    session_ = std::make_unique<Ort::Session>(env_, model_path_.c_str(), session_options_);

    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_count = static_cast<int>(session_->GetInputCount());
    const auto output_count = static_cast<int>(session_->GetOutputCount());
    if (input_count <= 0 || output_count <= 0)
    {
        throw std::runtime_error("[" + policy_tag_ + "] invalid ONNX model IO count");
    }

    input_names_.clear();
    output_names_.clear();
    input_names_.reserve(static_cast<size_t>(input_count));
    output_names_.reserve(static_cast<size_t>(output_count));

    for (int i = 0; i < input_count; ++i)
    {
        auto input_name = session_->GetInputNameAllocated(i, allocator);
        input_names_.emplace_back(input_name.get());
    }
    for (int i = 0; i < output_count; ++i)
    {
        auto output_name = session_->GetOutputNameAllocated(i, allocator);
        output_names_.emplace_back(output_name.get());
    }

    action_output_index_ = findOutputIndexByName(cfg_.action_output_name);
    if (action_output_index_ < 0 && !cfg_.strict_model_io)
    {
        action_output_index_ = 0;
    }
    if (action_output_index_ < 0)
    {
        throw std::runtime_error(
            "[" + policy_tag_ + "] action output '" + cfg_.action_output_name + "' not found");
    }

    selected_output_indices_.clear();
    selected_output_names_.clear();
    selected_output_indices_.push_back(action_output_index_);
    selected_output_names_.push_back(output_names_[static_cast<size_t>(action_output_index_)]);

    extra_output_indices_.clear();
    for (const auto &name : cfg_.extra_output_names)
    {
        const int output_index = findOutputIndexByName(name);
        if (output_index < 0)
        {
            if (cfg_.strict_model_io)
            {
                throw std::runtime_error("[" + policy_tag_ + "] extra output '" + name + "' not found");
            }
            std::cerr << "[" << policy_tag_ << "] extra output not found, ignored: " << name << std::endl;
            continue;
        }
        if (std::find(selected_output_indices_.begin(), selected_output_indices_.end(), output_index) != selected_output_indices_.end())
        {
            continue;
        }
        extra_output_indices_.push_back(output_index);
        selected_output_indices_.push_back(output_index);
        selected_output_names_.push_back(output_names_[static_cast<size_t>(output_index)]);
    }

    action_output_selected_index_ = -1;
    for (size_t i = 0; i < selected_output_indices_.size(); ++i)
    {
        if (selected_output_indices_[i] == action_output_index_)
        {
            action_output_selected_index_ = static_cast<int>(i);
            break;
        }
    }
    if (action_output_selected_index_ < 0)
    {
        throw std::runtime_error("[" + policy_tag_ + "] failed to resolve action output index");
    }

    input_bindings_.clear();
    input_buffers_.clear();
    std::vector<bool> bound_inputs(static_cast<size_t>(input_count), false);

    auto defaultShapeForInput = [&](int input_index) {
        try
        {
            auto input_type_info = session_->GetInputTypeInfo(input_index);
            auto tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
            auto normalized_shape = normalizedShape(tensor_info.GetShape());
            if (normalized_shape.empty())
            {
                normalized_shape = {1};
            }
            return normalized_shape;
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error(
                "[" + policy_tag_ + "] failed to query ONNX shape for input '" +
                input_names_[static_cast<size_t>(input_index)] + "': " + e.what() +
                ". Configure policy_io.onnx_inputs[*].shape explicitly if this input cannot be inferred.");
        }
    };

    auto appendBinding = [&](InputBinding binding) {
        if (binding.input_index < 0 || binding.input_index >= input_count)
        {
            throw std::runtime_error("[" + policy_tag_ + "] invalid input binding index");
        }
        if (binding.shape.empty())
        {
            binding.shape = defaultShapeForInput(binding.input_index);
        }
        bound_inputs[static_cast<size_t>(binding.input_index)] = true;
        input_bindings_.push_back(std::move(binding));

        AuxInputBuffer buffer;
        buffer.shape = input_bindings_.back().shape;
        buffer.data.assign(elementCountFromShape(buffer.shape), 0.0f);
        input_buffers_.push_back(std::move(buffer));
    };

    if (!cfg_.onnx_inputs.empty())
    {
        for (const auto &spec : cfg_.onnx_inputs)
        {
            if (spec.name.empty())
            {
                throw std::runtime_error("[" + policy_tag_ + "] onnx_inputs item missing name");
            }
            const int input_index = findInputIndexByName(spec.name);
            if (input_index < 0)
            {
                throw std::runtime_error("[" + policy_tag_ + "] configured input '" + spec.name + "' not found");
            }

            InputBinding binding;
            binding.input_index = input_index;
            binding.name = spec.name;
            binding.source = spec.source;
            binding.feature_name = spec.feature_name;
            binding.feature_names = spec.feature_names;
            binding.shape = spec.shape;
            binding.constant = spec.constant;
            appendBinding(std::move(binding));
        }

        std::vector<std::string> unbound_names;
        for (int input_index = 0; input_index < input_count; ++input_index)
        {
            if (!bound_inputs[static_cast<size_t>(input_index)])
            {
                unbound_names.push_back(input_names_[static_cast<size_t>(input_index)]);
            }
        }
        if (!unbound_names.empty())
        {
            std::ostringstream oss;
            for (size_t i = 0; i < unbound_names.size(); ++i)
            {
                if (i > 0)
                {
                    oss << ", ";
                }
                oss << unbound_names[i];
            }
            throw std::runtime_error("[" + policy_tag_ + "] onnx_inputs does not cover model inputs: " + oss.str());
        }
    }
    else
    {
        int obs_input_index = findInputIndexByName(cfg_.obs_input_name);
        if (obs_input_index < 0 && !cfg_.strict_model_io)
        {
            obs_input_index = 0;
        }
        if (obs_input_index < 0)
        {
            throw std::runtime_error(
                "[" + policy_tag_ + "] observation input '" + cfg_.obs_input_name + "' not found");
        }

        InputBinding obs_binding;
        obs_binding.input_index = obs_input_index;
        obs_binding.name = input_names_[static_cast<size_t>(obs_input_index)];
        obs_binding.source = "stacked_observation";
        appendBinding(std::move(obs_binding));

        int timestep_input_index = findInputIndexByName(cfg_.time_step_input_name);
        if (cfg_.enable_time_step_input)
        {
            if (timestep_input_index < 0)
            {
                throw std::runtime_error(
                    "[" + policy_tag_ + "] time_step input '" + cfg_.time_step_input_name + "' not found");
            }
        }
        if (timestep_input_index >= 0 && timestep_input_index != obs_input_index)
        {
            InputBinding time_binding;
            time_binding.input_index = timestep_input_index;
            time_binding.name = input_names_[static_cast<size_t>(timestep_input_index)];
            time_binding.source = "time_step";
            time_binding.shape = {1, 1};
            appendBinding(std::move(time_binding));
        }

        std::vector<std::string> zero_filled_inputs;
        for (int input_index = 0; input_index < input_count; ++input_index)
        {
            if (bound_inputs[static_cast<size_t>(input_index)])
            {
                continue;
            }
            if (cfg_.strict_model_io)
            {
                throw std::runtime_error(
                    "[" + policy_tag_ + "] unknown model input '" +
                    input_names_[static_cast<size_t>(input_index)] + "'");
            }

            InputBinding zero_binding;
            zero_binding.input_index = input_index;
            zero_binding.name = input_names_[static_cast<size_t>(input_index)];
            zero_binding.source = "constant";
            zero_binding.shape = defaultShapeForInput(input_index);
            zero_binding.constant.assign(elementCountFromShape(zero_binding.shape), 0.0f);
            appendBinding(std::move(zero_binding));
            zero_filled_inputs.push_back(input_names_[static_cast<size_t>(input_index)]);
        }

        if (!zero_filled_inputs.empty())
        {
            std::ostringstream oss;
            for (size_t i = 0; i < zero_filled_inputs.size(); ++i)
            {
                if (i > 0)
                {
                    oss << ", ";
                }
                oss << zero_filled_inputs[i];
            }
            std::cerr << "[" << policy_tag_ << "] fill unspecified inputs with zeros: " << oss.str() << std::endl;
        }
    }

    validateModelMetadata();
    reset();
    std::cout << summary() << std::endl;
}

void OnnxPolicyRunner::reset()
{
    time_step_ = cfg_.time_step_start;
}

PolicyInferenceResult OnnxPolicyRunner::forward(
    const std::vector<float> &stacked_observation,
    const std::vector<float> &current_observation,
    const std::vector<float> &last_action,
    const std::unordered_map<std::string, std::vector<float>> &features,
    bool advance_time_step)
{
    return runSelectedOutputs(
        selected_output_names_,
        stacked_observation,
        current_observation,
        last_action,
        features,
        advance_time_step);
}

std::unordered_map<std::string, std::vector<float>> OnnxPolicyRunner::prefetchExtraOutputs(
    const std::vector<std::string> &extra_output_names,
    const std::vector<float> &stacked_observation,
    const std::vector<float> &current_observation,
    const std::vector<float> &last_action,
    const std::unordered_map<std::string, std::vector<float>> &features,
    bool advance_time_step)
{
    if (extra_output_names.empty())
    {
        return {};
    }
    return runSelectedOutputs(
               extra_output_names,
               stacked_observation,
               current_observation,
               last_action,
               features,
               advance_time_step)
        .extra_outputs;
}

PolicyInferenceResult OnnxPolicyRunner::runSelectedOutputs(
    const std::vector<std::string> &requested_output_names,
    const std::vector<float> &stacked_observation,
    const std::vector<float> &current_observation,
    const std::vector<float> &last_action,
    const std::unordered_map<std::string, std::vector<float>> &features,
    bool advance_time_step)
{
    if (!session_)
    {
        throw std::runtime_error("[" + policy_tag_ + "] forward called before init()");
    }
    if (stacked_observation.empty() && current_observation.empty())
    {
        throw std::runtime_error("[" + policy_tag_ + "] observation inputs are empty");
    }
    if (requested_output_names.empty())
    {
        return {};
    }

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<const char *> input_name_ptrs;
    std::vector<Ort::Value> input_tensors;
    input_name_ptrs.reserve(input_bindings_.size());
    input_tensors.reserve(input_bindings_.size());

    for (size_t binding_index = 0; binding_index < input_bindings_.size(); ++binding_index)
    {
        const auto &binding = input_bindings_[binding_index];
        auto &buffer = input_buffers_[binding_index];
        buffer.data = resolveInputData(binding, stacked_observation, current_observation, last_action, features);
        input_name_ptrs.push_back(binding.name.c_str());
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
            memory_info,
            buffer.data.data(),
            buffer.data.size(),
            buffer.shape.data(),
            buffer.shape.size()));
    }

    std::vector<std::string> resolved_output_names;
    resolved_output_names.reserve(requested_output_names.size());
    for (const auto &requested_name : requested_output_names)
    {
        const int output_index = findOutputIndexByName(requested_name);
        if (output_index < 0)
        {
            if (cfg_.strict_model_io)
            {
                throw std::runtime_error("[" + policy_tag_ + "] requested output '" + requested_name + "' not found");
            }
            continue;
        }
        const std::string &resolved_name = output_names_[static_cast<size_t>(output_index)];
        if (std::find(resolved_output_names.begin(), resolved_output_names.end(), resolved_name) ==
            resolved_output_names.end())
        {
            resolved_output_names.push_back(resolved_name);
        }
    }
    if (resolved_output_names.empty())
    {
        return {};
    }

    std::vector<const char *> output_name_ptrs;
    output_name_ptrs.reserve(resolved_output_names.size());
    for (const auto &name : resolved_output_names)
    {
        output_name_ptrs.push_back(name.c_str());
    }

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_name_ptrs.data(),
        input_tensors.data(),
        input_tensors.size(),
        output_name_ptrs.data(),
        output_name_ptrs.size());

    PolicyInferenceResult result;
    int action_output_selected_index = -1;
    const std::string action_output_name = output_names_[static_cast<size_t>(action_output_index_)];
    for (size_t i = 0; i < resolved_output_names.size(); ++i)
    {
        if (resolved_output_names[i] == action_output_name)
        {
            action_output_selected_index = static_cast<int>(i);
            break;
        }
    }

    for (size_t output_idx = 0; output_idx < output_tensors.size(); ++output_idx)
    {
        if (static_cast<int>(output_idx) == action_output_selected_index)
        {
            const auto raw_action = flattenFloatTensor(output_tensors[output_idx]);
            if (cfg_.action_dim > 0)
            {
                size_t expected_dim = static_cast<size_t>(cfg_.action_dim);
                if (cfg_.action_output_layout == "chunk_flat")
                {
                    expected_dim *= static_cast<size_t>(std::max(1, cfg_.action_chunk_steps));
                }
                if (raw_action.size() < expected_dim)
                {
                    throw std::runtime_error(
                        "[" + policy_tag_ + "] action output dim " + std::to_string(raw_action.size()) +
                        " is smaller than configured action_dim " + std::to_string(expected_dim));
                }
                if (!warned_action_size_mismatch_ && raw_action.size() != expected_dim)
                {
                    std::cerr << "[" << policy_tag_ << "] action output dim mismatch. model="
                              << raw_action.size() << ", cfg=" << expected_dim
                              << ", using first " << expected_dim << " values." << std::endl;
                    warned_action_size_mismatch_ = true;
                }
                result.action.assign(
                    raw_action.begin(),
                    raw_action.begin() + static_cast<std::ptrdiff_t>(expected_dim));
            }
            else
            {
                result.action = raw_action;
            }
            if (!cfg_.action_output_indices.empty())
            {
                if (cfg_.action_output_indices.size() != result.action.size())
                {
                    throw std::runtime_error(
                        "[" + policy_tag_ + "] action_output_indices length " +
                        std::to_string(cfg_.action_output_indices.size()) +
                        " does not match action output dim " +
                        std::to_string(result.action.size()));
                }
                std::vector<float> reordered(result.action.size(), 0.0f);
                for (size_t out_i = 0; out_i < cfg_.action_output_indices.size(); ++out_i)
                {
                    const int in_i = cfg_.action_output_indices[out_i];
                    if (in_i < 0 || static_cast<size_t>(in_i) >= result.action.size())
                    {
                        throw std::runtime_error(
                            "[" + policy_tag_ + "] action_output_indices contains out-of-range index " +
                            std::to_string(in_i));
                    }
                    reordered[out_i] = result.action[static_cast<size_t>(in_i)];
                }
                result.action = std::move(reordered);
            }
            continue;
        }
        result.extra_outputs[resolved_output_names[output_idx]] = flattenFloatTensor(output_tensors[output_idx]);
    }

    const bool has_timestep_input = std::any_of(
        input_bindings_.begin(),
        input_bindings_.end(),
        [](const InputBinding &binding) {
            return trimCopy(binding.source) == "time_step";
        });
    if (advance_time_step && has_timestep_input)
    {
        ++time_step_;
    }
    return result;
}

std::vector<float> OnnxPolicyRunner::resolveInputData(
    const InputBinding &binding,
    const std::vector<float> &stacked_observation,
    const std::vector<float> &current_observation,
    const std::vector<float> &last_action,
    const std::unordered_map<std::string, std::vector<float>> &features) const
{
    const size_t target_count = elementCountFromShape(binding.shape);
    std::vector<float> source_data;
    const std::string source = trimCopy(binding.source);

    if (source == "stacked_observation")
    {
        source_data = stacked_observation;
    }
    else if (source == "observation")
    {
        source_data = current_observation;
    }
    else if (source == "last_action")
    {
        source_data = last_action;
    }
    else if (source == "time_step")
    {
        source_data = {static_cast<float>(time_step_)};
    }
    else if (source == "feature")
    {
        const auto it = features.find(binding.feature_name);
        if (it != features.end())
        {
            source_data = it->second;
        }
    }
    else if (source == "feature_concat")
    {
        for (const auto &feature_name : binding.feature_names)
        {
            if (feature_name == "__last_action__")
            {
                source_data.insert(source_data.end(), last_action.begin(), last_action.end());
                continue;
            }
            const auto it = features.find(feature_name);
            if (it == features.end())
            {
                throw std::runtime_error(
                    "[" + policy_tag_ + "] input '" + binding.name +
                    "' requires feature '" + feature_name + "' but it is missing");
            }
            source_data.insert(source_data.end(), it->second.begin(), it->second.end());
        }
    }
    else if (source == "constant")
    {
        source_data = binding.constant;
    }
    else if (source == "random_normal")
    {
        thread_local std::mt19937 rng(std::random_device{}());
        thread_local std::normal_distribution<float> normal(0.0f, 1.0f);
        source_data.resize(target_count);
        for (float &value : source_data)
        {
            value = normal(rng);
        }
    }
    else
    {
        throw std::runtime_error("[" + policy_tag_ + "] unsupported input source '" + binding.source + "'");
    }

    if (source_data.size() > target_count)
    {
        throw std::runtime_error(
            "[" + policy_tag_ + "] input '" + binding.name + "' source '" + binding.source +
            "' provides " + std::to_string(source_data.size()) +
            " values, exceeds target tensor size " + std::to_string(target_count));
    }
    if (source_data.size() < target_count)
    {
        throw std::runtime_error(
            "[" + policy_tag_ + "] input '" + binding.name + "' source '" + binding.source +
            "' provides " + std::to_string(source_data.size()) +
            " values, smaller than target tensor size " + std::to_string(target_count));
    }

    std::vector<float> out(target_count, 0.0f);
    const size_t copy_n = std::min(source_data.size(), target_count);
    std::copy(
        source_data.begin(),
        source_data.begin() + static_cast<std::ptrdiff_t>(copy_n),
        out.begin());
    return out;
}

void OnnxPolicyRunner::validateModelMetadata()
{
    if (!session_)
    {
        throw std::runtime_error("[" + policy_tag_ + "] metadata check called before init()");
    }

    custom_metadata_.clear();
    const bool strict = cfg_.metadata_check_strict;
    auto emitIssue = [&](const std::string &detail) {
        const std::string message = "[" + policy_tag_ + "] ONNX metadata check: " + detail;
        if (strict)
        {
            throw std::runtime_error(message);
        }
        std::cerr << message << std::endl;
    };

    std::unordered_map<std::string, std::string> custom_metadata;
    std::string producer;
    std::string graph_name;
    std::string domain;
    std::string description;
    int64_t version = 0;

    try
    {
        Ort::AllocatorWithDefaultOptions allocator;
        const auto metadata = session_->GetModelMetadata();
        version = metadata.GetVersion();
        {
            auto value = metadata.GetProducerNameAllocated(allocator);
            if (value)
            {
                producer = value.get();
            }
        }
        {
            auto value = metadata.GetGraphNameAllocated(allocator);
            if (value)
            {
                graph_name = value.get();
            }
        }
        {
            auto value = metadata.GetDomainAllocated(allocator);
            if (value)
            {
                domain = value.get();
            }
        }
        {
            auto value = metadata.GetDescriptionAllocated(allocator);
            if (value)
            {
                description = value.get();
            }
        }
        auto keys = metadata.GetCustomMetadataMapKeysAllocated(allocator);
        for (const auto &key_ptr : keys)
        {
            if (!key_ptr)
            {
                continue;
            }
            const std::string key = key_ptr.get();
            std::string value;
            auto value_ptr = metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
            if (value_ptr)
            {
                value = value_ptr.get();
            }
            custom_metadata[key] = value;
        }
    }
    catch (const std::exception &e)
    {
        emitIssue(std::string("failed to read metadata: ") + e.what());
        return;
    }

    custom_metadata_ = custom_metadata;

    if (!cfg_.enable_metadata_check)
    {
        return;
    }

    for (const auto &key : cfg_.required_metadata_keys)
    {
        if (key.empty())
        {
            continue;
        }
        if (custom_metadata.find(key) == custom_metadata.end())
        {
            emitIssue("missing required custom metadata key '" + key + "'");
        }
    }

    for (const auto &entry : cfg_.expected_metadata)
    {
        const auto it = custom_metadata.find(entry.first);
        if (it == custom_metadata.end())
        {
            emitIssue("missing expected custom metadata key '" + entry.first + "'");
            continue;
        }
        const std::string actual = trimCopy(it->second);
        const std::string expected = trimCopy(entry.second);
        if (actual != expected)
        {
            emitIssue("custom metadata mismatch for key '" + entry.first +
                      "': expected '" + expected + "', got '" + actual + "'");
        }
    }

    auto validateIntFieldIfPresent = [&](const std::string &key, int64_t expected) {
        const auto it = custom_metadata.find(key);
        if (it == custom_metadata.end())
        {
            return;
        }
        int64_t parsed = 0;
        if (!parseInt64Value(it->second, &parsed))
        {
            emitIssue("custom metadata key '" + key + "' value '" + it->second + "' is not an integer");
            return;
        }
        if (parsed != expected)
        {
            emitIssue("custom metadata key '" + key + "' mismatch: expected " +
                      std::to_string(expected) + ", got " + std::to_string(parsed));
        }
    };

    auto validateStringFieldIfPresent = [&](const std::string &key, const std::string &expected) {
        const auto it = custom_metadata.find(key);
        if (it == custom_metadata.end())
        {
            return;
        }
        if (trimCopy(it->second) != trimCopy(expected))
        {
            emitIssue("custom metadata key '" + key + "' mismatch: expected '" +
                      expected + "', got '" + it->second + "'");
        }
    };

    auto validateCsvListIfPresent = [&](const std::string &key, const std::vector<std::string> &expected) {
        if (expected.empty())
        {
            return;
        }
        const auto it = custom_metadata.find(key);
        if (it == custom_metadata.end())
        {
            return;
        }
        const std::vector<std::string> actual = parseCsvList(it->second);
        if (actual != expected)
        {
            emitIssue("custom metadata key '" + key + "' mismatch: expected [" +
                      joinStrings(expected) + "], got [" + joinStrings(actual) + "]");
        }
    };

    if (cfg_.obs_dim > 0)
    {
        validateIntFieldIfPresent("obs_dim", static_cast<int64_t>(cfg_.obs_dim));
    }
    if (cfg_.action_dim > 0)
    {
        validateIntFieldIfPresent("action_dim", static_cast<int64_t>(cfg_.action_dim));
    }
    validateStringFieldIfPresent("action_output_layout", cfg_.action_output_layout);
    if (cfg_.action_output_layout == "chunk_flat" && cfg_.action_chunk_steps > 0)
    {
        validateIntFieldIfPresent("action_chunk_steps", static_cast<int64_t>(cfg_.action_chunk_steps));
    }
    if (cfg_.obs_stack_N > 0)
    {
        validateIntFieldIfPresent("obs_stack_n", static_cast<int64_t>(cfg_.obs_stack_N));
        validateIntFieldIfPresent("obs_stack_N", static_cast<int64_t>(cfg_.obs_stack_N));
    }
    validateStringFieldIfPresent("obs_input_name", cfg_.obs_input_name);
    validateStringFieldIfPresent("action_output_name", cfg_.action_output_name);
    validateCsvListIfPresent("action_joint_names", cfg_.action_joint_order);
    validateCsvListIfPresent("policy_joint_names", cfg_.obs_joint_order);
    validateCsvListIfPresent("joint_names", cfg_.action_joint_order);
    if (cfg_.enable_reference_motion &&
        trimCopy(cfg_.reference_motion_source) != "file")
    {
        std::string reference_order_metadata_key =
            trimCopy(cfg_.source_contract.policy_extra_outputs.reference_joint_order_metadata_key);
        if (reference_order_metadata_key.empty())
        {
            reference_order_metadata_key = "command_joint_names";
        }
        validateCsvListIfPresent(reference_order_metadata_key, cfg_.reference_joint_order);
    }
    validateCsvListIfPresent("body_names", cfg_.reference_body_names);
    validateStringFieldIfPresent("anchor_body_name", cfg_.reference_anchor_body);

    std::cout << "[" << policy_tag_ << "] ONNX metadata check passed."
              << " custom_keys=" << custom_metadata.size()
              << ", strict=" << (strict ? "true" : "false")
              << ", version=" << version
              << ", producer='" << producer
              << "', graph='" << graph_name
              << "', domain='" << domain
              << "', description='" << description << "'" << std::endl;
}

std::string OnnxPolicyRunner::summary() const
{
    std::ostringstream oss;
    oss << "[" << policy_tag_ << "] ONNX policy loaded: " << model_path_;
    oss << "\n  inputs: ";
    for (size_t i = 0; i < input_names_.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << input_names_[i];
    }
    oss << "\n  outputs: ";
    for (size_t i = 0; i < output_names_.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << output_names_[i];
    }
    oss << "\n  input_bindings: ";
    for (size_t i = 0; i < input_bindings_.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        const auto &binding = input_bindings_[i];
        oss << binding.name << "<-" << binding.source;
        if (!binding.feature_name.empty())
        {
            oss << "(" << binding.feature_name << ")";
        }
    }
    oss << ", action_output=" << output_names_[static_cast<size_t>(action_output_index_)];
    return oss.str();
}

int OnnxPolicyRunner::findInputIndexByName(const std::string &name) const
{
    for (size_t i = 0; i < input_names_.size(); ++i)
    {
        if (input_names_[i] == name)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int OnnxPolicyRunner::findOutputIndexByName(const std::string &name) const
{
    for (size_t i = 0; i < output_names_.size(); ++i)
    {
        if (output_names_[i] == name)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

std::vector<float> OnnxPolicyRunner::flattenFloatTensor(const Ort::Value &tensor) const
{
    if (!tensor.IsTensor())
    {
        throw std::runtime_error("[" + policy_tag_ + "] output is not a tensor");
    }
    auto type_info = tensor.GetTensorTypeAndShapeInfo();
    const auto data_type = type_info.GetElementType();
    if (data_type != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
        throw std::runtime_error("[" + policy_tag_ + "] only float tensor outputs are supported");
    }
    const size_t count = type_info.GetElementCount();
    const float *ptr = tensor.GetTensorData<float>();
    return std::vector<float>(ptr, ptr + static_cast<std::ptrdiff_t>(count));
}

std::vector<int64_t> OnnxPolicyRunner::normalizedShape(const std::vector<int64_t> &shape) const
{
    std::vector<int64_t> normalized = shape;
    for (auto &dim : normalized)
    {
        if (dim <= 0)
        {
            dim = 1;
        }
    }
    return normalized;
}

size_t OnnxPolicyRunner::elementCountFromShape(const std::vector<int64_t> &shape)
{
    if (shape.empty())
    {
        return 1;
    }
    size_t count = 1;
    for (const auto dim : shape)
    {
        count *= static_cast<size_t>(std::max<int64_t>(dim, 1));
    }
    return count;
}
