#include "rl_master/onnx_policy_runner.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

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

    obs_input_index_ = findInputIndexByName(cfg_.obs_input_name);
    if (obs_input_index_ < 0 && !cfg_.strict_model_io)
    {
        obs_input_index_ = 0;
    }
    if (obs_input_index_ < 0)
    {
        throw std::runtime_error(
            "[" + policy_tag_ + "] observation input '" + cfg_.obs_input_name + "' not found");
    }

    timestep_input_index_ = -1;
    const int detected_timestep_index = findInputIndexByName(cfg_.time_step_input_name);
    if (cfg_.enable_time_step_input)
    {
        if (detected_timestep_index < 0)
        {
            throw std::runtime_error(
                "[" + policy_tag_ + "] time_step input '" + cfg_.time_step_input_name + "' not found");
        }
        timestep_input_index_ = detected_timestep_index;
    }
    else if (detected_timestep_index >= 0)
    {
        timestep_input_index_ = detected_timestep_index;
    }
    if (timestep_input_index_ == obs_input_index_)
    {
        timestep_input_index_ = -1;
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

    unknown_input_indices_.clear();
    aux_input_buffers_.clear();
    unknown_input_buffer_map_.clear();
    for (int input_index = 0; input_index < input_count; ++input_index)
    {
        if (input_index == obs_input_index_ || input_index == timestep_input_index_)
        {
            continue;
        }
        unknown_input_indices_.push_back(input_index);

        auto input_info = session_->GetInputTypeInfo(input_index).GetTensorTypeAndShapeInfo();
        auto normalized_shape = normalizedShape(input_info.GetShape());
        if (normalized_shape.empty())
        {
            normalized_shape = {1};
        }

        AuxInputBuffer buffer;
        buffer.shape = normalized_shape;
        buffer.data.assign(elementCountFromShape(normalized_shape), 0.0f);
        unknown_input_buffer_map_[input_index] = aux_input_buffers_.size();
        aux_input_buffers_.push_back(std::move(buffer));
    }

    if (!unknown_input_indices_.empty())
    {
        std::ostringstream oss;
        for (size_t i = 0; i < unknown_input_indices_.size(); ++i)
        {
            const int index = unknown_input_indices_[i];
            if (i > 0)
            {
                oss << ", ";
            }
            oss << input_names_[static_cast<size_t>(index)];
        }
        if (cfg_.strict_model_io)
        {
            throw std::runtime_error("[" + policy_tag_ + "] unknown model inputs: " + oss.str());
        }
        std::cerr << "[" << policy_tag_ << "] fill unspecified inputs with zeros: " << oss.str() << std::endl;
    }

    reset();
    std::cout << summary() << std::endl;
}

void OnnxPolicyRunner::reset()
{
    time_step_ = cfg_.time_step_start;
}

PolicyInferenceResult OnnxPolicyRunner::forward(const std::vector<float> &observation)
{
    if (!session_)
    {
        throw std::runtime_error("[" + policy_tag_ + "] forward called before init()");
    }
    if (observation.empty())
    {
        throw std::runtime_error("[" + policy_tag_ + "] observation is empty");
    }

    std::array<float, 1> timestep_value{static_cast<float>(time_step_)};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<const char *> input_name_ptrs;
    std::vector<Ort::Value> input_tensors;
    input_name_ptrs.reserve(input_names_.size());
    input_tensors.reserve(input_names_.size());

    for (size_t input_index = 0; input_index < input_names_.size(); ++input_index)
    {
        input_name_ptrs.push_back(input_names_[input_index].c_str());
        if (static_cast<int>(input_index) == obs_input_index_)
        {
            std::array<int64_t, 2> input_shape{1, static_cast<int64_t>(observation.size())};
            input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
                memory_info,
                const_cast<float *>(observation.data()),
                observation.size(),
                input_shape.data(),
                input_shape.size()));
            continue;
        }

        if (static_cast<int>(input_index) == timestep_input_index_)
        {
            std::array<int64_t, 2> input_shape{1, 1};
            input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
                memory_info,
                timestep_value.data(),
                timestep_value.size(),
                input_shape.data(),
                input_shape.size()));
            continue;
        }

        const auto buffer_it = unknown_input_buffer_map_.find(static_cast<int>(input_index));
        if (buffer_it == unknown_input_buffer_map_.end())
        {
            throw std::runtime_error("[" + policy_tag_ + "] unknown input buffer mapping missing");
        }
        auto &buffer = aux_input_buffers_[buffer_it->second];
        input_tensors.emplace_back(Ort::Value::CreateTensor<float>(
            memory_info,
            buffer.data.data(),
            buffer.data.size(),
            buffer.shape.data(),
            buffer.shape.size()));
    }

    std::vector<const char *> output_name_ptrs;
    output_name_ptrs.reserve(selected_output_names_.size());
    for (const auto &name : selected_output_names_)
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
    const auto raw_action = flattenFloatTensor(output_tensors[static_cast<size_t>(action_output_selected_index_)]);
    if (cfg_.action_dim > 0)
    {
        const size_t expected_dim = static_cast<size_t>(cfg_.action_dim);
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
        result.action.assign(raw_action.begin(), raw_action.begin() + static_cast<std::ptrdiff_t>(expected_dim));
    }
    else
    {
        result.action = raw_action;
    }

    for (size_t output_idx = 0; output_idx < output_tensors.size(); ++output_idx)
    {
        if (static_cast<int>(output_idx) == action_output_selected_index_)
        {
            continue;
        }
        result.extra_outputs[selected_output_names_[output_idx]] = flattenFloatTensor(output_tensors[output_idx]);
    }

    if (timestep_input_index_ >= 0)
    {
        ++time_step_;
    }
    return result;
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
    oss << "\n  obs_input=" << input_names_[static_cast<size_t>(obs_input_index_)];
    if (timestep_input_index_ >= 0)
    {
        oss << ", time_step_input=" << input_names_[static_cast<size_t>(timestep_input_index_)];
    }
    else
    {
        oss << ", time_step_input=disabled";
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
