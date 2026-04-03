#ifndef ONNX_POLICY_RUNNER_H
#define ONNX_POLICY_RUNNER_H

#include <onnxruntime_cxx_api.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rl_cfg.h"

struct PolicyInferenceResult
{
    std::vector<float> action;
    std::unordered_map<std::string, std::vector<float>> extra_outputs;
};

class OnnxPolicyRunner
{
public:
    OnnxPolicyRunner(
        Ort::Env &env,
        const std::string &model_path,
        const Sim2realCfg &cfg,
        std::string policy_tag);

    void init();
    void reset();

    PolicyInferenceResult forward(const std::vector<float> &observation);

    const std::vector<std::string> &input_names() const { return input_names_; }
    const std::vector<std::string> &output_names() const { return output_names_; }
    std::string summary() const;

private:
    struct AuxInputBuffer
    {
        std::vector<int64_t> shape;
        std::vector<float> data;
    };

    int findInputIndexByName(const std::string &name) const;
    int findOutputIndexByName(const std::string &name) const;
    std::vector<float> flattenFloatTensor(const Ort::Value &tensor) const;
    std::vector<int64_t> normalizedShape(const std::vector<int64_t> &shape) const;
    static size_t elementCountFromShape(const std::vector<int64_t> &shape);

    Ort::Env &env_;
    std::string model_path_;
    Sim2realCfg cfg_;
    std::string policy_tag_;

    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;

    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;

    int obs_input_index_ = -1;
    int timestep_input_index_ = -1;
    int action_output_index_ = -1;
    int action_output_selected_index_ = -1;
    std::vector<int> extra_output_indices_;
    std::vector<int> unknown_input_indices_;
    std::vector<AuxInputBuffer> aux_input_buffers_;
    std::unordered_map<int, size_t> unknown_input_buffer_map_;
    std::vector<int> selected_output_indices_;
    std::vector<std::string> selected_output_names_;
    bool warned_action_size_mismatch_ = false;
    int64_t time_step_ = 0;
};

#endif // ONNX_POLICY_RUNNER_H
