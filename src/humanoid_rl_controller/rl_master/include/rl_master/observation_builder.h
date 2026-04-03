#ifndef OBSERVATION_BUILDER_H
#define OBSERVATION_BUILDER_H

#include <cstddef>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include "cmd.h"
#include "rl_cfg.h"
#include "robot_state.h"

struct ObservationTermConfig
{
    std::string name;
    bool enabled = true;
    int count = 0;
    std::vector<std::string> components;
    std::string source;
};

struct ObservationFeatureContext
{
    std::unordered_map<std::string, std::vector<float>> named_features;
};

class ObservationManifest
{
public:
    static ObservationManifest loadFromYAML(const std::string &yaml_file);
    const std::vector<ObservationTermConfig> &terms() const;

private:
    std::vector<ObservationTermConfig> terms_;
};

class ObservationBuilder
{
public:
    explicit ObservationBuilder(ObservationManifest manifest);

    std::vector<float> build(
        const RobotState &robot,
        const Cmd &cmd,
        const std::vector<float> &last_action,
        double phase_t,
        const Sim2realCfg &cfg,
        const std::vector<int> &obs_index_map,
        const ObservationFeatureContext &feature_context = {}) const;

    size_t expectedDim() const;
    const std::vector<std::string> &layoutDescription() const;

private:
    using GatherFn = std::function<void(
        const ObservationTermConfig &,
        const RobotState &,
        const Cmd &,
        const std::vector<float> &,
        double,
        const Sim2realCfg &,
        const std::vector<int> &,
        const ObservationFeatureContext &,
        std::vector<float> *)>;

    struct ObservationProvider
    {
        GatherFn gather;
        int default_dim = 0;
        bool supports_count = false;
        bool supports_components = false;
    };

    struct ResolvedObservationTerm
    {
        ObservationTermConfig config;
        GatherFn gather;
        size_t offset = 0;
        size_t dim = 0;
        std::string description;
    };

    static const std::unordered_map<std::string, ObservationProvider> &registry();
    static size_t resolveTermDim(const ObservationTermConfig &term, const ObservationProvider &provider);

    ObservationManifest manifest_;
    std::vector<ResolvedObservationTerm> resolved_terms_;
    std::vector<std::string> layout_description_;
    size_t expected_dim_ = 0;
};

#endif // OBSERVATION_BUILDER_H
