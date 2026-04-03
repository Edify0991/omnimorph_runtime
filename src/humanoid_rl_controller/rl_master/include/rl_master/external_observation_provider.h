#ifndef RL_MASTER_EXTERNAL_OBSERVATION_PROVIDER_H
#define RL_MASTER_EXTERNAL_OBSERVATION_PROVIDER_H

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rl_cfg.h"

class ExternalObservationProvider
{
public:
    ExternalObservationProvider() = default;

    void setFeature(const std::string &name, const std::vector<float> &values);
    std::unordered_map<std::string, std::vector<float>> collect(
        const std::vector<ExternalObservationSpec> &specs) const;

private:
    static std::vector<float> fitDim(const std::vector<float> &values, int dim);

    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::vector<float>> feature_cache_;
};

#endif // RL_MASTER_EXTERNAL_OBSERVATION_PROVIDER_H
