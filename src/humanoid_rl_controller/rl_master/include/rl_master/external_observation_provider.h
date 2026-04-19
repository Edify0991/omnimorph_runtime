#ifndef RL_MASTER_EXTERNAL_OBSERVATION_PROVIDER_H
#define RL_MASTER_EXTERNAL_OBSERVATION_PROVIDER_H

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rl_cfg.h"

class ExternalObservationProvider
{
public:
    struct Sample
    {
        std::string name;
        std::vector<float> values;
        double monotonic_time_sec = 0.0;
        uint64_t update_seq = 0;
        bool present = false;
    };

    ExternalObservationProvider() = default;

    void setFeature(const std::string &name, const std::vector<float> &values);
    void setFeature(const std::string &name, const std::vector<float> &values, double monotonic_time_sec);
    std::unordered_map<std::string, std::vector<float>> collect(
        const std::vector<ExternalObservationSpec> &specs) const;
    std::vector<Sample> drainUpdatedSamples(const std::vector<ExternalObservationSpec> &specs);

private:
    struct CachedFeature
    {
        std::vector<float> values;
        double monotonic_time_sec = 0.0;
        uint64_t update_seq = 0;
    };

    static std::vector<float> fitDim(const std::vector<float> &values, int dim);

    mutable std::mutex mutex_;
    std::unordered_map<std::string, CachedFeature> feature_cache_;
    std::unordered_map<std::string, uint64_t> drained_update_seq_;
    uint64_t next_update_seq_ = 1;
};

#endif // RL_MASTER_EXTERNAL_OBSERVATION_PROVIDER_H
