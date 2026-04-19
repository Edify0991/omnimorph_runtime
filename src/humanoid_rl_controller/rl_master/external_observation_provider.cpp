#include "rl_master/external_observation_provider.h"

#include <algorithm>

#include "rl_master/rl_protocol.h"

void ExternalObservationProvider::setFeature(const std::string &name, const std::vector<float> &values)
{
    setFeature(name, values, rl_master::monotonicTimeSec());
}

void ExternalObservationProvider::setFeature(
    const std::string &name,
    const std::vector<float> &values,
    double monotonic_time_sec)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CachedFeature feature;
    feature.values = values;
    feature.monotonic_time_sec = monotonic_time_sec;
    feature.update_seq = next_update_seq_++;
    feature_cache_[name] = std::move(feature);
}

std::unordered_map<std::string, std::vector<float>> ExternalObservationProvider::collect(
    const std::vector<ExternalObservationSpec> &specs) const
{
    std::unordered_map<std::string, std::vector<float>> out;
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &spec : specs)
    {
        const int dim = std::max(spec.dim, 0);
        const auto it = feature_cache_.find(spec.name);
        if (it == feature_cache_.end())
        {
            out[spec.name] = std::vector<float>(static_cast<size_t>(dim), 0.0f);
            continue;
        }
        out[spec.name] = fitDim(it->second.values, dim);
    }
    return out;
}

std::vector<ExternalObservationProvider::Sample> ExternalObservationProvider::drainUpdatedSamples(
    const std::vector<ExternalObservationSpec> &specs)
{
    std::vector<Sample> out;
    std::lock_guard<std::mutex> lock(mutex_);
    out.reserve(specs.size());
    for (const auto &spec : specs)
    {
        const auto it = feature_cache_.find(spec.name);
        if (it == feature_cache_.end())
        {
            continue;
        }

        const uint64_t last_drained = [&]() {
            const auto drain_it = drained_update_seq_.find(spec.name);
            return drain_it == drained_update_seq_.end() ? 0 : drain_it->second;
        }();
        if (it->second.update_seq <= last_drained)
        {
            continue;
        }

        Sample sample;
        sample.name = spec.name;
        sample.values = fitDim(it->second.values, std::max(spec.dim, 0));
        sample.monotonic_time_sec = it->second.monotonic_time_sec;
        sample.update_seq = it->second.update_seq;
        sample.present = true;
        out.push_back(std::move(sample));
        drained_update_seq_[spec.name] = it->second.update_seq;
    }
    return out;
}

std::vector<float> ExternalObservationProvider::fitDim(const std::vector<float> &values, int dim)
{
    const size_t target_dim = static_cast<size_t>(std::max(dim, 0));
    std::vector<float> out(target_dim, 0.0f);
    const size_t copy_n = std::min(values.size(), target_dim);
    std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(copy_n), out.begin());
    return out;
}
