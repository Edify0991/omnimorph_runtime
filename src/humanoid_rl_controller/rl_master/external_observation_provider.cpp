#include "rl_master/external_observation_provider.h"

#include <algorithm>

void ExternalObservationProvider::setFeature(const std::string &name, const std::vector<float> &values)
{
    std::lock_guard<std::mutex> lock(mutex_);
    feature_cache_[name] = values;
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
        out[spec.name] = fitDim(it->second, dim);
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
