#include "rl_master/reference_motion_provider.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

bool ReferenceMotionProvider::load(const std::string &file_path, int expected_dim)
{
    clear();

    if (expected_dim <= 0)
    {
        return false;
    }

    std::ifstream fin(file_path);
    if (!fin.is_open())
    {
        return false;
    }

    std::string line;
    while (std::getline(fin, line))
    {
        if (line.empty())
        {
            continue;
        }
        if (line[0] == '#')
        {
            continue;
        }

        std::vector<float> frame = parseLine(line);
        if (frame.empty())
        {
            continue;
        }
        frames_.push_back(fitDim(frame, static_cast<size_t>(expected_dim)));
    }

    if (frames_.empty())
    {
        return false;
    }

    loaded_ = true;
    dim_ = expected_dim;
    return true;
}

void ReferenceMotionProvider::clear()
{
    loaded_ = false;
    dim_ = 0;
    frames_.clear();
}

std::vector<float> ReferenceMotionProvider::sampleByPhase(double phase_t, double cycle_time, int expected_dim) const
{
    if (!available())
    {
        return std::vector<float>(std::max(0, expected_dim), 0.0f);
    }

    const size_t n = frames_.size();
    if (n == 0)
    {
        return std::vector<float>(std::max(0, expected_dim), 0.0f);
    }

    const double safe_cycle = std::max(1e-6, cycle_time);
    const double normalized = std::fmod(std::max(0.0, phase_t), safe_cycle) / safe_cycle;
    size_t index = static_cast<size_t>(normalized * static_cast<double>(n));
    if (index >= n)
    {
        index = n - 1;
    }
    return fitDim(frames_[index], static_cast<size_t>(std::max(expected_dim, 0)));
}

std::vector<float> ReferenceMotionProvider::sampleByStep(size_t step_index, int expected_dim) const
{
    if (!available())
    {
        return std::vector<float>(std::max(0, expected_dim), 0.0f);
    }

    if (frames_.empty())
    {
        return std::vector<float>(std::max(0, expected_dim), 0.0f);
    }

    const size_t index = step_index % frames_.size();
    return fitDim(frames_[index], static_cast<size_t>(std::max(expected_dim, 0)));
}

std::vector<float> ReferenceMotionProvider::parseLine(const std::string &line)
{
    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), '\t', ' ');

    std::istringstream iss(normalized);
    std::vector<float> values;
    float v = 0.0f;
    while (iss >> v)
    {
        values.push_back(v);
    }
    return values;
}

std::vector<float> ReferenceMotionProvider::fitDim(const std::vector<float> &values, size_t dim)
{
    std::vector<float> out(dim, 0.0f);
    const size_t copy_n = std::min(values.size(), dim);
    std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(copy_n), out.begin());
    return out;
}
