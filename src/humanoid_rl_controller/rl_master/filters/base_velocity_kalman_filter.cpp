#include "rl_master/filters/base_velocity_kalman_filter.h"

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace rl_master::filters
{

void BaseVelocityKalmanFilter::configure(const Options &options)
{
    options_ = options;
    options_.initial_variance = std::max(options_.initial_variance, 1.0e-8f);
    options_.process_noise = std::max(options_.process_noise, 0.0f);
    options_.accel_noise = std::max(options_.accel_noise, 0.0f);
    options_.min_dt = std::max(options_.min_dt, 0.0f);
    options_.max_dt = std::max(options_.max_dt, options_.min_dt);
    if (!initialized_)
    {
        covariance_ = {
            options_.initial_variance,
            options_.initial_variance,
            options_.initial_variance};
    }
}

void BaseVelocityKalmanFilter::reset()
{
    velocity_ = {0.0f, 0.0f, 0.0f};
    covariance_ = {
        options_.initial_variance,
        options_.initial_variance,
        options_.initial_variance};
    initialized_ = false;
}

void BaseVelocityKalmanFilter::reset(const std::array<float, 3> &initial_velocity)
{
    velocity_ = initial_velocity;
    covariance_ = {
        options_.initial_variance,
        options_.initial_variance,
        options_.initial_variance};
    initialized_ = true;
}

bool BaseVelocityKalmanFilter::initialized() const
{
    return initialized_;
}

const std::array<float, 3> &BaseVelocityKalmanFilter::velocity() const
{
    return velocity_;
}

void BaseVelocityKalmanFilter::predict(const std::array<float, 3> &linear_accel_w, float dt)
{
    if (!initialized_)
    {
        reset();
    }
    if (!std::isfinite(dt) || dt <= options_.min_dt)
    {
        return;
    }
    dt = std::min(dt, options_.max_dt);
    const float process_variance = options_.process_noise * dt + options_.accel_noise * dt * dt;
    for (size_t i = 0; i < 3; ++i)
    {
        velocity_[i] += linear_accel_w[i] * dt;
        covariance_[i] = std::max(covariance_[i] + process_variance, 1.0e-8f);
    }
}

void BaseVelocityKalmanFilter::updateVelocity(
    const std::array<float, 3> &velocity_measurement_w,
    float measurement_noise)
{
    if (!initialized_)
    {
        reset(velocity_measurement_w);
        return;
    }

    const float r = std::max(measurement_noise, 1.0e-8f);
    for (size_t i = 0; i < 3; ++i)
    {
        const float denom = covariance_[i] + r;
        const float gain = denom > 1.0e-12f ? covariance_[i] / denom : 0.0f;
        velocity_[i] += gain * (velocity_measurement_w[i] - velocity_[i]);
        covariance_[i] = std::max((1.0f - gain) * covariance_[i], 1.0e-8f);
    }
}

} // namespace rl_master::filters
