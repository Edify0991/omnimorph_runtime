#ifndef RL_MASTER_FILTERS_BASE_VELOCITY_KALMAN_FILTER_H
#define RL_MASTER_FILTERS_BASE_VELOCITY_KALMAN_FILTER_H

#include <array>

namespace rl_master::filters
{

class BaseVelocityKalmanFilter
{
public:
    struct Options
    {
        float initial_variance = 0.25f;
        float process_noise = 0.05f;
        float accel_noise = 0.2f;
        float min_dt = 1.0e-4f;
        float max_dt = 0.05f;
    };

    void configure(const Options &options);
    void reset();
    void reset(const std::array<float, 3> &initial_velocity);

    bool initialized() const;
    const std::array<float, 3> &velocity() const;

    void predict(const std::array<float, 3> &linear_accel_w, float dt);
    void updateVelocity(const std::array<float, 3> &velocity_measurement_w, float measurement_noise);

private:
    Options options_{};
    std::array<float, 3> velocity_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> covariance_{0.25f, 0.25f, 0.25f};
    bool initialized_ = false;
};

} // namespace rl_master::filters

#endif // RL_MASTER_FILTERS_BASE_VELOCITY_KALMAN_FILTER_H
