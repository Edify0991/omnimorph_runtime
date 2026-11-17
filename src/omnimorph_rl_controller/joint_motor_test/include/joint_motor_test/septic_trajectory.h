#ifndef JOINT_MOTOR_TEST_SEPTIC_TRAJECTORY_H
#define JOINT_MOTOR_TEST_SEPTIC_TRAJECTORY_H

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace joint_motor_test
{

struct SepticLimits
{
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
};

struct SepticSample
{
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double jerk = 0.0;
    double progress = 0.0;
};

inline double septicDuration(double delta_q, const SepticLimits &limits)
{
    const double distance = std::abs(delta_q);
    if (distance <= 1e-12)
    {
        return 0.0;
    }
    if (!std::isfinite(limits.max_velocity) || limits.max_velocity <= 0.0 ||
        !std::isfinite(limits.max_acceleration) || limits.max_acceleration <= 0.0 ||
        !std::isfinite(limits.max_jerk) || limits.max_jerk <= 0.0)
    {
        throw std::invalid_argument("septic trajectory limits must be finite and positive");
    }

    const double velocity_time = 2.1875 * distance / limits.max_velocity;
    const double acceleration_time = std::sqrt(7.513188404399293 * distance / limits.max_acceleration);
    const double jerk_time = std::cbrt(52.5 * distance / limits.max_jerk);
    return std::max({velocity_time, acceleration_time, jerk_time});
}

inline SepticSample sampleSeptic(double q_start, double q_end, double elapsed, double duration)
{
    if (!std::isfinite(q_start) || !std::isfinite(q_end) || !std::isfinite(elapsed) ||
        !std::isfinite(duration) || duration < 0.0)
    {
        throw std::invalid_argument("invalid septic trajectory sample input");
    }
    if (duration <= 1e-12)
    {
        return {q_end, 0.0, 0.0, 0.0, 1.0};
    }

    const double u = std::clamp(elapsed / duration, 0.0, 1.0);
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double u6 = u5 * u;
    const double u7 = u6 * u;
    const double s = 35.0 * u4 - 84.0 * u5 + 70.0 * u6 - 20.0 * u7;
    const double ds = 140.0 * u3 - 420.0 * u4 + 420.0 * u5 - 140.0 * u6;
    const double dds = 420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u5;
    const double ddds = 840.0 * u - 5040.0 * u2 + 8400.0 * u3 - 4200.0 * u4;
    const double delta = q_end - q_start;

    SepticSample sample;
    sample.position = q_start + delta * s;
    sample.velocity = delta * ds / duration;
    sample.acceleration = delta * dds / (duration * duration);
    sample.jerk = delta * ddds / (duration * duration * duration);
    sample.progress = u;
    return sample;
}

} // namespace joint_motor_test

#endif // JOINT_MOTOR_TEST_SEPTIC_TRAJECTORY_H
