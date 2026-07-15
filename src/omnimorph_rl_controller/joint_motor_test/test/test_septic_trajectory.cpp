#include "joint_motor_test/septic_trajectory.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
bool near(double lhs, double rhs, double tolerance)
{
    return std::abs(lhs - rhs) <= tolerance;
}
}

int main()
{
    using joint_motor_test::SepticLimits;
    using joint_motor_test::sampleSeptic;
    using joint_motor_test::septicDuration;

    const double deg = 3.14159265358979323846 / 180.0;
    struct TestCase
    {
        double delta;
        SepticLimits limits;
    };
    const TestCase cases[] = {
        {105.0 * deg, {120.0 * deg, 300.0 * deg, 2000.0 * deg}},
        {95.0 * deg, {110.0 * deg, 300.0 * deg, 2000.0 * deg}},
        {25.0 * deg, {120.0 * deg, 900.0 * deg, 16000.0 * deg}},
        {20.0 * deg, {120.0 * deg, 900.0 * deg, 16000.0 * deg}},
    };

    for (const auto &test : cases)
    {
        const double duration = septicDuration(test.delta, test.limits);
        const auto start = sampleSeptic(0.0, test.delta, 0.0, duration);
        const auto finish = sampleSeptic(0.0, test.delta, duration, duration);
        if (!near(start.position, 0.0, 1e-12) || !near(finish.position, test.delta, 1e-10) ||
            !near(start.velocity, 0.0, 1e-12) || !near(finish.velocity, 0.0, 1e-10) ||
            !near(start.acceleration, 0.0, 1e-12) || !near(finish.acceleration, 0.0, 1e-10) ||
            !near(start.jerk, 0.0, 1e-12) || !near(finish.jerk, 0.0, 1e-9))
        {
            std::cerr << "septic endpoint continuity check failed\n";
            return 1;
        }

        double max_v = 0.0;
        double max_a = 0.0;
        double max_j = 0.0;
        for (int i = 0; i <= 20000; ++i)
        {
            const auto sample = sampleSeptic(0.0, test.delta, duration * i / 20000.0, duration);
            max_v = std::max(max_v, std::abs(sample.velocity));
            max_a = std::max(max_a, std::abs(sample.acceleration));
            max_j = std::max(max_j, std::abs(sample.jerk));
        }
        if (max_v > test.limits.max_velocity * (1.0 + 1e-8) ||
            max_a > test.limits.max_acceleration * (1.0 + 1e-6) ||
            max_j > test.limits.max_jerk * (1.0 + 1e-6))
        {
            std::cerr << "septic dynamic limit check failed\n";
            return 2;
        }
        const auto midpoint = sampleSeptic(0.0, test.delta, duration * 0.5, duration);
        const double expected_peak_v = 2.1875 * test.delta / duration;
        if (!near(midpoint.velocity, expected_peak_v, 1e-10))
        {
            std::cerr << "septic midpoint peak velocity check failed\n";
            return 3;
        }
    }
    return 0;
}
