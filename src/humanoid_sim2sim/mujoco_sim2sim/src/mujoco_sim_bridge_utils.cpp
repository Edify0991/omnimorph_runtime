#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{

std::array<float, 3> MujocoSimBridge::quatXyzwToRpy(const std::array<float, 4> &quat_xyzw)
{
    const double x = static_cast<double>(quat_xyzw[0]);
    const double y = static_cast<double>(quat_xyzw[1]);
    const double z = static_cast<double>(quat_xyzw[2]);
    const double w = static_cast<double>(quat_xyzw[3]);

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    constexpr double kPi = 3.14159265358979323846;
    const double pitch = std::abs(sinp) >= 1.0
                             ? std::copysign(kPi / 2.0, sinp)
                             : std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {
        static_cast<float>(roll),
        static_cast<float>(pitch),
        static_cast<float>(yaw)};
}

std::vector<std::string> MujocoSimBridge::normalizeNameParam(
    const std::vector<std::string> &input,
    const std::vector<std::string> &fallback)
{
    if (input.empty())
    {
        return fallback;
    }
    if (!fallback.empty() && input.size() != fallback.size())
    {
        throw std::runtime_error("Name vector size mismatch. Expect " + std::to_string(fallback.size()));
    }
    return input;
}

} // namespace mujoco_sim2sim
