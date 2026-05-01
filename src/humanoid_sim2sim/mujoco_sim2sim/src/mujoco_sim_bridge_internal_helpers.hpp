#pragma once

namespace mujoco_sim2sim::bridge_internal
{
inline constexpr const char *kDefaultViewerFrameTopic = "/humanoid/sim2sim/mujoco_viewer_frame";
inline constexpr float kViewerFrameMagic = 260413.0f;
inline constexpr float kViewerFrameVersion = 1.0f;

inline std::vector<std::string> defaultJointNames()
{
    return {
        "right_hip_roll",
        "right_hip_yaw",
        "right_hip_pitch",
        "right_knee_pitch",
        "right_ankle_pitch",
        "right_ankle_roll",
        "left_hip_roll",
        "left_hip_yaw",
        "left_hip_pitch",
        "left_knee_pitch",
        "left_ankle_pitch",
        "left_ankle_roll"};
}

inline bool endsWith(const std::string &value, const std::string &suffix)
{
    if (value.size() < suffix.size())
    {
        return false;
    }
    return value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

inline std::string normalizeNoCommandBehavior(const std::string &raw)
{
    std::string value = raw;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (value == "hold_position" || value == "position_hold" || value == "position" || value == "hold-pos")
    {
        return "hold_position";
    }
    if (value == "zero_torque" || value == "zero" || value == "torque_off" || value == "off")
    {
        return "zero_torque";
    }
    if (value == "hold_last" || value == "hold" || value == "last")
    {
        return "hold_last";
    }
    return "hold_position";
}

inline std::string trimCopy(const std::string &raw)
{
    size_t begin = 0;
    while (begin < raw.size() && std::isspace(static_cast<unsigned char>(raw[begin])) != 0)
    {
        ++begin;
    }

    size_t end = raw.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(raw[end - 1])) != 0)
    {
        --end;
    }
    return raw.substr(begin, end - begin);
}

inline std::string toLowerCopy(const std::string &raw)
{
    std::string out = raw;
    std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return out;
}

inline const char *baseLockReasonName(MujocoSimBridge::BaseLockReason reason)
{
    switch (reason)
    {
    case MujocoSimBridge::BaseLockReason::kStartupZeroing:
        return "startup_zeroing";
    case MujocoSimBridge::BaseLockReason::kExplicitZeroing:
        return "explicit_zeroing";
    case MujocoSimBridge::BaseLockReason::kIncompatibleSwitchZeroing:
        return "incompatible_switch_zeroing";
    case MujocoSimBridge::BaseLockReason::kPreRunHold:
        return "pre_run_hold";
    case MujocoSimBridge::BaseLockReason::kNone:
    default:
        return "none";
    }
}

inline std::array<double, 4> normalizeQuatWxyz(std::array<double, 4> q)
{
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!std::isfinite(norm) || norm < 1.0e-9)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }
    for (double &v : q)
    {
        v /= norm;
    }
    return q;
}

inline std::array<double, 4> multiplyQuatWxyz(
    const std::array<double, 4> &a,
    const std::array<double, 4> &b)
{
    return normalizeQuatWxyz({
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    });
}

inline std::array<double, 4> inverseQuatWxyz(const std::array<double, 4> &q)
{
    const double norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!std::isfinite(norm_sq) || norm_sq < 1.0e-12)
    {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq};
}

inline std::array<double, 4> yawQuatWxyz(const std::array<double, 4> &q)
{
    const auto nq = normalizeQuatWxyz(q);
    const double yaw = std::atan2(
        2.0 * (nq[0] * nq[3] + nq[1] * nq[2]),
        1.0 - 2.0 * (nq[2] * nq[2] + nq[3] * nq[3]));
    return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
}
} // namespace mujoco_sim2sim::bridge_internal
