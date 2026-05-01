#include "mujoco_sim_bridge_internal.hpp"

namespace mujoco_sim2sim
{
using namespace bridge_internal;

void MujocoSimBridge::publishRobotState(const rl_master::RobotStateData &state)
{
    if (!state_pub_)
    {
        return;
    }
    state_pub_->publish(rl_master::dds::encodeRobotState(state));
}

void MujocoSimBridge::publishViewerFrame()
{
    updateViewerFrameMirror();
}

void MujocoSimBridge::publishViewerFrameMirror(
    const std::vector<float> &qpos,
    const std::vector<float> &qvel,
    const std::vector<float> &ctrl,
    float sim_time)
{
    if (!enable_python_viewer_stream_ || !viewer_frame_pub_)
    {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(
        8 + static_cast<size_t>(model_->nq) + static_cast<size_t>(model_->nv) + static_cast<size_t>(model_->nu));

    msg.data.push_back(kViewerFrameMagic);
    msg.data.push_back(kViewerFrameVersion);
    msg.data.push_back(static_cast<float>(model_->nq));
    msg.data.push_back(static_cast<float>(model_->nv));
    msg.data.push_back(static_cast<float>(model_->nu));
    msg.data.push_back(sim_time);
    msg.data.push_back(control_hz_ > 0.0 ? static_cast<float>(1.0 / control_hz_) : 0.0f);
    msg.data.push_back(shouldEnforceBaseLock() ? 1.0f : 0.0f);

    for (size_t i = 0; i < qpos.size(); ++i)
    {
        msg.data.push_back(qpos[i]);
    }
    for (size_t i = 0; i < qvel.size(); ++i)
    {
        msg.data.push_back(qvel[i]);
    }
    for (size_t i = 0; i < ctrl.size(); ++i)
    {
        msg.data.push_back(ctrl[i]);
    }

    viewer_frame_pub_->publish(std::move(msg));
}

void MujocoSimBridge::publishViewerInspector(
    const rl_master::RobotStateData &state,
    const rl_master::RobotCommandData &command,
    const rl_master::CommandRuntimeDecision &runtime_mode)
{
    updateViewerInspectorMirror(state, command, runtime_mode);
}

void MujocoSimBridge::publishViewerInspectorText(const std::string &text)
{
    if (!enable_python_viewer_inspector_ || !viewer_inspector_pub_)
    {
        return;
    }

    std_msgs::msg::String msg;
    msg.data = text;
    viewer_inspector_pub_->publish(std::move(msg));
}

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

std::vector<double> MujocoSimBridge::normalizeGainParam(
    const std::vector<double> &input,
    double fallback,
    size_t expected_count)
{
    std::vector<double> out(expected_count, fallback);
    if (input.empty())
    {
        return out;
    }
    if (input.size() == 1)
    {
        out.assign(expected_count, input.front());
        return out;
    }
    if (input.size() != expected_count)
    {
        throw std::runtime_error("Gain vector size mismatch. Expect 1 or " + std::to_string(expected_count));
    }
    out = input;
    return out;
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
