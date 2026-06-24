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
    float sim_time,
    const ComSupportOverlay &overlay)
{
    if (!enable_python_viewer_stream_ || !viewer_frame_pub_)
    {
        return;
    }

    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(
        22 + static_cast<size_t>(model_->nq) + static_cast<size_t>(model_->nv) +
        static_cast<size_t>(model_->nu) + overlay.support_polygon.size() * 3);

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

    msg.data.push_back(overlay.valid ? 1.0f : 0.0f);
    msg.data.push_back(static_cast<float>(overlay.com[0]));
    msg.data.push_back(static_cast<float>(overlay.com[1]));
    msg.data.push_back(static_cast<float>(overlay.com[2]));
    msg.data.push_back(overlay.dcm_valid ? 1.0f : 0.0f);
    msg.data.push_back(overlay.dcm_inside_capture_region ? 1.0f : 0.0f);
    msg.data.push_back(static_cast<float>(overlay.dcm[0]));
    msg.data.push_back(static_cast<float>(overlay.dcm[1]));
    msg.data.push_back(static_cast<float>(overlay.dcm[2]));
    msg.data.push_back(overlay.cop_valid ? 1.0f : 0.0f);
    msg.data.push_back(static_cast<float>(overlay.cop[0]));
    msg.data.push_back(static_cast<float>(overlay.cop[1]));
    msg.data.push_back(static_cast<float>(overlay.cop[2]));
    msg.data.push_back(static_cast<float>(overlay.support_polygon.size()));
    for (const auto &point : overlay.support_polygon)
    {
        msg.data.push_back(static_cast<float>(point[0]));
        msg.data.push_back(static_cast<float>(point[1]));
        msg.data.push_back(static_cast<float>(point[2]));
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

} // namespace mujoco_sim2sim
