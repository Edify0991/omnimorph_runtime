#include "rl_master/dds_protocol.h"

#include <algorithm>
#include <cstddef>

namespace rl_master::dds
{

std_msgs::msg::Float32MultiArray encodePolicyCommand(
    const RobotCommandData &command,
    uint32_t sequence,
    double stamp_sec)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(kJointCmdValueCount, 0.0f);
    for (size_t i = 0; i < kLegJointCount; ++i)
    {
        const size_t offset = 3 * i;
        msg.data[offset] = command.joint_target_q[i];
        msg.data[offset + 1] = command.joint_target_dq[i];
        msg.data[offset + 2] = command.joint_target_tau[i];
    }
    msg.data[kJointStateValueCount] = command.open_rl;
    msg.data[kJointCmdSeqIndex] = static_cast<float>(sequence);
    msg.data[kJointCmdStampIndex] = static_cast<float>(stamp_sec);
    return msg;
}

bool decodePolicyCommand(
    const std_msgs::msg::Float32MultiArray &msg,
    RobotCommandData *command,
    uint32_t *sequence,
    double *stamp_sec)
{
    if (!command || msg.data.size() < kJointCmdValueCount)
    {
        return false;
    }

    for (size_t i = 0; i < kLegJointCount; ++i)
    {
        const size_t offset = 3 * i;
        command->joint_target_q[i] = msg.data[offset];
        command->joint_target_dq[i] = msg.data[offset + 1];
        command->joint_target_tau[i] = msg.data[offset + 2];
    }
    command->open_rl = msg.data[kJointStateValueCount];

    if (sequence)
    {
        *sequence = static_cast<uint32_t>(std::max(0.0f, msg.data[kJointCmdSeqIndex]));
    }
    if (stamp_sec)
    {
        *stamp_sec = static_cast<double>(msg.data[kJointCmdStampIndex]);
    }
    return true;
}

std_msgs::msg::Float32MultiArray encodeRobotState(const RobotStateData &state)
{
    std_msgs::msg::Float32MultiArray msg;
    msg.data.assign(kRobotStateValueCount, 0.0f);

    for (size_t i = 0; i < kLegJointCount; ++i)
    {
        const size_t offset = 3 * i;
        msg.data[offset] = state.joint_q[i];
        msg.data[offset + 1] = state.joint_dq[i];
        msg.data[offset + 2] = state.joint_tau[i];
    }

    size_t cursor = kJointStateValueCount;
    for (size_t i = 0; i < 3; ++i)
    {
        msg.data[cursor++] = state.base_ang_vel[i];
    }
    for (size_t i = 0; i < 4; ++i)
    {
        msg.data[cursor++] = state.base_quat[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
        msg.data[cursor++] = state.base_rpy[i];
    }
    return msg;
}

bool decodeRobotState(
    const std_msgs::msg::Float32MultiArray &msg,
    RobotStateData *state)
{
    if (!state || msg.data.size() < kRobotStateValueCount)
    {
        return false;
    }

    for (size_t i = 0; i < kLegJointCount; ++i)
    {
        const size_t offset = 3 * i;
        state->joint_q[i] = msg.data[offset];
        state->joint_dq[i] = msg.data[offset + 1];
        state->joint_tau[i] = msg.data[offset + 2];
    }

    size_t cursor = kJointStateValueCount;
    for (size_t i = 0; i < 3; ++i)
    {
        state->base_ang_vel[i] = msg.data[cursor++];
    }
    for (size_t i = 0; i < 4; ++i)
    {
        state->base_quat[i] = msg.data[cursor++];
    }
    for (size_t i = 0; i < 3; ++i)
    {
        state->base_rpy[i] = msg.data[cursor++];
    }
    return true;
}

} // namespace rl_master::dds
