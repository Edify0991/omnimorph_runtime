#include "rl_master/dds_protocol.h"

#include <algorithm>
#include <cstddef>
#include <cmath>

namespace rl_master::dds
{
namespace
{
constexpr size_t kPolicyCmdV2HeaderCount = 7;
constexpr size_t kStateV2HeaderCount = 4;
constexpr size_t kBaseStateTailCount = 3 + 4 + 3;
constexpr size_t kBaseStateExtendedTailCount = kBaseStateTailCount + 3 + 3 + 2;

inline int toIntField(float value)
{
    return static_cast<int>(std::lround(value));
}

inline bool isV2Envelope(
    const std_msgs::msg::Float32MultiArray &msg,
    int expected_payload_type)
{
    if (msg.data.size() < kStateV2HeaderCount)
    {
        return false;
    }
    if (toIntField(msg.data[0]) != kProtocolV2Magic)
    {
        return false;
    }
    if (toIntField(msg.data[1]) != kProtocolVersionDynamicJointsV2)
    {
        return false;
    }
    if (toIntField(msg.data[2]) != expected_payload_type)
    {
        return false;
    }
    return true;
}

inline size_t max3(size_t a, size_t b, size_t c)
{
    return std::max(a, std::max(b, c));
}

inline size_t resolveCommandJointCount(const RobotCommandData &command)
{
    if (command.active_joint_count > 0)
    {
        return static_cast<size_t>(command.active_joint_count);
    }
    return max3(
        command.joint_target_q.size(),
        command.joint_target_dq.size(),
        command.joint_target_tau.size());
}

inline size_t resolveStateJointCount(const RobotStateData &state)
{
    if (state.active_joint_count > 0)
    {
        return static_cast<size_t>(state.active_joint_count);
    }
    return max3(
        state.joint_q.size(),
        state.joint_dq.size(),
        state.joint_tau.size());
}

inline float readCommandQ(const RobotCommandData &command, size_t index)
{
    return index < command.joint_target_q.size() ? command.joint_target_q[index] : 0.0f;
}

inline float readCommandDq(const RobotCommandData &command, size_t index)
{
    return index < command.joint_target_dq.size() ? command.joint_target_dq[index] : 0.0f;
}

inline float readCommandTau(const RobotCommandData &command, size_t index)
{
    return index < command.joint_target_tau.size() ? command.joint_target_tau[index] : 0.0f;
}

inline float readStateQ(const RobotStateData &state, size_t index)
{
    return index < state.joint_q.size() ? state.joint_q[index] : 0.0f;
}

inline float readStateDq(const RobotStateData &state, size_t index)
{
    return index < state.joint_dq.size() ? state.joint_dq[index] : 0.0f;
}

inline float readStateTau(const RobotStateData &state, size_t index)
{
    return index < state.joint_tau.size() ? state.joint_tau[index] : 0.0f;
}
} // namespace

std_msgs::msg::Float32MultiArray encodeRuntimeCommand(
    const RobotCommandData &command,
    uint32_t sequence,
    double stamp_sec)
{
    std_msgs::msg::Float32MultiArray msg;

    const size_t joint_count = resolveCommandJointCount(command);
    msg.data.assign(kPolicyCmdV2HeaderCount + joint_count * 3, 0.0f);
    msg.data[0] = static_cast<float>(kProtocolV2Magic);
    msg.data[1] = static_cast<float>(kProtocolVersionDynamicJointsV2);
    msg.data[2] = static_cast<float>(kProtocolV2PayloadPolicyCommand);
    msg.data[3] = static_cast<float>(joint_count);
    msg.data[4] = command.open_rl;
    msg.data[5] = static_cast<float>(sequence);
    msg.data[6] = static_cast<float>(stamp_sec);

    size_t cursor = kPolicyCmdV2HeaderCount;
    for (size_t i = 0; i < joint_count; ++i)
    {
        msg.data[cursor++] = readCommandQ(command, i);
        msg.data[cursor++] = readCommandDq(command, i);
        msg.data[cursor++] = readCommandTau(command, i);
    }
    return msg;
}

bool decodeRuntimeCommand(
    const std_msgs::msg::Float32MultiArray &msg,
    RobotCommandData *command,
    uint32_t *sequence,
    double *stamp_sec)
{
    if (!command)
    {
        return false;
    }

    if (isV2Envelope(msg, kProtocolV2PayloadPolicyCommand))
    {
        if (msg.data.size() < kPolicyCmdV2HeaderCount)
        {
            return false;
        }

        const int joint_count_i = toIntField(msg.data[3]);
        if (joint_count_i < 0)
        {
            return false;
        }
        const size_t joint_count = static_cast<size_t>(joint_count_i);
        const size_t expected_size = kPolicyCmdV2HeaderCount + joint_count * 3;
        if (msg.data.size() < expected_size)
        {
            return false;
        }

        command->protocol_version = kProtocolVersionDynamicJointsV2;
        command->active_joint_count = joint_count_i;
        command->joint_target_q.assign(joint_count, 0.0f);
        command->joint_target_dq.assign(joint_count, 0.0f);
        command->joint_target_tau.assign(joint_count, 0.0f);

        size_t cursor = kPolicyCmdV2HeaderCount;
        for (size_t i = 0; i < joint_count; ++i)
        {
            command->joint_target_q[i] = msg.data[cursor++];
            command->joint_target_dq[i] = msg.data[cursor++];
            command->joint_target_tau[i] = msg.data[cursor++];
        }
        command->open_rl = msg.data[4];

        if (sequence)
        {
            *sequence = static_cast<uint32_t>(std::max(0, toIntField(msg.data[5])));
        }
        if (stamp_sec)
        {
            *stamp_sec = static_cast<double>(msg.data[6]);
        }
        return true;
    }
    return false;
}

std_msgs::msg::Float32MultiArray encodeRobotState(const RobotStateData &state)
{
    std_msgs::msg::Float32MultiArray msg;

    const size_t joint_count = resolveStateJointCount(state);
    msg.data.assign(kStateV2HeaderCount + joint_count * 3 + kBaseStateExtendedTailCount, 0.0f);
    msg.data[0] = static_cast<float>(kProtocolV2Magic);
    msg.data[1] = static_cast<float>(kProtocolVersionDynamicJointsV2);
    msg.data[2] = static_cast<float>(kProtocolV2PayloadRobotState);
    msg.data[3] = static_cast<float>(joint_count);

    size_t cursor = kStateV2HeaderCount;
    for (size_t i = 0; i < joint_count; ++i)
    {
        msg.data[cursor++] = readStateQ(state, i);
        msg.data[cursor++] = readStateDq(state, i);
        msg.data[cursor++] = readStateTau(state, i);
    }
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
    for (size_t i = 0; i < 3; ++i)
    {
        msg.data[cursor++] = state.base_lin_vel[i];
    }
    for (size_t i = 0; i < 3; ++i)
    {
        msg.data[cursor++] = state.base_lin_acc[i];
    }
    msg.data[cursor++] = state.base_lin_vel_valid ? 1.0f : 0.0f;
    msg.data[cursor++] = state.base_lin_acc_valid ? 1.0f : 0.0f;
    return msg;
}

bool decodeRobotState(
    const std_msgs::msg::Float32MultiArray &msg,
    RobotStateData *state)
{
    if (!state)
    {
        return false;
    }

    if (isV2Envelope(msg, kProtocolV2PayloadRobotState))
    {
        if (msg.data.size() < kStateV2HeaderCount)
        {
            return false;
        }

        const int joint_count_i = toIntField(msg.data[3]);
        if (joint_count_i < 0)
        {
            return false;
        }
        const size_t joint_count = static_cast<size_t>(joint_count_i);
        const size_t expected_size = kStateV2HeaderCount + joint_count * 3 + kBaseStateTailCount;
        if (msg.data.size() < expected_size)
        {
            return false;
        }

        state->protocol_version = kProtocolVersionDynamicJointsV2;
        state->active_joint_count = joint_count_i;
        state->joint_q.assign(joint_count, 0.0f);
        state->joint_dq.assign(joint_count, 0.0f);
        state->joint_tau.assign(joint_count, 0.0f);

        size_t cursor = kStateV2HeaderCount;
        for (size_t i = 0; i < joint_count; ++i)
        {
            state->joint_q[i] = msg.data[cursor++];
            state->joint_dq[i] = msg.data[cursor++];
            state->joint_tau[i] = msg.data[cursor++];
        }

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
        bool has_extended_lin_vel = false;
        bool has_extended_lin_acc = false;
        if (msg.data.size() >= cursor + 3)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                state->base_lin_vel[i] = msg.data[cursor++];
            }
            has_extended_lin_vel = true;
        }
        if (msg.data.size() >= cursor + 3)
        {
            for (size_t i = 0; i < 3; ++i)
            {
                state->base_lin_acc[i] = msg.data[cursor++];
            }
            has_extended_lin_acc = true;
        }
        if (msg.data.size() >= cursor + 2)
        {
            state->base_lin_vel_valid = msg.data[cursor++] > 0.5f;
            state->base_lin_acc_valid = msg.data[cursor++] > 0.5f;
        }
        else
        {
            state->base_lin_vel_valid = has_extended_lin_vel;
            state->base_lin_acc_valid = has_extended_lin_acc;
        }

        return true;
    }
    return false;
}

} // namespace rl_master::dds
