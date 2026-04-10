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

inline bool commandShouldUseV2(const RobotCommandData &command)
{
    return command.protocol_version >= kProtocolVersionDynamicJointsV2 ||
           command.active_joint_count != kLegJointCount ||
           !command.joint_target_q_full.empty() ||
           !command.joint_target_dq_full.empty() ||
           !command.joint_target_tau_full.empty();
}

inline bool stateShouldUseV2(const RobotStateData &state)
{
    return state.protocol_version >= kProtocolVersionDynamicJointsV2 ||
           state.active_joint_count != kLegJointCount ||
           !state.joint_q_full.empty() ||
           !state.joint_dq_full.empty() ||
           !state.joint_tau_full.empty();
}

inline size_t resolveCommandJointCount(const RobotCommandData &command)
{
    if (command.active_joint_count > 0)
    {
        return static_cast<size_t>(command.active_joint_count);
    }
    const size_t dyn_n = max3(
        command.joint_target_q_full.size(),
        command.joint_target_dq_full.size(),
        command.joint_target_tau_full.size());
    return dyn_n > 0 ? dyn_n : static_cast<size_t>(kLegJointCount);
}

inline size_t resolveStateJointCount(const RobotStateData &state)
{
    if (state.active_joint_count > 0)
    {
        return static_cast<size_t>(state.active_joint_count);
    }
    const size_t dyn_n = max3(
        state.joint_q_full.size(),
        state.joint_dq_full.size(),
        state.joint_tau_full.size());
    return dyn_n > 0 ? dyn_n : static_cast<size_t>(kLegJointCount);
}

inline float readCommandQ(const RobotCommandData &command, size_t index)
{
    if (index < command.joint_target_q_full.size())
    {
        return command.joint_target_q_full[index];
    }
    return index < static_cast<size_t>(kLegJointCount) ? command.joint_target_q[index] : 0.0f;
}

inline float readCommandDq(const RobotCommandData &command, size_t index)
{
    if (index < command.joint_target_dq_full.size())
    {
        return command.joint_target_dq_full[index];
    }
    return index < static_cast<size_t>(kLegJointCount) ? command.joint_target_dq[index] : 0.0f;
}

inline float readCommandTau(const RobotCommandData &command, size_t index)
{
    if (index < command.joint_target_tau_full.size())
    {
        return command.joint_target_tau_full[index];
    }
    return index < static_cast<size_t>(kLegJointCount) ? command.joint_target_tau[index] : 0.0f;
}

inline float readStateQ(const RobotStateData &state, size_t index)
{
    if (index < state.joint_q_full.size())
    {
        return state.joint_q_full[index];
    }
    return index < static_cast<size_t>(kLegJointCount) ? state.joint_q[index] : 0.0f;
}

inline float readStateDq(const RobotStateData &state, size_t index)
{
    if (index < state.joint_dq_full.size())
    {
        return state.joint_dq_full[index];
    }
    return index < static_cast<size_t>(kLegJointCount) ? state.joint_dq[index] : 0.0f;
}

inline float readStateTau(const RobotStateData &state, size_t index)
{
    if (index < state.joint_tau_full.size())
    {
        return state.joint_tau_full[index];
    }
    return index < static_cast<size_t>(kLegJointCount) ? state.joint_tau[index] : 0.0f;
}
} // namespace

std_msgs::msg::Float32MultiArray encodePolicyCommand(
    const RobotCommandData &command,
    uint32_t sequence,
    double stamp_sec)
{
    std_msgs::msg::Float32MultiArray msg;

    const bool use_v2 = commandShouldUseV2(command);
    if (!use_v2)
    {
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

bool decodePolicyCommand(
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
        command->joint_target_q_full.assign(joint_count, 0.0f);
        command->joint_target_dq_full.assign(joint_count, 0.0f);
        command->joint_target_tau_full.assign(joint_count, 0.0f);

        size_t cursor = kPolicyCmdV2HeaderCount;
        for (size_t i = 0; i < joint_count; ++i)
        {
            command->joint_target_q_full[i] = msg.data[cursor++];
            command->joint_target_dq_full[i] = msg.data[cursor++];
            command->joint_target_tau_full[i] = msg.data[cursor++];
        }
        command->syncLegacyFromDynamic();
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

    if (msg.data.size() < kJointCmdValueCount)
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
    command->syncDynamicFromLegacy();
    return true;
}

std_msgs::msg::Float32MultiArray encodeRobotState(const RobotStateData &state)
{
    std_msgs::msg::Float32MultiArray msg;

    const bool use_v2 = stateShouldUseV2(state);
    if (!use_v2)
    {
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

    const size_t joint_count = resolveStateJointCount(state);
    msg.data.assign(kStateV2HeaderCount + joint_count * 3 + kBaseStateTailCount, 0.0f);
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
        state->joint_q_full.assign(joint_count, 0.0f);
        state->joint_dq_full.assign(joint_count, 0.0f);
        state->joint_tau_full.assign(joint_count, 0.0f);

        size_t cursor = kStateV2HeaderCount;
        for (size_t i = 0; i < joint_count; ++i)
        {
            state->joint_q_full[i] = msg.data[cursor++];
            state->joint_dq_full[i] = msg.data[cursor++];
            state->joint_tau_full[i] = msg.data[cursor++];
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

        state->syncLegacyFromDynamic();
        return true;
    }

    if (msg.data.size() < kRobotStateValueCount)
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
    state->syncDynamicFromLegacy();
    return true;
}

} // namespace rl_master::dds
