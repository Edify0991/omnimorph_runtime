#ifndef RL_MASTER_DDS_PROTOCOL_H
#define RL_MASTER_DDS_PROTOCOL_H

#include <cstddef>
#include <cstdint>

#include <std_msgs/msg/float32_multi_array.hpp>

#include "rl_protocol.h"
#include "robot_types.h"

namespace rl_master::dds
{

constexpr const char *kTopicRobotState = "/omnimorph/rl/state";
constexpr const char *kTopicTeleopCommand = "/omnimorph/rl/teleop";
constexpr const char *kTopicModeControl = "/omnimorph/rl/mode_control";
constexpr const char *kTopicRuntimeCommand = "/omnimorph/rl/runtime_command";

// Protocol v2 dynamic joint payload markers.
constexpr int kProtocolV2Magic = 240426;
constexpr int kProtocolV2PayloadPolicyCommand = 1;
constexpr int kProtocolV2PayloadRobotState = 2;

std_msgs::msg::Float32MultiArray encodeRuntimeCommand(
    const RobotCommandData &command,
    uint32_t sequence,
    double stamp_sec);

bool decodeRuntimeCommand(
    const std_msgs::msg::Float32MultiArray &msg,
    RobotCommandData *command,
    uint32_t *sequence,
    double *stamp_sec);

std_msgs::msg::Float32MultiArray encodeRobotState(const RobotStateData &state);

bool decodeRobotState(
    const std_msgs::msg::Float32MultiArray &msg,
    RobotStateData *state);

} // namespace rl_master::dds

#endif // RL_MASTER_DDS_PROTOCOL_H
