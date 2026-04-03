#ifndef RL_MASTER_DDS_PROTOCOL_H
#define RL_MASTER_DDS_PROTOCOL_H

#include <cstddef>
#include <cstdint>

#include <std_msgs/msg/float32_multi_array.hpp>

#include "rl_protocol.h"
#include "robot_types.h"

namespace rl_master::dds
{

constexpr const char *kTopicPolicyCommand = "/humanoid/rl/command";
constexpr const char *kTopicRobotState = "/humanoid/rl/state";
constexpr const char *kTopicTeleopCommand = "/humanoid/rl/teleop";
constexpr const char *kTopicWalkMode = "/humanoid/rl/walk_mode";

constexpr size_t kRobotStateValueCount = kJointStateValueCount + 3 + 4 + 3;

std_msgs::msg::Float32MultiArray encodePolicyCommand(
    const RobotCommandData &command,
    uint32_t sequence,
    double stamp_sec);

bool decodePolicyCommand(
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
