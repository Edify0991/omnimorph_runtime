#ifndef RL_MASTER_ROBOT_TYPES_H
#define RL_MASTER_ROBOT_TYPES_H

#include <array>

#include "rl_protocol.h"

namespace rl_master
{

struct RobotStateData
{
    std::array<float, kLegJointCount> joint_q{};
    std::array<float, kLegJointCount> joint_dq{};
    std::array<float, kLegJointCount> joint_tau{};

    std::array<float, 3> base_ang_vel{};
    // Quaternion layout: [x, y, z, w]
    std::array<float, 4> base_quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> base_rpy{};
};

struct TeleopCommand
{
    float vx = 0.0f;
    float vy = 0.0f;
    float dyaw = 0.0f;
};

struct RobotCommandData
{
    std::array<float, kLegJointCount> joint_target_q{};
    std::array<float, kLegJointCount> joint_target_dq{};
    std::array<float, kLegJointCount> joint_target_tau{};
    float open_rl = kOpenRlDisabled;
};

} // namespace rl_master

#endif // RL_MASTER_ROBOT_TYPES_H
