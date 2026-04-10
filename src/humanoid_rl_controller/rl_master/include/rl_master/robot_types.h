#ifndef RL_MASTER_ROBOT_TYPES_H
#define RL_MASTER_ROBOT_TYPES_H

#include <algorithm>
#include <array>
#include <vector>

#include "rl_protocol.h"

namespace rl_master
{

struct RobotStateData
{
    std::array<float, kLegJointCount> joint_q{};
    std::array<float, kLegJointCount> joint_dq{};
    std::array<float, kLegJointCount> joint_tau{};

    // v2 dynamic joint payload fields.
    int protocol_version = kProtocolVersionLegacy;
    int active_joint_count = kLegJointCount;
    std::vector<float> joint_q_full;
    std::vector<float> joint_dq_full;
    std::vector<float> joint_tau_full;

    std::array<float, 3> base_ang_vel{};
    // Quaternion layout: [x, y, z, w]
    std::array<float, 4> base_quat{0.0f, 0.0f, 0.0f, 1.0f};
    std::array<float, 3> base_rpy{};

    void syncLegacyFromDynamic()
    {
        joint_q.fill(0.0f);
        joint_dq.fill(0.0f);
        joint_tau.fill(0.0f);
        const size_t n = std::min<size_t>(kLegJointCount, joint_q_full.size());
        for (size_t i = 0; i < n; ++i)
        {
            joint_q[i] = joint_q_full[i];
        }
        const size_t ndq = std::min<size_t>(kLegJointCount, joint_dq_full.size());
        for (size_t i = 0; i < ndq; ++i)
        {
            joint_dq[i] = joint_dq_full[i];
        }
        const size_t ntau = std::min<size_t>(kLegJointCount, joint_tau_full.size());
        for (size_t i = 0; i < ntau; ++i)
        {
            joint_tau[i] = joint_tau_full[i];
        }
    }

    void syncDynamicFromLegacy()
    {
        protocol_version = kProtocolVersionLegacy;
        active_joint_count = kLegJointCount;
        joint_q_full.assign(joint_q.begin(), joint_q.end());
        joint_dq_full.assign(joint_dq.begin(), joint_dq.end());
        joint_tau_full.assign(joint_tau.begin(), joint_tau.end());
    }
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

    // v2 dynamic joint payload fields.
    int protocol_version = kProtocolVersionLegacy;
    int active_joint_count = kLegJointCount;
    std::vector<float> joint_target_q_full;
    std::vector<float> joint_target_dq_full;
    std::vector<float> joint_target_tau_full;

    float open_rl = kOpenRlDisabled;

    void syncLegacyFromDynamic()
    {
        joint_target_q.fill(0.0f);
        joint_target_dq.fill(0.0f);
        joint_target_tau.fill(0.0f);
        const size_t n = std::min<size_t>(kLegJointCount, joint_target_q_full.size());
        for (size_t i = 0; i < n; ++i)
        {
            joint_target_q[i] = joint_target_q_full[i];
        }
        const size_t ndq = std::min<size_t>(kLegJointCount, joint_target_dq_full.size());
        for (size_t i = 0; i < ndq; ++i)
        {
            joint_target_dq[i] = joint_target_dq_full[i];
        }
        const size_t ntau = std::min<size_t>(kLegJointCount, joint_target_tau_full.size());
        for (size_t i = 0; i < ntau; ++i)
        {
            joint_target_tau[i] = joint_target_tau_full[i];
        }
    }

    void syncDynamicFromLegacy()
    {
        protocol_version = kProtocolVersionLegacy;
        active_joint_count = kLegJointCount;
        joint_target_q_full.assign(joint_target_q.begin(), joint_target_q.end());
        joint_target_dq_full.assign(joint_target_dq.begin(), joint_target_dq.end());
        joint_target_tau_full.assign(joint_target_tau.begin(), joint_target_tau.end());
    }
};

} // namespace rl_master

#endif // RL_MASTER_ROBOT_TYPES_H
