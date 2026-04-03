#ifndef RL_PROTOCOL_H
#define RL_PROTOCOL_H

#include <chrono>
#include <cstdint>

namespace rl_master
{
constexpr int kLegJointCount = 12;
constexpr int kJointStateValueCount = kLegJointCount * 3; // q, dq, tau
constexpr int kJointCmdLegacyCount = kJointStateValueCount + 1; // + open_rl
constexpr int kJointCmdSeqIndex = kJointCmdLegacyCount;
constexpr int kJointCmdStampIndex = kJointCmdLegacyCount + 1;
constexpr int kJointCmdValueCount = kJointCmdLegacyCount + 2;

constexpr float kOpenRlDisabled = 0.0f;
constexpr float kOpenRlEnabled = 10.0f;

inline double monotonicTimeSec()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
}
} // namespace rl_master

#endif // RL_PROTOCOL_H
