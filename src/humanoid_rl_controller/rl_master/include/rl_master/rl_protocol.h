#ifndef RL_PROTOCOL_H
#define RL_PROTOCOL_H

#include <chrono>
#include <cstdint>

namespace rl_master
{
// Protocol versions.
constexpr int kProtocolVersionDynamicJointsV2 = 2;

constexpr float kOpenRlDisabled = 0.0f;
constexpr float kOpenRlPolicyEnabled = 10.0f;
constexpr float kOpenRlCommandStream = 20.0f;
constexpr float kOpenRlTestCspStream = 30.0f;
constexpr float kOpenRlTestCstStream = 40.0f;
constexpr float kOpenRlTestR1Stream = 50.0f;

inline bool isOpenRlPolicyEnabled(float open_rl_value)
{
    return open_rl_value >= (kOpenRlPolicyEnabled - 0.5f) &&
           open_rl_value < (kOpenRlPolicyEnabled + 0.5f);
}

inline bool isOpenRlCommandStream(float open_rl_value)
{
    return open_rl_value >= (kOpenRlCommandStream - 0.5f) &&
           open_rl_value < (kOpenRlCommandStream + 0.5f);
}

inline bool isOpenRlTestCspStream(float open_rl_value)
{
    return open_rl_value >= (kOpenRlTestCspStream - 0.5f) &&
           open_rl_value < (kOpenRlTestCspStream + 0.5f);
}

inline bool isOpenRlTestCstStream(float open_rl_value)
{
    return open_rl_value >= (kOpenRlTestCstStream - 0.5f) &&
           open_rl_value < (kOpenRlTestCstStream + 0.5f);
}

inline bool isOpenRlTestR1Stream(float open_rl_value)
{
    return open_rl_value >= (kOpenRlTestR1Stream - 0.5f) &&
           open_rl_value < (kOpenRlTestR1Stream + 0.5f);
}

inline bool isOpenRlAnyEnabled(float open_rl_value)
{
    return isOpenRlPolicyEnabled(open_rl_value) ||
           isOpenRlCommandStream(open_rl_value) ||
           isOpenRlTestCspStream(open_rl_value) ||
           isOpenRlTestCstStream(open_rl_value) ||
           isOpenRlTestR1Stream(open_rl_value);
}

inline double monotonicTimeSec()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::duration<double>>(now).count();
}
} // namespace rl_master

#endif // RL_PROTOCOL_H
