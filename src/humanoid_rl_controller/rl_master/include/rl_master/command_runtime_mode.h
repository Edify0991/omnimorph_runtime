#ifndef RL_MASTER_COMMAND_RUNTIME_MODE_H
#define RL_MASTER_COMMAND_RUNTIME_MODE_H

#include "rl_protocol.h"

namespace rl_master
{

enum class CommandRuntimeMode
{
    kHold = 0,
    kPolicy,
    kCommandStream,
    kTestCsp,
    kTestCst,
    kTestR1,
};

struct CommandRuntimeDecision
{
    CommandRuntimeMode mode = CommandRuntimeMode::kHold;
    bool command_fresh = false;
    bool open_rl_active = false;
    bool unknown_open_rl_mode = false;
};

inline CommandRuntimeDecision resolveCommandRuntimeMode(
    bool command_fresh,
    float open_rl_value,
    float open_rl_enable_threshold = 1.0f)
{
    CommandRuntimeDecision decision;
    decision.command_fresh = command_fresh;
    if (!command_fresh)
    {
        return decision;
    }

    if (isOpenRlPolicyEnabled(open_rl_value))
    {
        decision.mode = CommandRuntimeMode::kPolicy;
        decision.open_rl_active = true;
        return decision;
    }
    if (isOpenRlCommandStream(open_rl_value))
    {
        decision.mode = CommandRuntimeMode::kCommandStream;
        decision.open_rl_active = true;
        return decision;
    }
    if (isOpenRlTestCspStream(open_rl_value))
    {
        decision.mode = CommandRuntimeMode::kTestCsp;
        decision.open_rl_active = true;
        return decision;
    }
    if (isOpenRlTestCstStream(open_rl_value))
    {
        decision.mode = CommandRuntimeMode::kTestCst;
        decision.open_rl_active = true;
        return decision;
    }
    if (isOpenRlTestR1Stream(open_rl_value))
    {
        decision.mode = CommandRuntimeMode::kTestR1;
        decision.open_rl_active = true;
        return decision;
    }

    if (open_rl_value > open_rl_enable_threshold)
    {
        decision.unknown_open_rl_mode = true;
    }
    return decision;
}

inline bool modeUsesPositionTargets(CommandRuntimeMode mode)
{
    return mode == CommandRuntimeMode::kPolicy ||
           mode == CommandRuntimeMode::kCommandStream ||
           mode == CommandRuntimeMode::kTestCsp ||
           mode == CommandRuntimeMode::kTestR1;
}

inline bool modeUsesVelocityTargets(CommandRuntimeMode mode)
{
    return mode == CommandRuntimeMode::kPolicy ||
           mode == CommandRuntimeMode::kTestR1;
}

inline bool modeUsesDirectTorqueTargets(CommandRuntimeMode mode)
{
    return mode == CommandRuntimeMode::kTestCst ||
           mode == CommandRuntimeMode::kTestR1;
}

inline bool modeUsesTorqueFeedForward(CommandRuntimeMode mode)
{
    return mode == CommandRuntimeMode::kPolicy ||
           mode == CommandRuntimeMode::kTestR1;
}

inline const char *commandRuntimeModeName(CommandRuntimeMode mode)
{
    switch (mode)
    {
    case CommandRuntimeMode::kHold:
        return "hold";
    case CommandRuntimeMode::kPolicy:
        return "policy";
    case CommandRuntimeMode::kCommandStream:
        return "command_stream";
    case CommandRuntimeMode::kTestCsp:
        return "test_csp";
    case CommandRuntimeMode::kTestCst:
        return "test_cst";
    case CommandRuntimeMode::kTestR1:
        return "test_r1";
    default:
        return "unknown";
    }
}

} // namespace rl_master

#endif // RL_MASTER_COMMAND_RUNTIME_MODE_H
