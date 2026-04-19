#ifndef RL_MASTER_DEPLOY_STATE_MACHINE_H
#define RL_MASTER_DEPLOY_STATE_MACHINE_H

#include <string>
#include <vector>

#include "rl_cfg.h"

namespace rl_master
{

// Generic mode id range (for deploy_mode_profiles mapping).
constexpr int kModeCodeMin = 0;
constexpr int kModeCodeMax = 999;

// Lifecycle control words (written to walk_mode control channel).
// 0..999 range is reserved for lifecycle commands in control channel.
constexpr int kCtrlWordStartPolicy = 10;
constexpr int kCtrlWordStopPolicy = 11;
constexpr int kCtrlWordZeroing = 12;
constexpr int kCtrlWordEstop = 13;

// Legacy lifecycle words kept for compatibility with tools that used 3001..3004.
constexpr int kLegacyCtrlWordLifecycleBase = 3000;
constexpr int kLegacyCtrlWordStartPolicy = kLegacyCtrlWordLifecycleBase + 1;
constexpr int kLegacyCtrlWordStopPolicy = kLegacyCtrlWordLifecycleBase + 2;
constexpr int kLegacyCtrlWordZeroing = kLegacyCtrlWordLifecycleBase + 3;
constexpr int kLegacyCtrlWordEstop = kLegacyCtrlWordLifecycleBase + 4;

// Generic extended mode control:
// - [1000, 1999]: set mode=(code-1000) and request start.
// - [2000, 2999]: set mode=(code-2000), do not change lifecycle.
constexpr int kCtrlWordStartModeBase = 1000;
constexpr int kCtrlWordSetModeBase = 2000;
constexpr int kCtrlWordModeRange = 1000;

enum class DeployLifecycleState
{
    kInitializing = 0,
    kHold = 1,
    kZeroing = 2,
    kRunning = 3,
    kEstop = 4
};

struct DecodedControlWord
{
    int locomotion_mode = kModeCodeMin;
    bool request_start = false;
    bool request_stop = false;
    bool request_zero = false;
    bool request_estop = false;
};

struct DeployStateOutput
{
    DeployLifecycleState state = DeployLifecycleState::kInitializing;
    int locomotion_mode = kModeCodeMin;
    bool enable_policy = false;
    bool enable_command_stream = false;
    std::vector<float> target_q;
};

class DeployStateMachine
{
public:
    DeployStateMachine() = default;

    void configure(const Sim2realCfg &cfg);
    void initialize(const std::vector<float> &current_q, const std::vector<float> &zero_pose, int initial_mode);
    void setZeroPose(const std::vector<float> &zero_pose);
    DeployStateOutput update(int control_word, double now_s, const std::vector<float> &current_q);

    DeployLifecycleState state() const { return state_; }
    int activeLocomotionMode() const { return active_locomotion_mode_; }

    static DecodedControlWord decodeControlWord(int control_word, int fallback_locomotion_mode);
    static bool isValidControlWord(int control_word);
    static const char *stateName(DeployLifecycleState state);

private:
    void startZeroing(double now_s, const std::vector<float> &current_q);

    bool initialized_ = false;
    bool auto_start_policy_ = true;
    double zeroing_duration_s_ = 2.0;

    int active_locomotion_mode_ = kModeCodeMin;
    DeployLifecycleState state_ = DeployLifecycleState::kInitializing;

    double zeroing_start_time_s_ = 0.0;
    std::vector<float> zeroing_start_pose_;
    std::vector<float> zeroing_target_pose_;
};

} // namespace rl_master

#endif // RL_MASTER_DEPLOY_STATE_MACHINE_H
