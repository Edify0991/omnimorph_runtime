#ifndef RL_MASTER_DEPLOY_STATE_MACHINE_H
#define RL_MASTER_DEPLOY_STATE_MACHINE_H

#include <functional>
#include <string>
#include <vector>

#include "rl_cfg.h"

namespace rl_master
{

// Generic mode id range (for deploy_mode_profiles mapping).
constexpr int kModeCodeMin = 0;
constexpr int kModeCodeMax = 999;

// Lifecycle control words (written to mode_control control channel).
// 0..999 range is reserved for lifecycle commands in control channel.
constexpr int kCtrlWordStartPolicy = 10;
constexpr int kCtrlWordStopPolicy = 11;
constexpr int kCtrlWordZeroing = 12;
constexpr int kCtrlWordEstop = 13;

// Generic extended mode control:
// - [1000, 1999]: request start for mode=(code-1000).
// - [2000, 2999]: set pending mode=(code-2000), do not change lifecycle.
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
    void setHotSwitchPredicate(std::function<bool(int, int)> predicate);
    DeployStateOutput update(
        int control_word,
        double now_s,
        const std::vector<float> &current_q,
        const std::vector<float> &current_dq);

    DeployLifecycleState state() const { return state_; }
    int activeLocomotionMode() const { return active_locomotion_mode_; }

    static DecodedControlWord decodeControlWord(int control_word, int fallback_locomotion_mode);
    static bool isValidControlWord(int control_word);
    static const char *stateName(DeployLifecycleState state);

private:
    void startZeroing(
        double now_s,
        const std::vector<float> &current_q,
        DeployLifecycleState completion_state);
    bool canHotSwitchToMode(int target_mode) const;

    bool initialized_ = false;
    double zeroing_duration_s_ = 2.0;
    double zeroing_position_tolerance_ = 0.05;
    double zeroing_velocity_tolerance_ = 0.2;
    bool startup_zeroing_pending_ = true;

    int active_locomotion_mode_ = kModeCodeMin;
    int pending_locomotion_mode_ = kModeCodeMin;
    DeployLifecycleState state_ = DeployLifecycleState::kInitializing;
    DeployLifecycleState startup_completion_state_ = DeployLifecycleState::kHold;
    DeployLifecycleState post_zeroing_state_ = DeployLifecycleState::kHold;

    double zeroing_start_time_s_ = 0.0;
    std::vector<float> zeroing_start_pose_;
    std::vector<float> zeroing_target_pose_;
    std::function<bool(int, int)> hot_switch_predicate_;
};

} // namespace rl_master

#endif // RL_MASTER_DEPLOY_STATE_MACHINE_H
