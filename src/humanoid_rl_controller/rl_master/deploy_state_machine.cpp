#include "rl_master/deploy_state_machine.h"

#include <algorithm>

namespace rl_master
{

void DeployStateMachine::configure(const Sim2realCfg &cfg)
{
    auto_start_policy_ = cfg.auto_start_policy;
    zeroing_duration_s_ = std::max(0.05, cfg.zeroing_duration_s);
}

void DeployStateMachine::initialize(const std::vector<float> &current_q, const std::vector<float> &zero_pose, int initial_mode)
{
    zeroing_start_pose_ = current_q;
    zeroing_target_pose_ = fitDim(zero_pose, current_q.size());
    active_locomotion_mode_ = initial_mode;
    state_ = auto_start_policy_ ? DeployLifecycleState::kRunning : DeployLifecycleState::kHold;
    initialized_ = true;
}

void DeployStateMachine::setZeroPose(const std::vector<float> &zero_pose)
{
    if (zeroing_target_pose_.empty())
    {
        zeroing_target_pose_ = zero_pose;
        return;
    }
    zeroing_target_pose_ = fitDim(zero_pose, zeroing_target_pose_.size());
}

DeployStateOutput DeployStateMachine::update(int control_word, double now_s, const std::vector<float> &current_q)
{
    if (!initialized_)
    {
        initialize(current_q, current_q, active_locomotion_mode_);
    }

    const DecodedControlWord command = decodeControlWord(control_word, active_locomotion_mode_);
    active_locomotion_mode_ = command.locomotion_mode;

    if (command.request_estop)
    {
        state_ = DeployLifecycleState::kEstop;
    }
    else if (state_ != DeployLifecycleState::kEstop)
    {
        if (command.request_zero)
        {
            startZeroing(now_s, current_q);
        }
        else if (command.request_stop)
        {
            state_ = DeployLifecycleState::kHold;
        }
        else if (command.request_start)
        {
            state_ = DeployLifecycleState::kRunning;
        }
    }

    DeployStateOutput output;
    output.state = state_;
    output.locomotion_mode = active_locomotion_mode_;
    output.target_q = current_q;

    if (state_ == DeployLifecycleState::kRunning)
    {
        output.enable_policy = true;
        output.enable_command_stream = true;
        return output;
    }

    if (state_ == DeployLifecycleState::kZeroing)
    {
        output.enable_policy = false;
        output.enable_command_stream = true;

        const size_t dim = current_q.size();
        output.target_q.resize(dim, 0.0f);

        const double duration = std::max(0.05, zeroing_duration_s_);
        const double alpha = std::clamp((now_s - zeroing_start_time_s_) / duration, 0.0, 1.0);
        const std::vector<float> start_pose = fitDim(zeroing_start_pose_, dim);
        const std::vector<float> target_pose = fitDim(zeroing_target_pose_, dim);
        for (size_t i = 0; i < dim; ++i)
        {
            output.target_q[i] = static_cast<float>(start_pose[i] + (target_pose[i] - start_pose[i]) * alpha);
        }

        if (alpha >= 1.0)
        {
            state_ = DeployLifecycleState::kHold;
            output.state = state_;
            output.enable_command_stream = false;
            output.target_q = current_q;
        }
        return output;
    }

    output.enable_policy = false;
    output.enable_command_stream = false;
    output.target_q = current_q;
    return output;
}

DecodedControlWord DeployStateMachine::decodeControlWord(int control_word, int fallback_locomotion_mode)
{
    DecodedControlWord decoded;
    decoded.locomotion_mode = fallback_locomotion_mode;

    if (control_word >= kModeCodeMin && control_word <= kModeCodeMax)
    {
        decoded.locomotion_mode = control_word;
        return decoded;
    }

    if (control_word == kCtrlWordStartPolicy)
    {
        decoded.request_start = true;
        return decoded;
    }
    if (control_word == kCtrlWordStopPolicy)
    {
        decoded.request_stop = true;
        return decoded;
    }
    if (control_word == kCtrlWordZeroing)
    {
        decoded.request_zero = true;
        return decoded;
    }
    if (control_word == kCtrlWordEstop)
    {
        decoded.request_estop = true;
        return decoded;
    }

    if (control_word >= kCtrlWordStartModeBase &&
        control_word < (kCtrlWordStartModeBase + kCtrlWordModeRange))
    {
        decoded.locomotion_mode = control_word - kCtrlWordStartModeBase;
        decoded.request_start = true;
        return decoded;
    }

    if (control_word >= kCtrlWordSetModeBase &&
        control_word < (kCtrlWordSetModeBase + kCtrlWordModeRange))
    {
        decoded.locomotion_mode = control_word - kCtrlWordSetModeBase;
        return decoded;
    }

    return decoded;
}

bool DeployStateMachine::isValidControlWord(int control_word)
{
    if (control_word >= kModeCodeMin && control_word <= kModeCodeMax)
    {
        return true;
    }

    if (control_word == kCtrlWordStartPolicy ||
        control_word == kCtrlWordStopPolicy ||
        control_word == kCtrlWordZeroing ||
        control_word == kCtrlWordEstop)
    {
        return true;
    }

    if ((control_word >= kCtrlWordStartModeBase &&
         control_word < (kCtrlWordStartModeBase + kCtrlWordModeRange)) ||
        (control_word >= kCtrlWordSetModeBase &&
         control_word < (kCtrlWordSetModeBase + kCtrlWordModeRange)))
    {
        return true;
    }

    return false;
}

const char *DeployStateMachine::stateName(DeployLifecycleState state)
{
    switch (state)
    {
    case DeployLifecycleState::kInitializing:
        return "INITIALIZING";
    case DeployLifecycleState::kHold:
        return "HOLD";
    case DeployLifecycleState::kZeroing:
        return "ZEROING";
    case DeployLifecycleState::kRunning:
        return "RUNNING";
    case DeployLifecycleState::kEstop:
        return "ESTOP";
    default:
        return "UNKNOWN";
    }
}

void DeployStateMachine::startZeroing(double now_s, const std::vector<float> &current_q)
{
    zeroing_start_time_s_ = now_s;
    zeroing_start_pose_ = current_q;
    zeroing_target_pose_ = fitDim(zeroing_target_pose_, current_q.size());
    state_ = DeployLifecycleState::kZeroing;
}

std::vector<float> DeployStateMachine::fitDim(const std::vector<float> &values, size_t dim)
{
    std::vector<float> out(dim, 0.0f);
    const size_t copy_n = std::min(values.size(), dim);
    std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(copy_n), out.begin());
    return out;
}

} // namespace rl_master
