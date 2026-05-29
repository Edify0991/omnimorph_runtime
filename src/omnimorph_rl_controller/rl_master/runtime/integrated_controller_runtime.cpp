#include "rl_master/runtime/integrated_controller_runtime.h"

#include <stdexcept>

#include "rl_master/RL_controller.h"

namespace rl_master::runtime
{

IntegratedControllerRuntime::IntegratedControllerRuntime(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry)
    : mode_registry_(std::move(mode_registry))
{
}

IntegratedControllerRuntime::~IntegratedControllerRuntime() = default;

void IntegratedControllerRuntime::setModeProfileRegistry(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry)
{
    if (controller_)
    {
        throw std::runtime_error("cannot change mode profile registry after controller runtime initialization");
    }
    mode_registry_ = std::move(mode_registry);
}

void IntegratedControllerRuntime::initialize(int startup_mode_id)
{
    if (!mode_registry_)
    {
        throw std::runtime_error(
            "IntegratedControllerRuntime requires an injected ModeProfileRegistry before initialize().");
    }

    controller_ = RL_controller::create(mode_registry_);
    if (!controller_)
    {
        throw std::runtime_error("failed to create integrated RL_controller runtime");
    }

    controller_->RL_controller_Init(startup_mode_id);
    phase_start_ = std::chrono::steady_clock::now();

    mode_command_cache_ = rl_master::kCtrlWordSetModeBase + controller_->activeModeId();
}

rl_master::RobotCommandData IntegratedControllerRuntime::step(
    const rl_master::RobotStateData &state,
    const std::optional<rl_master::TeleopCommand> &teleop_sample,
    const std::optional<int> &mode_command_sample,
    const std::optional<double> &phase_time_sample)
{
    if (!controller_)
    {
        throw std::runtime_error("IntegratedControllerRuntime::initialize must be called before step");
    }

    if (teleop_sample.has_value())
    {
        latest_teleop_ = *teleop_sample;
    }
    if (mode_command_sample.has_value())
    {
        mode_command_cache_ = *mode_command_sample;
    }

    const double phase_t =
        phase_time_sample.value_or(
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - phase_start_)
                .count());
    return controller_->step(state, latest_teleop_, mode_command_cache_, phase_t);
}

void IntegratedControllerRuntime::estop()
{
    if (controller_)
    {
        controller_->estop();
    }
}

bool IntegratedControllerRuntime::initialized() const
{
    return static_cast<bool>(controller_);
}

const Sim2realCfg &IntegratedControllerRuntime::runtimeCfg() const
{
    if (!controller_)
    {
        throw std::runtime_error("integrated runtime is not initialized");
    }
    return controller_->runtimeCfg();
}

int IntegratedControllerRuntime::activeModeId() const
{
    if (!controller_)
    {
        return rl_master::kModeCodeMin;
    }
    return controller_->activeModeId();
}

const std::string &IntegratedControllerRuntime::activeConfigSection() const
{
    if (!controller_)
    {
        static const std::string kEmpty;
        return kEmpty;
    }
    return controller_->activeConfigSection();
}

RL_controller &IntegratedControllerRuntime::controller()
{
    if (!controller_)
    {
        throw std::runtime_error("integrated runtime is not initialized");
    }
    return *controller_;
}

const RL_controller &IntegratedControllerRuntime::controller() const
{
    if (!controller_)
    {
        throw std::runtime_error("integrated runtime is not initialized");
    }
    return *controller_;
}

} // namespace rl_master::runtime
