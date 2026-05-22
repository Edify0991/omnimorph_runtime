#ifndef RL_MASTER_RUNTIME_INTEGRATED_CONTROLLER_RUNTIME_H
#define RL_MASTER_RUNTIME_INTEGRATED_CONTROLLER_RUNTIME_H

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "rl_master/deploy_state_machine.h"
#include "rl_master/mode_profile_registry.h"
#include "rl_master/robot_types.h"

class RL_controller;
class Sim2realCfg;

namespace rl_master::runtime
{

class IntegratedControllerRuntime
{
public:
    explicit IntegratedControllerRuntime(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry = nullptr);
    ~IntegratedControllerRuntime();

    void setModeProfileRegistry(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry);

    void initialize(int startup_mode_id = rl_master::kModeCodeMin);
    rl_master::RobotCommandData step(
        const rl_master::RobotStateData &state,
        const std::optional<rl_master::TeleopCommand> &teleop_sample = std::nullopt,
        const std::optional<int> &mode_command_sample = std::nullopt);
    void estop();

    bool initialized() const;
    const Sim2realCfg &runtimeCfg() const;
    int activeModeId() const;
    const std::string &activeConfigSection() const;
    RL_controller &controller();
    const RL_controller &controller() const;

private:
    std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry_;
    std::unique_ptr<RL_controller> controller_;
    rl_master::TeleopCommand latest_teleop_{};
    int mode_command_cache_ = rl_master::kCtrlWordSetModeBase + rl_master::kModeCodeMin;
    std::chrono::steady_clock::time_point phase_start_{};
};

} // namespace rl_master::runtime

#endif // RL_MASTER_RUNTIME_INTEGRATED_CONTROLLER_RUNTIME_H
