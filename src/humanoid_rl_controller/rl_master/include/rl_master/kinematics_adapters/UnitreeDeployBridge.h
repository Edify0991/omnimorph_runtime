#ifndef RL_MASTER_KINEMATICS_ADAPTERS_UNITREE_DEPLOY_BRIDGE_H
#define RL_MASTER_KINEMATICS_ADAPTERS_UNITREE_DEPLOY_BRIDGE_H

#include <string>

namespace rl_master
{

struct UnitreeDeployBridgeConfig
{
    std::string sdk_root;
    std::string transport;
    std::string robot_type;
};

// Placeholder external integration point for vendor deploy SDK adapters.
class UnitreeDeployBridge
{
public:
    bool configure(const UnitreeDeployBridgeConfig &cfg)
    {
        config_ = cfg;
        configured_ = true;
        return true;
    }

    bool configured() const { return configured_; }

private:
    UnitreeDeployBridgeConfig config_{};
    bool configured_ = false;
};

} // namespace rl_master

#endif
