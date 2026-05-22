#ifndef RL_MASTER_KINEMATICS_ADAPTER_H
#define RL_MASTER_KINEMATICS_ADAPTER_H

#include <string>

namespace rl_master
{

inline bool adapterRequiresKinConv(const std::string &adapter)
{
    return adapter == "jc01";
}

inline bool adapterUsesExternalDeployBridge(const std::string &adapter)
{
    return adapter == "unitree_g1";
}

inline bool isKnownKinematicsAdapter(const std::string &adapter)
{
    return adapter == "jc01" || adapter == "jc05" || adapter == "unitree_g1" || adapter == "none";
}

inline std::string normalizeKinematicsAdapter(std::string adapter)
{
    if (adapter.empty())
    {
        return "jc01";
    }
    return adapter;
}

} // namespace rl_master

#endif // RL_MASTER_KINEMATICS_ADAPTER_H
