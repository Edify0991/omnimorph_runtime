#ifndef RL_MASTER_SOLVER_UNITREE_SDK2_SUPPORT_H
#define RL_MASTER_SOLVER_UNITREE_SDK2_SUPPORT_H

#include <string>

#include "rl_master/rl_cfg.h"

namespace rl_master::solver
{

std::string summarizeUnitreeSdk2Config(const SourceContractUnitreeSdk2 &cfg);
void ensureUnitreeSdk2ChannelFactoryInitialized(const SourceContractUnitreeSdk2 &cfg);

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_UNITREE_SDK2_SUPPORT_H
