#ifndef RL_MASTER_RUNTIME_REALTIME_UTILS_H
#define RL_MASTER_RUNTIME_REALTIME_UTILS_H

namespace rl_master::runtime
{

void setRealtimePriority(int cpu_id, int priority);

} // namespace rl_master::runtime

#endif // RL_MASTER_RUNTIME_REALTIME_UTILS_H
