#ifndef RL_MASTER_RUNTIME_REALTIME_UTILS_H
#define RL_MASTER_RUNTIME_REALTIME_UTILS_H

#include <string>

namespace rl_master::runtime
{

struct RealtimeConfig
{
    bool enabled = true;
    bool lock_memory = true;
    bool set_affinity = true;
    int cpu_id = -1;
    bool use_fifo_scheduler = true;
    int fifo_priority = 90;
};

void configureRealtime(const RealtimeConfig &config, const std::string &tag = "");
RealtimeConfig overrideRealtimeConfigFromEnv(const RealtimeConfig &base, const std::string &prefix);
void setRealtimePriority(int cpu_id, int priority);

} // namespace rl_master::runtime

#endif // RL_MASTER_RUNTIME_REALTIME_UTILS_H
