#include "rl_master/runtime/realtime_utils.h"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <sched.h>
#include <sys/mman.h>

namespace rl_master::runtime
{

void setRealtimePriority(int cpu_id, int priority)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        std::cerr << "mlockall failed: " << strerror(errno) << std::endl;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
    {
        std::cerr << "sched_setaffinity failed: " << strerror(errno) << std::endl;
    }

    struct sched_param param;
    param.sched_priority = priority;
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
    {
        std::cerr << "sched_setscheduler failed: " << strerror(errno) << std::endl;
    }

    std::cout << "[RT] FIFO priority=" << priority
              << " cpu=" << cpu_id << std::endl;
}

} // namespace rl_master::runtime
