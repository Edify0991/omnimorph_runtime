#include "rl_master/solver/unitree_sdk2_support.h"

#ifdef RL_MASTER_HAS_UNITREE_SDK2
#include <mutex>
#include <sstream>
#include <stdexcept>

#include <unitree/robot/channel/channel_factory.hpp>
#endif

namespace rl_master::solver
{

std::string summarizeUnitreeSdk2Config(const SourceContractUnitreeSdk2 &cfg)
{
    std::ostringstream oss;
    oss << "domain_id=" << cfg.domain_id
        << ", network_interface='" << cfg.network_interface
        << "', lowcmd_topic='" << cfg.lowcmd_topic
        << "', lowstate_topic='" << cfg.lowstate_topic
        << "', imu_topic='" << cfg.imu_topic
        << "', queue_len=" << cfg.queue_len
        << ", writer_period_us=" << cfg.writer_period_us
        << ", mode_pr=" << cfg.mode_pr;
    return oss.str();
}

void ensureUnitreeSdk2ChannelFactoryInitialized(const SourceContractUnitreeSdk2 &cfg)
{
#ifdef RL_MASTER_HAS_UNITREE_SDK2
    static std::mutex init_mutex;
    static bool initialized = false;
    static std::string initialized_key;

    const std::string requested_key = summarizeUnitreeSdk2Config(cfg);
    std::lock_guard<std::mutex> lock(init_mutex);
    if (!initialized)
    {
        unitree::robot::ChannelFactory::Instance()->Init(cfg.domain_id, cfg.network_interface);
        initialized = true;
        initialized_key = requested_key;
        return;
    }
    if (initialized_key != requested_key)
    {
        throw std::runtime_error(
            "Unitree SDK2 ChannelFactory was already initialized with a different config. "
            "First: {" + initialized_key + "}, requested: {" + requested_key + "}");
    }
#else
    (void)cfg;
    throw std::runtime_error("Unitree SDK2 support is not compiled in");
#endif
}

} // namespace rl_master::solver
