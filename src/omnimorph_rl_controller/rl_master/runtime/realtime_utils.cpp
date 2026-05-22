#include "rl_master/runtime/realtime_utils.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <string>

#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

namespace rl_master::runtime
{
namespace
{
std::string toLowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool tryParseBool(const std::string &text, bool *out)
{
    if (!out)
    {
        return false;
    }
    const std::string normalized = toLowerCopy(text);
    if (normalized == "1" || normalized == "true" || normalized == "on" || normalized == "yes")
    {
        *out = true;
        return true;
    }
    if (normalized == "0" || normalized == "false" || normalized == "off" || normalized == "no")
    {
        *out = false;
        return true;
    }
    return false;
}

bool tryParseInt(const std::string &text, int *out)
{
    if (!out)
    {
        return false;
    }
    try
    {
        size_t consumed = 0;
        const long value = std::stol(text, &consumed, 10);
        if (consumed != text.size() ||
            value < static_cast<long>(std::numeric_limits<int>::min()) ||
            value > static_cast<long>(std::numeric_limits<int>::max()))
        {
            return false;
        }
        *out = static_cast<int>(value);
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

void applyBoolEnv(const std::string &key, bool *value)
{
    if (!value)
    {
        return;
    }
    const char *raw = std::getenv(key.c_str());
    if (!raw)
    {
        return;
    }
    if (raw[0] == '\0')
    {
        return;
    }
    bool parsed = *value;
    if (!tryParseBool(raw, &parsed))
    {
        std::cerr << "[RT] ignore invalid bool env " << key << "=" << raw << std::endl;
        return;
    }
    *value = parsed;
}

void applyIntEnv(const std::string &key, int *value)
{
    if (!value)
    {
        return;
    }
    const char *raw = std::getenv(key.c_str());
    if (!raw)
    {
        return;
    }
    if (raw[0] == '\0')
    {
        return;
    }
    int parsed = *value;
    if (!tryParseInt(raw, &parsed))
    {
        std::cerr << "[RT] ignore invalid int env " << key << "=" << raw << std::endl;
        return;
    }
    *value = parsed;
}
} // namespace

RealtimeConfig overrideRealtimeConfigFromEnv(const RealtimeConfig &base, const std::string &prefix)
{
    RealtimeConfig config = base;
    const std::string key_prefix = prefix.empty() ? "RL_MASTER_RT_" : prefix;
    applyBoolEnv(key_prefix + "ENABLED", &config.enabled);
    applyBoolEnv(key_prefix + "LOCK_MEMORY", &config.lock_memory);
    applyBoolEnv(key_prefix + "SET_AFFINITY", &config.set_affinity);
    applyIntEnv(key_prefix + "CPU_ID", &config.cpu_id);
    applyBoolEnv(key_prefix + "USE_FIFO", &config.use_fifo_scheduler);
    applyIntEnv(key_prefix + "FIFO_PRIORITY", &config.fifo_priority);
    return config;
}

void configureRealtime(const RealtimeConfig &config, const std::string &tag)
{
    const std::string label = tag.empty() ? "RT" : ("RT:" + tag);

    if (!config.enabled)
    {
        std::cout << "[" << label << "] realtime config disabled." << std::endl;
        return;
    }

    if (config.lock_memory)
    {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        {
            std::cerr << "[" << label << "] mlockall failed: " << strerror(errno) << std::endl;
        }
    }

    if (config.set_affinity)
    {
        if (config.cpu_id < 0)
        {
            std::cout << "[" << label << "] affinity requested but cpu_id < 0, skip." << std::endl;
        }
        else
        {
            long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
            if (cpu_count <= 0)
            {
                cpu_count = 1;
            }
            if (config.cpu_id >= cpu_count)
            {
                std::cerr << "[" << label << "] cpu_id " << config.cpu_id
                          << " out of range [0," << (cpu_count - 1) << "], skip affinity." << std::endl;
            }
            else
            {
                cpu_set_t cpuset;
                CPU_ZERO(&cpuset);
                CPU_SET(config.cpu_id, &cpuset);
                if (sched_setaffinity(0, sizeof(cpuset), &cpuset) != 0)
                {
                    std::cerr << "[" << label << "] sched_setaffinity failed: " << strerror(errno) << std::endl;
                }
            }
        }
    }

    if (config.use_fifo_scheduler)
    {
        const int priority = std::clamp(config.fifo_priority, 1, 99);
        struct sched_param param;
        param.sched_priority = priority;
        if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
        {
            std::cerr << "[" << label << "] sched_setscheduler failed: " << strerror(errno) << std::endl;
        }
    }

    std::cout << "[" << label << "] enabled=" << (config.enabled ? "true" : "false")
              << ", lock_memory=" << (config.lock_memory ? "true" : "false")
              << ", set_affinity=" << (config.set_affinity ? "true" : "false")
              << ", cpu_id=" << config.cpu_id
              << ", use_fifo=" << (config.use_fifo_scheduler ? "true" : "false")
              << ", fifo_priority=" << config.fifo_priority
              << std::endl;
}

void setRealtimePriority(int cpu_id, int priority)
{
    RealtimeConfig config;
    config.enabled = true;
    config.lock_memory = true;
    config.set_affinity = true;
    config.cpu_id = cpu_id;
    config.use_fifo_scheduler = true;
    config.fifo_priority = priority;
    configureRealtime(config, "legacy");
}

} // namespace rl_master::runtime
