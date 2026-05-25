#include "rl_master/solver/motor_io_backend.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

#include "rl_master/solver/motor_shm_io.h"

namespace rl_master::solver
{
namespace
{

std::string normalizeBackendId(std::string backend_id)
{
    std::transform(
        backend_id.begin(),
        backend_id.end(),
        backend_id.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return backend_id;
}

} // namespace

#ifdef RL_MASTER_HAS_UNITREE_HG
std::unique_ptr<MotorIoBackend> createUnitreeG1DdsMotorIoBackend();
#endif
#ifdef RL_MASTER_HAS_UNITREE_SDK2
std::unique_ptr<MotorIoBackend> createUnitreeG1Sdk2MotorIoBackend();
#endif

std::unique_ptr<MotorIoBackend> createMotorIoBackend(const std::string &backend_id)
{
    const std::string id = normalizeBackendId(backend_id);
    if (id.empty() || id == "shm" || id == "shared_memory" ||
        id == "jc01_shm_bridge" || id == "jc05_vendor_bridge")
    {
        return std::make_unique<ShmMotorIoBackend>();
    }

    if (id == "unitree_g1_sdk2" || id == "unitree_sdk2" || id == "unitree_sdk_bridge")
    {
#ifdef RL_MASTER_HAS_UNITREE_SDK2
        return createUnitreeG1Sdk2MotorIoBackend();
#else
        throw std::runtime_error(
            "motor_io_backend '" + backend_id +
            "' requires unitree_sdk2. Set UNITREE_SDK2_ROOT and rebuild rl_master "
            "with Unitree SDK2 available.");
#endif
    }

    if (id == "unitree_g1_dds" || id == "unitree_dds")
    {
#ifdef RL_MASTER_HAS_UNITREE_HG
        return createUnitreeG1DdsMotorIoBackend();
#else
        throw std::runtime_error(
            "motor_io_backend '" + backend_id +
            "' requires unitree_hg. Source the official Unitree ROS 2 workspace "
            "and rebuild rl_master with unitree_hg available.");
#endif
    }

    std::ostringstream oss;
    oss << "unsupported motor_io_backend '" << backend_id
        << "'. Supported backends: shm, unitree_g1_sdk2, unitree_g1_dds";
    throw std::runtime_error(oss.str());
}

} // namespace rl_master::solver
