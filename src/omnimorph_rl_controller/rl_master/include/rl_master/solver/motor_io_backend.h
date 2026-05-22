#ifndef RL_MASTER_SOLVER_MOTOR_IO_BACKEND_H
#define RL_MASTER_SOLVER_MOTOR_IO_BACKEND_H

#include <array>
#include <memory>
#include <string>

#include "rl_master/hardware/motor_shm_contract.h"

namespace rl_master::solver
{

using rl_master::hardware::kMotorShmSlotCount;
using rl_master::hardware::MotorHandle;

class MotorIoBackend
{
public:
    virtual ~MotorIoBackend() = default;

    virtual std::string backendId() const = 0;
    virtual void connect() = 0;
    virtual void readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback) = 0;
    virtual void writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target) = 0;
};

std::unique_ptr<MotorIoBackend> createMotorIoBackend(const std::string &backend_id);

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_MOTOR_IO_BACKEND_H
