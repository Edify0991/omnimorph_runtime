#ifndef RL_MASTER_SOLVER_MOTOR_SHM_IO_H
#define RL_MASTER_SOLVER_MOTOR_SHM_IO_H

#include <array>
#include <memory>
#include <string>

#include "rl_master/hardware/motor_shm_contract.h"

class SharedMemory;

namespace rl_master::solver
{

using rl_master::hardware::kMotorShmSlotCount;
using rl_master::hardware::MotorHandle;

class MotorShmIo
{
public:
    MotorShmIo();
    ~MotorShmIo();

    void connect();

    void readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback);
    void writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target);

private:
    std::unique_ptr<SharedMemory> shm_target_;
    std::unique_ptr<SharedMemory> shm_feedback_;
};

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_MOTOR_SHM_IO_H
