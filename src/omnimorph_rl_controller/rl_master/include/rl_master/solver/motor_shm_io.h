#ifndef RL_MASTER_SOLVER_MOTOR_SHM_IO_H
#define RL_MASTER_SOLVER_MOTOR_SHM_IO_H

#include <array>
#include <memory>
#include <string>

#include "rl_master/solver/motor_io_backend.h"

class SharedMemory;

namespace rl_master::solver
{

class ShmMotorIoBackend final : public MotorIoBackend
{
public:
    ShmMotorIoBackend();
    ~ShmMotorIoBackend() override;

    std::string backendId() const override;
    void connect() override;
    void writePdGains(size_t motor_index, MotorHandle *target, const JointData &joint_cmd) override;

    void readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback) override;
    void writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target) override;

private:
    std::unique_ptr<SharedMemory> shm_target_;
    std::unique_ptr<SharedMemory> shm_feedback_;
};

using MotorShmIo = ShmMotorIoBackend;

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_MOTOR_SHM_IO_H
