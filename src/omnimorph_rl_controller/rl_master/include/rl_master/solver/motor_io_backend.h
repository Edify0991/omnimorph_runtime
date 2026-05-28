#ifndef RL_MASTER_SOLVER_MOTOR_IO_BACKEND_H
#define RL_MASTER_SOLVER_MOTOR_IO_BACKEND_H

#include <array>
#include <memory>
#include <string>

#include "rl_master/hardware/motor_shm_contract.h"
#include "rl_master/kinematics/joint_data.h"

struct SourceContract;

namespace rl_master::solver
{

using rl_master::hardware::kMotorShmSlotCount;
using rl_master::hardware::MotorHandle;

class MotorIoBackend
{
public:
    virtual ~MotorIoBackend() = default;

    virtual std::string backendId() const = 0;
    virtual void updateSourceContract(const SourceContract &source_contract)
    {
        (void)source_contract;
    }
    virtual void writePdGains(MotorHandle *target, const JointData &joint_cmd) const
    {
        if (!target)
        {
            return;
        }
        if (joint_cmd.mode == RUN_MODE_R1)
        {
            target->pd[0] = joint_cmd.kp;
            target->pd[1] = joint_cmd.kd;
            return;
        }
        target->pd[0] = 0.0f;
        target->pd[1] = 0.0f;
    }
    virtual void connect() = 0;
    virtual void readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback) = 0;
    virtual void writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target) = 0;
};

std::unique_ptr<MotorIoBackend> createMotorIoBackend(const std::string &backend_id);

} // namespace rl_master::solver

#endif // RL_MASTER_SOLVER_MOTOR_IO_BACKEND_H
