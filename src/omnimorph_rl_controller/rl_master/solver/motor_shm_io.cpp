#include "rl_master/solver/motor_shm_io.h"

#include <iostream>
#include <stdexcept>

#include <SharedMemory.hpp>

namespace rl_master::solver
{

MotorShmIo::MotorShmIo() = default;
MotorShmIo::~MotorShmIo() = default;

void MotorShmIo::connect()
{
    shm_target_ = std::make_unique<SharedMemory>(
        rl_master::hardware::kMotorTargetShmPath,
        sizeof(MotorHandle) * kMotorShmSlotCount,
        rl_master::hardware::kMotorTargetShmKeyNum,
        LOCK_TYPE_MUTEX,
        rl_master::hardware::kMotorTargetShmSemName);

    shm_feedback_ = std::make_unique<SharedMemory>(
        rl_master::hardware::kMotorFeedbackShmPath,
        sizeof(MotorHandle) * kMotorShmSlotCount,
        rl_master::hardware::kMotorFeedbackShmKeyNum,
        LOCK_TYPE_MUTEX,
        rl_master::hardware::kMotorFeedbackShmSemName);

    shm_target_->connect();
    shm_feedback_->connect();
}

void MotorShmIo::readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback)
{
    if (!feedback)
    {
        return;
    }

    if (!shm_feedback_)
    {
        throw std::runtime_error("MotorShmIo::readFeedback called before connect().");
    }

    shm_feedback_->read(feedback->data(), static_cast<int>(kMotorShmSlotCount), 0);
}

void MotorShmIo::writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target)
{
    if (!shm_target_)
    {
        throw std::runtime_error("MotorShmIo::writeTarget called before connect().");
    }

    shm_target_->write(target.data(), static_cast<int>(kMotorShmSlotCount), 0);
}

} // namespace rl_master::solver
