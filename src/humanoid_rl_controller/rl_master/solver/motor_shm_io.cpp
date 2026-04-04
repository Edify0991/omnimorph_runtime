#include "rl_master/solver/motor_shm_io.h"

#include <iostream>
#include <stdexcept>

#include <SharedMemory.hpp>

namespace rl_master::solver
{
namespace
{
constexpr const char *kShmTargetPath = "/home/jc_robot/target_handle";
constexpr const char *kShmTargetSemName = "sem_target_handle";
constexpr int kShmTargetKeyNum = 0x01;

constexpr const char *kShmFeedbackPath = "/home/jc_robot/feedback_handle";
constexpr const char *kShmFeedbackSemName = "sem_feedback_handle";
constexpr int kShmFeedbackKeyNum = 0x02;
}

MotorShmIo::MotorShmIo() = default;
MotorShmIo::~MotorShmIo() = default;

void MotorShmIo::connect()
{
    shm_target_ = std::make_unique<SharedMemory>(
        kShmTargetPath,
        sizeof(MotorHandle) * kMotorCountMax,
        kShmTargetKeyNum,
        LOCK_TYPE_MUTEX,
        kShmTargetSemName);

    shm_feedback_ = std::make_unique<SharedMemory>(
        kShmFeedbackPath,
        sizeof(MotorHandle) * kMotorCountMax,
        kShmFeedbackKeyNum,
        LOCK_TYPE_MUTEX,
        kShmFeedbackSemName);

    shm_target_->connect();
    shm_feedback_->connect();
}

void MotorShmIo::readFeedback(std::array<MotorHandle, kMotorCountMax> *feedback)
{
    if (!feedback)
    {
        return;
    }

    if (!shm_feedback_)
    {
        throw std::runtime_error("MotorShmIo::readFeedback called before connect().");
    }

    shm_feedback_->read(feedback->data(), static_cast<int>(kMotorCountMax), 0);
}

void MotorShmIo::writeTarget(const std::array<MotorHandle, kMotorCountMax> &target)
{
    if (!shm_target_)
    {
        throw std::runtime_error("MotorShmIo::writeTarget called before connect().");
    }

    shm_target_->write(target.data(), static_cast<int>(kMotorCountMax), 0);
}

} // namespace rl_master::solver
