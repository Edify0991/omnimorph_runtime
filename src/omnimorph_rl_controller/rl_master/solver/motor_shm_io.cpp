#include "rl_master/solver/motor_shm_io.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <SharedMemory.hpp>

namespace rl_master::solver
{

ShmMotorIoBackend::ShmMotorIoBackend() = default;
ShmMotorIoBackend::~ShmMotorIoBackend() = default;

std::string ShmMotorIoBackend::backendId() const
{
    return "shm";
}

namespace
{
uint32_t encodeShmPdGain(float raw_gain)
{
    if (!std::isfinite(raw_gain) || raw_gain <= 0.0f)
    {
        return 0U;
    }
    const float clamped = std::min(raw_gain, static_cast<float>(std::numeric_limits<uint32_t>::max()));
    return static_cast<uint32_t>(clamped + 0.5f);
}
} // namespace

void ShmMotorIoBackend::writePdGains(MotorHandle *target, const JointData &joint_cmd) const
{
    if (!target)
    {
        return;
    }
    if (joint_cmd.mode == RUN_MODE_R1 || joint_cmd.mode == RUN_MODE_CSP)
    {
        target->pd_uint[0] = encodeShmPdGain(joint_cmd.kp);
        target->pd_uint[1] = encodeShmPdGain(joint_cmd.kd);
        return;
    }
    target->pd_uint[0] = 0U;
    target->pd_uint[1] = 0U;
}

void ShmMotorIoBackend::connect()
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

void ShmMotorIoBackend::readFeedback(std::array<MotorHandle, kMotorShmSlotCount> *feedback)
{
    if (!feedback)
    {
        return;
    }

    if (!shm_feedback_)
    {
        throw std::runtime_error("ShmMotorIoBackend::readFeedback called before connect().");
    }

    shm_feedback_->read(feedback->data(), static_cast<int>(kMotorShmSlotCount), 0);
}

void ShmMotorIoBackend::writeTarget(const std::array<MotorHandle, kMotorShmSlotCount> &target)
{
    if (!shm_target_)
    {
        throw std::runtime_error("ShmMotorIoBackend::writeTarget called before connect().");
    }

    shm_target_->write(target.data(), static_cast<int>(kMotorShmSlotCount), 0);
}

} // namespace rl_master::solver
