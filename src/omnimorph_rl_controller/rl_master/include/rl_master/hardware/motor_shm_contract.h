#ifndef RL_MASTER_HARDWARE_MOTOR_SHM_CONTRACT_H
#define RL_MASTER_HARDWARE_MOTOR_SHM_CONTRACT_H

#include <cstddef>
#include <cstdint>

namespace rl_master::hardware
{

constexpr size_t kMotorShmSlotCount = 30;

constexpr const char *kMotorTargetShmPath = "/home/jc_robot/target_handle";
constexpr const char *kMotorTargetShmSemName = "sem_target_handle";
constexpr int kMotorTargetShmKeyNum = 0x01;

constexpr const char *kMotorFeedbackShmPath = "/home/jc_robot/feedback_handle";
constexpr const char *kMotorFeedbackShmSemName = "sem_feedback_handle";
constexpr int kMotorFeedbackShmKeyNum = 0x02;

struct MotorHandle
{
    uint8_t run_mode = 0;
    uint8_t motor_type = 0;
    float pd[2] = {0.0f, 0.0f};

    union
    {
        struct
        {
            float target_speed;
            float target_pos;
            float target_torque;
        } target;

        struct
        {
            float feedback_speed;
            float feedback_pos;
            float feedback_torque;
        } feedback;
    } io{};
};

} // namespace rl_master::hardware

#endif // RL_MASTER_HARDWARE_MOTOR_SHM_CONTRACT_H
