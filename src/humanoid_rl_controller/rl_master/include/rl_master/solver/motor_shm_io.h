#ifndef RL_MASTER_SOLVER_MOTOR_SHM_IO_H
#define RL_MASTER_SOLVER_MOTOR_SHM_IO_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>

class SharedMemory;

namespace rl_master::solver
{

constexpr size_t kMotorShmSlotCount = 30;

struct MotorHandle
{
    uint8_t run_mode = 0;
    uint8_t motor_type = 0;
    uint8_t pd[2] = {0, 0};

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
