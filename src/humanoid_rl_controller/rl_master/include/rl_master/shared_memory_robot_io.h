#ifndef RL_MASTER_SHARED_MEMORY_ROBOT_IO_H
#define RL_MASTER_SHARED_MEMORY_ROBOT_IO_H

#include <array>
#include <cstdint>
#include <memory>

#include <SharedMemory.hpp>
#include <SharedMemoryDataType.hpp>

#include "robot_io.h"

class SharedMemoryRobotIO final : public RobotIO
{
public:
    SharedMemoryRobotIO() = default;
    ~SharedMemoryRobotIO() override = default;

    void connect() override;

    bool read_state(rl_master::RobotStateData &state) override;
    bool read_control_command(rl_master::TeleopCommand &command) override;
    int read_walk_mode(int fallback_mode) override;

    bool write_command(const rl_master::RobotCommandData &command) override;

    void estop() override;

private:
    static std::array<float, 3> quat_to_rpy(const std::array<float, 4> &quat_xyzw);
    static bool valid_walk_mode(int mode);

    SharedDataConfig shared_data_config_;
    std::unique_ptr<SharedMemory> imu_shm_;
    std::unique_ptr<SharedMemory> joint_cmd_shm_;
    std::unique_ptr<SharedMemory> joint_state_shm_;
    std::unique_ptr<SharedMemory> robot_cmd_shm_;
    std::unique_ptr<SharedMemory> walk_mode_shm_;

    std::array<float, rl_master::kJointStateValueCount> joint_state_buf_{};
    std::array<float, rl_master::kJointCmdValueCount> joint_cmd_buf_{};
    std::array<float, 10> imu_buf_{};
    std::array<float, 3> robot_cmd_buf_{};
    std::array<int, 1> walk_mode_buf_{0};

    rl_master::RobotStateData last_state_{};
    bool connected_ = false;
    uint32_t cmd_sequence_ = 0;
};

#endif // RL_MASTER_SHARED_MEMORY_ROBOT_IO_H
