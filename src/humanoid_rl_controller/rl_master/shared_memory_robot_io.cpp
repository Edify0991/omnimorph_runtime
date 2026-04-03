#include "rl_master/shared_memory_robot_io.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rl_master/deploy_state_machine.h"
#include "rl_master/rl_protocol.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

void SharedMemoryRobotIO::connect()
{
    imu_shm_ = std::make_unique<SharedMemory>(
        shared_data_config_.FILE_PATH,
        shared_data_config_.length[6],
        shared_data_config_.keyNum[6],
        shared_data_config_.semFlag,
        shared_data_config_.semName[6]);

    joint_cmd_shm_ = std::make_unique<SharedMemory>(
        shared_data_config_.FILE_PATH,
        shared_data_config_.length[8],
        shared_data_config_.keyNum[8],
        shared_data_config_.semFlag,
        shared_data_config_.semName[8]);

    joint_state_shm_ = std::make_unique<SharedMemory>(
        shared_data_config_.FILE_PATH,
        shared_data_config_.length[9],
        shared_data_config_.keyNum[9],
        shared_data_config_.semFlag,
        shared_data_config_.semName[9]);

    robot_cmd_shm_ = std::make_unique<SharedMemory>(
        shared_data_config_.FILE_PATH,
        shared_data_config_.length[2],
        shared_data_config_.keyNum[2],
        shared_data_config_.semFlag,
        shared_data_config_.semName[2]);

    walk_mode_shm_ = std::make_unique<SharedMemory>(
        shared_data_config_.FILE_PATH,
        shared_data_config_.length[12],
        shared_data_config_.keyNum[12],
        shared_data_config_.semFlag,
        shared_data_config_.semName[12]);

    try
    {
        imu_shm_->connect();
        joint_cmd_shm_->connect();
        joint_state_shm_->connect();
        robot_cmd_shm_->connect();
        walk_mode_shm_->connect();
    }
    catch (const std::exception &e)
    {
        throw std::runtime_error(std::string("[SharedMemoryRobotIO] connect() failed: ") + e.what());
    }

    connected_ = true;
}

bool SharedMemoryRobotIO::read_state(rl_master::RobotStateData &state)
{
    if (!connected_)
    {
        return false;
    }

    joint_state_shm_->read(joint_state_buf_.data(), rl_master::kJointStateValueCount, 0);
    for (int i = 0; i < rl_master::kLegJointCount; ++i)
    {
        state.joint_q[static_cast<size_t>(i)] = joint_state_buf_[static_cast<size_t>(3 * i)];
        state.joint_dq[static_cast<size_t>(i)] = joint_state_buf_[static_cast<size_t>(3 * i + 1)];
        state.joint_tau[static_cast<size_t>(i)] = joint_state_buf_[static_cast<size_t>(3 * i + 2)];
    }

    imu_shm_->read(imu_buf_.data(), static_cast<int>(imu_buf_.size()), 0);
    // IMU order in SHM: [qw, qx, qy, qz, roll, pitch, yaw, wx, wy, wz]
    state.base_quat[0] = imu_buf_[1];
    state.base_quat[1] = imu_buf_[2];
    state.base_quat[2] = imu_buf_[3];
    state.base_quat[3] = imu_buf_[0];

    const float quat_norm = std::sqrt(
        state.base_quat[0] * state.base_quat[0] +
        state.base_quat[1] * state.base_quat[1] +
        state.base_quat[2] * state.base_quat[2] +
        state.base_quat[3] * state.base_quat[3]);
    if (!std::isfinite(quat_norm) || quat_norm < 1e-6f)
    {
        state.base_quat = {0.0f, 0.0f, 0.0f, 1.0f};
    }

    for (int i = 0; i < 3; ++i)
    {
        state.base_ang_vel[static_cast<size_t>(i)] = imu_buf_[static_cast<size_t>(7 + i)];
    }
    state.base_rpy = quat_to_rpy(state.base_quat);
    last_state_ = state;
    return true;
}

bool SharedMemoryRobotIO::read_control_command(rl_master::TeleopCommand &command)
{
    if (!connected_)
    {
        return false;
    }
    robot_cmd_shm_->read(robot_cmd_buf_.data(), static_cast<int>(robot_cmd_buf_.size()), 0);
    command.vx = robot_cmd_buf_[0];
    command.vy = robot_cmd_buf_[1];
    command.dyaw = robot_cmd_buf_[2];
    return true;
}

int SharedMemoryRobotIO::read_walk_mode(int fallback_mode)
{
    if (!connected_)
    {
        return fallback_mode;
    }

    walk_mode_shm_->read(walk_mode_buf_.data(), 1, 0);
    const int mode = walk_mode_buf_[0];
    if (!valid_walk_mode(mode))
    {
        return fallback_mode;
    }
    return mode;
}

bool SharedMemoryRobotIO::write_command(const rl_master::RobotCommandData &command)
{
    if (!connected_)
    {
        return false;
    }

    for (int i = 0; i < rl_master::kLegJointCount; ++i)
    {
        const size_t offset = static_cast<size_t>(3 * i);
        joint_cmd_buf_[offset] = command.joint_target_q[static_cast<size_t>(i)];
        joint_cmd_buf_[offset + 1] = command.joint_target_dq[static_cast<size_t>(i)];
        joint_cmd_buf_[offset + 2] = command.joint_target_tau[static_cast<size_t>(i)];
    }
    joint_cmd_buf_[rl_master::kJointStateValueCount] = command.open_rl;
    joint_cmd_buf_[rl_master::kJointCmdSeqIndex] = static_cast<float>(++cmd_sequence_);
    joint_cmd_buf_[rl_master::kJointCmdStampIndex] = static_cast<float>(rl_master::monotonicTimeSec());

    joint_cmd_shm_->write(joint_cmd_buf_.data(), rl_master::kJointCmdValueCount, 0);
    return true;
}

void SharedMemoryRobotIO::estop()
{
    rl_master::RobotCommandData stop_cmd;
    stop_cmd.open_rl = rl_master::kOpenRlDisabled;
    stop_cmd.joint_target_q = last_state_.joint_q;
    stop_cmd.joint_target_dq.fill(0.0f);
    stop_cmd.joint_target_tau.fill(0.0f);
    (void)write_command(stop_cmd);
}

std::array<float, 3> SharedMemoryRobotIO::quat_to_rpy(const std::array<float, 4> &quat_xyzw)
{
    const double x = quat_xyzw[0];
    const double y = quat_xyzw[1];
    const double z = quat_xyzw[2];
    const double w = quat_xyzw[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (w * y - z * x);
    const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(kPi / 2.0, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw)};
}

bool SharedMemoryRobotIO::valid_walk_mode(int mode)
{
    return mode == rl_master::kWalkModeCode ||
           mode == rl_master::kStandModeCode ||
           mode == rl_master::kFixStandModeCode ||
           mode == rl_master::kCtrlWordStartPolicy ||
           mode == rl_master::kCtrlWordStopPolicy ||
           mode == rl_master::kCtrlWordZeroing ||
           mode == rl_master::kCtrlWordEstop ||
           mode == rl_master::kCtrlWordStartWalk ||
           mode == rl_master::kCtrlWordStartStand ||
           mode == rl_master::kCtrlWordStartFixStand;
}
