#include "rl_master/robot_state.h"

#include <algorithm>

std::unique_ptr<RobotState> RobotState::create()
{
    return std::unique_ptr<RobotState>(new RobotState());
}

void RobotState::initialize_buffers(size_t joint_count, const std::vector<float> &default_angles)
{
    joint_q.assign(joint_count, 0.0f);
    joint_dq.assign(joint_count, 0.0f);
    joint_tau.assign(joint_count, 0.0f);
    joint_target_q.assign(joint_count, 0.0f);
    joint_target_dq.assign(joint_count, 0.0f);
    joint_target_tau.assign(joint_count, 0.0f);

    base_pos_w.assign(3, 0.0f);
    base_lin_vel.assign(3, 0.0f);
    base_ang_vel.assign(3, 0.0f);
    base_quat.assign(4, 0.0f);
    base_quat[3] = 1.0f;
    base_rpy.assign(3, 0.0f);
    base_rpy = quaternion_to_euler_array(base_quat);

    default_angle.assign(joint_count, 0.0f);
    const size_t copy_n = std::min(default_angles.size(), joint_count);
    for (size_t i = 0; i < copy_n; ++i)
    {
        default_angle[i] = default_angles[i];
    }
}
