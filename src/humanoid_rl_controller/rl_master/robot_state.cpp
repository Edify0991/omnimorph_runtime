#include "rl_master/robot_state.h"

#include <unordered_map>

std::unique_ptr<RobotState> RobotState::create()
{
    return std::unique_ptr<RobotState>(new RobotState());
}

const std::array<std::string, rl_master::kLegJointCount> &RobotState::joint_name_order()
{
    static const std::array<std::string, rl_master::kLegJointCount> kJointOrder = {
        "right_hip_roll",
        "right_hip_yaw",
        "right_hip_pitch",
        "right_knee_pitch",
        "right_ankle_pitch",
        "right_ankle_roll",
        "left_hip_roll",
        "left_hip_yaw",
        "left_hip_pitch",
        "left_knee_pitch",
        "left_ankle_pitch",
        "left_ankle_roll"};
    return kJointOrder;
}

void RobotState::load_default_angles(const std::vector<std::pair<std::string, float>> &source, std::vector<float> *target) const
{
    if (!target)
    {
        return;
    }

    target->assign(rl_master::kLegJointCount, 0.0f);
    std::unordered_map<std::string, float> source_map;
    source_map.reserve(source.size());
    for (const auto &entry : source)
    {
        source_map[entry.first] = entry.second;
    }

    const auto &order = joint_name_order();
    for (size_t i = 0; i < order.size(); ++i)
    {
        const auto it = source_map.find(order[i]);
        if (it != source_map.end())
        {
            (*target)[i] = it->second;
        }
        else
        {
            std::cerr << "[RobotState] Missing default joint angle for " << order[i] << std::endl;
        }
    }
}

void RobotState::initialize_buffers()
{
    joint_q.assign(rl_master::kLegJointCount, 0.0f);
    joint_dq.assign(rl_master::kLegJointCount, 0.0f);
    joint_tau.assign(rl_master::kLegJointCount, 0.0f);
    joint_target_q.assign(rl_master::kLegJointCount, 0.0f);
    joint_target_dq.assign(rl_master::kLegJointCount, 0.0f);
    joint_target_tau.assign(rl_master::kLegJointCount, 0.0f);

    base_ang_vel.assign(3, 0.0f);
    base_quat.assign(4, 0.0f);
    base_quat[3] = 1.0f;
    base_rpy.assign(3, 0.0f);
    base_rpy = quaternion_to_euler_array(base_quat);

    default_angle_walk.assign(rl_master::kLegJointCount, 0.0f);
    default_angle_stand.assign(rl_master::kLegJointCount, 0.0f);
    default_angle.assign(rl_master::kLegJointCount, 0.0f);

    default_angle = default_angle_walk;
}
