#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <memory>
#include <string>
#include <vector>
#include "math_tool.h"
#include "rl_protocol.h"

class RobotState
{
public:
    RobotState() = default;
    ~RobotState() = default;

    static std::unique_ptr<RobotState> create();

    void initialize_buffers(size_t joint_count, const std::vector<float> &default_angles = {});

    std::vector<float> joint_q;
    std::vector<float> joint_dq;
    std::vector<float> joint_tau;

    std::vector<float> joint_target_q;
    std::vector<float> joint_target_dq;
    std::vector<float> joint_target_tau;

    std::vector<float> base_pos_w;
    std::vector<float> base_lin_vel;
    std::vector<float> base_ang_vel;
    std::vector<float> base_rpy;
    std::vector<float> base_quat;

    std::vector<float> default_angle;
    std::vector<std::string> joint_names;

    float open_rl = rl_master::kOpenRlDisabled;
};

#endif // ROBOT_STATE_H
