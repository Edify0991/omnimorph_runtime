#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "math_tool.h"
#include "rl_cfg.h"
#include "rl_protocol.h"

class RobotState
{
public:
    RobotState() = default;
    ~RobotState() = default;

    static std::unique_ptr<RobotState> create();

    void initialize_buffers();

    std::vector<float> joint_q;
    std::vector<float> joint_dq;
    std::vector<float> joint_tau;

    std::vector<float> joint_target_q;
    std::vector<float> joint_target_dq;
    std::vector<float> joint_target_tau;

    std::vector<float> base_ang_vel;
    std::vector<float> base_rpy;
    std::vector<float> base_quat;

    std::vector<float> default_angle_walk;
    std::vector<float> default_angle_stand;
    std::vector<float> default_angle;

    Sim2realCfg sim2realCfg;
    Sim2realCfg standSim2RealCfg;

    float open_rl = rl_master::kOpenRlDisabled;

private:
    void load_default_angles(const std::vector<std::pair<std::string, float>> &source, std::vector<float> *target) const;
    static const std::array<std::string, rl_master::kLegJointCount> &joint_name_order();
};

#endif // ROBOT_STATE_H
