#ifndef RL_MASTER_KINEMATICS_ADAPTERS_JC05_KINEMATICS_ADAPTER_H
#define RL_MASTER_KINEMATICS_ADAPTERS_JC05_KINEMATICS_ADAPTER_H

#include <vector>

#include "rl_master/KinConv.h"

namespace rl_master
{

class Jc05KinematicsAdapter
{
public:
    // Placeholder only: current behavior is pass-through until jc05 linkage geometry is defined.
    std::vector<JointData> jointToMotor(const std::vector<JointData> &joint_cmd) const { return joint_cmd; }
    std::vector<JointData> motorToJoint(const std::vector<JointData> &motor_state) const { return motor_state; }
};

} // namespace rl_master

#endif
