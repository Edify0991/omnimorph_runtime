#ifndef RL_MASTER_KINEMATICS_ROBOT_KINEMATICS_ADAPTER_H
#define RL_MASTER_KINEMATICS_ROBOT_KINEMATICS_ADAPTER_H

#include <memory>
#include <string>
#include <vector>

#include "rl_master/kinematics/joint_data.h"
#include "rl_master/rl_cfg.h"

namespace rl_master::kinematics
{

class RobotKinematicsAdapter
{
public:
    virtual ~RobotKinematicsAdapter() = default;

    virtual std::string adapterId() const = 0;

    virtual void configure(
        const std::vector<std::string> &global_joint_order,
        const std::vector<std::string> &global_motor_order,
        const JointGroupsConfig &joint_groups) = 0;

    virtual std::vector<JointData> motorToJoint(
        const std::vector<JointData> &motor_state,
        const std::vector<JointData> &direct_joint_state) = 0;

    virtual std::vector<JointData> jointToMotor(
        const std::vector<JointData> &joint_state,
        const std::vector<JointData> &joint_cmd,
        const std::vector<JointData> &direct_motor_cmd) = 0;
};

std::unique_ptr<RobotKinematicsAdapter> createRobotKinematicsAdapter(
    const std::string &adapter_id);

} // namespace rl_master::kinematics

#endif // RL_MASTER_KINEMATICS_ROBOT_KINEMATICS_ADAPTER_H
