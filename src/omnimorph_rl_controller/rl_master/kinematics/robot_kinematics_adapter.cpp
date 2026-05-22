#include "rl_master/kinematics/robot_kinematics_adapter.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "rl_master/kinematics/jc01/kin_conv.h"

namespace rl_master::kinematics
{
namespace
{

std::string normalizeAdapterId(std::string adapter_id)
{
    std::transform(
        adapter_id.begin(),
        adapter_id.end(),
        adapter_id.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return adapter_id;
}

std::vector<JointData> extractGroup(
    const std::vector<JointData> &full,
    const std::vector<int> &indices)
{
    std::vector<JointData> out;
    out.reserve(indices.size());
    for (const int index : indices)
    {
        if (index < 0 || static_cast<size_t>(index) >= full.size())
        {
            throw std::runtime_error("kinematics adapter group index out of range during extraction");
        }
        out.push_back(full[static_cast<size_t>(index)]);
    }
    return out;
}

void scatterGroup(
    const std::vector<JointData> &group,
    const std::vector<int> &indices,
    std::vector<JointData> *full)
{
    if (!full)
    {
        return;
    }
    if (group.size() != indices.size())
    {
        throw std::runtime_error("kinematics adapter group scatter size mismatch");
    }
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const int index = indices[i];
        if (index < 0 || static_cast<size_t>(index) >= full->size())
        {
            throw std::runtime_error("kinematics adapter group index out of range during scatter");
        }
        (*full)[static_cast<size_t>(index)] = group[i];
    }
}

std::vector<int> resolveMotorGroup(
    const std::vector<std::string> &group_names,
    const std::vector<std::string> &global_motor_order,
    const char *group_name)
{
    std::unordered_map<std::string, int> motor_index;
    motor_index.reserve(global_motor_order.size());
    for (size_t i = 0; i < global_motor_order.size(); ++i)
    {
        motor_index[global_motor_order[i]] = static_cast<int>(i);
    }

    std::vector<int> indices;
    indices.reserve(group_names.size());
    for (const auto &joint_name : group_names)
    {
        const auto it = motor_index.find(joint_name);
        if (it == motor_index.end())
        {
            throw std::runtime_error(
                std::string("motor order missing joint from kinematics group '") +
                group_name + "': " + joint_name);
        }
        indices.push_back(it->second);
    }
    return indices;
}

class DirectMappingKinematicsAdapter : public RobotKinematicsAdapter
{
public:
    explicit DirectMappingKinematicsAdapter(std::string adapter_id)
        : adapter_id_(std::move(adapter_id))
    {
    }

    std::string adapterId() const override
    {
        return adapter_id_;
    }

    void configure(
        const std::vector<std::string> &,
        const std::vector<std::string> &,
        const JointGroupsConfig &) override
    {
    }

    std::vector<JointData> motorToJoint(
        const std::vector<JointData> &,
        const std::vector<JointData> &direct_joint_state) override
    {
        return direct_joint_state;
    }

    std::vector<JointData> jointToMotor(
        const std::vector<JointData> &,
        const std::vector<JointData> &,
        const std::vector<JointData> &direct_motor_cmd) override
    {
        return direct_motor_cmd;
    }

private:
    std::string adapter_id_;
};

class Jc01KinematicsAdapter : public RobotKinematicsAdapter
{
public:
    std::string adapterId() const override
    {
        return "jc01";
    }

    void configure(
        const std::vector<std::string> &global_joint_order,
        const std::vector<std::string> &global_motor_order,
        const JointGroupsConfig &joint_groups) override
    {
        kin_conv_.configureJointGroups(global_joint_order, joint_groups);
        leg_motor_indices_ = resolveMotorGroup(joint_groups.leg, global_motor_order, "leg");
        arm_motor_indices_ = resolveMotorGroup(joint_groups.arm, global_motor_order, "arm");
        waist_motor_indices_ = resolveMotorGroup(joint_groups.waist, global_motor_order, "waist");
    }

    std::vector<JointData> motorToJoint(
        const std::vector<JointData> &motor_state,
        const std::vector<JointData> &direct_joint_state) override
    {
        std::vector<JointData> joint_state = direct_joint_state;

        const auto &leg_indices = kin_conv_.legGlobalIndices();
        if (!leg_indices.empty())
        {
            scatterGroup(
                kin_conv_.legMotorToJoint(extractGroup(motor_state, leg_motor_indices_)),
                leg_indices,
                &joint_state);
        }
        const auto &arm_indices = kin_conv_.armGlobalIndices();
        if (!arm_indices.empty())
        {
            scatterGroup(
                kin_conv_.armMotorToJoint(extractGroup(motor_state, arm_motor_indices_)),
                arm_indices,
                &joint_state);
        }
        const auto &waist_indices = kin_conv_.waistGlobalIndices();
        if (!waist_indices.empty())
        {
            scatterGroup(
                kin_conv_.waistMotorToJoint(extractGroup(motor_state, waist_motor_indices_)),
                waist_indices,
                &joint_state);
        }

        return joint_state;
    }

    std::vector<JointData> jointToMotor(
        const std::vector<JointData> &joint_state,
        const std::vector<JointData> &joint_cmd,
        const std::vector<JointData> &direct_motor_cmd) override
    {
        std::vector<JointData> motor_cmd = direct_motor_cmd;

        const auto &leg_indices = kin_conv_.legGlobalIndices();
        if (!leg_indices.empty())
        {
            scatterGroup(
                kin_conv_.legJointToMotor(
                    extractGroup(joint_state, leg_indices),
                    extractGroup(joint_cmd, leg_indices)),
                leg_motor_indices_,
                &motor_cmd);
        }
        const auto &arm_indices = kin_conv_.armGlobalIndices();
        if (!arm_indices.empty())
        {
            scatterGroup(
                kin_conv_.armJointToMotor(
                    extractGroup(joint_state, arm_indices),
                    extractGroup(joint_cmd, arm_indices)),
                arm_motor_indices_,
                &motor_cmd);
        }
        const auto &waist_indices = kin_conv_.waistGlobalIndices();
        if (!waist_indices.empty())
        {
            scatterGroup(
                kin_conv_.waistJointToMotor(
                    extractGroup(joint_state, waist_indices),
                    extractGroup(joint_cmd, waist_indices)),
                waist_motor_indices_,
                &motor_cmd);
        }

        return motor_cmd;
    }

private:
    KinConv kin_conv_;
    std::vector<int> leg_motor_indices_;
    std::vector<int> arm_motor_indices_;
    std::vector<int> waist_motor_indices_;
};

} // namespace

std::unique_ptr<RobotKinematicsAdapter> createRobotKinematicsAdapter(
    const std::string &adapter_id)
{
    const std::string id = normalizeAdapterId(adapter_id);
    if (id.empty() || id == "identity" || id == "direct" || id == "passthrough" || id == "none")
    {
        return std::make_unique<DirectMappingKinematicsAdapter>("identity");
    }
    if (id == "jc01" || id == "jingchu01")
    {
        return std::make_unique<Jc01KinematicsAdapter>();
    }
    if (id == "unitree_g1" || id == "g1")
    {
        return std::make_unique<DirectMappingKinematicsAdapter>("unitree_g1");
    }
    if (id == "jc05" || id == "jc05_quadruped")
    {
        return std::make_unique<DirectMappingKinematicsAdapter>("jc05_quadruped");
    }

    std::ostringstream oss;
    oss << "unsupported robot_kinematics_adapter '" << adapter_id
        << "'. Supported adapters: jc01, unitree_g1, jc05_quadruped, identity";
    throw std::runtime_error(oss.str());
}

} // namespace rl_master::kinematics
