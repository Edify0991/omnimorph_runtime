#include "rl_master/KinConv.h"

#include <unordered_map>

KinConv::KinConv() {}

KinConv::~KinConv() {}

void KinConv::configureJointGroups(
    const std::vector<std::string> &global_joint_order,
    const JointGroupsConfig &joint_groups)
{
    const std::vector<std::string> expected_leg_group = {
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
        "left_ankle_roll",
    };
    if (joint_groups.leg != expected_leg_group)
    {
        throw std::runtime_error(
            "joint_groups.leg must exactly match the current 12-joint leg conversion contract");
    }

    std::unordered_map<std::string, int> joint_index;
    joint_index.reserve(global_joint_order.size());
    for (size_t i = 0; i < global_joint_order.size(); ++i)
    {
        joint_index[global_joint_order[i]] = static_cast<int>(i);
    }

    auto resolveGroup = [&joint_index](const std::vector<std::string> &group_names, const char *group_name) {
        std::vector<int> indices;
        indices.reserve(group_names.size());
        for (const auto &joint_name : group_names)
        {
            const auto it = joint_index.find(joint_name);
            if (it == joint_index.end())
            {
                throw std::runtime_error(
                    std::string("KinConv group '") + group_name +
                    "' contains joint not present in global joint order: " + joint_name);
            }
            indices.push_back(it->second);
        }
        return indices;
    };

    leg_global_indices_ = resolveGroup(joint_groups.leg, "leg");
    arm_global_indices_ = resolveGroup(joint_groups.arm, "arm");
    waist_global_indices_ = resolveGroup(joint_groups.waist, "waist");
    validateLegGroupSize();
}

const std::vector<int> &KinConv::legGlobalIndices() const
{
    return leg_global_indices_;
}

const std::vector<int> &KinConv::armGlobalIndices() const
{
    return arm_global_indices_;
}

const std::vector<int> &KinConv::waistGlobalIndices() const
{
    return waist_global_indices_;
}

void KinConv::validateLegGroupSize() const
{
    if (leg_global_indices_.size() != LEG_MOTOR_COUNT)
    {
        throw std::runtime_error(
            "KinConv leg group must contain exactly " + std::to_string(LEG_MOTOR_COUNT) + " joints");
    }
}

std::vector<JointData> KinConv::legJointToMotor(const std::vector<JointData>& joint_state, const std::vector<JointData>& joint_cmd)
{
    if (joint_state.size() != LEG_MOTOR_COUNT || joint_cmd.size() != LEG_MOTOR_COUNT)
    {
        throw std::runtime_error("KinConv::legJointToMotor expects exactly 12 leg joints");
    }
    std::vector<JointData> motor_cmd(
        LEG_MOTOR_COUNT,
        {0.0f, 0.0f, 0.0f, RUN_MODE_CSP, 0.0f, 0.0f});

    for (size_t i = 0; i < LEG_MOTOR_COUNT; ++i) 
    {
        // std::cout << "app_mode[" << i << "]:" << app_mode[i] << std::endl;
        motor_cmd[i].dq = 0.0; //只有位控和力控
        if (joint_cmd[i].mode == RUN_MODE_CSP)
        {
            motor_cmd[i].tau = 0.0;
            if (JointName(i) == right_hip_roll || JointName(i) == right_hip_yaw || JointName(i) == right_hip_pitch ||
                JointName(i) == left_hip_roll || JointName(i) == left_hip_yaw || JointName(i) == left_hip_pitch)
            {
                motor_cmd[i].q = std::clamp<float>(JOINT_DIR[i] * joint_cmd[i].q, -MOTOR_POS_LIMIT[i], MOTOR_POS_LIMIT[i]);
            }
            else if (JointName(i) == right_knee || JointName(i) == left_knee)
            {
                // 位置
                auto [dLineMotorLen, _] = knee_kinematics.Knee_Inverse_Kinematics(joint_cmd[i].q); //单位mm
                motor_cmd[i].q = std::clamp<float>(JOINT_DIR[i] * dLineMotorLen, 5.0, MOTOR_POS_LIMIT[i]); // 单位mm
            }
            else if (JointName(i) == right_ankle_pitch)
            {
                // 位置
                InsKinematicsResult inverseResult = ankle_kinematics.Ankle_inverse_Kinematics(JOINT_DIR[i] * joint_cmd[i].q, JOINT_DIR[i+1] * joint_cmd[i+1].q, false);
                Eigen::Vector2f motorAngles = inverseResult.THETA; // Motor angles (theta)
                motor_cmd[i].q = std::clamp<float>(motorAngles[0], -MOTOR_POS_LIMIT[i], MOTOR_POS_LIMIT[i]);
                motor_cmd[i+1].q = std::clamp<float>(-motorAngles[1], -MOTOR_POS_LIMIT[i+1], MOTOR_POS_LIMIT[i+1]);
            }
            else if (JointName(i) == left_ankle_pitch)
            {
                // 位置
                InsKinematicsResult inverseResult = ankle_kinematics.Ankle_inverse_Kinematics(JOINT_DIR[i] * joint_cmd[i].q, JOINT_DIR[i+1] * joint_cmd[i+1].q, true);
                Eigen::Vector2f motorAngles = inverseResult.THETA; // Motor angles (theta)
                motor_cmd[i].q = std::clamp<float>(-motorAngles[0], -MOTOR_POS_LIMIT[i], MOTOR_POS_LIMIT[i]);
                motor_cmd[i+1].q = std::clamp<float>(motorAngles[1], -MOTOR_POS_LIMIT[i+1], MOTOR_POS_LIMIT[i+1]);
                // std::cout << joint_cmd[i].q <<  "  legJointToMotor left ankle motor_cmd[" << i << "]: q:" << motor_cmd[i].q << ", motor_cmd[" << i+1 << "]: q:" << motor_cmd[i+1].q << std::endl;
            }
        }
        else if (joint_cmd[i].mode == RUN_MODE_R1)
        {
            motor_cmd[i].tau = 0.0;
            if (JointName(i) == right_hip_roll || JointName(i) == right_hip_yaw || JointName(i) == right_hip_pitch ||
                JointName(i) == left_hip_roll || JointName(i) == left_hip_yaw || JointName(i) == left_hip_pitch)
            {
                motor_cmd[i].q = std::clamp<float>(JOINT_DIR[i] * joint_cmd[i].q, -MOTOR_MIXED_LIMIT[i], MOTOR_MIXED_LIMIT[i]);
            }
            else if (JointName(i) == right_knee || JointName(i) == left_knee)
            {
                // 位置
                auto [dLineMotorLen, _] = knee_kinematics.Knee_Inverse_Kinematics(joint_cmd[i].q); //单位mm
                motor_cmd[i].q = JOINT_DIR[i] * dLineMotorLen; // 单位mm
            }
            else if (JointName(i) == right_ankle_pitch)
            {
                // 位置
                InsKinematicsResult inverseResult = ankle_kinematics.Ankle_inverse_Kinematics(JOINT_DIR[i] * joint_cmd[i].q, JOINT_DIR[i+1] * joint_cmd[i+1].q, false);
                Eigen::Vector2f motorAngles = inverseResult.THETA; // Motor angles (theta)
                motor_cmd[i].q = motorAngles[0];
                motor_cmd[i+1].q = -motorAngles[1];
            }
            else if (JointName(i) == left_ankle_pitch)
            {
                // 位置
                InsKinematicsResult inverseResult = ankle_kinematics.Ankle_inverse_Kinematics(JOINT_DIR[i] * joint_cmd[i].q, JOINT_DIR[i+1] * joint_cmd[i+1].q, true);
                Eigen::Vector2f motorAngles = inverseResult.THETA; // Motor angles (theta)
                motor_cmd[i].q = -motorAngles[0];
                motor_cmd[i+1].q = motorAngles[1];
                // std::cout << joint_cmd[i].q <<  "  legJointToMotor left ankle motor_cmd[" << i << "]: q:" << motor_cmd[i].q << ", motor_cmd[" << i+1 << "]: q:" << motor_cmd[i+1].q << std::endl;
            }
        }
        else if (joint_cmd[i].mode == RUN_MODE_CST)
        {
            motor_cmd[i].q = 0.0;
            if (JointName(i) == right_hip_roll || JointName(i) == right_hip_yaw || JointName(i) == right_hip_pitch ||
                JointName(i) == left_hip_roll || JointName(i) == left_hip_yaw || JointName(i) == left_hip_pitch)
            {
                // std::cout << "joint_cmd[i].tau:" << joint_cmd[i].tau << std::endl;
                motor_cmd[i].tau = std::clamp(JOINT_DIR[i] * joint_cmd[i].tau, -MOTOR_TORQUE_LIMIT[i], MOTOR_TORQUE_LIMIT[i]); 
            }
            else if (JointName(i) == right_knee || JointName(i) == left_knee)
            {
                // auto [J_joint2motor, J_motor2joint] = knee_kinematics.Knee_Velocity_Jacobi(joint_state[i].q); //单位mm/rad 和 rad/mm
                auto [dLineMotorLen, lineMotor_len] = knee_kinematics.Knee_Inverse_Kinematics(joint_state[i].q);     
                auto [J_joint2motor, J_motor2joint] = knee_kinematics.Knee_Velocity_Jacobi_Analytical(dLineMotorLen);
                motor_cmd[i].tau = std::clamp(JOINT_DIR[i] * joint_cmd[i].tau / J_joint2motor * 1000, -MOTOR_TORQUE_LIMIT[i], MOTOR_TORQUE_LIMIT[i]); // 转换为 N
            }
            else if (JointName(i) == right_ankle_pitch)
            {
                std::pair<float, float> ankle_joint_q_left_pair = {
                    JOINT_DIR[i+6] * joint_state[i+6].q, 
                    JOINT_DIR[i+7] * joint_state[i+7].q
                };
                std::pair<float, float> ankle_joint_dq_left_pair = {
                    JOINT_DIR[i+6] * joint_state[i+6].dq, 
                    JOINT_DIR[i+7] * joint_state[i+7].dq
                };
                std::pair<float, float> ankle_joint_tau_left_pair = {
                    JOINT_DIR[i+6] * joint_cmd[i+6].tau, 
                    JOINT_DIR[i+7] * joint_cmd[i+7].tau
                };
                std::pair<float, float> ankle_joint_q_right_pair = {
                    JOINT_DIR[i] * joint_state[i].q, 
                    JOINT_DIR[i+1] * joint_state[i+1].q
                };
                std::pair<float, float> ankle_joint_dq_right_pair = {
                    JOINT_DIR[i] * joint_state[i].dq, 
                    JOINT_DIR[i+1] * joint_state[i+1].dq
                };
                std::pair<float, float> ankle_joint_tau_right_pair = {
                    JOINT_DIR[i] * joint_cmd[i].tau, 
                    JOINT_DIR[i+1] * joint_cmd[i+1].tau
                };
                auto [left_motor_q, right_motor_q, left_motor_dq, right_motor_dq, left_motor_tau, right_motor_tau] = ankle_kinematics.getDecoupleQVT(ankle_joint_q_left_pair, ankle_joint_dq_left_pair, ankle_joint_tau_left_pair,
                                                ankle_joint_q_right_pair, ankle_joint_dq_right_pair, ankle_joint_tau_right_pair);
                motor_cmd[i].tau = std::clamp<float>(right_motor_tau[0], -MOTOR_TORQUE_LIMIT[i], MOTOR_TORQUE_LIMIT[i]);
                motor_cmd[i+1].tau = std::clamp<float>(-right_motor_tau[1], -MOTOR_TORQUE_LIMIT[i+1], MOTOR_TORQUE_LIMIT[i+1]);
                motor_cmd[i+6].tau = std::clamp<float>(-left_motor_tau[0], -MOTOR_TORQUE_LIMIT[i+6], MOTOR_TORQUE_LIMIT[i+6]);
                motor_cmd[i+7].tau = std::clamp<float>(left_motor_tau[1], -MOTOR_TORQUE_LIMIT[i+7], MOTOR_TORQUE_LIMIT[i+7]);
            }
        }
    }
    return motor_cmd;
}

std::vector<JointData> KinConv::legMotorToJoint(const std::vector<JointData>& motor_state)
{
    if (motor_state.size() != LEG_MOTOR_COUNT)
    {
        throw std::runtime_error("KinConv::legMotorToJoint expects exactly 12 leg joints");
    }
    std::vector<JointData> joint_state(
        LEG_MOTOR_COUNT,
        {0.0f, 0.0f, 0.0f, RUN_MODE_CSP, 0.0f, 0.0f});

    for (size_t i = 0; i < LEG_MOTOR_COUNT; ++i) {
        if (MotorName(i) == hip_motor_r_roll || MotorName(i) == hip_motor_r_yaw || MotorName(i) == hip_motor_r_pitch ||
            MotorName(i) == hip_motor_l_roll || MotorName(i) == hip_motor_l_yaw || MotorName(i) == hip_motor_l_pitch)
        {
            joint_state[i].q   = JOINT_DIR[i] * motor_state[i].q;
            joint_state[i].dq  = JOINT_DIR[i] * motor_state[i].dq;
            joint_state[i].tau = JOINT_DIR[i] * motor_state[i].tau;
        }
        else if (MotorName(i) == knee_motor_r || MotorName(i) == knee_motor_l)
        {
            // 位置
            auto [dAlpha_rad, dAlpha_angle, _ ] = knee_kinematics.Knee_Forward_Kinematics(motor_state[i].q); //单位rad
            joint_state[i].q = JOINT_DIR[i] * dAlpha_rad;
            // 速度
            // auto [J_joint2motor, J_motor2joint] = knee_kinematics.Knee_Velocity_Jacobi(joint_state[i].q); //单位mm/rad 和 rad/mm
            auto [dLineMotorLen, lineMotor_len] = knee_kinematics.Knee_Inverse_Kinematics(joint_state[i].q);     
            auto [J_joint2motor, J_motor2joint] = knee_kinematics.Knee_Velocity_Jacobi_Analytical(dLineMotorLen);
            joint_state[i].dq  = JOINT_DIR[i] * motor_state[i].dq * J_motor2joint; // rad/s
            // 力矩
            joint_state[i].tau = JOINT_DIR[i] * motor_state[i].tau / J_motor2joint / 1000; // N
        }
        else if (MotorName(i) == ankle_motor_rl)
        {
            std::pair<float, float> ankle_motor_q_left_pair = {
                -motor_state[i+6].q, 
                motor_state[i+7].q
            };
            std::pair<float, float> ankle_motor_dq_left_pair = {
                -motor_state[i+6].dq, 
                motor_state[i+7].dq
            };
            std::pair<float, float> ankle_motor_tau_left_pair = {
                -motor_state[i+6].tau, 
                motor_state[i+7].tau
            };
            std::pair<float, float> ankle_motor_q_right_pair = {
                motor_state[i].q, 
                -motor_state[i+1].q
            };
            std::pair<float, float> ankle_motor_dq_right_pair = {
                motor_state[i].dq, 
                -motor_state[i+1].dq
            };
            std::pair<float, float> ankle_motor_tau_right_pair = {
                motor_state[i].tau, 
                -motor_state[i+1].tau
            };

            auto [ankle_joint_q_left, ankle_joint_q_right, ankle_joint_dq_left, ankle_joint_dq_right, ankle_joint_tau_left, ankle_joint_tau_right] = ankle_kinematics.getForwardQVT(ankle_motor_q_left_pair, ankle_motor_dq_left_pair, ankle_motor_tau_left_pair,
                                            ankle_motor_q_right_pair, ankle_motor_dq_right_pair, ankle_motor_tau_right_pair);
            joint_state[i].q   = JOINT_DIR[i] * ankle_joint_q_right[0];
            joint_state[i+1].q = JOINT_DIR[i+1] * ankle_joint_q_right[1];
            joint_state[i].dq  = JOINT_DIR[i] * ankle_joint_dq_right[0];
            joint_state[i+1].dq = JOINT_DIR[i+1] * ankle_joint_dq_right[1];
            joint_state[i].tau = JOINT_DIR[i] * ankle_joint_tau_right[0];
            joint_state[i+1].tau = JOINT_DIR[i+1] * ankle_joint_tau_right[1];
            joint_state[i+6].q = JOINT_DIR[i+6] * ankle_joint_q_left[0];
            joint_state[i+7].q = JOINT_DIR[i+7] * ankle_joint_q_left[1];
            joint_state[i+6].dq  = JOINT_DIR[i+6] * ankle_joint_dq_left[0];
            joint_state[i+7].dq = JOINT_DIR[i+7] * ankle_joint_dq_left[1];
            joint_state[i+6].tau = JOINT_DIR[i+6] * ankle_joint_tau_left[0];
            joint_state[i+7].tau = JOINT_DIR[i+7] * ankle_joint_tau_left[1];
        }
    }

    return joint_state;
}

std::vector<JointData> KinConv::armJointToMotor(
    const std::vector<JointData> &joint_state,
    const std::vector<JointData> &joint_cmd) const
{
    if (joint_state.size() != joint_cmd.size())
    {
        throw std::runtime_error("KinConv::armJointToMotor requires matching joint_state and joint_cmd sizes");
    }
    return joint_cmd;
}

std::vector<JointData> KinConv::armMotorToJoint(const std::vector<JointData> &motor_state) const
{
    return motor_state;
}

std::vector<JointData> KinConv::waistJointToMotor(
    const std::vector<JointData> &joint_state,
    const std::vector<JointData> &joint_cmd) const
{
    if (joint_state.size() != joint_cmd.size())
    {
        throw std::runtime_error("KinConv::waistJointToMotor requires matching joint_state and joint_cmd sizes");
    }
    return joint_cmd;
}

std::vector<JointData> KinConv::waistMotorToJoint(const std::vector<JointData> &motor_state) const
{
    return motor_state;
}
