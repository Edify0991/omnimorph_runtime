#include "rl_master/kinematics/jc01/ankle_kinematics.h"
#include <iostream>

// input: pitch, roll rad; output: dleftMotor, drightMotor rad
InsKinematicsResult Ankle_Kinematics::Ankle_inverse_Kinematics(float pitch, float roll, bool leftLegFlag)
{
    // 1. 计算足底旋转矩阵
    Eigen::AngleAxisf rotation_y_pitch(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rotation_x_roll(roll, Eigen::Vector3f::UnitX());
    Eigen::Quaternionf q_rot = rotation_y_pitch * rotation_x_roll;
    Eigen::Matrix3f x_rot = q_rot.toRotationMatrix();

    // 结果结构体 (如果需要详细信息可以保留，这里主要用于计算)
    InsKinematicsResult result;
    result.r_A.clear();
    result.r_B.clear();
    result.r_C.clear();
    result.r_bar.clear();
    result.r_rod.clear();
    result.THETA = Eigen::Vector2f::Zero();

    float l_bar = 60; // # up ??NP=60

    float l_rod[2] = {250, 140}; // # long rod
    float l_spacing1 = 41.25;    // # long rod ??FP_y=41.25
    float l_spacing2 = 41.25;    // # short rod ??FP'_y=41.25

    float short_link_angle_0 = 0 * M_PI / 180;
    float long_link_angle_0 = 0 * M_PI / 180;

    // Calculate initial positions (B1_0, B2_0)
    float r_B1_0_x = -13.08398906073595818052965740227 - l_bar * cos(long_link_angle_0);
    float r_B1_0_z = 249.65738368864346844612301460986 - l_bar * sin(long_link_angle_0);
    float r_B2_0_x = -7.327033874012136581096608145271 - l_bar * cos(short_link_angle_0);
    float r_B2_0_z = 139.80813486564034232982888818152 - l_bar * sin(short_link_angle_0);

    if (leftLegFlag)
    { // left leg
        l_spacing1 = -l_spacing1;
        l_spacing2 = -l_spacing2;
    }

    // Define initial points for both legs (assuming right leg first, then left will be flipped)
    std::vector<Eigen::Vector3f> r_A_0(2);
    std::vector<Eigen::Vector3f> r_B_0(2);
    std::vector<Eigen::Vector3f> r_C_0(2);

    // Right Leg (index 0) - Use positive spacing
    r_A_0[0] = Eigen::Vector3f(-13.08398906073595818052965740227, l_spacing1, 249.65738368864346844612301460986);
    r_B_0[0] = Eigen::Vector3f(r_B1_0_x, l_spacing1, r_B1_0_z);
    r_C_0[0] = Eigen::Vector3f(-l_bar, l_spacing1, 0);

    // Left Leg (index 1) - Use negative spacing
    r_A_0[1] = Eigen::Vector3f(-7.327033874012136581096608145271, -l_spacing2, 139.80813486564034232982888818152);
    r_B_0[1] = Eigen::Vector3f(r_B2_0_x, -l_spacing2, r_B2_0_z);
    r_C_0[1] = Eigen::Vector3f(-l_bar, -l_spacing2, 0);

    // Inverse Kinematics for both legs
    for (int i = 0; i < 2; i++)
    {
        Eigen::Vector3f r_A_i = r_A_0[i];
        Eigen::Vector3f r_C_i = x_rot * r_C_0[i];      // Apply rotation to foot point
        Eigen::Vector3f rBA_bar = r_B_0[i] - r_A_0[i]; // Vector from A to B in initial config

        Eigen::Vector3f r_AB_0 = r_B_0[i] - r_A_0[i]; // Vector from A to B (initial)
        Eigen::Vector3f r_CA = r_A_i - r_C_i;         // Vector from C to A (current)

        float M = pow(l_rod[i], 2) - pow(r_CA.norm(), 2) - pow(l_bar, 2);
        float N = r_CA[0] * r_AB_0[0] + r_CA[2] * r_AB_0[2];
        float K = r_CA[0] * r_AB_0[2] - r_CA[2] * r_AB_0[0];

        float a = 4 * (pow(K, 2) + pow(N, 2));
        float b = -4 * M * K;
        float c = pow(M, 2) - 4 * pow(N, 2);

        float theta_i = 0.0;
        bool legal_ori = true;
        // std::cout << "legal_ori" << std::endl;
        if (pow(b, 2) - 4 * a * c < 0.0)
        {
            theta_i = 0.0;
            legal_ori = false;
            // Optionally log or handle out-of-range
            std::cout << "Orientation out of range for leg " << i << std::endl;
        }
        else
        {
            // Two solutions exist, we take one (+sqrt). Consider which solution is physically valid.
            theta_i = std::asin((-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a));
        }

        // Create rotation matrix for the link
        Eigen::Matrix3f R_y_theta = Eigen::Matrix3f::Zero();
        R_y_theta << std::cos(theta_i), 0, std::sin(theta_i),
            0, 1, 0,
            -std::sin(theta_i), 0, std::cos(theta_i);

        // Calculate new position of B
        Eigen::Vector3f r_B_i = r_A_i + R_y_theta * rBA_bar;
        Eigen::Vector3f r_bar_i = r_B_i - r_A_i; // Vector along the bar
        Eigen::Vector3f r_rod_i = r_C_i - r_B_i; // Vector along the rod

        // Store results (if needed for debugging or other purposes)
        result.r_A.push_back(r_A_i);
        result.r_B.push_back(r_B_i);
        result.r_C.push_back(r_C_i);
        result.r_bar.push_back(r_bar_i);
        result.r_rod.push_back(r_rod_i);
        result.THETA[i] = theta_i;
    }
    // Return the two calculated joint angles (radians)
    // Assuming index 0 is left motor, index 1 is right motor (or vice versa, depends on your motor assignment)
    // return std::make_pair(result.THETA[0], result.THETA[1]);
    return result;
}

std::vector<Eigen::MatrixXf>
Ankle_Kinematics::Ankle_jacobian(const std::vector<Eigen::Vector3f> &r_C,
                                 const std::vector<Eigen::Vector3f> &r_bar,
                                 const std::vector<Eigen::Vector3f> &r_rod,
                                 float q_pitch)
{
    // Assuming r_C, r_bar, r_rod are vectors of Eigen::Vector3f with at least 2 elements each

    Eigen::Vector3f s_11{0, 1, 0}; // A1 point unit direction vector
    Eigen::Vector3f s_21{0, 1, 0}; // A2 point unit direction vector

    Eigen::MatrixXf J_x = Eigen::MatrixXf::Zero(2, 6);
    J_x.block<1, 3>(0, 0) = r_rod[0].transpose();
    J_x.block<1, 3>(1, 0) = r_rod[1].transpose();
    J_x.block<1, 3>(0, 3) = (r_C[0].cross(r_rod[0])).transpose();
    J_x.block<1, 3>(1, 3) = (r_C[1].cross(r_rod[1])).transpose();

    Eigen::MatrixXf J_theta = Eigen::MatrixXf::Zero(2, 2);
    J_theta(0, 0) = s_11.dot(r_bar[0].cross(r_rod[0]));
    J_theta(1, 1) = s_21.dot(r_bar[1].cross(r_rod[1]));

    /*after*/
    Eigen::MatrixXf J_q = Eigen::MatrixXf::Zero(6, 2); // 第一列对应qd_pitch，第二列对应qd_roll
    J_q(3, 1) = std::cos(q_pitch);
    J_q(4, 0) = 1;
    J_q(5, 1) = -std::sin(q_pitch);

    // std::cout << "J_x: " << J_x << std::endl;
    // std::cout << "J_q: " << J_q << std::endl;
    // std::cout << "J_x*J_q:" << J_x*J_q << std::endl;
    Eigen::MatrixXf J_Temp = (J_x * J_q);
    // Eigen::MatrixXf J_ankle = (J_x*J_q).inverse() * J_theta;
    Eigen::MatrixXf J_motor2Joint = J_Temp.inverse() * J_theta;
    Eigen::MatrixXf J_Joint2motor = J_theta.inverse() * J_Temp;
    // Eigen::MatrixXf J_ankle = (J_x*J_q).completeOrthogonalDecomposition().pseudoInverse() * J_theta;
    std::vector<Eigen::MatrixXf> J_ankle;
    J_ankle.push_back(J_motor2Joint);
    J_ankle.push_back(J_Joint2motor);
    return J_ankle;
    /*after*/
}

std::pair<Eigen::Vector2f, std::vector<Eigen::MatrixXf>>
Ankle_Kinematics::getDecouple(float roll, float pitch, bool leftLegFlag)
{
    InsKinematicsResult kinematics = Ankle_inverse_Kinematics(pitch, roll, leftLegFlag);
    std::vector<Eigen::MatrixXf> Jac = Ankle_jacobian(kinematics.r_C, kinematics.r_bar, kinematics.r_rod, pitch);
    return {kinematics.THETA, Jac};
}

ForwardMappingResult
Ankle_Kinematics::Ankle_forward_Kinematics(const Eigen::Vector2f &thetaRef, bool leftLegFlag)
{

    ForwardMappingResult mapping_result;

    int count = 0;
    Eigen::Vector2f f_error{10, 10};
    Eigen::Vector2f x_c_k{0, 0}; // {pitch, roll}

    std::vector<Eigen::MatrixXf> Jac;

    /*after*/
    while (f_error.norm() > 1e-3 && count < 100)
    {
        InsKinematicsResult kinematics = Ankle_inverse_Kinematics(x_c_k[0], x_c_k[1], leftLegFlag);
        // printKinematicsResult(kinematics);

        Jac = Ankle_jacobian(kinematics.r_C, kinematics.r_bar, kinematics.r_rod, x_c_k[0]);
        // std::cout << "===== count:" << count << "\n Jac: " << Jac << "\n THEAT:" << kinematics.THETA << std::endl;
        Eigen::MatrixXf J_motor2Joint = Jac[0];
        // Eigen::MatrixXf J_Joint2motor = Jac[1];
        if (J_motor2Joint.hasNaN())
        {
            std::cerr << "Decouple::forwardKinematics() Jac is nan!!" << std::endl;
            std::cerr << "  roll x_c_k[1],pitch  x_c_k[0] n!!" << x_c_k[1] << "   ---   " << x_c_k[0] << std::endl;
            x_c_k << 0, 0;
#ifdef TARGET_MUJOCO
            exit(1);
#else
            mapping_result.count = -1;
            mapping_result.ankle_joint_ori = x_c_k;
            mapping_result.Jac = Jac;
            return mapping_result; // -1 是失败的标记
#endif
        }

        Eigen::Vector2f thetaCal = Eigen::Vector2f(kinematics.THETA[0], kinematics.THETA[1]);

        f_error = thetaRef - thetaCal;

        x_c_k = x_c_k + J_motor2Joint * f_error;
        // std::cout <<  " thetaCal: " << thetaCal << "\n f_error: " << f_error << "\n pitch_roll:" << x_c_k << std::endl;

        count++;
    }
    /*after*/

    mapping_result.count = count;
    mapping_result.ankle_joint_ori = x_c_k;
    mapping_result.Jac = Jac;

    return mapping_result; // -1 是失败的标记
}
//////********************forward kinematics*****************//////

// from x to theta， from Serial to Parallel
void Ankle_Kinematics::getDecoupleTorque(Eigen::Matrix<float, -1, 1> &tau, Eigen::Matrix<float, -1, 1> &q)
{
    float rightPitch, rightRoll, leftPitch, leftRoll;
    rightPitch = q[10];
    rightRoll = q[11];
    leftPitch = q[4];
    leftRoll = q[5];

    /*after*/
    std::pair<Eigen::Vector2f, std::vector<Eigen::MatrixXf>> left, right; // 联立顺序：先right后left

    right = getDecouple(rightRoll, rightPitch, false);
    tau.segment<2>(10) = right.second[1].transpose() * (tau.segment<2>(10));

    left = getDecouple(leftRoll, leftPitch, true);
    q.segment<2>(4) = left.first;
    tau.segment<2>(4) = left.second[1].transpose() * (tau.segment<2>(4));
    /*after*/
}

// from x to theta， from Serial to Parallel
// force control ,should input current pitch roll
std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f>
Ankle_Kinematics::getDecoupleQVT(
    const std::pair<float, float> &q_left,
    const std::pair<float, float> &vel_left,
    const std::pair<float, float> &tau_left,
    const std::pair<float, float> &q_right,
    const std::pair<float, float> &vel_right,
    const std::pair<float, float> &tau_right)
{
    // q_right.second = -q_right.second; // rotation axis [-1 0 0]
    // vel_right.second = -vel_right.second;
    // tau_right.second = -tau_right.second;

    float rightPitch, rightRoll, leftPitch, leftRoll;
    rightPitch = q_right.first; // rotation axis [0 1 0]
    rightRoll = q_right.second;
    leftPitch = q_left.first; // rotation axis[0 1 0]
    leftRoll = q_left.second;

    std::pair<Eigen::Vector2f, std::vector<Eigen::MatrixXf>> left, right; // 联立顺序：先right后left

    right = getDecouple(rightRoll, rightPitch, false);
    Eigen::Vector2f right_q = right.first;
    Eigen::Vector2f vel_right_vec = right.second[1] * Eigen::Vector2f(vel_right.first, vel_right.second);
    Eigen::Vector2f tau_right_vec = right.second[0].transpose() * Eigen::Vector2f(tau_right.first, tau_right.second);

    left = getDecouple(leftRoll, leftPitch, true);
    Eigen::Vector2f left_q = left.first;
    Eigen::Vector2f vel_left_vec = left.second[1] * Eigen::Vector2f(vel_left.first, vel_left.second);
    Eigen::Vector2f tau_left_vec = left.second[0].transpose() * Eigen::Vector2f(tau_left.first, tau_left.second);

    return std::make_tuple(left_q, right_q, vel_left_vec, vel_right_vec, tau_left_vec, tau_right_vec);
}

std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f>
Ankle_Kinematics::getForwardQVT(
    const std::pair<float, float> &q_left,
    const std::pair<float, float> &dq_left,
    const std::pair<float, float> &tau_left,
    const std::pair<float, float> &q_right,
    const std::pair<float, float> &dq_right,
    const std::pair<float, float> &tau_right)
{
    // 提取右腿的 q、dq 和 tau
    Eigen::Vector2f right_leg_q(q_right.first, q_right.second);
    Eigen::Vector2f right_leg_dq(dq_right.first, dq_right.second);
    Eigen::Vector2f right_leg_tau(tau_right.first, tau_right.second);

    // 提取左腿的 q、dq 和 tau
    Eigen::Vector2f left_leg_q(q_left.first, q_left.second);
    Eigen::Vector2f left_leg_dq(dq_left.first, dq_left.second);
    Eigen::Vector2f left_leg_tau(tau_left.first, tau_left.second);

    // 右腿的前向运动学计算
    ForwardMappingResult right_leg_result = Ankle_forward_Kinematics(right_leg_q, false);
    right_leg_q = right_leg_result.ankle_joint_ori;
    right_leg_dq = right_leg_result.Jac[0] * right_leg_dq;               // vel transfer from motor to ankle joint
    right_leg_tau = right_leg_result.Jac[1].transpose() * right_leg_tau; // tau transfer from motor to ankle joint

    // 左腿的前向运动学计算
    ForwardMappingResult left_leg_result = Ankle_forward_Kinematics(left_leg_q, true);
    left_leg_q = left_leg_result.ankle_joint_ori;
    left_leg_dq = left_leg_result.Jac[0] * left_leg_dq;               // vel transfer from motor to ankle joint
    left_leg_tau = left_leg_result.Jac[1].transpose() * left_leg_tau; // tau transfer from motor to ankle joint

    return std::make_tuple(left_leg_q, right_leg_q, left_leg_dq, right_leg_dq, left_leg_tau, right_leg_tau);
}
