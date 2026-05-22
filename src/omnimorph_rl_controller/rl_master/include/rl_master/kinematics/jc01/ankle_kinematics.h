#ifndef ANKLE_KINEMATICS_H
#define ANKLE_KINEMATICS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility> // for std::pair
#include <vector>
#include <cmath>
#include <chrono>

// Forward declaration of result structure (if needed externally)
struct InsKinematicsResult
{
    std::vector<Eigen::Vector3f> r_A;
    std::vector<Eigen::Vector3f> r_B;
    std::vector<Eigen::Vector3f> r_C;
    std::vector<Eigen::Vector3f> r_bar;
    std::vector<Eigen::Vector3f> r_rod;
    Eigen::Vector2f THETA; // [right_motor_angle, left_motor_angle] (rad)
};

struct ForwardMappingResult
{
    int count;
    Eigen::Vector2f ankle_joint_ori;
    std::vector<Eigen::MatrixXf> Jac;
};

class Ankle_Kinematics
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    InsKinematicsResult Ankle_inverse_Kinematics(float q_pitch, float q_roll, bool leftLegFlag);

    std::vector<Eigen::MatrixXf> Ankle_jacobian(const std::vector<Eigen::Vector3f> &r_C, const std::vector<Eigen::Vector3f> &r_bar,
                                                const std::vector<Eigen::Vector3f> &r_rod, float q_pitch);

    std::pair<Eigen::Vector2f, std::vector<Eigen::MatrixXf>> getDecouple(float roll, float pitch, bool leftLegFlag);

    ForwardMappingResult Ankle_forward_Kinematics(const Eigen::Vector2f &thetaRef, bool leftLegFlag);

    void getDecoupleTorque(Eigen::Matrix<float, -1, 1> &tau, Eigen::Matrix<float, -1, 1> &q); // 实机
    std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f>
    getDecoupleQVT(
        const std::pair<float, float> &q_left,
        const std::pair<float, float> &dq_left,
        const std::pair<float, float> &tau_left,
        const std::pair<float, float> &q_right,
        const std::pair<float, float> &dq_right,
        const std::pair<float, float> &tau_right);
    std::tuple<Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f>
    getForwardQVT(
        const std::pair<float, float> &q_left,
        const std::pair<float, float> &dq_left,
        const std::pair<float, float> &tau_left,
        const std::pair<float, float> &q_right,
        const std::pair<float, float> &dq_right,
        const std::pair<float, float> &tau_right);
};

#endif // ANKLE_KINEMATICS_H
