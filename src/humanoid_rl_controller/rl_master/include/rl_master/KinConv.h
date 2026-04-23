#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

#include "rl_master/Ankle_Kinematics.h"
#include "rl_master/Knee_Kinematics.h"
#include "rl_master/rl_cfg.h"

#define LEG_MOTOR_COUNT (12) // 下肢有12个电机

typedef enum
{
    RUN_MODE_PP = 0x01,
    RUN_MODE_R1,
    RUN_MODE_PV,
    RUN_MODE_PT,
    RUN_MODE_R2,
    RUN_MODE_HM,
    RUN_MODE_R3,
    RUN_MODE_CSP,
    RUN_MODE_CSV,
    RUN_MODE_CST
} MotorRunMode;

// joint data, 用来存储关节的位置、速度和力矩的数据结构
struct JointData
{
    float q;  // rad
    float dq; // rad/mm
    float tau;
    MotorRunMode mode;
    float kp;
    float kd;
};

struct MotorPD
{
    int Kp;
    int Kd;
};

// joint name and number: the order is consistent with the observed order of reinforcement learning
enum JointName
{
    right_hip_roll = 0,
    right_hip_yaw = 1,
    right_hip_pitch = 2,
    right_knee = 3,
    right_ankle_pitch = 4,
    right_ankle_roll = 5,
    left_hip_roll = 6,
    left_hip_yaw = 7,
    left_hip_pitch = 8,
    left_knee = 9,
    left_ankle_pitch = 10,
    left_ankle_roll = 11
};

// motor name and number: the order is consistent with the actual ID of the motor
enum MotorName
{
    hip_motor_r_roll = 0,
    hip_motor_r_yaw = 1,
    hip_motor_r_pitch = 2,
    knee_motor_r = 3,
    ankle_motor_rl = 4,
    ankle_motor_rr = 5,
    hip_motor_l_roll = 6,
    hip_motor_l_yaw = 7,
    hip_motor_l_pitch = 8,
    knee_motor_l = 9,
    ankle_motor_ll = 10,
    ankle_motor_lr = 11
};

enum AppMode
{
    APP_MODE_STOP,
    APP_MODE_TOR = 0x0A,
    APP_MODE_POS = 0x08,
    APP_MODE_POS_TOR,
};

// 运动学的方向和电机的方向已经保证一致，这里是urdf方向相对于运动学方向的关系
static constexpr std::array<int, LEG_MOTOR_COUNT> JOINT_DIR = {
    -1, +1, -1, +1, +1, -1, // right leg
    +1, -1, +1, +1, +1, +1  // left leg
};
// // 运动学的方向和电机的方向已经保证一致，这里是urdf方向相对于运动学方向的关系
// static constexpr std::array<int, LEG_MOTOR_COUNT> JOINT_DIR = {
//     -1, +1, +1, +1, +1, -1,   // right leg
//     +1, -1, -1, +1, +1, +1    // left leg
// };

static constexpr std::array<float, LEG_MOTOR_COUNT> MOTOR_TORQUE_LIMIT = {
    150,
    150,
    150,
    2500,
    100,
    100,
    150,
    150,
    150,
    2500,
    100,
    100,
};

static constexpr std::array<float, LEG_MOTOR_COUNT> MOTOR_POS_LIMIT = {
    0.25, 1.2, 1.2, 50, 0.64, 0.64,
    0.25, 1.2, 1.2, 50, 0.64, 0.64};

static constexpr std::array<float, LEG_MOTOR_COUNT> MOTOR_MIXED_LIMIT = {
    0.15, 0.2, 0.55, 50, 0.64, 0.64,
    0.15, 0.2, 0.55, 50, 0.64, 0.64};

class KinConv
{
public:
    KinConv();
    ~KinConv();

    void configureJointGroups(
        const std::vector<std::string> &global_joint_order,
        const JointGroupsConfig &joint_groups);

    std::vector<JointData> legJointToMotor(const std::vector<JointData> &joint_state, const std::vector<JointData> &joint_cmd);
    std::vector<JointData> legMotorToJoint(const std::vector<JointData> &motor_state);
    std::vector<JointData> armJointToMotor(const std::vector<JointData> &joint_state, const std::vector<JointData> &joint_cmd);
    std::vector<JointData> armMotorToJoint(const std::vector<JointData> &motor_state);
    std::vector<JointData> waistJointToMotor(const std::vector<JointData> &joint_state, const std::vector<JointData> &joint_cmd);
    std::vector<JointData> waistMotorToJoint(const std::vector<JointData> &motor_state);

    const std::vector<int> &legGlobalIndices() const;
    const std::vector<int> &armGlobalIndices() const;
    const std::vector<int> &waistGlobalIndices() const;

private:
    void validateLegGroupSize() const;
    static std::vector<JointData> passThroughJointToMotor(
        const std::vector<JointData> &joint_cmd);
    static std::vector<JointData> passThroughMotorToJoint(
        const std::vector<JointData> &motor_state);

    // 调用运动学类
    Ankle_Kinematics ankle_kinematics;
    Knee_Kinematics knee_kinematics;
    std::vector<int> leg_global_indices_;
    std::vector<int> arm_global_indices_;
    std::vector<int> waist_global_indices_;
};
