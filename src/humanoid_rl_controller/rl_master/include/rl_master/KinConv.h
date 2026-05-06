#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>

#include "rl_master/Ankle_Kinematics.h"
#include "rl_master/Knee_Kinematics.h"
#include "rl_master/rl_cfg.h"

#define LEG_MOTOR_COUNT (12)   // 下肢有12个电机
#define WAIST_MOTOR_COUNT (2)  // 腰有2个电机
#define ARM_MOTOR_COUNT (14)   // 上肢有14个电机
#define TOTAL_MOTOR_COUNT (28) // 全身共28个电机

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

struct MotorLimitRange
{
    float min;
    float max;
};

// joint name and number: the order is consistent with the observed order of reinforcement learning
enum JointName
{
    right_hip_roll = 0,
    right_hip_yaw = 1,
    right_hip_pitch = 2,
    right_knee_pitch = 3,
    right_ankle_pitch = 4,
    right_ankle_roll = 5,
    left_hip_roll = 6,
    left_hip_yaw = 7,
    left_hip_pitch = 8,
    left_knee_pitch = 9,
    left_ankle_pitch = 10,
    left_ankle_roll = 11,
    waist_roll = 12,
    waist_yaw = 13,
    right_shoulder_pitch = 14,
    right_shoulder_roll = 15,
    right_shoulder_yaw = 16,
    right_elbow_pitch = 17,
    right_elbow_yaw = 18,
    right_wrist_pitch = 19,
    right_wrist_roll = 20,
    left_shoulder_pitch = 21,
    left_shoulder_roll = 22,
    left_shoulder_yaw = 23,
    left_elbow_pitch = 24,
    left_elbow_yaw = 25,
    left_wrist_pitch = 26,
    left_wrist_roll = 27,
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
    ankle_motor_lr = 11,
    waist_motor_roll = 12,
    waist_motor_yaw = 13,
    shoulder_motor_r_pitch = 14,
    shoulder_motor_r_roll = 15,
    shoulder_motor_r_yaw = 16,
    elbow_motor_r_pitch = 17,
    elbow_motor_r_yaw = 18,
    wrist_motor_r_pitch = 19,
    wrist_motor_r_roll = 20,
    shoulder_motor_l_pitch = 21,
    shoulder_motor_l_roll = 22,
    shoulder_motor_l_yaw = 23,
    elbow_motor_l_pitch = 24,
    elbow_motor_l_yaw = 25,
    wrist_motor_l_pitch = 26,
    wrist_motor_l_roll = 27,
};

enum AppMode
{
    APP_MODE_STOP,
    APP_MODE_TOR = 0x0A,
    APP_MODE_POS = 0x08,
    APP_MODE_POS_TOR,
};

// URDF 方向相对于电机/运动学方向的关系
static constexpr std::array<int, LEG_MOTOR_COUNT> LEG_JOINT_DIR = {
    -1, +1, -1, +1, +1, -1, // right leg
    +1, -1, +1, +1, +1, +1  // left leg
};

static constexpr std::array<int, WAIST_MOTOR_COUNT> WAIST_JOINT_DIR = {
    +1, +1
};

static constexpr std::array<int, ARM_MOTOR_COUNT> ARM_JOINT_DIR = {
    +1, -1, -1, +1, -1, +1, +1, // right arm
    -1, +1, -1, -1, -1, +1, -1  // left arm
};

static constexpr std::array<float, LEG_MOTOR_COUNT> LEG_MOTOR_TORQUE_LIMIT = {
    150, 150, 150, 2500, 100, 100, // right leg motors
    150, 150, 150, 2500, 100, 100  // left leg motors
};

static constexpr std::array<float, WAIST_MOTOR_COUNT> WAIST_MOTOR_TORQUE_LIMIT = {
    150, 100
};

static constexpr std::array<float, ARM_MOTOR_COUNT> ARM_MOTOR_TORQUE_LIMIT = {
    90, 90, 60, 60, 36, 36, 36,  // right arm
    90, 90, 60, 60, 36, 36, 36   // left arm
};

static constexpr std::array<MotorLimitRange, LEG_MOTOR_COUNT> LEG_MOTOR_POS_LIMIT = {{
    {-0.5233f, 0.1396f}, {-0.5233f, 0.5233f}, {-0.7850f, 1.256f}, {5.0f, 50.0f}, {-0.64f, 0.64f}, {-0.64f, 0.64f}, // right leg motors
    {-0.1396f, 0.5233f}, {-0.5233f, 0.5233f}, {-1.256f, 0.7850f}, {5.0f, 50.0f}, {-0.64f, 0.64f}, {-0.64f, 0.64f}  // left leg motors
}};

static constexpr std::array<MotorLimitRange, WAIST_MOTOR_COUNT> WAIST_MOTOR_POS_LIMIT = {{
    {-0.2617f, 0.2617f}, {-0.7853f, 0.7853f}
}};

static constexpr std::array<MotorLimitRange, ARM_MOTOR_COUNT> ARM_MOTOR_POS_LIMIT = {{
    {-1.0467f, 3.1415f}, {-2.2689f, 0.0872f}, {-2.0943f, 2.0943f}, {0.0f, 1.8325f}, {-2.0943f, 2.0943f}, {-1.2217f, 1.2217f}, {-1.2217f, 1.2217f}, // right arm
    {-3.1415f, 1.0467f}, {-0.0872f, 2.2689f}, {-2.0943f, 2.0943f}, {-1.8325f, 0.0f}, {-2.0943f, 2.0943f}, {-1.2217f, 1.2217f}, {-1.2217f, 1.2217f}  // left arm
}};

static constexpr std::array<MotorLimitRange, LEG_MOTOR_COUNT> LEG_MOTOR_MIXED_LIMIT = {{
    {-0.5233f, 0.1396f}, {-0.5233f, 0.5233f}, {-0.7850f, 1.256f}, {5.0f, 50.0f}, {-0.64f, 0.64f}, {-0.64f, 0.64f}, // right leg motors
    {-0.1396f, 0.5233f}, {-0.5233f, 0.5233f}, {-1.256f, 0.7850f}, {5.0f, 50.0f}, {-0.64f, 0.64f}, {-0.64f, 0.64f}  // left leg motors
}};

static constexpr std::array<MotorLimitRange, WAIST_MOTOR_COUNT> WAIST_MOTOR_MIXED_LIMIT = {{
    {-0.2617f, 0.2617f}, {-0.7853f, 0.7853f}
}};

static constexpr std::array<MotorLimitRange, ARM_MOTOR_COUNT> ARM_MOTOR_MIXED_LIMIT = {{
    {-1.0467f, 3.1415f}, {-2.2689f, 0.0872f}, {-2.0943f, 2.0943f}, {0.0f, 1.8325f}, {-2.0943f, 2.0943f}, {-1.2217f, 1.2217f}, {-1.2217f, 1.2217f}, // right arm
    {-3.1415f, 1.0467f}, {-0.0872f, 2.2689f}, {-2.0943f, 2.0943f}, {-1.8325f, 0.0f}, {-2.0943f, 2.0943f}, {-1.2217f, 1.2217f}, {-1.2217f, 1.2217f}  // left arm
}};

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

    // 调用运动学类
    Ankle_Kinematics ankle_kinematics;
    Knee_Kinematics knee_kinematics;
    std::vector<int> leg_global_indices_;
    std::vector<int> arm_global_indices_;
    std::vector<int> waist_global_indices_;
};
