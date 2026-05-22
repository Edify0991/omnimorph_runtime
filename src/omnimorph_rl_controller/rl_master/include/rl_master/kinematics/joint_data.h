#ifndef RL_MASTER_KINEMATICS_JOINT_DATA_H
#define RL_MASTER_KINEMATICS_JOINT_DATA_H

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

struct JointData
{
    float q;  // rad
    float dq; // rad/s for rotary joints, mm/s for linear actuators
    float tau;
    MotorRunMode mode;
    float kp;
    float kd;
};

#endif // RL_MASTER_KINEMATICS_JOINT_DATA_H
