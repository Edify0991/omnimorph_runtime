#pragma once
#ifndef __SHARED_MEMORY_DATA_TYPE_HPP
#define __SHARED_MEMORY_DATA_TYPE_HPP

#include <cstdint>
#include <string>
#include <unistd.h>
#include <pwd.h>

struct RotateMotorMixControl
{
    float kp;
    float kd;
    float position; // rad
    float speed;    // rad/s
    float torque;   // Nm
};

struct RotateMotorPosControl
{
    float position;           // rad
    float speed;              // rad/s
    uint16_t maxPhaseCurrent; // A
};

struct RotateMotorTauControl
{
    int16_t currentTorque;
    uint8_t ctrlStatus;
};

struct RotateMotorCmdData
{
    uint8_t slave;                // 电机对应从站
    uint8_t passage;              // 电机在从站对应位置
    uint8_t motorId;              // 电机ID号
    uint8_t motorRunningMode = 1; // 电机运行模式

    RotateMotorMixControl mixControl = {20.0, 5.0, 0, 0.0, 0.0}; // 混合控制模式
    RotateMotorPosControl posControl = {0, 2, 50};               // 位置控制模式
    RotateMotorTauControl tauControl = {0, 0};                   // 力矩控制模
};

struct LineMotorCmdData
{
    uint8_t motorId;
    uint8_t motorRunningMode;
    float linearPos = 0;
    float linearSpeed = 0;
    float linearTau = 0;
};

struct AllRotateMotorsCmdData
{
    RotateMotorCmdData hipAnkleMotorsCmd[10]; // 改为10个旋转电机
};

struct AllLineMotorsCmdData
{
    LineMotorCmdData kneeMotorsCmd[2]; // 保持2个线性电机（膝关节）
};
#pragma pack(1)
struct RotateMotorStateData
{
    uint8_t motorId;
    float torque;
    float position;
    float speed;
};
#pragma pack()
struct LineMotorStateData
{
    uint8_t motorId;
    float linearTau;
    float linearPos;
    float linearSpeed;
};

struct AllRotateMotorsStateData
{
    RotateMotorStateData hipAnkleMotorState[10]; // 改为10个旋转电机
};
struct AllLineMotorsStateData
{
    LineMotorStateData kneeMotorsState[2]; // 保持2个线性电机（膝关节）
};

struct ImuData3DType
{
    float x;
    float y;
    float z;
};

struct ImuData4DType
{
    float x;
    float y;
    float z;
    float w;
};

#pragma pack(4)
struct ImuData
{
    ImuData3DType imuAcceleration;
    ImuData3DType imuAngularVelocity;
    ImuData3DType imuAngularDegree;
    ImuData4DType imuQuaternion;
};
#pragma pack()

class SharedDataConfig
{
public:
    std::string filePathStr;
    const char *FILE_PATH;

    SharedDataConfig()
    {
        uid_t uid = getuid();
        passwd *pwd = getpwuid(uid);
        if (pwd && pwd->pw_dir)
        {
            filePathStr = std::string(pwd->pw_dir) + "/";
        }
        FILE_PATH = filePathStr.c_str();
    }
    const int keyNum[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    const std::string semName[13] = {
        "/rotateMotorCmdSharedMemory",
        "/lineMotorCmdSharedMemory",
        "/motorCmdSharedMemory_rl",
        "/rotateMotorStateSharedMemory",
        "/lineMotorStateSharedMemory",
        "/motorStateSharedMemory_rl",
        "/imuDataSharedMemory",
        "/rotateMotorPDSmSharedMemory",
        "/rlJointCmdSharedMemory",
        "/rlJointStateSharedMemory",
        "/rlRobotCmd",
        "/rlSolverFlag",
        "/modeControlSharedMemory"
    };
    const uint16_t length[13] = {
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
        4 * 400,
    };
    const int semFlag = 0;
};

#endif
