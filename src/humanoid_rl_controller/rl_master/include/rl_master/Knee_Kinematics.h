#ifndef KNEE_KINEMATICS_H
#define KNEE_KINEMATICS_H

#include <iostream>
#include <cmath>
#include <string>
#include <utility> // for std::pair
#include <tuple>
#include <chrono>

class Knee_Kinematics
{
public:
    // input: dLineMotorLen mm; output: dAlpha rad and dAlpha angle; Alpha angle
    std::tuple<float, float, float> Knee_Forward_Kinematics(float dLineMotorLen);
    // input: dAlpha rad; output: dLineMotorLen mm
    std::pair<float, float> Knee_Inverse_Kinematics(float dAlpha);
    std::pair<float, float> Knee_Velocity_Jacobi(float dAlpha);
    std::pair<float, float> Knee_Velocity_Jacobi_Analytical(float dLineMotorLen);

private:
    // new parameters
    float l_init = 219.81;  //275.04
    float l_AB = 50.0;  //60.0;
    float l_BC = 50.0;
    float l_CD = 70.0;
    float l_AD = 107.0;  //60.0;
    float l_AE = 107.0;  //60.0;
    float l_AF = 208.2051;  //95.0;?

    // new angles
    float Angle_init = 165 * M_PI / 180;    // angle ABC
    float Angle_BAF = 143.1 * M_PI / 180;
    float Angle_DAE = 14.5 * M_PI / 180;
    // 以上三个角度，定了初始位置之后即可确定。
    
};

#endif // KNEE_KINEMATICS_H