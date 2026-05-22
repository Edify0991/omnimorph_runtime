#include "rl_master/kinematics/jc01/knee_kinematics.h"
#include "rl_master/kinematics/jc01/ankle_kinematics.h"
#include <iostream>
#include <tuple>

#define PRINT_TUPLE(t) \
    do { \
        auto [a,b,c] = t; \
        std::cout << "(" << a << ", " << b << ", " << c << ")" << std::endl; \
    } while(0)


int main() {
    // Ankle_Kinematics AnkleKinematics;
    // double leftMotorLen = 313.76;
    // double rightMotorLen = 305.76;
    // AnkleKinematics.Ankle_forward_Kinematics(leftMotorLen, rightMotorLen);

    // double pitch = -1.15;
    // double roll = -10.39;    
    // double pitch = -0.24*1;
    // double roll = 0.24*0;
    // auto result = AnkleKinematics.Ankle_inverse_Kinematics(pitch, roll, false);
    // std::cout<< "left: " << result.first << "   " << "right: " << result.second << std::endl;
    
    Knee_Kinematics kneekinematics;
    // double dlen = 23.1005;
    double dlen = 23.1005 * 0;
    // kneekinematics.Knee_Forward_Kinematics(dlen);
    PRINT_TUPLE(kneekinematics.Knee_Forward_Kinematics(dlen));

    double dalpha = 0.48;
    auto [dLineMotorLen, lineMotor_len] = kneekinematics.Knee_Inverse_Kinematics(dalpha);
    std::cout<< "dLineMotorLen: " << dLineMotorLen << std::endl;
 
    return 0;
}
