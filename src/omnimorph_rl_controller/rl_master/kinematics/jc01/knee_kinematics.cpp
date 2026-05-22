#include "rl_master/kinematics/jc01/knee_kinematics.h"

// input: dLineMotorLen mm; output: dAlpha rad and dAlpha angle; Alpha angle
std::tuple<float, float, float> Knee_Kinematics::Knee_Forward_Kinematics(float dLineMotorLen) {
    
    auto start = std::chrono::high_resolution_clock::now();
    float lineMotor_len = l_init + dLineMotorLen;
    float Angle_EAF = std::acos((l_AF * l_AF + l_AE * l_AE - lineMotor_len * lineMotor_len) / (2 * l_AF * l_AE));
    float Angle_BAD = Angle_BAF - Angle_DAE - Angle_EAF;
    float l_BD = std::sqrt(l_AD * l_AD + l_AB * l_AB - 2 * l_AD * l_AB * std::cos(Angle_BAD));
    float Angle_ABD = std::acos((l_AB * l_AB + l_BD * l_BD - l_AD * l_AD) / (2 * l_AB * l_BD));
    float Angle_CBD = std::acos((l_BC * l_BC + l_BD * l_BD - l_CD * l_CD) / (2 * l_BC * l_BD));
    float Alpha = Angle_ABD + Angle_CBD;
    float dAlpha_rad = Alpha - Angle_init;
    float dAlpha_angle = dAlpha_rad * 180 / M_PI;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    // std::cout << "Knee_Forward_Kinematics computation time: " << elapsed.count() << " ms" << std::endl;

    return std::make_tuple(dAlpha_rad, dAlpha_angle, Alpha * 180 / M_PI);
}

// input: dAlpha rad; output: dLineMotorLen mm
std::pair<float, float> Knee_Kinematics::Knee_Inverse_Kinematics(float dAlpha) {

    auto start = std::chrono::high_resolution_clock::now();

    float Alpha_rad = dAlpha + Angle_init;
    float l_AC = std::sqrt(l_AB * l_AB + l_BC * l_BC - 2 * l_BC * l_AB * std::cos(Alpha_rad));
    float Angle_BAC = std::acos((l_AB * l_AB + l_AC * l_AC - l_BC * l_BC) / (2 * l_AB * l_AC));
    // std::cout << "Knee_Inverse_Kinematics Angle_BAC: " << Angle_BAC << std::endl;
    float Angle_DAC = std::acos((l_AC * l_AC + l_AD * l_AD - l_CD * l_CD) / (2 * l_AC * l_AD));
    float Angle_EAF = (dAlpha > 15 * M_PI / 180.0) ? (Angle_BAF + Angle_BAC - Angle_DAC - Angle_DAE) : (Angle_BAF - Angle_BAC - Angle_DAC - Angle_DAE);
    float lineMotor_len = std::sqrt(l_AE * l_AE + l_AF * l_AF - 2 * l_AE * l_AF * std::cos(Angle_EAF));
    float dLineMotorLen = lineMotor_len - l_init;

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    // std::cout << "Knee_Inverse_Kinematics computation time: " << elapsed.count() << " ms" << std::endl;

    return {dLineMotorLen, lineMotor_len};
    
}

// input: dAlpha rad; output: J_joint2motor mm/rad J_motor2joint rad/mm
std::pair<float, float> Knee_Kinematics::Knee_Velocity_Jacobi(float dAlpha) {

    float dq = 0.001;

    float dAlpha_pos = dAlpha + dq;
    float dAlpha_neg = dAlpha - dq;

    auto result_Alpha_pos = Knee_Inverse_Kinematics(dAlpha_pos);
    auto result_Alpha_neg = Knee_Inverse_Kinematics(dAlpha_neg);

    float J_joint2motor = (result_Alpha_pos.first - result_Alpha_neg.first) / (2 * dq);

    float J_motor2joint = 1 / J_joint2motor;

    return {J_joint2motor, J_motor2joint};
}

std::pair<float, float> Knee_Kinematics::Knee_Velocity_Jacobi_Analytical(float dLineMotorLen)
{
    // 1. 计算当前的绝对电机长度
    float lineMotor_len = l_init + dLineMotorLen;

    // ================= 前置几何参数计算 (与正解复用) =================
    float Angle_EAF = std::acos((l_AF * l_AF + l_AE * l_AE - lineMotor_len * lineMotor_len) / (2 * l_AF * l_AE));
    float Angle_BAD = Angle_BAF - Angle_DAE - Angle_EAF;
    float l_BD = std::sqrt(l_AD * l_AD + l_AB * l_AB - 2 * l_AD * l_AB * std::cos(Angle_BAD));
    
    float Angle_ABD = std::acos((l_AB * l_AB + l_BD * l_BD - l_AD * l_AD) / (2 * l_AB * l_BD));
    float Angle_CBD = std::acos((l_BC * l_BC + l_BD * l_BD - l_CD * l_CD) / (2 * l_BC * l_BD));

    // ================= 链式求导法计算偏导数 =================
    
    // (1) d(Angle_EAF) / d(L_motor)
    float sin_EAF = std::sin(Angle_EAF);
    float dAngleEAF_dLmotor = 0.0f;
    if (sin_EAF > 1e-6f) { // 防奇异保护
        dAngleEAF_dLmotor = lineMotor_len / (l_AF * l_AE * sin_EAF);
    }

    // (2) d(Angle_BAD) / d(Angle_EAF)
    float dAngleBAD_dAngleEAF = -1.0f;

    // (3) d(l_BD) / d(Angle_BAD)
    float dlBD_dAngleBAD = (l_AD * l_AB * std::sin(Angle_BAD)) / l_BD;

    // (4) d(Alpha) / d(l_BD) = d(Angle_ABD)/d(l_BD) + d(Angle_CBD)/d(l_BD)
    float sin_ABD = std::sin(Angle_ABD);
    float sin_CBD = std::sin(Angle_CBD);
    float dAngleABD_dlBD = 0.0f;
    float dAngleCBD_dlBD = 0.0f;
    
    if (sin_ABD > 1e-6f && sin_CBD > 1e-6f) { // 防奇异保护
        dAngleABD_dlBD = -(l_BD * l_BD + l_AD * l_AD - l_AB * l_AB) / (2.0f * l_AB * l_BD * l_BD * sin_ABD);
        dAngleCBD_dlBD = -(l_BD * l_BD + l_CD * l_CD - l_BC * l_BC) / (2.0f * l_BC * l_BD * l_BD * sin_CBD);
    }
    float dAlpha_dlBD = dAngleABD_dlBD + dAngleCBD_dlBD;

    // ================= 组合雅可比矩阵 =================

    // J_motor2joint = d(Alpha) / d(L_motor)
    float J_motor2joint = dAlpha_dlBD * dlBD_dAngleBAD * dAngleBAD_dAngleEAF * dAngleEAF_dLmotor;

    // J_joint2motor = d(L_motor) / d(Alpha) (即传动比的倒数)
    float J_joint2motor = 0.0f;
    if (std::abs(J_motor2joint) > 1e-6f) {
        J_joint2motor = 1.0f / J_motor2joint;
    }

    // 返回: {电机速度 -> 关节速度映射, 关节速度 -> 电机速度映射}
    return {J_joint2motor, J_motor2joint};
}
