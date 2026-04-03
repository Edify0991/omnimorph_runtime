#include "rl_master/math_tool.h"

std::vector<float> quaternion_to_euler_array(const std::vector<float>& quat) {
    float x = quat[0], y = quat[1], z = quat[2], w = quat[3];

    // Roll (x-axis rotation)
    float t0 = +2.0 * (w * x + y * z);
    float t1 = +1.0 - 2.0 * (x * x + y * y);
    float roll_x = std::atan2(t0, t1);

    // Pitch (y-axis rotation)
    float t2 = +2.0 * (w * y - z * x);
    t2 = std::clamp<float>(t2, -1.0, 1.0);
    float pitch_y = std::asin(t2);

    // Yaw (z-axis rotation)
    float t3 = +2.0 * (w * z + x * y);
    float t4 = +1.0 - 2.0 * (y * y + z * z);
    float yaw_z = std::atan2(t3, t4);

    return {roll_x, pitch_y, yaw_z};
}

std::vector<float> euler_to_quaternion(const std::vector<float>& rpy) {
    float roll = rpy[0], pitch = rpy[1], yaw = rpy[2];

    // 璁＄畻鍗婅
    float cy = std::cos(yaw * 0.5);
    float sy = std::sin(yaw * 0.5);
    float cp = std::cos(pitch * 0.5);
    float sp = std::sin(pitch * 0.5);
    float cr = std::cos(roll * 0.5);
    float sr = std::sin(roll * 0.5);

    // 鍥涘厓鏁板叕寮?
    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    return {qx, qy, qz, qw};
}

std::vector<float> limit_list_value(const std::vector<float>& list, const std::vector<std::vector<float>>& range) {
    std::vector<float> limit_list = list;
    size_t list_len = list.size();
    for (size_t i = 0; i < list_len; ++i) {
        limit_list[i] = std::clamp(list[i], range[i][0], range[i][1]);
    }
    return limit_list;
}
