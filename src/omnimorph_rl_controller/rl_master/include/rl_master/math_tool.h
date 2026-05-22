#ifndef MATH_TOOL_H
#define MATH_TOOL_H

#include <vector>
#include <algorithm> // for std::clamp
#include <cmath>
#include <cstring>

std::vector<float> quaternion_to_euler_array(const std::vector<float>& quat);
std::vector<float> euler_to_quaternion(const std::vector<float>& rpy);
std::vector<float> limit_list_value(const std::vector<float>& list, const std::vector<std::vector<float>>& range);

class LowPassFilter {
private:
    float alpha;  // 鎴棰戠巼鐩稿叧鍙傛暟
    float y_last;
    bool initialized;

public:
    LowPassFilter(float cutoff_freq = 10.0, float sample_time = 0.001) 
        : y_last(0.0), initialized(false) {
        float RC = 1.0 / (2.0 * M_PI * cutoff_freq);
        alpha = sample_time / (RC + sample_time);
    }

    float update(float new_value) {
        if (!initialized) {
            y_last = new_value;
            initialized = true;
            return new_value;
        }
        
        float y = alpha * new_value + (1.0 - alpha) * y_last;
        y_last = y;
        return y;
    }

    void reset() {
        initialized = false;
        y_last = 0.0;
    }
};

#endif // MATH_TOOL_H
