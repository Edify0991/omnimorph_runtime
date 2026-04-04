#ifndef RL_MASTER_FILTERS_MOVING_AVERAGE_FILTER_H
#define RL_MASTER_FILTERS_MOVING_AVERAGE_FILTER_H

#include <cstddef>
#include <deque>

namespace rl_master::filters
{

class MovingAverageFilter
{
public:
    explicit MovingAverageFilter(size_t window_size = 5);

    float update(float new_value);
    void reset();

private:
    std::deque<float> buffer_;
    size_t window_size_;
    float sum_;
};

} // namespace rl_master::filters

#endif // RL_MASTER_FILTERS_MOVING_AVERAGE_FILTER_H
