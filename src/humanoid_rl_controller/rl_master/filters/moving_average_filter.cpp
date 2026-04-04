#include "rl_master/filters/moving_average_filter.h"

namespace rl_master::filters
{

MovingAverageFilter::MovingAverageFilter(size_t window_size)
    : window_size_(window_size), sum_(0.0f)
{
}

float MovingAverageFilter::update(float new_value)
{
    buffer_.push_back(new_value);
    sum_ += new_value;

    if (buffer_.size() > window_size_)
    {
        sum_ -= buffer_.front();
        buffer_.pop_front();
    }

    return sum_ / static_cast<float>(buffer_.size());
}

void MovingAverageFilter::reset()
{
    buffer_.clear();
    sum_ = 0.0f;
}

} // namespace rl_master::filters
