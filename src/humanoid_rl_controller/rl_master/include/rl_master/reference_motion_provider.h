#ifndef RL_MASTER_REFERENCE_MOTION_PROVIDER_H
#define RL_MASTER_REFERENCE_MOTION_PROVIDER_H

#include <cstddef>
#include <string>
#include <vector>

class ReferenceMotionProvider
{
public:
    bool load(const std::string &file_path, int expected_dim);
    void clear();

    bool available() const { return loaded_ && !frames_.empty(); }
    int dim() const { return dim_; }
    size_t frameCount() const { return frames_.size(); }

    std::vector<float> sampleByPhase(double phase_t, double cycle_time, int expected_dim) const;
    std::vector<float> sampleByStep(size_t step_index, int expected_dim) const;

private:
    static std::vector<float> parseLine(const std::string &line);
    static std::vector<float> fitDim(const std::vector<float> &values, size_t dim);

    bool loaded_ = false;
    int dim_ = 0;
    std::vector<std::vector<float>> frames_;
};

#endif // RL_MASTER_REFERENCE_MOTION_PROVIDER_H
