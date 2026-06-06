#ifndef RL_MASTER_REFERENCE_MOTION_PROVIDER_H
#define RL_MASTER_REFERENCE_MOTION_PROVIDER_H

#include <cstddef>
#include <string>
#include <vector>

struct ReferenceMotionMetadata
{
    std::string source_format = "unknown";
    std::string anchor_body;
    std::vector<std::string> body_names;
    std::string body_quat_format = "wxyz"; // quaternion layout in source file (loaded data is converted to xyzw).
    double frame_dt = 0.0;
    double cycle_time = 0.0;
    bool structured_file = false;
};

struct ReferenceMotionFrame
{
    std::vector<float> reference_motion;
    std::vector<float> joint_pos;
    std::vector<float> joint_vel;
    std::vector<float> body_pos_w;
    // canonical quaternion order: [x, y, z, w] repeated per body.
    std::vector<float> body_quat_w;
};

struct ReferenceMotionFieldMap
{
    std::string reference_motion_key = "reference_motion";
    std::string joint_pos_key = "joint_pos";
    std::string joint_vel_key = "joint_vel";
    std::string body_pos_w_key = "body_pos_w";
    std::string body_quat_w_key = "body_quat_w";
};

struct ReferenceFeatureRequirements
{
    bool reference_motion = false;
    bool reference_joint_pos = false;
    bool reference_joint_vel = false;
    bool reference_body_pos_w = false;
    bool reference_body_quat_w = false;
    bool named_body_layout = false;

    bool any() const
    {
        return reference_motion || reference_joint_pos || reference_joint_vel ||
               reference_body_pos_w || reference_body_quat_w || named_body_layout;
    }

    bool sourceAny() const
    {
        return reference_motion || reference_joint_pos || reference_joint_vel ||
               reference_body_pos_w || reference_body_quat_w;
    }
};

class ReferenceMotionProvider
{
public:
    bool load(
        const std::string &file_path,
        int expected_dim,
        const ReferenceFeatureRequirements &requirements = {},
        const ReferenceMotionFieldMap &field_map = {},
        const std::string &body_quat_format_override = "");
    void clear();

    bool available() const { return loaded_ && !frames_.empty(); }
    bool hasStructuredFrames() const { return available() && !structured_frames_.empty(); }
    int dim() const { return dim_; }
    size_t frameCount() const { return frames_.size(); }
    const ReferenceMotionMetadata &metadata() const { return metadata_; }

    std::vector<float> sampleByPhase(double phase_t, double cycle_time, int expected_dim) const;
    std::vector<float> sampleByStep(size_t step_index, int expected_dim) const;
    ReferenceMotionFrame sampleFrameByPhase(double phase_t, double cycle_time, int expected_dim) const;
    ReferenceMotionFrame sampleFrameByStep(size_t step_index, int expected_dim) const;

private:
    static size_t sampleIndexByPhase(size_t frame_count, double phase_t, double cycle_time);
    bool loadStructuredFile(
        const std::string &file_path,
        int expected_dim,
        const ReferenceFeatureRequirements &requirements,
        const ReferenceMotionFieldMap &field_map,
        const std::string &body_quat_format_override);
    bool loadNpzFile(
        const std::string &file_path,
        int expected_dim,
        const ReferenceFeatureRequirements &requirements,
        const ReferenceMotionFieldMap &field_map,
        const std::string &body_quat_format_override);
    bool loadLegacyTextFile(const std::string &file_path, int expected_dim);
    static std::vector<float> parseLine(const std::string &line);
    static std::vector<float> fitDim(const std::vector<float> &values, size_t dim);

    bool loaded_ = false;
    int dim_ = 0;
    ReferenceMotionMetadata metadata_;
    std::vector<std::vector<float>> frames_;
    std::vector<ReferenceMotionFrame> structured_frames_;
};

#endif // RL_MASTER_REFERENCE_MOTION_PROVIDER_H
