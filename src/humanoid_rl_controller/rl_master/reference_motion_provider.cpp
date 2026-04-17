#include "rl_master/reference_motion_provider.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace
{
constexpr size_t kMaxReferenceFrames = 200000;
constexpr float kGenericAbsLimit = 1.0e6f;
constexpr float kJointPosAbsLimit = 8.0f;
constexpr float kJointVelAbsLimit = 120.0f;
constexpr float kBodyPosAbsLimit = 20.0f;
constexpr float kQuatNormMin = 1.0e-5f;
constexpr float kQuatNormDeviationWarn = 0.2f;
constexpr float kQuatNormDeviationMax = 0.5f;

std::string toLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

bool parseFloatVectorNode(
    const YAML::Node &frame_node,
    const char *field_name,
    std::vector<float> *out,
    std::string *error)
{
    if (!frame_node[field_name])
    {
        out->clear();
        return true;
    }
    try
    {
        *out = frame_node[field_name].as<std::vector<float>>();
        return true;
    }
    catch (const std::exception &e)
    {
        if (error)
        {
            *error = std::string("invalid field '") + field_name + "': " + e.what();
        }
        return false;
    }
}

bool finiteAndBounded(
    const std::vector<float> &values,
    float abs_limit,
    std::string field_name,
    std::string *error)
{
    for (size_t i = 0; i < values.size(); ++i)
    {
        const float value = values[i];
        if (!std::isfinite(value))
        {
            if (error)
            {
                *error = field_name + "[" + std::to_string(i) + "] is NaN/Inf";
            }
            return false;
        }
        if (std::fabs(value) > abs_limit)
        {
            if (error)
            {
                *error = field_name + "[" + std::to_string(i) + "] exceeds abs limit " +
                         std::to_string(abs_limit) + ", value=" + std::to_string(value);
            }
            return false;
        }
    }
    return true;
}

bool convertQuatVectorToXyzw(
    const std::vector<float> &raw_values,
    const std::string &format_raw,
    std::vector<float> *out,
    std::string *error)
{
    const std::string format = toLower(format_raw);
    if (raw_values.empty())
    {
        out->clear();
        return true;
    }
    if (raw_values.size() % 4 != 0)
    {
        if (error)
        {
            *error = "body_quat_w length must be multiple of 4";
        }
        return false;
    }

    out->assign(raw_values.size(), 0.0f);
    for (size_t i = 0; i < raw_values.size(); i += 4)
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float w = 1.0f;
        if (format == "wxyz")
        {
            w = raw_values[i + 0];
            x = raw_values[i + 1];
            y = raw_values[i + 2];
            z = raw_values[i + 3];
        }
        else if (format == "xyzw")
        {
            x = raw_values[i + 0];
            y = raw_values[i + 1];
            z = raw_values[i + 2];
            w = raw_values[i + 3];
        }
        else
        {
            if (error)
            {
                *error = "unsupported quat format '" + format_raw + "', expected 'wxyz' or 'xyzw'";
            }
            return false;
        }

        const float norm = std::sqrt(x * x + y * y + z * z + w * w);
        if (!std::isfinite(norm) || norm < kQuatNormMin)
        {
            if (error)
            {
                *error = "body_quat_w contains invalid zero quaternion";
            }
            return false;
        }
        const float deviation = std::fabs(norm - 1.0f);
        if (deviation > kQuatNormDeviationMax)
        {
            if (error)
            {
                *error = "body_quat_w normalization deviation too large: " + std::to_string(deviation);
            }
            return false;
        }
        if (deviation > kQuatNormDeviationWarn)
        {
            std::cerr << "[ReferenceMotionProvider] warning: renormalize quaternion with deviation "
                      << deviation << std::endl;
        }

        (*out)[i + 0] = x / norm;
        (*out)[i + 1] = y / norm;
        (*out)[i + 2] = z / norm;
        (*out)[i + 3] = w / norm;
    }
    return true;
}

ReferenceMotionFrame emptyFrame(int expected_dim)
{
    ReferenceMotionFrame frame;
    if (expected_dim > 0)
    {
        frame.reference_motion.assign(static_cast<size_t>(expected_dim), 0.0f);
    }
    return frame;
}
} // namespace

bool ReferenceMotionProvider::load(
    const std::string &file_path,
    int expected_dim,
    const std::string &body_quat_format_override)
{
    clear();

    if (file_path.empty())
    {
        return false;
    }

    const std::string extension = toLower(std::filesystem::path(file_path).extension().string());
    const bool prefer_structured = extension == ".yaml" || extension == ".yml" || extension == ".json";

    if (loadStructuredFile(file_path, expected_dim, body_quat_format_override))
    {
        return true;
    }

    if (prefer_structured)
    {
        return false;
    }

    if (loadLegacyTextFile(file_path, expected_dim))
    {
        return true;
    }

    clear();
    return false;
}

void ReferenceMotionProvider::clear()
{
    loaded_ = false;
    dim_ = 0;
    metadata_ = ReferenceMotionMetadata{};
    frames_.clear();
    structured_frames_.clear();
}

std::vector<float> ReferenceMotionProvider::sampleByPhase(double phase_t, double cycle_time, int expected_dim) const
{
    return sampleFrameByPhase(phase_t, cycle_time, expected_dim).reference_motion;
}

std::vector<float> ReferenceMotionProvider::sampleByStep(size_t step_index, int expected_dim) const
{
    return sampleFrameByStep(step_index, expected_dim).reference_motion;
}

ReferenceMotionFrame ReferenceMotionProvider::sampleFrameByPhase(double phase_t, double cycle_time, int expected_dim) const
{
    if (!available())
    {
        return emptyFrame(expected_dim);
    }

    const size_t index = sampleIndexByPhase(frames_.size(), phase_t, cycle_time);
    return sampleFrameByStep(index, expected_dim);
}

ReferenceMotionFrame ReferenceMotionProvider::sampleFrameByStep(size_t step_index, int expected_dim) const
{
    if (!available())
    {
        return emptyFrame(expected_dim);
    }

    const size_t index = step_index % frames_.size();
    if (index >= frames_.size())
    {
        return emptyFrame(expected_dim);
    }

    ReferenceMotionFrame frame;
    if (!structured_frames_.empty() && index < structured_frames_.size())
    {
        frame = structured_frames_[index];
    }
    else
    {
        frame.reference_motion = frames_[index];
    }

    const int dim = expected_dim > 0 ? expected_dim : dim_;
    if (dim > 0)
    {
        frame.reference_motion = fitDim(frame.reference_motion, static_cast<size_t>(dim));
    }
    return frame;
}

size_t ReferenceMotionProvider::sampleIndexByPhase(size_t frame_count, double phase_t, double cycle_time)
{
    if (frame_count == 0)
    {
        return 0;
    }
    const double safe_cycle = std::max(1e-6, cycle_time);
    const double normalized = std::fmod(std::max(0.0, phase_t), safe_cycle) / safe_cycle;
    size_t index = static_cast<size_t>(normalized * static_cast<double>(frame_count));
    if (index >= frame_count)
    {
        index = frame_count - 1;
    }
    return index;
}

bool ReferenceMotionProvider::loadStructuredFile(
    const std::string &file_path,
    int expected_dim,
    const std::string &body_quat_format_override)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(file_path);
    }
    catch (const std::exception &)
    {
        return false;
    }

    const YAML::Node motion_root = root["reference_motion"] ? root["reference_motion"] : root;
    if (!motion_root.IsMap() || !motion_root["frames"] || !motion_root["frames"].IsSequence())
    {
        return false;
    }

    try
    {
        ReferenceMotionMetadata metadata;
        metadata.structured_file = true;
        metadata.source_format = motion_root["source_format"] ? motion_root["source_format"].as<std::string>() : "structured";
        metadata.anchor_body = motion_root["anchor_body"] ? motion_root["anchor_body"].as<std::string>() : "";
        metadata.body_quat_format = !body_quat_format_override.empty()
                                        ? body_quat_format_override
                                        : (motion_root["body_quat_format"] ? motion_root["body_quat_format"].as<std::string>() : "wxyz");
        if (motion_root["body_names"])
        {
            metadata.body_names = motion_root["body_names"].as<std::vector<std::string>>();
        }
        if (motion_root["frame_dt"])
        {
            metadata.frame_dt = motion_root["frame_dt"].as<double>();
        }
        else if (motion_root["fps"])
        {
            const double fps = motion_root["fps"].as<double>();
            if (fps > 1e-6)
            {
                metadata.frame_dt = 1.0 / fps;
            }
        }
        if (motion_root["cycle_time"])
        {
            metadata.cycle_time = motion_root["cycle_time"].as<double>();
        }

        std::vector<ReferenceMotionFrame> parsed_frames;
        parsed_frames.reserve(motion_root["frames"].size());
        int resolved_dim = std::max(expected_dim, 0);
        size_t inferred_body_count = metadata.body_names.size();

        for (size_t frame_index = 0; frame_index < motion_root["frames"].size(); ++frame_index)
        {
            if (frame_index >= kMaxReferenceFrames)
            {
                std::cerr << "[ReferenceMotionProvider] frame count exceeds limit " << kMaxReferenceFrames
                          << ", stop loading more frames." << std::endl;
                break;
            }

            const YAML::Node frame_node = motion_root["frames"][frame_index];
            if (!frame_node.IsMap())
            {
                continue;
            }

            ReferenceMotionFrame frame;
            std::string error;

            if (!parseFloatVectorNode(frame_node, "reference_motion", &frame.reference_motion, &error) ||
                !parseFloatVectorNode(frame_node, "joint_pos", &frame.joint_pos, &error) ||
                !parseFloatVectorNode(frame_node, "joint_vel", &frame.joint_vel, &error) ||
                !parseFloatVectorNode(frame_node, "body_pos_w", &frame.body_pos_w, &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }

            std::vector<float> raw_quat_w;
            if (!parseFloatVectorNode(frame_node, "body_quat_w", &raw_quat_w, &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }
            if (!convertQuatVectorToXyzw(raw_quat_w, metadata.body_quat_format, &frame.body_quat_w, &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }

            if (frame.reference_motion.empty() && !frame.joint_pos.empty() && !frame.joint_vel.empty())
            {
                frame.reference_motion.reserve(frame.joint_pos.size() + frame.joint_vel.size());
                frame.reference_motion.insert(frame.reference_motion.end(), frame.joint_pos.begin(), frame.joint_pos.end());
                frame.reference_motion.insert(frame.reference_motion.end(), frame.joint_vel.begin(), frame.joint_vel.end());
            }

            if (!frame.body_pos_w.empty() || !frame.body_quat_w.empty())
            {
                if (frame.body_pos_w.size() % 3 != 0)
                {
                    std::cerr << "[ReferenceMotionProvider] frame[" << frame_index
                              << "] body_pos_w length must be multiple of 3." << std::endl;
                    return false;
                }
                if (frame.body_quat_w.size() % 4 != 0)
                {
                    std::cerr << "[ReferenceMotionProvider] frame[" << frame_index
                              << "] body_quat_w length must be multiple of 4." << std::endl;
                    return false;
                }
                const size_t pos_body_count = frame.body_pos_w.size() / 3;
                const size_t quat_body_count = frame.body_quat_w.size() / 4;
                if (pos_body_count != quat_body_count)
                {
                    std::cerr << "[ReferenceMotionProvider] frame[" << frame_index
                              << "] body_pos_w/body_quat_w body count mismatch." << std::endl;
                    return false;
                }
                if (inferred_body_count == 0)
                {
                    inferred_body_count = pos_body_count;
                }
                if (inferred_body_count != pos_body_count)
                {
                    std::cerr << "[ReferenceMotionProvider] frame[" << frame_index
                              << "] body count changed across frames." << std::endl;
                    return false;
                }
            }

            if (!finiteAndBounded(frame.reference_motion, kGenericAbsLimit, "reference_motion", &error) ||
                !finiteAndBounded(frame.joint_pos, kJointPosAbsLimit, "joint_pos", &error) ||
                !finiteAndBounded(frame.joint_vel, kJointVelAbsLimit, "joint_vel", &error) ||
                !finiteAndBounded(frame.body_pos_w, kBodyPosAbsLimit, "body_pos_w", &error) ||
                !finiteAndBounded(frame.body_quat_w, kGenericAbsLimit, "body_quat_w", &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }

            if (resolved_dim <= 0 && !frame.reference_motion.empty())
            {
                resolved_dim = static_cast<int>(frame.reference_motion.size());
            }
            parsed_frames.push_back(std::move(frame));
        }

        if (parsed_frames.empty() || resolved_dim <= 0)
        {
            return false;
        }

        if (!metadata.body_names.empty() && inferred_body_count > 0 && metadata.body_names.size() != inferred_body_count)
        {
            std::cerr << "[ReferenceMotionProvider] body_names count (" << metadata.body_names.size()
                      << ") mismatches frame body count (" << inferred_body_count << ")." << std::endl;
            return false;
        }
        if (metadata.body_names.empty() && inferred_body_count > 0)
        {
            metadata.body_names.reserve(inferred_body_count);
            for (size_t i = 0; i < inferred_body_count; ++i)
            {
                metadata.body_names.push_back("body_" + std::to_string(i));
            }
        }

        frames_.clear();
        structured_frames_.clear();
        frames_.reserve(parsed_frames.size());
        structured_frames_.reserve(parsed_frames.size());
        for (auto &frame : parsed_frames)
        {
            frame.reference_motion = fitDim(frame.reference_motion, static_cast<size_t>(resolved_dim));
            frames_.push_back(frame.reference_motion);
            structured_frames_.push_back(std::move(frame));
        }

        dim_ = resolved_dim;
        metadata_ = std::move(metadata);
        loaded_ = !frames_.empty();
        return loaded_;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ReferenceMotionProvider] structured file parse failed: " << e.what() << std::endl;
        return false;
    }
}

bool ReferenceMotionProvider::loadLegacyTextFile(const std::string &file_path, int expected_dim)
{
    std::ifstream fin(file_path);
    if (!fin.is_open())
    {
        return false;
    }

    int resolved_dim = std::max(expected_dim, 0);
    std::vector<std::vector<float>> raw_frames;
    raw_frames.reserve(2048);

    std::string line;
    while (std::getline(fin, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::vector<float> frame = parseLine(line);
        if (frame.empty())
        {
            continue;
        }
        std::string error;
        if (!finiteAndBounded(frame, kGenericAbsLimit, "legacy_frame", &error))
        {
            std::cerr << "[ReferenceMotionProvider] " << error << std::endl;
            return false;
        }

        if (resolved_dim <= 0)
        {
            resolved_dim = static_cast<int>(frame.size());
        }
        raw_frames.push_back(std::move(frame));
        if (raw_frames.size() >= kMaxReferenceFrames)
        {
            std::cerr << "[ReferenceMotionProvider] frame count exceeds limit " << kMaxReferenceFrames
                      << ", stop loading more frames." << std::endl;
            break;
        }
    }

    if (raw_frames.empty() || resolved_dim <= 0)
    {
        return false;
    }

    frames_.clear();
    structured_frames_.clear();
    frames_.reserve(raw_frames.size());
    structured_frames_.reserve(raw_frames.size());
    for (const auto &raw_frame : raw_frames)
    {
        ReferenceMotionFrame frame;
        frame.reference_motion = fitDim(raw_frame, static_cast<size_t>(resolved_dim));
        frames_.push_back(frame.reference_motion);
        structured_frames_.push_back(std::move(frame));
    }

    dim_ = resolved_dim;
    metadata_ = ReferenceMotionMetadata{};
    metadata_.source_format = "legacy_text";
    loaded_ = !frames_.empty();
    return loaded_;
}

std::vector<float> ReferenceMotionProvider::parseLine(const std::string &line)
{
    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');
    std::replace(normalized.begin(), normalized.end(), '\t', ' ');

    std::istringstream iss(normalized);
    std::vector<float> values;
    float v = 0.0f;
    while (iss >> v)
    {
        values.push_back(v);
    }
    return values;
}

std::vector<float> ReferenceMotionProvider::fitDim(const std::vector<float> &values, size_t dim)
{
    std::vector<float> out(dim, 0.0f);
    const size_t copy_n = std::min(values.size(), dim);
    std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(copy_n), out.begin());
    return out;
}
