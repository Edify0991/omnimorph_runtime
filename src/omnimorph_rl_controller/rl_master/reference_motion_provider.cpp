#include "rl_master/reference_motion_provider.h"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
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

struct NpyArray
{
    std::string descr;
    bool fortran_order = false;
    std::vector<size_t> shape;
    std::vector<float> float_values;
    std::vector<int64_t> int64_values;
};

std::string toLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

bool parseFloatVectorNode(
    const YAML::Node &frame_node,
    const std::string &field_name,
    std::vector<float> *out,
    std::string *error)
{
    if (field_name.empty())
    {
        if (error)
        {
            *error = "field name is empty";
        }
        return false;
    }
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

bool hasNodeField(const YAML::Node &frame_node, const std::string &field_name)
{
    return !field_name.empty() && static_cast<bool>(frame_node[field_name]);
}

bool requireStructuredField(
    const YAML::Node &frame_node,
    const std::string &field_name,
    const std::vector<float> &values,
    size_t frame_index,
    const char *label)
{
    if (!hasNodeField(frame_node, field_name))
    {
        std::cerr << "[ReferenceMotionProvider] frame[" << frame_index
                  << "] missing required field '" << field_name
                  << "' for " << label << "." << std::endl;
        return false;
    }
    if (values.empty())
    {
        std::cerr << "[ReferenceMotionProvider] frame[" << frame_index
                  << "] required field '" << field_name
                  << "' for " << label << " is empty." << std::endl;
        return false;
    }
    return true;
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

uint16_t readLe16(const std::vector<unsigned char> &bytes, size_t offset)
{
    return static_cast<uint16_t>(bytes[offset]) |
           static_cast<uint16_t>(static_cast<uint16_t>(bytes[offset + 1]) << 8);
}

uint32_t readLe32(const std::vector<unsigned char> &bytes, size_t offset)
{
    return static_cast<uint32_t>(bytes[offset]) |
           (static_cast<uint32_t>(bytes[offset + 1]) << 8) |
           (static_cast<uint32_t>(bytes[offset + 2]) << 16) |
           (static_cast<uint32_t>(bytes[offset + 3]) << 24);
}

uint64_t readLe64(const unsigned char *ptr)
{
    uint64_t value = 0;
    for (size_t i = 0; i < 8; ++i)
    {
        value |= static_cast<uint64_t>(ptr[i]) << (8 * i);
    }
    return value;
}

bool parseZip64LocalExtraSizes(
    const std::vector<unsigned char> &bytes,
    size_t extra_offset,
    size_t extra_len,
    bool need_uncompressed_size,
    bool need_compressed_size,
    uint64_t *uncompressed_size,
    uint64_t *compressed_size)
{
    size_t cursor = extra_offset;
    const size_t extra_end = extra_offset + extra_len;
    while (cursor + 4 <= extra_end && cursor + 4 <= bytes.size())
    {
        const uint16_t header_id = readLe16(bytes, cursor);
        const uint16_t data_size = readLe16(bytes, cursor + 2);
        cursor += 4;
        if (cursor + data_size > extra_end || cursor + data_size > bytes.size())
        {
            return false;
        }
        if (header_id == 0x0001U)
        {
            size_t field_offset = cursor;
            if (need_uncompressed_size)
            {
                if (field_offset + 8 > cursor + data_size)
                {
                    return false;
                }
                if (uncompressed_size)
                {
                    *uncompressed_size = readLe64(bytes.data() + field_offset);
                }
                field_offset += 8;
            }
            if (need_compressed_size)
            {
                if (field_offset + 8 > cursor + data_size)
                {
                    return false;
                }
                if (compressed_size)
                {
                    *compressed_size = readLe64(bytes.data() + field_offset);
                }
                field_offset += 8;
            }
            return true;
        }
        cursor += data_size;
    }
    return !need_uncompressed_size && !need_compressed_size;
}

std::string trim(std::string text)
{
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

bool parseNpyHeaderString(const std::string &header, NpyArray *array, std::string *error)
{
    if (!array)
    {
        return false;
    }

    const auto descr_key = header.find("'descr'");
    const auto fortran_key = header.find("'fortran_order'");
    const auto shape_key = header.find("'shape'");
    if (descr_key == std::string::npos || fortran_key == std::string::npos || shape_key == std::string::npos)
    {
        if (error)
        {
            *error = "npy header is missing descr/fortran_order/shape";
        }
        return false;
    }

    const auto descr_colon = header.find(':', descr_key);
    const auto descr_quote_1 = header.find_first_of("'\"", descr_colon + 1);
    const auto descr_quote_2 = header.find_first_of("'\"", descr_quote_1 + 1);
    if (descr_colon == std::string::npos || descr_quote_1 == std::string::npos || descr_quote_2 == std::string::npos)
    {
        if (error)
        {
            *error = "npy descr field is malformed";
        }
        return false;
    }
    array->descr = header.substr(descr_quote_1 + 1, descr_quote_2 - descr_quote_1 - 1);

    const auto fortran_colon = header.find(':', fortran_key);
    const auto fortran_comma = header.find(',', fortran_colon + 1);
    const std::string fortran_value = trim(header.substr(fortran_colon + 1, fortran_comma - fortran_colon - 1));
    array->fortran_order = (fortran_value == "True");
    if (array->fortran_order)
    {
        if (error)
        {
            *error = "fortran_order=True npy arrays are not supported";
        }
        return false;
    }

    const auto shape_paren_1 = header.find('(', shape_key);
    const auto shape_paren_2 = header.find(')', shape_paren_1 + 1);
    if (shape_paren_1 == std::string::npos || shape_paren_2 == std::string::npos)
    {
        if (error)
        {
            *error = "npy shape field is malformed";
        }
        return false;
    }

    array->shape.clear();
    std::stringstream shape_stream(header.substr(shape_paren_1 + 1, shape_paren_2 - shape_paren_1 - 1));
    std::string item;
    while (std::getline(shape_stream, item, ','))
    {
        item = trim(item);
        if (item.empty())
        {
            continue;
        }
        try
        {
            const unsigned long long dim = std::stoull(item);
            array->shape.push_back(static_cast<size_t>(dim));
        }
        catch (const std::exception &)
        {
            if (error)
            {
                *error = "npy shape contains non-integer dimension '" + item + "'";
            }
            return false;
        }
    }
    if (array->shape.empty())
    {
        if (error)
        {
            *error = "npy shape is empty";
        }
        return false;
    }
    return true;
}

bool parseNpyArray(const std::vector<unsigned char> &bytes, NpyArray *array, std::string *error)
{
    if (!array || bytes.size() < 10)
    {
        if (error)
        {
            *error = "npy payload is too small";
        }
        return false;
    }
    const unsigned char kMagic[] = {0x93, 'N', 'U', 'M', 'P', 'Y'};
    if (!std::equal(std::begin(kMagic), std::end(kMagic), bytes.begin()))
    {
        if (error)
        {
            *error = "npy magic mismatch";
        }
        return false;
    }

    const unsigned char major = bytes[6];
    size_t header_len = 0;
    size_t header_offset = 0;
    if (major == 1)
    {
        header_len = readLe16(bytes, 8);
        header_offset = 10;
    }
    else if (major == 2 || major == 3)
    {
        if (bytes.size() < 12)
        {
            if (error)
            {
                *error = "npy v2/v3 payload is too small";
            }
            return false;
        }
        header_len = readLe32(bytes, 8);
        header_offset = 12;
    }
    else
    {
        if (error)
        {
            *error = "unsupported npy version";
        }
        return false;
    }
    if (header_offset + header_len > bytes.size())
    {
        if (error)
        {
            *error = "npy header extends beyond payload";
        }
        return false;
    }

    const std::string header(
        reinterpret_cast<const char *>(bytes.data() + header_offset),
        header_len);
    if (!parseNpyHeaderString(header, array, error))
    {
        return false;
    }

    size_t element_count = 1;
    for (const size_t dim : array->shape)
    {
        if (dim == 0 || element_count > std::numeric_limits<size_t>::max() / dim)
        {
            if (error)
            {
                *error = "npy shape element count overflow";
            }
            return false;
        }
        element_count *= dim;
    }

    const size_t data_offset = header_offset + header_len;
    if (array->descr == "<f4" || array->descr == "|f4")
    {
        const size_t byte_count = element_count * sizeof(float);
        if (data_offset + byte_count > bytes.size())
        {
            if (error)
            {
                *error = "npy float32 data extends beyond payload";
            }
            return false;
        }
        array->float_values.resize(element_count);
        std::memcpy(array->float_values.data(), bytes.data() + data_offset, byte_count);
        return true;
    }
    if (array->descr == "<i8" || array->descr == "|i8")
    {
        const size_t byte_count = element_count * sizeof(int64_t);
        if (data_offset + byte_count > bytes.size())
        {
            if (error)
            {
                *error = "npy int64 data extends beyond payload";
            }
            return false;
        }
        array->int64_values.resize(element_count);
        for (size_t i = 0; i < element_count; ++i)
        {
            const uint64_t raw = readLe64(bytes.data() + data_offset + i * sizeof(int64_t));
            array->int64_values[i] = static_cast<int64_t>(raw);
        }
        return true;
    }

    if (error)
    {
        *error = "unsupported npy dtype '" + array->descr + "'";
    }
    return false;
}

bool loadUncompressedNpz(
    const std::string &file_path,
    std::map<std::string, NpyArray> *arrays,
    std::string *error)
{
    if (!arrays)
    {
        return false;
    }
    std::ifstream fin(file_path, std::ios::binary);
    if (!fin.is_open())
    {
        return false;
    }
    fin.seekg(0, std::ios::end);
    const std::streamoff file_size = fin.tellg();
    if (file_size <= 0)
    {
        return false;
    }
    fin.seekg(0, std::ios::beg);
    std::vector<unsigned char> bytes(static_cast<size_t>(file_size));
    fin.read(reinterpret_cast<char *>(bytes.data()), file_size);
    if (!fin)
    {
        if (error)
        {
            *error = "failed to read npz file";
        }
        return false;
    }

    arrays->clear();
    size_t offset = 0;
    while (offset + 30 <= bytes.size())
    {
        const uint32_t signature = readLe32(bytes, offset);
        if (signature != 0x04034b50U)
        {
            break;
        }
        const uint16_t flags = readLe16(bytes, offset + 6);
        const uint16_t compression = readLe16(bytes, offset + 8);
        uint64_t compressed_size = readLe32(bytes, offset + 18);
        uint64_t uncompressed_size = readLe32(bytes, offset + 22);
        const uint16_t name_len = readLe16(bytes, offset + 26);
        const uint16_t extra_len = readLe16(bytes, offset + 28);
        const size_t name_offset = offset + 30;
        const size_t extra_offset = name_offset + name_len;
        const size_t data_offset = name_offset + name_len + extra_len;
        const bool need_zip64_uncompressed = uncompressed_size == 0xFFFFFFFFULL;
        const bool need_zip64_compressed = compressed_size == 0xFFFFFFFFULL;
        if (need_zip64_uncompressed || need_zip64_compressed)
        {
            if (!parseZip64LocalExtraSizes(
                    bytes,
                    extra_offset,
                    extra_len,
                    need_zip64_uncompressed,
                    need_zip64_compressed,
                    &uncompressed_size,
                    &compressed_size))
            {
                if (error)
                {
                    *error = "npz ZIP64 local extra field is invalid";
                }
                return false;
            }
        }
        if (data_offset > bytes.size() || data_offset + compressed_size > bytes.size())
        {
            if (error)
            {
                *error = "npz local file header is out of range";
            }
            return false;
        }
        if ((flags & 0x0008U) != 0)
        {
            if (error)
            {
                *error = "npz files with data descriptors are not supported";
            }
            return false;
        }
        if (compression != 0 || compressed_size != uncompressed_size)
        {
            if (error)
            {
                *error = "only uncompressed npz entries are supported";
            }
            return false;
        }

        std::string name(
            reinterpret_cast<const char *>(bytes.data() + name_offset),
            name_len);
        if (name.size() > 4 && name.substr(name.size() - 4) == ".npy")
        {
            std::vector<unsigned char> npy(
                bytes.begin() + static_cast<std::ptrdiff_t>(data_offset),
                bytes.begin() + static_cast<std::ptrdiff_t>(data_offset + compressed_size));
            NpyArray array;
            if (!parseNpyArray(npy, &array, error))
            {
                if (error)
                {
                    *error = "failed to parse '" + name + "': " + *error;
                }
                return false;
            }
            arrays->emplace(name.substr(0, name.size() - 4), std::move(array));
        }
        offset = data_offset + compressed_size;
    }
    return !arrays->empty();
}

bool requireNpzArray(
    const std::map<std::string, NpyArray> &arrays,
    const std::string &key,
    bool required,
    const NpyArray **out,
    std::string *error)
{
    if (out)
    {
        *out = nullptr;
    }
    if (key.empty())
    {
        if (required && error)
        {
            *error = "required npz key is empty";
        }
        return !required;
    }
    const auto it = arrays.find(key);
    if (it == arrays.end())
    {
        if (required && error)
        {
            *error = "npz is missing required key '" + key + "'";
        }
        return !required;
    }
    if (out)
    {
        *out = &it->second;
    }
    return true;
}

std::vector<float> sliceFrame(
    const NpyArray *array,
    size_t frame_index)
{
    if (!array || array->shape.empty() || array->float_values.empty() || frame_index >= array->shape[0])
    {
        return {};
    }
    size_t stride = 1;
    for (size_t i = 1; i < array->shape.size(); ++i)
    {
        stride *= array->shape[i];
    }
    const size_t offset = frame_index * stride;
    if (offset + stride > array->float_values.size())
    {
        return {};
    }
    return std::vector<float>(
        array->float_values.begin() + static_cast<std::ptrdiff_t>(offset),
        array->float_values.begin() + static_cast<std::ptrdiff_t>(offset + stride));
}

bool validateNpzFloatArray(
    const NpyArray *array,
    const std::string &key,
    size_t min_rank,
    std::string *error)
{
    if (!array)
    {
        return true;
    }
    if (array->float_values.empty() || array->shape.size() < min_rank)
    {
        if (error)
        {
            *error = "npz key '" + key + "' has invalid shape or dtype";
        }
        return false;
    }
    return true;
}
} // namespace

bool ReferenceMotionProvider::load(
    const std::string &file_path,
    int expected_dim,
    const ReferenceFeatureRequirements &requirements,
    const ReferenceMotionFieldMap &field_map,
    const std::string &body_quat_format_override)
{
    clear();

    if (file_path.empty())
    {
        return false;
    }

    const std::string extension = toLower(std::filesystem::path(file_path).extension().string());
    const bool prefer_structured = extension == ".yaml" || extension == ".yml" || extension == ".json";
    const bool prefer_npz = extension == ".npz";

    if (loadStructuredFile(file_path, expected_dim, requirements, field_map, body_quat_format_override))
    {
        return true;
    }

    if (prefer_structured)
    {
        return false;
    }

    if (loadNpzFile(file_path, expected_dim, requirements, field_map, body_quat_format_override))
    {
        return true;
    }

    if (prefer_npz)
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
    if (dim > 0 && !frame.reference_motion.empty())
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
    const ReferenceFeatureRequirements &requirements,
    const ReferenceMotionFieldMap &field_map,
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

            if (!parseFloatVectorNode(frame_node, field_map.reference_motion_key, &frame.reference_motion, &error) ||
                !parseFloatVectorNode(frame_node, field_map.joint_pos_key, &frame.joint_pos, &error) ||
                !parseFloatVectorNode(frame_node, field_map.joint_vel_key, &frame.joint_vel, &error) ||
                !parseFloatVectorNode(frame_node, field_map.body_pos_w_key, &frame.body_pos_w, &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }

            std::vector<float> raw_quat_w;
            if (!parseFloatVectorNode(frame_node, field_map.body_quat_w_key, &raw_quat_w, &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }
            if (!convertQuatVectorToXyzw(raw_quat_w, metadata.body_quat_format, &frame.body_quat_w, &error))
            {
                std::cerr << "[ReferenceMotionProvider] frame[" << frame_index << "] " << error << std::endl;
                return false;
            }

            if (requirements.reference_motion &&
                !requireStructuredField(
                    frame_node,
                    field_map.reference_motion_key,
                    frame.reference_motion,
                    frame_index,
                    "reference_motion"))
            {
                return false;
            }
            if (requirements.reference_joint_pos &&
                !requireStructuredField(
                    frame_node,
                    field_map.joint_pos_key,
                    frame.joint_pos,
                    frame_index,
                    "reference_joint_pos"))
            {
                return false;
            }
            if (requirements.reference_joint_vel &&
                !requireStructuredField(
                    frame_node,
                    field_map.joint_vel_key,
                    frame.joint_vel,
                    frame_index,
                    "reference_joint_vel"))
            {
                return false;
            }
            if (requirements.reference_body_pos_w &&
                !requireStructuredField(
                    frame_node,
                    field_map.body_pos_w_key,
                    frame.body_pos_w,
                    frame_index,
                    "reference_body_pos_w"))
            {
                return false;
            }
            if (requirements.reference_body_quat_w &&
                !requireStructuredField(
                    frame_node,
                    field_map.body_quat_w_key,
                    frame.body_quat_w,
                    frame_index,
                    "reference_body_quat_w"))
            {
                return false;
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

        if (parsed_frames.empty())
        {
            return false;
        }

        if (!metadata.body_names.empty() && inferred_body_count > 0 && metadata.body_names.size() != inferred_body_count)
        {
            std::cerr << "[ReferenceMotionProvider] body_names count (" << metadata.body_names.size()
                      << ") mismatches frame body count (" << inferred_body_count << ")." << std::endl;
            return false;
        }

        frames_.clear();
        structured_frames_.clear();
        frames_.reserve(parsed_frames.size());
        structured_frames_.reserve(parsed_frames.size());
        for (auto &frame : parsed_frames)
        {
            if (!frame.reference_motion.empty() && resolved_dim > 0)
            {
                frame.reference_motion = fitDim(frame.reference_motion, static_cast<size_t>(resolved_dim));
            }
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

bool ReferenceMotionProvider::loadNpzFile(
    const std::string &file_path,
    int expected_dim,
    const ReferenceFeatureRequirements &requirements,
    const ReferenceMotionFieldMap &field_map,
    const std::string &body_quat_format_override)
{
    std::map<std::string, NpyArray> arrays;
    std::string error;
    if (!loadUncompressedNpz(file_path, &arrays, &error))
    {
        if (!error.empty())
        {
            std::cerr << "[ReferenceMotionProvider] npz load failed: " << error << std::endl;
        }
        return false;
    }

    const NpyArray *reference_motion_array = nullptr;
    const NpyArray *joint_pos_array = nullptr;
    const NpyArray *joint_vel_array = nullptr;
    const NpyArray *body_pos_w_array = nullptr;
    const NpyArray *body_quat_w_array = nullptr;
    const bool has_reference_motion_key =
        !field_map.reference_motion_key.empty() &&
        arrays.find(field_map.reference_motion_key) != arrays.end();
    if (!requireNpzArray(arrays, field_map.reference_motion_key, requirements.reference_motion, &reference_motion_array, &error) ||
        !requireNpzArray(arrays, field_map.joint_pos_key, requirements.reference_joint_pos, &joint_pos_array, &error) ||
        !requireNpzArray(arrays, field_map.joint_vel_key, requirements.reference_joint_vel, &joint_vel_array, &error) ||
        !requireNpzArray(arrays, field_map.body_pos_w_key, requirements.reference_body_pos_w, &body_pos_w_array, &error) ||
        !requireNpzArray(arrays, field_map.body_quat_w_key, requirements.reference_body_quat_w, &body_quat_w_array, &error))
    {
        std::cerr << "[ReferenceMotionProvider] " << error << std::endl;
        return false;
    }
    if (!has_reference_motion_key)
    {
        reference_motion_array = nullptr;
    }

    if (!validateNpzFloatArray(reference_motion_array, field_map.reference_motion_key, 2, &error) ||
        !validateNpzFloatArray(joint_pos_array, field_map.joint_pos_key, 2, &error) ||
        !validateNpzFloatArray(joint_vel_array, field_map.joint_vel_key, 2, &error) ||
        !validateNpzFloatArray(body_pos_w_array, field_map.body_pos_w_key, 3, &error) ||
        !validateNpzFloatArray(body_quat_w_array, field_map.body_quat_w_key, 3, &error))
    {
        std::cerr << "[ReferenceMotionProvider] " << error << std::endl;
        return false;
    }

    size_t frame_count = 0;
    auto absorb_frame_count = [&](const NpyArray *array, const std::string &key) -> bool {
        if (!array)
        {
            return true;
        }
        if (array->shape.empty() || array->shape[0] == 0)
        {
            error = "npz key '" + key + "' has no frames";
            return false;
        }
        if (frame_count == 0)
        {
            frame_count = array->shape[0];
            return true;
        }
        if (frame_count != array->shape[0])
        {
            error = "npz key '" + key + "' frame count mismatch";
            return false;
        }
        return true;
    };
    if (!absorb_frame_count(reference_motion_array, field_map.reference_motion_key) ||
        !absorb_frame_count(joint_pos_array, field_map.joint_pos_key) ||
        !absorb_frame_count(joint_vel_array, field_map.joint_vel_key) ||
        !absorb_frame_count(body_pos_w_array, field_map.body_pos_w_key) ||
        !absorb_frame_count(body_quat_w_array, field_map.body_quat_w_key))
    {
        std::cerr << "[ReferenceMotionProvider] " << error << std::endl;
        return false;
    }
    if (frame_count == 0)
    {
        return false;
    }

    ReferenceMotionMetadata metadata;
    metadata.structured_file = true;
    metadata.source_format = "npz";
    metadata.body_quat_format = !body_quat_format_override.empty() ? body_quat_format_override : "wxyz";
    const auto fps_it = arrays.find("fps");
    if (fps_it != arrays.end())
    {
        double fps = 0.0;
        if (!fps_it->second.int64_values.empty())
        {
            fps = static_cast<double>(fps_it->second.int64_values.front());
        }
        else if (!fps_it->second.float_values.empty())
        {
            fps = static_cast<double>(fps_it->second.float_values.front());
        }
        if (fps > 1.0e-6)
        {
            metadata.frame_dt = 1.0 / fps;
            metadata.cycle_time = metadata.frame_dt * static_cast<double>(frame_count);
        }
    }

    std::vector<ReferenceMotionFrame> parsed_frames;
    parsed_frames.reserve(std::min(frame_count, kMaxReferenceFrames));
    int resolved_dim = std::max(expected_dim, 0);
    for (size_t frame_index = 0; frame_index < frame_count && frame_index < kMaxReferenceFrames; ++frame_index)
    {
        ReferenceMotionFrame frame;
        frame.reference_motion = sliceFrame(reference_motion_array, frame_index);
        frame.joint_pos = sliceFrame(joint_pos_array, frame_index);
        frame.joint_vel = sliceFrame(joint_vel_array, frame_index);
        frame.body_pos_w = sliceFrame(body_pos_w_array, frame_index);

        std::vector<float> raw_quat_w = sliceFrame(body_quat_w_array, frame_index);
        if (!convertQuatVectorToXyzw(raw_quat_w, metadata.body_quat_format, &frame.body_quat_w, &error))
        {
            std::cerr << "[ReferenceMotionProvider] npz frame[" << frame_index << "] " << error << std::endl;
            return false;
        }

        if (!finiteAndBounded(frame.reference_motion, kGenericAbsLimit, "reference_motion", &error) ||
            !finiteAndBounded(frame.joint_pos, kJointPosAbsLimit, "joint_pos", &error) ||
            !finiteAndBounded(frame.joint_vel, kJointVelAbsLimit, "joint_vel", &error) ||
            !finiteAndBounded(frame.body_pos_w, kBodyPosAbsLimit, "body_pos_w", &error) ||
            !finiteAndBounded(frame.body_quat_w, kGenericAbsLimit, "body_quat_w", &error))
        {
            std::cerr << "[ReferenceMotionProvider] npz frame[" << frame_index << "] " << error << std::endl;
            return false;
        }

        if (resolved_dim <= 0 && !frame.reference_motion.empty())
        {
            resolved_dim = static_cast<int>(frame.reference_motion.size());
        }
        parsed_frames.push_back(std::move(frame));
    }

    if (parsed_frames.empty())
    {
        return false;
    }

    frames_.clear();
    structured_frames_.clear();
    frames_.reserve(parsed_frames.size());
    structured_frames_.reserve(parsed_frames.size());
    for (auto &frame : parsed_frames)
    {
        if (!frame.reference_motion.empty() && resolved_dim > 0)
        {
            frame.reference_motion = fitDim(frame.reference_motion, static_cast<size_t>(resolved_dim));
        }
        frames_.push_back(frame.reference_motion);
        structured_frames_.push_back(std::move(frame));
    }

    dim_ = resolved_dim;
    metadata_ = std::move(metadata);
    loaded_ = !frames_.empty();
    return loaded_;
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
