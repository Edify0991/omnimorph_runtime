/*
onnxruntime:
https://blog.csdn.net/yangyu0515/article/details/142093965
https://blog.csdn.net/m0_57254760/article/details/138304321
*/
#include "rl_master/RL_controller.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <map>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "rl_master/rl_protocol.h"

namespace
{
struct Vec3
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Mat3
{
    std::array<float, 9> v{};
};

std::vector<double> toDoubleVector(const std::vector<float> &values)
{
    std::vector<double> out(values.size(), 0.0);
    for (size_t i = 0; i < values.size(); ++i)
    {
        out[i] = static_cast<double>(values[i]);
    }
    return out;
}

std::string toLowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

float meanOf(const std::vector<float> &values)
{
    if (values.empty())
    {
        return 0.0f;
    }
    float sum = 0.0f;
    for (const float v : values)
    {
        sum += v;
    }
    return sum / static_cast<float>(values.size());
}

std::vector<float> fitDim(const std::vector<float> &values, size_t dim)
{
    std::vector<float> out(dim, 0.0f);
    const size_t copy_n = std::min(values.size(), dim);
    std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(copy_n), out.begin());
    return out;
}

Mat3 makeIdentity()
{
    Mat3 out;
    out.v = {1.0f, 0.0f, 0.0f,
             0.0f, 1.0f, 0.0f,
             0.0f, 0.0f, 1.0f};
    return out;
}

Mat3 transpose(const Mat3 &m)
{
    Mat3 out;
    out.v = {m.v[0], m.v[3], m.v[6],
             m.v[1], m.v[4], m.v[7],
             m.v[2], m.v[5], m.v[8]};
    return out;
}

Mat3 multiply(const Mat3 &a, const Mat3 &b)
{
    Mat3 out = makeIdentity();
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            float value = 0.0f;
            for (int k = 0; k < 3; ++k)
            {
                value += a.v[static_cast<size_t>(r * 3 + k)] * b.v[static_cast<size_t>(k * 3 + c)];
            }
            out.v[static_cast<size_t>(r * 3 + c)] = value;
        }
    }
    return out;
}

Vec3 subtract(const Vec3 &a, const Vec3 &b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 rotate(const Mat3 &m, const Vec3 &v)
{
    return {
        m.v[0] * v.x + m.v[1] * v.y + m.v[2] * v.z,
        m.v[3] * v.x + m.v[4] * v.y + m.v[5] * v.z,
        m.v[6] * v.x + m.v[7] * v.y + m.v[8] * v.z};
}

float yawFromQuatXyzw(const std::vector<float> &quat_xyzw)
{
    if (quat_xyzw.size() < 4)
    {
        return 0.0f;
    }
    const float x = quat_xyzw[0];
    const float y = quat_xyzw[1];
    const float z = quat_xyzw[2];
    const float w = quat_xyzw[3];
    const float t3 = 2.0f * (w * z + x * y);
    const float t4 = 1.0f - 2.0f * (y * y + z * z);
    return std::atan2(t3, t4);
}

Mat3 yawRotation(float yaw)
{
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    Mat3 out;
    out.v = {c, -s, 0.0f,
             s, c, 0.0f,
             0.0f, 0.0f, 1.0f};
    return out;
}

Mat3 quatToRotXyzw(float x, float y, float z, float w)
{
    const float norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!std::isfinite(norm) || norm < 1e-8f)
    {
        return makeIdentity();
    }
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;

    Mat3 out;
    out.v = {
        1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - z * w), 2.0f * (x * z + y * w),
        2.0f * (x * y + z * w), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - x * w),
        2.0f * (x * z - y * w), 2.0f * (y * z + x * w), 1.0f - 2.0f * (x * x + y * y)};
    return out;
}

Mat3 quatToRotXyzw(const std::vector<float> &quat_xyzw)
{
    if (quat_xyzw.size() < 4)
    {
        return makeIdentity();
    }
    return quatToRotXyzw(quat_xyzw[0], quat_xyzw[1], quat_xyzw[2], quat_xyzw[3]);
}

bool extractVec3(const std::vector<float> &data, size_t index, Vec3 *out)
{
    const size_t offset = index * 3;
    if (!out || offset + 2 >= data.size())
    {
        return false;
    }
    out->x = data[offset + 0];
    out->y = data[offset + 1];
    out->z = data[offset + 2];
    return std::isfinite(out->x) && std::isfinite(out->y) && std::isfinite(out->z);
}

bool extractQuatXyzw(const std::vector<float> &data, size_t index, std::vector<float> *out)
{
    const size_t offset = index * 4;
    if (!out || offset + 3 >= data.size())
    {
        return false;
    }
    out->assign({data[offset + 0], data[offset + 1], data[offset + 2], data[offset + 3]});
    return std::isfinite((*out)[0]) && std::isfinite((*out)[1]) && std::isfinite((*out)[2]) && std::isfinite((*out)[3]);
}

std::vector<float> rotToRot6(const Mat3 &rot)
{
    return {rot.v[0], rot.v[1], rot.v[3], rot.v[4], rot.v[6], rot.v[7]};
}

std::vector<float> convertQuatVectorWxyzToXyzw(const std::vector<float> &wxyz)
{
    if (wxyz.size() % 4 != 0)
    {
        return {};
    }
    std::vector<float> out(wxyz.size(), 0.0f);
    for (size_t i = 0; i < wxyz.size(); i += 4)
    {
        out[i + 0] = wxyz[i + 1];
        out[i + 1] = wxyz[i + 2];
        out[i + 2] = wxyz[i + 3];
        out[i + 3] = wxyz[i + 0];
    }
    return out;
}

std::vector<float> convertQuatVectorToXyzw(
    const std::vector<float> &values,
    const std::string &source_order_raw)
{
    const std::string source_order = toLowerCopy(source_order_raw);
    if (source_order.empty() || source_order == "xyzw")
    {
        return values;
    }
    if (source_order == "wxyz")
    {
        return convertQuatVectorWxyzToXyzw(values);
    }
    return {};
}

std::vector<float> remapJointVectorToCanonical(
    const std::vector<float> &values,
    const std::vector<std::string> &source_joint_order,
    const std::vector<std::string> &canonical_joint_order)
{
    std::vector<float> out(canonical_joint_order.size(), 0.0f);
    if (values.empty() || source_joint_order.empty() || canonical_joint_order.empty())
    {
        return out;
    }

    std::unordered_map<std::string, size_t> source_index;
    source_index.reserve(source_joint_order.size());
    for (size_t i = 0; i < source_joint_order.size(); ++i)
    {
        source_index[source_joint_order[i]] = i;
    }

    for (size_t canonical_idx = 0; canonical_idx < canonical_joint_order.size(); ++canonical_idx)
    {
        const auto it = source_index.find(canonical_joint_order[canonical_idx]);
        if (it == source_index.end() || it->second >= values.size())
        {
            continue;
        }
        out[canonical_idx] = values[it->second];
    }
    return out;
}

const std::vector<float> *findNamedFeature(
    const std::unordered_map<std::string, std::vector<float>> &features,
    const std::string &name)
{
    const auto it = features.find(name);
    if (it == features.end())
    {
        return nullptr;
    }
    return &it->second;
}

const std::vector<float> *findExtraOutputByName(
    const std::unordered_map<std::string, std::vector<float>> &extra_outputs,
    const std::string &preferred_prefix,
    const std::string &output_name)
{
    const std::string preferred_key = preferred_prefix + output_name;
    const auto preferred_it = extra_outputs.find(preferred_key);
    if (preferred_it != extra_outputs.end())
    {
        return &preferred_it->second;
    }

    const std::string suffix = "/" + output_name;
    for (const auto &[key, value] : extra_outputs)
    {
        if (key.size() >= suffix.size() &&
            key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0)
        {
            return &value;
        }
    }
    return nullptr;
}

void setFeatureIfNonEmpty(
    ObservationFeatureContext *feature_context,
    const std::string &name,
    const std::vector<float> &values)
{
    if (!feature_context || values.empty())
    {
        return;
    }
    feature_context->named_features[name] = values;
}

size_t resolveAnchorIndex(
    const std::vector<std::string> &body_names,
    const std::string &anchor_body)
{
    if (body_names.empty())
    {
        return 0;
    }
    if (!anchor_body.empty())
    {
        for (size_t i = 0; i < body_names.size(); ++i)
        {
            if (body_names[i] == anchor_body)
            {
                return i;
            }
        }
    }
    return 0;
}

bool buildReferenceMotionLocalFeatures(
    const std::vector<float> &reference_body_pos_w,
    const std::vector<float> &reference_body_quat_w_xyzw,
    size_t anchor_index,
    const std::vector<float> &reference_anchor_init_pos_w,
    const std::vector<float> &reference_anchor_init_quat_w,
    const std::vector<float> &robot_base_init_quat_w,
    const std::vector<float> &robot_base_curr_quat_w,
    std::vector<float> *motion_anchor_pos_b,
    std::vector<float> *motion_anchor_ori_b,
    std::vector<float> *motion_body_pos_b,
    std::vector<float> *motion_body_ori_b)
{
    if (reference_body_pos_w.size() % 3 != 0 ||
        reference_body_quat_w_xyzw.size() % 4 != 0)
    {
        return false;
    }
    const size_t body_count = std::min(reference_body_pos_w.size() / 3, reference_body_quat_w_xyzw.size() / 4);
    if (body_count == 0 || anchor_index >= body_count)
    {
        return false;
    }
    if (reference_anchor_init_pos_w.size() < 3 ||
        reference_anchor_init_quat_w.size() < 4 ||
        robot_base_init_quat_w.size() < 4 ||
        robot_base_curr_quat_w.size() < 4)
    {
        return false;
    }

    Vec3 anchor_ref_init{};
    anchor_ref_init.x = reference_anchor_init_pos_w[0];
    anchor_ref_init.y = reference_anchor_init_pos_w[1];
    anchor_ref_init.z = reference_anchor_init_pos_w[2];

    const Mat3 ref_init_yaw = yawRotation(yawFromQuatXyzw(reference_anchor_init_quat_w));
    const Mat3 robot_init_yaw = yawRotation(yawFromQuatXyzw(robot_base_init_quat_w));
    const Mat3 robot_curr_yaw = yawRotation(yawFromQuatXyzw(robot_base_curr_quat_w));
    const Mat3 world_to_init = multiply(robot_init_yaw, transpose(ref_init_yaw));
    const Mat3 world_to_base_local = transpose(robot_curr_yaw);
    const Mat3 local_rot = multiply(world_to_base_local, world_to_init);

    Vec3 anchor_ref_curr{};
    std::vector<float> anchor_ref_quat;
    if (!extractVec3(reference_body_pos_w, anchor_index, &anchor_ref_curr) ||
        !extractQuatXyzw(reference_body_quat_w_xyzw, anchor_index, &anchor_ref_quat))
    {
        return false;
    }
    const Vec3 anchor_delta = subtract(anchor_ref_curr, anchor_ref_init);
    const Vec3 anchor_local = rotate(local_rot, anchor_delta);
    const Mat3 anchor_rot_local = multiply(local_rot, quatToRotXyzw(anchor_ref_quat));

    if (motion_anchor_pos_b)
    {
        motion_anchor_pos_b->assign({anchor_local.x, anchor_local.y, anchor_local.z});
    }
    if (motion_anchor_ori_b)
    {
        *motion_anchor_ori_b = rotToRot6(anchor_rot_local);
    }
    if (motion_body_pos_b)
    {
        motion_body_pos_b->clear();
        motion_body_pos_b->reserve(body_count * 3);
    }
    if (motion_body_ori_b)
    {
        motion_body_ori_b->clear();
        motion_body_ori_b->reserve(body_count * 6);
    }

    for (size_t i = 0; i < body_count; ++i)
    {
        Vec3 body_pos{};
        std::vector<float> body_quat;
        if (!extractVec3(reference_body_pos_w, i, &body_pos) ||
            !extractQuatXyzw(reference_body_quat_w_xyzw, i, &body_quat))
        {
            continue;
        }

        const Vec3 body_delta = subtract(body_pos, anchor_ref_init);
        const Vec3 body_local = rotate(local_rot, body_delta);
        const Mat3 body_rot_local = multiply(local_rot, quatToRotXyzw(body_quat));

        if (motion_body_pos_b)
        {
            motion_body_pos_b->push_back(body_local.x);
            motion_body_pos_b->push_back(body_local.y);
            motion_body_pos_b->push_back(body_local.z);
        }
        if (motion_body_ori_b)
        {
            const auto rot6 = rotToRot6(body_rot_local);
            motion_body_ori_b->insert(motion_body_ori_b->end(), rot6.begin(), rot6.end());
        }
    }
    return true;
}

bool buildRobotBodyLocalFeatures(
    const std::vector<float> &robot_body_pos_w,
    const std::vector<float> &robot_body_quat_w_xyzw,
    size_t anchor_index,
    std::vector<float> *robot_body_pos_b,
    std::vector<float> *robot_body_ori_b)
{
    if (robot_body_pos_w.size() % 3 != 0 ||
        robot_body_quat_w_xyzw.size() % 4 != 0)
    {
        return false;
    }
    const size_t body_count = std::min(robot_body_pos_w.size() / 3, robot_body_quat_w_xyzw.size() / 4);
    if (body_count == 0 || anchor_index >= body_count)
    {
        return false;
    }

    Vec3 anchor_pos{};
    std::vector<float> anchor_quat;
    if (!extractVec3(robot_body_pos_w, anchor_index, &anchor_pos) ||
        !extractQuatXyzw(robot_body_quat_w_xyzw, anchor_index, &anchor_quat))
    {
        return false;
    }
    const Mat3 anchor_inv = transpose(quatToRotXyzw(anchor_quat));

    if (robot_body_pos_b)
    {
        robot_body_pos_b->clear();
        robot_body_pos_b->reserve(body_count * 3);
    }
    if (robot_body_ori_b)
    {
        robot_body_ori_b->clear();
        robot_body_ori_b->reserve(body_count * 6);
    }

    for (size_t i = 0; i < body_count; ++i)
    {
        Vec3 body_pos{};
        std::vector<float> body_quat;
        if (!extractVec3(robot_body_pos_w, i, &body_pos) ||
            !extractQuatXyzw(robot_body_quat_w_xyzw, i, &body_quat))
        {
            continue;
        }

        const Vec3 body_local_pos = rotate(anchor_inv, subtract(body_pos, anchor_pos));
        const Mat3 body_local_rot = multiply(anchor_inv, quatToRotXyzw(body_quat));

        if (robot_body_pos_b)
        {
            robot_body_pos_b->push_back(body_local_pos.x);
            robot_body_pos_b->push_back(body_local_pos.y);
            robot_body_pos_b->push_back(body_local_pos.z);
        }
        if (robot_body_ori_b)
        {
            const auto rot6 = rotToRot6(body_local_rot);
            robot_body_ori_b->insert(robot_body_ori_b->end(), rot6.begin(), rot6.end());
        }
    }
    return true;
}
} // namespace

RL_controller::RL_controller()
    : RL_controller(nullptr)
{
}

RL_controller::RL_controller(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry)
    : onnx_env_(ORT_LOGGING_LEVEL_WARNING, "RL_controller")
    , mode_registry_(std::move(mode_registry))
{
    robot = RobotState::create();
    if (!robot)
    {
        throw std::runtime_error("Failed to create RobotState object!");
    }
}

RL_controller::~RL_controller()
{
    if (data_logger_)
    {
        data_logger_->close();
    }
}

std::unique_ptr<RL_controller> RL_controller::create(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry)
{
    return std::make_unique<RL_controller>(std::move(mode_registry));
}

std::vector<int> RL_controller::buildActionRobotIndices(
    const Sim2realCfg &cfg,
    const std::vector<std::string> &joint_names,
    const std::string &cfg_name) const
{
    if (cfg.action_dim <= 0)
    {
        throw std::runtime_error(cfg_name + ": action_dim must be > 0");
    }
    if (cfg.action_dim > static_cast<int>(joint_names.size()))
    {
        throw std::runtime_error(cfg_name + ": action_dim exceeds robot joint count");
    }

    std::vector<std::string> action_joint_names = cfg.action_joint_order;
    if (action_joint_names.empty())
    {
        if (cfg.robot_joint_order.size() < static_cast<size_t>(cfg.action_dim))
        {
            throw std::runtime_error(cfg_name + ": robot_joint_order is shorter than action_dim");
        }
        action_joint_names.assign(
            cfg.robot_joint_order.begin(),
            cfg.robot_joint_order.begin() + static_cast<std::ptrdiff_t>(cfg.action_dim));
    }
    if (action_joint_names.size() != static_cast<size_t>(cfg.action_dim))
    {
        throw std::runtime_error(cfg_name + ": action_joint_order length must equal action_dim");
    }

    std::unordered_map<std::string, int> joint_name_to_index;
    joint_name_to_index.reserve(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        joint_name_to_index[joint_names[i]] = static_cast<int>(i);
    }

    std::vector<int> map_policy_idx_to_robot_idx(static_cast<size_t>(cfg.action_dim), -1);
    std::unordered_map<std::string, bool> seen_names;
    seen_names.reserve(action_joint_names.size());
    for (size_t i = 0; i < action_joint_names.size(); ++i)
    {
        const auto &name = action_joint_names[i];
        if (seen_names.find(name) != seen_names.end())
        {
            throw std::runtime_error(cfg_name + ": duplicate joint name in action_joint_order: " + name);
        }
        seen_names[name] = true;
        const auto it = joint_name_to_index.find(name);
        if (it == joint_name_to_index.end())
        {
            throw std::runtime_error(cfg_name + ": unknown joint in action_joint_order: " + name);
        }
        map_policy_idx_to_robot_idx[i] = it->second;
    }

    return map_policy_idx_to_robot_idx;
}

std::vector<int> RL_controller::buildObsIndexMap(
    const Sim2realCfg &cfg,
    const std::vector<std::string> &joint_names,
    const std::string &cfg_name) const
{
    if (cfg.motor_N <= 0)
    {
        throw std::runtime_error(cfg_name + ": motor_N must be > 0");
    }
    if (cfg.motor_N > static_cast<int>(joint_names.size()))
    {
        throw std::runtime_error(cfg_name + ": motor_N exceeds robot joint count");
    }

    std::vector<int> map_policy_idx_to_robot_idx(static_cast<size_t>(cfg.motor_N), 0);
    std::vector<std::string> obs_joint_names = cfg.obs_joint_order;
    if (obs_joint_names.empty())
    {
        if (cfg.robot_joint_order.size() < static_cast<size_t>(cfg.motor_N))
        {
            throw std::runtime_error(cfg_name + ": robot_joint_order is shorter than motor_N");
        }
        obs_joint_names.assign(
            cfg.robot_joint_order.begin(),
            cfg.robot_joint_order.begin() + static_cast<std::ptrdiff_t>(cfg.motor_N));
    }
    if (obs_joint_names.size() != static_cast<size_t>(cfg.motor_N))
    {
        throw std::runtime_error(cfg_name + ": obs_joint_order length must equal motor_N");
    }

    std::unordered_map<std::string, int> joint_name_to_index;
    joint_name_to_index.reserve(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        joint_name_to_index[joint_names[i]] = static_cast<int>(i);
    }

    for (size_t policy_idx = 0; policy_idx < obs_joint_names.size(); ++policy_idx)
    {
        const auto &name = obs_joint_names[policy_idx];
        const auto it = joint_name_to_index.find(name);
        if (it == joint_name_to_index.end())
        {
            throw std::runtime_error(cfg_name + ": unknown joint in obs_joint_order: " + name);
        }
        map_policy_idx_to_robot_idx[policy_idx] = it->second;
    }
    return map_policy_idx_to_robot_idx;
}

std::vector<int> RL_controller::buildReferenceIndexMap(
    const Sim2realCfg &cfg,
    const std::vector<std::string> &joint_names,
    const std::string &cfg_name) const
{
    std::vector<std::string> reference_joint_names = cfg.reference_joint_order;
    if (reference_joint_names.empty())
    {
        reference_joint_names = cfg.action_joint_order;
    }
    if (reference_joint_names.empty())
    {
        return {};
    }

    std::unordered_map<std::string, int> joint_name_to_index;
    joint_name_to_index.reserve(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        joint_name_to_index[joint_names[i]] = static_cast<int>(i);
    }

    std::vector<int> map_reference_idx_to_robot_idx(reference_joint_names.size(), -1);
    for (size_t reference_idx = 0; reference_idx < reference_joint_names.size(); ++reference_idx)
    {
        const auto it = joint_name_to_index.find(reference_joint_names[reference_idx]);
        if (it == joint_name_to_index.end())
        {
            throw std::runtime_error(cfg_name + ": unknown joint in reference_joint_order: " + reference_joint_names[reference_idx]);
        }
        map_reference_idx_to_robot_idx[reference_idx] = it->second;
    }
    return map_reference_idx_to_robot_idx;
}

const std::vector<int> &RL_controller::currentActionIndexMap() const
{
    return activeModeProfile().action_robot_indices;
}

const std::vector<int> &RL_controller::currentObsIndexMap() const
{
    return activeModeProfile().obs_index_map;
}

const std::vector<int> &RL_controller::currentReferenceIndexMap() const
{
    return activeModeProfile().reference_index_map;
}

const Sim2realCfg &RL_controller::activePolicyCfg() const
{
    return activeModeProfile().cfg;
}

const ObservationBuilder &RL_controller::activeObservationBuilder() const
{
    if (!activeModeProfile().observation_builder)
    {
        throw std::runtime_error("Active observation builder is null");
    }
    return *activeModeProfile().observation_builder;
}

RL_controller::PolicyRuntimeGroup &RL_controller::activePolicyGroup()
{
    return activeModeProfile().policy_group;
}

const ReferenceMotionProvider &RL_controller::activeReferenceMotionProvider() const
{
    return activeModeProfile().reference_motion;
}

std::vector<float> RL_controller::activeZeroPose() const
{
    const auto &profile = activeModeProfile();
    if (!profile.zero_pose.empty())
    {
        return profile.zero_pose;
    }
    return robot->default_angle;
}

void RL_controller::refreshPolicyMode(int requested_mode, bool sanitize_invalid_mode)
{
    if (mode_profiles_.empty())
    {
        throw std::runtime_error("No mode profile is loaded");
    }

    const size_t profile_index = profileIndexForMode(requested_mode, sanitize_invalid_mode);
    active_profile_index_ = profile_index;
    active_mode_id_ = mode_profiles_[profile_index].mode_id;
    syncActiveProfileToRobotState();
}

void RL_controller::handlePolicySwitch()
{
    if (last_active_mode_id_ == active_mode_id_)
    {
        return;
    }

    const auto &cfg = activePolicyCfg();
    if (cfg.obs_dim <= 0 || cfg.obs_stack_N <= 0)
    {
        throw std::runtime_error("Invalid active policy dimensions");
    }

    obs.assign(static_cast<size_t>(cfg.obs_dim), 0.0f);
    obs_deque.clear();
    for (int i = 0; i < cfg.obs_stack_N; ++i)
    {
        obs_deque.push_back(std::vector<float>(static_cast<size_t>(cfg.obs_dim), 0.0f));
    }
    stacked_obs_buffer_.assign(static_cast<size_t>(cfg.obs_dim * cfg.obs_stack_N), 0.0f);
    action.assign(static_cast<size_t>(cfg.action_dim), 0.0f);
    joint_target_q.assign(joint_order_.size(), 0.0f);
    joint_target_torque.assign(joint_order_.size(), 0.0f);
    latest_policy_extra_outputs_.clear();
    deploy_step_counter_ = 0;
    phase_reset_pending_ = true;
    auto &profile = activeModeProfile();
    profile.reference_alignment_initialized = false;
    profile.reference_anchor_init_pos_w.clear();
    profile.reference_anchor_init_quat_w.clear();
    profile.robot_base_init_quat_w.clear();

    if (cfg.reset_policy_on_mode_switch)
    {
        auto &group = activePolicyGroup();
        for (auto &node : group.runners)
        {
            if (node.runner)
            {
                node.runner->reset();
            }
        }
        if (profile.amp_discriminator)
        {
            profile.amp_discriminator->reset();
        }
    }

    deploy_state_machine_.configure(cfg);
    deploy_state_machine_.setZeroPose(activeZeroPose());

    last_active_mode_id_ = active_mode_id_;
    std::cout << "[RL_controller] switch policy to "
              << activeModeProfile().tag
              << ", mode_id=" << active_mode_id_ << std::endl;
}

const Sim2realCfg &RL_controller::runtimeCfg() const
{
    return activeModeProfile().cfg;
}

int RL_controller::activeModeId() const
{
    return active_mode_id_;
}

const std::string &RL_controller::activeConfigSection() const
{
    return activeModeProfile().config_section;
}

RL_controller::ModeProfile &RL_controller::activeModeProfile()
{
    if (mode_profiles_.empty() || active_profile_index_ >= mode_profiles_.size())
    {
        throw std::runtime_error("Active mode profile index is invalid");
    }
    return mode_profiles_[active_profile_index_];
}

const RL_controller::ModeProfile &RL_controller::activeModeProfile() const
{
    if (mode_profiles_.empty() || active_profile_index_ >= mode_profiles_.size())
    {
        throw std::runtime_error("Active mode profile index is invalid");
    }
    return mode_profiles_[active_profile_index_];
}

void RL_controller::syncActiveProfileToRobotState()
{
    if (!robot)
    {
        return;
    }
    if (mode_profiles_.empty() || active_profile_index_ >= mode_profiles_.size())
    {
        return;
    }
    const auto &profile = mode_profiles_[active_profile_index_];
    robot->joint_names = joint_order_;
    robot->default_angle = profile.default_angle;
}

size_t RL_controller::profileIndexForMode(int mode_id, bool sanitize_invalid_mode) const
{
    const auto it = mode_to_profile_index_.find(mode_id);
    if (it != mode_to_profile_index_.end())
    {
        return it->second;
    }

    if (!sanitize_invalid_mode)
    {
        throw std::runtime_error("Unknown mode id: " + std::to_string(mode_id));
    }

    const auto fallback_it = mode_to_profile_index_.find(default_mode_id_);
    if (fallback_it != mode_to_profile_index_.end())
    {
        return fallback_it->second;
    }
    return 0;
}

std::vector<float> RL_controller::buildDefaultAnglesFromCfg(
    const Sim2realCfg::RobotCfg &robot_cfg,
    const std::vector<std::string> &joint_names) const
{
    std::vector<float> out(joint_names.size(), 0.0f);
    std::unordered_map<std::string, float> default_angle_map;
    default_angle_map.reserve(robot_cfg.default_joint_angles.size());
    for (const auto &entry : robot_cfg.default_joint_angles)
    {
        default_angle_map[entry.first] = entry.second;
    }

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        const auto it = default_angle_map.find(joint_names[i]);
        if (it != default_angle_map.end())
        {
            out[i] = it->second;
        }
    }
    return out;
}

std::vector<float> RL_controller::buildZeroPoseFromCfg(
    const Sim2realCfg::RobotCfg &robot_cfg,
    const std::vector<std::string> &joint_names) const
{
    if (robot_cfg.zero_joint_angles.empty())
    {
        return {};
    }

    std::vector<float> out(joint_names.size(), 0.0f);
    std::unordered_map<std::string, float> zero_pose_map;
    zero_pose_map.reserve(robot_cfg.zero_joint_angles.size());
    for (const auto &entry : robot_cfg.zero_joint_angles)
    {
        zero_pose_map[entry.first] = entry.second;
    }

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        const auto it = zero_pose_map.find(joint_names[i]);
        if (it == zero_pose_map.end())
        {
            throw std::runtime_error(
                "robot.zero_joint_angles missing joint for runtime order: " + joint_names[i]);
        }
        out[i] = it->second;
    }
    return out;
}

std::vector<RL_controller::ModeProfileSpec> RL_controller::loadModeProfileSpecsFromYaml() const
{
    std::shared_ptr<const rl_master::ModeProfileRegistry> registry = mode_registry_;
    if (!registry)
    {
        registry = rl_master::ModeProfileRegistry::loadFromYaml(RL_CFG_PATH, "engineai_walk");
    }

    const auto &parsed_specs = registry->specs();
    if (parsed_specs.empty())
    {
        throw std::runtime_error("mode registry is empty");
    }

    std::vector<ModeProfileSpec> parsed;
    parsed.reserve(parsed_specs.size());
    for (const auto &spec : parsed_specs)
    {
        ModeProfileSpec node_spec;
        node_spec.mode_id = spec.mode_id;
        node_spec.config_section = spec.config_section;
        node_spec.tag = spec.tag.empty() ? spec.config_section : spec.tag;
        parsed.push_back(std::move(node_spec));
    }

    return parsed;
}

void RL_controller::initModeProfiles()
{
    mode_profiles_.clear();
    mode_to_profile_index_.clear();

    if (!mode_registry_)
    {
        mode_registry_ = rl_master::ModeProfileRegistry::loadFromYaml(RL_CFG_PATH, "engineai_walk");
    }

    const std::vector<ModeProfileSpec> specs = loadModeProfileSpecsFromYaml();
    if (specs.empty())
    {
        throw std::runtime_error("No deploy mode profile spec is configured");
    }

    joint_order_ = mode_registry_->jointOrder();
    if (joint_order_.empty())
    {
        throw std::runtime_error("Global robot joint order resolved from mode registry is empty");
    }

    for (const auto &spec : specs)
    {
        if (mode_to_profile_index_.find(spec.mode_id) != mode_to_profile_index_.end())
        {
            throw std::runtime_error("Duplicate mode_id in deploy_mode_profiles: " + std::to_string(spec.mode_id));
        }
        if (spec.mode_id < rl_master::kModeCodeMin || spec.mode_id > rl_master::kModeCodeMax)
        {
            throw std::runtime_error(
                "mode_id out of supported range [" +
                std::to_string(rl_master::kModeCodeMin) +
                ", " +
                std::to_string(rl_master::kModeCodeMax) +
                "]: " +
                std::to_string(spec.mode_id));
        }

        const auto &cfg = mode_registry_->cfgForSection(spec.config_section);

        ModeProfile profile;
        profile.mode_id = spec.mode_id;
        profile.config_section = spec.config_section;
        profile.tag = spec.tag;
        profile.cfg = cfg;
        profile.joint_names = joint_order_;
        profile.default_angle = buildDefaultAnglesFromCfg(profile.cfg.robotCfg, joint_order_);
        profile.zero_pose = buildZeroPoseFromCfg(profile.cfg.robotCfg, joint_order_);
        profile.action_robot_indices = buildActionRobotIndices(profile.cfg, joint_order_, profile.config_section);
        profile.obs_index_map = buildObsIndexMap(profile.cfg, joint_order_, profile.config_section);
        profile.reference_index_map = buildReferenceIndexMap(profile.cfg, joint_order_, profile.config_section);
        profile.observation_manifest = ObservationManifest::loadFromYAML(profile.cfg.observation_manifest_path);
        profile.observation_builder = std::make_unique<ObservationBuilder>(profile.observation_manifest);
        if (profile.observation_builder->expectedDim() != static_cast<size_t>(profile.cfg.obs_dim))
        {
            throw std::runtime_error(
                profile.tag + " observation manifest dim (" +
                std::to_string(profile.observation_builder->expectedDim()) +
                ") does not match cfg obs_dim (" + std::to_string(profile.cfg.obs_dim) + ")");
        }
        initPolicyGroup(profile.cfg, profile.tag, &profile.policy_group);
        initAmpDiscriminatorRunner(profile.cfg, profile.tag, &profile.amp_discriminator);
        initReferenceMotionProvider(profile.cfg, &profile.reference_motion, profile.tag);

        mode_to_profile_index_[profile.mode_id] = mode_profiles_.size();
        mode_profiles_.push_back(std::move(profile));
    }

    default_mode_id_ = mode_profiles_.front().mode_id;
    active_mode_id_ = default_mode_id_;
    active_profile_index_ = profileIndexForMode(default_mode_id_, true);

    std::cout << "[RL_controller] mode profiles loaded: " << mode_profiles_.size() << std::endl;
    for (const auto &profile : mode_profiles_)
    {
        std::cout << "  - mode_id=" << profile.mode_id
                  << ", tag=" << profile.tag
                  << ", section=" << profile.config_section
                  << ", policy=" << profile.cfg.policy_name << std::endl;
    }
}

void RL_controller::updateStateFromIO(const rl_master::RobotStateData &state)
{
    const size_t joint_count = joint_order_.size();
    if (robot->joint_q.size() != joint_count)
    {
        robot->initialize_buffers(joint_count, robot->default_angle);
        robot->joint_names = joint_order_;
    }
    for (size_t i = 0; i < joint_count; ++i)
    {
        robot->joint_q[i] = (i < state.joint_q.size()) ? state.joint_q[i] : robot->default_angle[i];
        robot->joint_dq[i] = (i < state.joint_dq.size()) ? state.joint_dq[i] : 0.0f;
        robot->joint_tau[i] = (i < state.joint_tau.size()) ? state.joint_tau[i] : 0.0f;
    }

    for (size_t i = 0; i < 3; ++i)
    {
        robot->base_ang_vel[i] = state.base_ang_vel[i];
        robot->base_rpy[i] = state.base_rpy[i];
    }

    for (size_t i = 0; i < 4; ++i)
    {
        robot->base_quat[i] = state.base_quat[i];
    }
}

void RL_controller::updateCommandFromIO(const rl_master::TeleopCommand &command)
{
    cmd.vx = command.vx;
    cmd.vy = command.vy;
    cmd.dyaw = command.dyaw;
}

void RL_controller::initPolicyGroup(const Sim2realCfg &cfg, const std::string &tag, PolicyRuntimeGroup *group)
{
    if (!group)
    {
        throw std::runtime_error("initPolicyGroup: group is null");
    }

    group->runners.clear();

    PolicyRunnerNode primary;
    primary.name = tag + "/main";
    primary.weight = 1.0f;
    primary.runner = std::make_unique<OnnxPolicyRunner>(
        onnx_env_,
        cfg.policy_path,
        cfg,
        primary.name);
    primary.runner->init();
    group->runners.push_back(std::move(primary));

    for (const auto &sub : cfg.sub_models)
    {
        if (!sub.enabled)
        {
            continue;
        }

        Sim2realCfg sub_cfg = cfg;
        sub_cfg.policy_path = sub.policy_path;
        if (sub.action_dim > 0)
        {
            sub_cfg.action_dim = sub.action_dim;
        }
        sub_cfg.obs_input_name = sub.obs_input_name;
        sub_cfg.action_output_name = sub.action_output_name;
        sub_cfg.time_step_input_name = sub.time_step_input_name;
        sub_cfg.time_step_start = sub.time_step_start;
        sub_cfg.enable_time_step_input = sub.enable_time_step_input;
        sub_cfg.strict_model_io = sub.strict_model_io;
        sub_cfg.extra_output_names = sub.extra_output_names;
        sub_cfg.onnx_inputs = sub.onnx_inputs;
        sub_cfg.enable_metadata_check = sub.enable_metadata_check;
        sub_cfg.metadata_check_strict = sub.metadata_check_strict;
        sub_cfg.required_metadata_keys = sub.required_metadata_keys;
        sub_cfg.expected_metadata = sub.expected_metadata;

        PolicyRunnerNode node;
        node.name = tag + "/" + sub.name;
        node.weight = std::max(0.0f, sub.weight);
        node.runner = std::make_unique<OnnxPolicyRunner>(
            onnx_env_,
            sub_cfg.policy_path,
            sub_cfg,
            node.name);
        node.runner->init();
        group->runners.push_back(std::move(node));
    }
}

void RL_controller::initAmpDiscriminatorRunner(
    const Sim2realCfg &cfg,
    const std::string &tag,
    std::unique_ptr<OnnxPolicyRunner> *runner)
{
    if (!runner)
    {
        return;
    }
    runner->reset();

    if (!cfg.amp_discriminator.enabled)
    {
        return;
    }
    if (cfg.amp_discriminator.policy_path.empty())
    {
        std::cerr << "[RL_controller][" << tag << "] amp_discriminator enabled but policy path is empty." << std::endl;
        return;
    }

    Sim2realCfg disc_cfg = cfg;
    disc_cfg.policy_path = cfg.amp_discriminator.policy_path;
    disc_cfg.obs_input_name = cfg.amp_discriminator.obs_input_name;
    disc_cfg.action_output_name = cfg.amp_discriminator.score_output_name;
    disc_cfg.time_step_input_name = cfg.amp_discriminator.time_step_input_name;
    disc_cfg.time_step_start = cfg.amp_discriminator.time_step_start;
    disc_cfg.enable_time_step_input = cfg.amp_discriminator.enable_time_step_input;
    disc_cfg.strict_model_io = cfg.amp_discriminator.strict_model_io;
    disc_cfg.extra_output_names = cfg.amp_discriminator.extra_output_names;
    disc_cfg.onnx_inputs = cfg.amp_discriminator.onnx_inputs;
    disc_cfg.enable_metadata_check = cfg.amp_discriminator.enable_metadata_check;
    disc_cfg.metadata_check_strict = cfg.amp_discriminator.metadata_check_strict;
    disc_cfg.required_metadata_keys = cfg.amp_discriminator.required_metadata_keys;
    disc_cfg.expected_metadata = cfg.amp_discriminator.expected_metadata;
    disc_cfg.action_dim = 0;

    auto local_runner = std::make_unique<OnnxPolicyRunner>(
        onnx_env_,
        disc_cfg.policy_path,
        disc_cfg,
        tag + "/amp_discriminator");
    local_runner->init();

    *runner = std::move(local_runner);
    std::cout << "[RL_controller][" << tag << "] amp_discriminator loaded: "
              << disc_cfg.policy_path << std::endl;
}

void RL_controller::runAmpDiscriminator(
    const Sim2realCfg &cfg,
    const std::string &tag,
    OnnxPolicyRunner *runner,
    const std::vector<float> &current_observation,
    const std::vector<float> &stacked_observation)
{
    if (!runner || !cfg.amp_discriminator.enabled)
    {
        return;
    }

    const std::string source = toLowerCopy(cfg.amp_discriminator.input_source);
    const std::vector<float> *input = &stacked_observation;
    if (source == "observation" || source == "policy_observation")
    {
        input = &current_observation;
    }
    if (!input || input->empty())
    {
        return;
    }

    const std::unordered_map<std::string, std::vector<float>> empty_features;
    PolicyInferenceResult disc_result = runner->forward(*input, current_observation, empty_features);
    const std::string prefix = tag + "/amp_discriminator/";
    latest_policy_extra_outputs_[prefix + "score"] = disc_result.action;
    for (auto &kv : disc_result.extra_outputs)
    {
        latest_policy_extra_outputs_[prefix + kv.first] = std::move(kv.second);
    }

    if (cfg.amp_discriminator.warn_below > -1.0e8f &&
        !disc_result.action.empty() &&
        (deploy_step_counter_ % 100 == 0))
    {
        const float score_mean = meanOf(disc_result.action);
        if (score_mean < cfg.amp_discriminator.warn_below)
        {
            std::cerr << "[RL_controller][" << tag << "] amp_discriminator score low: "
                      << score_mean << " < " << cfg.amp_discriminator.warn_below << std::endl;
        }
    }
}

RL_controller::PolicyRunOutput RL_controller::runPolicyGroup(
    PolicyRuntimeGroup *group,
    const std::vector<float> &stacked_obs,
    const std::vector<float> &current_observation,
    const ObservationFeatureContext &feature_context)
{
    if (!group || group->runners.empty())
    {
        throw std::runtime_error("runPolicyGroup: empty policy group");
    }

    const size_t expected_dim = static_cast<size_t>(std::max(0, activePolicyCfg().action_dim));
    PolicyRunOutput output;
    output.action.assign(expected_dim, 0.0f);

    float total_weight = 0.0f;
    bool primary_done = false;
    for (auto &node : group->runners)
    {
        if (!node.runner)
        {
            continue;
        }

        PolicyInferenceResult result =
            node.runner->forward(stacked_obs, current_observation, feature_context.named_features);
        const float weight = primary_done ? node.weight : 1.0f;
        const size_t dim = std::min(output.action.size(), result.action.size());
        for (size_t i = 0; i < dim; ++i)
        {
            output.action[i] += weight * result.action[i];
        }
        total_weight += weight;

        for (auto &kv : result.extra_outputs)
        {
            output.extra_outputs[node.name + "/" + kv.first] = std::move(kv.second);
        }

        primary_done = true;
    }

    if (total_weight > 1e-6f)
    {
        for (auto &v : output.action)
        {
            v /= total_weight;
        }
    }
    return output;
}

void RL_controller::initReferenceMotionProvider(const Sim2realCfg &cfg, ReferenceMotionProvider *provider, const std::string &tag)
{
    if (!provider)
    {
        return;
    }

    provider->clear();
    if (!cfg.enable_reference_motion)
    {
        return;
    }

    std::string source = toLowerCopy(cfg.reference_motion_source);
    if (source == "policy_outputs")
    {
        return;
    }
    if (cfg.reference_motion_path.empty())
    {
        if (source == "file")
        {
            std::cerr << "[RL_controller][" << tag << "] reference_motion_source=file but reference_motion_path is empty."
                      << std::endl;
        }
        return;
    }

    if (!provider->load(
            cfg.reference_motion_path,
            cfg.reference_motion_dim,
            cfg.source_contract.reference_file.body_quat_order))
    {
        std::cerr << "[RL_controller][" << tag << "] failed to load reference motion file: "
                  << cfg.reference_motion_path << std::endl;
        return;
    }

    const auto &metadata = provider->metadata();
    std::cout << "[RL_controller][" << tag << "] reference motion loaded. frames="
              << provider->frameCount() << ", dim=" << provider->dim()
              << ", source_format=" << metadata.source_format
              << ", body_count=" << metadata.body_names.size() << std::endl;
}

ObservationFeatureContext RL_controller::buildObservationFeatureContext(const Sim2realCfg &cfg, double phase_t)
{
    ObservationFeatureContext feature_context;
    auto &profile = activeModeProfile();
    std::string source = toLowerCopy(cfg.reference_motion_source);

    auto external = external_observation_provider_.collect(cfg.external_observations);
    for (auto &kv : external)
    {
        feature_context.named_features.emplace(std::move(kv.first), std::move(kv.second));
    }

    const bool enable_reference = cfg.enable_reference_motion;
    const bool use_file_source = enable_reference && source != "policy_outputs";
    const bool use_policy_source = enable_reference && source != "file";
    const int reference_motion_dim = cfg.reference_motion_dim > 0 ? cfg.reference_motion_dim : activeReferenceMotionProvider().dim();

    if (use_file_source && activeReferenceMotionProvider().available())
    {
        const auto &provider = activeReferenceMotionProvider();
        const ReferenceMotionFrame sampled_frame =
            (cfg.reference_motion_sampling == "step")
                ? provider.sampleFrameByStep(deploy_step_counter_, reference_motion_dim)
                : provider.sampleFrameByPhase(phase_t, cfg.cycle_time, reference_motion_dim);

        if (!sampled_frame.reference_motion.empty())
        {
            setFeatureIfNonEmpty(
                &feature_context,
                "reference_motion",
                reference_motion_dim > 0 ? fitDim(sampled_frame.reference_motion, static_cast<size_t>(reference_motion_dim))
                                         : sampled_frame.reference_motion);
        }
        setFeatureIfNonEmpty(
            &feature_context,
            "reference_joint_pos",
            remapJointVectorToCanonical(
                sampled_frame.joint_pos,
                cfg.reference_joint_order,
                joint_order_));
        setFeatureIfNonEmpty(
            &feature_context,
            "reference_joint_vel",
            remapJointVectorToCanonical(
                sampled_frame.joint_vel,
                cfg.reference_joint_order,
                joint_order_));
        setFeatureIfNonEmpty(&feature_context, "reference_body_pos_w", sampled_frame.body_pos_w);
        setFeatureIfNonEmpty(&feature_context, "reference_body_quat_w", sampled_frame.body_quat_w);
    }

    if (use_policy_source)
    {
        const std::string preferred_prefix = profile.tag + "/main/";
        if (const auto *joint_pos = findExtraOutputByName(latest_policy_extra_outputs_, preferred_prefix, "joint_pos"))
        {
            setFeatureIfNonEmpty(
                &feature_context,
                "reference_joint_pos",
                remapJointVectorToCanonical(
                    *joint_pos,
                    cfg.reference_joint_order,
                    joint_order_));
        }
        if (const auto *joint_vel = findExtraOutputByName(latest_policy_extra_outputs_, preferred_prefix, "joint_vel"))
        {
            setFeatureIfNonEmpty(
                &feature_context,
                "reference_joint_vel",
                remapJointVectorToCanonical(
                    *joint_vel,
                    cfg.reference_joint_order,
                    joint_order_));
        }
        if (const auto *body_pos_w = findExtraOutputByName(latest_policy_extra_outputs_, preferred_prefix, "body_pos_w"))
        {
            setFeatureIfNonEmpty(&feature_context, "reference_body_pos_w", *body_pos_w);
        }
        if (const auto *body_quat_w = findExtraOutputByName(latest_policy_extra_outputs_, preferred_prefix, "body_quat_w"))
        {
            const std::vector<float> quat_xyzw = convertQuatVectorToXyzw(
                *body_quat_w,
                cfg.source_contract.policy_extra_outputs.body_quat_order);
            setFeatureIfNonEmpty(&feature_context, "reference_body_quat_w", quat_xyzw);
        }
    }

    if (findNamedFeature(feature_context.named_features, "reference_motion") == nullptr)
    {
        const auto *joint_pos = findNamedFeature(feature_context.named_features, "reference_joint_pos");
        const auto *joint_vel = findNamedFeature(feature_context.named_features, "reference_joint_vel");
        if (joint_pos && joint_vel)
        {
            std::vector<float> packed;
            packed.reserve(joint_pos->size() + joint_vel->size());
            packed.insert(packed.end(), joint_pos->begin(), joint_pos->end());
            packed.insert(packed.end(), joint_vel->begin(), joint_vel->end());
            if (reference_motion_dim > 0)
            {
                packed = fitDim(packed, static_cast<size_t>(reference_motion_dim));
            }
            setFeatureIfNonEmpty(&feature_context, "reference_motion", packed);
        }
    }
    if (reference_motion_dim > 0 &&
        findNamedFeature(feature_context.named_features, "reference_motion") == nullptr)
    {
        feature_context.named_features["reference_motion"] =
            std::vector<float>(static_cast<size_t>(reference_motion_dim), 0.0f);
    }

    std::vector<std::string> body_names = cfg.reference_body_names;
    std::string anchor_body = cfg.reference_anchor_body;
    if (activeReferenceMotionProvider().available())
    {
        const auto &provider_metadata = activeReferenceMotionProvider().metadata();
        if (!provider_metadata.body_names.empty())
        {
            body_names = provider_metadata.body_names;
        }
        if (!provider_metadata.anchor_body.empty())
        {
            anchor_body = provider_metadata.anchor_body;
        }
    }

    const auto *reference_body_pos_w = findNamedFeature(feature_context.named_features, "reference_body_pos_w");
    const auto *reference_body_quat_w = findNamedFeature(feature_context.named_features, "reference_body_quat_w");
    if (reference_body_pos_w && reference_body_quat_w &&
        reference_body_pos_w->size() % 3 == 0 && reference_body_quat_w->size() % 4 == 0)
    {
        const size_t body_count = std::min(reference_body_pos_w->size() / 3, reference_body_quat_w->size() / 4);
        if (body_names.empty())
        {
            body_names.reserve(body_count);
            for (size_t i = 0; i < body_count; ++i)
            {
                body_names.push_back("body_" + std::to_string(i));
            }
        }

        const size_t anchor_index = resolveAnchorIndex(body_names, anchor_body);
        Vec3 anchor_pos{};
        std::vector<float> anchor_quat;
        if (anchor_index < body_count &&
            extractVec3(*reference_body_pos_w, anchor_index, &anchor_pos) &&
            extractQuatXyzw(*reference_body_quat_w, anchor_index, &anchor_quat))
        {
            if (!profile.reference_alignment_initialized)
            {
                profile.reference_anchor_init_pos_w = {anchor_pos.x, anchor_pos.y, anchor_pos.z};
                profile.reference_anchor_init_quat_w = anchor_quat;
                profile.robot_base_init_quat_w = robot->base_quat;
                profile.reference_alignment_initialized = true;
            }

            std::vector<float> motion_anchor_pos_b;
            std::vector<float> motion_anchor_ori_b;
            std::vector<float> motion_body_pos_b;
            std::vector<float> motion_body_ori_b;
            if (buildReferenceMotionLocalFeatures(
                    *reference_body_pos_w,
                    *reference_body_quat_w,
                    anchor_index,
                    profile.reference_anchor_init_pos_w,
                    profile.reference_anchor_init_quat_w,
                    profile.robot_base_init_quat_w,
                    robot->base_quat,
                    &motion_anchor_pos_b,
                    &motion_anchor_ori_b,
                    &motion_body_pos_b,
                    &motion_body_ori_b))
            {
                setFeatureIfNonEmpty(&feature_context, "motion_anchor_pos_b", motion_anchor_pos_b);
                setFeatureIfNonEmpty(&feature_context, "motion_ref_pos_b", motion_anchor_pos_b);
                setFeatureIfNonEmpty(&feature_context, "motion_anchor_ori_b", motion_anchor_ori_b);
                setFeatureIfNonEmpty(&feature_context, "motion_ref_ori_b", motion_anchor_ori_b);
                setFeatureIfNonEmpty(&feature_context, "motion_body_pos_b", motion_body_pos_b);
                setFeatureIfNonEmpty(&feature_context, "motion_body_ori_b", motion_body_ori_b);
            }
        }
    }

    const auto *robot_body_pos_w = findNamedFeature(feature_context.named_features, "robot_body_pos_w");
    const auto *robot_body_quat_w = findNamedFeature(feature_context.named_features, "robot_body_quat_w");
    if (robot_body_pos_w && robot_body_quat_w &&
        robot_body_pos_w->size() % 3 == 0 && robot_body_quat_w->size() % 4 == 0)
    {
        const size_t body_count = std::min(robot_body_pos_w->size() / 3, robot_body_quat_w->size() / 4);
        if (body_count > 0)
        {
            if (body_names.empty())
            {
                body_names.reserve(body_count);
                for (size_t i = 0; i < body_count; ++i)
                {
                    body_names.push_back("body_" + std::to_string(i));
                }
            }
            const size_t anchor_index = resolveAnchorIndex(body_names, anchor_body);
            std::vector<float> robot_body_pos_b;
            std::vector<float> robot_body_ori_b;
            if (buildRobotBodyLocalFeatures(
                    *robot_body_pos_w,
                    *robot_body_quat_w,
                    anchor_index,
                    &robot_body_pos_b,
                    &robot_body_ori_b))
            {
                setFeatureIfNonEmpty(&feature_context, "robot_body_pos", robot_body_pos_b);
                setFeatureIfNonEmpty(&feature_context, "robot_body_ori", robot_body_ori_b);
            }
        }
    }

    return feature_context;
}

void RL_controller::initDataLogger()
{
    const auto &runtime_cfg = activeModeProfile().cfg;
    if (!runtime_cfg.save_data_flag)
    {
        return;
    }

    data_logger_ = std::make_unique<rl_master::logging::StructuredLogger>();
    rl_master::logging::LoggerMetadata metadata;
    metadata.string_fields["module"] = "RL_controller";
    metadata.numeric_fields["profile_count"] = static_cast<double>(mode_profiles_.size());

    auto collectSubModelNames = [](const Sim2realCfg &cfg) {
        std::vector<std::string> names;
        for (const auto &sub : cfg.sub_models)
        {
            names.push_back(sub.name);
        }
        return names;
    };
    auto collectSubModelPaths = [](const Sim2realCfg &cfg) {
        std::vector<std::string> paths;
        for (const auto &sub : cfg.sub_models)
        {
            paths.push_back(sub.policy_path);
        }
        return paths;
    };

    for (size_t i = 0; i < mode_profiles_.size(); ++i)
    {
        const auto &profile = mode_profiles_[i];
        const std::string prefix = "profile_" + std::to_string(i) + "_";

        metadata.numeric_fields[prefix + "mode_id"] = static_cast<double>(profile.mode_id);
        metadata.numeric_fields[prefix + "obs_dim"] = static_cast<double>(profile.cfg.obs_dim);
        metadata.numeric_fields[prefix + "action_dim"] = static_cast<double>(profile.cfg.action_dim);
        metadata.numeric_fields[prefix + "obs_stack"] = static_cast<double>(profile.cfg.obs_stack_N);
        metadata.numeric_fields[prefix + "control_hz"] = static_cast<double>(profile.cfg.RL_control_f);
        metadata.numeric_fields[prefix + "amp_discriminator_enabled"] = profile.cfg.amp_discriminator.enabled ? 1.0 : 0.0;
        metadata.numeric_fields[prefix + "amp_discriminator_warn_below"] = static_cast<double>(profile.cfg.amp_discriminator.warn_below);

        metadata.string_fields[prefix + "tag"] = profile.tag;
        metadata.string_fields[prefix + "config_section"] = profile.config_section;
        metadata.string_fields[prefix + "policy_name"] = profile.cfg.policy_name;
        metadata.string_fields[prefix + "policy_family"] = profile.cfg.policy_family;
        metadata.string_fields[prefix + "policy_path"] = profile.cfg.policy_path;
        metadata.string_fields[prefix + "observation_manifest"] = profile.cfg.observation_manifest_path;
        metadata.string_fields[prefix + "amp_discriminator_path"] = profile.cfg.amp_discriminator.policy_path;
        metadata.string_fields[prefix + "amp_discriminator_input_source"] = profile.cfg.amp_discriminator.input_source;
        metadata.string_fields[prefix + "amp_discriminator_score_output"] = profile.cfg.amp_discriminator.score_output_name;
        metadata.string_fields[prefix + "reference_motion_source"] = profile.cfg.reference_motion_source;
        metadata.string_fields[prefix + "reference_motion_path"] = profile.cfg.reference_motion_path;
        metadata.string_fields[prefix + "reference_anchor_body_cfg"] = profile.cfg.reference_anchor_body;
        metadata.string_fields[prefix + "reference_source_format"] = profile.reference_motion.metadata().source_format;
        metadata.string_fields[prefix + "reference_anchor_body_loaded"] = profile.reference_motion.metadata().anchor_body;

        metadata.vector_fields[prefix + "kps"] = toDoubleVector(profile.cfg.kps);
        metadata.vector_fields[prefix + "kds"] = toDoubleVector(profile.cfg.kds);
        metadata.vector_fields[prefix + "tau_limit"] = toDoubleVector(profile.cfg.tau_limit);
        metadata.numeric_fields[prefix + "reference_motion_dim_cfg"] = static_cast<double>(profile.cfg.reference_motion_dim);
        metadata.numeric_fields[prefix + "reference_motion_dim_loaded"] = static_cast<double>(profile.reference_motion.dim());
        metadata.numeric_fields[prefix + "reference_motion_frames"] = static_cast<double>(profile.reference_motion.frameCount());

        metadata.string_list_fields[prefix + "action_joint_order"] = profile.cfg.action_joint_order;
        metadata.string_list_fields[prefix + "obs_joint_order"] = profile.cfg.obs_joint_order;
        metadata.string_list_fields[prefix + "sub_model_names"] = collectSubModelNames(profile.cfg);
        metadata.string_list_fields[prefix + "sub_model_paths"] = collectSubModelPaths(profile.cfg);
        metadata.string_list_fields[prefix + "amp_discriminator_extra_outputs"] = profile.cfg.amp_discriminator.extra_output_names;
        metadata.string_list_fields[prefix + "reference_body_names_cfg"] = profile.cfg.reference_body_names;
        metadata.string_list_fields[prefix + "reference_body_names_loaded"] = profile.reference_motion.metadata().body_names;
    }

    if (!data_logger_->open(runtime_cfg.data_path, "controller", metadata))
    {
        std::cerr << "[RL_controller] failed to open structured data logger." << std::endl;
        data_logger_.reset();
        return;
    }

    data_logger_->writeEvent(
        rl_master::monotonicTimeSec(),
        "controller_initialized",
        {{"session_base_path", runtime_cfg.data_path}});
    std::cout << "RL Controller structured log: " << data_logger_->recordsPath() << std::endl;
}

void RL_controller::logStepRecord(
    double phase_t,
    double phase_t_global,
    int requested_mode_command,
    const rl_master::DeployStateOutput &deploy_output)
{
    if (!data_logger_ || !data_logger_->isOpen())
    {
        return;
    }

    std::map<std::string, double> scalars;
    scalars["frame_index"] = static_cast<double>(data_log_frame_index_++);
    scalars["phase_t"] = phase_t;
    scalars["phase_t_global"] = phase_t_global;
    scalars["phase_origin_t"] = phase_origin_t_;
    scalars["requested_mode_command"] = static_cast<double>(requested_mode_command);
    scalars["active_mode_id"] = static_cast<double>(deploy_output.locomotion_mode);
    scalars["deploy_state"] = static_cast<double>(static_cast<int>(deploy_output.state));
    scalars["active_profile_index"] = static_cast<double>(active_profile_index_);
    scalars["open_rl"] = static_cast<double>(robot->open_rl);
    scalars["cmd_vx"] = static_cast<double>(cmd.vx);
    scalars["cmd_vy"] = static_cast<double>(cmd.vy);
    scalars["cmd_dyaw"] = static_cast<double>(cmd.dyaw);

    std::map<std::string, std::vector<float>> vectors;
    vectors["joint_q"] = robot->joint_q;
    vectors["joint_dq"] = robot->joint_dq;
    vectors["joint_tau"] = robot->joint_tau;
    vectors["joint_target_q"] = robot->joint_target_q;
    vectors["joint_target_tau"] = robot->joint_target_tau;
    vectors["observation"] = obs;
    vectors["policy_action"] = action;
    const auto disc_score_key = activeModeProfile().tag + "/amp_discriminator/score";
    const auto disc_it = latest_policy_extra_outputs_.find(disc_score_key);
    if (disc_it != latest_policy_extra_outputs_.end())
    {
        vectors["amp_discriminator_score"] = disc_it->second;
        if (!disc_it->second.empty())
        {
            scalars["amp_discriminator_score_mean"] = static_cast<double>(meanOf(disc_it->second));
        }
    }

    data_logger_->writeRecord(rl_master::monotonicTimeSec(), "controller_step", scalars, vectors);
}

void RL_controller::RL_controller_Init(int startup_mode_id)
{
    cmd.vx = 0.0f;
    cmd.vy = 0.0f;
    cmd.dyaw = 0.0f;

    initModeProfiles();
    robot->initialize_buffers(joint_order_.size(), mode_profiles_.front().default_angle);
    robot->joint_names = joint_order_;

    int initial_mode_id = startup_mode_id;
    if (mode_to_profile_index_.find(initial_mode_id) == mode_to_profile_index_.end())
    {
        if (initial_mode_id != default_mode_id_)
        {
            std::cerr << "[RL_controller] startup mode_id " << initial_mode_id
                      << " not found in deploy_mode_profiles, fallback to default mode_id "
                      << default_mode_id_ << std::endl;
        }
        initial_mode_id = default_mode_id_;
    }

    refreshPolicyMode(initial_mode_id, true);
    handlePolicySwitch();
    deploy_state_machine_initialized_ = false;
    last_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;
    deploy_step_counter_ = 0;
    phase_origin_t_ = 0.0;
    phase_origin_initialized_ = false;
    phase_reset_pending_ = true;

    start_time = std::chrono::high_resolution_clock::now();
    initDataLogger();
}

rl_master::RobotCommandData RL_controller::step(
    const rl_master::RobotStateData &state,
    const rl_master::TeleopCommand &command,
    int mode_command,
    double phase_t)
{
    updateStateFromIO(state);
    updateCommandFromIO(command);

    if (!deploy_state_machine_initialized_)
    {
        handlePolicySwitch();
        deploy_state_machine_.configure(activePolicyCfg());
        deploy_state_machine_.initialize(robot->joint_q, activeZeroPose(), active_mode_id_);
        deploy_state_machine_initialized_ = true;
        last_deploy_state_ = deploy_state_machine_.state();
    }

    const double now_s = rl_master::monotonicTimeSec();
    const auto deploy_output = deploy_state_machine_.update(mode_command, now_s, robot->joint_q);
    const auto previous_state = last_deploy_state_;

    refreshPolicyMode(deploy_output.locomotion_mode, true);
    handlePolicySwitch();

    const bool entered_running =
        (deploy_output.state == rl_master::DeployLifecycleState::kRunning) &&
        (previous_state != rl_master::DeployLifecycleState::kRunning);
    if (!phase_origin_initialized_ || phase_reset_pending_ || entered_running)
    {
        phase_origin_t_ = phase_t;
        phase_origin_initialized_ = true;
        phase_reset_pending_ = false;
        if (entered_running)
        {
            deploy_step_counter_ = 0;
        }
    }
    const double local_phase_t = std::max(0.0, phase_t - phase_origin_t_);

    if (deploy_output.state != previous_state)
    {
        std::cout << "[RL_controller] lifecycle -> "
                  << rl_master::DeployStateMachine::stateName(deploy_output.state)
                  << std::endl;
        if (data_logger_ && data_logger_->isOpen())
        {
            data_logger_->writeEvent(
                rl_master::monotonicTimeSec(),
                "lifecycle_transition",
                {{"state", rl_master::DeployStateMachine::stateName(deploy_output.state)}});
        }
        last_deploy_state_ = deploy_output.state;
    }

    if (deploy_output.enable_policy)
    {
        std::vector<float> current_obs = get_robot_observation(local_phase_t);
        update_obs_deque(current_obs);

        const std::vector<float> policy_action = run_policy();
        robot->joint_target_q = get_joint_target_q(policy_action);
        robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
        robot->open_rl = rl_master::kOpenRlPolicyEnabled;
        ++deploy_step_counter_;
    }
    else if (deploy_output.enable_command_stream)
    {
        robot->joint_target_q.assign(robot->default_angle.begin(), robot->default_angle.end());
        const size_t copy_n = std::min(robot->joint_target_q.size(), deploy_output.target_q.size());
        for (size_t i = 0; i < copy_n; ++i)
        {
            robot->joint_target_q[i] = deploy_output.target_q[i];
        }
        robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
        // Non-policy command stream (for lifecycle actions like zeroing).
        robot->open_rl = rl_master::kOpenRlCommandStream;
        latest_policy_extra_outputs_.clear();
    }
    else
    {
        robot->joint_target_q = robot->joint_q;
        robot->joint_target_tau.assign(robot->joint_q.size(), 0.0f);
        robot->open_rl = rl_master::kOpenRlDisabled;
        latest_policy_extra_outputs_.clear();
    }

    rl_master::RobotCommandData out_cmd;
    out_cmd.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    out_cmd.active_joint_count = static_cast<int>(robot->joint_target_q.size());
    out_cmd.open_rl = robot->open_rl;
    out_cmd.joint_target_q = robot->joint_target_q;
    out_cmd.joint_target_dq.assign(robot->joint_target_q.size(), 0.0f);
    out_cmd.joint_target_tau = robot->joint_target_tau;

    logStepRecord(local_phase_t, phase_t, mode_command, deploy_output);
    return out_cmd;
}

void RL_controller::estop()
{
    robot->open_rl = rl_master::kOpenRlDisabled;
}

std::vector<float> RL_controller::get_robot_observation(double phase_t)
{
    const auto &active_cfg = activePolicyCfg();

    const ObservationFeatureContext feature_context = buildObservationFeatureContext(active_cfg, phase_t);
    latest_observation_feature_context_ = feature_context;
    obs = activeObservationBuilder().build(
        *robot,
        cmd,
        action,
        phase_t,
        active_cfg,
        currentObsIndexMap(),
        currentReferenceIndexMap(),
        feature_context);
    return obs;
}

std::vector<float> RL_controller::run_policy(std::deque<std::vector<float>> *obs_deque_ptr)
{
    if (!obs_deque_ptr)
    {
        obs_deque_ptr = &obs_deque;
    }

    const auto &active_cfg = activePolicyCfg();
    const size_t expected_obs_size = static_cast<size_t>(active_cfg.obs_dim) * obs_deque_ptr->size();
    if (stacked_obs_buffer_.size() != expected_obs_size)
    {
        stacked_obs_buffer_.assign(expected_obs_size, 0.0f);
    }

    size_t offset = 0;
    for (const auto &frame_obs : *obs_deque_ptr)
    {
        if (frame_obs.size() != static_cast<size_t>(active_cfg.obs_dim))
        {
            throw std::runtime_error(
                "Stacked observation frame dim mismatch. got=" + std::to_string(frame_obs.size()) +
                ", expected=" + std::to_string(active_cfg.obs_dim));
        }
        std::copy(frame_obs.begin(), frame_obs.end(), stacked_obs_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
        offset += frame_obs.size();
    }

    PolicyRunOutput policy_output = runPolicyGroup(
        &activePolicyGroup(),
        stacked_obs_buffer_,
        obs,
        latest_observation_feature_context_);
    std::vector<float> target_action = std::move(policy_output.action);
    latest_policy_extra_outputs_ = std::move(policy_output.extra_outputs);
    runAmpDiscriminator(
        active_cfg,
        activeModeProfile().tag,
        activeModeProfile().amp_discriminator.get(),
        obs,
        stacked_obs_buffer_);

    for (auto &value : target_action)
    {
        value = std::clamp(value, -active_cfg.clip_actions, active_cfg.clip_actions);
    }

    if (action.size() != target_action.size())
    {
        action.assign(target_action.size(), 0.0f);
    }
    for (size_t i = 0; i < target_action.size(); ++i)
    {
        target_action[i] = (1.0f - active_cfg.action_filter) * target_action[i] + active_cfg.action_filter * action[i];
    }

    action = target_action;
    return target_action;
}

std::deque<std::vector<float>> RL_controller::update_obs_deque(const std::vector<float> &new_obs)
{
    const auto &active_cfg = activePolicyCfg();
    if (new_obs.size() != static_cast<size_t>(active_cfg.obs_dim))
    {
        throw std::runtime_error(
            "Observation dim mismatch in deque update. got=" + std::to_string(new_obs.size()) +
            ", expected=" + std::to_string(active_cfg.obs_dim));
    }

    const size_t expected_stack = static_cast<size_t>(active_cfg.obs_stack_N);
    if (obs_deque.size() != expected_stack)
    {
        obs_deque.clear();
        for (size_t i = 0; i < expected_stack; ++i)
        {
            obs_deque.push_back(std::vector<float>(new_obs.size(), 0.0f));
        }
    }

    obs_deque.push_back(new_obs);
    while (obs_deque.size() > expected_stack)
    {
        obs_deque.pop_front();
    }
    return obs_deque;
}

std::vector<float> RL_controller::pd_control(
    const std::vector<float> &target_q,
    const std::vector<float> &kp,
    const std::vector<float> &target_dq,
    const std::vector<float> &kd)
{
    const std::vector<float> &q = robot->joint_q;
    const std::vector<float> &dq = robot->joint_dq;
    std::vector<float> torque(target_q.size(), 0.0f);

    const size_t n = std::min(
        std::min(target_q.size(), target_dq.size()),
        std::min(std::min(kp.size(), kd.size()), std::min(q.size(), dq.size())));

    for (size_t i = 0; i < n; ++i)
    {
        torque[i] = (target_q[i] - q[i]) * kp[i] + (target_dq[i] - dq[i]) * kd[i];
    }
    return torque;
}

std::vector<float> RL_controller::get_joint_target_torque(const std::vector<float> &target_q)
{
    const auto &active_cfg = activePolicyCfg();
    std::vector<float> target_dq(target_q.size(), 0.0f);
    std::vector<float> target_tau = pd_control(target_q, active_cfg.kps, target_dq, active_cfg.kds);
    for (size_t i = 0; i < target_tau.size(); ++i)
    {
        float limit = 0.0f;
        if (i < active_cfg.tau_limit.size())
        {
            limit = std::max(limit, std::abs(active_cfg.tau_limit[i]));
        }
        if (i < joint_order_.size())
        {
            const auto it = active_cfg.robotCfg.motor_torque_limit.find(joint_order_[i]);
            if (it != active_cfg.robotCfg.motor_torque_limit.end())
            {
                limit = std::max(limit, std::abs(it->second));
            }
        }
        if (limit > 0.0f)
        {
            target_tau[i] = std::clamp(target_tau[i], -limit, limit);
        }
    }

    joint_target_torque = target_tau;
    return target_tau;
}

std::vector<float> RL_controller::get_joint_target_q(const std::vector<float> &policy_action)
{
    const Sim2realCfg &active_cfg = activePolicyCfg();
    const std::vector<int> &action_robot_indices = currentActionIndexMap();
    std::vector<float> target_q = robot->default_angle;
    if (target_q.size() != joint_order_.size())
    {
        target_q.assign(joint_order_.size(), 0.0f);
    }
    const size_t action_count = std::min(
        policy_action.size(),
        std::min(action_robot_indices.size(), static_cast<size_t>(std::max(0, active_cfg.action_dim))));
    for (size_t policy_idx = 0; policy_idx < action_count; ++policy_idx)
    {
        const int robot_idx = action_robot_indices[policy_idx];
        if (robot_idx < 0 || static_cast<size_t>(robot_idx) >= target_q.size())
        {
            continue;
        }
        target_q[static_cast<size_t>(robot_idx)] =
            robot->default_angle[static_cast<size_t>(robot_idx)] +
            policy_action[policy_idx] * active_cfg.action_scale;
    }

    joint_target_q = target_q;
    return target_q;
}
