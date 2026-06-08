/*
onnxruntime:
https://blog.csdn.net/yangyu0515/article/details/142093965
https://blog.csdn.net/m0_57254760/article/details/138304321
*/
#include "rl_master/RL_controller.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "rl_master/pinocchio_motion_features.h"
#include "rl_master/rl_protocol.h"

namespace
{
std::string toLowerCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

std::vector<float> fitDim(const std::vector<float> &values, size_t dim)
{
    std::vector<float> out(dim, 0.0f);
    const size_t copy_n = std::min(values.size(), dim);
    std::copy(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(copy_n), out.begin());
    return out;
}

std::vector<std::string> jointOrderByAlias(
    const Sim2realCfg &cfg,
    const std::vector<std::string> &robot_global_joint_order,
    const std::string &alias,
    const std::string &label)
{
    const std::string normalized = toLowerCopy(alias);
    if (normalized.empty())
    {
        return {};
    }
    if (normalized == "robot_global_joint_order" || normalized == "global_joint_order")
    {
        return robot_global_joint_order;
    }
    if (normalized == "action_joint_order" || normalized == "action_order")
    {
        return cfg.action_joint_order;
    }
    if (normalized == "obs_joint_order" || normalized == "observation_joint_order" || normalized == "observation_order")
    {
        return cfg.obs_joint_order;
    }
    if (normalized == "reference_joint_order" || normalized == "reference_order")
    {
        return cfg.reference_joint_order;
    }
    throw std::runtime_error(label + " unknown joint order alias: " + alias);
}

std::vector<int> buildIndexMapFromJointOrders(
    const std::vector<std::string> &source_order,
    const std::vector<std::string> &target_order,
    const std::string &label)
{
    if (source_order.empty() || target_order.empty())
    {
        return {};
    }

    std::unordered_map<std::string, int> source_index;
    source_index.reserve(source_order.size());
    for (size_t i = 0; i < source_order.size(); ++i)
    {
        if (!source_index.emplace(source_order[i], static_cast<int>(i)).second)
        {
            throw std::runtime_error(label + " source_order contains duplicate joint: " + source_order[i]);
        }
    }

    std::vector<int> indices;
    indices.reserve(target_order.size());
    for (const std::string &joint_name : target_order)
    {
        const auto it = source_index.find(joint_name);
        if (it == source_index.end())
        {
            throw std::runtime_error(label + " target joint is missing from source_order: " + joint_name);
        }
        indices.push_back(it->second);
    }
    return indices;
}

std::vector<int> buildIndexMapFromJointOrderAliases(
    const Sim2realCfg &cfg,
    const std::vector<std::string> &robot_global_joint_order,
    const std::string &source_order_alias,
    const std::string &target_order_alias,
    const std::string &label)
{
    if (source_order_alias.empty() && target_order_alias.empty())
    {
        return {};
    }
    if (source_order_alias.empty() || target_order_alias.empty())
    {
        throw std::runtime_error(label + " requires both source_order and target_order");
    }
    return buildIndexMapFromJointOrders(
        jointOrderByAlias(cfg, robot_global_joint_order, source_order_alias, label + ".source_order"),
        jointOrderByAlias(cfg, robot_global_joint_order, target_order_alias, label + ".target_order"),
        label);
}

bool parsePositiveIntValue(const std::string &text, int *out)
{
    if (!out)
    {
        return false;
    }
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
    {
        return false;
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    const std::string trimmed = text.substr(first, last - first + 1);
    try
    {
        size_t parsed_count = 0;
        const int value = std::stoi(trimmed, &parsed_count, 10);
        if (parsed_count != trimmed.size())
        {
            return false;
        }
        if (value <= 0)
        {
            return false;
        }
        *out = value;
        return true;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

float applyDcMotorTorqueSpeedClip(
    float effort,
    float joint_vel,
    float saturation_effort,
    float effort_limit,
    float velocity_limit)
{
    if (saturation_effort <= 0.0f || effort_limit <= 0.0f || velocity_limit <= 0.0f)
    {
        return effort;
    }

    const float vel_at_effort_limit = velocity_limit * (1.0f + effort_limit / saturation_effort);
    const float clipped_joint_vel = std::clamp(joint_vel, -vel_at_effort_limit, vel_at_effort_limit);
    const float torque_speed_top = saturation_effort * (1.0f - clipped_joint_vel / velocity_limit);
    const float torque_speed_bottom = saturation_effort * (-1.0f - clipped_joint_vel / velocity_limit);
    const float max_effort = std::min(torque_speed_top, effort_limit);
    const float min_effort = std::max(torque_speed_bottom, -effort_limit);
    return std::clamp(effort, min_effort, max_effort);
}

float clampTargetQToVelocityEnvelope(
    float target_position,
    float current_position,
    float current_velocity,
    float kp,
    float kd,
    float x1,
    float x2,
    float y1,
    float y2,
    float zero_velocity_epsilon)
{
    if (kp <= 0.0f || x2 <= x1 || y1 <= 0.0f || y2 <= 0.0f)
    {
        return target_position;
    }

    const float abs_velocity = std::abs(current_velocity);
    const float over_velocity = std::max(0.0f, abs_velocity - x1);
    const float span = std::max(1.0e-6f, x2 - x1);

    const float positive_base =
        (abs_velocity <= zero_velocity_epsilon) ? y2 : (current_velocity >= 0.0f ? y1 : y2);
    const float positive_slope = positive_base / span;
    const float tau_high = std::max(0.0f, positive_base - positive_slope * over_velocity);

    const float negative_base =
        (abs_velocity <= zero_velocity_epsilon) ? -y2 : (current_velocity >= 0.0f ? -y2 : -y1);
    const float negative_slope = (-negative_base) / span;
    const float tau_low = std::min(0.0f, negative_base + negative_slope * over_velocity);

    const float p_low = tau_low + kd * current_velocity;
    const float p_high = tau_high + kd * current_velocity;
    const float target_low = p_low / kp + current_position;
    const float target_high = p_high / kp + current_position;
    return std::clamp(target_position, target_low, target_high);
}

std::array<float, 3> rotateVectorByQuat(
    const std::array<float, 3> &vec,
    const std::array<float, 4> &quat_xyzw)
{
    const float x = quat_xyzw[0];
    const float y = quat_xyzw[1];
    const float z = quat_xyzw[2];
    const float w = quat_xyzw[3];
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;
    return {
        (1.0f - 2.0f * (yy + zz)) * vec[0] + 2.0f * (xy - wz) * vec[1] + 2.0f * (xz + wy) * vec[2],
        2.0f * (xy + wz) * vec[0] + (1.0f - 2.0f * (xx + zz)) * vec[1] + 2.0f * (yz - wx) * vec[2],
        2.0f * (xz - wy) * vec[0] + 2.0f * (yz + wx) * vec[1] + (1.0f - 2.0f * (xx + yy)) * vec[2]};
}

std::array<float, 3> rotateWorldVectorToBodyFrame(
    const std::array<float, 3> &vec_world,
    const std::array<float, 4> &body_quat_xyzw)
{
    const std::array<float, 4> quat_conjugate = {
        -body_quat_xyzw[0],
        -body_quat_xyzw[1],
        -body_quat_xyzw[2],
        body_quat_xyzw[3]};
    return rotateVectorByQuat(vec_world, quat_conjugate);
}

float vectorNorm3(const std::array<float, 3> &vec)
{
    return std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

std::array<float, 4> normalizedQuatXyzw(std::array<float, 4> quat)
{
    const float norm = std::sqrt(
        quat[0] * quat[0] +
        quat[1] * quat[1] +
        quat[2] * quat[2] +
        quat[3] * quat[3]);
    if (!std::isfinite(norm) || norm < 1.0e-8f)
    {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }
    for (float &value : quat)
    {
        value /= norm;
    }
    return quat;
}

std::array<float, 4> multiplyQuatXyzw(
    const std::array<float, 4> &a,
    const std::array<float, 4> &b)
{
    return normalizedQuatXyzw({
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    });
}

std::array<float, 4> conjugateQuatXyzw(const std::array<float, 4> &quat)
{
    return {-quat[0], -quat[1], -quat[2], quat[3]};
}

std::vector<float> quatXyzwToRot6(const std::array<float, 4> &quat_raw)
{
    const std::array<float, 4> quat = normalizedQuatXyzw(quat_raw);
    const float x = quat[0];
    const float y = quat[1];
    const float z = quat[2];
    const float w = quat[3];
    const float xx = x * x;
    const float yy = y * y;
    const float zz = z * z;
    const float xy = x * y;
    const float xz = x * z;
    const float yz = y * z;
    const float wx = w * x;
    const float wy = w * y;
    const float wz = w * z;
    return {
        1.0f - 2.0f * (yy + zz),
        2.0f * (xy - wz),
        2.0f * (xy + wz),
        1.0f - 2.0f * (xx + zz),
        2.0f * (xz - wy),
        2.0f * (yz + wx),
    };
}

std::vector<float> referenceAnchorOri6dFromBodyQuat(
    const std::vector<float> &reference_body_quat_w_xyzw,
    const std::vector<float> &robot_base_quat_xyzw,
    size_t body_quat_index)
{
    constexpr size_t kQuatDim = 4;
    const size_t ref_offset = body_quat_index * kQuatDim;
    if (reference_body_quat_w_xyzw.size() < ref_offset + kQuatDim ||
        robot_base_quat_xyzw.size() < kQuatDim)
    {
        return {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
    }
    const std::array<float, 4> current_anchor_quat = normalizedQuatXyzw({
        robot_base_quat_xyzw[0],
        robot_base_quat_xyzw[1],
        robot_base_quat_xyzw[2],
        robot_base_quat_xyzw[3],
    });
    const std::array<float, 4> reference_anchor_quat = normalizedQuatXyzw({
        reference_body_quat_w_xyzw[ref_offset + 0],
        reference_body_quat_w_xyzw[ref_offset + 1],
        reference_body_quat_w_xyzw[ref_offset + 2],
        reference_body_quat_w_xyzw[ref_offset + 3],
    });
    const std::array<float, 4> relative_quat =
        multiplyQuatXyzw(conjugateQuatXyzw(current_anchor_quat), reference_anchor_quat);
    return quatXyzwToRot6(relative_quat);
}

void markRequiredReferenceSource(
    const std::string &canonical_source_raw,
    ReferenceFeatureRequirements *requirements)
{
    if (!requirements)
    {
        return;
    }

    const std::string canonical_source = toLowerCopy(canonical_source_raw);
    if (canonical_source == "reference_motion")
    {
        requirements->reference_motion = true;
    }
    else if (canonical_source == "reference_joint_pos")
    {
        requirements->reference_joint_pos = true;
    }
    else if (canonical_source == "reference_joint_vel")
    {
        requirements->reference_joint_vel = true;
    }
    else if (canonical_source == "reference_body_pos_w")
    {
        requirements->reference_body_pos_w = true;
    }
    else if (canonical_source == "reference_body_quat_w")
    {
        requirements->reference_body_quat_w = true;
    }
    else if (canonical_source == "motion_anchor_pos_b" ||
             canonical_source == "motion_ref_pos_b" ||
             canonical_source == "motion_anchor_ori_b" ||
             canonical_source == "motion_ref_ori_b" ||
             canonical_source == "motion_body_pos_b" ||
             canonical_source == "motion_body_ori_b")
    {
        requirements->reference_body_pos_w = true;
        requirements->reference_body_quat_w = true;
        requirements->named_body_layout = true;
    }
    else if (canonical_source == "robot_body_pos" ||
             canonical_source == "robot_body_ori")
    {
        requirements->named_body_layout = true;
    }
}

ReferenceFeatureRequirements collectRequiredReferenceFeatures(const ObservationManifest &manifest)
{
    ReferenceFeatureRequirements requirements;
    for (const auto &term : manifest.terms())
    {
        if (!term.enabled)
        {
            continue;
        }

        if (term.name == "reference_motion")
        {
            requirements.reference_motion = true;
            continue;
        }

        if (term.name == "reference_joint_pos")
        {
            markRequiredReferenceSource(
                term.source.empty() ? "reference_joint_pos" : term.source,
                &requirements);
            continue;
        }

        if (term.name == "reference_joint_vel")
        {
            markRequiredReferenceSource(
                term.source.empty() ? "reference_joint_vel" : term.source,
                &requirements);
            continue;
        }

        if (term.name == "motion_anchor_pos_b" ||
            term.name == "motion_ref_pos_b" ||
            term.name == "motion_anchor_ori_b" ||
            term.name == "motion_ref_ori_b" ||
            term.name == "motion_body_pos_b" ||
            term.name == "motion_body_ori_b")
        {
            markRequiredReferenceSource(
                term.source.empty() ? term.name : term.source,
                &requirements);
            continue;
        }

        if ((term.name == "feature" || term.name == "external_sensor") && !term.source.empty())
        {
            markRequiredReferenceSource(term.source, &requirements);
        }
        if (term.name == "robot_body_pos" || term.name == "robot_body_ori")
        {
            requirements.named_body_layout = true;
        }
    }
    return requirements;
}

void collectRequiredReferenceFeatures(
    const std::vector<ComputedFeatureCfg> &features,
    ReferenceFeatureRequirements *requirements)
{
    if (!requirements)
    {
        return;
    }
    for (const auto &feature : features)
    {
        for (const auto &part : feature.parts)
        {
            if (part.source == "reference_joint_pos")
            {
                requirements->reference_joint_pos = true;
            }
            else if (part.source == "reference_joint_vel")
            {
                requirements->reference_joint_vel = true;
            }
            else if (part.source == "reference_anchor_ori6d")
            {
                requirements->reference_body_quat_w = true;
            }
            else if (part.source == "feature" && !part.feature_name.empty())
            {
                markRequiredReferenceSource(part.feature_name, requirements);
            }
        }
    }
}

std::vector<ComputedFeatureCfg> mergedComputedFeatures(
    const ObservationManifest &manifest,
    const Sim2realCfg &cfg)
{
    std::vector<ComputedFeatureCfg> features = manifest.computedFeatures();
    features.insert(features.end(), cfg.computed_features.begin(), cfg.computed_features.end());
    return features;
}

void requireNonEmptyReferenceKey(
    const std::string &key_value,
    const std::string &tag,
    const char *contract_scope,
    const char *field_name)
{
    if (!key_value.empty())
    {
        return;
    }
    throw std::runtime_error(
        "[RL_controller][" + tag + "] " + contract_scope + "." + field_name +
        " must be non-empty because the active observation manifest requires that reference feature.");
}

void validateRequiredReferenceSourceContract(
    const Sim2realCfg &cfg,
    const ReferenceFeatureRequirements &requirements,
    const std::string &tag)
{
    if (!cfg.enable_reference_motion || !requirements.sourceAny())
    {
        return;
    }

    const std::string source = toLowerCopy(cfg.reference_motion_source);
    if (source != "policy_outputs")
    {
        if (requirements.reference_motion)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.reference_file.reference_motion_key,
                tag,
                "source_contract.reference_file",
                "reference_motion_key");
        }
        if (requirements.reference_joint_pos)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.reference_file.reference_joint_pos_key,
                tag,
                "source_contract.reference_file",
                "reference_joint_pos_key");
        }
        if (requirements.reference_joint_vel)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.reference_file.reference_joint_vel_key,
                tag,
                "source_contract.reference_file",
                "reference_joint_vel_key");
        }
        if (requirements.reference_body_pos_w)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.reference_file.reference_body_pos_w_key,
                tag,
                "source_contract.reference_file",
                "reference_body_pos_w_key");
        }
        if (requirements.reference_body_quat_w)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.reference_file.reference_body_quat_w_key,
                tag,
                "source_contract.reference_file",
                "reference_body_quat_w_key");
        }
    }

    if (source != "file")
    {
        if (requirements.reference_motion)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.policy_extra_outputs.reference_motion_key,
                tag,
                "source_contract.policy_extra_outputs",
                "reference_motion_key");
        }
        if (requirements.reference_joint_pos)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.policy_extra_outputs.reference_joint_pos_key,
                tag,
                "source_contract.policy_extra_outputs",
                "reference_joint_pos_key");
        }
        if (requirements.reference_joint_vel)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.policy_extra_outputs.reference_joint_vel_key,
                tag,
                "source_contract.policy_extra_outputs",
                "reference_joint_vel_key");
        }
        if (requirements.reference_body_pos_w)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.policy_extra_outputs.reference_body_pos_w_key,
                tag,
                "source_contract.policy_extra_outputs",
                "reference_body_pos_w_key");
        }
        if (requirements.reference_body_quat_w)
        {
            requireNonEmptyReferenceKey(
                cfg.source_contract.policy_extra_outputs.reference_body_quat_w_key,
                tag,
                "source_contract.policy_extra_outputs",
                "reference_body_quat_w_key");
        }
    }
}

std::vector<std::string> effectiveReferenceBodyNames(
    const Sim2realCfg &cfg,
    const ReferenceMotionProvider *provider)
{
    std::vector<std::string> body_names = cfg.reference_body_names;
    if (provider && provider->available())
    {
        const auto &metadata = provider->metadata();
        if (!metadata.body_names.empty())
        {
            body_names = metadata.body_names;
        }
    }
    return body_names;
}

std::string effectiveReferenceAnchorBody(
    const Sim2realCfg &cfg,
    const ReferenceMotionProvider *provider)
{
    std::string anchor_body = cfg.reference_anchor_body;
    if (provider && provider->available())
    {
        const auto &metadata = provider->metadata();
        if (!metadata.anchor_body.empty())
        {
            anchor_body = metadata.anchor_body;
        }
    }
    return anchor_body;
}

void validateRequiredNamedBodyLayout(
    const Sim2realCfg &cfg,
    const ReferenceFeatureRequirements &requirements,
    const ReferenceMotionProvider *provider,
    const std::string &tag)
{
    if (!requirements.named_body_layout)
    {
        return;
    }

    const std::vector<std::string> body_names = effectiveReferenceBodyNames(cfg, provider);
    const std::string anchor_body = effectiveReferenceAnchorBody(cfg, provider);
    if (body_names.empty())
    {
        throw std::runtime_error(
            "[RL_controller][" + tag +
            "] active observation manifest requires body-name-based features, "
            "but no explicit body_names are available from reference_body_names or reference file metadata.");
    }
    if (anchor_body.empty())
    {
        throw std::runtime_error(
            "[RL_controller][" + tag +
            "] active observation manifest requires body-name-based features, "
            "but anchor_body is empty.");
    }
    const auto it = std::find(body_names.begin(), body_names.end(), anchor_body);
    if (it == body_names.end())
    {
        throw std::runtime_error(
            "[RL_controller][" + tag +
            "] active observation manifest requires body-name-based features, "
            "but anchor_body '" + anchor_body + "' is not present in the effective body_names.");
    }
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

void setFeatureIfNonEmptyWithContract(
    ObservationFeatureContext *feature_context,
    const std::string &name,
    const std::vector<float> &values,
    const ObservationFeatureContract &contract)
{
    if (!feature_context || values.empty())
    {
        return;
    }
    feature_context->named_features[name] = values;
    feature_context->named_feature_contracts[name] = contract;
}

const std::vector<float> *findPolicyOutputByName(
    const std::unordered_map<std::string, std::vector<float>> &prefetched_extra_outputs,
    const std::unordered_map<std::string, std::vector<float>> &latest_extra_outputs,
    const std::string &preferred_prefix,
    const std::string &output_name)
{
    if (const auto *prefetched =
            findExtraOutputByName(prefetched_extra_outputs, preferred_prefix, output_name))
    {
        return prefetched;
    }
    return findExtraOutputByName(latest_extra_outputs, preferred_prefix, output_name);
}

std::vector<float> convertQuatVectorToCanonical(
    const std::vector<float> &values,
    const std::string &source_order,
    const std::string &canonical_order)
{
    const std::string normalized_canonical = toLowerCopy(canonical_order);
    if (normalized_canonical != "xyzw")
    {
        throw std::runtime_error(
            "Unsupported canonical quaternion order: " + canonical_order +
            ". Current implementation only supports xyzw.");
    }
    return convertQuatVectorToXyzw(values, source_order);
}

std::vector<float> gatherByIndicesOrZeros(
    const std::vector<float> &values,
    const std::vector<int> &indices)
{
    std::vector<float> out;
    out.reserve(indices.size());
    for (const int index : indices)
    {
        if (index >= 0 && static_cast<size_t>(index) < values.size())
        {
            out.push_back(values[static_cast<size_t>(index)]);
        }
        else
        {
            out.push_back(0.0f);
        }
    }
    return out;
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
    deploy_state_machine_.setHotSwitchPredicate(
        [this](int from_mode, int to_mode) {
            return this->canHotSwitch(from_mode, to_mode);
        });
}

RL_controller::~RL_controller()
{
}

std::unique_ptr<RL_controller> RL_controller::create(std::shared_ptr<const rl_master::ModeProfileRegistry> mode_registry)
{
    return std::make_unique<RL_controller>(std::move(mode_registry));
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

std::vector<std::string> RL_controller::activeResolvedReferenceBodyNames() const
{
    const Sim2realCfg &cfg = activePolicyCfg();
    return effectiveReferenceBodyNames(cfg, &activeReferenceMotionProvider());
}

std::string RL_controller::activeResolvedReferenceAnchorBody() const
{
    const Sim2realCfg &cfg = activePolicyCfg();
    return effectiveReferenceAnchorBody(cfg, &activeReferenceMotionProvider());
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
    observation_history_prefill_pending_ = true;
    prefetched_policy_extra_outputs_.clear();
    latest_policy_extra_outputs_.clear();
    pending_auto_mode_switch_target_mode_id_ = -1;
    pending_auto_mode_switch_trigger_step_ = 0;
    pending_auto_mode_switch_reason_.clear();
    resetPolicyScheduler();
    phase_reset_pending_ = true;
    auto &profile = activeModeProfile();
    if (profile.pinocchio_motion_features)
    {
        profile.pinocchio_motion_features->resetAlignment();
    }
    if (cfg.source_contract.base_velocity_estimator.reset_on_mode_switch)
    {
        base_velocity_filter_.reset();
        base_velocity_filter_time_initialized_ = false;
    }

    if (cfg.reset_policy_on_mode_switch)
    {
        auto &group = activePolicyGroup();
        if (group.strategy)
        {
            group.strategy->reset();
        }
        for (auto &node : group.runners)
        {
            if (node.runner)
            {
                node.runner->reset();
            }
        }
    }

    deploy_state_machine_.configure(cfg);
    deploy_state_machine_.setZeroPose(activeZeroPose());

    last_active_mode_id_ = active_mode_id_;
    std::cout << "[RL_controller] switch policy to "
              << activeModeProfile().tag
              << ", mode_id=" << active_mode_id_ << std::endl;
}

void RL_controller::queueRuntimeWarningEvent(
    const std::string &event_type,
    const std::string &message,
    const std::map<std::string, std::string> &tags)
{
    pending_runtime_warning_seq_ = ++runtime_warning_seq_counter_;
    pending_runtime_warning_type_ = event_type;
    pending_runtime_warning_message_ = message;
    pending_runtime_warning_tags_ = tags;
}

bool RL_controller::isKnownMode(int mode_id) const
{
    return mode_to_profile_index_.find(mode_id) != mode_to_profile_index_.end();
}

int RL_controller::sanitizeRuntimeModeCommand(int mode_command)
{
    const bool is_start_mode_command =
        mode_command >= rl_master::kCtrlWordStartModeBase &&
        mode_command < (rl_master::kCtrlWordStartModeBase + rl_master::kCtrlWordModeRange);
    const bool is_set_mode_command =
        mode_command >= rl_master::kCtrlWordSetModeBase &&
        mode_command < (rl_master::kCtrlWordSetModeBase + rl_master::kCtrlWordModeRange);

    if (!is_start_mode_command && !is_set_mode_command)
    {
        return mode_command;
    }

    const int requested_mode = is_start_mode_command
                                   ? (mode_command - rl_master::kCtrlWordStartModeBase)
                                   : (mode_command - rl_master::kCtrlWordSetModeBase);
    if (isKnownMode(requested_mode))
    {
        return mode_command;
    }

    if (last_rejected_mode_command_ != mode_command ||
        last_rejected_mode_id_ != requested_mode)
    {
        const std::string warning_message =
            "ignore invalid runtime mode request and keep current active mode";
        std::cerr << "[RL_controller] " << warning_message
                  << ": control_word=" << mode_command
                  << ", requested_mode_id=" << requested_mode
                  << ", keep active_mode_id=" << active_mode_id_
                  << std::endl;
        queueRuntimeWarningEvent(
            "invalid_runtime_mode_request_ignored",
            warning_message,
            {
                {"control_word", std::to_string(mode_command)},
                {"requested_mode_id", std::to_string(requested_mode)},
                {"kept_active_mode_id", std::to_string(active_mode_id_)},
            });
        last_rejected_mode_command_ = mode_command;
        last_rejected_mode_id_ = requested_mode;
    }

    if (is_start_mode_command)
    {
        return rl_master::kCtrlWordStartPolicy;
    }
    return -1;
}

void RL_controller::resetPolicyScheduler()
{
    policy_step_counter_ = 0;
    policy_schedule_initialized_ = false;
    next_policy_phase_t_ = 0.0;
    last_policy_sample_time_sec_ = 0.0;
    last_policy_sample_phase_t_ = 0.0;
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

const rl_master::logging::ControllerLogSnapshot &RL_controller::latestLogSnapshot() const
{
    return latest_log_snapshot_;
}

void RL_controller::setExternalObservationFeature(const std::string &name, const std::vector<float> &values)
{
    external_observation_provider_.setFeature(name, values);
}

void RL_controller::setExternalObservationFeature(const std::string &name, const std::vector<float> &values, double monotonic_time_sec)
{
    external_observation_provider_.setFeature(name, values, monotonic_time_sec);
}

std::vector<rl_master::logging::RuntimeSourceSampleRecord> RL_controller::drainExternalObservationSamplesForLogging()
{
    std::vector<rl_master::logging::RuntimeSourceSampleRecord> out;
    const auto samples = external_observation_provider_.drainUpdatedSamples(activePolicyCfg().external_observations);
    out.reserve(samples.size());
    for (const auto &sample : samples)
    {
        rl_master::logging::RuntimeSourceSampleRecord record;
        record.monotonic_time_sec = sample.monotonic_time_sec;
        record.topic = "runtime/source/external/" + sample.name;
        record.sample_name = sample.name;
        record.tags["mode_id"] = std::to_string(activeModeId());
        record.tags["config_section"] = activeConfigSection();
        record.tags["update_seq"] = std::to_string(sample.update_seq);
        record.values["value"] = sample.values;
        out.push_back(std::move(record));
    }
    return out;
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

const RL_controller::ModeProfile &RL_controller::modeProfileForModeId(int mode_id) const
{
    const size_t profile_index = profileIndexForMode(mode_id, false);
    if (profile_index >= mode_profiles_.size())
    {
        throw std::runtime_error("Resolved mode profile index is invalid");
    }
    return mode_profiles_[profile_index];
}

void RL_controller::buildStackedObservation(
    const std::deque<std::vector<float>> &observation_history,
    const char *context)
{
    const auto &active_cfg = activePolicyCfg();
    const size_t frame_dim = static_cast<size_t>(active_cfg.obs_dim);
    const size_t expected_obs_size = frame_dim * observation_history.size();
    if (stacked_obs_buffer_.size() != expected_obs_size)
    {
        stacked_obs_buffer_.assign(expected_obs_size, 0.0f);
    }

    for (const auto &frame_obs : observation_history)
    {
        if (frame_obs.size() != frame_dim)
        {
            throw std::runtime_error(
                std::string("Stacked observation frame dim mismatch") +
                (context ? std::string(" during ") + context : std::string()) +
                ". got=" + std::to_string(frame_obs.size()) +
                ", expected=" + std::to_string(active_cfg.obs_dim));
        }
    }

    if (active_cfg.observation_stack_layout == "frame_major")
    {
        size_t offset = 0;
        for (const auto &frame_obs : observation_history)
        {
            std::copy(
                frame_obs.begin(),
                frame_obs.end(),
                stacked_obs_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
            offset += frame_obs.size();
        }
        return;
    }

    if (active_cfg.observation_stack_layout != "term_major")
    {
        throw std::runtime_error(
            "Unsupported observation_stack_layout: " + active_cfg.observation_stack_layout);
    }

    size_t offset = 0;
    for (const auto &term : activeObservationBuilder().termLayout())
    {
        if (term.offset + term.dim > frame_dim)
        {
            throw std::runtime_error(
                "Observation term layout exceeds frame dim for term '" + term.name + "'");
        }
        for (const auto &frame_obs : observation_history)
        {
            std::copy(
                frame_obs.begin() + static_cast<std::ptrdiff_t>(term.offset),
                frame_obs.begin() + static_cast<std::ptrdiff_t>(term.offset + term.dim),
                stacked_obs_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
            offset += term.dim;
        }
    }
}

bool RL_controller::canHotSwitch(int from_mode, int to_mode) const
{
    if (from_mode == to_mode)
    {
        return true;
    }

    const auto from_it = mode_to_profile_index_.find(from_mode);
    const auto to_it = mode_to_profile_index_.find(to_mode);
    if (from_it == mode_to_profile_index_.end() ||
        to_it == mode_to_profile_index_.end())
    {
        std::cout << "[RL_controller] hot switch rejected: unknown mode id transition from "
                  << from_mode << " to " << to_mode << std::endl;
        return false;
    }

    const auto &from_profile = modeProfileForModeId(from_mode);
    const auto &to_profile = modeProfileForModeId(to_mode);
    const auto &from_cfg = from_profile.cfg;
    const auto &to_cfg = to_profile.cfg;

    auto reject = [&](const std::string &reason) {
        std::cout << "[RL_controller] hot switch rejected: from mode_id=" << from_mode
                  << " (" << from_profile.tag << ")"
                  << " to mode_id=" << to_mode
                  << " (" << to_profile.tag << ")"
                  << ", reason=" << reason << std::endl;
        return false;
    };

    if (from_cfg.policy_family != to_cfg.policy_family)
    {
        return reject(
            "policy_family mismatch: '" + from_cfg.policy_family +
            "' vs '" + to_cfg.policy_family + "'");
    }
    if (from_cfg.action_dim != to_cfg.action_dim)
    {
        return reject("action_dim mismatch");
    }
    if (from_cfg.control_mode != to_cfg.control_mode)
    {
        return reject(
            "control_mode mismatch: '" + from_cfg.control_mode +
            "' vs '" + to_cfg.control_mode + "'");
    }
    if (from_cfg.action_joint_order != to_cfg.action_joint_order)
    {
        return reject("action_joint_order mismatch");
    }
    if (from_cfg.installed_joint_run_modes != to_cfg.installed_joint_run_modes)
    {
        return reject("installed_joint_run_modes mismatch");
    }
    if (from_cfg.obs_joint_order != to_cfg.obs_joint_order)
    {
        return reject("obs_joint_order mismatch");
    }
    if (from_cfg.reference_joint_order != to_cfg.reference_joint_order)
    {
        return reject("reference_joint_order mismatch");
    }
    if (from_cfg.enable_reference_motion != to_cfg.enable_reference_motion)
    {
        return reject("enable_reference_motion mismatch");
    }
    if (from_cfg.observation_manifest_path != to_cfg.observation_manifest_path)
    {
        return reject("observation_manifest_path mismatch");
    }
    if (from_cfg.observation_stack_layout != to_cfg.observation_stack_layout)
    {
        return reject("observation_stack_layout mismatch");
    }
    if (from_cfg.external_observations.size() != to_cfg.external_observations.size())
    {
        return reject("external_observations size mismatch");
    }
    for (size_t i = 0; i < from_cfg.external_observations.size(); ++i)
    {
        const auto &from_spec = from_cfg.external_observations[i];
        const auto &to_spec = to_cfg.external_observations[i];
        if (from_spec.name != to_spec.name ||
            from_spec.dim != to_spec.dim ||
            from_spec.required != to_spec.required ||
            from_spec.topic != to_spec.topic ||
            from_spec.message_type != to_spec.message_type)
        {
            return reject("external_observations contract mismatch");
        }
    }
    return true;
}

size_t RL_controller::profileIndexForMode(int mode_id, bool sanitize_invalid_mode) const
{
    (void)sanitize_invalid_mode;
    const auto it = mode_to_profile_index_.find(mode_id);
    if (it != mode_to_profile_index_.end())
    {
        return it->second;
    }
    throw std::runtime_error("Unknown mode id: " + std::to_string(mode_id));
}

std::vector<RL_controller::ModeProfileSpec> RL_controller::loadModeProfileSpecsFromYaml() const
{
    std::shared_ptr<const rl_master::ModeProfileRegistry> registry = mode_registry_;
    if (!registry)
    {
        throw std::runtime_error(
            "RL_controller requires an injected ModeProfileRegistry. "
            "Lazy registry self-loading has been disabled for strict debugging.");
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
        throw std::runtime_error(
            "RL_controller::initModeProfiles requires an injected ModeProfileRegistry. "
            "Lazy registry self-loading has been disabled for strict debugging.");
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
        const auto &layout = mode_registry_->layoutForSection(spec.config_section);

        ModeProfile profile;
        profile.mode_id = spec.mode_id;
        profile.config_section = spec.config_section;
        profile.tag = spec.tag;
        profile.cfg = cfg;
        profile.joint_names = joint_order_;
        profile.default_angle = layout.default_angle;
        profile.zero_pose = layout.zero_pose;
        profile.action_robot_indices = layout.action_global_indices;
        profile.obs_index_map = layout.obs_global_indices;
        profile.reference_index_map = layout.reference_global_indices;
        profile.observation_manifest = ObservationManifest::loadFromYAML(profile.cfg.observation_manifest_path);
        profile.required_reference_features = collectRequiredReferenceFeatures(profile.observation_manifest);
        collectRequiredReferenceFeatures(
            mergedComputedFeatures(profile.observation_manifest, profile.cfg),
            &profile.required_reference_features);
        validateRequiredReferenceSourceContract(profile.cfg, profile.required_reference_features, profile.tag);
        profile.observation_builder = std::make_unique<ObservationBuilder>(profile.observation_manifest);
        if (profile.observation_builder->expectedDim() != static_cast<size_t>(profile.cfg.obs_dim))
        {
            throw std::runtime_error(
                profile.tag + " observation manifest dim (" +
                std::to_string(profile.observation_builder->expectedDim()) +
                ") does not match cfg obs_dim (" + std::to_string(profile.cfg.obs_dim) + ")");
        }
        initPolicyGroup(profile.cfg, profile.tag, &profile.policy_group);
        initReferenceMotionProvider(
            profile.cfg,
            profile.required_reference_features,
            &profile.reference_motion,
            profile.tag);
        profile.resolved_reference_end_total_steps =
            resolveReferenceEndAutoSwitchTotalSteps(profile);
        validateRequiredNamedBodyLayout(
            profile.cfg,
            profile.required_reference_features,
            &profile.reference_motion,
            profile.tag);
        if (profile.required_reference_features.named_body_layout)
        {
            if (profile.cfg.pinocchio_urdf_path.empty())
            {
                throw std::runtime_error(
                    "[RL_controller][" + profile.tag +
                    "] named-body observation features require pinocchio_urdf_path or pinocchio_urdf_file.");
            }
            if (!std::filesystem::exists(profile.cfg.pinocchio_urdf_path))
            {
                throw std::runtime_error(
                    "[RL_controller][" + profile.tag +
                    "] pinocchio URDF does not exist: " + profile.cfg.pinocchio_urdf_path);
            }

            profile.pinocchio_motion_features = std::make_unique<rl_master::PinocchioMotionFeatures>(
                profile.cfg.pinocchio_urdf_path,
                profile.joint_names);
            if (!profile.pinocchio_motion_features->available())
            {
                throw std::runtime_error(
                    "[RL_controller][" + profile.tag +
                    "] failed to initialize Pinocchio motion features: " +
                    profile.pinocchio_motion_features->lastError());
            }
        }
        if (profile.cfg.reference_anchor_current_source == "fk_onnx")
        {
            if (profile.cfg.reference_anchor_fk_path.empty() ||
                !std::filesystem::exists(profile.cfg.reference_anchor_fk_path))
            {
                throw std::runtime_error(
                    "[RL_controller][" + profile.tag +
                    "] reference_anchor_current_source='fk_onnx' but FK ONNX does not exist: " +
                    profile.cfg.reference_anchor_fk_path);
            }
            Ort::SessionOptions fk_session_options;
            fk_session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
            fk_session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
            fk_session_options.SetIntraOpNumThreads(std::max(1, profile.cfg.onnx_intra_threads));
            fk_session_options.SetInterOpNumThreads(std::max(1, profile.cfg.onnx_inter_threads));
            profile.reference_anchor_fk_session = std::make_unique<Ort::Session>(
                onnx_env_,
                profile.cfg.reference_anchor_fk_path.c_str(),
                fk_session_options);
        }

        mode_to_profile_index_[profile.mode_id] = mode_profiles_.size();
        mode_profiles_.push_back(std::move(profile));
    }

    default_mode_id_ = mode_profiles_.front().mode_id;
    active_mode_id_ = default_mode_id_;
    active_profile_index_ = profileIndexForMode(default_mode_id_, false);

    for (const auto &profile : mode_profiles_)
    {
        const auto &auto_cfg = profile.cfg.auto_switch_on_reference_end;
        if (!auto_cfg.enabled)
        {
            continue;
        }
        if (!isKnownMode(auto_cfg.target_mode_id))
        {
            throw std::runtime_error(
                "[RL_controller][" + profile.tag +
                "] auto_switch_on_reference_end.target_mode_id is not present in deploy_mode_profiles: " +
                std::to_string(auto_cfg.target_mode_id));
        }
        if (profile.resolved_reference_end_total_steps <= 0)
        {
            throw std::runtime_error(
                "[RL_controller][" + profile.tag +
                "] auto_switch_on_reference_end is enabled but total reference steps could not be resolved.");
        }
        std::cout << "[RL_controller][" << profile.tag
                  << "] reference-end auto switch armed: total_steps="
                  << profile.resolved_reference_end_total_steps
                  << ", target_mode_id=" << auto_cfg.target_mode_id
                  << std::endl;
    }

    std::cout << "[RL_controller] mode profiles loaded: " << mode_profiles_.size() << std::endl;
    for (const auto &profile : mode_profiles_)
    {
        std::cout << "  - mode_id=" << profile.mode_id
                  << ", tag=" << profile.tag
                  << ", section=" << profile.config_section
                  << ", policy=" << profile.cfg.policy_name << std::endl;
    }
}

void RL_controller::configureBaseVelocityEstimator(const SourceContractBaseVelocityEstimator &cfg)
{
    rl_master::filters::BaseVelocityKalmanFilter::Options options;
    options.initial_variance = cfg.initial_variance;
    options.process_noise = cfg.process_noise;
    options.accel_noise = cfg.accel_noise;
    options.min_dt = cfg.min_dt;
    options.max_dt = cfg.max_dt;
    base_velocity_filter_.configure(options);
}

std::array<float, 3> RL_controller::estimateBaseLinearVelocity(const rl_master::RobotStateData &state)
{
    const auto &cfg = activePolicyCfg().source_contract.base_velocity_estimator;
    if (!cfg.enabled)
    {
        return state.base_lin_vel;
    }

    configureBaseVelocityEstimator(cfg);

    const double now_sec = rl_master::monotonicTimeSec();
    float dt = 0.0f;
    if (base_velocity_filter_time_initialized_)
    {
        dt = static_cast<float>(now_sec - base_velocity_filter_last_time_sec_);
    }
    base_velocity_filter_last_time_sec_ = now_sec;
    base_velocity_filter_time_initialized_ = true;

    if (!base_velocity_filter_.initialized())
    {
        base_velocity_filter_.reset(state.base_lin_vel_valid ? state.base_lin_vel : std::array<float, 3>{0.0f, 0.0f, 0.0f});
    }

    if (cfg.use_imu_prediction && state.base_lin_acc_valid)
    {
        std::array<float, 3> linear_acc_w = state.base_lin_acc;
        if (toLowerCopy(cfg.imu_accel_frame) == "body")
        {
            linear_acc_w = rotateVectorByQuat(linear_acc_w, state.base_quat);
        }
        if (cfg.imu_accel_includes_gravity)
        {
            linear_acc_w[2] -= cfg.gravity_mps2;
        }
        base_velocity_filter_.predict(linear_acc_w, dt);
    }
    else
    {
        base_velocity_filter_.predict({0.0f, 0.0f, 0.0f}, dt);
    }

    if (cfg.use_input_velocity_measurement && state.base_lin_vel_valid)
    {
        base_velocity_filter_.updateVelocity(state.base_lin_vel, cfg.input_velocity_measurement_noise);
    }

    if (cfg.zero_velocity_update && state.base_lin_acc_valid)
    {
        float max_joint_dq = 0.0f;
        for (const float dq : state.joint_dq)
        {
            max_joint_dq = std::max(max_joint_dq, std::fabs(dq));
        }
        const float ang_vel_norm = vectorNorm3(state.base_ang_vel);
        const float acc_norm = vectorNorm3(state.base_lin_acc);
        const float expected_acc_norm = cfg.imu_accel_includes_gravity ? cfg.gravity_mps2 : 0.0f;
        const bool stationary =
            max_joint_dq <= cfg.stationary_joint_velocity_threshold &&
            ang_vel_norm <= cfg.stationary_ang_vel_threshold &&
            std::fabs(acc_norm - expected_acc_norm) <= cfg.stationary_accel_norm_tolerance;
        if (stationary)
        {
            base_velocity_filter_.updateVelocity({0.0f, 0.0f, 0.0f}, cfg.zero_velocity_measurement_noise);
        }
    }

    return base_velocity_filter_.velocity();
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
        robot->base_pos_w[i] = state.base_pos_w[i];
        robot->base_ang_vel[i] = state.base_ang_vel[i];
        robot->base_rpy[i] = state.base_rpy[i];
    }

    const std::array<float, 3> estimated_base_lin_vel = estimateBaseLinearVelocity(state);
    for (size_t i = 0; i < 3; ++i)
    {
        robot->base_lin_vel[i] = estimated_base_lin_vel[i];
    }

    for (size_t i = 0; i < 4; ++i)
    {
        robot->base_quat[i] = state.base_quat[i];
    }
}

void RL_controller::updateCommandFromIO(const rl_master::TeleopCommand &command)
{
    const auto &limits = activePolicyCfg().command_limits;
    if (limits.enabled)
    {
        cmd.vx = std::clamp(command.vx, limits.vx_min, limits.vx_max);
        cmd.vy = std::clamp(command.vy, limits.vy_min, limits.vy_max);
        cmd.dyaw = std::clamp(command.dyaw, limits.dyaw_min, limits.dyaw_max);
        return;
    }
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
    if (cfg.inference_strategy == "chunked_receding")
    {
        group->strategy = std::make_unique<ChunkedRecedingInferenceStrategy>();
    }
    else if (cfg.inference_strategy == "residual_additive")
    {
        group->strategy = std::make_unique<ResidualAdditiveInferenceStrategy>();
    }
    else
    {
        group->strategy = std::make_unique<SyncWeightedInferenceStrategy>();
    }
    group->strategy->configure(PolicyInferenceConfig{
        cfg.inference_strategy,
        static_cast<size_t>(std::max(0, cfg.action_dim)),
        cfg.action_chunk_steps,
        cfg.action_chunk_execute_steps,
        cfg.action_chunk_replan_interval});
    group->strategy->reset();

    Sim2realCfg primary_cfg = cfg;
    if (primary_cfg.action_output_indices.empty() &&
        (!primary_cfg.action_output_source_order.empty() || !primary_cfg.action_output_target_order.empty()))
    {
        primary_cfg.action_output_indices = buildIndexMapFromJointOrderAliases(
            primary_cfg,
            joint_order_,
            primary_cfg.action_output_source_order,
            primary_cfg.action_output_target_order,
            tag + "/main.action_output_order_mapping");
    }

    PolicyRunnerNode primary;
    primary.name = tag + "/main";
    primary.weight = 1.0f;
    primary.runner = std::make_unique<OnnxPolicyAdapter>(
        onnx_env_,
        primary_cfg.policy_path,
        primary_cfg,
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
        sub_cfg.action_output_indices = sub.action_output_indices;
        sub_cfg.action_output_source_order = sub.action_output_source_order;
        sub_cfg.action_output_target_order = sub.action_output_target_order;
        sub_cfg.enable_metadata_check = sub.enable_metadata_check;
        sub_cfg.metadata_check_strict = sub.metadata_check_strict;
        sub_cfg.required_metadata_keys = sub.required_metadata_keys;
        sub_cfg.expected_metadata = sub.expected_metadata;
        if (sub_cfg.action_output_indices.empty() &&
            (!sub_cfg.action_output_source_order.empty() || !sub_cfg.action_output_target_order.empty()))
        {
            sub_cfg.action_output_indices = buildIndexMapFromJointOrderAliases(
                sub_cfg,
                joint_order_,
                sub_cfg.action_output_source_order,
                sub_cfg.action_output_target_order,
                tag + "/" + sub.name + ".action_output_order_mapping");
        }

        PolicyRunnerNode node;
        node.name = tag + "/" + sub.name;
        node.weight = std::max(0.0f, sub.weight);
        node.primary_action_indices = sub.primary_action_indices;
        if (node.primary_action_indices.empty() &&
            (!sub.primary_action_source_order.empty() || !sub.primary_action_target_order.empty()))
        {
            node.primary_action_indices = buildIndexMapFromJointOrderAliases(
                cfg,
                joint_order_,
                sub.primary_action_source_order,
                sub.primary_action_target_order,
                tag + "/" + sub.name + ".primary_action_order_mapping");
        }
        node.runner = std::make_unique<OnnxPolicyAdapter>(
            onnx_env_,
            sub_cfg.policy_path,
            sub_cfg,
            node.name);
        node.runner->init();
        group->runners.push_back(std::move(node));
    }
}

RL_controller::PolicyRunOutput RL_controller::runPolicyGroup(
    PolicyRuntimeGroup *group,
    const std::vector<float> &stacked_obs,
    const std::vector<float> &current_observation,
    const ObservationFeatureContext &feature_context,
    bool advance_time_step)
{
    if (!group || group->runners.empty())
    {
        throw std::runtime_error("runPolicyGroup: empty policy group");
    }
    if (!group->strategy)
    {
        throw std::runtime_error("runPolicyGroup: policy inference strategy is null");
    }

    std::vector<PolicyAdapterNodeView> nodes;
    nodes.reserve(group->runners.size());
    for (auto &node : group->runners)
    {
        if (!node.runner)
        {
            continue;
        }
        nodes.push_back(PolicyAdapterNodeView{node.name, node.weight, node.primary_action_indices, node.runner.get()});
    }
    if (nodes.empty())
    {
        throw std::runtime_error("runPolicyGroup: no valid policy adapters");
    }

    const PolicyExecutionRequest request{
        &stacked_obs,
        &current_observation,
        &action,
        &feature_context.named_features,
        advance_time_step,
        static_cast<uint64_t>(policy_step_counter_)};
    PolicyGroupExecutionResult strategy_output =
        group->strategy->execute(
            nodes,
            request);

    PolicyRunOutput output;
    output.action = std::move(strategy_output.action);
    output.extra_outputs = std::move(strategy_output.extra_outputs);
    return output;
}

void RL_controller::prefetchCurrentPolicyReferenceOutputs(bool advance_time_step)
{
    prefetched_policy_extra_outputs_.clear();

    auto &profile = activeModeProfile();
    const auto &cfg = profile.cfg;
    if (!cfg.enable_reference_motion)
    {
        return;
    }
    if (!profile.required_reference_features.sourceAny())
    {
        return;
    }
    if (toLowerCopy(cfg.reference_motion_source) == "file")
    {
        return;
    }
    if (profile.policy_group.runners.empty() || !profile.policy_group.runners.front().runner)
    {
        return;
    }
    if (!profile.policy_group.strategy)
    {
        throw std::runtime_error("prefetchCurrentPolicyReferenceOutputs: policy inference strategy is null");
    }

    std::vector<std::string> requested_output_names;
    auto append_requested = [&](bool enabled, const std::string &name) {
        if (!enabled || name.empty())
        {
            return;
        }
        if (std::find(requested_output_names.begin(), requested_output_names.end(), name) ==
            requested_output_names.end())
        {
            requested_output_names.push_back(name);
        }
    };
    auto append_configured_extra = [&](const std::string &name) {
        if (name.empty())
        {
            return;
        }
        if (std::find(cfg.extra_output_names.begin(), cfg.extra_output_names.end(), name) ==
            cfg.extra_output_names.end())
        {
            return;
        }
        append_requested(true, name);
    };

    append_requested(
        profile.required_reference_features.reference_motion,
        cfg.source_contract.policy_extra_outputs.reference_motion_key);
    append_requested(
        profile.required_reference_features.reference_joint_pos,
        cfg.source_contract.policy_extra_outputs.reference_joint_pos_key);
    append_requested(
        profile.required_reference_features.reference_joint_vel,
        cfg.source_contract.policy_extra_outputs.reference_joint_vel_key);
    append_requested(
        profile.required_reference_features.reference_body_pos_w ||
            profile.required_reference_features.named_body_layout,
        cfg.source_contract.policy_extra_outputs.reference_body_pos_w_key);
    append_requested(
        profile.required_reference_features.reference_body_quat_w ||
            profile.required_reference_features.named_body_layout,
        cfg.source_contract.policy_extra_outputs.reference_body_quat_w_key);
    append_configured_extra(cfg.source_contract.policy_extra_outputs.reference_body_lin_vel_w_key);
    append_configured_extra(cfg.source_contract.policy_extra_outputs.reference_body_ang_vel_w_key);
    if (requested_output_names.empty())
    {
        return;
    }

    const auto &active_cfg = activePolicyCfg();
    buildStackedObservation(obs_deque, "reference prefetch");

    std::vector<float> current_observation = obs;
    if (current_observation.size() != static_cast<size_t>(active_cfg.obs_dim))
    {
        current_observation.assign(static_cast<size_t>(active_cfg.obs_dim), 0.0f);
    }

    std::vector<PolicyAdapterNodeView> nodes;
    nodes.reserve(profile.policy_group.runners.size());
    for (auto &node : profile.policy_group.runners)
    {
        if (!node.runner)
        {
            continue;
        }
        nodes.push_back(PolicyAdapterNodeView{node.name, node.weight, node.primary_action_indices, node.runner.get()});
    }
    if (nodes.empty())
    {
        return;
    }

    const PolicyExecutionRequest request{
        &stacked_obs_buffer_,
        &current_observation,
        &action,
        &latest_observation_feature_context_.named_features,
        advance_time_step,
        static_cast<uint64_t>(policy_step_counter_)};
    const auto prefetched =
        profile.policy_group.strategy->prefetchPrimaryExtraOutputs(
            nodes,
            requested_output_names,
            request);

    const std::string prefix = profile.policy_group.runners.front().name + "/";
    for (const auto &kv : prefetched)
    {
        prefetched_policy_extra_outputs_[prefix + kv.first] = kv.second;
    }
}

void RL_controller::warmStartPolicyState(double phase_t)
{
    (void)phase_t;
    const auto &cfg = activePolicyCfg();
    const int warmup_steps = std::max(0, cfg.policy_startup_warmup_steps);
    if (warmup_steps <= 0)
    {
        return;
    }

    for (int i = 0; i < warmup_steps; ++i)
    {
        const std::vector<float> zero_obs(static_cast<size_t>(cfg.obs_dim), 0.0f);
        std::vector<float> zero_stacked(static_cast<size_t>(cfg.obs_dim * std::max(1, cfg.obs_stack_N)), 0.0f);
        ObservationFeatureContext empty_feature_context;
        PolicyRunOutput warmup_output = runPolicyGroup(
            &activePolicyGroup(),
            zero_stacked,
            zero_obs,
            empty_feature_context,
            true);

        (void)warmup_output;
        prefetched_policy_extra_outputs_.clear();
    }
    action.assign(static_cast<size_t>(cfg.action_dim), 0.0f);
    latest_policy_extra_outputs_.clear();
    prefetched_policy_extra_outputs_.clear();

    std::cout << "[RL_controller] startup warmup complete: steps=" << warmup_steps
              << ", mode=" << activeModeProfile().tag << std::endl;
}

void RL_controller::initReferenceMotionProvider(
    const Sim2realCfg &cfg,
    const ReferenceFeatureRequirements &required_features,
    ReferenceMotionProvider *provider,
    const std::string &tag)
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
    if (!required_features.sourceAny())
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
            throw std::runtime_error(
                "[RL_controller][" + tag +
                "] reference_motion_source=file but reference_motion_path is empty.");
        }
        return;
    }

    const ReferenceMotionFieldMap field_map{
        cfg.source_contract.reference_file.reference_motion_key,
        cfg.source_contract.reference_file.reference_joint_pos_key,
        cfg.source_contract.reference_file.reference_joint_vel_key,
        cfg.source_contract.reference_file.reference_body_pos_w_key,
        cfg.source_contract.reference_file.reference_body_quat_w_key};

    if (!provider->load(
            cfg.reference_motion_path,
            cfg.reference_motion_dim,
            required_features,
            field_map,
            cfg.source_contract.reference_file.body_quat_order))
    {
        throw std::runtime_error(
            "[RL_controller][" + tag + "] failed to load reference motion file: " +
            cfg.reference_motion_path);
    }

    const auto &metadata = provider->metadata();
    std::cout << "[RL_controller][" << tag << "] reference motion loaded. frames="
              << provider->frameCount() << ", dim=" << provider->dim()
              << ", source_format=" << metadata.source_format
              << ", body_count=" << metadata.body_names.size() << std::endl;
}

int RL_controller::resolveReferenceEndAutoSwitchTotalSteps(const ModeProfile &profile) const
{
    const auto &auto_cfg = profile.cfg.auto_switch_on_reference_end;
    if (!auto_cfg.enabled)
    {
        return -1;
    }
    if (auto_cfg.total_steps > 0)
    {
        return auto_cfg.total_steps;
    }

    if (!profile.policy_group.runners.empty() &&
        profile.policy_group.runners.front().runner)
    {
        const auto *onnx_adapter =
            dynamic_cast<const OnnxPolicyAdapter *>(profile.policy_group.runners.front().runner.get());
        if (onnx_adapter)
        {
            const auto &metadata = onnx_adapter->customMetadata();
            for (const auto &key : auto_cfg.metadata_keys)
            {
                const auto it = metadata.find(key);
                if (it == metadata.end())
                {
                    continue;
                }
                int total_steps = -1;
                if (parsePositiveIntValue(it->second, &total_steps))
                {
                    return total_steps;
                }
            }
        }
    }

    if (profile.reference_motion.available())
    {
        const size_t frame_count = profile.reference_motion.frameCount();
        if (frame_count > 0 && frame_count <= static_cast<size_t>(std::numeric_limits<int>::max()))
        {
            return static_cast<int>(frame_count);
        }
    }

    return -1;
}

void RL_controller::scheduleReferenceEndAutoModeSwitch()
{
    const auto &profile = activeModeProfile();
    const auto &auto_cfg = profile.cfg.auto_switch_on_reference_end;
    if (!auto_cfg.enabled || auto_cfg.target_mode_id < 0)
    {
        return;
    }
    if (profile.resolved_reference_end_total_steps <= 0)
    {
        return;
    }
    if (pending_auto_mode_switch_target_mode_id_ >= 0)
    {
        return;
    }
    if (static_cast<int>(policy_step_counter_) < profile.resolved_reference_end_total_steps)
    {
        return;
    }

    pending_auto_mode_switch_target_mode_id_ = auto_cfg.target_mode_id;
    pending_auto_mode_switch_trigger_step_ = static_cast<uint64_t>(policy_step_counter_);
    pending_auto_mode_switch_reason_ = "reference_end";

    const std::string message =
        "schedule automatic hot switch after reference end";
    std::cout << "[RL_controller] " << message
              << ": from mode_id=" << active_mode_id_
              << " to mode_id=" << pending_auto_mode_switch_target_mode_id_
              << ", trigger_step=" << pending_auto_mode_switch_trigger_step_
              << ", resolved_total_steps=" << profile.resolved_reference_end_total_steps
              << std::endl;
    queueRuntimeWarningEvent(
        "reference_end_auto_mode_switch_scheduled",
        message,
        {
            {"from_mode_id", std::to_string(active_mode_id_)},
            {"to_mode_id", std::to_string(pending_auto_mode_switch_target_mode_id_)},
            {"trigger_step", std::to_string(pending_auto_mode_switch_trigger_step_)},
            {"resolved_total_steps", std::to_string(profile.resolved_reference_end_total_steps)},
        });
}

bool RL_controller::applyPendingAutoModeSwitch(double phase_t)
{
    if (pending_auto_mode_switch_target_mode_id_ < 0)
    {
        return false;
    }

    const int target_mode_id = pending_auto_mode_switch_target_mode_id_;
    const uint64_t trigger_step = pending_auto_mode_switch_trigger_step_;
    const std::string reason = pending_auto_mode_switch_reason_;
    pending_auto_mode_switch_target_mode_id_ = -1;
    pending_auto_mode_switch_trigger_step_ = 0;
    pending_auto_mode_switch_reason_.clear();

    if (!isKnownMode(target_mode_id))
    {
        const std::string message =
            "drop automatic hot switch because target mode is unknown";
        std::cerr << "[RL_controller] " << message
                  << ": target_mode_id=" << target_mode_id << std::endl;
        queueRuntimeWarningEvent(
            "reference_end_auto_mode_switch_ignored",
            message,
            {
                {"target_mode_id", std::to_string(target_mode_id)},
                {"trigger_step", std::to_string(trigger_step)},
                {"reason", reason},
            });
        return false;
    }
    if (target_mode_id == active_mode_id_)
    {
        return false;
    }

    refreshPolicyMode(target_mode_id, false);
    handlePolicySwitch();
    deploy_state_machine_.forceLocomotionMode(target_mode_id);
    phase_origin_t_ = phase_t;
    phase_origin_initialized_ = true;
    phase_reset_pending_ = false;
    running_start_reference_observation_seed_pending_ =
        activePolicyCfg().seed_running_start_observation_from_reference;

    const std::string message =
        "apply automatic hot switch after reference end";
    std::cout << "[RL_controller] " << message
              << ": target_mode_id=" << target_mode_id
              << ", trigger_step=" << trigger_step
              << std::endl;
    queueRuntimeWarningEvent(
        "reference_end_auto_mode_switch_applied",
        message,
        {
            {"target_mode_id", std::to_string(target_mode_id)},
            {"trigger_step", std::to_string(trigger_step)},
            {"reason", reason},
        });
    return true;
}

ObservationFeatureContext RL_controller::buildObservationFeatureContext(const Sim2realCfg &cfg, double phase_t)
{
    ObservationFeatureContext feature_context;
    auto &profile = activeModeProfile();
    const ReferenceFeatureRequirements &required_features = profile.required_reference_features;
    std::string source = toLowerCopy(cfg.reference_motion_source);

    const ObservationFeatureContract reference_joint_contract{
        joint_order_,
        "joint_vector",
        "joint_space"};
    const ObservationFeatureContract reference_body_quat_contract{
        {"x", "y", "z", "w"},
        cfg.observation_canonical_contract.quat_representation,
        cfg.observation_canonical_contract.body_quat_frame};

    auto external = external_observation_provider_.collect(cfg.external_observations);
    for (auto &kv : external)
    {
        feature_context.named_features.emplace(std::move(kv.first), std::move(kv.second));
    }

    const bool enable_reference = cfg.enable_reference_motion && required_features.sourceAny();
    const bool use_file_source = enable_reference && source != "policy_outputs";
    const bool use_policy_source = enable_reference && source != "file";
    const bool need_named_body_layout = required_features.named_body_layout;
    const int reference_motion_dim = cfg.reference_motion_dim > 0 ? cfg.reference_motion_dim : activeReferenceMotionProvider().dim();
    std::vector<std::string> body_names = effectiveReferenceBodyNames(cfg, &activeReferenceMotionProvider());
    std::string anchor_body = effectiveReferenceAnchorBody(cfg, &activeReferenceMotionProvider());

    if (use_file_source && activeReferenceMotionProvider().available())
    {
        const auto &provider = activeReferenceMotionProvider();
        const ReferenceMotionFrame sampled_frame =
            (cfg.reference_motion_sampling == "step")
                ? provider.sampleFrameByStep(
                      policy_step_counter_ + static_cast<size_t>(std::max(0, cfg.reference_motion_step_offset)),
                      reference_motion_dim)
                : provider.sampleFrameByPhase(phase_t, cfg.cycle_time, reference_motion_dim);

        if (!sampled_frame.reference_motion.empty())
        {
            setFeatureIfNonEmpty(
                &feature_context,
                "reference_motion",
                reference_motion_dim > 0 ? fitDim(sampled_frame.reference_motion, static_cast<size_t>(reference_motion_dim))
                                         : sampled_frame.reference_motion);
        }
        setFeatureIfNonEmptyWithContract(
            &feature_context,
            "reference_joint_pos",
            remapJointVectorToCanonical(
                sampled_frame.joint_pos,
                cfg.reference_joint_order,
                joint_order_),
            reference_joint_contract);
        setFeatureIfNonEmptyWithContract(
            &feature_context,
            "reference_joint_vel",
            remapJointVectorToCanonical(
                sampled_frame.joint_vel,
                cfg.reference_joint_order,
                joint_order_),
            reference_joint_contract);
        setFeatureIfNonEmptyWithContract(
            &feature_context,
            "reference_body_pos_w",
            sampled_frame.body_pos_w,
            ObservationFeatureContract{body_names, "vec3_array", "world"});
        setFeatureIfNonEmptyWithContract(
            &feature_context,
            "reference_body_quat_w",
            sampled_frame.body_quat_w,
            reference_body_quat_contract);
    }

    if (use_policy_source)
    {
        const std::string preferred_prefix = profile.tag + "/main/";
        if (const auto *reference_motion = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_motion_key))
        {
            setFeatureIfNonEmpty(
                &feature_context,
                "reference_motion",
                reference_motion_dim > 0 ? fitDim(*reference_motion, static_cast<size_t>(reference_motion_dim))
                                         : *reference_motion);
        }
        if (const auto *joint_pos = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_joint_pos_key))
        {
            const bool preserve_policy_reference_order =
                cfg.source_contract.policy_extra_outputs.preserve_reference_joint_order;
            const ObservationFeatureContract policy_reference_joint_contract{
                preserve_policy_reference_order ? cfg.reference_joint_order : joint_order_,
                "joint_vector",
                "joint_space"};
            setFeatureIfNonEmptyWithContract(
                &feature_context,
                "reference_joint_pos",
                preserve_policy_reference_order
                    ? *joint_pos
                    : remapJointVectorToCanonical(
                          *joint_pos,
                          cfg.reference_joint_order,
                          joint_order_),
                policy_reference_joint_contract);
        }
        if (const auto *joint_vel = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_joint_vel_key))
        {
            const bool preserve_policy_reference_order =
                cfg.source_contract.policy_extra_outputs.preserve_reference_joint_order;
            const ObservationFeatureContract policy_reference_joint_contract{
                preserve_policy_reference_order ? cfg.reference_joint_order : joint_order_,
                "joint_vector",
                "joint_space"};
            setFeatureIfNonEmptyWithContract(
                &feature_context,
                "reference_joint_vel",
                preserve_policy_reference_order
                    ? *joint_vel
                    : remapJointVectorToCanonical(
                          *joint_vel,
                          cfg.reference_joint_order,
                          joint_order_),
                policy_reference_joint_contract);
        }
        if (const auto *body_pos_w = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_body_pos_w_key))
        {
            setFeatureIfNonEmptyWithContract(
                &feature_context,
                "reference_body_pos_w",
                *body_pos_w,
                ObservationFeatureContract{body_names, "vec3_array", "world"});
        }
        if (const auto *body_quat_w = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_body_quat_w_key))
        {
            const std::vector<float> quat_xyzw = convertQuatVectorToCanonical(
                *body_quat_w,
                cfg.source_contract.policy_extra_outputs.body_quat_order,
                cfg.observation_canonical_contract.quat_order);
            setFeatureIfNonEmptyWithContract(
                &feature_context,
                "reference_body_quat_w",
                quat_xyzw,
                reference_body_quat_contract);
        }
        if (const auto *body_lin_vel_w = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_body_lin_vel_w_key))
        {
            setFeatureIfNonEmptyWithContract(
                &feature_context,
                "reference_body_lin_vel_w",
                *body_lin_vel_w,
                ObservationFeatureContract{body_names, "vec3_array", "world"});
        }
        if (const auto *body_ang_vel_w = findPolicyOutputByName(
                prefetched_policy_extra_outputs_,
                latest_policy_extra_outputs_,
                preferred_prefix,
                cfg.source_contract.policy_extra_outputs.reference_body_ang_vel_w_key))
        {
            setFeatureIfNonEmptyWithContract(
                &feature_context,
                "reference_body_ang_vel_w",
                *body_ang_vel_w,
                ObservationFeatureContract{body_names, "vec3_array", "world"});
        }
    }

    const auto *reference_body_pos_w = findNamedFeature(feature_context.named_features, "reference_body_pos_w");
    const auto *reference_body_quat_w = findNamedFeature(feature_context.named_features, "reference_body_quat_w");
    if (need_named_body_layout &&
        reference_body_pos_w && reference_body_quat_w &&
        reference_body_pos_w->size() % 3 == 0 && reference_body_quat_w->size() % 4 == 0)
    {
        const size_t body_count = std::min(reference_body_pos_w->size() / 3, reference_body_quat_w->size() / 4);
        if (body_count == body_names.size() &&
            robot->base_pos_w.size() >= 3 &&
            robot->base_quat.size() >= 4 &&
            profile.pinocchio_motion_features)
        {
            std::vector<float> motion_anchor_pos_b;
            std::vector<float> motion_anchor_ori_b;
            std::vector<float> motion_body_pos_b;
            std::vector<float> motion_body_ori_b;
            if (profile.pinocchio_motion_features->buildMotionLocalFeatures(
                    *robot,
                    body_names,
                    anchor_body,
                    cfg.motion_reference_alignment,
                    *reference_body_pos_w,
                    *reference_body_quat_w,
                    &motion_anchor_pos_b,
                    &motion_anchor_ori_b,
                    &motion_body_pos_b,
                    &motion_body_ori_b))
            {
                setFeatureIfNonEmpty(&feature_context, "motion_anchor_pos_b", motion_anchor_pos_b);
                setFeatureIfNonEmpty(&feature_context, "motion_ref_pos_b", motion_anchor_pos_b);
                setFeatureIfNonEmptyWithContract(
                    &feature_context,
                    "motion_anchor_ori_b",
                    motion_anchor_ori_b,
                    ObservationFeatureContract{{}, cfg.observation_canonical_contract.body_orientation_representation, "body_local"});
                setFeatureIfNonEmptyWithContract(
                    &feature_context,
                    "motion_ref_ori_b",
                    motion_anchor_ori_b,
                    ObservationFeatureContract{{}, cfg.observation_canonical_contract.body_orientation_representation, "body_local"});
                setFeatureIfNonEmpty(&feature_context, "motion_body_pos_b", motion_body_pos_b);
                setFeatureIfNonEmptyWithContract(
                    &feature_context,
                    "motion_body_ori_b",
                    motion_body_ori_b,
                    ObservationFeatureContract{{}, cfg.observation_canonical_contract.body_orientation_representation, "body_local"});
            }
        }
    }

    if (need_named_body_layout &&
        profile.pinocchio_motion_features)
    {
        const size_t body_count = body_names.size();
        if (body_count > 0)
        {
            std::vector<float> robot_body_pos_b;
            std::vector<float> robot_body_ori_b;
            if (profile.pinocchio_motion_features->buildRobotBodyLocalFeatures(
                    *robot,
                    body_names,
                    anchor_body,
                    &robot_body_pos_b,
                    &robot_body_ori_b))
            {
                setFeatureIfNonEmpty(&feature_context, "robot_body_pos", robot_body_pos_b);
                setFeatureIfNonEmptyWithContract(
                    &feature_context,
                    "robot_body_ori",
                    robot_body_ori_b,
                    ObservationFeatureContract{{}, cfg.observation_canonical_contract.body_orientation_representation, "body_local"});
            }
        }
    }

    const std::vector<ComputedFeatureCfg> computed_features =
        mergedComputedFeatures(profile.observation_manifest, cfg);
    if (!computed_features.empty())
    {
        const std::vector<int> &obs_indices = currentObsIndexMap();
        const size_t dof_count = obs_indices.size();
        auto applyPartShape = [&](std::vector<float> values, const ComputedFeaturePartCfg &part) {
            if (!part.indices.empty())
            {
                values = gatherByIndicesOrZeros(values, part.indices);
            }
            else if (!part.source_order.empty() || !part.target_order.empty())
            {
                values = gatherByIndicesOrZeros(
                    values,
                    buildIndexMapFromJointOrderAliases(
                        cfg,
                        joint_order_,
                        part.source_order,
                        part.target_order,
                        "computed feature part '" + part.source + "'"));
            }
            if (part.dim > 0)
            {
                values = fitDim(values, static_cast<size_t>(part.dim));
            }
            return values;
        };
        auto currentJointPosRel = [&]() {
            std::vector<float> values;
            values.reserve(dof_count);
            for (const int robot_idx : obs_indices)
            {
                const bool valid = robot_idx >= 0 &&
                                   static_cast<size_t>(robot_idx) < robot->joint_q.size() &&
                                   static_cast<size_t>(robot_idx) < robot->default_angle.size();
                values.push_back(
                    valid ? (robot->joint_q[static_cast<size_t>(robot_idx)] -
                             robot->default_angle[static_cast<size_t>(robot_idx)]) *
                                cfg.scales.dof_pos
                          : 0.0f);
            }
            return values;
        };
        auto currentJointVel = [&]() {
            std::vector<float> values;
            values.reserve(dof_count);
            for (const int robot_idx : obs_indices)
            {
                values.push_back(
                    (robot_idx >= 0 && static_cast<size_t>(robot_idx) < robot->joint_dq.size())
                        ? robot->joint_dq[static_cast<size_t>(robot_idx)] * cfg.scales.dof_vel
                        : 0.0f);
            }
            return values;
        };
        auto baseAngVel = [&]() {
            std::vector<float> values(3, 0.0f);
            for (size_t i = 0; i < std::min<size_t>(3, robot->base_ang_vel.size()); ++i)
            {
                values[i] = robot->base_ang_vel[i] * cfg.scales.ang_vel;
            }
            return values;
        };
        auto referenceJointPos = [&]() {
            const auto it = feature_context.named_features.find("reference_joint_pos");
            return it != feature_context.named_features.end()
                       ? fitDim(it->second, dof_count)
                       : gatherByIndicesOrZeros(robot->joint_q, obs_indices);
        };
        auto referenceJointVel = [&]() {
            const auto it = feature_context.named_features.find("reference_joint_vel");
            return it != feature_context.named_features.end()
                       ? fitDim(it->second, dof_count)
                       : gatherByIndicesOrZeros(robot->joint_dq, obs_indices);
        };
        auto currentAnchorQuatXyzw = [&]() {
            if (cfg.reference_anchor_current_source == "fk_onnx" &&
                profile.reference_anchor_fk_session)
            {
                try
                {
                    std::vector<float> joint_angles;
                    joint_angles.reserve(cfg.reference_anchor_fk_joint_indices.size());
                    for (const int joint_index : cfg.reference_anchor_fk_joint_indices)
                    {
                        joint_angles.push_back(
                            (joint_index >= 0 && static_cast<size_t>(joint_index) < robot->joint_q.size())
                                ? robot->joint_q[static_cast<size_t>(joint_index)]
                                : 0.0f);
                    }
                    std::vector<float> base_pos = fitDim(cfg.reference_anchor_fk_base_pos, 3);
                    std::vector<float> base_quat_wxyz(4, 0.0f);
                    if (robot->base_quat.size() >= 4)
                    {
                        base_quat_wxyz[0] = robot->base_quat[3];
                        base_quat_wxyz[1] = robot->base_quat[0];
                        base_quat_wxyz[2] = robot->base_quat[1];
                        base_quat_wxyz[3] = robot->base_quat[2];
                    }
                    else
                    {
                        base_quat_wxyz[0] = 1.0f;
                    }

                    Ort::MemoryInfo memory_info =
                        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
                    std::array<int64_t, 2> joint_shape{
                        1,
                        static_cast<int64_t>(joint_angles.size())};
                    std::array<int64_t, 2> vec3_shape{1, 3};
                    std::array<int64_t, 2> quat_shape{1, 4};
                    std::array<Ort::Value, 3> input_tensors{
                        Ort::Value::CreateTensor<float>(
                            memory_info,
                            joint_angles.data(),
                            joint_angles.size(),
                            joint_shape.data(),
                            joint_shape.size()),
                        Ort::Value::CreateTensor<float>(
                            memory_info,
                            base_pos.data(),
                            base_pos.size(),
                            vec3_shape.data(),
                            vec3_shape.size()),
                        Ort::Value::CreateTensor<float>(
                            memory_info,
                            base_quat_wxyz.data(),
                            base_quat_wxyz.size(),
                            quat_shape.data(),
                            quat_shape.size())};
                    std::array<const char *, 3> input_names{
                        cfg.reference_anchor_fk_joint_angles_input_name.c_str(),
                        cfg.reference_anchor_fk_base_pos_input_name.c_str(),
                        cfg.reference_anchor_fk_base_quat_input_name.c_str()};
                    std::array<const char *, 1> output_names{
                        cfg.reference_anchor_fk_output_quat_name.c_str()};
                    Ort::RunOptions run_options;
                    std::vector<Ort::Value> outputs =
                        profile.reference_anchor_fk_session->Run(
                            run_options,
                            input_names.data(),
                            input_tensors.data(),
                            input_tensors.size(),
                            output_names.data(),
                            output_names.size());
                    if (!outputs.empty())
                    {
                        Ort::TensorTypeAndShapeInfo shape_info =
                            outputs.front().GetTensorTypeAndShapeInfo();
                        const size_t value_count = shape_info.GetElementCount();
                        const float *data = outputs.front().GetTensorData<float>();
                        if (data && value_count >= 4)
                        {
                            if (cfg.reference_anchor_fk_output_quat_order == "xyzw")
                            {
                                return std::vector<float>{data[0], data[1], data[2], data[3]};
                            }
                            return std::vector<float>{data[1], data[2], data[3], data[0]};
                        }
                    }
                }
                catch (const std::exception &e)
                {
                    std::cerr << "[RL_controller][" << profile.tag
                              << "] reference anchor FK failed, falling back to base quat: "
                              << e.what() << std::endl;
                }
            }
            return fitDim(robot->base_quat, 4);
        };
        auto referenceAnchorOri6d = [&](size_t body_quat_index) {
            const auto anchor_ori_it = feature_context.named_features.find("motion_anchor_ori_b");
            if (anchor_ori_it != feature_context.named_features.end())
            {
                return fitDim(anchor_ori_it->second, 6);
            }
            const auto body_quat_it = feature_context.named_features.find("reference_body_quat_w");
            if (body_quat_it != feature_context.named_features.end())
            {
                return fitDim(
                    referenceAnchorOri6dFromBodyQuat(
                        body_quat_it->second,
                        currentAnchorQuatXyzw(),
                        body_quat_index),
                    6);
            }
            return std::vector<float>{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
        };
        auto policyOutput = [&](const ComputedFeaturePartCfg &part) {
            const std::string node = part.policy_node.empty() ? "main" : part.policy_node;
            const std::string output_name = part.output_name.empty() ? "action" : part.output_name;
            const std::string preferred_prefix = profile.tag + "/" + node + "/";
            if (const auto *output = findExtraOutputByName(
                    latest_policy_extra_outputs_,
                    preferred_prefix,
                    output_name))
            {
                return *output;
            }
            const size_t fallback_dim =
                part.dim > 0 ? static_cast<size_t>(part.dim) : static_cast<size_t>(std::max(0, cfg.action_dim));
            return std::vector<float>(fallback_dim, 0.0f);
        };
        auto namedFeature = [&](const ComputedFeaturePartCfg &part) {
            const auto it = feature_context.named_features.find(part.feature_name);
            if (it != feature_context.named_features.end())
            {
                return it->second;
            }
            const size_t fallback_dim = part.dim > 0 ? static_cast<size_t>(part.dim) : 0;
            return std::vector<float>(fallback_dim, 0.0f);
        };
        auto resolvePart = [&](const ComputedFeaturePartCfg &part) {
            std::vector<float> values;
            if (part.source == "joint_pos_rel")
            {
                values = currentJointPosRel();
            }
            else if (part.source == "joint_vel")
            {
                values = currentJointVel();
            }
            else if (part.source == "base_ang_vel")
            {
                values = baseAngVel();
            }
            else if (part.source == "last_action")
            {
                values = fitDim(action, dof_count);
            }
            else if (part.source == "policy_output")
            {
                values = policyOutput(part);
            }
            else if (part.source == "reference_joint_pos")
            {
                values = referenceJointPos();
            }
            else if (part.source == "reference_joint_vel")
            {
                values = referenceJointVel();
            }
            else if (part.source == "reference_anchor_ori6d")
            {
                values = referenceAnchorOri6d(static_cast<size_t>(part.body_quat_index));
            }
            else if (part.source == "feature")
            {
                values = namedFeature(part);
            }
            return applyPartShape(std::move(values), part);
        };

        for (const auto &computed_feature : computed_features)
        {
            if (computed_feature.op != "concat")
            {
                continue;
            }
            std::vector<std::vector<float>> parts;
            parts.reserve(computed_feature.parts.size());
            for (const auto &part : computed_feature.parts)
            {
                parts.push_back(resolvePart(part));
            }
            size_t total_dim = 0;
            for (const auto &part_values : parts)
            {
                total_dim += part_values.size();
            }
            std::vector<float> values;
            values.reserve(total_dim);
            for (const auto &part_values : parts)
            {
                values.insert(values.end(), part_values.begin(), part_values.end());
            }
            setFeatureIfNonEmpty(&feature_context, computed_feature.name, values);
        }
    }

    return feature_context;
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
        throw std::runtime_error(
            "[RL_controller] startup mode_id " + std::to_string(initial_mode_id) +
            " is not present in deploy_mode_profiles");
    }

    refreshPolicyMode(initial_mode_id, false);
    handlePolicySwitch();
    deploy_state_machine_initialized_ = false;
    last_deploy_state_ = rl_master::DeployLifecycleState::kInitializing;
    resetPolicyScheduler();
    phase_origin_t_ = 0.0;
    phase_origin_initialized_ = false;
    phase_reset_pending_ = true;

    start_time = std::chrono::high_resolution_clock::now();
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

    const bool auto_hot_switch_applied = applyPendingAutoModeSwitch(phase_t);

    const double now_s = rl_master::monotonicTimeSec();
    const int sanitized_mode_command = sanitizeRuntimeModeCommand(mode_command);
    const auto deploy_output = deploy_state_machine_.update(
        sanitized_mode_command,
        now_s,
        robot->joint_q,
        robot->joint_dq);
    const auto previous_state = last_deploy_state_;

    if (isKnownMode(deploy_output.locomotion_mode))
    {
        refreshPolicyMode(deploy_output.locomotion_mode, false);
        handlePolicySwitch();
    }
    else
    {
        const std::string warning_message =
            "deploy state machine produced unknown locomotion_mode, keep current active mode";
        std::cerr << "[RL_controller] " << warning_message
                  << ": locomotion_mode=" << deploy_output.locomotion_mode
                  << ", keep active_mode_id=" << active_mode_id_
                  << std::endl;
        queueRuntimeWarningEvent(
            "unknown_locomotion_mode_ignored",
            warning_message,
            {
                {"locomotion_mode", std::to_string(deploy_output.locomotion_mode)},
                {"kept_active_mode_id", std::to_string(active_mode_id_)},
            });
    }

    const bool entered_running =
        (deploy_output.state == rl_master::DeployLifecycleState::kRunning) &&
        (previous_state != rl_master::DeployLifecycleState::kRunning);
    const bool policy_context_restarted = auto_hot_switch_applied;
    if (!phase_origin_initialized_ || phase_reset_pending_ || entered_running || policy_context_restarted)
    {
        phase_origin_t_ = phase_t;
        phase_origin_initialized_ = true;
        phase_reset_pending_ = false;
        if (entered_running || policy_context_restarted)
        {
            resetPolicyScheduler();
            running_start_reference_observation_seed_pending_ =
                activePolicyCfg().seed_running_start_observation_from_reference;
        }
    }
    const double local_phase_t = std::max(0.0, phase_t - phase_origin_t_);

    if (deploy_output.state != previous_state)
    {
        std::cout << "[RL_controller] lifecycle -> "
                  << rl_master::DeployStateMachine::stateName(deploy_output.state)
                  << std::endl;
        last_deploy_state_ = deploy_output.state;
    }
    if (deploy_output.state != rl_master::DeployLifecycleState::kRunning)
    {
        running_start_reference_observation_seed_pending_ = false;
    }

    bool policy_ran_this_tick = false;
    if (deploy_output.enable_policy)
    {
        const int policy_hz = std::max(1, activePolicyCfg().RL_control_f);
        const double policy_period_s = 1.0 / static_cast<double>(policy_hz);
        if (!policy_schedule_initialized_)
        {
            next_policy_phase_t_ = local_phase_t;
        }

        constexpr double kPolicyScheduleEps = 1.0e-9;
        const bool should_run_policy =
            !policy_schedule_initialized_ ||
            (local_phase_t + kPolicyScheduleEps >= next_policy_phase_t_);

        if (should_run_policy)
        {
            if (entered_running || policy_context_restarted)
            {
                warmStartPolicyState(local_phase_t);
            }
            if (latest_policy_extra_outputs_.empty())
            {
                prefetchCurrentPolicyReferenceOutputs(
                    activePolicyCfg().advance_time_step_on_reference_prefetch);
            }
            std::vector<float> current_obs = get_robot_observation(local_phase_t);
            const bool should_prefill_observation_history =
                activePolicyCfg().prefill_observation_history_on_running_start &&
                (entered_running || observation_history_prefill_pending_);
            if (should_prefill_observation_history)
            {
                obs_deque.clear();
                for (int i = 0; i < std::max(1, activePolicyCfg().obs_stack_N); ++i)
                {
                    obs_deque.push_back(current_obs);
                }
                observation_history_prefill_pending_ = false;
            }
            else
            {
                update_obs_deque(current_obs);
            }

            const std::vector<float> policy_action = run_policy();
            robot->joint_target_q = get_joint_target_q(policy_action);
            robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
            last_policy_sample_time_sec_ = now_s;
            last_policy_sample_phase_t_ = local_phase_t;
            policy_ran_this_tick = true;
            ++policy_step_counter_;
            scheduleReferenceEndAutoModeSwitch();

            // The policy forward above has already consumed the current
            // time_step. Prefetch the next reference frame immediately so the
            // next observation does not reuse the same reference twice.
            prefetchCurrentPolicyReferenceOutputs(false);

            if (!policy_schedule_initialized_)
            {
                next_policy_phase_t_ = local_phase_t + policy_period_s;
            }
            else
            {
                while (next_policy_phase_t_ <= local_phase_t + kPolicyScheduleEps)
                {
                    next_policy_phase_t_ += policy_period_s;
                }
            }
            policy_schedule_initialized_ = true;
        }
        else
        {
            if (robot->joint_target_q.size() != joint_order_.size())
            {
                if (!action.empty())
                {
                    robot->joint_target_q = get_joint_target_q(action);
                    robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
                }
                else
                {
                    robot->joint_target_q = robot->default_angle;
                    robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
                }
            }
            else
            {
                robot->joint_target_tau = get_joint_target_torque(robot->joint_target_q);
            }
        }
        robot->open_rl = rl_master::kOpenRlPolicyEnabled;
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
        prefetched_policy_extra_outputs_.clear();
        latest_policy_extra_outputs_.clear();
    }
    else
    {
        robot->joint_target_q = robot->joint_q;
        robot->joint_target_tau.assign(robot->joint_q.size(), 0.0f);
        robot->open_rl = rl_master::kOpenRlDisabled;
        prefetched_policy_extra_outputs_.clear();
        latest_policy_extra_outputs_.clear();
    }

    rl_master::RobotCommandData out_cmd;
    out_cmd.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    out_cmd.active_joint_count = static_cast<int>(robot->joint_target_q.size());
    out_cmd.open_rl = robot->open_rl;
    out_cmd.joint_target_q = robot->joint_target_q;
    out_cmd.joint_target_dq.assign(robot->joint_target_q.size(), 0.0f);
    out_cmd.joint_target_tau = robot->joint_target_tau;

    latest_log_snapshot_.valid = true;
    latest_log_snapshot_.frame_index = log_frame_index_++;
    latest_log_snapshot_.monotonic_time_sec = rl_master::monotonicTimeSec();
    latest_log_snapshot_.phase_t = local_phase_t;
    latest_log_snapshot_.phase_t_global = phase_t;
    latest_log_snapshot_.phase_origin_t = phase_origin_t_;
    latest_log_snapshot_.requested_mode_command = mode_command;
    latest_log_snapshot_.active_mode_id = active_mode_id_;
    latest_log_snapshot_.deploy_state = static_cast<int>(deploy_output.state);
    latest_log_snapshot_.active_profile_index = static_cast<int>(active_profile_index_);
    latest_log_snapshot_.policy_step_index = static_cast<uint64_t>(policy_step_counter_);
    latest_log_snapshot_.policy_ran_this_tick = policy_ran_this_tick;
    latest_log_snapshot_.policy_sample_time_sec = last_policy_sample_time_sec_;
    latest_log_snapshot_.policy_sample_age_sec =
        last_policy_sample_time_sec_ > 0.0 ? std::max(0.0, now_s - last_policy_sample_time_sec_) : 0.0;
    latest_log_snapshot_.open_rl = robot->open_rl;
    latest_log_snapshot_.cmd_vx = cmd.vx;
    latest_log_snapshot_.cmd_vy = cmd.vy;
    latest_log_snapshot_.cmd_dyaw = cmd.dyaw;
    latest_log_snapshot_.active_tag = activeModeProfile().tag;
    latest_log_snapshot_.active_config_section = activeModeProfile().config_section;
    latest_log_snapshot_.policy_name = activePolicyCfg().policy_name;
    latest_log_snapshot_.runtime_warning_seq = pending_runtime_warning_seq_;
    latest_log_snapshot_.runtime_warning_type = pending_runtime_warning_type_;
    latest_log_snapshot_.runtime_warning_message = pending_runtime_warning_message_;
    latest_log_snapshot_.runtime_warning_tags = pending_runtime_warning_tags_;
    latest_log_snapshot_.joint_q = robot->joint_q;
    latest_log_snapshot_.joint_dq = robot->joint_dq;
    latest_log_snapshot_.joint_tau = robot->joint_tau;
    latest_log_snapshot_.joint_target_q = robot->joint_target_q;
    latest_log_snapshot_.joint_target_tau = robot->joint_target_tau;
    latest_log_snapshot_.observation = deploy_output.enable_policy ? obs : std::vector<float>{};
    latest_log_snapshot_.policy_action = deploy_output.enable_policy ? action : std::vector<float>{};
    latest_log_snapshot_.named_features =
        deploy_output.enable_policy ? latest_observation_feature_context_.named_features
                                    : std::unordered_map<std::string, std::vector<float>>{};
    latest_log_snapshot_.external_feature_names.clear();
    for (const auto &spec : activePolicyCfg().external_observations)
    {
        latest_log_snapshot_.external_feature_names.push_back(spec.name);
    }
    pending_runtime_warning_seq_ = 0;
    pending_runtime_warning_type_.clear();
    pending_runtime_warning_message_.clear();
    pending_runtime_warning_tags_.clear();
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
    const RobotState *observation_robot = robot.get();
    RobotState running_start_seeded_robot;
    if (running_start_reference_observation_seed_pending_ && robot)
    {
        running_start_seeded_robot = *robot;
        bool seeded_any = false;

        const auto reference_joint_vel_it = feature_context.named_features.find("reference_joint_vel");
        if (reference_joint_vel_it != feature_context.named_features.end())
        {
            const auto &reference_joint_vel = reference_joint_vel_it->second;
            if (reference_joint_vel.size() == running_start_seeded_robot.joint_dq.size())
            {
                running_start_seeded_robot.joint_dq = reference_joint_vel;
                seeded_any = true;
            }
        }

        const auto reference_body_lin_vel_it = feature_context.named_features.find("reference_body_lin_vel_w");
        const auto reference_body_ang_vel_it = feature_context.named_features.find("reference_body_ang_vel_w");
        if ((reference_body_lin_vel_it != feature_context.named_features.end() ||
             reference_body_ang_vel_it != feature_context.named_features.end()) &&
            running_start_seeded_robot.base_quat.size() >= 4)
        {
            const std::vector<std::string> body_names =
                effectiveReferenceBodyNames(active_cfg, &activeReferenceMotionProvider());
            const std::string anchor_body =
                effectiveReferenceAnchorBody(active_cfg, &activeReferenceMotionProvider());
            const auto anchor_it = std::find(body_names.begin(), body_names.end(), anchor_body);
            if (anchor_it != body_names.end())
            {
                const size_t anchor_index =
                    static_cast<size_t>(std::distance(body_names.begin(), anchor_it));
                const std::array<float, 4> base_quat = {
                    running_start_seeded_robot.base_quat[0],
                    running_start_seeded_robot.base_quat[1],
                    running_start_seeded_robot.base_quat[2],
                    running_start_seeded_robot.base_quat[3]};
                const std::string velocity_source =
                    toLowerCopy(active_cfg.source_contract.sim_base.velocity_source);

                if (reference_body_lin_vel_it != feature_context.named_features.end())
                {
                    const auto &reference_body_lin_vel = reference_body_lin_vel_it->second;
                    const size_t offset = anchor_index * 3;
                    if ((offset + 2) < reference_body_lin_vel.size() &&
                        running_start_seeded_robot.base_lin_vel.size() >= 3)
                    {
                        std::array<float, 3> lin_vel_world = {
                            reference_body_lin_vel[offset + 0],
                            reference_body_lin_vel[offset + 1],
                            reference_body_lin_vel[offset + 2]};
                        if (velocity_source == "body_object_velocity_local" ||
                            velocity_source == "body_object_velocity_root_local" ||
                            velocity_source == "body_cvel")
                        {
                            lin_vel_world = rotateWorldVectorToBodyFrame(lin_vel_world, base_quat);
                        }
                        for (size_t i = 0; i < 3; ++i)
                        {
                            running_start_seeded_robot.base_lin_vel[i] = lin_vel_world[i];
                        }
                        seeded_any = true;
                    }
                }

                if (reference_body_ang_vel_it != feature_context.named_features.end())
                {
                    const auto &reference_body_ang_vel = reference_body_ang_vel_it->second;
                    const size_t offset = anchor_index * 3;
                    if ((offset + 2) < reference_body_ang_vel.size() &&
                        running_start_seeded_robot.base_ang_vel.size() >= 3)
                    {
                        std::array<float, 3> ang_vel_world = {
                            reference_body_ang_vel[offset + 0],
                            reference_body_ang_vel[offset + 1],
                            reference_body_ang_vel[offset + 2]};
                        const std::array<float, 3> ang_vel_observation =
                            rotateWorldVectorToBodyFrame(ang_vel_world, base_quat);
                        for (size_t i = 0; i < 3; ++i)
                        {
                            running_start_seeded_robot.base_ang_vel[i] = ang_vel_observation[i];
                        }
                        seeded_any = true;
                    }
                }
            }
        }

        if (seeded_any)
        {
            observation_robot = &running_start_seeded_robot;
            running_start_reference_observation_seed_pending_ = false;
        }
    }
    obs = activeObservationBuilder().build(
        *observation_robot,
        cmd,
        action,
        phase_t,
        active_cfg,
        currentObsIndexMap(),
        currentReferenceIndexMap(),
        feature_context);
    return obs;
}

std::vector<float> RL_controller::run_policy(std::deque<std::vector<float>> *obs_deque_ptr, bool advance_time_step)
{
    if (!obs_deque_ptr)
    {
        obs_deque_ptr = &obs_deque;
    }

    const auto &active_cfg = activePolicyCfg();
    buildStackedObservation(*obs_deque_ptr, "policy run");

    PolicyRunOutput policy_output = runPolicyGroup(
        &activePolicyGroup(),
        stacked_obs_buffer_,
        obs,
        latest_observation_feature_context_,
        advance_time_step);
    std::vector<float> target_action = std::move(policy_output.action);
    latest_policy_extra_outputs_ = std::move(policy_output.extra_outputs);

    if (active_cfg.action_clip_stage == "raw_action")
    {
        for (auto &value : target_action)
        {
            value = std::clamp(value, -active_cfg.clip_actions, active_cfg.clip_actions);
        }
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

std::vector<float> RL_controller::get_joint_target_torque(const std::vector<float> &target_q)
{
    const auto &active_cfg = activePolicyCfg();
    const std::vector<int> &action_robot_indices = currentActionIndexMap();
    const std::vector<float> &q = robot->joint_q;
    const std::vector<float> &dq = robot->joint_dq;
    std::vector<float> target_tau(target_q.size(), 0.0f);

    const size_t policy_dim = std::min(
        action_robot_indices.size(),
        std::min(active_cfg.kps.size(), active_cfg.kds.size()));
    for (size_t policy_idx = 0; policy_idx < policy_dim; ++policy_idx)
    {
        const int robot_idx = action_robot_indices[policy_idx];
        if (robot_idx < 0)
        {
            continue;
        }
        const size_t joint_idx = static_cast<size_t>(robot_idx);
        if (joint_idx >= target_q.size() || joint_idx >= q.size() || joint_idx >= dq.size())
        {
            continue;
        }
        float tau = 
            (target_q[joint_idx] - q[joint_idx]) * active_cfg.kps[policy_idx] +
            (0.0f - dq[joint_idx]) * active_cfg.kds[policy_idx];

        if (active_cfg.sim_dc_motor.enabled && policy_idx < active_cfg.action_joint_order.size())
        {
            const std::string &joint_name = active_cfg.action_joint_order[policy_idx];
            const auto vel_limit_it = active_cfg.sim_dc_motor.velocity_limit.find(joint_name);
            if (vel_limit_it != active_cfg.sim_dc_motor.velocity_limit.end())
            {
                tau = applyDcMotorTorqueSpeedClip(
                    tau,
                    dq[joint_idx],
                    active_cfg.sim_dc_motor.saturation_effort,
                    active_cfg.sim_dc_motor.effort_limit,
                    vel_limit_it->second);
            }
        }

        float limit = 0.0f;
        if (policy_idx < active_cfg.tau_limit.size())
        {
            limit = std::max(limit, std::abs(active_cfg.tau_limit[policy_idx]));
        }
        if (limit > 0.0f)
        {
            tau = std::clamp(tau, -limit, limit);
        }
        target_tau[joint_idx] = tau;
    }

    joint_target_torque = target_tau;
    return target_tau;
}

std::vector<float> RL_controller::get_joint_target_q(const std::vector<float> &policy_action)
{
    const Sim2realCfg &active_cfg = activePolicyCfg();
    const std::vector<int> &action_robot_indices = currentActionIndexMap();
    const std::vector<float> &q = robot->joint_q;
    const std::vector<float> &dq = robot->joint_dq;
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
        const float joint_action_scale =
            policy_idx < active_cfg.action_scales.size() ? active_cfg.action_scales[policy_idx] : active_cfg.action_scale;
        float target_delta = policy_action[policy_idx] * joint_action_scale;
        if (active_cfg.action_clip_stage == "target_delta" && active_cfg.target_delta_clip > 0.0f)
        {
            target_delta = std::clamp(
                target_delta,
                -active_cfg.target_delta_clip,
                active_cfg.target_delta_clip);
        }
        float target_position =
            robot->default_angle[static_cast<size_t>(robot_idx)] +
            target_delta;
        if (active_cfg.action_clip_stage == "target_q" && active_cfg.target_q_clip > 0.0f)
        {
            target_position = std::clamp(
                target_position,
                -active_cfg.target_q_clip,
                active_cfg.target_q_clip);
        }
        if (active_cfg.target_q_velocity_envelope.enabled &&
            policy_idx < active_cfg.action_joint_order.size() &&
            policy_idx < active_cfg.kps.size() &&
            policy_idx < active_cfg.kds.size() &&
            static_cast<size_t>(robot_idx) < q.size() &&
            static_cast<size_t>(robot_idx) < dq.size())
        {
            const std::string &joint_name = active_cfg.action_joint_order[policy_idx];
            const auto x1_it = active_cfg.target_q_velocity_envelope.x1.find(joint_name);
            const auto x2_it = active_cfg.target_q_velocity_envelope.x2.find(joint_name);
            const auto y1_it = active_cfg.target_q_velocity_envelope.y1.find(joint_name);
            const auto y2_it = active_cfg.target_q_velocity_envelope.y2.find(joint_name);
            if (x1_it != active_cfg.target_q_velocity_envelope.x1.end() &&
                x2_it != active_cfg.target_q_velocity_envelope.x2.end() &&
                y1_it != active_cfg.target_q_velocity_envelope.y1.end() &&
                y2_it != active_cfg.target_q_velocity_envelope.y2.end())
            {
                target_position = clampTargetQToVelocityEnvelope(
                    target_position,
                    q[static_cast<size_t>(robot_idx)],
                    dq[static_cast<size_t>(robot_idx)],
                    active_cfg.kps[policy_idx],
                    active_cfg.kds[policy_idx],
                    x1_it->second,
                    x2_it->second,
                    y1_it->second,
                    y2_it->second,
                    active_cfg.target_q_velocity_envelope.zero_velocity_epsilon);
            }
        }
        if (active_cfg.clamp_target_q_to_joint_limits && policy_idx < active_cfg.action_joint_order.size())
        {
            const std::string &joint_name = active_cfg.action_joint_order[policy_idx];
            const auto limit_it = active_cfg.robotCfg.joint_limit_range.find(joint_name);
            if (limit_it != active_cfg.robotCfg.joint_limit_range.end() && limit_it->second.size() >= 2)
            {
                const float lower = limit_it->second[0] + active_cfg.target_q_joint_limit_margin;
                const float upper = limit_it->second[1] - active_cfg.target_q_joint_limit_margin;
                if (lower <= upper)
                {
                    target_position = std::clamp(target_position, lower, upper);
                }
            }
        }
        target_q[static_cast<size_t>(robot_idx)] = target_position;
    }

    joint_target_q = target_q;
    return target_q;
}
