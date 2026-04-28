#include "rl_master/mode_profile_registry.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace rl_master
{

namespace
{

std::unordered_map<std::string, size_t> buildJointIndexMap(
    const std::vector<std::string> &joint_order)
{
    std::unordered_map<std::string, size_t> out;
    out.reserve(joint_order.size());
    for (size_t i = 0; i < joint_order.size(); ++i)
    {
        out[joint_order[i]] = i;
    }
    return out;
}

std::vector<float> buildDefaultAngleVector(
    const Sim2realCfg::RobotCfg &robot_cfg,
    const std::vector<std::string> &joint_order,
    const std::string &section_name)
{
    if (robot_cfg.default_joint_angles.empty())
    {
        throw std::runtime_error(
            "config section '" + section_name +
            "' robot.default_joint_angles is required and must cover robot_global_joint_order");
    }

    std::vector<float> out(joint_order.size(), 0.0f);
    std::unordered_map<std::string, float> default_angle_map;
    default_angle_map.reserve(robot_cfg.default_joint_angles.size());
    for (const auto &entry : robot_cfg.default_joint_angles)
    {
        default_angle_map[entry.first] = entry.second;
    }

    for (size_t i = 0; i < joint_order.size(); ++i)
    {
        const auto it = default_angle_map.find(joint_order[i]);
        if (it == default_angle_map.end())
        {
            throw std::runtime_error(
                "config section '" + section_name +
                "' robot.default_joint_angles must cover robot_global_joint_order; missing: " +
                joint_order[i]);
        }
        out[i] = it->second;
    }
    return out;
}

std::vector<float> buildZeroPoseVector(
    const Sim2realCfg::RobotCfg &robot_cfg,
    const std::vector<std::string> &joint_order,
    const std::string &section_name)
{
    if (robot_cfg.zero_joint_angles.empty())
    {
        throw std::runtime_error(
            "config section '" + section_name +
            "' robot.zero_joint_angles is required and must cover robot_global_joint_order");
    }

    std::vector<float> out(joint_order.size(), 0.0f);
    std::unordered_map<std::string, float> zero_pose_map;
    zero_pose_map.reserve(robot_cfg.zero_joint_angles.size());
    for (const auto &entry : robot_cfg.zero_joint_angles)
    {
        zero_pose_map[entry.first] = entry.second;
    }

    for (size_t i = 0; i < joint_order.size(); ++i)
    {
        const auto it = zero_pose_map.find(joint_order[i]);
        if (it == zero_pose_map.end())
        {
            throw std::runtime_error(
                "config section '" + section_name +
                "' robot.zero_joint_angles must cover robot_global_joint_order; missing: " +
                joint_order[i]);
        }
        out[i] = it->second;
    }
    return out;
}

std::vector<int> buildActionIndices(
    const Sim2realCfg &cfg,
    const std::unordered_map<std::string, size_t> &joint_index_map,
    size_t joint_count,
    const std::string &section_name)
{
    if (cfg.action_dim <= 0)
    {
        throw std::runtime_error(section_name + ": action_dim must be > 0");
    }
    if (cfg.action_dim > static_cast<int>(joint_count))
    {
        throw std::runtime_error(section_name + ": action_dim exceeds robot joint count");
    }
    if (cfg.action_joint_order.size() != static_cast<size_t>(cfg.action_dim))
    {
        throw std::runtime_error(section_name + ": action_joint_order length must equal action_dim");
    }

    std::vector<int> out(static_cast<size_t>(cfg.action_dim), -1);
    std::unordered_set<std::string> seen;
    seen.reserve(cfg.action_joint_order.size());
    for (size_t i = 0; i < cfg.action_joint_order.size(); ++i)
    {
        const std::string &name = cfg.action_joint_order[i];
        if (!seen.insert(name).second)
        {
            throw std::runtime_error(section_name + ": duplicate joint name in action_joint_order: " + name);
        }
        const auto it = joint_index_map.find(name);
        if (it == joint_index_map.end())
        {
            throw std::runtime_error(section_name + ": unknown joint in action_joint_order: " + name);
        }
        out[i] = static_cast<int>(it->second);
    }
    return out;
}

std::vector<int> buildObsIndices(
    const Sim2realCfg &cfg,
    const std::unordered_map<std::string, size_t> &joint_index_map,
    size_t joint_count,
    const std::string &section_name)
{
    if (cfg.motor_N <= 0)
    {
        throw std::runtime_error(section_name + ": motor_N must be > 0");
    }
    if (cfg.motor_N > static_cast<int>(joint_count))
    {
        throw std::runtime_error(section_name + ": motor_N exceeds robot joint count");
    }
    if (cfg.obs_joint_order.size() != static_cast<size_t>(cfg.motor_N))
    {
        throw std::runtime_error(section_name + ": obs_joint_order length must equal motor_N");
    }

    std::vector<int> out(static_cast<size_t>(cfg.motor_N), -1);
    for (size_t i = 0; i < cfg.obs_joint_order.size(); ++i)
    {
        const std::string &name = cfg.obs_joint_order[i];
        const auto it = joint_index_map.find(name);
        if (it == joint_index_map.end())
        {
            throw std::runtime_error(section_name + ": unknown joint in obs_joint_order: " + name);
        }
        out[i] = static_cast<int>(it->second);
    }
    return out;
}

std::vector<int> buildReferenceIndices(
    const Sim2realCfg &cfg,
    const std::unordered_map<std::string, size_t> &joint_index_map,
    const std::string &section_name)
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

    std::vector<int> out(reference_joint_names.size(), -1);
    for (size_t i = 0; i < reference_joint_names.size(); ++i)
    {
        const std::string &name = reference_joint_names[i];
        const auto it = joint_index_map.find(name);
        if (it == joint_index_map.end())
        {
            throw std::runtime_error(section_name + ": unknown joint in reference_joint_order: " + name);
        }
        out[i] = static_cast<int>(it->second);
    }
    return out;
}

void validateJointGroupSubset(
    const std::vector<std::string> &group,
    const std::string &group_name,
    const std::unordered_set<std::string> &global_joint_names)
{
    for (const auto &joint_name : group)
    {
        if (global_joint_names.find(joint_name) == global_joint_names.end())
        {
            throw std::runtime_error(
                "joint_groups." + group_name +
                " contains joint not present in robot_global_joint_order: " + joint_name);
        }
    }
}

void validateJointGroupOverlap(const JointGroupsConfig &joint_groups)
{
    std::unordered_map<std::string, std::string> owner;
    auto registerGroup = [&owner](const std::vector<std::string> &group, const std::string &group_name) {
        for (const auto &joint_name : group)
        {
            const auto it = owner.find(joint_name);
            if (it != owner.end())
            {
                throw std::runtime_error(
                    "joint '" + joint_name + "' is declared in both joint_groups." +
                    it->second + " and joint_groups." + group_name);
            }
            owner.emplace(joint_name, group_name);
        }
    };

    registerGroup(joint_groups.leg, "leg");
    registerGroup(joint_groups.arm, "arm");
    registerGroup(joint_groups.waist, "waist");
}

std::string joinNames(const std::vector<std::string> &names)
{
    std::ostringstream oss;
    for (size_t i = 0; i < names.size(); ++i)
    {
        if (i > 0)
        {
            oss << ", ";
        }
        oss << names[i];
    }
    return oss.str();
}

template <typename NameContainer>
void validateExactJointCoverage(
    const std::vector<std::string> &expected_joint_order,
    const NameContainer &provided_names,
    const std::string &field_name,
    const std::string &section_name)
{
    std::unordered_set<std::string> expected_joint_names(
        expected_joint_order.begin(),
        expected_joint_order.end());
    std::unordered_set<std::string> provided_joint_names(
        provided_names.begin(),
        provided_names.end());

    std::vector<std::string> unexpected_joint_names;
    for (const auto &joint_name : provided_joint_names)
    {
        if (expected_joint_names.find(joint_name) == expected_joint_names.end())
        {
            unexpected_joint_names.push_back(joint_name);
        }
    }
    std::sort(unexpected_joint_names.begin(), unexpected_joint_names.end());
    if (!unexpected_joint_names.empty())
    {
        throw std::runtime_error(
            "config section '" + section_name + "' " + field_name +
            " contains joints outside its required set: " + joinNames(unexpected_joint_names));
    }

    std::vector<std::string> missing_joint_names;
    for (const auto &joint_name : expected_joint_order)
    {
        if (provided_joint_names.find(joint_name) == provided_joint_names.end())
        {
            missing_joint_names.push_back(joint_name);
        }
    }
    if (!missing_joint_names.empty())
    {
        throw std::runtime_error(
            "config section '" + section_name + "' " + field_name +
            " must exactly cover its required joint set; missing: " + joinNames(missing_joint_names));
    }
}

ModeProfileJointLayout buildJointLayout(
    const Sim2realCfg &cfg,
    const std::vector<std::string> &joint_order,
    const std::string &section_name)
{
    ModeProfileJointLayout layout;
    layout.joint_name_to_global_index = buildJointIndexMap(joint_order);
    layout.default_angle = buildDefaultAngleVector(cfg.robotCfg, joint_order, section_name);
    layout.zero_pose = buildZeroPoseVector(cfg.robotCfg, joint_order, section_name);
    layout.action_global_indices = buildActionIndices(cfg, layout.joint_name_to_global_index, joint_order.size(), section_name);
    layout.obs_global_indices = buildObsIndices(cfg, layout.joint_name_to_global_index, joint_order.size(), section_name);
    layout.reference_global_indices = buildReferenceIndices(cfg, layout.joint_name_to_global_index, section_name);
    return layout;
}

} // namespace

std::shared_ptr<const ModeProfileRegistry> ModeProfileRegistry::loadFromYaml(
    const std::string &yaml_file,
    const std::string &fallback_section)
{
    auto registry = std::shared_ptr<ModeProfileRegistry>(new ModeProfileRegistry());
    registry->loadInternal(yaml_file, fallback_section);
    return registry;
}

const std::string &ModeProfileRegistry::yamlPath() const
{
    return yaml_file_;
}

const std::vector<DeployModeProfileSpec> &ModeProfileRegistry::specs() const
{
    return specs_;
}

bool ModeProfileRegistry::empty() const
{
    return entries_.empty();
}

bool ModeProfileRegistry::hasMode(int mode_id) const
{
    return mode_to_entry_index_.find(mode_id) != mode_to_entry_index_.end();
}

int ModeProfileRegistry::defaultModeId() const
{
    return default_mode_id_;
}

const std::string &ModeProfileRegistry::defaultConfigSection() const
{
    return default_config_section_;
}

const std::vector<std::string> &ModeProfileRegistry::jointOrder() const
{
    return joint_order_;
}

const JointGroupsConfig &ModeProfileRegistry::jointGroups() const
{
    return joint_groups_;
}

const std::string &ModeProfileRegistry::configSectionForMode(int mode_id, bool allow_fallback) const
{
    return entryForMode(mode_id, allow_fallback).spec.config_section;
}

const DeployModeProfileSpec &ModeProfileRegistry::specForMode(int mode_id, bool allow_fallback) const
{
    return entryForMode(mode_id, allow_fallback).spec;
}

const Sim2realCfg &ModeProfileRegistry::cfgForMode(int mode_id, bool allow_fallback) const
{
    return entryForMode(mode_id, allow_fallback).cfg;
}

const Sim2realCfg &ModeProfileRegistry::cfgForSection(const std::string &section) const
{
    return entryForSection(section).cfg;
}

const ModeProfileJointLayout &ModeProfileRegistry::layoutForMode(int mode_id, bool allow_fallback) const
{
    return entryForMode(mode_id, allow_fallback).layout;
}

const ModeProfileJointLayout &ModeProfileRegistry::layoutForSection(const std::string &section) const
{
    return entryForSection(section).layout;
}

void ModeProfileRegistry::loadInternal(const std::string &yaml_file, const std::string &fallback_section)
{
    yaml_file_ = yaml_file;
    specs_.clear();
    entries_.clear();
    mode_to_entry_index_.clear();
    section_to_entry_index_.clear();
    joint_order_.clear();
    joint_groups_ = JointGroupsConfig{};
    default_mode_id_ = rl_master::kModeCodeMin;
    default_config_section_ = fallback_section.empty() ? "engineai_walk" : fallback_section;

    specs_ = loadDeployModeProfilesFromYAML(yaml_file);
    if (specs_.empty())
    {
        DeployModeProfileSpec fallback_spec;
        fallback_spec.mode_id = rl_master::kModeCodeMin;
        fallback_spec.config_section = default_config_section_;
        fallback_spec.tag = default_config_section_;
        specs_.push_back(std::move(fallback_spec));
    }

    default_mode_id_ = specs_.front().mode_id;
    default_config_section_ = specs_.front().config_section.empty() ? default_config_section_ : specs_.front().config_section;

    const std::vector<std::string> explicit_global_joint_order = loadRobotGlobalJointOrderFromYAML(yaml_file);
    std::unordered_set<std::string> explicit_global_joint_names;
    joint_order_ = explicit_global_joint_order;
    joint_groups_ = loadJointGroupsFromYAML(yaml_file);
    explicit_global_joint_names.reserve(explicit_global_joint_order.size());
    for (const auto &joint_name : explicit_global_joint_order)
    {
        explicit_global_joint_names.insert(joint_name);
    }
    validateJointGroupSubset(joint_groups_.leg, "leg", explicit_global_joint_names);
    validateJointGroupSubset(joint_groups_.arm, "arm", explicit_global_joint_names);
    validateJointGroupSubset(joint_groups_.waist, "waist", explicit_global_joint_names);
    validateJointGroupOverlap(joint_groups_);
    auto validateJointNames = [&](const auto &names, const std::string &field_name, const std::string &section_name) {
        for (const auto &joint_name : names)
        {
            if (explicit_global_joint_names.find(joint_name) == explicit_global_joint_names.end())
            {
                throw std::runtime_error(
                    "config section '" + section_name +
                    "' contains joint not present in robot_global_joint_order for " +
                    field_name + ": " + joint_name);
            }
        }
    };
    auto validatePairJointNames = [&](const auto &named_values, const std::string &field_name, const std::string &section_name) {
        for (const auto &entry : named_values)
        {
            if (explicit_global_joint_names.find(entry.first) == explicit_global_joint_names.end())
            {
                throw std::runtime_error(
                    "config section '" + section_name +
                    "' contains joint not present in robot_global_joint_order for " +
                    field_name + ": " + entry.first);
            }
        }
    };

    std::unordered_map<std::string, Sim2realCfg> cfg_cache;
    entries_.reserve(specs_.size());
    for (const auto &spec : specs_)
    {
        if (spec.config_section.empty())
        {
            throw std::runtime_error("mode profile config_section is empty for mode_id=" + std::to_string(spec.mode_id));
        }
        if (mode_to_entry_index_.find(spec.mode_id) != mode_to_entry_index_.end())
        {
            throw std::runtime_error("duplicate mode_id in mode profile registry: " + std::to_string(spec.mode_id));
        }

        Sim2realCfg cfg;
        const auto cache_it = cfg_cache.find(spec.config_section);
        if (cache_it != cfg_cache.end())
        {
            cfg = cache_it->second;
        }
        else
        {
            if (!cfg.loadFromYAML(yaml_file, spec.config_section))
            {
                throw std::runtime_error("failed to load config section: " + spec.config_section);
            }
            cfg_cache[spec.config_section] = cfg;
        }

        Entry entry;
        entry.spec = spec;
        entry.cfg = cfg;
        entry.layout = buildJointLayout(cfg, joint_order_, spec.config_section);
        const size_t index = entries_.size();
        entries_.push_back(std::move(entry));
        mode_to_entry_index_[spec.mode_id] = index;
        section_to_entry_index_.emplace(spec.config_section, index);

        validateJointNames(cfg.action_joint_order, "action_joint_order", spec.config_section);
        validateJointNames(cfg.obs_joint_order, "obs_joint_order", spec.config_section);
        validateJointNames(cfg.reference_joint_order, "reference_joint_order", spec.config_section);
        validatePairJointNames(cfg.robotCfg.default_joint_angles, "robot.default_joint_angles", spec.config_section);
        validatePairJointNames(cfg.robotCfg.zero_joint_angles, "robot.zero_joint_angles", spec.config_section);
        validatePairJointNames(cfg.robotCfg.joint_limit_range, "robot.joint_limit_range", spec.config_section);
        validatePairJointNames(cfg.robotCfg.motor_torque_limit, "robot.motor_torque_limit", spec.config_section);
        validatePairJointNames(cfg.named_kps, "kps", spec.config_section);
        validatePairJointNames(cfg.named_kds, "kds", spec.config_section);
        validatePairJointNames(cfg.named_tau_limit, "tau_limit", spec.config_section);
        validatePairJointNames(cfg.installed_joint_run_modes, "installed_joint_run_modes", spec.config_section);
        std::vector<std::string> default_joint_names;
        default_joint_names.reserve(cfg.robotCfg.default_joint_angles.size());
        for (const auto &entry : cfg.robotCfg.default_joint_angles)
        {
            default_joint_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            explicit_global_joint_order,
            default_joint_names,
            "robot.default_joint_angles",
            spec.config_section);

        std::vector<std::string> zero_joint_names;
        zero_joint_names.reserve(cfg.robotCfg.zero_joint_angles.size());
        for (const auto &entry : cfg.robotCfg.zero_joint_angles)
        {
            zero_joint_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            explicit_global_joint_order,
            zero_joint_names,
            "robot.zero_joint_angles",
            spec.config_section);

        std::vector<std::string> joint_limit_names;
        joint_limit_names.reserve(cfg.robotCfg.joint_limit_range.size());
        for (const auto &entry : cfg.robotCfg.joint_limit_range)
        {
            joint_limit_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            explicit_global_joint_order,
            joint_limit_names,
            "robot.joint_limit_range",
            spec.config_section);

        std::vector<std::string> motor_torque_limit_names;
        motor_torque_limit_names.reserve(cfg.robotCfg.motor_torque_limit.size());
        for (const auto &entry : cfg.robotCfg.motor_torque_limit)
        {
            motor_torque_limit_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            explicit_global_joint_order,
            motor_torque_limit_names,
            "robot.motor_torque_limit",
            spec.config_section);

        std::vector<std::string> installed_run_mode_names;
        installed_run_mode_names.reserve(cfg.installed_joint_run_modes.size());
        for (const auto &entry : cfg.installed_joint_run_modes)
        {
            installed_run_mode_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            explicit_global_joint_order,
            installed_run_mode_names,
            "installed_joint_run_modes",
            spec.config_section);

        std::vector<std::string> kp_names;
        kp_names.reserve(cfg.named_kps.size());
        for (const auto &entry : cfg.named_kps)
        {
            kp_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            cfg.action_joint_order,
            kp_names,
            "kps",
            spec.config_section);

        std::vector<std::string> kd_names;
        kd_names.reserve(cfg.named_kds.size());
        for (const auto &entry : cfg.named_kds)
        {
            kd_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            cfg.action_joint_order,
            kd_names,
            "kds",
            spec.config_section);

        std::vector<std::string> tau_limit_names;
        tau_limit_names.reserve(cfg.named_tau_limit.size());
        for (const auto &entry : cfg.named_tau_limit)
        {
            tau_limit_names.push_back(entry.first);
        }
        validateExactJointCoverage(
            cfg.action_joint_order,
            tau_limit_names,
            "tau_limit",
            spec.config_section);
    }
}

const ModeProfileRegistry::Entry &ModeProfileRegistry::entryForMode(int mode_id, bool allow_fallback) const
{
    const auto it = mode_to_entry_index_.find(mode_id);
    if (it != mode_to_entry_index_.end())
    {
        return entries_.at(it->second);
    }
    if (allow_fallback)
    {
        const auto fallback_it = mode_to_entry_index_.find(default_mode_id_);
        if (fallback_it != mode_to_entry_index_.end())
        {
            return entries_.at(fallback_it->second);
        }
        if (!entries_.empty())
        {
            return entries_.front();
        }
    }
    throw std::runtime_error("unknown mode_id in mode profile registry: " + std::to_string(mode_id));
}

const ModeProfileRegistry::Entry &ModeProfileRegistry::entryForSection(const std::string &section) const
{
    const auto it = section_to_entry_index_.find(section);
    if (it != section_to_entry_index_.end())
    {
        return entries_.at(it->second);
    }
    throw std::runtime_error("unknown config section in mode profile registry: " + section);
}

} // namespace rl_master
