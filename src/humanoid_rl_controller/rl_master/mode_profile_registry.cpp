#include "rl_master/mode_profile_registry.h"

#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace rl_master
{

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

void ModeProfileRegistry::loadInternal(const std::string &yaml_file, const std::string &fallback_section)
{
    yaml_file_ = yaml_file;
    specs_.clear();
    entries_.clear();
    mode_to_entry_index_.clear();
    section_to_entry_index_.clear();
    joint_order_.clear();
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
    explicit_global_joint_names.reserve(explicit_global_joint_order.size());
    for (const auto &joint_name : explicit_global_joint_order)
    {
        explicit_global_joint_names.insert(joint_name);
    }

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
        const size_t index = entries_.size();
        entries_.push_back(std::move(entry));
        mode_to_entry_index_[spec.mode_id] = index;
        section_to_entry_index_.emplace(spec.config_section, index);

        for (const auto &joint_name : cfg.robot_joint_order)
        {
            if (explicit_global_joint_names.find(joint_name) == explicit_global_joint_names.end())
            {
                throw std::runtime_error(
                    "config section '" + spec.config_section +
                    "' contains joint not present in robot_global_joint_order: " + joint_name);
            }
        }

        if (!cfg.robotCfg.zero_joint_angles.empty())
        {
            std::unordered_set<std::string> zero_joint_names;
            zero_joint_names.reserve(cfg.robotCfg.zero_joint_angles.size());
            for (const auto &entry : cfg.robotCfg.zero_joint_angles)
            {
                zero_joint_names.insert(entry.first);
            }

            std::vector<std::string> missing_zero_joints;
            for (const auto &joint_name : explicit_global_joint_order)
            {
                if (zero_joint_names.find(joint_name) == zero_joint_names.end())
                {
                    missing_zero_joints.push_back(joint_name);
                }
            }
            if (!missing_zero_joints.empty())
            {
                std::ostringstream oss;
                for (size_t i = 0; i < missing_zero_joints.size(); ++i)
                {
                    if (i > 0)
                    {
                        oss << ", ";
                    }
                    oss << missing_zero_joints[i];
                }
                throw std::runtime_error(
                    "config section '" + spec.config_section +
                    "' robot.zero_joint_angles must cover robot_global_joint_order; missing: " +
                    oss.str());
            }
        }
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
