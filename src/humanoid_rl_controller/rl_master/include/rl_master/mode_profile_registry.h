#ifndef RL_MASTER_MODE_PROFILE_REGISTRY_H
#define RL_MASTER_MODE_PROFILE_REGISTRY_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rl_master/deploy_state_machine.h"
#include "rl_master/rl_cfg.h"

namespace rl_master
{

class ModeProfileRegistry
{
public:
    static std::shared_ptr<const ModeProfileRegistry> loadFromYaml(
        const std::string &yaml_file,
        const std::string &fallback_section = "engineai_walk");

    const std::string &yamlPath() const;
    const std::vector<DeployModeProfileSpec> &specs() const;
    bool empty() const;

    bool hasMode(int mode_id) const;
    int defaultModeId() const;
    const std::string &defaultConfigSection() const;
    const std::string &configSectionForMode(int mode_id, bool allow_fallback = true) const;
    const DeployModeProfileSpec &specForMode(int mode_id, bool allow_fallback = true) const;
    const Sim2realCfg &cfgForMode(int mode_id, bool allow_fallback = true) const;
    const Sim2realCfg &cfgForSection(const std::string &section) const;

private:
    struct Entry
    {
        DeployModeProfileSpec spec;
        Sim2realCfg cfg;
    };

    ModeProfileRegistry() = default;
    void loadInternal(const std::string &yaml_file, const std::string &fallback_section);

    const Entry &entryForMode(int mode_id, bool allow_fallback) const;
    const Entry &entryForSection(const std::string &section) const;

    std::string yaml_file_;
    std::vector<DeployModeProfileSpec> specs_;
    std::vector<Entry> entries_;
    std::unordered_map<int, size_t> mode_to_entry_index_;
    std::unordered_map<std::string, size_t> section_to_entry_index_;
    int default_mode_id_ = rl_master::kModeCodeMin;
    std::string default_config_section_ = "engineai_walk";
};

} // namespace rl_master

#endif // RL_MASTER_MODE_PROFILE_REGISTRY_H
