#include "rl_master/observation_builder.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "rl_master/math_tool.h"

namespace
{
int getCountOrDefault(const YAML::Node &node, int default_count)
{
    if (!node["count"])
    {
        return default_count;
    }
    return node["count"].as<int>();
}

std::vector<std::string> getComponents(const YAML::Node &node)
{
    if (!node["components"])
    {
        return {};
    }
    return node["components"].as<std::vector<std::string>>();
}

std::string getNameCompat(const YAML::Node &node)
{
    if (node["name"])
    {
        return node["name"].as<std::string>();
    }
    if (node["type"])
    {
        return node["type"].as<std::string>();
    }
    throw std::runtime_error("Observation term missing both 'name' and 'type'");
}

std::string getSourceOrEmpty(const YAML::Node &node)
{
    if (!node["source"])
    {
        return "";
    }
    return node["source"].as<std::string>();
}

bool getEnabled(const YAML::Node &node)
{
    if (!node["enabled"])
    {
        return true;
    }
    return node["enabled"].as<bool>();
}

void pushPadded(std::vector<float> *out, float value, int i, int max_count)
{
    if (!out)
    {
        return;
    }
    out->push_back(i < max_count ? value : 0.0f);
}

const std::vector<float> *findFeature(
    const ObservationFeatureContext &feature_context,
    const std::string &source_name)
{
    const auto it = feature_context.named_features.find(source_name);
    if (it == feature_context.named_features.end())
    {
        return nullptr;
    }
    return &it->second;
}

void appendFeatureVector(
    const ObservationTermConfig &term,
    const ObservationFeatureContext &feature_context,
    const std::string &default_source_name,
    std::vector<float> *out)
{
    const std::string source_name = term.source.empty() ? default_source_name : term.source;
    const std::vector<float> *source = findFeature(feature_context, source_name);
    const int max_count = source ? std::min(term.count, static_cast<int>(source->size())) : 0;
    for (int i = 0; i < term.count; ++i)
    {
        pushPadded(out, (source && i < max_count) ? (*source)[static_cast<size_t>(i)] : 0.0f, i, max_count);
    }
}
} // namespace

ObservationManifest ObservationManifest::loadFromYAML(const std::string &yaml_file)
{
    ObservationManifest manifest;
    const YAML::Node root = YAML::LoadFile(yaml_file);
    const YAML::Node observation_manifest = root["observation_manifest"];
    if (!observation_manifest || !observation_manifest["terms"])
    {
        throw std::runtime_error("observation_manifest.terms is missing in " + yaml_file);
    }

    for (const auto &term_node : observation_manifest["terms"])
    {
        ObservationTermConfig term;
        term.name = getNameCompat(term_node);
        term.enabled = getEnabled(term_node);
        term.components = getComponents(term_node);
        term.source = getSourceOrEmpty(term_node);

        if (term.name == "phase")
        {
            term.count = getCountOrDefault(term_node, 2);
        }
        else if (term.name == "command")
        {
            const int default_count = term.components.empty() ? 3 : static_cast<int>(term.components.size());
            term.count = getCountOrDefault(term_node, default_count);
            if (!term.components.empty() && term.count != static_cast<int>(term.components.size()))
            {
                throw std::runtime_error("command term count must equal components length");
            }
            if (term.components.empty() && term.count > 3)
            {
                throw std::runtime_error("command term count cannot exceed 3 when components is empty");
            }
        }
        else if (term.name == "joint_pos" || term.name == "joint_vel" || term.name == "last_action")
        {
            term.count = getCountOrDefault(term_node, 12);
        }
        else if (term.name == "reference_joint_pos" || term.name == "reference_joint_vel")
        {
            term.count = getCountOrDefault(term_node, 12);
        }
        else if (term.name == "base_ang_vel" || term.name == "base_rpy")
        {
            term.count = getCountOrDefault(term_node, 3);
        }
        else if (term.name == "motion_anchor_pos_b" || term.name == "motion_ref_pos_b")
        {
            term.count = getCountOrDefault(term_node, 3);
        }
        else if (term.name == "motion_anchor_ori_b" || term.name == "motion_ref_ori_b")
        {
            term.count = getCountOrDefault(term_node, 6);
        }
        else if (term.name == "motion_body_pos_b" || term.name == "motion_body_ori_b" ||
                 term.name == "robot_body_pos" || term.name == "robot_body_ori")
        {
            term.count = getCountOrDefault(term_node, 0);
        }
        else if (term.name == "reference_motion" || term.name == "external_sensor" || term.name == "feature")
        {
            term.count = getCountOrDefault(term_node, 0);
        }
        else
        {
            term.count = getCountOrDefault(term_node, 0);
        }

        manifest.terms_.push_back(term);
    }
    return manifest;
}

const std::vector<ObservationTermConfig> &ObservationManifest::terms() const
{
    return terms_;
}

const std::unordered_map<std::string, ObservationBuilder::ObservationProvider> &ObservationBuilder::registry()
{
    static const std::unordered_map<std::string, ObservationProvider> kRegistry = {
        {"phase",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double phase_t, const Sim2realCfg &cfg, const std::vector<int> &, const ObservationFeatureContext &, std::vector<float> *out) {
              const int phase_count = std::min(term.count, 2);
              const double cycle = std::max(1e-6, cfg.cycle_time);
              if (phase_count >= 1)
              {
                  out->push_back(static_cast<float>(std::sin(2.0 * M_PI * phase_t / cycle)));
              }
              if (phase_count >= 2)
              {
                  out->push_back(static_cast<float>(std::cos(2.0 * M_PI * phase_t / cycle)));
              }
          },
          2,
          true,
          false}},
        {"command",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &cmd, const std::vector<float> &, double, const Sim2realCfg &cfg, const std::vector<int> &, const ObservationFeatureContext &, std::vector<float> *out) {
              if (term.components.empty())
              {
                  const int count = term.count <= 0 ? 3 : term.count;
                  if (count >= 1)
                  {
                      out->push_back(cmd.vx * cfg.scales.lin_vel);
                  }
                  if (count >= 2)
                  {
                      out->push_back(cmd.vy * cfg.scales.lin_vel);
                  }
                  if (count >= 3)
                  {
                      out->push_back(cmd.dyaw * cfg.scales.ang_vel);
                  }
                  return;
              }
              for (const auto &component : term.components)
              {
                  if (component == "vx")
                  {
                      out->push_back(cmd.vx * cfg.scales.lin_vel);
                  }
                  else if (component == "vy")
                  {
                      out->push_back(cmd.vy * cfg.scales.lin_vel);
                  }
                  else if (component == "dyaw")
                  {
                      out->push_back(cmd.dyaw * cfg.scales.ang_vel);
                  }
                  else
                  {
                      throw std::runtime_error("Unknown command component: " + component);
                  }
              }
          },
          3,
          true,
          true}},
        {"joint_pos",
         {[](const ObservationTermConfig &term, const RobotState &robot, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &cfg, const std::vector<int> &obs_index_map, const ObservationFeatureContext &, std::vector<float> *out) {
              const int max_count = std::min(term.count, static_cast<int>(obs_index_map.size()));
              for (int i = 0; i < term.count; ++i)
              {
                  if (i < max_count)
                  {
                      const int robot_idx = obs_index_map[static_cast<size_t>(i)];
                      const bool valid = robot_idx >= 0 &&
                                         static_cast<size_t>(robot_idx) < robot.joint_q.size() &&
                                         static_cast<size_t>(robot_idx) < robot.default_angle.size();
                      const float v = valid ? (robot.joint_q[static_cast<size_t>(robot_idx)] - robot.default_angle[static_cast<size_t>(robot_idx)]) * cfg.scales.dof_pos : 0.0f;
                      out->push_back(v);
                  }
                  else
                  {
                      out->push_back(0.0f);
                  }
              }
          },
          12,
          true,
          false}},
        {"joint_vel",
         {[](const ObservationTermConfig &term, const RobotState &robot, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &cfg, const std::vector<int> &obs_index_map, const ObservationFeatureContext &, std::vector<float> *out) {
              const int max_count = std::min(term.count, static_cast<int>(obs_index_map.size()));
              for (int i = 0; i < term.count; ++i)
              {
                  if (i < max_count)
                  {
                      const int robot_idx = obs_index_map[static_cast<size_t>(i)];
                      const bool valid = robot_idx >= 0 && static_cast<size_t>(robot_idx) < robot.joint_dq.size();
                      out->push_back(valid ? robot.joint_dq[static_cast<size_t>(robot_idx)] * cfg.scales.dof_vel : 0.0f);
                  }
                  else
                  {
                      out->push_back(0.0f);
                  }
              }
          },
          12,
          true,
          false}},
        {"last_action",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &last_action, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &, std::vector<float> *out) {
              const int max_count = std::min(term.count, static_cast<int>(last_action.size()));
              for (int i = 0; i < term.count; ++i)
              {
                  pushPadded(out, i < max_count ? last_action[static_cast<size_t>(i)] : 0.0f, i, max_count);
              }
          },
          12,
          true,
          false}},
        {"base_ang_vel",
         {[](const ObservationTermConfig &term, const RobotState &robot, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &cfg, const std::vector<int> &, const ObservationFeatureContext &, std::vector<float> *out) {
              const int max_count = std::min(term.count, static_cast<int>(robot.base_ang_vel.size()));
              for (int i = 0; i < term.count; ++i)
              {
                  pushPadded(out, i < max_count ? robot.base_ang_vel[static_cast<size_t>(i)] * cfg.scales.ang_vel : 0.0f, i, max_count);
              }
          },
          3,
          true,
          false}},
        {"base_rpy",
         {[](const ObservationTermConfig &term, const RobotState &robot, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &cfg, const std::vector<int> &, const ObservationFeatureContext &, std::vector<float> *out) {
              const std::vector<float> rpy = quaternion_to_euler_array(robot.base_quat);
              const int max_count = std::min(term.count, static_cast<int>(rpy.size()));
              for (int i = 0; i < term.count; ++i)
              {
                  pushPadded(out, i < max_count ? rpy[static_cast<size_t>(i)] * cfg.scales.quat : 0.0f, i, max_count);
              }
          },
          3,
          true,
          false}},
        {"reference_motion",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "reference_motion", out);
          },
          0,
          true,
          false}},
        {"reference_joint_pos",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "reference_joint_pos", out);
          },
          12,
          true,
          false}},
        {"reference_joint_vel",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "reference_joint_vel", out);
          },
          12,
          true,
          false}},
        {"motion_anchor_pos_b",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "motion_anchor_pos_b", out);
          },
          3,
          true,
          false}},
        {"motion_ref_pos_b",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "motion_anchor_pos_b", out);
          },
          3,
          true,
          false}},
        {"motion_anchor_ori_b",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "motion_anchor_ori_b", out);
          },
          6,
          true,
          false}},
        {"motion_ref_ori_b",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "motion_anchor_ori_b", out);
          },
          6,
          true,
          false}},
        {"motion_body_pos_b",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "motion_body_pos_b", out);
          },
          0,
          true,
          false}},
        {"motion_body_ori_b",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "motion_body_ori_b", out);
          },
          0,
          true,
          false}},
        {"robot_body_pos",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "robot_body_pos", out);
          },
          0,
          true,
          false}},
        {"robot_body_ori",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "robot_body_ori", out);
          },
          0,
          true,
          false}},
        {"external_sensor",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              appendFeatureVector(term, feature_context, "external_sensor", out);
          },
          0,
          true,
          false}},
        {"feature",
         {[](const ObservationTermConfig &term, const RobotState &, const Cmd &, const std::vector<float> &, double, const Sim2realCfg &, const std::vector<int> &, const ObservationFeatureContext &feature_context, std::vector<float> *out) {
              if (term.source.empty())
              {
                  throw std::runtime_error("feature term requires non-empty source field");
              }
              const std::vector<float> *source = findFeature(feature_context, term.source);
              const int max_count = source ? std::min(term.count, static_cast<int>(source->size())) : 0;
              for (int i = 0; i < term.count; ++i)
              {
                  pushPadded(out, (source && i < max_count) ? (*source)[static_cast<size_t>(i)] : 0.0f, i, max_count);
              }
          },
          0,
          true,
          false}},
    };
    return kRegistry;
}

size_t ObservationBuilder::resolveTermDim(const ObservationTermConfig &term, const ObservationProvider &provider)
{
    if (!provider.supports_count && term.count > 0)
    {
        return static_cast<size_t>(provider.default_dim);
    }
    if (term.name == "command" && !term.components.empty())
    {
        return static_cast<size_t>(term.components.size());
    }
    if (term.count > 0)
    {
        return static_cast<size_t>(term.count);
    }
    return static_cast<size_t>(provider.default_dim);
}

ObservationBuilder::ObservationBuilder(ObservationManifest manifest)
    : manifest_(std::move(manifest))
{
    const auto &provider_map = registry();
    size_t running_offset = 0;
    for (const auto &term : manifest_.terms())
    {
        if (!term.enabled)
        {
            continue;
        }

        const auto provider_it = provider_map.find(term.name);
        if (provider_it == provider_map.end())
        {
            throw std::runtime_error("Unsupported observation term name: " + term.name);
        }
        const auto &provider = provider_it->second;

        if (!provider.supports_components && !term.components.empty())
        {
            throw std::runtime_error("Term does not support components: " + term.name);
        }
        if (!provider.supports_count && term.count > 0)
        {
            throw std::runtime_error("Term does not support count override: " + term.name);
        }
        if (term.name == "phase" && term.count > 2)
        {
            throw std::runtime_error("phase term count cannot exceed 2");
        }
        if ((term.name == "reference_motion" || term.name == "external_sensor" || term.name == "feature" ||
             term.name == "motion_body_pos_b" || term.name == "motion_body_ori_b" ||
             term.name == "robot_body_pos" || term.name == "robot_body_ori") &&
            term.count <= 0)
        {
            throw std::runtime_error(term.name + " term requires count > 0");
        }

        ResolvedObservationTerm resolved;
        resolved.config = term;
        resolved.gather = provider.gather;
        resolved.offset = running_offset;
        resolved.dim = resolveTermDim(term, provider);

        std::ostringstream oss;
        oss << resolved.config.name
            << " [offset=" << resolved.offset
            << ", dim=" << resolved.dim;
        if (!resolved.config.source.empty())
        {
            oss << ", source=" << resolved.config.source;
        }
        oss << "]";
        resolved.description = oss.str();
        layout_description_.push_back(resolved.description);

        running_offset += resolved.dim;
        resolved_terms_.push_back(resolved);
    }

    expected_dim_ = running_offset;
}

size_t ObservationBuilder::expectedDim() const
{
    return expected_dim_;
}

const std::vector<std::string> &ObservationBuilder::layoutDescription() const
{
    return layout_description_;
}

std::vector<float> ObservationBuilder::build(
    const RobotState &robot,
    const Cmd &cmd,
    const std::vector<float> &last_action,
    double phase_t,
    const Sim2realCfg &cfg,
    const std::vector<int> &obs_index_map,
    const ObservationFeatureContext &feature_context) const
{
    std::vector<float> obs;
    obs.reserve(expected_dim_);

    for (const auto &resolved_term : resolved_terms_)
    {
        const size_t before = obs.size();
        resolved_term.gather(resolved_term.config, robot, cmd, last_action, phase_t, cfg, obs_index_map, feature_context, &obs);
        const size_t added = obs.size() - before;
        if (added != resolved_term.dim)
        {
            throw std::runtime_error(
                "Observation term dim mismatch for '" + resolved_term.config.name +
                "': expected " + std::to_string(resolved_term.dim) +
                ", got " + std::to_string(added));
        }
    }

    for (auto &value : obs)
    {
        value = std::clamp(value, -cfg.clip_observations, cfg.clip_observations);
    }

    if (obs.size() != expected_dim_)
    {
        throw std::runtime_error("Observation dim mismatch: got " + std::to_string(obs.size()) +
                                 ", expected " + std::to_string(expected_dim_));
    }
    return obs;
}
