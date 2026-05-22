#ifndef RL_MASTER_LOGGING_RUNTIME_LOG_TYPES_H
#define RL_MASTER_LOGGING_RUNTIME_LOG_TYPES_H

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace rl_master::logging
{

struct RuntimeEventRecord
{
    double monotonic_time_sec = 0.0;
    std::string event_type;
    std::map<std::string, std::string> tags;
};

struct RuntimeSourceSampleRecord
{
    double monotonic_time_sec = 0.0;
    std::string topic;
    std::string sample_name;
    std::map<std::string, std::string> tags;
    std::unordered_map<std::string, std::vector<float>> values;
};

struct ControllerLogSnapshot
{
    bool valid = false;
    uint64_t frame_index = 0;
    double monotonic_time_sec = 0.0;
    double phase_t = 0.0;
    double phase_t_global = 0.0;
    double phase_origin_t = 0.0;
    int requested_mode_command = 0;
    int active_mode_id = 0;
    int deploy_state = 0;
    int active_profile_index = 0;
    uint64_t policy_step_index = 0;
    bool policy_ran_this_tick = false;
    double policy_sample_time_sec = 0.0;
    double policy_sample_age_sec = 0.0;
    float open_rl = 0.0f;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_dyaw = 0.0f;
    std::string active_tag;
    std::string active_config_section;
    std::string policy_name;
    uint64_t runtime_warning_seq = 0;
    std::string runtime_warning_type;
    std::string runtime_warning_message;
    std::map<std::string, std::string> runtime_warning_tags;
    std::vector<float> joint_q;
    std::vector<float> joint_dq;
    std::vector<float> joint_tau;
    std::vector<float> joint_target_q;
    std::vector<float> joint_target_tau;
    std::vector<float> observation;
    std::vector<float> policy_action;
    std::unordered_map<std::string, std::vector<float>> named_features;
    std::vector<std::string> external_feature_names;
};

struct RuntimeTickLogRecord
{
    uint64_t frame_index = 0;
    double monotonic_time_sec = 0.0;
    double phase_t = 0.0;
    double phase_t_global = 0.0;
    double phase_origin_t = 0.0;
    int requested_mode_command = 0;
    int active_mode_id = 0;
    int deploy_state = 0;
    int active_profile_index = 0;
    uint64_t policy_step_index = 0;
    bool policy_ran_this_tick = false;
    double policy_sample_time_sec = 0.0;
    double policy_sample_age_sec = 0.0;
    float open_rl = 0.0f;
    float cmd_vx = 0.0f;
    float cmd_vy = 0.0f;
    float cmd_dyaw = 0.0f;
    bool latest_cmd_fresh = true;
    uint64_t loop_overrun_count = 0;
    std::string active_tag;
    std::string active_config_section;
    std::string policy_name;
    uint64_t runtime_warning_seq = 0;
    std::string runtime_warning_type;
    std::string runtime_warning_message;
    std::map<std::string, std::string> runtime_warning_tags;

    std::vector<float> joint_q;
    std::vector<float> joint_dq;
    std::vector<float> joint_tau;
    std::vector<float> joint_target_q;
    std::vector<float> joint_target_tau;
    std::vector<float> observation;
    std::vector<float> policy_action;

    std::vector<float> joint_cmd_q;
    std::vector<float> joint_cmd_dq;
    std::vector<float> joint_cmd_tau;
    std::vector<float> joint_state_q;
    std::vector<float> joint_state_dq;
    std::vector<float> joint_state_tau;
    std::vector<float> motor_cmd_q;
    std::vector<float> motor_cmd_dq;
    std::vector<float> motor_cmd_tau;
    std::vector<float> motor_state_q;
    std::vector<float> motor_state_dq;
    std::vector<float> motor_state_tau;
    std::vector<float> motor_cmd_mode;

    std::unordered_map<std::string, std::vector<float>> named_features;
    std::vector<std::string> external_feature_names;
};

} // namespace rl_master::logging

#endif // RL_MASTER_LOGGING_RUNTIME_LOG_TYPES_H
