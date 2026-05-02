#include "rl_master/logging/runtime_recorder.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>

#include "rl_master/rl_protocol.h"

namespace rl_master::logging
{
namespace
{
uint64_t secToNs(double sec)
{
    if (!std::isfinite(sec) || sec <= 0.0)
    {
        return 0;
    }
    return static_cast<uint64_t>(sec * 1.0e9);
}

std::string escapeJsonString(const std::string &raw)
{
    std::ostringstream oss;
    for (char ch : raw)
    {
        switch (ch)
        {
        case '"':
            oss << "\\\"";
            break;
        case '\\':
            oss << "\\\\";
            break;
        case '\b':
            oss << "\\b";
            break;
        case '\f':
            oss << "\\f";
            break;
        case '\n':
            oss << "\\n";
            break;
        case '\r':
            oss << "\\r";
            break;
        case '\t':
            oss << "\\t";
            break;
        default:
            oss << ch;
            break;
        }
    }
    return oss.str();
}

std::string formatDouble(double value)
{
    std::ostringstream oss;
    oss.precision(10);
    oss << value;
    return oss.str();
}

void appendQuoted(std::ostringstream &oss, const std::string &value)
{
    oss << '"' << escapeJsonString(value) << '"';
}

void appendStringMap(std::ostringstream &oss, const std::map<std::string, std::string> &data)
{
    oss << "{";
    bool first = true;
    for (const auto &[key, value] : data)
    {
        if (!first)
        {
            oss << ",";
        }
        first = false;
        appendQuoted(oss, key);
        oss << ":";
        appendQuoted(oss, value);
    }
    oss << "}";
}

void appendStringVector(std::ostringstream &oss, const std::vector<std::string> &values)
{
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0)
        {
            oss << ",";
        }
        appendQuoted(oss, values[i]);
    }
    oss << "]";
}

void appendFloatVector(std::ostringstream &oss, const std::vector<float> &values)
{
    oss << "[";
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (i > 0)
        {
            oss << ",";
        }
        oss << formatDouble(static_cast<double>(values[i]));
    }
    oss << "]";
}

void appendNamedVectorMap(
    std::ostringstream &oss,
    const std::unordered_map<std::string, std::vector<float>> &data)
{
    oss << "{";
    bool first = true;
    for (const auto &[key, value] : data)
    {
        if (!first)
        {
            oss << ",";
        }
        first = false;
        appendQuoted(oss, key);
        oss << ":";
        appendFloatVector(oss, value);
    }
    oss << "}";
}

bool startsWith(const std::string &value, const std::string &prefix)
{
    return value.size() >= prefix.size() && value.compare(0, prefix.size(), prefix) == 0;
}

std::map<std::string, std::string> makeChannelMetadata(const RuntimeLoggingConfig &config)
{
    return {
        {"compression", config.writer.compression},
        {"chunk_size_bytes", std::to_string(std::max(1, config.writer.chunk_size_kb) * 1024)}};
}

std::string serializeEvent(const RuntimeEventRecord &record)
{
    std::ostringstream oss;
    oss << "{";
    oss << "\"monotonic_time_sec\":" << formatDouble(record.monotonic_time_sec) << ",";
    oss << "\"event_type\":";
    appendQuoted(oss, record.event_type);
    oss << ",\"tags\":";
    appendStringMap(oss, record.tags);
    oss << "}";
    return oss.str();
}

std::string serializeSourceSample(const RuntimeSourceSampleRecord &record)
{
    std::ostringstream oss;
    oss << "{";
    oss << "\"monotonic_time_sec\":" << formatDouble(record.monotonic_time_sec) << ",";
    oss << "\"sample_name\":";
    appendQuoted(oss, record.sample_name);
    oss << ",\"tags\":";
    appendStringMap(oss, record.tags);
    oss << ",\"values\":";
    appendNamedVectorMap(oss, record.values);
    oss << "}";
    return oss.str();
}

std::string serializeTick(const RuntimeTickLogRecord &record, const RuntimeLoggingConfig &config)
{
    std::ostringstream oss;
    oss << "{";
    oss << "\"frame_index\":" << record.frame_index << ",";
    oss << "\"monotonic_time_sec\":" << formatDouble(record.monotonic_time_sec) << ",";
    oss << "\"phase_t\":" << formatDouble(record.phase_t) << ",";
    oss << "\"phase_t_global\":" << formatDouble(record.phase_t_global) << ",";
    oss << "\"phase_origin_t\":" << formatDouble(record.phase_origin_t) << ",";
    oss << "\"requested_mode_command\":" << record.requested_mode_command << ",";
    oss << "\"active_mode_id\":" << record.active_mode_id << ",";
    oss << "\"deploy_state\":" << record.deploy_state << ",";
    oss << "\"active_profile_index\":" << record.active_profile_index << ",";
    oss << "\"policy_step_index\":" << record.policy_step_index << ",";
    oss << "\"policy_ran_this_tick\":" << (record.policy_ran_this_tick ? "true" : "false") << ",";
    oss << "\"policy_sample_time_sec\":" << formatDouble(record.policy_sample_time_sec) << ",";
    oss << "\"policy_sample_age_sec\":" << formatDouble(record.policy_sample_age_sec) << ",";
    oss << "\"open_rl\":" << formatDouble(record.open_rl) << ",";
    oss << "\"cmd_vx\":" << formatDouble(record.cmd_vx) << ",";
    oss << "\"cmd_vy\":" << formatDouble(record.cmd_vy) << ",";
    oss << "\"cmd_dyaw\":" << formatDouble(record.cmd_dyaw) << ",";
    oss << "\"latest_cmd_fresh\":" << (record.latest_cmd_fresh ? "true" : "false") << ",";
    oss << "\"loop_overrun_count\":" << record.loop_overrun_count << ",";
    oss << "\"active_tag\":";
    appendQuoted(oss, record.active_tag);
    oss << ",\"active_config_section\":";
    appendQuoted(oss, record.active_config_section);
    oss << ",\"policy_name\":";
    appendQuoted(oss, record.policy_name);
    oss << ",\"runtime_warning_seq\":" << record.runtime_warning_seq;
    oss << ",\"runtime_warning_type\":";
    appendQuoted(oss, record.runtime_warning_type);
    oss << ",\"runtime_warning_message\":";
    appendQuoted(oss, record.runtime_warning_message);
    oss << ",\"runtime_warning_tags\":";
    appendStringMap(oss, record.runtime_warning_tags);
    oss << ",\"joint_q\":";
    appendFloatVector(oss, record.joint_q);
    oss << ",\"joint_dq\":";
    appendFloatVector(oss, record.joint_dq);
    oss << ",\"joint_tau\":";
    appendFloatVector(oss, record.joint_tau);
    if (config.tick.include_joint_targets)
    {
        oss << ",\"joint_target_q\":";
        appendFloatVector(oss, record.joint_target_q);
        oss << ",\"joint_target_tau\":";
        appendFloatVector(oss, record.joint_target_tau);
    }
    if (config.tick.include_observation)
    {
        oss << ",\"observation\":";
        appendFloatVector(oss, record.observation);
    }
    if (config.tick.include_policy_action)
    {
        oss << ",\"policy_action\":";
        appendFloatVector(oss, record.policy_action);
    }
    if (config.tick.include_motor_io)
    {
        oss << ",\"joint_cmd_q\":";
        appendFloatVector(oss, record.joint_cmd_q);
        oss << ",\"joint_cmd_dq\":";
        appendFloatVector(oss, record.joint_cmd_dq);
        oss << ",\"joint_cmd_tau\":";
        appendFloatVector(oss, record.joint_cmd_tau);
        oss << ",\"joint_state_q\":";
        appendFloatVector(oss, record.joint_state_q);
        oss << ",\"joint_state_dq\":";
        appendFloatVector(oss, record.joint_state_dq);
        oss << ",\"joint_state_tau\":";
        appendFloatVector(oss, record.joint_state_tau);
        oss << ",\"motor_cmd_q\":";
        appendFloatVector(oss, record.motor_cmd_q);
        oss << ",\"motor_cmd_dq\":";
        appendFloatVector(oss, record.motor_cmd_dq);
        oss << ",\"motor_cmd_tau\":";
        appendFloatVector(oss, record.motor_cmd_tau);
        oss << ",\"motor_state_q\":";
        appendFloatVector(oss, record.motor_state_q);
        oss << ",\"motor_state_dq\":";
        appendFloatVector(oss, record.motor_state_dq);
        oss << ",\"motor_state_tau\":";
        appendFloatVector(oss, record.motor_state_tau);
        oss << ",\"motor_cmd_mode\":";
        appendFloatVector(oss, record.motor_cmd_mode);
    }
    if (config.tick.include_external_observations)
    {
        oss << ",\"named_features\":";
        appendNamedVectorMap(oss, record.named_features);
        oss << ",\"external_feature_names\":";
        appendStringVector(oss, record.external_feature_names);
    }
    oss << "}";
    return oss.str();
}

std::string serializeFeaturePayload(
    const RuntimeTickLogRecord &record,
    const std::string &name,
    const std::vector<float> &value)
{
    std::ostringstream oss;
    oss << "{";
    oss << "\"frame_index\":" << record.frame_index << ",";
    oss << "\"monotonic_time_sec\":" << formatDouble(record.monotonic_time_sec) << ",";
    oss << "\"active_mode_id\":" << record.active_mode_id << ",";
    oss << "\"name\":";
    appendQuoted(oss, name);
    oss << ",\"value\":";
    appendFloatVector(oss, value);
    oss << "}";
    return oss.str();
}

std::string serializeReferencePayload(const RuntimeTickLogRecord &record)
{
    std::unordered_map<std::string, std::vector<float>> selected;
    for (const auto &[name, value] : record.named_features)
    {
        if (name == "reference_motion" || startsWith(name, "reference_") || startsWith(name, "motion_"))
        {
            selected[name] = value;
        }
    }

    std::ostringstream oss;
    oss << "{";
    oss << "\"frame_index\":" << record.frame_index << ",";
    oss << "\"monotonic_time_sec\":" << formatDouble(record.monotonic_time_sec) << ",";
    oss << "\"active_mode_id\":" << record.active_mode_id << ",";
    oss << "\"features\":";
    appendNamedVectorMap(oss, selected);
    oss << "}";
    return oss.str();
}

} // namespace

RuntimeRecorder::~RuntimeRecorder()
{
    close();
}

bool RuntimeRecorder::open(
    const RuntimeLoggingConfig &config,
    const std::string &config_snapshot_json,
    const std::map<std::string, std::string> &session_metadata)
{
    close();
    config_ = config;
    if (!config_.enabled)
    {
        return false;
    }
    if (!writer_.open(config_.output_file_path, "rl_master_runtime_recorder"))
    {
        return false;
    }
    writer_.configureChunking(
        config_.writer.compression,
        static_cast<size_t>(std::max(1, config_.writer.chunk_size_kb)) * 1024);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        dropped_records_ = 0;
        stop_requested_ = false;
    }
    open_.store(true);
    worker_thread_ = std::thread(&RuntimeRecorder::workerLoop, this);
    std::map<std::string, std::string> effective_session_metadata = session_metadata;
    effective_session_metadata["requested_compression"] = config_.writer.compression;
    effective_session_metadata["effective_compression"] = writer_.effectiveCompression();
    enqueue(MetadataRecord{"session", effective_session_metadata});
    enqueue(ConfigSnapshotRecord{config_snapshot_json});
    return true;
}

void RuntimeRecorder::close()
{
    if (!open_.exchange(false))
    {
        if (worker_thread_.joinable())
        {
            worker_thread_.join();
        }
        writer_.close();
        return;
    }

    enqueue(StopRecord{});
    if (worker_thread_.joinable())
    {
        worker_thread_.join();
    }
    writer_.close();
}

void RuntimeRecorder::flush()
{
    if (!open_.load())
    {
        return;
    }
    enqueue(FlushRecord{});
}

bool RuntimeRecorder::isOpen() const
{
    return open_.load() && writer_.isOpen();
}

const std::string &RuntimeRecorder::filePath() const
{
    return writer_.filePath();
}

const std::string &RuntimeRecorder::effectiveCompression() const
{
    return writer_.effectiveCompression();
}

void RuntimeRecorder::recordEvent(const RuntimeEventRecord &record)
{
    if (!isOpen() || !config_.events.enabled)
    {
        return;
    }
    enqueue(record);
}

void RuntimeRecorder::recordTick(const RuntimeTickLogRecord &record)
{
    if (!isOpen() || !config_.tick.enabled)
    {
        return;
    }
    const uint64_t tick_index = tick_counter_.fetch_add(1);
    if ((tick_index % static_cast<uint64_t>(config_.tick.decimation)) != 0)
    {
        return;
    }
    enqueue(record);
}

void RuntimeRecorder::recordSourceSample(const RuntimeSourceSampleRecord &record)
{
    if (!isOpen() || !config_.source_samples.enabled || record.topic.empty())
    {
        return;
    }
    enqueue(record);
}

void RuntimeRecorder::enqueue(QueueItem item)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stop_requested_)
        {
            return;
        }
        if (queue_.size() >= static_cast<size_t>(config_.writer.queue_capacity))
        {
            queue_.pop_front();
            ++dropped_records_;
        }
        if (std::holds_alternative<StopRecord>(item))
        {
            stop_requested_ = true;
        }
        queue_.push_back(std::move(item));
    }
    cv_.notify_one();
}

void RuntimeRecorder::workerLoop()
{
    const auto flush_period = std::chrono::milliseconds(config_.writer.flush_period_ms);
    while (true)
    {
        QueueItem item;
        bool has_item = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait_for(lock, flush_period, [this]() {
                return !queue_.empty() || stop_requested_;
            });
            if (!queue_.empty())
            {
                item = std::move(queue_.front());
                queue_.pop_front();
                has_item = true;
            }
            else if (stop_requested_)
            {
                break;
            }
        }

        writeDropNoticeIfNeeded();

        if (!has_item)
        {
            writer_.flush();
            continue;
        }

        if (std::holds_alternative<StopRecord>(item))
        {
            break;
        }
        if (std::holds_alternative<FlushRecord>(item))
        {
            writer_.flush();
            continue;
        }
        if (const auto *metadata = std::get_if<MetadataRecord>(&item))
        {
            writer_.writeMetadata(metadata->name, metadata->metadata);
            continue;
        }
        if (const auto *config_snapshot = std::get_if<ConfigSnapshotRecord>(&item))
        {
            const uint64_t now_ns = secToNs(rl_master::monotonicTimeSec());
            writer_.writeJsonMessage("runtime/config", config_snapshot->json_payload, now_ns, now_ns, makeChannelMetadata(config_));
            continue;
        }
        if (const auto *event = std::get_if<RuntimeEventRecord>(&item))
        {
            const uint64_t time_ns = secToNs(event->monotonic_time_sec);
            writer_.writeJsonMessage("runtime/event", serializeEvent(*event), time_ns, time_ns, makeChannelMetadata(config_));
            continue;
        }
        if (const auto *source = std::get_if<RuntimeSourceSampleRecord>(&item))
        {
            const uint64_t time_ns = secToNs(source->monotonic_time_sec);
            writer_.writeJsonMessage(source->topic, serializeSourceSample(*source), time_ns, time_ns, makeChannelMetadata(config_));
            continue;
        }
        if (const auto *tick = std::get_if<RuntimeTickLogRecord>(&item))
        {
            const uint64_t time_ns = secToNs(tick->monotonic_time_sec);
            writer_.writeJsonMessage("runtime/tick", serializeTick(*tick, config_), time_ns, time_ns, makeChannelMetadata(config_));
            if (config_.reference_motion.enabled)
            {
                bool has_reference = false;
                for (const auto &[name, _] : tick->named_features)
                {
                    if (name == "reference_motion" || startsWith(name, "reference_") || startsWith(name, "motion_"))
                    {
                        has_reference = true;
                        break;
                    }
                }
                if (has_reference)
                {
                    writer_.writeJsonMessage("runtime/reference_motion", serializeReferencePayload(*tick), time_ns, time_ns, makeChannelMetadata(config_));
                }
            }
            if (config_.tick.include_external_observations)
            {
                for (const auto &name : tick->external_feature_names)
                {
                    const auto it = tick->named_features.find(name);
                    if (it == tick->named_features.end())
                    {
                        continue;
                    }
                    writer_.writeJsonMessage(
                        "runtime/external_obs/" + name,
                        serializeFeaturePayload(*tick, name, it->second),
                        time_ns,
                        time_ns,
                        makeChannelMetadata(config_));
                }
            }
            continue;
        }
    }

    while (true)
    {
        QueueItem item;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.empty())
            {
                break;
            }
            item = std::move(queue_.front());
            queue_.pop_front();
        }
        if (const auto *event = std::get_if<RuntimeEventRecord>(&item))
        {
            const uint64_t time_ns = secToNs(event->monotonic_time_sec);
            writer_.writeJsonMessage("runtime/event", serializeEvent(*event), time_ns, time_ns, makeChannelMetadata(config_));
        }
        else if (const auto *source = std::get_if<RuntimeSourceSampleRecord>(&item))
        {
            const uint64_t time_ns = secToNs(source->monotonic_time_sec);
            writer_.writeJsonMessage(source->topic, serializeSourceSample(*source), time_ns, time_ns, makeChannelMetadata(config_));
        }
        else if (const auto *tick = std::get_if<RuntimeTickLogRecord>(&item))
        {
            const uint64_t time_ns = secToNs(tick->monotonic_time_sec);
            writer_.writeJsonMessage("runtime/tick", serializeTick(*tick, config_), time_ns, time_ns, makeChannelMetadata(config_));
            if (config_.reference_motion.enabled)
            {
                bool has_reference = false;
                for (const auto &[name, _] : tick->named_features)
                {
                    if (name == "reference_motion" || startsWith(name, "reference_") || startsWith(name, "motion_"))
                    {
                        has_reference = true;
                        break;
                    }
                }
                if (has_reference)
                {
                    writer_.writeJsonMessage("runtime/reference_motion", serializeReferencePayload(*tick), time_ns, time_ns, makeChannelMetadata(config_));
                }
            }
            if (config_.tick.include_external_observations)
            {
                for (const auto &name : tick->external_feature_names)
                {
                    const auto it = tick->named_features.find(name);
                    if (it == tick->named_features.end())
                    {
                        continue;
                    }
                    writer_.writeJsonMessage(
                        "runtime/external_obs/" + name,
                        serializeFeaturePayload(*tick, name, it->second),
                        time_ns,
                        time_ns,
                        makeChannelMetadata(config_));
                }
            }
        }
        else if (const auto *config_snapshot = std::get_if<ConfigSnapshotRecord>(&item))
        {
            const uint64_t now_ns = secToNs(rl_master::monotonicTimeSec());
            writer_.writeJsonMessage("runtime/config", config_snapshot->json_payload, now_ns, now_ns, makeChannelMetadata(config_));
        }
        else if (const auto *metadata = std::get_if<MetadataRecord>(&item))
        {
            writer_.writeMetadata(metadata->name, metadata->metadata);
        }
    }
    writeDropNoticeIfNeeded();
    writer_.flush();
}

void RuntimeRecorder::writeDropNoticeIfNeeded()
{
    uint64_t dropped = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        dropped = dropped_records_;
        dropped_records_ = 0;
    }
    if (dropped == 0 || !writer_.isOpen())
    {
        return;
    }

    RuntimeEventRecord event;
    event.monotonic_time_sec = rl_master::monotonicTimeSec();
    event.event_type = "runtime_log_queue_drop";
    event.tags["dropped_count"] = std::to_string(dropped);
    const uint64_t time_ns = secToNs(event.monotonic_time_sec);
    writer_.writeJsonMessage("runtime/event", serializeEvent(event), time_ns, time_ns, makeChannelMetadata(config_));
}

} // namespace rl_master::logging
