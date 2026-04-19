#ifndef RL_MASTER_LOGGING_RUNTIME_RECORDER_H
#define RL_MASTER_LOGGING_RUNTIME_RECORDER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <variant>

#include "rl_master/logging/mcap_writer.h"
#include "rl_master/logging/runtime_log_types.h"
#include "rl_master/rl_cfg.h"

namespace rl_master::logging
{

class RuntimeRecorder
{
public:
    RuntimeRecorder() = default;
    ~RuntimeRecorder();

    bool open(
        const RuntimeLoggingConfig &config,
        const std::string &config_snapshot_json,
        const std::map<std::string, std::string> &session_metadata);
    void close();
    void flush();

    bool isOpen() const;
    const std::string &filePath() const;
    const std::string &effectiveCompression() const;

    void recordEvent(const RuntimeEventRecord &record);
    void recordTick(const RuntimeTickLogRecord &record);
    void recordSourceSample(const RuntimeSourceSampleRecord &record);

private:
    struct ConfigSnapshotRecord
    {
        std::string json_payload;
    };

    struct MetadataRecord
    {
        std::string name;
        std::map<std::string, std::string> metadata;
    };

    struct FlushRecord
    {
    };

    struct StopRecord
    {
    };

    using QueueItem = std::variant<RuntimeEventRecord, RuntimeTickLogRecord, RuntimeSourceSampleRecord, ConfigSnapshotRecord, MetadataRecord, FlushRecord, StopRecord>;

    void enqueue(QueueItem item);
    void workerLoop();
    void writeDropNoticeIfNeeded();

    RuntimeLoggingConfig config_{};
    McapWriter writer_;
    std::atomic<bool> open_{false};
    std::atomic<uint64_t> tick_counter_{0};

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<QueueItem> queue_;
    uint64_t dropped_records_ = 0;
    bool stop_requested_ = false;
    std::thread worker_thread_;
};

} // namespace rl_master::logging

#endif // RL_MASTER_LOGGING_RUNTIME_RECORDER_H
