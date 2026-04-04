#ifndef RL_MASTER_LOGGING_STRUCTURED_LOGGER_H
#define RL_MASTER_LOGGING_STRUCTURED_LOGGER_H

#include <cstdint>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace rl_master::logging
{

struct LoggerMetadata
{
    std::map<std::string, std::string> string_fields;
    std::map<std::string, double> numeric_fields;
    std::map<std::string, std::vector<double>> vector_fields;
    std::map<std::string, std::vector<std::string>> string_list_fields;
};

class StructuredLogger
{
public:
    StructuredLogger() = default;
    ~StructuredLogger();

    bool open(
        const std::string &session_base_path,
        const std::string &module_name,
        const LoggerMetadata &metadata);

    bool isOpen() const;

    void writeRecord(
        double monotonic_time_sec,
        const std::string &record_type,
        const std::map<std::string, double> &scalars,
        const std::map<std::string, std::vector<float>> &vectors);

    void writeEvent(
        double monotonic_time_sec,
        const std::string &event_type,
        const std::map<std::string, std::string> &tags = {});

    void flush();
    void close();

    const std::string &metadataPath() const;
    const std::string &recordsPath() const;

    static std::string escapeJsonString(const std::string &raw);
    static std::string formatDouble(double value);

private:
    void writeMetadataFile(const LoggerMetadata &metadata);

    mutable std::mutex mutex_;
    std::ofstream records_file_;
    std::string metadata_path_;
    std::string records_path_;
    bool open_ = false;
};

} // namespace rl_master::logging

#endif // RL_MASTER_LOGGING_STRUCTURED_LOGGER_H
