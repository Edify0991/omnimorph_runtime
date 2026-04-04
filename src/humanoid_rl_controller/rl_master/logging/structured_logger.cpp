#include "rl_master/logging/structured_logger.h"

#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace rl_master::logging
{
namespace
{
constexpr int kSchemaVersion = 1;

void appendQuoted(std::ostringstream &oss, const std::string &value)
{
    oss << '"' << StructuredLogger::escapeJsonString(value) << '"';
}

void appendStringMap(
    std::ostringstream &oss,
    const std::map<std::string, std::string> &data)
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

void appendNumericMap(
    std::ostringstream &oss,
    const std::map<std::string, double> &data)
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
        oss << ":" << StructuredLogger::formatDouble(value);
    }
    oss << "}";
}

void appendFloatVectorMap(
    std::ostringstream &oss,
    const std::map<std::string, std::vector<float>> &data)
{
    oss << "{";
    bool first_key = true;
    for (const auto &[key, values] : data)
    {
        if (!first_key)
        {
            oss << ",";
        }
        first_key = false;
        appendQuoted(oss, key);
        oss << ":[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                oss << ",";
            }
            oss << StructuredLogger::formatDouble(static_cast<double>(values[i]));
        }
        oss << "]";
    }
    oss << "}";
}

void appendDoubleVectorMap(
    std::ostringstream &oss,
    const std::map<std::string, std::vector<double>> &data)
{
    oss << "{";
    bool first_key = true;
    for (const auto &[key, values] : data)
    {
        if (!first_key)
        {
            oss << ",";
        }
        first_key = false;
        appendQuoted(oss, key);
        oss << ":[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                oss << ",";
            }
            oss << StructuredLogger::formatDouble(values[i]);
        }
        oss << "]";
    }
    oss << "}";
}

void appendStringListMap(
    std::ostringstream &oss,
    const std::map<std::string, std::vector<std::string>> &data)
{
    oss << "{";
    bool first_key = true;
    for (const auto &[key, values] : data)
    {
        if (!first_key)
        {
            oss << ",";
        }
        first_key = false;
        appendQuoted(oss, key);
        oss << ":[";
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
    oss << "}";
}

} // namespace

StructuredLogger::~StructuredLogger()
{
    close();
}

bool StructuredLogger::open(
    const std::string &session_base_path,
    const std::string &module_name,
    const LoggerMetadata &metadata)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (records_file_.is_open())
    {
        records_file_.flush();
        records_file_.close();
    }
    open_ = false;

    try
    {
        const std::filesystem::path base(session_base_path);
        const std::filesystem::path parent = base.parent_path();
        if (!parent.empty())
        {
            std::filesystem::create_directories(parent);
        }
    }
    catch (const std::exception &)
    {
        return false;
    }

    metadata_path_ = session_base_path + "_" + module_name + "_metadata.json";
    records_path_ = session_base_path + "_" + module_name + "_records.jsonl";

    records_file_.open(records_path_, std::ios::out | std::ios::trunc);
    if (!records_file_.is_open())
    {
        open_ = false;
        return false;
    }

    writeMetadataFile(metadata);
    open_ = true;
    return true;
}

bool StructuredLogger::isOpen() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return open_ && records_file_.is_open();
}

void StructuredLogger::writeRecord(
    double monotonic_time_sec,
    const std::string &record_type,
    const std::map<std::string, double> &scalars,
    const std::map<std::string, std::vector<float>> &vectors)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_ || !records_file_.is_open())
    {
        return;
    }

    std::ostringstream oss;
    oss << "{";
    oss << "\"schema_version\":" << kSchemaVersion << ",";
    oss << "\"record_type\":";
    appendQuoted(oss, record_type);
    oss << ",\"monotonic_time_sec\":" << formatDouble(monotonic_time_sec) << ",";
    oss << "\"scalars\":";
    appendNumericMap(oss, scalars);
    oss << ",\"vectors\":";
    appendFloatVectorMap(oss, vectors);
    oss << "}";

    records_file_ << oss.str() << "\n";
}

void StructuredLogger::writeEvent(
    double monotonic_time_sec,
    const std::string &event_type,
    const std::map<std::string, std::string> &tags)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_ || !records_file_.is_open())
    {
        return;
    }

    std::ostringstream oss;
    oss << "{";
    oss << "\"schema_version\":" << kSchemaVersion << ",";
    oss << "\"record_type\":\"event\",";
    oss << "\"event_type\":";
    appendQuoted(oss, event_type);
    oss << ",\"monotonic_time_sec\":" << formatDouble(monotonic_time_sec) << ",";
    oss << "\"tags\":";
    appendStringMap(oss, tags);
    oss << "}";

    records_file_ << oss.str() << "\n";
}

void StructuredLogger::flush()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (records_file_.is_open())
    {
        records_file_.flush();
    }
}

void StructuredLogger::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (records_file_.is_open())
    {
        records_file_.flush();
        records_file_.close();
    }
    open_ = false;
}

const std::string &StructuredLogger::metadataPath() const
{
    return metadata_path_;
}

const std::string &StructuredLogger::recordsPath() const
{
    return records_path_;
}

std::string StructuredLogger::escapeJsonString(const std::string &raw)
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

std::string StructuredLogger::formatDouble(double value)
{
    std::ostringstream oss;
    oss << std::setprecision(10) << value;
    return oss.str();
}

void StructuredLogger::writeMetadataFile(const LoggerMetadata &metadata)
{
    std::ofstream metadata_file(metadata_path_, std::ios::out | std::ios::trunc);
    if (!metadata_file.is_open())
    {
        return;
    }

    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto now_sec = std::chrono::duration_cast<std::chrono::duration<double>>(now).count();

    std::ostringstream oss;
    oss << "{";
    oss << "\"schema_version\":" << kSchemaVersion << ",";
    oss << "\"created_time_unix_sec\":" << formatDouble(now_sec) << ",";
    oss << "\"string_fields\":";
    appendStringMap(oss, metadata.string_fields);
    oss << ",\"numeric_fields\":";
    appendNumericMap(oss, metadata.numeric_fields);
    oss << ",\"vector_fields\":";
    appendDoubleVectorMap(oss, metadata.vector_fields);
    oss << ",\"string_list_fields\":";
    appendStringListMap(oss, metadata.string_list_fields);
    oss << "}";

    metadata_file << oss.str() << "\n";
    metadata_file.flush();
}

} // namespace rl_master::logging
