#ifndef RL_MASTER_LOGGING_STRUCTURED_LOGGER_H
#define RL_MASTER_LOGGING_STRUCTURED_LOGGER_H

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <sstream>
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
    ~StructuredLogger()
    {
        flush();
    }

    bool open(
        const std::string &base_dir,
        const std::string &session_name,
        const LoggerMetadata &metadata)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        closeUnlocked();

        try
        {
            std::filesystem::path dir_path(base_dir);
            if (dir_path.empty())
            {
                return false;
            }
            std::filesystem::create_directories(dir_path);
            records_path_ = (dir_path / (session_name + ".jsonl")).string();
            stream_.open(records_path_, std::ios::out | std::ios::trunc);
            if (!stream_.is_open())
            {
                records_path_.clear();
                return false;
            }

            opened_ = true;
            writeMetadataUnlocked(metadata);
            stream_.flush();
            return stream_.good();
        }
        catch (const std::exception &)
        {
            closeUnlocked();
            return false;
        }
    }

    void writeEvent(
        double time_sec,
        const std::string &event_name,
        const std::map<std::string, std::string> &fields)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!opened_)
        {
            return;
        }

        stream_ << "{";
        writeJsonFieldUnlocked("kind", "event");
        stream_ << ",";
        writeJsonNumericFieldUnlocked("time_sec", time_sec);
        stream_ << ",";
        writeJsonFieldUnlocked("event", event_name);
        stream_ << ",\"fields\":";
        writeStringMapUnlocked(fields);
        stream_ << "}\n";
    }

    void writeRecord(
        double time_sec,
        const std::string &channel,
        const std::map<std::string, double> &scalars,
        const std::map<std::string, std::vector<float>> &vectors)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!opened_)
        {
            return;
        }

        stream_ << "{";
        writeJsonFieldUnlocked("kind", "record");
        stream_ << ",";
        writeJsonNumericFieldUnlocked("time_sec", time_sec);
        stream_ << ",";
        writeJsonFieldUnlocked("channel", channel);
        stream_ << ",\"scalars\":";
        writeNumericMapUnlocked(scalars);
        stream_ << ",\"vectors\":";
        writeFloatVectorMapUnlocked(vectors);
        stream_ << "}\n";
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stream_.is_open())
        {
            stream_.flush();
        }
    }

    const std::string &recordsPath() const
    {
        return records_path_;
    }

private:
    static std::string escapeJson(const std::string &value)
    {
        std::string out;
        out.reserve(value.size() + 8);
        for (const char ch : value)
        {
            switch (ch)
            {
            case '\\':
                out += "\\\\";
                break;
            case '"':
                out += "\\\"";
                break;
            case '\n':
                out += "\\n";
                break;
            case '\r':
                out += "\\r";
                break;
            case '\t':
                out += "\\t";
                break;
            default:
                out += ch;
                break;
            }
        }
        return out;
    }

    template <typename T>
    void writeVectorUnlocked(const std::vector<T> &values)
    {
        stream_ << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                stream_ << ",";
            }
            stream_ << values[i];
        }
        stream_ << "]";
    }

    void writeStringVectorUnlocked(const std::vector<std::string> &values)
    {
        stream_ << "[";
        for (size_t i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                stream_ << ",";
            }
            stream_ << "\"" << escapeJson(values[i]) << "\"";
        }
        stream_ << "]";
    }

    void writeJsonFieldUnlocked(const std::string &key, const std::string &value)
    {
        stream_ << "\"" << escapeJson(key) << "\":\"" << escapeJson(value) << "\"";
    }

    void writeJsonNumericFieldUnlocked(const std::string &key, double value)
    {
        stream_ << "\"" << escapeJson(key) << "\":" << std::setprecision(17) << value;
    }

    void writeStringMapUnlocked(const std::map<std::string, std::string> &fields)
    {
        stream_ << "{";
        bool first = true;
        for (const auto &entry : fields)
        {
            if (!first)
            {
                stream_ << ",";
            }
            first = false;
            writeJsonFieldUnlocked(entry.first, entry.second);
        }
        stream_ << "}";
    }

    template <typename T>
    void writeNumericMapUnlocked(const std::map<std::string, T> &fields)
    {
        stream_ << "{";
        bool first = true;
        for (const auto &entry : fields)
        {
            if (!first)
            {
                stream_ << ",";
            }
            first = false;
            stream_ << "\"" << escapeJson(entry.first) << "\":" << std::setprecision(17) << entry.second;
        }
        stream_ << "}";
    }

    void writeDoubleVectorMapUnlocked(const std::map<std::string, std::vector<double>> &fields)
    {
        stream_ << "{";
        bool first = true;
        for (const auto &entry : fields)
        {
            if (!first)
            {
                stream_ << ",";
            }
            first = false;
            stream_ << "\"" << escapeJson(entry.first) << "\":";
            writeVectorUnlocked(entry.second);
        }
        stream_ << "}";
    }

    void writeFloatVectorMapUnlocked(const std::map<std::string, std::vector<float>> &fields)
    {
        stream_ << "{";
        bool first = true;
        for (const auto &entry : fields)
        {
            if (!first)
            {
                stream_ << ",";
            }
            first = false;
            stream_ << "\"" << escapeJson(entry.first) << "\":";
            writeVectorUnlocked(entry.second);
        }
        stream_ << "}";
    }

    void writeStringListMapUnlocked(const std::map<std::string, std::vector<std::string>> &fields)
    {
        stream_ << "{";
        bool first = true;
        for (const auto &entry : fields)
        {
            if (!first)
            {
                stream_ << ",";
            }
            first = false;
            stream_ << "\"" << escapeJson(entry.first) << "\":";
            writeStringVectorUnlocked(entry.second);
        }
        stream_ << "}";
    }

    void writeMetadataUnlocked(const LoggerMetadata &metadata)
    {
        stream_ << "{";
        writeJsonFieldUnlocked("kind", "metadata");
        stream_ << ",\"string_fields\":";
        writeStringMapUnlocked(metadata.string_fields);
        stream_ << ",\"numeric_fields\":";
        writeNumericMapUnlocked(metadata.numeric_fields);
        stream_ << ",\"vector_fields\":";
        writeDoubleVectorMapUnlocked(metadata.vector_fields);
        stream_ << ",\"string_list_fields\":";
        writeStringListMapUnlocked(metadata.string_list_fields);
        stream_ << "}\n";
    }

    void closeUnlocked()
    {
        if (stream_.is_open())
        {
            stream_.flush();
            stream_.close();
        }
        opened_ = false;
        records_path_.clear();
    }

    mutable std::mutex mutex_;
    std::ofstream stream_;
    std::string records_path_;
    bool opened_ = false;
};

} // namespace rl_master::logging

#endif // RL_MASTER_LOGGING_STRUCTURED_LOGGER_H
