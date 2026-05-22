#include "rl_master/logging/mcap_writer.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <lz4frame.h>
#include <zstd.h>

namespace rl_master::logging
{
namespace
{
constexpr unsigned char kMagic[8] = {0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
}

McapWriter::~McapWriter()
{
    close();
}

bool McapWriter::open(const std::string &file_path, const std::string &library_name)
{
    close();

    try
    {
        const std::filesystem::path target(file_path);
        if (!target.parent_path().empty())
        {
            std::filesystem::create_directories(target.parent_path());
        }
    }
    catch (const std::exception &)
    {
        return false;
    }

    output_.open(file_path, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!output_.is_open())
    {
        return false;
    }

    file_path_ = file_path;
    next_channel_id_ = 1;
    topic_to_channel_id_.clear();
    channel_sequence_.clear();
    chunk_records_.clear();
    chunk_start_time_ns_ = 0;
    chunk_end_time_ns_ = 0;
    target_chunk_size_bytes_ = 1024 * 1024;
    effective_compression_ = normalizeCompression(requested_compression_);

    output_.write(reinterpret_cast<const char *>(kMagic), sizeof(kMagic));
    if (!writeHeader(library_name))
    {
        close();
        return false;
    }

    open_ = true;
    return true;
}

bool McapWriter::isOpen() const
{
    return open_ && output_.is_open();
}

void McapWriter::configureChunking(const std::string &compression, size_t chunk_size_bytes)
{
    requested_compression_ = compression;
    effective_compression_ = normalizeCompression(compression);
    if (chunk_size_bytes > 0)
    {
        target_chunk_size_bytes_ = chunk_size_bytes;
    }
}

bool McapWriter::writeMetadata(
    const std::string &name,
    const std::map<std::string, std::string> &metadata)
{
    if (!isOpen())
    {
        return false;
    }

    const uint64_t content_length =
        static_cast<uint64_t>(encodedStringSize(name)) +
        static_cast<uint64_t>(4 + encodedStringMapSize(metadata));
    writeRecordPrefix(kOpcodeMetadata, content_length);
    writeString(name);
    writeUint32(encodedStringMapSize(metadata));
    for (const auto &[key, value] : metadata)
    {
        writeString(key);
        writeString(value);
    }
    return static_cast<bool>(output_);
}

bool McapWriter::writeJsonMessage(
    const std::string &topic,
    const std::string &json_payload,
    uint64_t log_time_ns,
    uint64_t publish_time_ns,
    const std::map<std::string, std::string> &channel_metadata)
{
    if (!isOpen())
    {
        return false;
    }

    const uint16_t channel_id = ensureJsonChannel(topic, channel_metadata);
    if (channel_id == 0)
    {
        return false;
    }

    uint32_t sequence = 0;
    auto seq_it = channel_sequence_.find(channel_id);
    if (seq_it == channel_sequence_.end())
    {
        channel_sequence_[channel_id] = 1;
    }
    else
    {
        sequence = seq_it->second;
        ++seq_it->second;
    }
    return writeMessage(channel_id, sequence, log_time_ns, publish_time_ns, json_payload);
}

void McapWriter::flush()
{
    flushChunk();
    if (output_.is_open())
    {
        output_.flush();
    }
}

void McapWriter::close()
{
    if (open_ && output_.is_open())
    {
        flushChunk();
        writeDataEnd();
        writeFooter();
        output_.write(reinterpret_cast<const char *>(kMagic), sizeof(kMagic));
        output_.flush();
        output_.close();
    }
    else if (output_.is_open())
    {
        output_.close();
    }

    open_ = false;
    chunk_records_.clear();
    chunk_start_time_ns_ = 0;
    chunk_end_time_ns_ = 0;
}

const std::string &McapWriter::filePath() const
{
    return file_path_;
}

const std::string &McapWriter::effectiveCompression() const
{
    return effective_compression_;
}

bool McapWriter::writeHeader(const std::string &library_name)
{
    const std::string profile;
    const uint64_t content_length =
        static_cast<uint64_t>(encodedStringSize(profile)) +
        static_cast<uint64_t>(encodedStringSize(library_name));
    writeRecordPrefix(kOpcodeHeader, content_length);
    writeString(profile);
    writeString(library_name);
    return static_cast<bool>(output_);
}

bool McapWriter::writeChannel(
    uint16_t channel_id,
    const std::string &topic,
    const std::string &message_encoding,
    const std::map<std::string, std::string> &metadata)
{
    const uint64_t content_length =
        sizeof(uint16_t) +
        sizeof(uint16_t) +
        static_cast<uint64_t>(encodedStringSize(topic)) +
        static_cast<uint64_t>(encodedStringSize(message_encoding)) +
        static_cast<uint64_t>(4 + encodedStringMapSize(metadata));
    writeRecordPrefix(kOpcodeChannel, content_length);
    writeUint16(channel_id);
    writeUint16(0);
    writeString(topic);
    writeString(message_encoding);
    writeUint32(encodedStringMapSize(metadata));
    for (const auto &[key, value] : metadata)
    {
        writeString(key);
        writeString(value);
    }
    return static_cast<bool>(output_);
}

bool McapWriter::writeMessage(
    uint16_t channel_id,
    uint32_t sequence,
    uint64_t log_time_ns,
    uint64_t publish_time_ns,
    const std::string &payload)
{
    std::string record;
    record.reserve(1 + 8 + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint64_t) * 2 + payload.size());
    const uint64_t content_length =
        sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint64_t) + sizeof(uint64_t) + payload.size();
    appendRecordPrefix(&record, kOpcodeMessage, content_length);
    appendUint16(&record, channel_id);
    appendUint32(&record, sequence);
    appendUint64(&record, log_time_ns);
    appendUint64(&record, publish_time_ns);
    appendBytes(&record, payload);

    if (!chunk_records_.empty() &&
        (chunk_records_.size() + record.size()) > target_chunk_size_bytes_)
    {
        if (!flushChunk())
        {
            return false;
        }
    }

    if (chunk_records_.empty())
    {
        chunk_start_time_ns_ = log_time_ns;
        chunk_end_time_ns_ = log_time_ns;
    }
    else
    {
        updateChunkTimeRange(log_time_ns);
    }
    chunk_records_ += record;
    return true;
}

bool McapWriter::flushChunk()
{
    if (!isOpen() || chunk_records_.empty())
    {
        return true;
    }

    const std::string compressed_records = compressChunkRecords(chunk_records_);
    const std::string compression = effective_compression_;
    const uint64_t content_length =
        sizeof(uint64_t) +
        sizeof(uint64_t) +
        sizeof(uint64_t) +
        sizeof(uint32_t) +
        static_cast<uint64_t>(encodedStringSize(compression)) +
        static_cast<uint64_t>(compressed_records.size());

    writeRecordPrefix(kOpcodeChunk, content_length);
    writeUint64(chunk_start_time_ns_);
    writeUint64(chunk_end_time_ns_);
    writeUint64(static_cast<uint64_t>(chunk_records_.size()));
    writeUint32(0);
    writeString(compression);
    writeBytes(compressed_records);

    chunk_records_.clear();
    chunk_start_time_ns_ = 0;
    chunk_end_time_ns_ = 0;
    return static_cast<bool>(output_);
}

bool McapWriter::writeDataEnd()
{
    writeRecordPrefix(kOpcodeDataEnd, sizeof(uint32_t));
    writeUint32(0);
    return static_cast<bool>(output_);
}

bool McapWriter::writeFooter()
{
    writeRecordPrefix(kOpcodeFooter, sizeof(uint64_t) + sizeof(uint64_t) + sizeof(uint32_t));
    writeUint64(0);
    writeUint64(0);
    writeUint32(0);
    return static_cast<bool>(output_);
}

std::string McapWriter::compressChunkRecords(const std::string &records)
{
    if (effective_compression_ == "none" || records.empty())
    {
        return records;
    }

    if (effective_compression_ == "zstd")
    {
        const size_t bound = ZSTD_compressBound(records.size());
        std::string output(bound, '\0');
        const size_t compressed_size = ZSTD_compress(
            output.data(),
            bound,
            records.data(),
            records.size(),
            1);
        if (ZSTD_isError(compressed_size))
        {
            throw std::runtime_error(
                std::string("zstd compression failed: ") +
                ZSTD_getErrorName(compressed_size));
        }
        output.resize(compressed_size);
        return output;
    }

    if (effective_compression_ == "lz4")
    {
        const size_t bound = LZ4F_compressFrameBound(records.size(), nullptr);
        if (bound == 0)
        {
            throw std::runtime_error("lz4 frame compression bound failure");
        }
        std::string output(bound, '\0');
        const size_t compressed_size = LZ4F_compressFrame(
            output.data(),
            output.size(),
            records.data(),
            records.size(),
            nullptr);
        if (LZ4F_isError(compressed_size))
        {
            throw std::runtime_error(
                std::string("lz4 frame compression failed: ") +
                LZ4F_getErrorName(compressed_size));
        }
        output.resize(compressed_size);
        return output;
    }

    return records;
}

void McapWriter::updateChunkTimeRange(uint64_t log_time_ns)
{
    chunk_start_time_ns_ = std::min(chunk_start_time_ns_, log_time_ns);
    chunk_end_time_ns_ = std::max(chunk_end_time_ns_, log_time_ns);
}

std::string McapWriter::normalizeCompression(const std::string &raw) const
{
    std::string normalized = raw;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    if (normalized == "zstd" || normalized == "lz4")
    {
        return normalized;
    }
    return "none";
}

void McapWriter::writeRecordPrefix(uint8_t opcode, uint64_t content_length)
{
    output_.put(static_cast<char>(opcode));
    writeUint64(content_length);
}

void McapWriter::appendRecordPrefix(std::string *out, uint8_t opcode, uint64_t content_length)
{
    if (!out)
    {
        return;
    }
    out->push_back(static_cast<char>(opcode));
    appendUint64(out, content_length);
}

void McapWriter::writeUint16(uint16_t value)
{
    const char bytes[2] = {
        static_cast<char>(value & 0xFF),
        static_cast<char>((value >> 8) & 0xFF),
    };
    output_.write(bytes, sizeof(bytes));
}

void McapWriter::writeUint32(uint32_t value)
{
    const char bytes[4] = {
        static_cast<char>(value & 0xFF),
        static_cast<char>((value >> 8) & 0xFF),
        static_cast<char>((value >> 16) & 0xFF),
        static_cast<char>((value >> 24) & 0xFF),
    };
    output_.write(bytes, sizeof(bytes));
}

void McapWriter::writeUint64(uint64_t value)
{
    const char bytes[8] = {
        static_cast<char>(value & 0xFF),
        static_cast<char>((value >> 8) & 0xFF),
        static_cast<char>((value >> 16) & 0xFF),
        static_cast<char>((value >> 24) & 0xFF),
        static_cast<char>((value >> 32) & 0xFF),
        static_cast<char>((value >> 40) & 0xFF),
        static_cast<char>((value >> 48) & 0xFF),
        static_cast<char>((value >> 56) & 0xFF),
    };
    output_.write(bytes, sizeof(bytes));
}

void McapWriter::writeString(const std::string &value)
{
    writeUint32(static_cast<uint32_t>(value.size()));
    writeBytes(value);
}

void McapWriter::writeBytes(const std::string &value)
{
    if (!value.empty())
    {
        output_.write(value.data(), static_cast<std::streamsize>(value.size()));
    }
}

void McapWriter::appendUint16(std::string *out, uint16_t value)
{
    if (!out)
    {
        return;
    }
    out->push_back(static_cast<char>(value & 0xFF));
    out->push_back(static_cast<char>((value >> 8) & 0xFF));
}

void McapWriter::appendUint32(std::string *out, uint32_t value)
{
    if (!out)
    {
        return;
    }
    out->push_back(static_cast<char>(value & 0xFF));
    out->push_back(static_cast<char>((value >> 8) & 0xFF));
    out->push_back(static_cast<char>((value >> 16) & 0xFF));
    out->push_back(static_cast<char>((value >> 24) & 0xFF));
}

void McapWriter::appendUint64(std::string *out, uint64_t value)
{
    if (!out)
    {
        return;
    }
    out->push_back(static_cast<char>(value & 0xFF));
    out->push_back(static_cast<char>((value >> 8) & 0xFF));
    out->push_back(static_cast<char>((value >> 16) & 0xFF));
    out->push_back(static_cast<char>((value >> 24) & 0xFF));
    out->push_back(static_cast<char>((value >> 32) & 0xFF));
    out->push_back(static_cast<char>((value >> 40) & 0xFF));
    out->push_back(static_cast<char>((value >> 48) & 0xFF));
    out->push_back(static_cast<char>((value >> 56) & 0xFF));
}

void McapWriter::appendString(std::string *out, const std::string &value)
{
    if (!out)
    {
        return;
    }
    appendUint32(out, static_cast<uint32_t>(value.size()));
    appendBytes(out, value);
}

void McapWriter::appendBytes(std::string *out, const std::string &value)
{
    if (!out || value.empty())
    {
        return;
    }
    out->append(value);
}

uint32_t McapWriter::encodedStringSize(const std::string &value) const
{
    return static_cast<uint32_t>(sizeof(uint32_t) + value.size());
}

uint32_t McapWriter::encodedStringMapSize(const std::map<std::string, std::string> &metadata) const
{
    uint32_t size = 0;
    for (const auto &[key, value] : metadata)
    {
        size += encodedStringSize(key);
        size += encodedStringSize(value);
    }
    return size;
}

uint16_t McapWriter::ensureJsonChannel(
    const std::string &topic,
    const std::map<std::string, std::string> &channel_metadata)
{
    const auto it = topic_to_channel_id_.find(topic);
    if (it != topic_to_channel_id_.end())
    {
        return it->second;
    }

    const uint16_t channel_id = next_channel_id_++;
    if (!writeChannel(channel_id, topic, "json", channel_metadata))
    {
        return 0;
    }
    topic_to_channel_id_[topic] = channel_id;
    channel_sequence_[channel_id] = 0;

    return channel_id;
}

} // namespace rl_master::logging
