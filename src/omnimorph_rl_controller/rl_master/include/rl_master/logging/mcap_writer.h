#ifndef RL_MASTER_LOGGING_MCAP_WRITER_H
#define RL_MASTER_LOGGING_MCAP_WRITER_H

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace rl_master::logging
{

class McapWriter
{
public:
    McapWriter() = default;
    ~McapWriter();

    bool open(const std::string &file_path, const std::string &library_name);
    bool isOpen() const;
    void configureChunking(const std::string &compression, size_t chunk_size_bytes);

    bool writeMetadata(const std::string &name, const std::map<std::string, std::string> &metadata);
    bool writeJsonMessage(
        const std::string &topic,
        const std::string &json_payload,
        uint64_t log_time_ns,
        uint64_t publish_time_ns,
        const std::map<std::string, std::string> &channel_metadata = {});

    void flush();
    void close();

    const std::string &filePath() const;
    const std::string &effectiveCompression() const;

private:
    static constexpr uint8_t kOpcodeHeader = 0x01;
    static constexpr uint8_t kOpcodeFooter = 0x02;
    static constexpr uint8_t kOpcodeChannel = 0x04;
    static constexpr uint8_t kOpcodeMessage = 0x05;
    static constexpr uint8_t kOpcodeChunk = 0x06;
    static constexpr uint8_t kOpcodeMetadata = 0x0C;
    static constexpr uint8_t kOpcodeDataEnd = 0x0F;

    bool writeHeader(const std::string &library_name);
    bool writeChannel(
        uint16_t channel_id,
        const std::string &topic,
        const std::string &message_encoding,
        const std::map<std::string, std::string> &metadata);
    bool writeMessage(uint16_t channel_id, uint32_t sequence, uint64_t log_time_ns, uint64_t publish_time_ns, const std::string &payload);
    bool flushChunk();
    bool writeDataEnd();
    bool writeFooter();
    std::string compressChunkRecords(const std::string &records);
    void updateChunkTimeRange(uint64_t log_time_ns);
    std::string normalizeCompression(const std::string &raw) const;

    void writeRecordPrefix(uint8_t opcode, uint64_t content_length);
    static void appendRecordPrefix(std::string *out, uint8_t opcode, uint64_t content_length);
    void writeUint16(uint16_t value);
    void writeUint32(uint32_t value);
    void writeUint64(uint64_t value);
    void writeString(const std::string &value);
    void writeBytes(const std::string &value);
    static void appendUint16(std::string *out, uint16_t value);
    static void appendUint32(std::string *out, uint32_t value);
    static void appendUint64(std::string *out, uint64_t value);
    static void appendString(std::string *out, const std::string &value);
    static void appendBytes(std::string *out, const std::string &value);
    uint32_t encodedStringSize(const std::string &value) const;
    uint32_t encodedStringMapSize(const std::map<std::string, std::string> &metadata) const;
    uint16_t ensureJsonChannel(const std::string &topic, const std::map<std::string, std::string> &channel_metadata);

    std::ofstream output_;
    std::string file_path_;
    bool open_ = false;
    uint16_t next_channel_id_ = 1;
    std::unordered_map<std::string, uint16_t> topic_to_channel_id_;
    std::unordered_map<uint16_t, uint32_t> channel_sequence_;
    std::string chunk_records_;
    uint64_t chunk_start_time_ns_ = 0;
    uint64_t chunk_end_time_ns_ = 0;
    size_t target_chunk_size_bytes_ = 1024 * 1024;
    std::string requested_compression_ = "none";
    std::string effective_compression_ = "none";
};

} // namespace rl_master::logging

#endif // RL_MASTER_LOGGING_MCAP_WRITER_H
