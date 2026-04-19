#!/usr/bin/env python3
"""Helpers for reading rl_master runtime MCAP logs.

Supported in-repo MCAP subset:
- Header
- Channel
- Message
- Chunk (none / zstd / lz4)
- Metadata
- DataEnd
- Footer
"""

from __future__ import annotations

import json
import struct
from pathlib import Path
from typing import Dict, Iterable, Iterator, List, Optional

import lz4.frame
import zstandard

MAGIC = bytes([0x89, ord("M"), ord("C"), ord("A"), ord("P"), ord("0"), 0x0D, 0x0A])

OP_HEADER = 0x01
OP_FOOTER = 0x02
OP_CHANNEL = 0x04
OP_MESSAGE = 0x05
OP_CHUNK = 0x06
OP_METADATA = 0x0C
OP_DATA_END = 0x0F


class McapParseError(RuntimeError):
    pass


class _Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0

    def remaining(self) -> int:
        return len(self.data) - self.pos

    def read(self, n: int) -> bytes:
        if self.pos + n > len(self.data):
            raise McapParseError("Unexpected EOF while parsing MCAP record")
        out = self.data[self.pos : self.pos + n]
        self.pos += n
        return out

    def read_u1(self) -> int:
        return self.read(1)[0]

    def read_u2(self) -> int:
        return struct.unpack("<H", self.read(2))[0]

    def read_u4(self) -> int:
        return struct.unpack("<I", self.read(4))[0]

    def read_u8(self) -> int:
        return struct.unpack("<Q", self.read(8))[0]

    def read_string(self) -> str:
        length = self.read_u4()
        return self.read(length).decode("utf-8")

    def read_string_map(self) -> Dict[str, str]:
        length = self.read_u4()
        end = self.pos + length
        out: Dict[str, str] = {}
        while self.pos < end:
            key = self.read_string()
            value = self.read_string()
            out[key] = value
        if self.pos != end:
            raise McapParseError("Malformed MCAP string map length")
        return out


def _parse_channel(payload: bytes) -> Dict:
    reader = _Reader(payload)
    channel_id = reader.read_u2()
    schema_id = reader.read_u2()
    topic = reader.read_string()
    message_encoding = reader.read_string()
    metadata = reader.read_string_map()
    return {
        "id": channel_id,
        "schema_id": schema_id,
        "topic": topic,
        "message_encoding": message_encoding,
        "metadata": metadata,
    }


def _parse_message(payload: bytes) -> Dict:
    reader = _Reader(payload)
    channel_id = reader.read_u2()
    sequence = reader.read_u4()
    log_time_ns = reader.read_u8()
    publish_time_ns = reader.read_u8()
    data = reader.read(reader.remaining())
    return {
        "channel_id": channel_id,
        "sequence": sequence,
        "log_time_ns": log_time_ns,
        "publish_time_ns": publish_time_ns,
        "data": data,
    }


def _parse_metadata(payload: bytes) -> Dict:
    reader = _Reader(payload)
    name = reader.read_string()
    metadata = reader.read_string_map()
    return {"name": name, "metadata": metadata}


def _decompress_chunk(compression: str, payload: bytes, uncompressed_size: int) -> bytes:
    compression = compression.strip().lower()
    if compression in {"", "none"}:
        return payload
    if compression == "zstd":
        data = zstandard.ZstdDecompressor().decompress(payload)
    elif compression == "lz4":
        data = lz4.frame.decompress(payload)
    else:
        raise McapParseError(f"Unsupported MCAP chunk compression: {compression}")
    if uncompressed_size > 0 and len(data) != uncompressed_size:
        raise McapParseError(
            f"Chunk decompressed size mismatch: expected {uncompressed_size}, got {len(data)}"
        )
    return data


def _iter_embedded_records(payload: bytes) -> Iterator[Dict]:
    reader = _Reader(payload)
    while reader.remaining() > 0:
        opcode = reader.read_u1()
        content_length = reader.read_u8()
        record_payload = reader.read(content_length)
        yield {"opcode": opcode, "payload": record_payload}


def _decode_message_record(message: Dict, channel_map: Dict[int, Dict]) -> Dict:
    channel = channel_map.get(message["channel_id"], {})
    data_bytes = message["data"]
    text = data_bytes.decode("utf-8") if data_bytes else ""
    parsed = None
    try:
        parsed = json.loads(text) if text else None
    except json.JSONDecodeError:
        parsed = text
    return {
        "type": "message",
        "topic": channel.get("topic", f"channel_{message['channel_id']}"),
        "message_encoding": channel.get("message_encoding", ""),
        "channel_metadata": channel.get("metadata", {}),
        "channel_id": message["channel_id"],
        "sequence": message["sequence"],
        "log_time_ns": message["log_time_ns"],
        "publish_time_ns": message["publish_time_ns"],
        "data": parsed,
        "raw_text": text,
    }


def iter_runtime_records(path: Path) -> Iterator[Dict]:
    with path.open("rb") as f:
        if f.read(len(MAGIC)) != MAGIC:
            raise McapParseError(f"Not an MCAP file or unsupported magic: {path}")

        channel_map: Dict[int, Dict] = {}

        while True:
            opcode_raw = f.read(1)
            if not opcode_raw:
                break
            if opcode_raw == MAGIC[:1]:
                trailer = opcode_raw + f.read(len(MAGIC) - 1)
                if trailer != MAGIC:
                    raise McapParseError("Trailing MCAP magic mismatch")
                break

            opcode = opcode_raw[0]
            length_raw = f.read(8)
            if len(length_raw) != 8:
                raise McapParseError("Truncated MCAP record length")
            content_length = struct.unpack("<Q", length_raw)[0]
            payload = f.read(content_length)
            if len(payload) != content_length:
                raise McapParseError("Truncated MCAP record payload")

            if opcode in {OP_HEADER, OP_FOOTER, OP_DATA_END}:
                continue
            if opcode == OP_CHANNEL:
                channel = _parse_channel(payload)
                channel_map[channel["id"]] = channel
                yield {"type": "channel", **channel}
                continue
            if opcode == OP_METADATA:
                metadata = _parse_metadata(payload)
                yield {"type": "metadata", **metadata}
                continue
            if opcode == OP_MESSAGE:
                yield _decode_message_record(_parse_message(payload), channel_map)
                continue
            if opcode == OP_CHUNK:
                reader = _Reader(payload)
                message_start_time = reader.read_u8()
                message_end_time = reader.read_u8()
                uncompressed_size = reader.read_u8()
                _uncompressed_crc = reader.read_u4()
                compression = reader.read_string()
                compressed_records = reader.read(reader.remaining())
                records_payload = _decompress_chunk(compression, compressed_records, uncompressed_size)
                yield {
                    "type": "chunk",
                    "compression": compression,
                    "message_start_time": message_start_time,
                    "message_end_time": message_end_time,
                }
                for record in _iter_embedded_records(records_payload):
                    if record["opcode"] == OP_MESSAGE:
                        yield _decode_message_record(_parse_message(record["payload"]), channel_map)
                    elif record["opcode"] == OP_CHANNEL:
                        channel = _parse_channel(record["payload"])
                        channel_map[channel["id"]] = channel
                        yield {"type": "channel", **channel}
                    elif record["opcode"] == OP_METADATA:
                        metadata = _parse_metadata(record["payload"])
                        yield {"type": "metadata", **metadata}
                continue


def load_runtime_messages(path: Path, topic: Optional[str] = None) -> List[Dict]:
    out: List[Dict] = []
    for record in iter_runtime_records(path):
        if record["type"] != "message":
            continue
        if topic and record.get("topic") != topic:
            continue
        out.append(record)
    return out


def load_runtime_metadata(path: Path) -> Dict[str, Dict[str, str]]:
    metadata: Dict[str, Dict[str, str]] = {}
    for record in iter_runtime_records(path):
        if record["type"] == "metadata":
            metadata[record["name"]] = record["metadata"]
    return metadata


def flatten_message_rows(messages: Iterable[Dict]) -> List[Dict]:
    rows: List[Dict] = []
    for message in messages:
        payload = message.get("data")
        row: Dict[str, object] = {
            "topic": message.get("topic", ""),
            "sequence": message.get("sequence", 0),
            "log_time_ns": message.get("log_time_ns", 0),
            "publish_time_ns": message.get("publish_time_ns", 0),
        }
        if isinstance(payload, dict):
            for key, value in payload.items():
                if isinstance(value, list) and all(isinstance(v, (int, float, bool)) for v in value):
                    for idx, item in enumerate(value):
                        row[f"{key}__{idx}"] = item
                elif isinstance(value, (int, float, str, bool)) or value is None:
                    row[key] = value
                else:
                    row[key] = json.dumps(value, ensure_ascii=False)
        else:
            row["payload"] = payload
        rows.append(row)
    return rows
