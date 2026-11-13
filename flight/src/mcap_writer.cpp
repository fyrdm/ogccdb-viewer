// MCAP writer: magic, Header, then the data section (Schema and Channel
// records at top level, messages in chunks, each chunk followed by its
// MessageIndex records), DataEnd, the summary (Schemas, Channels, Statistics,
// ChunkIndexes), SummaryOffsets, Footer and the magic again. All integers are
// little endian; CRCs are written as 0, which means "not computed".
#include <algorithm>
#include <limits>
#include <stdexcept>

#include "ogccdb/flight/bag_writer.hpp"

#ifdef OGCCDB_FLIGHT_HAVE_ZSTD
#include <zstd.h>
#endif
#ifdef OGCCDB_FLIGHT_HAVE_LZ4
#include <lz4frame.h>
#endif

namespace ogccdb::flight {
namespace {

constexpr char kMagic[8] = {'\x89', 'M', 'C', 'A', 'P', '0', '\r', '\n'};
enum Op : std::uint8_t {
    kHeader = 0x01,
    kFooter = 0x02,
    kSchema = 0x03,
    kChannel = 0x04,
    kMessage = 0x05,
    kChunk = 0x06,
    kMessageIndex = 0x07,
    kChunkIndex = 0x08,
    kStatistics = 0x0B,
    kSummaryOffset = 0x0E,
    kDataEnd = 0x0F,
};

template <typename T>
void num(std::string& s, T v) {
    for (std::size_t i = 0; i < sizeof(T); ++i) s += static_cast<char>((static_cast<std::uint64_t>(v) >> (8 * i)) & 0xFF);
}
void str(std::string& s, const std::string& v) {
    num<std::uint32_t>(s, static_cast<std::uint32_t>(v.size()));
    s += v;
}
std::string record(std::uint8_t op, const std::string& body) {
    std::string r;
    r += static_cast<char>(op);
    num<std::uint64_t>(r, body.size());
    r += body;
    return r;
}

std::string compress(const std::string& compression, const std::string& plain) {
    if (compression.empty()) return plain;
#ifdef OGCCDB_FLIGHT_HAVE_ZSTD
    if (compression == "zstd") {
        std::string out(ZSTD_compressBound(plain.size()), '\0');
        const std::size_t n = ZSTD_compress(out.data(), out.size(), plain.data(), plain.size(), 3);
        if (ZSTD_isError(n)) throw std::runtime_error("MCAP: zstd compression failed");
        out.resize(n);
        return out;
    }
#endif
#ifdef OGCCDB_FLIGHT_HAVE_LZ4
    if (compression == "lz4") {
        std::string out(LZ4F_compressFrameBound(plain.size(), nullptr), '\0');
        const std::size_t n = LZ4F_compressFrame(out.data(), out.size(), plain.data(), plain.size(), nullptr);
        if (LZ4F_isError(n)) throw std::runtime_error("MCAP: lz4 compression failed");
        out.resize(n);
        return out;
    }
#endif
    throw std::runtime_error("MCAP: compression '" + compression + "' is not supported in this build");
}

}  // namespace

McapWriterOptions::McapWriterOptions() {
#ifdef OGCCDB_FLIGHT_HAVE_ZSTD
    compression = "zstd";
#endif
}

bool McapWriter::supports(const std::string& compression) {
    if (compression.empty()) return true;
#ifdef OGCCDB_FLIGHT_HAVE_ZSTD
    if (compression == "zstd") return true;
#endif
#ifdef OGCCDB_FLIGHT_HAVE_LZ4
    if (compression == "lz4") return true;
#endif
    return false;
}

McapWriter::McapWriter(const std::filesystem::path& file, McapWriterOptions options)
    : path_(file), options_(std::move(options)) {
    if (!supports(options_.compression)) {
        throw std::runtime_error("MCAP: compression '" + options_.compression + "' is not supported in this build");
    }
    out_.open(file, std::ios::binary | std::ios::trunc);
    if (!out_) throw std::runtime_error("cannot create " + file.string());
    std::string magic(kMagic, 8);
    put(magic);
    std::string h;
    str(h, options_.profile);
    str(h, "ogccdb-flight");
    put(record(kHeader, h));
}

McapWriter::~McapWriter() {
    if (!closed_) {
        try {
            close();
        } catch (...) {
            // destructors must not throw; call close() to see errors
        }
    }
}

void McapWriter::put(const std::string& bytes) {
    out_.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    if (!out_) throw std::runtime_error("cannot write " + path_.string());
    position_ += bytes.size();
}

std::uint16_t McapWriter::add_topic(const BagTopic& topic) {
    if (closed_) throw std::runtime_error("MCAP: add_topic after close");
    std::uint16_t schema_id = 0;  // 0: no schema
    if (!topic.type.empty()) {
        for (const Schema& s : schemas_) {
            if (s.name == topic.type && s.encoding == topic.schema_encoding && s.data == topic.schema) schema_id = s.id;
        }
        if (schema_id == 0) {
            if (schemas_.size() >= std::numeric_limits<std::uint16_t>::max() - 1) throw std::runtime_error("MCAP: too many schemas");
            Schema s{static_cast<std::uint16_t>(schemas_.size() + 1), topic.type, topic.schema_encoding, topic.schema};
            std::string b;
            num<std::uint16_t>(b, s.id);
            str(b, s.name);
            str(b, s.encoding);
            str(b, s.data);
            put(record(kSchema, b));
            schema_id = s.id;
            schemas_.push_back(std::move(s));
        }
    }
    if (channels_.size() >= std::numeric_limits<std::uint16_t>::max()) throw std::runtime_error("MCAP: too many channels");
    Channel c;
    c.id = static_cast<std::uint16_t>(channels_.size());
    c.schema_id = schema_id;
    c.topic = topic.name;
    c.encoding = topic.encoding.empty() ? "cdr" : topic.encoding;
    c.metadata = topic.metadata;
    std::string b, m;
    num<std::uint16_t>(b, c.id);
    num<std::uint16_t>(b, c.schema_id);
    str(b, c.topic);
    str(b, c.encoding);
    for (const auto& [k, v] : c.metadata) {
        str(m, k);
        str(m, v);
    }
    str(b, m);  // Map<string, string>: byte length, then the pairs
    put(record(kChannel, b));
    channels_.push_back(std::move(c));
    return channels_.back().id;
}

std::vector<BagTopic> McapWriter::topics() const {
    std::vector<BagTopic> out;
    for (const Channel& c : channels_) {
        BagTopic t;
        t.name = c.topic;
        t.encoding = c.encoding;
        t.count = c.count;
        t.metadata = c.metadata;
        for (const Schema& s : schemas_) {
            if (s.id == c.schema_id) {
                t.type = s.name;
                t.schema_encoding = s.encoding;
            }
        }
        out.push_back(std::move(t));
    }
    return out;
}

void McapWriter::write(std::uint16_t channel, std::int64_t log_time_ns, const std::uint8_t* data, std::size_t size,
                       std::int64_t publish_time_ns) {
    if (closed_) throw std::runtime_error("MCAP: write after close");
    if (channel >= channels_.size()) throw std::runtime_error("MCAP: unknown channel");
    Channel& c = channels_[channel];
    const auto t = static_cast<std::uint64_t>(std::max<std::int64_t>(log_time_ns, 0));
    const auto pt = publish_time_ns < 0 ? t : static_cast<std::uint64_t>(publish_time_ns);
    std::string b;
    num<std::uint16_t>(b, c.id);
    num<std::uint32_t>(b, c.sequence++);
    num<std::uint64_t>(b, t);
    num<std::uint64_t>(b, pt);
    b.append(reinterpret_cast<const char*>(data), size);
    if (chunk_.empty()) chunk_start_ = chunk_end_ = t;
    chunk_start_ = std::min(chunk_start_, t);
    chunk_end_ = std::max(chunk_end_, t);
    chunk_messages_[c.id].emplace_back(t, chunk_.size());
    chunk_ += record(kMessage, b);
    ++c.count;
    if (message_count_ == 0) start_time_ = end_time_ = t;
    start_time_ = std::min(start_time_, t);
    end_time_ = std::max(end_time_, t);
    ++message_count_;
    if (chunk_.size() >= options_.chunk_size) flush_chunk();
}

void McapWriter::flush_chunk() {
    if (chunk_.empty()) return;
    const std::string body_records = compress(options_.compression, chunk_);
    std::string b;
    num<std::uint64_t>(b, chunk_start_);
    num<std::uint64_t>(b, chunk_end_);
    num<std::uint64_t>(b, chunk_.size());
    num<std::uint32_t>(b, 0);  // uncompressed CRC: not computed
    str(b, options_.compression);
    num<std::uint64_t>(b, body_records.size());
    b += body_records;
    ChunkIndex ci;
    ci.start_time = chunk_start_;
    ci.end_time = chunk_end_;
    ci.offset = position();
    const std::string chunk_record = record(kChunk, b);
    ci.length = chunk_record.size();
    ci.compressed_size = body_records.size();
    ci.uncompressed_size = chunk_.size();
    put(chunk_record);
    const std::uint64_t index_start = position();
    for (const auto& [channel, entries] : chunk_messages_) {
        std::string m, e;
        num<std::uint16_t>(m, channel);
        for (const auto& [time, offset] : entries) {
            num<std::uint64_t>(e, time);
            num<std::uint64_t>(e, offset);
        }
        str(m, e);  // Array: byte length, then (log time, offset) pairs
        ci.index_offsets[channel] = position();
        put(record(kMessageIndex, m));
    }
    ci.index_length = position() - index_start;
    chunk_indexes_.push_back(std::move(ci));
    chunk_.clear();
    chunk_messages_.clear();
}

void McapWriter::close() {
    if (closed_) return;
    closed_ = true;
    flush_chunk();
    std::string de;
    num<std::uint32_t>(de, 0);  // data section CRC: not computed
    put(record(kDataEnd, de));

    // Summary: each group of records, remembered for the SummaryOffsets.
    const std::uint64_t summary_start = position();
    std::vector<std::pair<std::uint8_t, std::pair<std::uint64_t, std::uint64_t>>> groups;
    auto group = [&](std::uint8_t op, const std::vector<std::string>& bodies) {
        if (bodies.empty()) return;
        const std::uint64_t start = position();
        for (const auto& body : bodies) put(record(op, body));
        groups.push_back({op, {start, position() - start}});
    };
    std::vector<std::string> bodies;
    for (const Schema& s : schemas_) {
        std::string b;
        num<std::uint16_t>(b, s.id);
        str(b, s.name);
        str(b, s.encoding);
        str(b, s.data);
        bodies.push_back(std::move(b));
    }
    group(kSchema, bodies);
    bodies.clear();
    for (const Channel& c : channels_) {
        std::string b, m;
        num<std::uint16_t>(b, c.id);
        num<std::uint16_t>(b, c.schema_id);
        str(b, c.topic);
        str(b, c.encoding);
        for (const auto& [k, v] : c.metadata) {
            str(m, k);
            str(m, v);
        }
        str(b, m);
        bodies.push_back(std::move(b));
    }
    group(kChannel, bodies);
    bodies.clear();
    {
        std::string b, counts;
        num<std::uint64_t>(b, message_count_);
        num<std::uint16_t>(b, static_cast<std::uint16_t>(schemas_.size()));
        num<std::uint32_t>(b, static_cast<std::uint32_t>(channels_.size()));
        num<std::uint32_t>(b, 0);  // attachments
        num<std::uint32_t>(b, 0);  // metadata records
        num<std::uint32_t>(b, static_cast<std::uint32_t>(chunk_indexes_.size()));
        num<std::uint64_t>(b, start_time_);
        num<std::uint64_t>(b, end_time_);
        for (const Channel& c : channels_) {
            num<std::uint16_t>(counts, c.id);
            num<std::uint64_t>(counts, c.count);
        }
        str(b, counts);
        bodies.push_back(std::move(b));
    }
    group(kStatistics, bodies);
    bodies.clear();
    for (const ChunkIndex& ci : chunk_indexes_) {
        std::string b, offsets;
        num<std::uint64_t>(b, ci.start_time);
        num<std::uint64_t>(b, ci.end_time);
        num<std::uint64_t>(b, ci.offset);
        num<std::uint64_t>(b, ci.length);
        for (const auto& [channel, offset] : ci.index_offsets) {
            num<std::uint16_t>(offsets, channel);
            num<std::uint64_t>(offsets, offset);
        }
        str(b, offsets);
        num<std::uint64_t>(b, ci.index_length);
        str(b, options_.compression);
        num<std::uint64_t>(b, ci.compressed_size);
        num<std::uint64_t>(b, ci.uncompressed_size);
        bodies.push_back(std::move(b));
    }
    group(kChunkIndex, bodies);

    const std::uint64_t offsets_start = position();
    for (const auto& [op, range] : groups) {
        std::string b;
        num<std::uint8_t>(b, op);
        num<std::uint64_t>(b, range.first);
        num<std::uint64_t>(b, range.second);
        put(record(kSummaryOffset, b));
    }
    std::string f;
    num<std::uint64_t>(f, summary_start);
    num<std::uint64_t>(f, offsets_start);
    num<std::uint32_t>(f, 0);  // summary CRC: not computed
    put(record(kFooter, f));
    put(std::string(kMagic, 8));
    out_.close();
    if (!out_) throw std::runtime_error("cannot write " + path_.string());
}

}  // namespace ogccdb::flight
