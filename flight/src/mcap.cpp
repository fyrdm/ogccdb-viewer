// MCAP storage (the rosbag2 default since ROS 2 Iron): a sequence of
// records, opcode (1 byte) + length (8 bytes) + body, little endian.
// Messages sit at top level or inside chunks, which may be compressed.
#include <cstring>
#include <fstream>
#include <map>
#include <stdexcept>

#include "ogccdb/flight/bag.hpp"

#ifdef OGCCDB_FLIGHT_HAVE_ZSTD
#include <zstd.h>
#endif
#ifdef OGCCDB_FLIGHT_HAVE_LZ4
#include <lz4frame.h>
#endif

namespace ogccdb::flight {
namespace {

constexpr unsigned char kMagic[8] = {0x89, 'M', 'C', 'A', 'P', '0', '\r', '\n'};
enum Op : std::uint8_t { kHeader = 0x01, kFooter = 0x02, kSchema = 0x03, kChannel = 0x04, kMessage = 0x05, kChunk = 0x06 };

/// Cursor over a record body.
struct Cursor {
    const std::uint8_t* p;
    const std::uint8_t* end;
    void need(std::size_t n) const {
        if (static_cast<std::size_t>(end - p) < n) throw std::runtime_error("MCAP record ends early");
    }
    template <typename T>
    T num() {
        need(sizeof(T));
        T v = 0;
        for (std::size_t i = 0; i < sizeof(T); ++i) v |= static_cast<T>(static_cast<T>(p[i]) << (8 * i));
        p += sizeof(T);
        return v;
    }
    std::string str() {
        const auto n = num<std::uint32_t>();
        need(n);
        std::string s(reinterpret_cast<const char*>(p), n);
        p += n;
        return s;
    }
};

std::vector<std::uint8_t> decompress(const std::string& compression, const std::uint8_t* src, std::size_t n,
                                     std::size_t uncompressed) {
    std::vector<std::uint8_t> out(uncompressed);
    if (compression == "zstd") {
#ifdef OGCCDB_FLIGHT_HAVE_ZSTD
        const std::size_t got = ZSTD_decompress(out.data(), out.size(), src, n);
        if (ZSTD_isError(got) || got != uncompressed) throw std::runtime_error("MCAP: bad zstd chunk");
        return out;
#endif
    } else if (compression == "lz4") {
#ifdef OGCCDB_FLIGHT_HAVE_LZ4
        LZ4F_dctx* ctx = nullptr;
        LZ4F_createDecompressionContext(&ctx, LZ4F_VERSION);
        std::size_t dst = out.size(), srcn = n;
        const std::size_t rc = LZ4F_decompress(ctx, out.data(), &dst, src, &srcn, nullptr);
        LZ4F_freeDecompressionContext(ctx);
        if (LZ4F_isError(rc) || dst != uncompressed) throw std::runtime_error("MCAP: bad lz4 chunk");
        return out;
#endif
    }
    throw std::runtime_error("MCAP: chunk compression '" + compression + "' is not supported in this build");
}

class McapBag : public Bag {
public:
    McapBag(std::vector<std::filesystem::path> files, std::filesystem::path path) : files_(std::move(files)) {
        path_ = std::move(path);
        bool first = true;
        pass([&](std::size_t topic, std::int64_t t, const std::uint8_t*, std::size_t) {
            ++topics_[topic].count;
            if (first || t < start_ns_) start_ns_ = t;
            if (first || t > end_ns_) end_ns_ = t;
            first = false;
        });
    }

    std::string storage() const override { return "MCAP"; }

    void read(const std::function<void(const BagMessage&)>& fn,
              const std::function<bool(const BagTopic&)>& want) override {
        std::vector<char> wanted(topics_.size());
        for (std::size_t i = 0; i < topics_.size(); ++i) wanted[i] = !want || want(topics_[i]);
        pass([&](std::size_t topic, std::int64_t t, const std::uint8_t* d, std::size_t n) {
            if (topic < wanted.size() && wanted[topic]) fn(BagMessage{&topics_[topic], t, d, n});
        });
    }

private:
    using Sink = std::function<void(std::size_t topic, std::int64_t t, const std::uint8_t*, std::size_t)>;

    struct FileState {
        std::map<std::uint16_t, std::string> schema_name;
        std::map<std::uint16_t, std::size_t> channel_topic;  // channel id -> index in topics_
    };

    std::size_t topic_index(const std::string& name, const std::string& type, const std::string& encoding) {
        for (std::size_t i = 0; i < topics_.size(); ++i) {
            if (topics_[i].name == name) return i;
        }
        topics_.push_back(BagTopic{name, type, encoding, 0});
        return topics_.size() - 1;
    }

    void record(FileState& fs, std::uint8_t op, const std::uint8_t* body, std::size_t len, const Sink& sink) {
        Cursor c{body, body + len};
        switch (op) {
            case kSchema: {
                const auto id = c.num<std::uint16_t>();
                fs.schema_name[id] = c.str();
                break;
            }
            case kChannel: {
                const auto id = c.num<std::uint16_t>();
                const auto schema = c.num<std::uint16_t>();
                const std::string topic = c.str();
                const std::string encoding = c.str();
                const auto it = fs.schema_name.find(schema);
                fs.channel_topic[id] = topic_index(topic, it != fs.schema_name.end() ? it->second : "", encoding);
                break;
            }
            case kMessage: {
                const auto channel = c.num<std::uint16_t>();
                c.num<std::uint32_t>();  // sequence
                const auto log_time = static_cast<std::int64_t>(c.num<std::uint64_t>());
                c.num<std::uint64_t>();  // publish time
                const auto it = fs.channel_topic.find(channel);
                if (it != fs.channel_topic.end()) sink(it->second, log_time, c.p, static_cast<std::size_t>(c.end - c.p));
                break;
            }
            case kChunk: {
                c.num<std::uint64_t>();  // message start time
                c.num<std::uint64_t>();  // message end time
                const auto usize = c.num<std::uint64_t>();
                c.num<std::uint32_t>();  // crc
                const std::string compression = c.str();
                const auto n = c.num<std::uint64_t>();
                c.need(n);
                std::vector<std::uint8_t> plain;
                const std::uint8_t* rec = c.p;
                std::size_t rec_len = n;
                if (!compression.empty()) {
                    plain = decompress(compression, c.p, n, usize);
                    rec = plain.data();
                    rec_len = plain.size();
                }
                Cursor r{rec, rec + rec_len};
                while (r.end - r.p >= 9) {
                    const std::uint8_t sub = *r.p++;
                    const auto sublen = r.num<std::uint64_t>();
                    r.need(sublen);
                    record(fs, sub, r.p, sublen, sink);
                    r.p += sublen;
                }
                break;
            }
            default:
                break;  // indexes, statistics, attachments, metadata: not needed
        }
    }

    void pass(const Sink& sink) {
        for (const auto& file : files_) {
            std::ifstream in(file, std::ios::binary);
            if (!in) throw std::runtime_error("cannot open " + file.string());
            unsigned char magic[8];
            if (!in.read(reinterpret_cast<char*>(magic), 8) || std::memcmp(magic, kMagic, 8) != 0) {
                throw std::runtime_error(file.string() + " is not an MCAP file");
            }
            FileState fs;
            std::vector<std::uint8_t> body;
            for (;;) {
                unsigned char head[9];
                if (!in.read(reinterpret_cast<char*>(head), 9)) break;  // end of a truncated file
                std::uint64_t len = 0;
                for (int i = 0; i < 8; ++i) len |= static_cast<std::uint64_t>(head[1 + i]) << (8 * i);
                if (head[0] == kFooter) break;
                if (head[0] == kHeader || head[0] == kSchema || head[0] == kChannel || head[0] == kMessage ||
                    head[0] == kChunk) {
                    body.resize(len);
                    if (!in.read(reinterpret_cast<char*>(body.data()), static_cast<std::streamsize>(len))) break;
                    record(fs, head[0], body.data(), body.size(), sink);
                } else {
                    in.seekg(static_cast<std::streamoff>(len), std::ios::cur);
                }
            }
        }
    }

    std::vector<std::filesystem::path> files_;
};

}  // namespace

std::unique_ptr<Bag> open_mcap(const std::vector<std::filesystem::path>& files, const std::filesystem::path& path) {
    return std::make_unique<McapBag>(files, path);
}

}  // namespace ogccdb::flight
