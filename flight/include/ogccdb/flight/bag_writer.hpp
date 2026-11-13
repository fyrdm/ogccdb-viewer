#pragma once
// Writes ROS 2 bags as single MCAP files (the rosbag2 default storage since
// ROS 2 Iron): chunked, optionally compressed, with message indexes and a
// summary, so indexed readers and players can seek in them.
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "ogccdb/flight/bag.hpp"

namespace ogccdb::flight {

struct McapWriterOptions {
    std::string profile = "ros2";
    /// Chunk compression: "zstd", "lz4" or "" (none). zstd when this build has it.
    std::string compression;
    /// Uncompressed bytes collected in a chunk before it is written.
    std::size_t chunk_size = std::size_t{4} << 20;

    McapWriterOptions();
};

class McapWriter {
public:
    /// Creates (or replaces) `file`; throws std::runtime_error.
    explicit McapWriter(const std::filesystem::path& file, McapWriterOptions options = {});
    /// Closes the file if close() was not called (errors are then ignored).
    ~McapWriter();
    McapWriter(const McapWriter&) = delete;
    McapWriter& operator=(const McapWriter&) = delete;

    /// Adds a channel for `topic` (its name, type, encoding, schema and
    /// metadata; the count is ignored) and returns its id for write().
    /// Topics of the same type and definition share a schema.
    std::uint16_t add_topic(const BagTopic& topic);
    /// Writes a message; `publish_time_ns` < 0 means the log time.
    void write(std::uint16_t channel, std::int64_t log_time_ns, const std::uint8_t* data, std::size_t size,
               std::int64_t publish_time_ns = -1);
    /// Writes the last chunk, the indexes, the summary and the footer.
    void close();

    std::uint64_t message_count() const { return message_count_; }
    /// The topics added so far, in channel order, each with the number of
    /// messages written to it (without its schema text).
    std::vector<BagTopic> topics() const;
    /// Log times of the first and last message written; 0 if none.
    std::int64_t start_ns() const { return static_cast<std::int64_t>(start_time_); }
    std::int64_t end_ns() const { return static_cast<std::int64_t>(end_time_); }
    /// True if this build can write chunks compressed with `compression`.
    static bool supports(const std::string& compression);

private:
    struct Schema {
        std::uint16_t id;
        std::string name, encoding, data;
    };
    struct Channel {
        std::uint16_t id, schema_id;
        std::string topic, encoding;
        std::vector<std::pair<std::string, std::string>> metadata;
        std::uint32_t sequence = 0;
        std::uint64_t count = 0;
    };
    struct ChunkIndex {
        std::uint64_t start_time, end_time, offset, length;
        std::map<std::uint16_t, std::uint64_t> index_offsets;
        std::uint64_t index_length;
        std::uint64_t compressed_size, uncompressed_size;
    };

    void put(const std::string& record);
    void flush_chunk();
    std::uint64_t position() const { return position_; }

    std::ofstream out_;
    std::filesystem::path path_;
    McapWriterOptions options_;
    std::uint64_t position_ = 0;
    std::vector<Schema> schemas_;
    std::vector<Channel> channels_;
    // The chunk being collected: its records, time range and message offsets.
    std::string chunk_;
    std::uint64_t chunk_start_ = 0, chunk_end_ = 0;
    std::map<std::uint16_t, std::vector<std::pair<std::uint64_t, std::uint64_t>>> chunk_messages_;
    std::vector<ChunkIndex> chunk_indexes_;
    std::uint64_t message_count_ = 0, start_time_ = 0, end_time_ = 0;
    bool closed_ = false;
};

/// Writes `folder`/metadata.yaml (rosbag2 metadata version 9) for a bag of one
/// storage file `storage_file` inside it, so that ROS 2 tools open the folder
/// as a bag. `topics` carry their message counts and metadata
/// (offered_qos_profiles in the words form, topic_type_hash); `ros_distro` is
/// the distribution the bag is for, e.g. "jazzy". Throws std::runtime_error.
void write_bag_metadata(const std::filesystem::path& folder, const std::string& storage_file,
                        const std::vector<BagTopic>& topics, std::int64_t start_ns, std::int64_t end_ns,
                        const std::string& ros_distro);

}  // namespace ogccdb::flight
