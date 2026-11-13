#pragma once
// Reads ROS 2 bags without ROS: a rosbag2 folder (metadata.yaml and its
// storage files) or a single storage file, in MCAP or SQLite (.db3) storage.
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace ogccdb::flight {

struct BagTopic {
    std::string name;      ///< e.g. /mavros/global_position/global
    std::string type;      ///< e.g. sensor_msgs/msg/NavSatFix
    std::string encoding;  ///< message encoding, "cdr" for ROS 2
    std::uint64_t count = 0;
    /// Message definition as the storage holds it (MCAP schema: encoding
    /// "ros2msg" and the .msg text with its dependencies); empty if it has none.
    std::string schema_encoding;
    std::string schema;
    /// Key-value metadata of the topic, e.g. offered_qos_profiles (YAML).
    std::vector<std::pair<std::string, std::string>> metadata;
};

struct BagMessage {
    const BagTopic* topic = nullptr;
    std::int64_t log_time_ns = 0;  ///< when the recorder received it
    const std::uint8_t* data = nullptr;
    std::size_t size = 0;
    std::int64_t publish_time_ns = 0;  ///< when it was sent, if stored; else log_time_ns
};

class Bag {
public:
    /// Opens a bag folder or storage file; throws std::runtime_error.
    static std::unique_ptr<Bag> open(const std::filesystem::path& path);
    virtual ~Bag() = default;

    const std::filesystem::path& path() const { return path_; }
    /// "MCAP" or "SQLite".
    virtual std::string storage() const = 0;
    const std::vector<BagTopic>& topics() const { return topics_; }
    const BagTopic* topic(const std::string& name) const;
    std::int64_t start_ns() const { return start_ns_; }
    std::int64_t end_ns() const { return end_ns_; }
    std::uint64_t message_count() const;

    /// Calls `fn` for every message of the topics `want` accepts (all if
    /// empty), in storage order (log time order for a recorder's bag).
    virtual void read(const std::function<void(const BagMessage&)>& fn,
                      const std::function<bool(const BagTopic&)>& want = {}) = 0;

protected:
    std::filesystem::path path_;
    std::vector<BagTopic> topics_;
    std::int64_t start_ns_ = 0, end_ns_ = 0;
};

/// True if `path` is a bag Bag::open can read: a folder with metadata.yaml or
/// storage files, a .mcap or .db3 file, or an uncompressed .tar holding a bag.
bool is_bag(const std::filesystem::path& path);

}  // namespace ogccdb::flight
