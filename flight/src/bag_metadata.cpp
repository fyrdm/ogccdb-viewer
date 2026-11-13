// metadata.yaml of a rosbag2 folder, in the layout of metadata version 9
// (offered QoS profiles as nested YAML with policy names, a type hash per
// topic, the distribution's name).
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "ogccdb/flight/bag_writer.hpp"

namespace ogccdb::flight {
namespace {

std::string value_of(const BagTopic& t, const std::string& key) {
    for (const auto& [k, v] : t.metadata) {
        if (k == key) return v;
    }
    return {};
}

/// A YAML double-quoted scalar.
std::string quoted(const std::string& s) {
    std::string q = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') q += '\\';
        q += c;
    }
    return q + "\"";
}

}  // namespace

void write_bag_metadata(const std::filesystem::path& folder, const std::string& storage_file,
                        const std::vector<BagTopic>& topics, std::int64_t start_ns, std::int64_t end_ns,
                        const std::string& ros_distro) {
    std::uint64_t total = 0;
    for (const BagTopic& t : topics) total += t.count;
    std::ostringstream y;
    y << "rosbag2_bagfile_information:\n"
      << "  version: 9\n"
      << "  storage_identifier: mcap\n"
      << "  duration:\n    nanoseconds: " << (end_ns - start_ns) << "\n"
      << "  starting_time:\n    nanoseconds_since_epoch: " << start_ns << "\n"
      << "  message_count: " << total << "\n"
      << "  topics_with_message_count:\n";
    for (const BagTopic& t : topics) {
        y << "    - topic_metadata:\n"
          << "        name: " << t.name << "\n"
          << "        type: " << t.type << "\n"
          << "        serialization_format: " << (t.encoding.empty() ? "cdr" : t.encoding) << "\n";
        // The profiles are YAML already (a sequence): nest them as they are.
        const std::string qos = value_of(t, "offered_qos_profiles");
        if (qos.rfind("- ", 0) == 0) {
            y << "        offered_qos_profiles:\n";
            std::istringstream lines(qos);
            for (std::string line; std::getline(lines, line);) y << "          " << line << "\n";
        } else {
            y << "        offered_qos_profiles:\n          []\n";
        }
        y << "        type_description_hash: " << quoted(value_of(t, "topic_type_hash")) << "\n"
          << "      message_count: " << t.count << "\n";
    }
    y << "  compression_format: \"\"\n"
      << "  compression_mode: \"\"\n"
      << "  relative_file_paths:\n    - " << storage_file << "\n"
      << "  files:\n"
      << "    - path: " << storage_file << "\n"
      << "      starting_time:\n        nanoseconds_since_epoch: " << start_ns << "\n"
      << "      duration:\n        nanoseconds: " << (end_ns - start_ns) << "\n"
      << "      message_count: " << total << "\n"
      << "  custom_data: ~\n"
      << "  ros_distro: " << ros_distro << "\n";
    const std::filesystem::path file = folder / "metadata.yaml";
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out << y.str();
    out.close();
    if (!out) throw std::runtime_error("cannot write " + file.string());
}

}  // namespace ogccdb::flight
