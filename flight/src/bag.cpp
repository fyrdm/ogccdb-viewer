#include "ogccdb/flight/bag.hpp"

#include <algorithm>
#include <stdexcept>

namespace ogccdb::flight {

std::unique_ptr<Bag> open_mcap(const std::vector<std::filesystem::path>& files, const std::filesystem::path& path);
std::unique_ptr<Bag> open_sqlite(const std::vector<std::filesystem::path>& files, const std::filesystem::path& path);

namespace {

std::string lower_ext(const std::filesystem::path& p) {
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return e;
}

/// Storage files of a bag folder, in name order (rosbag2 names them _0, _1, ...).
std::vector<std::filesystem::path> storage_files(const std::filesystem::path& dir, const std::string& ext) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        if (e.is_regular_file(ec) && lower_ext(e.path()) == ext) files.push_back(e.path());
    }
    std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) {
        const auto sa = a.filename().string(), sb = b.filename().string();
        return sa.size() != sb.size() ? sa.size() < sb.size() : sa < sb;  // _2 before _10
    });
    return files;
}

}  // namespace

const BagTopic* Bag::topic(const std::string& name) const {
    for (const BagTopic& t : topics_) {
        if (t.name == name) return &t;
    }
    return nullptr;
}

std::uint64_t Bag::message_count() const {
    std::uint64_t n = 0;
    for (const BagTopic& t : topics_) n += t.count;
    return n;
}

std::unique_ptr<Bag> Bag::open(const std::filesystem::path& path) {
    std::error_code ec;
    if (std::filesystem::is_directory(path, ec)) {
        if (auto f = storage_files(path, ".mcap"); !f.empty()) return open_mcap(f, path);
        if (auto f = storage_files(path, ".db3"); !f.empty()) return open_sqlite(f, path);
        throw std::runtime_error("no .mcap or .db3 storage files in " + path.string());
    }
    const std::string ext = lower_ext(path);
    if (ext == ".mcap") return open_mcap({path}, path);
    if (ext == ".db3") return open_sqlite({path}, path);
    if (ext == ".bag") throw std::runtime_error("ROS 1 .bag files are not supported yet; convert it to a ROS 2 bag");
    throw std::runtime_error("not a ROS 2 bag: " + path.string());
}

bool is_bag(const std::filesystem::path& path) {
    std::error_code ec;
    if (std::filesystem::is_directory(path, ec)) {
        return std::filesystem::exists(path / "metadata.yaml", ec) || !storage_files(path, ".mcap").empty() ||
               !storage_files(path, ".db3").empty();
    }
    const std::string ext = lower_ext(path);
    return ext == ".mcap" || ext == ".db3";
}

}  // namespace ogccdb::flight
