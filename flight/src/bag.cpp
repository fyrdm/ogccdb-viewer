#include "ogccdb/flight/bag.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include "storage.hpp"

namespace ogccdb::flight {

namespace {

std::string lower_ext(const std::filesystem::path& p) {
    std::string e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return e;
}

bool storage_order(const std::string& a, const std::string& b);

/// Storage files of a bag folder, in name order (rosbag2 names them _0, _1, ...).
std::vector<std::filesystem::path> storage_files(const std::filesystem::path& dir, const std::string& ext) {
    std::vector<std::filesystem::path> files;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
        if (e.is_regular_file(ec) && lower_ext(e.path()) == ext) files.push_back(e.path());
    }
    std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b) { return storage_order(a.filename().string(), b.filename().string()); });
    return files;
}

/// rosbag2 order: _2 before _10.
bool storage_order(const std::string& a, const std::string& b) { return a.size() != b.size() ? a.size() < b.size() : a < b; }

std::vector<StoragePart> whole_files(const std::vector<std::filesystem::path>& files) {
    std::vector<StoragePart> parts;
    for (const auto& f : files) parts.push_back(StoragePart{f, 0, std::numeric_limits<std::uint64_t>::max(), f.string()});
    return parts;
}

std::string member_dir(const std::string& name) {
    const auto slash = name.rfind('/');
    return slash == std::string::npos ? std::string() : name.substr(0, slash);
}

std::string member_ext(const std::string& name) { return lower_ext(std::filesystem::path(name)); }

/// The storage members (.mcap, else .db3) of the first bag in a tar, in storage order.
std::vector<TarMember> tar_storage(const std::vector<TarMember>& members, std::string* ext) {
    for (const char* e : {".mcap", ".db3"}) {
        std::vector<TarMember> found;
        for (const TarMember& m : members) {
            if (member_ext(m.name) == e && std::filesystem::path(m.name).filename().string()[0] != '.') found.push_back(m);
        }
        if (found.empty()) continue;
        std::sort(found.begin(), found.end(), [](const TarMember& a, const TarMember& b) {
            const std::string da = member_dir(a.name), db = member_dir(b.name);
            return da != db ? da < db : storage_order(a.name, b.name);
        });
        const std::string dir = member_dir(found.front().name);  // a tar of several bags: the first one
        found.erase(std::remove_if(found.begin(), found.end(), [&](const TarMember& m) { return member_dir(m.name) != dir; }),
                    found.end());
        *ext = e;
        return found;
    }
    return {};
}

/// SQLite needs real files: copies the .db3 members to a cache folder (once per tar
/// size and time) and returns their paths.
std::vector<std::filesystem::path> extract(const std::filesystem::path& tar, const std::vector<TarMember>& members) {
    std::error_code ec;
    const auto stamp = std::filesystem::last_write_time(tar, ec).time_since_epoch().count();
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "ogccdb-bags" /
                                      (tar.stem().string() + "-" + std::to_string(std::filesystem::file_size(tar)) + "-" +
                                       std::to_string(static_cast<std::uint64_t>(stamp) % 1000000007u));
    std::filesystem::create_directories(dir);
    std::vector<std::filesystem::path> files;
    std::ifstream in(tar, std::ios::binary);
    std::vector<char> buf(1 << 20);
    for (const TarMember& m : members) {
        const std::filesystem::path out_path = dir / std::filesystem::path(m.name).filename();
        files.push_back(out_path);
        if (std::filesystem::exists(out_path, ec) && std::filesystem::file_size(out_path, ec) == m.size) continue;
        const std::filesystem::path tmp = out_path.string() + ".part";
        std::ofstream out(tmp, std::ios::binary);
        in.seekg(static_cast<std::streamoff>(m.offset));
        for (std::uint64_t left = m.size; left > 0;) {
            const auto n = static_cast<std::streamsize>(std::min<std::uint64_t>(left, buf.size()));
            if (!in.read(buf.data(), n)) throw std::runtime_error(tar.string() + ": tar member " + m.name + " ends early");
            out.write(buf.data(), n);
            left -= static_cast<std::uint64_t>(n);
        }
        out.close();
        if (!out) throw std::runtime_error("cannot write " + tmp.string());
        std::filesystem::rename(tmp, out_path);
    }
    return files;
}

std::unique_ptr<Bag> open_tar(const std::filesystem::path& path) {
    std::string ext;
    const std::vector<TarMember> storage = tar_storage(tar_members(path), &ext);
    if (storage.empty()) throw std::runtime_error("no .mcap or .db3 bag inside " + path.string());
    if (ext == ".db3") return open_sqlite(extract(path, storage), path);
    std::vector<StoragePart> parts;
    for (const TarMember& m : storage) parts.push_back(StoragePart{path, m.offset, m.size, path.string() + ":" + m.name});
    return open_mcap(parts, path);  // read in place, nothing extracted
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
        if (auto f = storage_files(path, ".mcap"); !f.empty()) return open_mcap(whole_files(f), path);
        if (auto f = storage_files(path, ".db3"); !f.empty()) return open_sqlite(f, path);
        throw std::runtime_error("no .mcap or .db3 storage files in " + path.string());
    }
    const std::string ext = lower_ext(path);
    if (ext == ".mcap") return open_mcap(whole_files({path}), path);
    if (ext == ".tar") return open_tar(path);
    if (ext == ".gz" || ext == ".tgz" || ext == ".xz" || ext == ".zst" || ext == ".bz2" || ext == ".zip") {
        throw std::runtime_error("compressed archives are not read directly; extract it, or use an uncompressed .tar");
    }
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
    if (ext == ".tar") {
        try {
            std::string found;
            return !tar_storage(tar_members(path), &found).empty();
        } catch (const std::exception&) {
            return false;
        }
    }
    return ext == ".mcap" || ext == ".db3";
}

}  // namespace ogccdb::flight
