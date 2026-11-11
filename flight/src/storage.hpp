// Internal: storage files of a bag, possibly members of a tar archive.
#pragma once

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ogccdb::flight {

class Bag;

/// A byte range of a file: a whole file, or one member of an uncompressed tar.
struct StoragePart {
    std::filesystem::path file;
    std::uint64_t offset = 0;
    std::uint64_t size = std::numeric_limits<std::uint64_t>::max();
    std::string name;  ///< file name, or member name inside the tar
};

/// One regular file inside a tar archive.
struct TarMember {
    std::string name;  ///< path inside the archive, '/' separated
    std::uint64_t offset = 0, size = 0;
};

/// Lists the regular files of an uncompressed tar (ustar, GNU or pax);
/// throws std::runtime_error if `file` is not one.
std::vector<TarMember> tar_members(const std::filesystem::path& file);

std::unique_ptr<Bag> open_mcap(const std::vector<StoragePart>& parts, const std::filesystem::path& path);
std::unique_ptr<Bag> open_sqlite(const std::vector<std::filesystem::path>& files, const std::filesystem::path& path);

}  // namespace ogccdb::flight
