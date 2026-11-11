// Reading the member list of an uncompressed tar archive (POSIX ustar, with
// GNU long names and pax path records), so a bag can be played straight
// from the .tar it was shipped in.
#include <cstring>
#include <fstream>
#include <stdexcept>

#include "storage.hpp"

namespace ogccdb::flight {
namespace {

std::uint64_t octal(const char* p, std::size_t n) {
    // Base-256 (GNU) for sizes of 8 GiB and more: high bit of the first byte set.
    if (static_cast<unsigned char>(p[0]) & 0x80) {
        std::uint64_t v = static_cast<unsigned char>(p[0]) & 0x7f;
        for (std::size_t i = 1; i < n; ++i) v = (v << 8) | static_cast<unsigned char>(p[i]);
        return v;
    }
    std::uint64_t v = 0;
    for (std::size_t i = 0; i < n && p[i]; ++i) {
        if (p[i] == ' ') continue;
        if (p[i] < '0' || p[i] > '7') break;
        v = v * 8 + static_cast<std::uint64_t>(p[i] - '0');
    }
    return v;
}

std::string field(const char* p, std::size_t n) { return std::string(p, strnlen(p, n)); }

bool checksum_ok(const unsigned char* h) {
    std::uint64_t sum = 0;
    for (int i = 0; i < 512; ++i) sum += (i >= 148 && i < 156) ? ' ' : h[i];
    return sum == octal(reinterpret_cast<const char*>(h) + 148, 8);
}

/// The "path" record of a pax extended header, if any.
std::string pax_path(const std::string& data) {
    std::size_t i = 0;
    while (i < data.size()) {
        const std::size_t sp = data.find(' ', i);
        if (sp == std::string::npos) break;
        const std::size_t len = std::stoul(data.substr(i, sp - i));
        if (len == 0 || i + len > data.size()) break;
        const std::string rec = data.substr(sp + 1, i + len - sp - 2);  // without the trailing '\n'
        if (rec.compare(0, 5, "path=") == 0) return rec.substr(5);
        i += len;
    }
    return {};
}

}  // namespace

std::vector<TarMember> tar_members(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open " + file.string());
    std::vector<TarMember> members;
    std::string long_name;
    std::uint64_t pos = 0;
    unsigned char h[512];
    for (;;) {
        if (!in.read(reinterpret_cast<char*>(h), 512)) break;
        pos += 512;
        if (h[0] == 0) break;  // end-of-archive block
        if (!checksum_ok(h)) {
            if (members.empty() && long_name.empty()) throw std::runtime_error(file.string() + " is not a tar archive");
            throw std::runtime_error(file.string() + ": damaged tar header");
        }
        const char* c = reinterpret_cast<const char*>(h);
        const std::uint64_t size = octal(c + 124, 12);
        const char type = c[156];
        const std::uint64_t padded = (size + 511) / 512 * 512;
        if (type == 'L' || type == 'x') {  // GNU long name / pax header for the next member
            std::string data(static_cast<std::size_t>(size), '\0');
            if (!in.read(data.data(), static_cast<std::streamsize>(size))) break;
            in.seekg(static_cast<std::streamoff>(padded - size), std::ios::cur);
            long_name = type == 'L' ? std::string(data.c_str()) : pax_path(data);
            pos += padded;
            continue;
        }
        std::string name = long_name;
        long_name.clear();
        if (name.empty()) {
            name = field(c, 100);
            if (std::memcmp(c + 257, "ustar", 5) == 0) {
                const std::string prefix = field(c + 345, 155);
                if (!prefix.empty()) name = prefix + "/" + name;
            }
        }
        if (type == '0' || type == '\0' || type == '7') members.push_back(TarMember{name, pos, size});
        pos += padded;
        in.seekg(static_cast<std::streamoff>(pos), std::ios::beg);
    }
    return members;
}

}  // namespace ogccdb::flight
