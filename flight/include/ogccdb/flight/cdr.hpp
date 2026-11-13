#pragma once
// Minimal reader and writer for ROS 2 messages in CDR (XCDR1, the ROS 2 default): a
// 4-byte encapsulation header, then fields aligned to their size, counted
// from the end of the header.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ogccdb::flight {

class CdrReader {
public:
    CdrReader(const std::uint8_t* data, std::size_t size) : data_(data), size_(size) {
        if (size < 4) throw std::runtime_error("CDR message shorter than its header");
        little_ = (data[1] & 1) != 0;  // 0x00 0x01 = CDR little endian, 0x00 0x00 = big endian
        pos_ = 4;
    }

    template <typename T>
    T read() {
        align(sizeof(T));
        need(sizeof(T));
        unsigned char b[sizeof(T)];
        std::memcpy(b, data_ + pos_, sizeof(T));
        if (little_ != host_little()) {
            for (std::size_t i = 0; i < sizeof(T) / 2; ++i) std::swap(b[i], b[sizeof(T) - 1 - i]);
        }
        pos_ += sizeof(T);
        T v;
        std::memcpy(&v, b, sizeof(T));
        return v;
    }
    std::string string() {
        const auto n = read<std::uint32_t>();  // includes the terminating zero
        need(n);
        std::string s(reinterpret_cast<const char*>(data_ + pos_), n > 0 ? n - 1 : 0);
        pos_ += n;
        return s;
    }
    void skip_doubles(std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) read<double>();
    }
    std::size_t position() const { return pos_; }

private:
    static bool host_little() {
        const std::uint16_t one = 1;
        unsigned char c;
        std::memcpy(&c, &one, 1);
        return c == 1;
    }
    void align(std::size_t n) {
        const std::size_t off = pos_ - 4;
        pos_ += (n - off % n) % n;
    }
    void need(std::size_t n) const {
        if (pos_ + n > size_) throw std::runtime_error("CDR message ends early");
    }

    const std::uint8_t* data_;
    std::size_t size_;
    std::size_t pos_ = 0;
    bool little_ = true;
};

/// Writer for ROS 2 messages in CDR (XCDR1, little endian), the counterpart of
/// CdrReader.
class CdrWriter {
public:
    CdrWriter() : buf_{0x00, 0x01, 0x00, 0x00} {}  // CDR little endian, no options

    template <typename T>
    void write(T v) {
        align(sizeof(T));
        unsigned char b[sizeof(T)];
        std::memcpy(b, &v, sizeof(T));
        if (!host_little()) {
            for (std::size_t i = 0; i < sizeof(T) / 2; ++i) std::swap(b[i], b[sizeof(T) - 1 - i]);
        }
        buf_.insert(buf_.end(), b, b + sizeof(T));
    }
    void string(const std::string& s) {
        write<std::uint32_t>(static_cast<std::uint32_t>(s.size() + 1));  // with the terminating zero
        buf_.insert(buf_.end(), s.begin(), s.end());
        buf_.push_back(0);
    }
    /// A uint8[] sequence: its length, then the bytes.
    void bytes(const std::uint8_t* data, std::size_t n) {
        write<std::uint32_t>(static_cast<std::uint32_t>(n));
        buf_.insert(buf_.end(), data, data + n);
    }
    const std::vector<std::uint8_t>& data() const { return buf_; }
    std::vector<std::uint8_t> take() { return std::move(buf_); }

private:
    static bool host_little() {
        const std::uint16_t one = 1;
        unsigned char c;
        std::memcpy(&c, &one, 1);
        return c == 1;
    }
    void align(std::size_t n) {
        while ((buf_.size() - 4) % n) buf_.push_back(0);
    }

    std::vector<std::uint8_t> buf_;
};

}  // namespace ogccdb::flight
