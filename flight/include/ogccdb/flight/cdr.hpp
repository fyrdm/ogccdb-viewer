#pragma once
// Minimal reader for ROS 2 messages in CDR (XCDR1, the ROS 2 default): a
// 4-byte encapsulation header, then fields aligned to their size, counted
// from the end of the header.
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

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

}  // namespace ogccdb::flight
