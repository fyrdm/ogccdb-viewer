#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ogccdb/flight/bag.hpp"
#include "ogccdb/flight/cdr.hpp"
#include "ogccdb/flight/messages.hpp"
#include "ogccdb/flight/track.hpp"

using namespace ogccdb::flight;

namespace {

/// Builds a little-endian CDR message.
struct Writer {
    std::vector<std::uint8_t> b{0x00, 0x01, 0x00, 0x00};
    void align(std::size_t n) {
        while ((b.size() - 4) % n) b.push_back(0);
    }
    template <typename T>
    void put(T v) {
        align(sizeof(T));
        const auto* p = reinterpret_cast<const std::uint8_t*>(&v);
        b.insert(b.end(), p, p + sizeof(T));
    }
    void str(const std::string& s) {
        put<std::uint32_t>(static_cast<std::uint32_t>(s.size() + 1));
        b.insert(b.end(), s.begin(), s.end());
        b.push_back(0);
    }
    void header(std::int32_t sec, std::uint32_t nsec, const std::string& frame) {
        put(sec);
        put(nsec);
        str(frame);
    }
};

/// A minimal MCAP file: one channel, messages at the given log times.
std::string tiny_mcap(const std::vector<std::uint64_t>& times) {
    std::string out("\x89MCAP0\r\n", 8);
    auto u16 = [](std::string& s, std::uint16_t v) { s.append(reinterpret_cast<const char*>(&v), 2); };
    auto u32 = [](std::string& s, std::uint32_t v) { s.append(reinterpret_cast<const char*>(&v), 4); };
    auto u64 = [](std::string& s, std::uint64_t v) { s.append(reinterpret_cast<const char*>(&v), 8); };
    auto str = [&](std::string& s, const std::string& v) { u32(s, static_cast<std::uint32_t>(v.size())); s += v; };
    auto rec = [&](std::uint8_t op, const std::string& body) {
        out += static_cast<char>(op);
        u64(out, body.size());
        out += body;
    };
    std::string b;
    str(b, "ros2");
    str(b, "test");
    rec(0x01, b);  // header
    b.clear();
    u16(b, 1);
    str(b, "sensor_msgs/msg/NavSatFix");
    str(b, "ros2msg");
    str(b, "");
    rec(0x03, b);  // schema
    b.clear();
    u16(b, 7);
    u16(b, 1);
    str(b, "/fix");
    str(b, "cdr");
    u32(b, 0);
    rec(0x04, b);  // channel
    for (std::size_t i = 0; i < times.size(); ++i) {
        b.clear();
        u16(b, 7);
        u32(b, static_cast<std::uint32_t>(i));
        u64(b, times[i]);
        u64(b, times[i]);
        b += "data";
        rec(0x05, b);  // message
    }
    rec(0x02, std::string(20, '\0'));  // footer
    out.append("\x89MCAP0\r\n", 8);
    return out;
}

/// A ustar archive with one regular file.
std::string tiny_tar(const std::string& name, const std::string& data) {
    char h[512] = {};
    std::snprintf(h, 100, "%s", name.c_str());
    std::snprintf(h + 100, 8, "%07o", 0644);
    std::snprintf(h + 124, 12, "%011o", static_cast<unsigned>(data.size()));
    std::snprintf(h + 136, 12, "%011o", 0u);
    h[156] = '0';
    std::memcpy(h + 257, "ustar", 6);
    std::memcpy(h + 263, "00", 2);
    std::memset(h + 148, ' ', 8);
    unsigned sum = 0;
    for (unsigned char c : h) sum += c;
    std::snprintf(h + 148, 8, "%06o", sum);
    std::string out(h, 512);
    out += data;
    out.append((512 - data.size() % 512) % 512, '\0');
    out.append(1024, '\0');
    return out;
}

void write_file(const std::filesystem::path& p, const std::string& s) {
    std::ofstream(p, std::ios::binary).write(s.data(), static_cast<std::streamsize>(s.size()));
}

}  // namespace

TEST(Messages, DecodesNavSatFix) {
    Writer w;
    w.header(100, 5, "base_link");
    w.put<std::int8_t>(0);
    w.put<std::uint16_t>(1);
    w.put(39.5);
    w.put(32.25);
    w.put(1416.5);
    for (int i = 0; i < 9; ++i) w.put(0.0);
    w.put<std::uint8_t>(2);
    const NavSatFix f = decode_navsatfix(w.b.data(), w.b.size());
    EXPECT_EQ(f.header.ns, 100000000005LL);
    EXPECT_EQ(f.header.frame_id, "base_link");
    EXPECT_EQ(f.status, 0);
    EXPECT_DOUBLE_EQ(f.latitude, 39.5);
    EXPECT_DOUBLE_EQ(f.longitude, 32.25);
    EXPECT_DOUBLE_EQ(f.altitude, 1416.5);
}

TEST(Messages, DecodesPoseStampedAligned) {
    Writer w;
    w.header(1, 2, "map");  // 4 + 4 + 4 + "map\0" = 16: doubles start aligned
    for (double v : {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0}) w.put(v);
    const auto p = decode_local_pose("geometry_msgs/msg/PoseStamped", w.b.data(), w.b.size());
    ASSERT_TRUE(p);
    EXPECT_DOUBLE_EQ(p->x, 1.0);
    EXPECT_DOUBLE_EQ(p->z, 3.0);
    EXPECT_DOUBLE_EQ(p->orientation.w, 1.0);

    Writer w2;
    w2.header(1, 2, "odom");  // 17 bytes: the doubles need 7 bytes of padding
    w2.str("base");
    for (double v : {4.0, 5.0, 6.0, 0.0, 0.0, 0.7071, 0.7071}) w2.put(v);
    const auto o = decode_local_pose("nav_msgs/msg/Odometry", w2.b.data(), w2.b.size());
    ASSERT_TRUE(o);
    EXPECT_DOUBLE_EQ(o->x, 4.0);
    EXPECT_DOUBLE_EQ(o->orientation.z, 0.7071);
}

TEST(Messages, ShortMessageThrows) {
    Writer w;
    w.header(1, 2, "map");
    EXPECT_THROW(decode_local_pose("geometry_msgs/msg/PoseStamped", w.b.data(), w.b.size()), std::runtime_error);
}

TEST(Track, RotateAndSlerp) {
    const double s = std::sqrt(0.5);
    const Quat yaw90{0, 0, s, s};  // ENU: nose from east to north
    const Vec3d f = rotate(yaw90, {1, 0, 0});
    EXPECT_NEAR(f.x, 0.0, 1e-12);
    EXPECT_NEAR(f.y, 1.0, 1e-12);
    const Quat half = slerp(Quat{}, yaw90, 0.5);
    EXPECT_NEAR(rotate(half, {1, 0, 0}).y, std::sin(0.78539816339744831), 1e-12);
    // -q is the same rotation: slerp takes the short way.
    const Quat neg{-yaw90.x, -yaw90.y, -yaw90.z, -yaw90.w};
    EXPECT_NEAR(rotate(slerp(Quat{}, neg, 0.5), {1, 0, 0}).y, std::sin(0.78539816339744831), 1e-12);
}

TEST(Track, DownwardCameraLooksDownWithTopForward) {
    FlightPose p;
    p.forward = {1, 0, 0};
    p.left = {0, 1, 0};
    p.up = {0, 0, 1};
    Vec3d f, u;
    camera_axes(p, -90.0, f, u);
    EXPECT_NEAR(f.z, -1.0, 1e-12);
    EXPECT_NEAR(u.x, 1.0, 1e-12);
    camera_axes(p, 0.0, f, u);
    EXPECT_NEAR(f.x, 1.0, 1e-12);
    EXPECT_NEAR(u.z, 1.0, 1e-12);
}

TEST(Bag, ReadsMcapInsideTar) {
    const auto dir = std::filesystem::temp_directory_path() / "ogccdb_flight_tar_test";
    std::filesystem::create_directories(dir);
    const std::string mcap = tiny_mcap({1000, 2000, 5000});
    write_file(dir / "b.mcap", mcap);
    write_file(dir / "b.tar", tiny_tar("bag/b_0.mcap", mcap) );
    EXPECT_TRUE(is_bag(dir / "b.tar"));
    for (const char* f : {"b.mcap", "b.tar"}) {
        auto bag = Bag::open(dir / f);
        ASSERT_EQ(bag->topics().size(), 1u) << f;
        EXPECT_EQ(bag->topics()[0].name, "/fix");
        EXPECT_EQ(bag->topics()[0].type, "sensor_msgs/msg/NavSatFix");
        EXPECT_EQ(bag->message_count(), 3u);
        EXPECT_EQ(bag->start_ns(), 1000);
        EXPECT_EQ(bag->end_ns(), 5000);
        std::size_t n = 0;
        bag->read([&](const BagMessage& m) {
            EXPECT_EQ(std::string(reinterpret_cast<const char*>(m.data), m.size), "data");
            ++n;
        });
        EXPECT_EQ(n, 3u);
    }
    write_file(dir / "empty.tar", tiny_tar("notes.txt", "hello"));
    EXPECT_FALSE(is_bag(dir / "empty.tar"));
    EXPECT_THROW(Bag::open(dir / "empty.tar"), std::runtime_error);
    std::filesystem::remove_all(dir);
}

// Reads a real bag if OGCCDB_FLIGHT_TEST_BAG points to one.
TEST(Bag, ReadsTestBag) {
    const char* path = std::getenv("OGCCDB_FLIGHT_TEST_BAG");
    if (!path) GTEST_SKIP() << "set OGCCDB_FLIGHT_TEST_BAG to a ROS 2 bag";
    auto bag = Bag::open(path);
    EXPECT_GT(bag->message_count(), 0u);
    const FlightTrack track = FlightTrack::build(*bag);
    EXPECT_FALSE(track.empty());
    EXPECT_GT(track.duration(), 0.0);
}
