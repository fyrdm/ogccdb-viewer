#include <cmath>
#include <cstdlib>
#include <cstring>
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
