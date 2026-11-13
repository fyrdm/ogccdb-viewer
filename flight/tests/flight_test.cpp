#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "ogccdb/flight/bag.hpp"
#include "ogccdb/flight/bag_writer.hpp"
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

TEST(Messages, EncodersRoundTrip) {
    GeoPose g;
    g.header.ns = 1794409507138737172;
    g.header.frame_id = "base_link";
    g.latitude = 39.7697167;
    g.longitude = 32.0482443;
    g.altitude = 774.25;
    g.orientation = {0.1, -0.2, 0.3, 0.927};
    const auto gb = encode_geopose(g);
    const GeoPose g2 = decode_geopose(gb.data(), gb.size());
    EXPECT_EQ(g2.header.ns, g.header.ns);
    EXPECT_EQ(g2.header.frame_id, "base_link");
    EXPECT_DOUBLE_EQ(g2.latitude, g.latitude);
    EXPECT_DOUBLE_EQ(g2.longitude, g.longitude);
    EXPECT_DOUBLE_EQ(g2.altitude, g.altitude);
    EXPECT_DOUBLE_EQ(g2.orientation.y, -0.2);
    EXPECT_DOUBLE_EQ(g2.orientation.w, 0.927);

    CameraInfo c;
    c.header.ns = -1;  // just before the epoch: sec -1, nanosec 999999999
    c.header.frame_id = "camera_optical";
    c.width = 1280;
    c.height = 720;
    c.d = {-0.1, 0.02, 0.001, -0.002, 0.0};
    c.k = {900, 0, 639.5, 0, 900, 359.5, 0, 0, 1};
    c.p = {900, 0, 639.5, 0, 0, 900, 359.5, 0, 0, 0, 1, 0};
    const auto cb = encode_camera_info(c);
    const CameraInfo c2 = decode_camera_info(cb.data(), cb.size());
    EXPECT_EQ(c2.header.ns, -1);
    EXPECT_EQ(c2.width, 1280u);
    EXPECT_EQ(c2.height, 720u);
    EXPECT_EQ(c2.distortion_model, "plumb_bob");
    EXPECT_EQ(c2.d, c.d);
    EXPECT_EQ(c2.k, c.k);
    EXPECT_EQ(c2.p, c.p);
    // Worked out by hand: 4 (encapsulation) + header to 27, pad 1, height and
    // width to 36, model to 50, pad 2, d count to 56, 35 doubles to 336,
    // binning to 344, roi to 361.
    EXPECT_EQ(cb.size(), 4u + 361u);

    const std::uint8_t jpeg[3] = {0xFF, 0xD8, 0xFF};
    const auto ib = encode_compressed_image(Stamp{5, "cam"}, "jpeg", jpeg, 3);
    CdrReader r(ib.data(), ib.size());
    EXPECT_EQ(r.read<std::int32_t>(), 0);
    EXPECT_EQ(r.read<std::uint32_t>(), 5u);
    EXPECT_EQ(r.string(), "cam");
    EXPECT_EQ(r.string(), "jpeg");
    EXPECT_EQ(r.read<std::uint32_t>(), 3u);
    EXPECT_EQ(ib.size(), r.position() + 3);

    for (const char* type : {"sensor_msgs/msg/CompressedImage", "sensor_msgs/msg/CameraInfo",
                             "tf2_msgs/msg/TFMessage", "geographic_msgs/msg/GeoPoseStamped"}) {
        const std::string def = message_definition(type);
        EXPECT_NE(def.find("MSG: builtin_interfaces/Time"), std::string::npos) << type;
        const std::string hash = type_hash(type);
        EXPECT_EQ(hash.rfind("RIHS01_", 0), 0u) << type;
        EXPECT_EQ(hash.size(), 7u + 64u) << type;
    }
    EXPECT_TRUE(message_definition("std_msgs/msg/String").empty());
    EXPECT_TRUE(type_hash("std_msgs/msg/String").empty());
}

namespace {

std::uint64_t le64(const std::string& s, std::size_t at) {
    std::uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(static_cast<unsigned char>(s[at + i])) << (8 * i);
    return v;
}

}  // namespace

TEST(BagWriter, McapRoundTripWithIndexes) {
    const auto dir = std::filesystem::temp_directory_path() / "ogccdb_flight_writer_test";
    std::filesystem::create_directories(dir);
    for (const std::string compression : {"", "zstd", "lz4"}) {
        if (!McapWriter::supports(compression)) continue;
        const auto file = dir / ("w_" + (compression.empty() ? std::string("none") : compression) + ".mcap");
        BagTopic fix;
        fix.name = "/fix";
        fix.type = "sensor_msgs/msg/NavSatFix";
        fix.encoding = "cdr";
        fix.schema_encoding = "ros2msg";
        fix.schema = "uint8 x\n";
        fix.metadata = {{"offered_qos_profiles", "- depth: 0\n"}};
        BagTopic pose = fix;
        pose.name = "/pose";
        pose.type = "geographic_msgs/msg/GeoPoseStamped";
        pose.schema = message_definition(pose.type);
        {
            McapWriterOptions o;
            o.compression = compression;
            o.chunk_size = 200;  // several chunks
            McapWriter w(file, o);
            const auto a = w.add_topic(fix);
            const auto b = w.add_topic(pose);
            for (int i = 0; i < 40; ++i) {
                const std::string payload = "message " + std::to_string(i);
                w.write(i % 3 ? a : b, 1000 + i * 10, reinterpret_cast<const std::uint8_t*>(payload.data()),
                        payload.size(), i == 5 ? 7 : -1);
            }
            w.close();
            EXPECT_EQ(w.message_count(), 40u);
        }
        auto bag = Bag::open(file);
        ASSERT_EQ(bag->topics().size(), 2u) << compression;
        EXPECT_EQ(bag->topics()[0].name, "/fix");
        EXPECT_EQ(bag->topics()[0].schema, "uint8 x\n");
        EXPECT_EQ(bag->topics()[0].schema_encoding, "ros2msg");
        ASSERT_EQ(bag->topics()[0].metadata.size(), 1u);
        EXPECT_EQ(bag->topics()[0].metadata[0].second, "- depth: 0\n");
        EXPECT_EQ(bag->topics()[1].type, "geographic_msgs/msg/GeoPoseStamped");
        EXPECT_EQ(bag->message_count(), 40u);
        EXPECT_EQ(bag->start_ns(), 1000);
        EXPECT_EQ(bag->end_ns(), 1390);
        int n = 0;
        bag->read([&](const BagMessage& m) {
            EXPECT_EQ(std::string(reinterpret_cast<const char*>(m.data), m.size), "message " + std::to_string(n));
            EXPECT_EQ(m.log_time_ns, 1000 + n * 10);
            EXPECT_EQ(m.publish_time_ns, n == 5 ? 7 : m.log_time_ns);
            ++n;
        });
        EXPECT_EQ(n, 40);

        // The footer leads to the summary, whose chunk indexes point at chunks
        // and message indexes.
        std::ifstream in(file, std::ios::binary);
        const std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        ASSERT_GT(s.size(), 8u + 29u);
        const std::size_t footer = s.size() - 8 - 29;
        ASSERT_EQ(s[footer], '\x02');
        std::size_t p = le64(s, footer + 9);  // summary start
        const std::size_t summary_offsets = le64(s, footer + 17);
        int chunk_indexes = 0;
        while (p < summary_offsets) {
            const auto op = static_cast<unsigned char>(s[p]);
            const std::size_t len = le64(s, p + 1);
            if (op == 0x08) {
                const std::size_t body = p + 9;
                EXPECT_EQ(s[le64(s, body + 16)], '\x06');  // chunk_start_offset
                const auto map_len = static_cast<std::uint32_t>(le64(s, body + 32) & 0xFFFFFFFF);
                for (std::size_t e = 0; e < map_len; e += 10) EXPECT_EQ(s[le64(s, body + 36 + e + 2)], '\x07');
                ++chunk_indexes;
            }
            p += 9 + len;
        }
        EXPECT_GT(chunk_indexes, 1) << compression;
    }
    std::filesystem::remove_all(dir);
}

TEST(BagWriter, FolderMetadataListsTopicsAndTimes) {
    const std::filesystem::path dir = std::filesystem::temp_directory_path() / "ogccdb_flight_metadata_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    BagTopic image;
    image.name = "/camera/image_raw/compressed";
    image.type = "sensor_msgs/msg/CompressedImage";
    image.encoding = "cdr";
    image.schema_encoding = "ros2msg";
    image.schema = message_definition(image.type);
    image.metadata = {{"offered_qos_profiles", "- history: unknown\n  depth: 0\n  reliability: reliable"},
                      {"topic_type_hash", type_hash(image.type)}};
    BagTopic plain;
    plain.name = "/other";
    plain.type = "std_msgs/msg/String";
    {
        McapWriterOptions options;
        options.compression.clear();
        McapWriter writer(dir / "bag_0.mcap", options);
        const std::uint16_t a = writer.add_topic(image), b = writer.add_topic(plain);
        const std::uint8_t byte = 0;
        writer.write(a, 2'000'000'000, &byte, 1);
        writer.write(a, 5'500'000'000, &byte, 1);
        writer.write(b, 3'000'000'000, &byte, 1);
        writer.close();
        const std::vector<BagTopic> topics = writer.topics();
        ASSERT_EQ(topics.size(), 2u);
        EXPECT_EQ(topics[0].name, image.name);
        EXPECT_EQ(topics[0].type, image.type);
        EXPECT_EQ(topics[0].count, 2u);
        EXPECT_EQ(topics[1].count, 1u);
        EXPECT_EQ(writer.start_ns(), 2'000'000'000);
        EXPECT_EQ(writer.end_ns(), 5'500'000'000);
        write_bag_metadata(dir, "bag_0.mcap", topics, writer.start_ns(), writer.end_ns(), "jazzy");
    }
    std::ifstream in(dir / "metadata.yaml");
    const std::string y((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    EXPECT_EQ(y.rfind("rosbag2_bagfile_information:\n  version: 9\n  storage_identifier: mcap\n", 0), 0u);
    EXPECT_NE(y.find("  duration:\n    nanoseconds: 3500000000\n"), std::string::npos);
    EXPECT_NE(y.find("  starting_time:\n    nanoseconds_since_epoch: 2000000000\n"), std::string::npos);
    EXPECT_NE(y.find("  message_count: 3\n"), std::string::npos);
    EXPECT_NE(y.find("        name: /camera/image_raw/compressed\n        type: sensor_msgs/msg/CompressedImage\n"
                     "        serialization_format: cdr\n        offered_qos_profiles:\n"
                     "          - history: unknown\n            depth: 0\n            reliability: reliable\n"
                     "        type_description_hash: \"" + type_hash(image.type) + "\"\n      message_count: 2\n"),
              std::string::npos);
    // A topic without QoS text or hash still gets both keys.
    EXPECT_NE(y.find("        name: /other\n        type: std_msgs/msg/String\n        serialization_format: cdr\n"
                     "        offered_qos_profiles:\n          []\n        type_description_hash: \"\"\n"
                     "      message_count: 1\n"),
              std::string::npos);
    EXPECT_NE(y.find("  relative_file_paths:\n    - bag_0.mcap\n"), std::string::npos);
    EXPECT_NE(y.find("  ros_distro: jazzy\n"), std::string::npos);
    // The folder still opens as a bag.
    const auto bag = Bag::open(dir);
    EXPECT_EQ(bag->message_count(), 3u);
    std::filesystem::remove_all(dir);
}

TEST(Track, NeverPicksATopicMarkedAsAnEstimate) {
    // A fix at 1 Hz, and a busier GeoPoseStamped that is 0.01 degrees off:
    // unmarked, the busier topic gives the position; marked as an estimate, it does not.
    const auto make = [](bool marked, const std::filesystem::path& file) {
        McapWriterOptions options;
        options.compression.clear();
        McapWriter writer(file, options);
        BagTopic fix;
        fix.name = "/fix";
        fix.type = "sensor_msgs/msg/NavSatFix";
        BagTopic prior;
        prior.name = "/prior";
        prior.type = "geographic_msgs/msg/GeoPoseStamped";
        if (marked) prior.metadata.emplace_back(kRoleKey, kRoleEstimate);
        const std::uint16_t a = writer.add_topic(fix), b = writer.add_topic(prior);
        for (int i = 0; i < 5; ++i) {
            Writer w;
            w.header(i, 0, "gps");
            w.put<std::int8_t>(0);
            w.put<std::uint16_t>(1);
            w.put(40.0);
            w.put(30.0 + i * 1e-5);
            w.put(100.0);
            for (int k = 0; k < 9; ++k) w.put(0.0);
            w.put<std::uint8_t>(0);
            writer.write(a, i * 1'000'000'000ll, w.b.data(), w.b.size());
        }
        for (int i = 0; i < 9; ++i) {
            GeoPose g;
            g.header.ns = i * 500'000'000ll;
            g.latitude = 40.01;
            g.longitude = 30.0;
            g.altitude = 100.0;
            const auto m = encode_geopose(g);
            writer.write(b, g.header.ns, m.data(), m.size());
        }
        writer.close();
    };
    const auto dir = std::filesystem::temp_directory_path() / "ogccdb_flight_estimate_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    make(false, dir / "plain.mcap");
    make(true, dir / "marked.mcap");
    {
        const auto bag = Bag::open(dir / "plain.mcap");
        EXPECT_FALSE(is_estimate(*bag->topic("/prior")));
        EXPECT_EQ(FlightTrack::build(*bag).info().fix_topic, "/prior");
    }
    {
        const auto bag = Bag::open(dir / "marked.mcap");
        EXPECT_TRUE(is_estimate(*bag->topic("/prior")));
        const FlightTrack track = FlightTrack::build(*bag);
        EXPECT_EQ(track.info().fix_topic, "/fix");
        EXPECT_NEAR(track.at(1.0).geodetic.lat, 40.0, 1e-9);
        // Named, it is used all the same.
        TrackOptions named;
        named.fix_topic = "/prior";
        EXPECT_NEAR(FlightTrack::build(*bag, named).at(1.0).geodetic.lat, 40.01, 1e-9);
    }
    std::filesystem::remove_all(dir);
}
