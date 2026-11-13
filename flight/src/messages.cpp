#include "ogccdb/flight/messages.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include "ogccdb/flight/cdr.hpp"

namespace ogccdb::flight {
namespace {

Stamp read_header(CdrReader& r) {
    Stamp s;
    const auto sec = r.read<std::int32_t>();
    const auto nsec = r.read<std::uint32_t>();
    s.ns = static_cast<std::int64_t>(sec) * 1000000000 + nsec;
    s.frame_id = r.string();
    return s;
}

Quat read_quat(CdrReader& r) {
    Quat q;
    q.x = r.read<double>();
    q.y = r.read<double>();
    q.z = r.read<double>();
    q.w = r.read<double>();
    return q;
}

void read_pose(CdrReader& r, LocalPose& p) {  // geometry_msgs/Pose
    p.x = r.read<double>();
    p.y = r.read<double>();
    p.z = r.read<double>();
    p.orientation = read_quat(r);
    p.has_position = p.has_orientation = true;
}

void write_header(CdrWriter& w, const Stamp& s) {  // std_msgs/Header
    // builtin_interfaces/Time: whole seconds (floor) and the nanoseconds after them.
    std::int64_t sec = s.ns / 1000000000, nsec = s.ns % 1000000000;
    if (nsec < 0) {
        nsec += 1000000000;
        --sec;
    }
    w.write<std::int32_t>(static_cast<std::int32_t>(sec));
    w.write<std::uint32_t>(static_cast<std::uint32_t>(nsec));
    w.string(s.frame_id);
}

void write_quat(CdrWriter& w, const Quat& q) {
    w.write(q.x);
    w.write(q.y);
    w.write(q.z);
    w.write(q.w);
}

const char* const kSeparator =
    "\n================================================================================\n";

const char* const kHeaderDef = "builtin_interfaces/Time stamp\nstring frame_id\n";
const char* const kTimeDef = "int32 sec\nuint32 nanosec\n";
const char* const kQuaternionDef = "float64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n";

std::string with_dependencies(const std::string& main,
                              const std::vector<std::pair<const char*, const char*>>& dependencies) {
    std::string s = main;
    for (const auto& [name, text] : dependencies) {
        s += kSeparator;
        s += "MSG: ";
        s += name;
        s += "\n";
        s += text;
    }
    return s;
}

}  // namespace

MessageKind message_kind(const std::string& type) {
    if (type == "sensor_msgs/msg/NavSatFix") return MessageKind::NavSatFix;
    if (type == "geometry_msgs/msg/PoseStamped" || type == "geometry_msgs/msg/PoseWithCovarianceStamped" ||
        type == "nav_msgs/msg/Odometry") {
        return MessageKind::LocalPose;
    }
    if (type == "sensor_msgs/msg/Imu" || type == "geometry_msgs/msg/QuaternionStamped") return MessageKind::Attitude;
    if (type == "geographic_msgs/msg/GeoPoseStamped") return MessageKind::GeoPose;
    return MessageKind::Unknown;
}

NavSatFix decode_navsatfix(const std::uint8_t* data, std::size_t size) {
    CdrReader r(data, size);
    NavSatFix f;
    f.header = read_header(r);
    f.status = r.read<std::int8_t>();
    r.read<std::uint16_t>();  // service
    f.latitude = r.read<double>();
    f.longitude = r.read<double>();
    f.altitude = r.read<double>();
    return f;
}

std::optional<LocalPose> decode_local_pose(const std::string& type, const std::uint8_t* data, std::size_t size) {
    CdrReader r(data, size);
    LocalPose p;
    if (type == "geometry_msgs/msg/PoseStamped" || type == "geometry_msgs/msg/PoseWithCovarianceStamped") {
        p.header = read_header(r);
        read_pose(r, p);
    } else if (type == "nav_msgs/msg/Odometry") {
        p.header = read_header(r);
        r.string();  // child_frame_id
        read_pose(r, p);
    } else if (type == "sensor_msgs/msg/Imu" || type == "geometry_msgs/msg/QuaternionStamped") {
        p.header = read_header(r);
        p.orientation = read_quat(r);
        p.has_orientation = true;
        // An Imu without orientation sets its covariance[0] to -1 (sensor_msgs/Imu).
        if (type == "sensor_msgs/msg/Imu" && r.read<double>() < 0.0) p.has_orientation = false;
    } else {
        return std::nullopt;
    }
    return p;
}

GeoPose decode_geopose(const std::uint8_t* data, std::size_t size) {
    CdrReader r(data, size);
    GeoPose g;
    g.header = read_header(r);
    g.latitude = r.read<double>();
    g.longitude = r.read<double>();
    g.altitude = r.read<double>();
    g.orientation = read_quat(r);
    return g;
}

CameraInfo decode_camera_info(const std::uint8_t* data, std::size_t size) {
    CdrReader r(data, size);
    CameraInfo c;
    c.header = read_header(r);
    c.height = r.read<std::uint32_t>();
    c.width = r.read<std::uint32_t>();
    c.distortion_model = r.string();
    const auto n = r.read<std::uint32_t>();
    if (n > size) throw std::runtime_error("CameraInfo: bad distortion count");
    c.d.resize(n);
    for (auto& v : c.d) v = r.read<double>();
    for (auto& v : c.k) v = r.read<double>();
    for (auto& v : c.r) v = r.read<double>();
    for (auto& v : c.p) v = r.read<double>();
    return c;
}

std::vector<std::uint8_t> encode_compressed_image(const Stamp& header, const std::string& format,
                                                  const std::uint8_t* data, std::size_t size) {
    CdrWriter w;
    write_header(w, header);
    w.string(format);
    w.bytes(data, size);
    return w.take();
}

std::vector<std::uint8_t> encode_camera_info(const CameraInfo& c) {
    CdrWriter w;
    write_header(w, c.header);
    w.write(c.height);
    w.write(c.width);
    w.string(c.distortion_model);
    w.write<std::uint32_t>(static_cast<std::uint32_t>(c.d.size()));
    for (double v : c.d) w.write(v);
    for (double v : c.k) w.write(v);
    for (double v : c.r) w.write(v);
    for (double v : c.p) w.write(v);
    w.write<std::uint32_t>(0);  // binning_x
    w.write<std::uint32_t>(0);  // binning_y
    for (int i = 0; i < 4; ++i) w.write<std::uint32_t>(0);  // roi: x_offset, y_offset, height, width
    w.write<std::uint8_t>(0);  // roi.do_rectify
    return w.take();
}

std::vector<std::uint8_t> encode_tf_message(const std::vector<Transform>& transforms) {
    CdrWriter w;
    w.write<std::uint32_t>(static_cast<std::uint32_t>(transforms.size()));
    for (const Transform& t : transforms) {
        write_header(w, t.header);
        w.string(t.child_frame_id);
        w.write(t.x);
        w.write(t.y);
        w.write(t.z);
        write_quat(w, t.rotation);
    }
    return w.take();
}

std::vector<std::uint8_t> encode_geopose(const GeoPose& g) {
    CdrWriter w;
    write_header(w, g.header);
    w.write(g.latitude);
    w.write(g.longitude);
    w.write(g.altitude);
    write_quat(w, g.orientation);
    return w.take();
}

std::string type_hash(const std::string& type) {
    // SHA-256 of each type's description (its fields and those of the types it
    // uses), as ROS 2 computes it since its type hashes were introduced. The
    // computation was checked against the hashes a recorder stored for other types.
    if (type == "sensor_msgs/msg/CompressedImage") return "RIHS01_15640771531571185e2efc8a100baf923961a4d15d5569652e6cb6691e8e371a";
    if (type == "sensor_msgs/msg/CameraInfo") return "RIHS01_b3dfd68ff46c9d56c80fd3bd4ed22c7a4ddce8c8348f2f59c299e73118e7e275";
    if (type == "tf2_msgs/msg/TFMessage") return "RIHS01_e369d0f05a23ae52508854b66f6aa0437f3449d652e8cbf22d5abe85d020f087";
    if (type == "geographic_msgs/msg/GeoPoseStamped") return "RIHS01_a8e603c050fa992a74283bb462a96fe9d2b151b7a67b14d77250ed7cb4ed96b7";
    return {};
}

std::string message_definition(const std::string& type) {
    if (type == "sensor_msgs/msg/CompressedImage") {
        return with_dependencies("std_msgs/Header header\nstring format\nuint8[] data\n",
                                 {{"std_msgs/Header", kHeaderDef}, {"builtin_interfaces/Time", kTimeDef}});
    }
    if (type == "sensor_msgs/msg/CameraInfo") {
        return with_dependencies(
            "std_msgs/Header header\nuint32 height\nuint32 width\nstring distortion_model\nfloat64[] d\n"
            "float64[9] k\nfloat64[9] r\nfloat64[12] p\nuint32 binning_x\nuint32 binning_y\n"
            "RegionOfInterest roi\n",
            {{"std_msgs/Header", kHeaderDef},
             {"builtin_interfaces/Time", kTimeDef},
             {"sensor_msgs/RegionOfInterest",
              "uint32 x_offset\nuint32 y_offset\nuint32 height\nuint32 width\nbool do_rectify\n"}});
    }
    if (type == "tf2_msgs/msg/TFMessage") {
        return with_dependencies(
            "geometry_msgs/TransformStamped[] transforms\n",
            {{"geometry_msgs/TransformStamped", "std_msgs/Header header\nstring child_frame_id\nTransform transform\n"},
             {"std_msgs/Header", kHeaderDef},
             {"builtin_interfaces/Time", kTimeDef},
             {"geometry_msgs/Transform", "Vector3 translation\nQuaternion rotation\n"},
             {"geometry_msgs/Vector3", "float64 x\nfloat64 y\nfloat64 z\n"},
             {"geometry_msgs/Quaternion", kQuaternionDef}});
    }
    if (type == "geographic_msgs/msg/GeoPoseStamped") {
        return with_dependencies(
            "std_msgs/Header header\ngeographic_msgs/GeoPose pose\n",
            {{"std_msgs/Header", kHeaderDef},
             {"builtin_interfaces/Time", kTimeDef},
             {"geographic_msgs/GeoPose", "GeoPoint position\ngeometry_msgs/Quaternion orientation\n"},
             {"geographic_msgs/GeoPoint", "float64 latitude\nfloat64 longitude\nfloat64 altitude\n"},
             {"geometry_msgs/Quaternion", kQuaternionDef}});
    }
    return {};
}

}  // namespace ogccdb::flight
