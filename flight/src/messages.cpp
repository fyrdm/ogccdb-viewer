#include "ogccdb/flight/messages.hpp"

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

}  // namespace ogccdb::flight
