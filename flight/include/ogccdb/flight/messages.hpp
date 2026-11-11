#pragma once
// Decoders for the standard ROS 2 messages that carry a vehicle's position
// and attitude. Frames follow ROS REP 103/105: world ENU (x east, y north,
// z up), body FLU (x forward, y left, z up).
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace ogccdb::flight {

struct Quat {
    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
};

struct Stamp {
    std::int64_t ns = 0;  ///< header stamp; 0 if unset
    std::string frame_id;
};

/// sensor_msgs/msg/NavSatFix. Altitude is above the WGS-84 ellipsoid.
struct NavSatFix {
    Stamp header;
    int status = -1;  ///< -1 no fix, 0 fix, 1 SBAS, 2 GBAS
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;
};

/// Pose in a local frame: geometry_msgs PoseStamped, PoseWithCovarianceStamped,
/// nav_msgs/Odometry, or attitude only (sensor_msgs/Imu, QuaternionStamped).
struct LocalPose {
    Stamp header;
    bool has_position = false;
    double x = 0.0, y = 0.0, z = 0.0;
    bool has_orientation = false;
    Quat orientation;
};

/// geographic_msgs/msg/GeoPoseStamped.
struct GeoPose {
    Stamp header;
    double latitude = 0.0, longitude = 0.0, altitude = 0.0;
    Quat orientation;
};

/// What a message type can give.
enum class MessageKind { Unknown, NavSatFix, LocalPose, Attitude, GeoPose };
MessageKind message_kind(const std::string& type);

/// Decoders for CDR messages; throw std::runtime_error on short data.
NavSatFix decode_navsatfix(const std::uint8_t* data, std::size_t size);
/// Any LocalPose or Attitude type; nullopt for other types.
std::optional<LocalPose> decode_local_pose(const std::string& type, const std::uint8_t* data, std::size_t size);
GeoPose decode_geopose(const std::uint8_t* data, std::size_t size);

}  // namespace ogccdb::flight
