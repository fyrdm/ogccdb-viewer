#pragma once
// Decoders for the standard ROS 2 messages that carry a vehicle's position
// and attitude. Frames follow ROS REP 103/105: world ENU (x east, y north,
// z up), body FLU (x forward, y left, z up).
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

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

/// sensor_msgs/msg/CameraInfo (a pinhole camera; `d` per `distortion_model`,
/// e.g. "plumb_bob": k1, k2, t1, t2, k3).
struct CameraInfo {
    Stamp header;
    std::uint32_t height = 0, width = 0;
    std::string distortion_model = "plumb_bob";
    std::vector<double> d;
    std::array<double, 9> k{};   ///< row-major 3x3 intrinsics
    std::array<double, 9> r{1, 0, 0, 0, 1, 0, 0, 0, 1};
    std::array<double, 12> p{};  ///< row-major 3x4 projection
};

/// geometry_msgs/msg/TransformStamped: `child_frame_id` relative to header.frame_id.
struct Transform {
    Stamp header;
    std::string child_frame_id;
    double x = 0.0, y = 0.0, z = 0.0;
    Quat rotation;
};

/// What a message type can give.
enum class MessageKind { Unknown, NavSatFix, LocalPose, Attitude, GeoPose };
MessageKind message_kind(const std::string& type);

/// Decoders for CDR messages; throw std::runtime_error on short data.
NavSatFix decode_navsatfix(const std::uint8_t* data, std::size_t size);
/// Any LocalPose or Attitude type; nullopt for other types.
std::optional<LocalPose> decode_local_pose(const std::string& type, const std::uint8_t* data, std::size_t size);
GeoPose decode_geopose(const std::uint8_t* data, std::size_t size);
CameraInfo decode_camera_info(const std::uint8_t* data, std::size_t size);

/// Encoders for messages this library writes, in CDR.
std::vector<std::uint8_t> encode_compressed_image(const Stamp& header, const std::string& format,
                                                  const std::uint8_t* data, std::size_t size);
std::vector<std::uint8_t> encode_camera_info(const CameraInfo& info);
std::vector<std::uint8_t> encode_tf_message(const std::vector<Transform>& transforms);  ///< tf2_msgs/msg/TFMessage
std::vector<std::uint8_t> encode_geopose(const GeoPose& pose);                          ///< GeoPoseStamped

/// Message definition (ros2msg: the .msg text, then each type it uses) of
/// the types the encoders write, for a bag's schema; empty for other types.
std::string message_definition(const std::string& type);
/// Type hash ("RIHS01_" and 64 hex digits) of the same types, as newer bags
/// store with each topic; empty for other types.
std::string type_hash(const std::string& type);

}  // namespace ogccdb::flight
