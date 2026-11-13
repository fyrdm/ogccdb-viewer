#pragma once
// A vehicle's flight from a ROS 2 bag: position and attitude over time, in
// ECEF, for driving a camera or drawing the flight path.
#include <cstdint>
#include <string>
#include <vector>

#include "ogccdb/flight/bag.hpp"
#include "ogccdb/flight/messages.hpp"
#include "ogccdb/runtime/geodesy.hpp"
#include "ogccdb/runtime/math.hpp"

namespace ogccdb::flight {

using runtime::Geodetic;
using runtime::Vec3d;

/// Rotates `v` by the unit quaternion `q`.
Vec3d rotate(const Quat& q, Vec3d v);
/// Spherical interpolation between unit quaternions (shortest arc).
Quat slerp(Quat a, Quat b, double t);

struct FlightPose {
    double t = 0.0;  ///< seconds from the start of the bag
    Vec3d position;  ///< ECEF
    Geodetic geodetic;
    bool has_attitude = false;
    Quat attitude;            ///< body (x forward, y left, z up) to local ENU
    Vec3d forward, left, up;  ///< body axes in ECEF
    double roll = 0.0, pitch = 0.0, yaw = 0.0;  ///< degrees; yaw clockwise from north
    double ground_speed = 0.0, climb = 0.0;     ///< m/s
};

/// A topic can say in its metadata that it is somebody's estimate of the pose
/// (a prior, a filter output) and not a measurement of it. Such a topic is
/// never picked as the flight's position or attitude unless it is named.
inline constexpr const char* kRoleKey = "ogccdb_role";
inline constexpr const char* kRoleEstimate = "estimate";
bool is_estimate(const BagTopic& topic);

struct TrackOptions {
    std::string fix_topic;       ///< NavSatFix or GeoPoseStamped; empty = the busiest one
    std::string local_topic;     ///< local pose in an ENU frame; empty = the busiest one, "-" = none
    std::string attitude_topic;  ///< empty = from the local pose, else Imu or QuaternionStamped
    double altitude_offset = 0.0;  ///< metres added to every height
};

struct TrackInfo {
    std::string fix_topic, local_topic, attitude_topic;
    std::string position_source;  ///< how positions were made, for display
    std::size_t fixes = 0, local_poses = 0, attitudes = 0;
    double anchor_rms = 0.0;  ///< metres between the fixes and the anchored local track
    Geodetic origin;          ///< geodetic origin of the local frame
};

class FlightTrack {
public:
    /// Reads the bag's position and attitude topics; throws std::runtime_error
    /// if it has no usable geodetic position.
    static FlightTrack build(Bag& bag, const TrackOptions& options = {});

    bool empty() const { return positions_.empty(); }
    double start() const { return positions_.empty() ? 0.0 : positions_.front().t; }
    double end() const { return positions_.empty() ? 0.0 : positions_.back().t; }
    double duration() const { return end() - start(); }  ///< of the track
    double bag_duration() const { return bag_duration_; }
    FlightPose at(double t) const;
    /// ECEF positions in time order, for drawing the path.
    const std::vector<Vec3d>& path() const { return path_; }
    /// Time of each path() point, seconds from the start of the bag.
    const std::vector<double>& path_times() const { return path_times_; }
    const TrackInfo& info() const { return info_; }

    /// Log times (seconds from the start of the bag) of each bag topic's
    /// messages, in the order of Bag::topics(), for showing playback progress.
    const std::vector<std::vector<double>>& topic_times() const { return topic_times_; }

private:
    struct PositionSample {
        double t;
        Vec3d p;
    };
    struct AttitudeSample {
        double t;
        Quat q;
    };
    std::vector<PositionSample> positions_;
    std::vector<AttitudeSample> attitudes_;
    std::vector<Vec3d> path_;
    std::vector<double> path_times_;
    std::vector<std::vector<double>> topic_times_;
    double bag_duration_ = 0.0;
    TrackInfo info_;
};

/// Axes of a camera mounted on the body: `mount_pitch_deg` -90 looks straight
/// down (image top toward the body's forward), 0 looks forward.
void camera_axes(const FlightPose& pose, double mount_pitch_deg, Vec3d& forward, Vec3d& up);

}  // namespace ogccdb::flight
