#include "ogccdb/flight/track.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>

namespace ogccdb::flight {

using runtime::ecef_to_geodetic;
using runtime::ellipsoid_up;
using runtime::geodetic_to_ecef;
using runtime::kDegToRad;
using runtime::kRadToDeg;
using runtime::local_east;
using runtime::local_north;

Vec3d rotate(const Quat& q, Vec3d v) {
    // v' = v + 2 w (u x v) + 2 u x (u x v), u = (x, y, z)
    const Vec3d u{q.x, q.y, q.z};
    const Vec3d t = runtime::cross(u, v) * 2.0;
    return v + t * q.w + runtime::cross(u, t);
}

Quat slerp(Quat a, Quat b, double t) {
    double d = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    if (d < 0.0) {  // q and -q are the same rotation: take the short way
        b = {-b.x, -b.y, -b.z, -b.w};
        d = -d;
    }
    double wa = 1.0 - t, wb = t;
    if (d < 0.9995) {
        const double th = std::acos(d), s = std::sin(th);
        wa = std::sin((1.0 - t) * th) / s;
        wb = std::sin(t * th) / s;
    }
    Quat q{a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb};
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return n > 0.0 ? Quat{q.x / n, q.y / n, q.z / n, q.w / n} : Quat{};
}

namespace {

struct Fix {
    double t;
    Geodetic g;
};
struct Local {
    double t;
    double x, y, z;
};

double seconds(std::int64_t ns, std::int64_t t0) { return static_cast<double>(ns - t0) * 1e-9; }

/// Header stamp if the publisher set one, else the log time.
std::int64_t message_time(const Stamp& s, std::int64_t log_time) { return s.ns > 0 ? s.ns : log_time; }

const BagTopic* busiest(const Bag& bag, MessageKind kind) {
    const BagTopic* best = nullptr;
    for (const BagTopic& t : bag.topics()) {
        if (message_kind(t.type) == kind && (!best || t.count > best->count)) best = &t;
    }
    return best;
}

template <typename S>
std::vector<S> sorted(std::vector<S> v) {
    std::stable_sort(v.begin(), v.end(), [](const S& a, const S& b) { return a.t < b.t; });
    return v;
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(v.size() / 2), v.end());
    return v[v.size() / 2];
}

}  // namespace

FlightTrack FlightTrack::build(Bag& bag, const TrackOptions& opt) {
    FlightTrack track;
    TrackInfo& info = track.info_;
    const std::int64_t t0 = bag.start_ns();
    track.bag_duration_ = seconds(bag.end_ns(), t0);

    // Topics: the named ones, else the busiest of each kind.
    const BagTopic* fix = nullptr;
    if (!opt.fix_topic.empty()) {
        fix = bag.topic(opt.fix_topic);
        if (!fix) throw std::runtime_error("no topic " + opt.fix_topic + " in the bag");
    } else {
        fix = busiest(bag, MessageKind::GeoPose);
        if (!fix) fix = busiest(bag, MessageKind::NavSatFix);
    }
    if (!fix) throw std::runtime_error("the bag has no NavSatFix or GeoPoseStamped topic for the position");
    const BagTopic* local = nullptr;
    if (opt.local_topic != "-") local = opt.local_topic.empty() ? busiest(bag, MessageKind::LocalPose) : bag.topic(opt.local_topic);
    const BagTopic* att = opt.attitude_topic.empty() ? nullptr : bag.topic(opt.attitude_topic);
    if (!att && message_kind(fix->type) == MessageKind::GeoPose) att = fix;
    if (!att && local) att = local;
    if (!att) att = busiest(bag, MessageKind::Attitude);
    info.fix_topic = fix->name;
    info.local_topic = local ? local->name : "";
    info.attitude_topic = att ? att->name : "";

    std::vector<Fix> fixes;
    std::vector<Local> locals;
    std::vector<AttitudeSample> attitudes;
    std::map<const BagTopic*, std::size_t> index;
    for (std::size_t i = 0; i < bag.topics().size(); ++i) index[&bag.topics()[i]] = i;
    track.topic_times_.assign(bag.topics().size(), {});

    bag.read([&](const BagMessage& m) {
        track.topic_times_[index[m.topic]].push_back(seconds(m.log_time_ns, t0));
        try {
            if (m.topic == fix) {
                if (message_kind(fix->type) == MessageKind::GeoPose) {
                    const GeoPose g = decode_geopose(m.data, m.size);
                    fixes.push_back({seconds(message_time(g.header, m.log_time_ns), t0), {g.latitude, g.longitude, g.altitude}});
                } else {
                    const NavSatFix f = decode_navsatfix(m.data, m.size);
                    if (f.status >= 0 && std::isfinite(f.latitude) && std::isfinite(f.longitude) && std::isfinite(f.altitude)) {
                        fixes.push_back({seconds(message_time(f.header, m.log_time_ns), t0), {f.latitude, f.longitude, f.altitude}});
                    }
                }
            }
            if (m.topic == local || (m.topic == att && att != fix)) {
                if (const auto p = decode_local_pose(m.topic->type, m.data, m.size)) {
                    const double t = seconds(message_time(p->header, m.log_time_ns), t0);
                    if (m.topic == local && p->has_position) locals.push_back({t, p->x, p->y, p->z});
                    if (m.topic == att && p->has_orientation) attitudes.push_back({t, p->orientation});
                }
            }
            if (m.topic == att && att == fix && message_kind(fix->type) == MessageKind::GeoPose) {
                const GeoPose g = decode_geopose(m.data, m.size);
                attitudes.push_back({seconds(message_time(g.header, m.log_time_ns), t0), g.orientation});
            }
        } catch (const std::runtime_error&) {
            // A malformed message: skip it.
        }
    }, [&](const BagTopic&) { return true; });

    for (auto& f : fixes) f.g.height += opt.altitude_offset;
    fixes = sorted(std::move(fixes));
    locals = sorted(std::move(locals));
    track.attitudes_ = sorted(std::move(attitudes));
    info.fixes = fixes.size();
    info.local_poses = locals.size();
    info.attitudes = track.attitudes_.size();
    if (fixes.empty()) throw std::runtime_error("no valid fixes on " + fix->name);

    auto fix_at = [&](double t) {  // ECEF of the fixes, linear in time
        auto it = std::lower_bound(fixes.begin(), fixes.end(), t, [](const Fix& f, double v) { return f.t < v; });
        if (it == fixes.begin()) return geodetic_to_ecef(it->g);
        if (it == fixes.end()) return geodetic_to_ecef(fixes.back().g);
        const Fix& a = *(it - 1);
        const Fix& b = *it;
        const double k = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0.0;
        return geodetic_to_ecef(a.g) * (1.0 - k) + geodetic_to_ecef(b.g) * k;
    };

    if (locals.size() >= 2) {
        // The local frame is ENU at an unknown origin: find the origin that
        // best puts the local track on the fixes (median, robust to outliers).
        info.origin = fixes.front().g;
        const Vec3d e = local_east(info.origin.lat, info.origin.lon), n = local_north(info.origin.lat, info.origin.lon),
                    u = ellipsoid_up(info.origin.lat, info.origin.lon);
        std::vector<double> ox, oy, oz;
        for (const Local& l : locals) {
            if (l.t < fixes.front().t || l.t > fixes.back().t) continue;
            const Vec3d o = fix_at(l.t) - (e * l.x + n * l.y + u * l.z);
            ox.push_back(o.x);
            oy.push_back(o.y);
            oz.push_back(o.z);
        }
        if (!ox.empty()) {
            const Vec3d origin{median(ox), median(oy), median(oz)};
            info.origin = ecef_to_geodetic(origin);
            double sum = 0.0;
            std::size_t count = 0;
            for (const Local& l : locals) {
                const Vec3d p = origin + e * l.x + n * l.y + u * l.z;
                track.positions_.push_back({l.t, p});
                if (l.t >= fixes.front().t && l.t <= fixes.back().t) {
                    const Vec3d d = p - fix_at(l.t);
                    sum += runtime::dot(d, d);
                    ++count;
                }
            }
            info.anchor_rms = count ? std::sqrt(sum / count) : 0.0;
            info.position_source = "local pose, anchored to the fixes";
        }
    }
    if (track.positions_.empty()) {
        for (const Fix& f : fixes) track.positions_.push_back({f.t, geodetic_to_ecef(f.g)});
        info.position_source = "fixes";
        info.local_topic.clear();
    }
    track.path_.reserve(track.positions_.size());
    track.path_times_.reserve(track.positions_.size());
    for (const auto& p : track.positions_) {
        track.path_.push_back(p.p);
        track.path_times_.push_back(p.t);
    }
    return track;
}

FlightPose FlightTrack::at(double t) const {
    FlightPose pose;
    pose.t = t;
    if (positions_.empty()) return pose;
    auto position = [&](double tt) {
        tt = std::clamp(tt, start(), end());
        auto it = std::lower_bound(positions_.begin(), positions_.end(), tt,
                                   [](const PositionSample& s, double v) { return s.t < v; });
        if (it == positions_.begin()) return it->p;
        if (it == positions_.end()) return positions_.back().p;
        const auto& a = *(it - 1);
        const auto& b = *it;
        const double k = b.t > a.t ? (tt - a.t) / (b.t - a.t) : 0.0;
        return a.p * (1.0 - k) + b.p * k;
    };
    pose.position = position(t);
    pose.geodetic = ecef_to_geodetic(pose.position);
    const double lat = pose.geodetic.lat, lon = pose.geodetic.lon;
    const Vec3d e = local_east(lat, lon), n = local_north(lat, lon), u = ellipsoid_up(lat, lon);

    // Speeds over half a second either side.
    const Vec3d v = (position(t + 0.5) - position(t - 0.5)) / std::max(std::min(t + 0.5, end()) - std::max(t - 0.5, start()), 1e-3);
    pose.ground_speed = std::hypot(runtime::dot(v, e), runtime::dot(v, n));
    pose.climb = runtime::dot(v, u);

    if (!attitudes_.empty()) {
        const double tt = std::clamp(t, attitudes_.front().t, attitudes_.back().t);
        auto it = std::lower_bound(attitudes_.begin(), attitudes_.end(), tt,
                                   [](const AttitudeSample& s, double v2) { return s.t < v2; });
        if (it == attitudes_.begin()) {
            pose.attitude = it->q;
        } else if (it == attitudes_.end()) {
            pose.attitude = attitudes_.back().q;
        } else {
            const auto& a = *(it - 1);
            const auto& b = *it;
            pose.attitude = slerp(a.q, b.q, b.t > a.t ? (tt - a.t) / (b.t - a.t) : 0.0);
        }
        pose.has_attitude = true;
    } else {
        // No attitude: level, nose along the track (north when hovering).
        const double h = std::atan2(runtime::dot(v, n), runtime::dot(v, e));  // ENU yaw from east
        const double yaw = pose.ground_speed > 0.5 ? h : runtime::kPi / 2;
        pose.attitude = {0.0, 0.0, std::sin(yaw / 2), std::cos(yaw / 2)};
    }
    // Body axes: ENU components of the rotated body axes, then to ECEF.
    auto to_ecef = [&](Vec3d enu) { return e * enu.x + n * enu.y + u * enu.z; };
    pose.forward = to_ecef(rotate(pose.attitude, {1, 0, 0}));
    pose.left = to_ecef(rotate(pose.attitude, {0, 1, 0}));
    pose.up = to_ecef(rotate(pose.attitude, {0, 0, 1}));
    // Aviation angles: heading clockwise from north, pitch nose up, roll right wing down.
    pose.yaw = std::atan2(runtime::dot(pose.forward, e), runtime::dot(pose.forward, n)) * kRadToDeg;
    if (pose.yaw < 0) pose.yaw += 360.0;
    pose.pitch = std::asin(std::clamp(runtime::dot(pose.forward, u), -1.0, 1.0)) * kRadToDeg;
    pose.roll = std::atan2(runtime::dot(pose.left, u), runtime::dot(pose.up, u)) * kRadToDeg;
    return pose;
}

void camera_axes(const FlightPose& p, double mount_pitch_deg, Vec3d& forward, Vec3d& up) {
    // Tilt the body's forward axis about its left axis: -90 deg points it down
    // the body's -z; the image's top then points along the body's forward.
    const double a = mount_pitch_deg * kDegToRad;
    forward = runtime::normalize(p.forward * std::cos(a) + p.up * std::sin(a));
    up = runtime::normalize(p.up * std::cos(a) - p.forward * std::sin(a));
}

}  // namespace ogccdb::flight
