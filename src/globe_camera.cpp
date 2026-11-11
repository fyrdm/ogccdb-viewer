#include "globe_camera.hpp"

#include <algorithm>
#include <cmath>

#include "ogccdb/runtime/geodesy.hpp"

using namespace ogccdb::runtime;

Vec3d GlobeCamera::target_ecef() const { return geodetic_to_ecef({lat, lon, height}); }

Vec3d GlobeCamera::forward() const {
    const double h = heading * kDegToRad, p = pitch * kDegToRad;
    return local_east(lat, lon) * (std::sin(h) * std::cos(p)) + local_north(lat, lon) * (std::cos(h) * std::cos(p)) +
           ellipsoid_up(lat, lon) * std::sin(p);
}

Vec3d GlobeCamera::right() const {
    const double h = heading * kDegToRad;
    return local_east(lat, lon) * std::cos(h) - local_north(lat, lon) * std::sin(h);
}

Vec3d GlobeCamera::eye() const { return target_ecef() - forward() * distance; }

Vec3d GlobeCamera::up() const { return normalize(cross(right(), forward())); }

void GlobeCamera::pan_pixels(double dx, double dy, int viewport_height) {
    const double metres_per_pixel = distance * 2.0 * std::tan(fov_y_deg * kDegToRad / 2.0) / std::max(viewport_height, 1);
    const double h = heading * kDegToRad;
    // Content follows the mouse: the target moves against the drag.
    const double right_m = -dx * metres_per_pixel;
    const double ahead_m = dy * metres_per_pixel / std::max(0.2, std::sin(-pitch * kDegToRad));
    const double east = right_m * std::cos(h) + ahead_m * std::sin(h);
    const double north = -right_m * std::sin(h) + ahead_m * std::cos(h);
    lat += north / kMeanEarthRadius * kRadToDeg;
    lon += east / (kMeanEarthRadius * std::max(0.01, std::cos(lat * kDegToRad))) * kRadToDeg;
    clamp();
}

void GlobeCamera::rotate(double dheading, double dpitch) {
    heading += dheading;
    pitch += dpitch;
    clamp();
}

void GlobeCamera::zoom(double steps) {
    distance *= std::pow(0.85, steps);
    clamp();
}

void GlobeCamera::clamp() {
    lat = std::clamp(lat, -89.9, 89.9);
    if (lon > 180.0) lon -= 360.0;
    if (lon < -180.0) lon += 360.0;
    heading = std::fmod(heading + 360.0, 360.0);
    pitch = std::clamp(pitch, -89.9, -1.0);
    distance = std::clamp(distance, 10.0, 3.0e7);
}

Camera make_camera(Vec3d eye, Vec3d forward, Vec3d up, double fov_y_deg, int width, int height_px,
                   double ground_height, double max_near) {
    Camera c;
    c.position = eye;
    c.direction = forward;
    c.up = up;
    c.fov_y = fov_y_deg * kDegToRad;
    c.aspect = static_cast<double>(std::max(width, 1)) / std::max(height_px, 1);
    c.viewport_height = std::max(height_px, 1);
    // Distance to the ground, above or below it (the eye may be underground).
    const double to_ground = std::max(std::fabs(ecef_to_geodetic(eye).height - ground_height), 1.0);
    c.near_plane = std::clamp(std::min(to_ground * 0.05, max_near), 0.1, 5.0e4);
    const double r = length(eye);
    const double horizon = std::sqrt(std::max(r * r - kWgs84B * kWgs84B, 0.0));
    c.far_plane = std::max(std::min(r + 1.0e5, horizon + 5.0e5), c.near_plane * 10.0);
    return c;
}

Camera GlobeCamera::to_runtime(int width, int height_px, double ground_height) const {
    return make_camera(eye(), forward(), up(), fov_y_deg, width, height_px, ground_height, distance * 0.1);
}
