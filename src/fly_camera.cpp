#include "fly_camera.hpp"

#include <algorithm>
#include <cmath>

#include "ogccdb/runtime/geodesy.hpp"

using namespace ogccdb::runtime;

namespace {
Geodetic where(const FlyCamera& c) { return ecef_to_geodetic(c.position); }
}  // namespace

Vec3d FlyCamera::local_up() const {
    const Geodetic g = where(*this);
    return ellipsoid_up(g.lat, g.lon);
}

Vec3d FlyCamera::forward() const {
    const Geodetic g = where(*this);
    const double h = heading * kDegToRad, p = pitch * kDegToRad;
    return local_east(g.lat, g.lon) * (std::sin(h) * std::cos(p)) +
           local_north(g.lat, g.lon) * (std::cos(h) * std::cos(p)) + ellipsoid_up(g.lat, g.lon) * std::sin(p);
}

Vec3d FlyCamera::right() const {
    const Geodetic g = where(*this);
    const double h = heading * kDegToRad;
    return local_east(g.lat, g.lon) * std::cos(h) - local_north(g.lat, g.lon) * std::sin(h);
}

Vec3d FlyCamera::up() const { return normalize(cross(right(), forward())); }

double FlyCamera::speed(double height_above_ground) const {
    return std::clamp(std::fabs(height_above_ground), 5.0, 2.0e6) * speed_factor;
}

void FlyCamera::move(double ahead, double side, double rise, double metres_per_second, double dt, bool level) {
    const double step = metres_per_second * dt;
    Vec3d dir = forward();
    if (level) {
        const Geodetic g = where(*this);
        const double h = heading * kDegToRad;
        dir = local_east(g.lat, g.lon) * std::sin(h) + local_north(g.lat, g.lon) * std::cos(h);
    }
    position += dir * (ahead * step) + right() * (side * step) + local_up() * (rise * step);
}

void FlyCamera::set_height(double height) {
    const Geodetic g = where(*this);
    position = geodetic_to_ecef({g.lat, g.lon, height});
}

void FlyCamera::look(double dheading, double dpitch) {
    heading += dheading;
    pitch += dpitch;
    clamp();
}

void FlyCamera::clamp() {
    heading = std::fmod(std::fmod(heading, 360.0) + 360.0, 360.0);
    pitch = std::clamp(pitch, -89.9, 89.9);
    speed_factor = std::clamp(speed_factor, 0.01, 100.0);
}
