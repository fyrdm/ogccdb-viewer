#pragma once
// Orbit camera around a target point on the earth, in ECEF.
#include "ogccdb/runtime/selection.hpp"

struct GlobeCamera {
    double lat = 47.5;       ///< target, degrees
    double lon = 8.5;
    double height = 0.0;     ///< target height above the ellipsoid, metres
    double distance = 1.5e5; ///< eye to target, metres
    double heading = 0.0;    ///< degrees clockwise from north
    double pitch = -60.0;    ///< degrees; -90 looks straight down
    double fov_y_deg = 45.0;

    ogccdb::runtime::Vec3d target_ecef() const;
    ogccdb::runtime::Vec3d forward() const;
    ogccdb::runtime::Vec3d right() const;
    ogccdb::runtime::Vec3d eye() const;
    ogccdb::runtime::Vec3d up() const;

    /// Moves the target by screen pixels (drag), scaled to the view.
    void pan_pixels(double dx, double dy, int viewport_height);
    void rotate(double dheading, double dpitch);
    void zoom(double steps);
    void clamp();

    /// Runtime camera for a viewport; near and far planes follow the eye height.
    ogccdb::runtime::Camera to_runtime(int width, int height, double ground_height) const;
};
