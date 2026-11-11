#pragma once
// Free-fly camera: a position anywhere (also below the terrain) with a view
// direction given as heading and pitch in the local east-north-up frame.
#include "ogccdb/runtime/selection.hpp"

struct FlyCamera {
    ogccdb::runtime::Vec3d position;  ///< ECEF metres
    double heading = 0.0;             ///< degrees clockwise from north
    double pitch = 0.0;               ///< degrees; +90 looks straight up, -90 straight down
    double speed_factor = 1.0;        ///< speed in heights-above-ground per second
    double fov_y_deg = 45.0;

    ogccdb::runtime::Vec3d forward() const;
    ogccdb::runtime::Vec3d right() const;
    ogccdb::runtime::Vec3d up() const;
    ogccdb::runtime::Vec3d local_up() const;

    /// Speed in m/s at a height above (or below) the ground.
    double speed(double height_above_ground) const;
    /// Moves along the view (`ahead`), sideways (`side`) and the local vertical
    /// (`rise`), each in -1..1 times `speed` for `dt` seconds.
    void move(double ahead, double side, double rise, double speed, double dt);
    void look(double dheading, double dpitch);
    void clamp();
};
