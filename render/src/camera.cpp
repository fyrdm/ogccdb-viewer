#include "ogccdb/render/camera.hpp"

#include <algorithm>
#include <cmath>

#include "ogccdb/cdb13/lod.hpp"
#include "ogccdb/runtime/geodesy.hpp"

namespace ogccdb::render {

using namespace ogccdb::runtime;

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

std::optional<double> ground_height(const std::vector<DrawTile>& tiles, double lat, double lon, int resolution,
                                    int* lod) {
    const DrawTile* best = nullptr;
    for (const DrawTile& d : tiles) {
        const GeoExtent e = cdb13::extent(d.tile);
        if (lat >= e.south && lat <= e.north && lon >= e.west && lon <= e.east && (!best || d.tile.lod > best->tile.lod)) {
            best = &d;
        }
    }
    if (!best) return std::nullopt;
    if (lod) *lod = best->tile.lod;
    const GeoExtent e = cdb13::extent(best->tile);
    const auto& h = best->data->mesh.heights;
    const int side = resolution + 1;
    if (h.size() < static_cast<std::size_t>(side) * side) return std::nullopt;
    const double x = std::clamp((lon - e.west) / (e.east - e.west), 0.0, 1.0) * resolution;
    const double y = std::clamp((e.north - lat) / (e.north - e.south), 0.0, 1.0) * resolution;
    const int i0 = std::min(static_cast<int>(x), resolution - 1), j0 = std::min(static_cast<int>(y), resolution - 1);
    const double fx = x - i0, fy = y - j0;
    auto at = [&](int i, int j) { return static_cast<double>(h[static_cast<std::size_t>(j) * side + i]); };
    return (at(i0, j0) * (1 - fx) + at(i0 + 1, j0) * fx) * (1 - fy) + (at(i0, j0 + 1) * (1 - fx) + at(i0 + 1, j0 + 1) * fx) * fy;
}

}  // namespace ogccdb::render
