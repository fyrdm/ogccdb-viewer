#pragma once
// Runtime cameras and terrain heights for drawing.
#include <optional>
#include <vector>

#include "ogccdb/runtime/selection.hpp"
#include "ogccdb/runtime/terrain.hpp"

namespace ogccdb::render {

/// Runtime camera at `eye` for a viewport. Near and far planes follow the
/// distance to the ground, above or below it; `max_near` caps the near plane.
runtime::Camera make_camera(runtime::Vec3d eye, runtime::Vec3d forward, runtime::Vec3d up, double fov_y_deg, int width,
                            int height, double ground_height, double max_near);

/// Terrain height at (lat, lon) from the finest drawn tile over it, by bilinear
/// interpolation of its mesh grid (`resolution` cells a side); nullopt if no
/// drawn tile covers it. `lod`, if given, receives that tile's LOD.
std::optional<double> ground_height(const std::vector<runtime::DrawTile>& tiles, double lat, double lon, int resolution,
                                    int* lod = nullptr);

}  // namespace ogccdb::render
