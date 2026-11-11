#pragma once
// Geoid undulation (geoid height above the WGS-84 ellipsoid) through PROJ, for
// the sea-level reference surface. Needs the geoid grid installed (or
// PROJ_NETWORK=ON); without it, available() is false.
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

class Geoid {
public:
    /// `vertical_epsg`: a vertical CRS on the WGS-84 datum, e.g. 3855 (EGM2008) or 5773 (EGM96).
    explicit Geoid(int vertical_epsg);
    ~Geoid();
    Geoid(const Geoid&) = delete;
    Geoid& operator=(const Geoid&) = delete;

    /// True if PROJ can convert to this geoid here (the grid is installed).
    bool available();
    /// Geoid height N in metres above the ellipsoid at (lat, lon) degrees,
    /// bilinear between nodes every kStep degrees that are computed on first use.
    std::optional<double> height(double lat, double lon);
    /// Why the geoid is not available; empty when it is.
    const std::string& error() const { return error_; }

    static constexpr double kStep = 0.125;

private:
    std::optional<float> node(int i, int j);

    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::unordered_map<std::int64_t, float> nodes_;
    std::string error_;
};
