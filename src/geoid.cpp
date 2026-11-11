#include "geoid.hpp"

#include <algorithm>
#include <cmath>

#include <cpl_error.h>
#include <ogr_spatialref.h>

struct Geoid::Impl {
    OGRSpatialReference src;
    OGRSpatialReference dst;
    std::unique_ptr<OGRCoordinateTransformation> ct;
    bool tried = false;
    bool failed = false;
};

Geoid::Geoid(int vertical_epsg) : impl_(new Impl) {
    impl_->src.importFromEPSG(4979);  // WGS-84 geographic 3D: ellipsoid heights
    impl_->src.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    OGRSpatialReference horizontal, vertical;
    horizontal.importFromEPSG(4326);
    vertical.importFromEPSG(vertical_epsg);
    impl_->dst.SetCompoundCS("WGS 84 + geoid", &horizontal, &vertical);
    impl_->dst.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
}

Geoid::~Geoid() = default;

bool Geoid::available() {
    if (!impl_->tried) {
        impl_->tried = true;
        OGRCoordinateTransformationOptions opt;
        opt.SetOnlyBest(true);  // fail instead of silently ignoring a missing grid
        CPLPushErrorHandler(CPLQuietErrorHandler);
        impl_->ct.reset(OGRCreateCoordinateTransformation(&impl_->src, &impl_->dst, opt));
        CPLPopErrorHandler();
        if (!impl_->ct) {
            impl_->failed = true;
            error_ = "PROJ has no transformation to this geoid";
        } else if (!node(0, 0)) {  // a grid that is listed but not installed fails here
            impl_->failed = true;
            error_ = "the geoid grid is not installed";
        }
    }
    return !impl_->failed;
}

std::optional<float> Geoid::node(int i, int j) {
    const int nlon = static_cast<int>(std::lround(360.0 / kStep));
    j = ((j % nlon) + nlon) % nlon;
    const std::int64_t key = static_cast<std::int64_t>(i) * nlon + j;
    if (auto it = nodes_.find(key); it != nodes_.end()) return it->second;
    double x = j * kStep, y = std::clamp(i * kStep, -90.0, 90.0), z = 0.0;
    if (x > 180.0) x -= 360.0;
    int ok = 0;
    CPLPushErrorHandler(CPLQuietErrorHandler);
    const bool done = impl_->ct->Transform(1, &x, &y, &z, nullptr, &ok);
    CPLPopErrorHandler();
    if (!done || !ok || z == 0.0) return std::nullopt;
    const float n = static_cast<float>(-z);  // ellipsoid height 0 is orthometric height -N
    nodes_.emplace(key, n);
    return n;
}

std::optional<double> Geoid::height(double lat, double lon) {
    if (!available()) return std::nullopt;
    const double y = lat / kStep, x = lon / kStep;
    const int i0 = static_cast<int>(std::floor(y)), j0 = static_cast<int>(std::floor(x));
    const double fy = y - i0, fx = x - j0;
    const auto a = node(i0, j0), b = node(i0, j0 + 1), c = node(i0 + 1, j0), d = node(i0 + 1, j0 + 1);
    if (!a || !b || !c || !d) return std::nullopt;
    return (*a * (1 - fx) + *b * fx) * (1 - fy) + (*c * (1 - fx) + *d * fx) * fy;
}
