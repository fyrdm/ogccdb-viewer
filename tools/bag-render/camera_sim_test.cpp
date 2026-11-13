#include "camera_sim.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <gtest/gtest.h>

#include "ogccdb/runtime/geodesy.hpp"

using namespace ogccdb::bagrender;
using ogccdb::render::RgbImage;
using ogccdb::runtime::cross;
using ogccdb::runtime::dot;
using ogccdb::runtime::kPi;

namespace {

void expect_vec(Vec3d a, Vec3d b, double tolerance = 1e-12) {
    EXPECT_NEAR(a.x, b.x, tolerance);
    EXPECT_NEAR(a.y, b.y, tolerance);
    EXPECT_NEAR(a.z, b.z, tolerance);
}

TEST(Intrinsics, FromHfovPutsTheFieldOfViewOnTheImageEdges) {
    const Intrinsics k = Intrinsics::from_hfov(1280, 720, 90.0);
    EXPECT_NEAR(k.fx, 640.0, 1e-9);
    EXPECT_DOUBLE_EQ(k.fx, k.fy);
    EXPECT_DOUBLE_EQ(k.cx, 639.5);
    EXPECT_DOUBLE_EQ(k.cy, 359.5);
    // A ray 45 degrees to the right lands on the right edge of the last pixel.
    EXPECT_NEAR(k.project(1.0, 0.0).first, 1279.5, 1e-9);
    EXPECT_FALSE(k.distorted());
}

TEST(Intrinsics, UnprojectInvertsProject) {
    Intrinsics k = Intrinsics::from_hfov(1280, 720, 80.0);
    k.d = {-0.2, 0.05, 0.001, -0.0005, 0.0};  // one-to-one over the whole image
    ASSERT_TRUE(k.distorted());
    for (const auto& [u, v] : {std::pair{0.0, 0.0}, {1279.0, 719.0}, {640.0, 360.0}, {100.5, 650.25}, {1279.0, 0.0}}) {
        double x = 0.0, y = 0.0;
        ASSERT_TRUE(k.unproject(u, v, x, y)) << u << ", " << v;
        const auto [pu, pv] = k.project(x, y);
        EXPECT_NEAR(pu, u, 1e-6);
        EXPECT_NEAR(pv, v, 1e-6);
    }
    // Barrel distortion pulls the corners in: the ray is further out than the pixel suggests.
    double x = 0.0, y = 0.0;
    ASSERT_TRUE(k.unproject(0.0, 0.0, x, y));
    EXPECT_LT(x, (0.0 - k.cx) / k.fx);
}

TEST(Intrinsics, UnprojectRefusesPixelsBeyondTheFold) {
    // r (1 + k1 r^2) has its maximum 0.385 at r = 0.577 for k1 = -1: nothing projects further out.
    Intrinsics k = Intrinsics::from_hfov(1000, 1000, 90.0);
    k.d = {-1.0, 0.0, 0.0, 0.0, 0.0};
    double x = 0.0, y = 0.0;
    EXPECT_TRUE(k.unproject(k.cx + 0.3 * k.fx, k.cy, x, y));
    EXPECT_FALSE(k.unproject(k.cx + 0.5 * k.fx, k.cy, x, y));
}

TEST(Intrinsics, CameraInfoCarriesTheMatrices) {
    Intrinsics k = Intrinsics::from_hfov(640, 480, 60.0);
    k.d = {0.1, 0.0, 0.0, 0.0, 0.0};
    const auto info = k.camera_info();
    EXPECT_EQ(info.width, 640u);
    EXPECT_EQ(info.height, 480u);
    EXPECT_EQ(info.distortion_model, "plumb_bob");
    ASSERT_EQ(info.d.size(), 5u);
    EXPECT_DOUBLE_EQ(info.d[0], 0.1);
    EXPECT_DOUBLE_EQ(info.k[0], k.fx);
    EXPECT_DOUBLE_EQ(info.k[2], k.cx);
    EXPECT_DOUBLE_EQ(info.k[4], k.fy);
    EXPECT_DOUBLE_EQ(info.k[5], k.cy);
    EXPECT_DOUBLE_EQ(info.k[8], 1.0);
    EXPECT_DOUBLE_EQ(info.p[0], k.fx);
    EXPECT_DOUBLE_EQ(info.p[5], k.fy);
    EXPECT_DOUBLE_EQ(info.p[6], k.cy);
    EXPECT_DOUBLE_EQ(info.p[10], 1.0);
    EXPECT_DOUBLE_EQ(info.p[3], 0.0);
}

TEST(DistortionMap, IdentityForAnIdealCentredCamera) {
    const Intrinsics k = Intrinsics::from_hfov(1280, 720, 80.0);
    const DistortionMap m = make_distortion_map(k);
    EXPECT_TRUE(m.identity);
    EXPECT_EQ(m.render_width, 1280);
    EXPECT_EQ(m.render_height, 720);
    // Vertical field of view of the same focal length.
    EXPECT_NEAR(std::tan(m.render_fov_y_deg * kPi / 360.0), 360.0 / k.fy, 1e-12);
}

TEST(DistortionMap, SamplesTheRenderWhereTheLensModelSays) {
    Intrinsics k = Intrinsics::from_hfov(320, 240, 85.0);
    k.d = {-0.25, 0.05, 0.002, -0.001, 0.0};
    k.cx += 3.0;  // an off-centre principal point
    const DistortionMap m = make_distortion_map(k);
    ASSERT_FALSE(m.identity);
    EXPECT_EQ(m.outside, 0u);
    EXPECT_GT(m.render_width, k.width);  // barrel distortion sees more than the pinhole of the same focal length
    EXPECT_GT(m.render_height, k.height);
    EXPECT_NEAR(std::tan(m.render_fov_y_deg * kPi / 360.0), (m.render_height / 2.0) / m.render_focal, 1e-12);
    const double rcx = (m.render_width - 1) / 2.0, rcy = (m.render_height - 1) / 2.0;
    for (const auto& [u, v] : {std::pair{0, 0}, {319, 239}, {160, 120}, {10, 200}, {319, 0}}) {
        const std::size_t i = static_cast<std::size_t>(v) * k.width + u;
        const double sx = m.src_x[i], sy = m.src_y[i];
        // Inside the render, with room for the bilinear neighbours.
        EXPECT_GE(sx, 0.0);
        EXPECT_GE(sy, 0.0);
        EXPECT_LE(sx, m.render_width - 1.0);
        EXPECT_LE(sy, m.render_height - 1.0);
        // The ray of that render position projects to this pixel.
        const auto [pu, pv] = k.project((sx - rcx) / m.render_focal, (sy - rcy) / m.render_focal);
        EXPECT_NEAR(pu, u, 2e-3);
        EXPECT_NEAR(pv, v, 2e-3);
    }
}

TEST(DistortionMap, CapsTheRenderAndCountsWhatIsLeftOut) {
    Intrinsics k = Intrinsics::from_hfov(400, 300, 100.0);
    k.d = {-0.2, 0.0, 0.0, 0.0, 0.0};
    const DistortionMap m = make_distortion_map(k, 512);
    EXPECT_LE(m.render_width, 512);
    EXPECT_LE(m.render_height, 512);
    EXPECT_GT(m.outside, 0u);
    EXPECT_TRUE(std::isnan(m.src_x[0]));  // a corner
}

TEST(Remap, ShiftsAnImageByAnOffCentrePrincipalPoint) {
    // No distortion, principal point 2 pixels right of the centre: the image
    // is the render moved 2 pixels to the right.
    Intrinsics k = Intrinsics::from_hfov(16, 8, 60.0);
    k.cx += 2.0;
    const DistortionMap m = make_distortion_map(k);
    ASSERT_FALSE(m.identity);
    RgbImage src;
    src.width = m.render_width;
    src.height = m.render_height;
    src.rgb.resize(static_cast<std::size_t>(src.width) * src.height * 3);
    for (int y = 0; y < src.height; ++y) {
        for (int x = 0; x < src.width; ++x) {
            std::uint8_t* p = &src.rgb[(static_cast<std::size_t>(y) * src.width + x) * 3];
            p[0] = static_cast<std::uint8_t>(10 * x);
            p[1] = static_cast<std::uint8_t>(20 * y);
            p[2] = 7;
        }
    }
    const RgbImage out = remap_rgb(src, m, k.width, k.height);
    ASSERT_EQ(out.rgb.size(), 16u * 8u * 3u);
    const double rcx = (m.render_width - 1) / 2.0, rcy = (m.render_height - 1) / 2.0;
    for (const auto& [u, v] : {std::pair{0, 0}, {15, 7}, {8, 4}}) {
        const std::uint8_t* p = &out.rgb[(static_cast<std::size_t>(v) * 16 + u) * 3];
        EXPECT_NEAR(p[0], 10.0 * (rcx + (u - k.cx)), 1.0);
        EXPECT_NEAR(p[1], 20.0 * (rcy + (v - k.cy)), 1.0);
        EXPECT_EQ(p[2], 7);
    }
}

TEST(Remap, PositionsAreInterpolatedOnASurfaceButNotAcrossAnEdge) {
    DistortionMap m;
    m.identity = false;
    m.render_width = 3;
    m.render_height = 2;
    // Render: a plane at range 100 in the left two columns (x grows by 1 a
    // pixel), something much nearer in the right column, nothing bottom right.
    const std::vector<float> src = {0, 0, 5, 100, 1, 0, 5, 100, 9, 9, 9, 40,
                                    0, 1, 5, 100, 1, 1, 5, 100, 0, 0, 0, 0};
    m.src_x = {0.25f, 1.3f, 1.8f, 1.6f, 7.0f};
    m.src_y = {0.5f, 0.0f, 0.2f, 0.9f, 0.0f};
    const std::vector<float> out = remap_positions(src, m, 5, 1);
    ASSERT_EQ(out.size(), 20u);
    // On the plane: the point of the pixel's own ray.
    EXPECT_FLOAT_EQ(out[0], 0.25f);
    EXPECT_FLOAT_EQ(out[1], 0.5f);
    EXPECT_FLOAT_EQ(out[2], 5.0f);
    EXPECT_FLOAT_EQ(out[3], 100.0f);
    // Across the edge: the nearest pixel as it is, never a mix.
    EXPECT_FLOAT_EQ(out[4], 1.0f);
    EXPECT_FLOAT_EQ(out[7], 100.0f);
    EXPECT_FLOAT_EQ(out[8], 9.0f);
    EXPECT_FLOAT_EQ(out[11], 40.0f);
    // Nearest is the empty pixel: empty.
    EXPECT_FLOAT_EQ(out[15], 0.0f);
    // Outside the render: empty.
    EXPECT_FLOAT_EQ(out[19], 0.0f);
}

TEST(Remap, LeavesPixelsOutsideTheRenderEmpty) {
    DistortionMap m;
    m.identity = false;
    m.render_width = 2;
    m.render_height = 2;
    m.src_x = {0.5f, 5.0f, std::nanf("")};
    m.src_y = {0.5f, 0.0f, 0.0f};
    RgbImage src;
    src.width = src.height = 2;
    src.rgb = {0, 0, 0, 100, 100, 100, 100, 100, 100, 200, 200, 200};
    const RgbImage out = remap_rgb(src, m, 3, 1);
    EXPECT_EQ(out.rgb[0], 100);  // the mean of the four
    EXPECT_EQ(out.rgb[3], 0);
    EXPECT_EQ(out.rgb[6], 0);
    const std::vector<float> f = remap_positions(std::vector<float>(16, 1.0f), m, 3, 1);
    EXPECT_FLOAT_EQ(f[3], 1.0f);
    EXPECT_FLOAT_EQ(f[7], 0.0f);
    EXPECT_FLOAT_EQ(f[11], 0.0f);
}

TEST(Mount, StraightDownHasTheImageTopForward) {
    const CameraAxes a = mount_axes(Mount{});
    expect_vec(a.forward, {0, 0, -1});
    expect_vec(a.up, {1, 0, 0});
    expect_vec(a.right, {0, -1, 0});  // the body's right
}

TEST(Mount, ForwardTiltPanAndRoll) {
    Mount m;
    m.tilt_deg = 0.0;
    CameraAxes a = mount_axes(m);
    expect_vec(a.forward, {1, 0, 0});
    expect_vec(a.up, {0, 0, 1});
    expect_vec(a.right, {0, -1, 0});
    m.pan_deg = 90.0;  // looking left
    a = mount_axes(m);
    expect_vec(a.forward, {0, 1, 0});
    expect_vec(a.right, {1, 0, 0});
    m.pan_deg = 0.0;
    m.roll_deg = 90.0;  // the image top turns to the right
    a = mount_axes(m);
    expect_vec(a.forward, {1, 0, 0});
    expect_vec(a.up, {0, -1, 0});
    m.roll_deg = 0.0;
    m.tilt_deg = -45.0;
    a = mount_axes(m);
    expect_vec(a.forward, {std::sqrt(0.5), 0, -std::sqrt(0.5)});
    expect_vec(a.up, {std::sqrt(0.5), 0, std::sqrt(0.5)});
    // Always orthonormal and right-handed (right = forward x up).
    m = {33.0, -57.0, 21.0, {}};
    a = mount_axes(m);
    EXPECT_NEAR(dot(a.forward, a.up), 0.0, 1e-12);
    EXPECT_NEAR(dot(a.forward, a.forward), 1.0, 1e-12);
    expect_vec(cross(a.forward, a.up), a.right);
}

TEST(Mount, OpticalRotationMapsTheOpticalAxes) {
    for (const Mount& m : {Mount{}, Mount{0.0, 0.0, 0.0, {}}, Mount{40.0, -30.0, 10.0, {}}, Mount{180.0, -90.0, 0.0, {}}}) {
        const CameraAxes a = mount_axes(m);
        const Quat q = optical_rotation(a);
        EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, 1e-12);
        expect_vec(rotate(q, {1, 0, 0}), a.right, 1e-9);        // x: image right
        expect_vec(rotate(q, {0, 1, 0}), -a.up, 1e-9);          // y: image down
        expect_vec(rotate(q, {0, 0, 1}), a.forward, 1e-9);      // z: into the scene
    }
}

TEST(Quaternions, ComposeAndInvert) {
    const Quat a = axis_angle({0, 0, 1}, kPi / 2), b = axis_angle({1, 0, 0}, kPi / 2);
    // a * b: b first, then a. x -> (b) x -> (a) y; y -> (b) z -> (a) z.
    expect_vec(rotate(multiply(a, b), {1, 0, 0}), {0, 1, 0});
    expect_vec(rotate(multiply(a, b), {0, 1, 0}), {0, 0, 1});
    expect_vec(rotate(multiply(a, conjugate(a)), {0.3, -0.2, 0.9}), {0.3, -0.2, 0.9});
    // from_columns for a rotation of 180 degrees (trace -1) and of 120 degrees.
    const Quat half = from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});
    expect_vec(rotate(half, {1, 2, 3}), {-1, -2, 3});
    const Quat third = from_columns({0, 1, 0}, {0, 0, 1}, {1, 0, 0});
    expect_vec(rotate(third, {1, 0, 0}), {0, 1, 0});
    expect_vec(rotate(third, {0, 0, 1}), {1, 0, 0});
}

RgbImage flat(int w, int h, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    RgbImage img;
    img.width = w;
    img.height = h;
    img.rgb.resize(static_cast<std::size_t>(w) * h * 3);
    for (std::size_t i = 0; i < img.rgb.size(); i += 3) {
        img.rgb[i] = r;
        img.rgb[i + 1] = g;
        img.rgb[i + 2] = b;
    }
    return img;
}

TEST(Realism, NeutralSettingsChangeNothing) {
    RgbImage img = flat(8, 8, 10, 128, 250);
    img.rgb[5] = 77;
    const RgbImage before = img;
    Realism r;
    r.jpeg_quality = 40;  // not a pixel operation here
    EXPECT_FALSE(r.changes_pixels());
    apply_realism(img, r, 1);
    EXPECT_EQ(img.rgb, before.rgb);
    EXPECT_TRUE(Realism::preset().changes_pixels());
}

TEST(Realism, BlurSpreadsAPointAndKeepsItsSum) {
    RgbImage img = flat(21, 21, 0, 0, 0);
    img.rgb[(10 * 21 + 10) * 3] = 250;
    Realism r;
    r.blur_sigma = 1.5;
    apply_realism(img, r, 1);
    double sum = 0.0;
    for (std::size_t i = 0; i < img.rgb.size(); i += 3) sum += img.rgb[i];
    EXPECT_NEAR(sum, 250.0, 12.0);  // rounding to 8 bits loses a little
    const auto at = [&](int x, int y) { return img.rgb[(static_cast<std::size_t>(y) * 21 + x) * 3]; };
    EXPECT_LT(at(10, 10), 30);
    EXPECT_GT(at(10, 10), at(11, 10));
    EXPECT_EQ(at(11, 10), at(10, 11));  // the same in both directions
    EXPECT_EQ(at(9, 10), at(11, 10));
    EXPECT_GT(at(12, 10), 0);
    // A flat image stays flat, also at the edges.
    RgbImage grey = flat(12, 9, 90, 90, 90);
    apply_realism(grey, r, 1);
    for (const std::uint8_t v : grey.rgb) EXPECT_EQ(v, 90);
}

TEST(Realism, VignetteDarkensTheCornersOnly) {
    RgbImage img = flat(101, 51, 200, 200, 200);
    Realism r;
    r.vignette = 0.5;
    apply_realism(img, r, 1);
    EXPECT_EQ(img.rgb[(25 * 101 + 50) * 3], 200);  // the centre
    EXPECT_EQ(img.rgb[0], 100);                    // a corner: half as bright
    EXPECT_EQ(img.rgb[(50 * 101 + 100) * 3], 100);
}

TEST(Realism, ColourControls) {
    Realism r;
    r.gains = {0.5, 1.0, 1.0};
    RgbImage img = flat(2, 2, 200, 100, 50);
    apply_realism(img, r, 1);
    EXPECT_EQ(img.rgb[0], 100);
    EXPECT_EQ(img.rgb[1], 100);
    EXPECT_EQ(img.rgb[2], 50);

    r = {};
    r.brightness = 2.0;
    img = flat(2, 2, 200, 100, 50);
    apply_realism(img, r, 1);
    EXPECT_EQ(img.rgb[0], 255);  // clipped
    EXPECT_EQ(img.rgb[1], 200);

    r = {};
    r.contrast = 0.5;  // toward mid grey (127.5)
    img = flat(2, 2, 0, 255, 128);
    apply_realism(img, r, 1);
    EXPECT_NEAR(img.rgb[0], 64, 1);
    EXPECT_NEAR(img.rgb[1], 191, 1);
    EXPECT_NEAR(img.rgb[2], 128, 1);

    r = {};
    r.saturation = 0.0;
    img = flat(2, 2, 255, 0, 0);
    apply_realism(img, r, 1);
    EXPECT_EQ(img.rgb[0], img.rgb[1]);
    EXPECT_EQ(img.rgb[1], img.rgb[2]);
    EXPECT_NEAR(img.rgb[0], 76, 1);  // 0.299 of red

    r = {};
    r.gamma = 2.0;  // brightens mid tones, keeps black and white
    img = flat(2, 2, 0, 64, 255);
    apply_realism(img, r, 1);
    EXPECT_EQ(img.rgb[0], 0);
    EXPECT_NEAR(img.rgb[1], 128, 1);
    EXPECT_EQ(img.rgb[2], 255);
}

TEST(Realism, NoiseHasTheAskedSpreadAndFollowsTheSeed) {
    Realism r;
    r.noise_sigma = 6.0;
    RgbImage a = flat(200, 200, 128, 128, 128), b = a, c = a;
    apply_realism(a, r, 42);
    apply_realism(b, r, 42);
    apply_realism(c, r, 43);
    EXPECT_EQ(a.rgb, b.rgb);
    EXPECT_NE(a.rgb, c.rgb);
    double sum = 0.0, sq = 0.0;
    for (const std::uint8_t v : a.rgb) {
        sum += v - 128.0;
        sq += (v - 128.0) * (v - 128.0);
    }
    const double n = static_cast<double>(a.rgb.size()), mean = sum / n;
    EXPECT_NEAR(mean, 0.0, 0.1);
    EXPECT_NEAR(std::sqrt(sq / n - mean * mean), 6.0, 0.15);
}

TEST(Drift, StartsAtTheBiasAndRepeatsForASeed) {
    DriftOptions o;
    o.seed = 7;
    o.start_error = 25.0;
    Drift a(o), b(o);
    const DriftSample s0 = a.at(100.0);  // any first time is the start
    EXPECT_NEAR(std::hypot(s0.east, s0.north), 25.0, 1e-9);
    EXPECT_DOUBLE_EQ(s0.up, 0.0);
    EXPECT_DOUBLE_EQ(s0.yaw_deg, o.yaw_bias);
    b.at(100.0);
    for (int i = 1; i <= 50; ++i) {
        const DriftSample x = a.at(100.0 + i * 0.1), y = b.at(100.0 + i * 0.1);
        EXPECT_DOUBLE_EQ(x.east, y.east);
        EXPECT_DOUBLE_EQ(x.yaw_deg, y.yaw_deg);
        EXPECT_DOUBLE_EQ(x.roll_deg, y.roll_deg);
    }
    o.seed = 8;
    Drift c(o);
    c.at(100.0);
    EXPECT_NE(c.at(105.0).east, a.at(105.0).east);
    // Time standing still adds no drift.
    const DriftSample before = a.at(105.0), after = a.at(105.0);
    EXPECT_DOUBLE_EQ(before.east, after.east);
    EXPECT_DOUBLE_EQ(before.yaw_deg, after.yaw_deg);
}

/// Spread of the errors at `t` over many seeds, sampled every `step` seconds.
struct Spread {
    double east = 0.0, up = 0.0, yaw = 0.0, roll = 0.0;
};
Spread spread_at(double t, double step, const DriftOptions& base) {
    const int runs = 4000;
    Spread s;
    for (int i = 0; i < runs; ++i) {
        DriftOptions o = base;
        o.seed = 1000 + static_cast<std::uint64_t>(i);
        Drift d(o);
        DriftSample x = d.at(0.0);
        const int steps = static_cast<int>(std::lround(t / step));
        for (int k = 1; k <= steps; ++k) x = d.at(k * step);
        s.east += x.east * x.east;
        s.up += x.up * x.up;
        s.yaw += (x.yaw_deg - o.yaw_bias) * (x.yaw_deg - o.yaw_bias);
        s.roll += x.roll_deg * x.roll_deg;
    }
    return {std::sqrt(s.east / runs), std::sqrt(s.up / runs), std::sqrt(s.yaw / runs), std::sqrt(s.roll / runs)};
}

TEST(Drift, GrowsAsDocumentedWhateverTheStep) {
    DriftOptions o;
    o.velocity_walk = 0.02;
    o.height_walk = 0.05;
    o.yaw_walk = 0.1;
    o.tilt_noise = 0.3;
    const double t = 100.0;
    const double expected_east = o.velocity_walk * std::pow(t, 1.5) / std::sqrt(3.0);  // 11.5 m
    for (const double step : {0.1, 1.0, 10.0}) {
        const Spread s = spread_at(t, step, o);
        // 4000 runs: the estimate of a standard deviation is good to about 1.1 %; allow 5 %.
        EXPECT_NEAR(s.east, expected_east, 0.05 * expected_east) << "step " << step;
        EXPECT_NEAR(s.up, o.height_walk * std::sqrt(t), 0.05 * o.height_walk * std::sqrt(t)) << "step " << step;
        EXPECT_NEAR(s.yaw, o.yaw_walk * std::sqrt(t), 0.05 * o.yaw_walk * std::sqrt(t)) << "step " << step;
        EXPECT_NEAR(s.roll, o.tilt_noise, 0.05 * o.tilt_noise) << "step " << step;
    }
}

TEST(Drift, AppliesHeadingAndTiltWithTheirSigns) {
    const Quat east_facing{};  // body forward = east, left = north
    DriftSample d;
    d.yaw_deg = 10.0;  // clockwise seen from above: east turns toward south
    Vec3d f = rotate(apply_drift(east_facing, d), {1, 0, 0});
    expect_vec(f, {std::cos(10.0 * kPi / 180), -std::sin(10.0 * kPi / 180), 0.0});
    d = {};
    d.pitch_deg = 10.0;  // nose up
    f = rotate(apply_drift(east_facing, d), {1, 0, 0});
    EXPECT_NEAR(f.z, std::sin(10.0 * kPi / 180), 1e-12);
    d = {};
    d.roll_deg = 10.0;  // right side down: the left side goes up
    const Vec3d left = rotate(apply_drift(east_facing, d), {0, 1, 0});
    EXPECT_NEAR(left.z, std::sin(10.0 * kPi / 180), 1e-12);
    // The heading error is about the world's up, whatever the attitude.
    const Quat banked = axis_angle({1, 0, 0}, 0.4);
    d = {};
    d.yaw_deg = 90.0;
    expect_vec(rotate(apply_drift(banked, d), {1, 0, 0}), {0, -1, 0});
}

TEST(Files, NpyHasANumpyHeaderAndTheData) {
    const std::filesystem::path file = std::filesystem::temp_directory_path() / "ogccdb_camera_sim_test.npy";
    const std::vector<float> data = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};
    ASSERT_TRUE(write_npy(file.string(), data.data(), {1, 2, 3}));
    std::ifstream in(file, std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    std::filesystem::remove(file);
    ASSERT_GT(bytes.size(), 10u);
    EXPECT_EQ(bytes.substr(0, 6), "\x93NUMPY");
    EXPECT_EQ(bytes[6], 1);
    EXPECT_EQ(bytes[7], 0);
    const std::size_t header = static_cast<unsigned char>(bytes[8]) + 256u * static_cast<unsigned char>(bytes[9]);
    EXPECT_EQ((10 + header) % 64, 0u);
    const std::string dict = bytes.substr(10, header);
    EXPECT_NE(dict.find("'descr': '<f4'"), std::string::npos);
    EXPECT_NE(dict.find("'fortran_order': False"), std::string::npos);
    EXPECT_NE(dict.find("'shape': (1, 2, 3), }"), std::string::npos);
    EXPECT_EQ(dict.back(), '\n');
    ASSERT_EQ(bytes.size(), 10 + header + data.size() * sizeof(float));
    EXPECT_EQ(std::memcmp(bytes.data() + 10 + header, data.data(), data.size() * sizeof(float)), 0);
}

TEST(Files, NpyShapeOfOneDimension) {
    const std::filesystem::path file = std::filesystem::temp_directory_path() / "ogccdb_camera_sim_test_1d.npy";
    const float v[2] = {1.0f, 2.0f};
    ASSERT_TRUE(write_npy(file.string(), v, {2}));
    std::ifstream in(file, std::ios::binary);
    const std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    std::filesystem::remove(file);
    EXPECT_NE(bytes.find("'shape': (2,), }"), std::string::npos);
    EXPECT_FALSE(write_npy((std::filesystem::temp_directory_path() / "no-such-folder" / "x.npy").string(), v, {2}));
}

TEST(BagText, QosProfileFollowsTheBagsFormat) {
    const std::string words = qos_profile(true, false, "");
    EXPECT_EQ(words.rfind("- history: unknown\n", 0), 0u);
    EXPECT_NE(words.find("\n  reliability: reliable\n"), std::string::npos);
    EXPECT_NE(words.find("\n  durability: volatile\n"), std::string::npos);
    EXPECT_NE(words.find("\n  liveliness: automatic\n"), std::string::npos);
    EXPECT_EQ(words.back(), 'e');  // no newline after the last line, as a recorder writes it
    const std::string latched = qos_profile(true, true, "- history: unknown\n  depth: 0\n  reliability: best_effort\n");
    EXPECT_NE(latched.find("durability: transient_local"), std::string::npos);
    EXPECT_NE(qos_profile(false, false, "").find("reliability: best_effort"), std::string::npos);
    // An older bag writes numbers.
    const std::string numbers = qos_profile(true, true, "- history: 3\n  depth: 0\n  reliability: 2\n  durability: 2\n");
    EXPECT_EQ(numbers.rfind("- history: 3\n", 0), 0u);
    EXPECT_NE(numbers.find("\n  reliability: 1\n"), std::string::npos);
    EXPECT_NE(numbers.find("\n  durability: 1\n"), std::string::npos);
    EXPECT_NE(numbers.find("\n  liveliness: 1\n"), std::string::npos);
}

}  // namespace
