#include "camera_sim.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>

#include "ogccdb/runtime/geodesy.hpp"

namespace ogccdb::bagrender {

using runtime::cross;
using runtime::kDegToRad;
using runtime::kPi;
using runtime::kRadToDeg;
using runtime::normalize;

// --- Camera model -----------------------------------------------------------

Intrinsics Intrinsics::from_hfov(int width, int height, double hfov_deg) {
    Intrinsics k;
    k.width = width;
    k.height = height;
    // The image is `width` pixels wide: half of them span half the field of view.
    k.fx = k.fy = (width / 2.0) / std::tan(hfov_deg * kDegToRad / 2.0);
    k.cx = (width - 1) / 2.0;
    k.cy = (height - 1) / 2.0;
    return k;
}

bool Intrinsics::distorted() const {
    return std::any_of(d.begin(), d.end(), [](double v) { return v != 0.0; });
}

namespace {

// The plumb_bob lens model on normalised coordinates, and its Jacobian.
struct Distorted {
    double x, y;
    double xx, xy, yx, yy;  // d(x, y) / d(input x, y)
};
Distorted distort(const std::array<double, 5>& d, double x, double y) {
    const double k1 = d[0], k2 = d[1], p1 = d[2], p2 = d[3], k3 = d[4];
    const double r2 = x * x + y * y;
    const double radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
    const double slope = k1 + r2 * (2.0 * k2 + r2 * 3.0 * k3);  // d radial / d r2
    Distorted o;
    o.x = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    o.y = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    o.xx = radial + 2.0 * x * x * slope + 2.0 * p1 * y + 6.0 * p2 * x;
    o.xy = 2.0 * x * y * slope + 2.0 * p1 * x + 2.0 * p2 * y;
    o.yx = o.xy;
    o.yy = radial + 2.0 * y * y * slope + 6.0 * p1 * y + 2.0 * p2 * x;
    return o;
}

}  // namespace

std::pair<double, double> Intrinsics::project(double x, double y) const {
    const Distorted o = distort(d, x, y);
    return {fx * o.x + cx, fy * o.y + cy};
}

bool Intrinsics::unproject(double u, double v, double& x, double& y) const {
    const double xd = (u - cx) / fx, yd = (v - cy) / fy;
    x = xd;
    y = yd;
    if (!distorted()) return true;
    // Newton's method from the distorted point. Inside the image the model is
    // one-to-one (its Jacobian keeps a positive determinant); a pixel beyond
    // the point where the model folds back has no answer.
    for (int i = 0; i < 60; ++i) {
        const Distorted o = distort(d, x, y);
        const double det = o.xx * o.yy - o.xy * o.yx;
        if (!(det > 1e-6)) return false;
        const double ex = o.x - xd, ey = o.y - yd;
        const double dx = (o.yy * ex - o.xy * ey) / det, dy = (o.xx * ey - o.yx * ex) / det;
        x -= dx;
        y -= dy;
        if (!std::isfinite(x) || !std::isfinite(y)) return false;
        if (std::fabs(dx) < 1e-14 && std::fabs(dy) < 1e-14) break;
    }
    const Distorted o = distort(d, x, y);
    return o.xx * o.yy - o.xy * o.yx > 1e-6 && std::fabs(o.x - xd) < 1e-9 && std::fabs(o.y - yd) < 1e-9;
}

flight::CameraInfo Intrinsics::camera_info() const {
    flight::CameraInfo c;
    c.width = static_cast<std::uint32_t>(width);
    c.height = static_cast<std::uint32_t>(height);
    c.distortion_model = "plumb_bob";
    c.d.assign(d.begin(), d.end());
    c.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    c.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return c;
}

DistortionMap make_distortion_map(const Intrinsics& k, int max_render_side) {
    DistortionMap m;
    const int w = k.width, h = k.height;
    const bool centred = std::fabs(k.cx - (w - 1) / 2.0) < 1e-6 && std::fabs(k.cy - (h - 1) / 2.0) < 1e-6 &&
                         std::fabs(k.fx - k.fy) < 1e-9 * k.fx;
    if (!k.distorted() && centred) {
        m.render_width = w;
        m.render_height = h;
        m.render_focal = k.fy;
        m.render_fov_y_deg = 2.0 * std::atan((h / 2.0) / k.fy) * kRadToDeg;
        return m;
    }
    m.identity = false;
    // The render keeps the image's finer focal length, so no detail is lost
    // at the image centre.
    const double f = std::max(k.fx, k.fy);
    m.render_focal = f;
    const std::size_t n = static_cast<std::size_t>(w) * h;
    m.src_x.assign(n, std::numeric_limits<float>::quiet_NaN());
    m.src_y.assign(n, std::numeric_limits<float>::quiet_NaN());
    std::vector<double> nx(n), ny(n);
    const double limit = (max_render_side / 2.0 - 2.0) / f;  // normalised half-extent the render may reach
    double max_x = 0.0, max_y = 0.0;
    for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            const std::size_t i = static_cast<std::size_t>(v) * w + u;
            double x = 0.0, y = 0.0;
            if (k.unproject(u, v, x, y) && std::fabs(x) <= limit && std::fabs(y) <= limit) {
                nx[i] = x;
                ny[i] = y;
                max_x = std::max(max_x, std::fabs(x));
                max_y = std::max(max_y, std::fabs(y));
            } else {
                nx[i] = ny[i] = std::numeric_limits<double>::quiet_NaN();
                ++m.outside;
            }
        }
    }
    // Symmetric about the optical axis, with a pixel to spare on each side
    // for the bilinear samples.
    m.render_width = 2 * static_cast<int>(std::ceil(f * max_x + 1.0));
    m.render_height = 2 * static_cast<int>(std::ceil(f * max_y + 1.0));
    m.render_fov_y_deg = 2.0 * std::atan((m.render_height / 2.0) / f) * kRadToDeg;
    const double rcx = (m.render_width - 1) / 2.0, rcy = (m.render_height - 1) / 2.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (std::isnan(nx[i])) continue;
        m.src_x[i] = static_cast<float>(rcx + f * nx[i]);
        m.src_y[i] = static_cast<float>(rcy + f * ny[i]);
    }
    return m;
}

render::RgbImage remap_rgb(const render::RgbImage& src, const DistortionMap& m, int width, int height) {
    render::RgbImage out;
    out.width = width;
    out.height = height;
    const std::size_t n = static_cast<std::size_t>(width) * height;
    out.rgb.assign(n * 3, 0);
    const int sw = src.width, sh = src.height;
    if (sw < 1 || sh < 1 || m.src_x.size() != n || m.src_y.size() != n) return out;
    const float max_x = static_cast<float>(sw - 1), max_y = static_cast<float>(sh - 1);
    for (std::size_t i = 0; i < n; ++i) {
        const float sx = m.src_x[i], sy = m.src_y[i];
        if (!(sx >= 0.0f && sy >= 0.0f && sx <= max_x && sy <= max_y)) continue;  // outside, or NaN
        const int x0 = std::min(static_cast<int>(sx), std::max(sw - 2, 0));
        const int y0 = std::min(static_cast<int>(sy), std::max(sh - 2, 0));
        const int x1 = std::min(x0 + 1, sw - 1), y1 = std::min(y0 + 1, sh - 1);
        const float ax = sx - static_cast<float>(x0), ay = sy - static_cast<float>(y0);
        const std::uint8_t* p00 = &src.rgb[(static_cast<std::size_t>(y0) * sw + x0) * 3];
        const std::uint8_t* p10 = &src.rgb[(static_cast<std::size_t>(y0) * sw + x1) * 3];
        const std::uint8_t* p01 = &src.rgb[(static_cast<std::size_t>(y1) * sw + x0) * 3];
        const std::uint8_t* p11 = &src.rgb[(static_cast<std::size_t>(y1) * sw + x1) * 3];
        for (int c = 0; c < 3; ++c) {
            const float top = p00[c] * (1.0f - ax) + p10[c] * ax, bottom = p01[c] * (1.0f - ax) + p11[c] * ax;
            out.rgb[i * 3 + c] = static_cast<std::uint8_t>(top * (1.0f - ay) + bottom * ay + 0.5f);
        }
    }
    return out;
}

std::vector<float> remap_positions(const std::vector<float>& src, const DistortionMap& m, int width, int height) {
    const std::size_t n = static_cast<std::size_t>(width) * height;
    std::vector<float> out(n * 4, 0.0f);
    const int sw = m.render_width, sh = m.render_height;
    if (sw < 1 || sh < 1 || src.size() != static_cast<std::size_t>(sw) * sh * 4 || m.src_x.size() != n || m.src_y.size() != n) {
        return out;
    }
    const float max_x = static_cast<float>(sw - 1), max_y = static_cast<float>(sh - 1);
    for (std::size_t i = 0; i < n; ++i) {
        const float sx = m.src_x[i], sy = m.src_y[i];
        if (!(sx >= 0.0f && sy >= 0.0f && sx <= max_x && sy <= max_y)) continue;  // outside, or NaN
        const int x0 = std::min(static_cast<int>(sx), std::max(sw - 2, 0));
        const int y0 = std::min(static_cast<int>(sy), std::max(sh - 2, 0));
        const int x1 = std::min(x0 + 1, sw - 1), y1 = std::min(y0 + 1, sh - 1);
        const float ax = sx - static_cast<float>(x0), ay = sy - static_cast<float>(y0);
        const float* p00 = &src[(static_cast<std::size_t>(y0) * sw + x0) * 4];
        const float* p10 = &src[(static_cast<std::size_t>(y0) * sw + x1) * 4];
        const float* p01 = &src[(static_cast<std::size_t>(y1) * sw + x0) * 4];
        const float* p11 = &src[(static_cast<std::size_t>(y1) * sw + x1) * 4];
        const float near = std::min(std::min(p00[3], p10[3]), std::min(p01[3], p11[3]));
        const float far = std::max(std::max(p00[3], p10[3]), std::max(p01[3], p11[3]));
        if (near > 0.0f && far - near <= 0.02f * near) {
            for (int c = 0; c < 4; ++c) {
                const float top = p00[c] * (1.0f - ax) + p10[c] * ax, bottom = p01[c] * (1.0f - ax) + p11[c] * ax;
                out[i * 4 + c] = top * (1.0f - ay) + bottom * ay;
            }
        } else {
            const float* nearest = ay < 0.5f ? (ax < 0.5f ? p00 : p10) : (ax < 0.5f ? p01 : p11);
            std::copy_n(nearest, 4, &out[i * 4]);
        }
    }
    return out;
}

CameraAxes mount_axes(const Mount& m) {
    const double pan = m.pan_deg * kDegToRad, tilt = m.tilt_deg * kDegToRad, roll = m.roll_deg * kDegToRad;
    const Vec3d heading{std::cos(pan), std::sin(pan), 0.0}, z{0.0, 0.0, 1.0};
    // Tilt turns the viewing direction from the heading toward the body's up
    // (down, for a negative tilt); the image's top stays in that vertical plane.
    const Vec3d forward = heading * std::cos(tilt) + z * std::sin(tilt);
    const Vec3d up = z * std::cos(tilt) - heading * std::sin(tilt);
    const Vec3d right = cross(forward, up);
    CameraAxes a;
    a.forward = normalize(forward);
    a.up = normalize(up * std::cos(roll) + right * std::sin(roll));
    a.right = normalize(cross(a.forward, a.up));
    return a;
}

Quat optical_rotation(const CameraAxes& a) { return from_columns(a.right, -a.up, a.forward); }

// --- Quaternions ------------------------------------------------------------

namespace {

Quat unit(Quat q) {
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return n > 0.0 ? Quat{q.x / n, q.y / n, q.z / n, q.w / n} : Quat{};
}

}  // namespace

Quat multiply(const Quat& a, const Quat& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

Quat conjugate(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }

Quat axis_angle(Vec3d axis, double angle_rad) {
    const Vec3d a = normalize(axis);
    const double s = std::sin(angle_rad / 2.0);
    return {a.x * s, a.y * s, a.z * s, std::cos(angle_rad / 2.0)};
}

Quat from_columns(Vec3d c0, Vec3d c1, Vec3d c2) {
    // m[row][column]; the largest of w, x, y, z is found first, for accuracy.
    const double m00 = c0.x, m10 = c0.y, m20 = c0.z, m01 = c1.x, m11 = c1.y, m21 = c1.z, m02 = c2.x, m12 = c2.y, m22 = c2.z;
    const double trace = m00 + m11 + m22;
    Quat q;
    if (trace > 0.0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q = {(m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4.0};
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q = {s / 4.0, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s};
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q = {(m01 + m10) / s, s / 4.0, (m12 + m21) / s, (m02 - m20) / s};
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q = {(m02 + m20) / s, (m12 + m21) / s, s / 4.0, (m10 - m01) / s};
    }
    return unit(q);
}

// --- Image realism ----------------------------------------------------------

bool Realism::changes_pixels() const {
    return blur_sigma > 0.0 || vignette != 0.0 || gains[0] != 1.0 || gains[1] != 1.0 || gains[2] != 1.0 ||
           brightness != 1.0 || contrast != 1.0 || saturation != 1.0 || gamma != 1.0 || noise_sigma > 0.0;
}

Realism Realism::preset() {
    Realism r;
    r.blur_sigma = 0.8;
    r.vignette = 0.25;
    r.gains = {1.05, 1.0, 0.93};
    r.brightness = 0.95;
    r.contrast = 0.92;
    r.saturation = 0.85;
    r.gamma = 1.1;
    r.noise_sigma = 3.0;
    r.jpeg_quality = 75;
    return r;
}

namespace {

// A small fast generator (xorshift64*), seeded through splitmix64 so that
// neighbouring seeds give unrelated sequences.
struct FastRng {
    std::uint64_t s;
    explicit FastRng(std::uint64_t seed) {
        seed += 0x9E3779B97F4A7C15ull;
        seed = (seed ^ (seed >> 30)) * 0xBF58476D1CE4E5B9ull;
        seed = (seed ^ (seed >> 27)) * 0x94D049BB133111EBull;
        s = seed ^ (seed >> 31);
        if (s == 0) s = 0x9E3779B97F4A7C15ull;
    }
    std::uint64_t next() {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return s * 0x2545F4914F6CDD1Dull;
    }
    /// Close to a standard normal: the sum of eight 16-bit uniforms (within
    /// 4.9 sigma), which is plenty for sensor noise and much faster than an
    /// exact one.
    float normal() {
        std::uint64_t a = next(), b = next();
        std::uint32_t sum = 0;
        for (int i = 0; i < 4; ++i) {
            sum += static_cast<std::uint32_t>(a & 0xFFFF) + static_cast<std::uint32_t>(b & 0xFFFF);
            a >>= 16;
            b >>= 16;
        }
        // Mean 8 * 32767.5; variance 8 * (65536^2 - 1) / 12.
        return (static_cast<float>(sum) - 262140.0f) * (1.0f / 53509.9f);
    }
};

void blur(std::vector<float>& img, int w, int h, double sigma) {
    const int radius = std::clamp(static_cast<int>(std::ceil(3.0 * sigma)), 1, 64);
    std::vector<float> kernel(static_cast<std::size_t>(2 * radius + 1));
    float sum = 0.0f;
    for (int i = -radius; i <= radius; ++i) {
        kernel[static_cast<std::size_t>(i + radius)] = static_cast<float>(std::exp(-0.5 * i * i / (sigma * sigma)));
        sum += kernel[static_cast<std::size_t>(i + radius)];
    }
    for (float& k : kernel) k /= sum;
    std::vector<float> tmp(img.size());
    // Rows, then columns; the edge pixel repeats beyond the image.
    for (int y = 0; y < h; ++y) {
        const float* row = &img[static_cast<std::size_t>(y) * w * 3];
        float* out = &tmp[static_cast<std::size_t>(y) * w * 3];
        for (int x = 0; x < w; ++x) {
            float acc[3] = {0.0f, 0.0f, 0.0f};
            for (int i = -radius; i <= radius; ++i) {
                const float* p = row + static_cast<std::size_t>(std::clamp(x + i, 0, w - 1)) * 3;
                const float k = kernel[static_cast<std::size_t>(i + radius)];
                acc[0] += p[0] * k;
                acc[1] += p[1] * k;
                acc[2] += p[2] * k;
            }
            std::copy_n(acc, 3, out + static_cast<std::size_t>(x) * 3);
        }
    }
    const std::size_t stride = static_cast<std::size_t>(w) * 3;
    for (int y = 0; y < h; ++y) {
        float* out = &img[static_cast<std::size_t>(y) * stride];
        std::fill_n(out, stride, 0.0f);
        for (int i = -radius; i <= radius; ++i) {
            const float* row = &tmp[static_cast<std::size_t>(std::clamp(y + i, 0, h - 1)) * stride];
            const float k = kernel[static_cast<std::size_t>(i + radius)];
            for (std::size_t j = 0; j < stride; ++j) out[j] += row[j] * k;
        }
    }
}

}  // namespace

void apply_realism(render::RgbImage& image, const Realism& r, std::uint64_t seed) {
    const int w = image.width, h = image.height;
    if (w < 1 || h < 1 || !r.changes_pixels()) return;
    const std::size_t n = static_cast<std::size_t>(w) * h;
    std::vector<float> img(n * 3);
    for (std::size_t i = 0; i < n * 3; ++i) img[i] = image.rgb[i] * (1.0f / 255.0f);

    if (r.blur_sigma > 0.0) blur(img, w, h, r.blur_sigma);

    const bool colour = r.gains[0] != 1.0 || r.gains[1] != 1.0 || r.gains[2] != 1.0 || r.brightness != 1.0 ||
                        r.contrast != 1.0 || r.saturation != 1.0 || r.gamma != 1.0;
    if (r.vignette != 0.0 || colour) {
        const float gain[3] = {static_cast<float>(r.gains[0] * r.brightness), static_cast<float>(r.gains[1] * r.brightness),
                               static_cast<float>(r.gains[2] * r.brightness)};
        const float contrast = static_cast<float>(r.contrast), saturation = static_cast<float>(r.saturation);
        const float inv_gamma = static_cast<float>(1.0 / std::max(r.gamma, 1e-3));
        const double cx = (w - 1) / 2.0, cy = (h - 1) / 2.0;
        const double corner2 = std::max(cx * cx + cy * cy, 1.0);
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                float* p = &img[(static_cast<std::size_t>(y) * w + x) * 3];
                // Vignetting: darker with the squared distance from the centre.
                const double r2 = ((x - cx) * (x - cx) + (y - cy) * (y - cy)) / corner2;
                const float shade = static_cast<float>(1.0 - r.vignette * r2);
                float c[3];
                for (int k = 0; k < 3; ++k) c[k] = (p[k] * shade * gain[k] - 0.5f) * contrast + 0.5f;
                const float grey = 0.299f * c[0] + 0.587f * c[1] + 0.114f * c[2];
                for (int k = 0; k < 3; ++k) {
                    const float v = std::clamp(grey + (c[k] - grey) * saturation, 0.0f, 1.0f);
                    p[k] = r.gamma != 1.0 ? std::pow(v, inv_gamma) : v;
                }
            }
        }
    }

    const float sigma = static_cast<float>(std::max(r.noise_sigma, 0.0));
    FastRng rng(seed);
    for (std::size_t i = 0; i < n * 3; ++i) {
        float v = img[i] * 255.0f;
        if (sigma > 0.0f) v += sigma * rng.normal();
        image.rgb[i] = static_cast<std::uint8_t>(std::clamp(v + 0.5f, 0.0f, 255.0f));
    }
}

// --- Prior pose -------------------------------------------------------------

Drift::Drift(const DriftOptions& o) : o_(o), rng_(o.seed) {}

double Drift::gauss() {
    // Box-Muller on the generator's 53-bit uniforms: std::mt19937_64 gives the
    // same numbers on every platform, the standard's distributions do not.
    const double u1 = (static_cast<double>(rng_() >> 11) + 1.0) * (1.0 / 9007199254740992.0);  // (0, 1]
    const double u2 = static_cast<double>(rng_() >> 11) * (1.0 / 9007199254740992.0);          // [0, 1)
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * kPi * u2);
}

DriftSample Drift::at(double t) {
    if (!started_) {
        started_ = true;
        t_ = t;
        const double direction = 2.0 * kPi * static_cast<double>(rng_() >> 11) * (1.0 / 9007199254740992.0);
        s_.east = o_.start_error * std::cos(direction);
        s_.north = o_.start_error * std::sin(direction);
        s_.yaw_deg = o_.yaw_bias;
    }
    const double dt = t - t_;
    if (dt > 0.0) {
        t_ = t;
        const double root = std::sqrt(dt);
        // An exact step of a random-walk velocity and the position it adds up
        // to: over the step the position gets v * dt and its own share of the
        // step's noise (variance q dt^3 / 3, covariance q dt^2 / 2 with the
        // velocity's), so the result does not depend on the step length.
        auto step = [&](double& position, double& velocity) {
            const double n1 = gauss(), n2 = gauss();
            position += velocity * dt + o_.velocity_walk * dt * root * (n1 / 2.0 + n2 / (2.0 * std::sqrt(3.0)));
            velocity += o_.velocity_walk * root * n1;
        };
        step(s_.east, ve_);
        step(s_.north, vn_);
        s_.up += o_.height_walk * root * gauss();
        s_.yaw_deg += o_.yaw_walk * root * gauss();
    }
    s_.roll_deg = o_.tilt_noise * gauss();
    s_.pitch_deg = o_.tilt_noise * gauss();
    return s_;
}

Quat apply_drift(const Quat& attitude, const DriftSample& d) {
    // Heading clockwise seen from above: a turn about ENU up by minus the angle.
    const Quat yaw = axis_angle({0.0, 0.0, 1.0}, -d.yaw_deg * kDegToRad);
    // In the body (FLU): right side down is a turn about x, nose up a turn about -y.
    const Quat roll = axis_angle({1.0, 0.0, 0.0}, d.roll_deg * kDegToRad);
    const Quat pitch = axis_angle({0.0, 1.0, 0.0}, -d.pitch_deg * kDegToRad);
    return unit(multiply(yaw, multiply(attitude, multiply(pitch, roll))));
}

// --- Files and bag text -------------------------------------------------------

bool write_npy(const std::string& path, const float* data, const std::vector<std::size_t>& shape) {
    // NumPy format 1.0: magic, version, the header's length (uint16, little
    // endian), then a Python dict as text, padded with spaces and ended by a
    // newline so that the data starts at a multiple of 64 bytes.
    std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (";
    std::size_t count = 1;
    for (std::size_t i = 0; i < shape.size(); ++i) {
        header += std::to_string(shape[i]);
        if (shape.size() == 1 || i + 1 < shape.size()) header += ",";
        if (i + 1 < shape.size()) header += " ";
        count *= shape[i];
    }
    header += "), }";
    const std::size_t unpadded = 10 + header.size() + 1;
    header.append((64 - unpadded % 64) % 64, ' ');
    header += '\n';
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    const char magic[8] = {'\x93', 'N', 'U', 'M', 'P', 'Y', 1, 0};
    const char length[2] = {static_cast<char>(header.size() & 0xFF), static_cast<char>((header.size() >> 8) & 0xFF)};
    out.write(magic, 8);
    out.write(length, 2);
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    // float32 as the machine holds it: little endian on the platforms we build for.
    out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(count * sizeof(float)));
    out.close();
    return static_cast<bool>(out);
}

std::string qos_profile(bool reliable, bool transient_local, const std::string& like) {
    // Older bags write the policies as numbers (history unknown = 3,
    // reliable = 1, best effort = 2, transient local = 1, volatile = 2,
    // liveliness automatic = 1), newer ones as words.
    bool numbers = false;
    if (const std::size_t at = like.find("reliability: "); at != std::string::npos && at + 13 < like.size()) {
        numbers = std::isdigit(static_cast<unsigned char>(like[at + 13])) != 0;
    }
    const char* forever = "    sec: 9223372036\n    nsec: 854775807\n";
    std::string y;
    y += std::string("- history: ") + (numbers ? "3" : "unknown") + "\n";
    y += "  depth: 0\n";
    y += std::string("  reliability: ") + (numbers ? (reliable ? "1" : "2") : (reliable ? "reliable" : "best_effort")) + "\n";
    y += std::string("  durability: ") +
         (numbers ? (transient_local ? "1" : "2") : (transient_local ? "transient_local" : "volatile")) + "\n";
    y += std::string("  deadline:\n") + forever;
    y += std::string("  lifespan:\n") + forever;
    y += std::string("  liveliness: ") + (numbers ? "1" : "automatic") + "\n";
    y += std::string("  liveliness_lease_duration:\n") + forever;
    y += "  avoid_ros_namespace_conventions: false";
    return y;
}

}  // namespace ogccdb::bagrender
