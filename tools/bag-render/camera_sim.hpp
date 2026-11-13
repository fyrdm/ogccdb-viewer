#pragma once
// Camera simulation for ogccdb-bag-render, without OpenGL: the camera model
// (intrinsics, lens distortion, mount), the image "realism" applied to a
// rendered frame, and the drifting prior pose given to a localizer.
#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "ogccdb/flight/messages.hpp"
#include "ogccdb/flight/track.hpp"
#include "ogccdb/render/image.hpp"
#include "ogccdb/runtime/math.hpp"

namespace ogccdb::bagrender {

using flight::Quat;
using flight::rotate;  // Vec3d rotate(const Quat&, Vec3d)
using runtime::Vec3d;

// --- Camera model ---------------------------------------------------------

/// A pinhole camera with optional lens distortion, in the convention of
/// sensor_msgs/CameraInfo: pixel centres at whole numbers, (0, 0) the centre of
/// the top-left pixel; distortion "plumb_bob" (k1, k2, p1, p2, k3).
struct Intrinsics {
    int width = 1280, height = 720;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    std::array<double, 5> d{};

    /// Square pixels, principal point at the image centre.
    static Intrinsics from_hfov(int width, int height, double hfov_deg);
    bool distorted() const;
    /// Normalised image coordinates (x/z, y/z) to a pixel of the distorted image.
    std::pair<double, double> project(double x, double y) const;
    /// A pixel of the distorted image to normalised coordinates (the inverse
    /// of project(), by fixed-point iteration). False where the lens model has
    /// no answer (far outside the calibrated image); x and y are then unset.
    bool unproject(double u, double v, double& x, double& y) const;
    /// The message for these intrinsics (K, D, R = identity, P = [K | 0]).
    flight::CameraInfo camera_info() const;
};

/// How to make the camera's image from an ideal pinhole render: the render's
/// size and vertical field of view (square pixels of focal length
/// `render_focal`, principal point at its centre), and for each pixel of the
/// image where to sample the render.
struct DistortionMap {
    int render_width = 0, render_height = 0;
    double render_fov_y_deg = 0.0;
    double render_focal = 0.0;        ///< pixels
    bool identity = true;             ///< the render is the image, pixel for pixel
    std::size_t outside = 0;          ///< image pixels the render cannot cover (left black)
    std::vector<float> src_x, src_y;  ///< per image pixel, render pixel coordinates (NaN: none)
};
/// `max_render_side` caps the render's width and height (a strong barrel
/// distortion needs a much wider render than the image).
DistortionMap make_distortion_map(const Intrinsics& k, int max_render_side = 8192);

/// Bilinear resampling of a render through the map (black outside it).
render::RgbImage remap_rgb(const render::RgbImage& src, const DistortionMap& m, int width, int height);
/// Resampling of a position render (x, y, z and the range from the eye, four
/// floats a pixel; range 0 where nothing was drawn). Bilinear where the four
/// neighbours lie on one surface (ranges within 2 %), so a pixel gets the point
/// its own ray meets; the nearest neighbour elsewhere, so that nothing is mixed
/// across a depth edge. Range 0 outside the render.
std::vector<float> remap_positions(const std::vector<float>& src, const DistortionMap& m, int width, int height);

/// Camera mount on the body (FLU: x forward, y left, z up). Applied in this
/// order: pan about the body's z (positive turns the camera left), tilt about
/// the camera's left axis (-90 looks straight down with the image top toward
/// the body's forward, 0 looks forward), roll about the viewing direction
/// (positive turns the image top to the right).
struct Mount {
    double pan_deg = 0.0, tilt_deg = -90.0, roll_deg = 0.0;
    Vec3d offset;  ///< the camera's position on the body (FLU, metres)
};
struct CameraAxes {
    Vec3d forward, up, right;  ///< viewing direction, image up, image right
};
/// The camera's axes in body coordinates.
CameraAxes mount_axes(const Mount& m);
/// Rotation from the camera's optical frame (x right, y down, z forward: the
/// ROS convention for frames named *_optical) to the frame the axes are in.
Quat optical_rotation(const CameraAxes& axes);

// --- Quaternions ------------------------------------------------------------

/// The rotation that applies `b` first, then `a` (the Hamilton product a * b).
Quat multiply(const Quat& a, const Quat& b);
Quat conjugate(const Quat& q);
Quat axis_angle(Vec3d axis, double angle_rad);
/// The rotation whose matrix has these columns (orthonormal, right-handed).
Quat from_columns(Vec3d c0, Vec3d c1, Vec3d c2);

// --- Image realism ----------------------------------------------------------

/// Cheap degradations of a rendered frame, applied in this order: lens blur,
/// vignetting, colour (gains, brightness, contrast, saturation, gamma), sensor
/// noise. JPEG compression comes last, when the frame is encoded.
struct Realism {
    double blur_sigma = 0.0;   ///< pixels, Gaussian
    double vignette = 0.0;     ///< 0..1: how much darker the corners are
    std::array<double, 3> gains{1.0, 1.0, 1.0};  ///< red, green, blue (white balance)
    double brightness = 1.0;   ///< multiplies
    double contrast = 1.0;     ///< around mid grey
    double saturation = 1.0;   ///< 0 = grey
    double gamma = 1.0;        ///< > 1 brightens mid tones
    double noise_sigma = 0.0;  ///< 8-bit levels, Gaussian, per channel
    int jpeg_quality = 90;     ///< 1..100

    bool changes_pixels() const;
    /// A moderate amount of each, for a quick start.
    static Realism preset();
};
/// Applies all but JPEG in place; the same `seed` gives the same noise.
void apply_realism(render::RgbImage& img, const Realism& r, std::uint64_t seed);

// --- Prior pose -------------------------------------------------------------

/// A dead-reckoning-like error. Horizontal: the velocity error of each axis is
/// a random walk, so the position error's standard deviation grows like
/// velocity_walk * t^1.5 / sqrt(3). Height and heading errors are random walks
/// (the heading from a constant bias); roll and pitch get white noise.
struct DriftOptions {
    double velocity_walk = 0.01;  ///< m/s per sqrt(s), each horizontal axis
    double height_walk = 0.05;    ///< m per sqrt(s)
    double yaw_walk = 0.05;       ///< degrees per sqrt(s)
    double yaw_bias = 2.0;        ///< degrees, heading error at the start
    double tilt_noise = 0.2;      ///< degrees, roll and pitch, white
    double start_error = 0.0;     ///< metres, horizontal position error at the start
    std::uint64_t seed = 1;
};
struct DriftSample {
    double east = 0.0, north = 0.0, up = 0.0;  ///< position error, metres
    double yaw_deg = 0.0;                      ///< heading error, clockwise seen from above
    double roll_deg = 0.0, pitch_deg = 0.0;    ///< right side down, nose up
};
class Drift {
public:
    explicit Drift(const DriftOptions& o);
    /// The error at time `t` seconds; call with times that do not decrease.
    /// The statistics do not depend on how often it is called.
    DriftSample at(double t);

private:
    double gauss();

    DriftOptions o_;
    std::mt19937_64 rng_;
    bool started_ = false;
    double t_ = 0.0;
    DriftSample s_;
    double ve_ = 0.0, vn_ = 0.0;
};
/// A body attitude (FLU to local ENU) with the drift's heading, roll and pitch errors.
Quat apply_drift(const Quat& attitude, const DriftSample& d);

// --- Files and bag text -------------------------------------------------------

/// Writes float32 data as a NumPy .npy file of the given shape (C order).
bool write_npy(const std::string& path, const float* data, const std::vector<std::size_t>& shape);

/// The offered_qos_profiles text of a bag topic: the YAML a recorder stores
/// for a publisher that is reliable or best effort, and transient local (as
/// /tf_static) or volatile. `like` is such a text from the bag being copied:
/// an older bag format writes numbers instead of words, and the result
/// follows it (words if `like` is empty).
std::string qos_profile(bool reliable, bool transient_local, const std::string& like);

}  // namespace ogccdb::bagrender
