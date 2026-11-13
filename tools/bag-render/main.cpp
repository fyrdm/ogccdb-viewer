// ogccdb-bag-render: adds a simulated camera to a recorded flight. It flies a
// ROS 2 bag's positions and attitudes over a CDB store, renders what a camera
// on the vehicle would see, and writes a copy of the bag with the frames,
// their camera_info, the camera mount as a static transform and a drifting
// prior pose. Side files hold the truth for each frame. No window is shown.
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera_sim.hpp"
#include "ogccdb/flight/bag.hpp"
#include "ogccdb/flight/bag_writer.hpp"
#include "ogccdb/flight/messages.hpp"
#include "ogccdb/flight/track.hpp"
#include "ogccdb/render/camera.hpp"
#include "ogccdb/render/framebuffer.hpp"
#include "ogccdb/render/gl.hpp"
#include "ogccdb/render/image.hpp"
#include "ogccdb/render/renderer.hpp"
#include "ogccdb/runtime/geodesy.hpp"
#include "ogccdb/runtime/terrain.hpp"

using namespace ogccdb;
using namespace ogccdb::runtime;
using namespace ogccdb::render;
using namespace ogccdb::flight;
using namespace ogccdb::bagrender;

namespace {

namespace fs = std::filesystem;

struct Options {
    fs::path cdb, bag, out;
    // Camera
    int width = 1280, height = 720;
    double hfov = 80.0;
    std::optional<std::array<double, 4>> intrinsics;  // fx, fy, cx, cy
    std::array<double, 5> distortion{};
    Mount mount;
    double rate = 10.0;
    // Flight
    std::optional<double> from, to;  // seconds from the start of the bag
    std::string altitude = "auto";
    double min_agl = 1.0;
    TrackOptions track;
    // Image and prior
    Realism realism;
    bool prior = true;
    DriftOptions drift;
    int truth_every = 10;
    // Bag
    std::string image_topic = "/camera/image_raw/compressed", info_topic = "/camera/camera_info";
    std::string prior_topic = "/prior/geo_pose";
    std::string camera_frame = "camera_optical_frame", body_frame = "base_link";
    std::vector<std::string> drop;
    std::string compression;
    bool overwrite = false;
    double screen_error = 1.5;
};

void usage() {
    std::puts(
        "usage: ogccdb-bag-render <cdb> <bag> <output folder> [options]\n"
        "\n"
        "Flies the bag's positions and attitudes over the CDB store and writes a copy of the\n"
        "bag with a camera added: compressed images, camera_info, the mount on /tf_static and\n"
        "a drifting prior pose. <output folder> becomes the new bag; its truth/ folder holds\n"
        "the true pose of every frame and the ground point seen by each pixel.\n"
        "\n"
        "Camera\n"
        "  --size <w>x<h>              image size (default 1280x720)\n"
        "  --hfov <deg>                horizontal field of view (default 80)\n"
        "  --intrinsics <fx,fy,cx,cy>  pixels, instead of --hfov\n"
        "  --distortion <k1,k2,p1,p2,k3>  lens distortion, plumb_bob (default none)\n"
        "  --tilt <deg>                -90 = straight down (default), 0 = forward\n"
        "  --pan <deg>                 turn the camera to the left (negative: right)\n"
        "  --roll <deg>                turn the image about the viewing direction\n"
        "  --mount-offset <x,y,z>      camera position on the body: forward, left, up (m)\n"
        "  --rate <hz>                 frames per second (default 10)\n"
        "Flight\n"
        "  --from <s> --to <s>         part of the bag to render, seconds from its start\n"
        "  --altitude <mode>           auto (default), ground (shift the flight so that it\n"
        "                              starts on the terrain), recorded, or metres to add\n"
        "  --min-agl <m>               no frames below this height above the terrain (default 1)\n"
        "  --fix-topic <t> --local-topic <t> --attitude-topic <t>   as picked by ogccdb-bag-info\n"
        "Image realism (applied in this order; all off by default)\n"
        "  --blur <px>                 lens blur, Gaussian sigma\n"
        "  --vignette <0..1>           how much darker the corners are\n"
        "  --gains <r,g,b>             white balance\n"
        "  --brightness <k> --contrast <k> --saturation <k> --gamma <g>\n"
        "  --noise <levels>            sensor noise, sigma in 8-bit levels\n"
        "  --jpeg-quality <1..100>     default 90\n"
        "  --realistic                 a moderate amount of each; later options adjust it\n"
        "Prior pose (the truth plus a dead-reckoning-like error)\n"
        "  --drift-velocity <v>        m/s per sqrt(s) on each horizontal axis (default 0.01)\n"
        "  --drift-height <v>          m per sqrt(s) (default 0.05)\n"
        "  --drift-yaw <v>             degrees per sqrt(s) (default 0.05)\n"
        "  --yaw-bias <deg>            heading error from the start (default 2)\n"
        "  --tilt-noise <deg>          roll and pitch noise (default 0.2)\n"
        "  --start-error <m>           horizontal position error at the start (default 0)\n"
        "  --seed <n>                  for the drift and the image noise (default 1)\n"
        "  --no-prior                  no prior topic\n"
        "Truth\n"
        "  --truth-every <n>           per-pixel ground points for every n-th frame\n"
        "                              (default 10; 0 = none; each file is 12 bytes a pixel)\n"
        "Bag\n"
        "  --image-topic <t>           default /camera/image_raw/compressed\n"
        "  --info-topic <t>            default /camera/camera_info\n"
        "  --prior-topic <t>           default /prior/geo_pose\n"
        "  --camera-frame <id>         default camera_optical_frame\n"
        "  --body-frame <id>           default base_link\n"
        "  --drop <topic>              leave a topic of the input bag out (repeatable)\n"
        "  --compression <c>           none (default), zstd or lz4\n"
        "  --overwrite                 replace a previous output in <output folder>\n"
        "Rendering\n"
        "  --screen-error <px>         LOD refinement threshold (default 1.5; lower = finer)\n"
        "\n"
        "Needs OpenGL 3.3 and a display; on a machine without one run it under xvfb-run.");
}

bool parse_numbers(const char* text, double* out, int count) {
    const char* p = text;
    for (int i = 0; i < count; ++i) {
        char* end = nullptr;
        out[i] = std::strtod(p, &end);
        if (end == p || !std::isfinite(out[i])) return false;
        p = end;
        if (i + 1 < count) {
            if (*p != ',') return false;
            ++p;
        }
    }
    return *p == '\0';
}

std::optional<Options> parse(int argc, char** argv) {
    Options o;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        // --option value, or --option=value
        std::optional<std::string> inline_value;
        if (const std::size_t eq = s.find('='); s.rfind("--", 0) == 0 && eq != std::string::npos) {
            inline_value = s.substr(eq + 1);
            s.erase(eq);
        }
        bool ok = true;
        auto next = [&]() -> const char* {
            if (inline_value) return inline_value->c_str();
            if (i + 1 >= argc) {
                ok = false;
                return "";
            }
            return argv[++i];
        };
        auto number = [&](double& v) { ok = parse_numbers(next(), &v, 1) && ok; };
        auto text = [&](std::string& v) { v = next(); };
        if (s == "-h" || s == "--help") return std::nullopt;
        if (s == "--size") {
            if (std::sscanf(next(), "%dx%d", &o.width, &o.height) != 2 || o.width < 16 || o.height < 16) ok = false;
        } else if (s == "--hfov") number(o.hfov);
        else if (s == "--intrinsics") {
            std::array<double, 4> k{};
            ok = parse_numbers(next(), k.data(), 4) && ok;
            o.intrinsics = k;
        } else if (s == "--distortion") ok = parse_numbers(next(), o.distortion.data(), 5) && ok;
        else if (s == "--tilt") number(o.mount.tilt_deg);
        else if (s == "--pan") number(o.mount.pan_deg);
        else if (s == "--roll") number(o.mount.roll_deg);
        else if (s == "--mount-offset") {
            double v[3] = {0, 0, 0};
            ok = parse_numbers(next(), v, 3) && ok;
            o.mount.offset = {v[0], v[1], v[2]};
        } else if (s == "--rate") number(o.rate);
        else if (s == "--from") { double v = 0; number(v); o.from = v; }
        else if (s == "--to") { double v = 0; number(v); o.to = v; }
        else if (s == "--altitude") text(o.altitude);
        else if (s == "--min-agl") number(o.min_agl);
        else if (s == "--fix-topic") text(o.track.fix_topic);
        else if (s == "--local-topic") text(o.track.local_topic);
        else if (s == "--attitude-topic") text(o.track.attitude_topic);
        else if (s == "--blur") number(o.realism.blur_sigma);
        else if (s == "--vignette") number(o.realism.vignette);
        else if (s == "--gains") ok = parse_numbers(next(), o.realism.gains.data(), 3) && ok;
        else if (s == "--brightness") number(o.realism.brightness);
        else if (s == "--contrast") number(o.realism.contrast);
        else if (s == "--saturation") number(o.realism.saturation);
        else if (s == "--gamma") number(o.realism.gamma);
        else if (s == "--noise") number(o.realism.noise_sigma);
        else if (s == "--jpeg-quality") { double v = 0; number(v); o.realism.jpeg_quality = static_cast<int>(v); }
        else if (s == "--realistic") o.realism = Realism::preset();
        else if (s == "--drift-velocity") number(o.drift.velocity_walk);
        else if (s == "--drift-height") number(o.drift.height_walk);
        else if (s == "--drift-yaw") number(o.drift.yaw_walk);
        else if (s == "--yaw-bias") number(o.drift.yaw_bias);
        else if (s == "--tilt-noise") number(o.drift.tilt_noise);
        else if (s == "--start-error") number(o.drift.start_error);
        else if (s == "--seed") { double v = 0; number(v); o.drift.seed = static_cast<std::uint64_t>(std::max(v, 0.0)); }
        else if (s == "--no-prior") o.prior = false;
        else if (s == "--truth-every") { double v = 0; number(v); o.truth_every = static_cast<int>(v); }
        else if (s == "--image-topic") text(o.image_topic);
        else if (s == "--info-topic") text(o.info_topic);
        else if (s == "--prior-topic") text(o.prior_topic);
        else if (s == "--camera-frame") text(o.camera_frame);
        else if (s == "--body-frame") text(o.body_frame);
        else if (s == "--drop") { std::string t; text(t); o.drop.push_back(t); }
        else if (s == "--compression") { text(o.compression); if (o.compression == "none") o.compression.clear(); }
        else if (s == "--overwrite") o.overwrite = true;
        else if (s == "--screen-error") number(o.screen_error);
        else if (!s.empty() && s[0] == '-' && s.size() > 1) {
            std::fprintf(stderr, "ogccdb-bag-render: unknown option %s\n", s.c_str());
            return std::nullopt;
        } else positional.push_back(s);
        if (!ok) {
            std::fprintf(stderr, "ogccdb-bag-render: bad or missing value for %s\n", s.c_str());
            return std::nullopt;
        }
    }
    if (positional.size() != 3) return std::nullopt;
    o.cdb = positional[0];
    o.bag = positional[1];
    o.out = positional[2];
    auto fail = [](const char* what) {
        std::fprintf(stderr, "ogccdb-bag-render: %s\n", what);
        return std::nullopt;
    };
    if (!(o.hfov > 1.0 && o.hfov < 170.0)) return fail("--hfov must be between 1 and 170 degrees");
    if (o.intrinsics && !((*o.intrinsics)[0] > 0.0 && (*o.intrinsics)[1] > 0.0)) return fail("--intrinsics: fx and fy must be positive");
    if (!(o.rate > 0.0 && o.rate <= 1000.0)) return fail("--rate must be positive");
    if (!(o.mount.tilt_deg >= -90.0 && o.mount.tilt_deg <= 90.0)) return fail("--tilt must be between -90 and 90 degrees");
    if (o.realism.jpeg_quality < 1 || o.realism.jpeg_quality > 100) return fail("--jpeg-quality must be 1..100");
    if (!(o.realism.gamma > 0.0)) return fail("--gamma must be positive");
    if (!(o.realism.vignette >= 0.0 && o.realism.vignette <= 1.0)) return fail("--vignette must be 0..1");
    if (o.truth_every < 0) return fail("--truth-every must be 0 or more");
    if (!McapWriter::supports(o.compression)) return fail("this build cannot write that --compression");
    if (o.altitude != "auto" && o.altitude != "ground" && o.altitude != "recorded") {
        double v = 0.0;
        if (!parse_numbers(o.altitude.c_str(), &v, 1)) return fail("--altitude takes auto, ground, recorded or metres");
    }
    return o;
}

/// A hidden window, for its OpenGL context.
struct GlContext {
    GLFWwindow* window = nullptr;

    GlContext() {
        if (!glfwInit()) throw std::runtime_error("cannot start GLFW (no display? try xvfb-run)");
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        window = glfwCreateWindow(64, 64, "ogccdb-bag-render", nullptr, nullptr);
        if (!window) {
            glfwTerminate();
            throw std::runtime_error("cannot create an OpenGL 3.3 core context");
        }
        glfwMakeContextCurrent(window);
        if (const char* missing = gl::load(reinterpret_cast<gl::LoadFn>(glfwGetProcAddress))) {
            const std::string name = missing;
            glfwDestroyWindow(window);
            glfwTerminate();
            throw std::runtime_error("OpenGL function " + name + " is missing");
        }
    }
    ~GlContext() {
        glfwDestroyWindow(window);
        glfwTerminate();
    }
    GlContext(const GlContext&) = delete;
    GlContext& operator=(const GlContext&) = delete;
};

/// The tiles to draw for `camera`, once every tile it needs is loaded.
const std::vector<DrawTile>& settle(Terrain& terrain, const Camera& camera) {
    const std::vector<DrawTile>* draw = &terrain.update(camera);
    for (int i = 0; i < 200; ++i) {
        const TerrainStats s = terrain.stats();
        if (s.queued == 0 && s.loading == 0) break;
        terrain.wait_idle();
        draw = &terrain.update(camera);
    }
    return *draw;
}

/// Terrain height under (lat, lon), from the finest tile the store has there:
/// looks straight down from just above, moving the eye to each better answer.
std::optional<double> terrain_height(Terrain& terrain, double lat, double lon, double first_guess) {
    const int res = terrain.options().mesh.resolution;
    const Vec3d up = ellipsoid_up(lat, lon), north = local_north(lat, lon);
    std::optional<double> ground;
    double guess = first_guess;
    int best_lod = std::numeric_limits<int>::min();
    for (int i = 0; i < 8; ++i) {
        const Vec3d eye = geodetic_to_ecef({lat, lon, guess + 30.0});
        const Camera camera = make_camera(eye, -up, north, 60.0, 640, 480, guess, 5.0e4);
        int lod = 0;
        const auto g = ground_height(settle(terrain, camera), lat, lon, res, &lod);
        if (!g) break;
        const bool better = lod > best_lod || std::fabs(*g - guess) > 0.01;
        ground = *g;
        guess = *g;
        best_lod = std::max(best_lod, lod);
        if (!better) break;
    }
    return ground;
}

/// Median of (header stamp - log time) on the position topic, nanoseconds: how
/// the publishers' clock relates to the recorder's. 0 if nothing is stamped.
std::int64_t clock_offset(Bag& bag, const std::string& topic_name) {
    const BagTopic* topic = bag.topic(topic_name);
    if (!topic) return 0;
    std::vector<std::int64_t> d;
    bag.read(
        [&](const BagMessage& m) {
            try {
                std::int64_t stamp = 0;
                if (message_kind(topic->type) == MessageKind::GeoPose) {
                    stamp = decode_geopose(m.data, m.size).header.ns;
                } else if (message_kind(topic->type) == MessageKind::NavSatFix) {
                    stamp = decode_navsatfix(m.data, m.size).header.ns;
                } else if (const auto p = decode_local_pose(topic->type, m.data, m.size)) {
                    stamp = p->header.ns;
                }
                if (stamp > 0) d.push_back(stamp - m.log_time_ns);
            } catch (const std::runtime_error&) {
                // a malformed message: skip it
            }
        },
        [&](const BagTopic& t) { return &t == topic; });
    if (d.empty()) return 0;
    std::nth_element(d.begin(), d.begin() + static_cast<std::ptrdiff_t>(d.size() / 2), d.end());
    return d[d.size() / 2];
}

std::string metadata_value(const BagTopic& t, const std::string& key) {
    for (const auto& [k, v] : t.metadata) {
        if (k == key) return v;
    }
    return {};
}

/// "ros_distro" of the input bag's metadata.yaml; empty if it has none.
std::string input_distro(const fs::path& bag) {
    std::ifstream in(fs::is_directory(bag) ? bag / "metadata.yaml" : bag.parent_path() / "metadata.yaml");
    for (std::string line; std::getline(in, line);) {
        const std::size_t at = line.find("ros_distro:");
        if (at == std::string::npos) continue;
        std::string v = line.substr(at + 11);
        v.erase(0, v.find_first_not_of(" \t\"'"));
        v.erase(v.find_last_not_of(" \t\r\"'") + 1);
        return v;
    }
    return {};
}

std::string json_string(const std::string& s) {
    std::string q = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') {
            q += '\\';
            q += c;
        } else if (static_cast<unsigned char>(c) < 0x20) {
            char b[8];
            std::snprintf(b, sizeof b, "\\u%04x", c);
            q += b;
        } else {
            q += c;
        }
    }
    return q + "\"";
}

std::string num(double v, int digits = 9) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*g", digits, v);
    return b;
}

struct Tick {
    double t = 0.0;             ///< seconds from the start of the bag (the track's time)
    std::int64_t stamp_ns = 0;  ///< header stamp
    std::int64_t log_ns = 0;    ///< when a recorder would have received it
};

int run(const Options& o) {
    // --- The flight ----------------------------------------------------------
    std::unique_ptr<Bag> bag = Bag::open(o.bag);
    const FlightTrack track = FlightTrack::build(*bag, o.track);
    const TrackInfo& info = track.info();
    std::printf("bag: %s, %.1f s, %llu messages; position from %s\n", o.bag.string().c_str(), track.bag_duration(),
                static_cast<unsigned long long>(bag->message_count()), info.position_source.c_str());
    if (info.attitude_topic.empty()) std::printf("  no attitude in the bag: the vehicle is level, nose along the track\n");

    const double t_from = std::max(track.start(), o.from.value_or(track.start()));
    const double t_to = std::min(track.end(), o.to.value_or(track.end()));
    if (!(t_to > t_from)) throw std::runtime_error("nothing to render between --from and --to");
    const std::int64_t offset_ns = clock_offset(*bag, info.fix_topic);
    std::vector<Tick> ticks;
    for (std::size_t k = 0;; ++k) {
        Tick tk;
        tk.t = t_from + static_cast<double>(k) / o.rate;
        if (tk.t > t_to + 1e-9) break;
        tk.stamp_ns = bag->start_ns() + std::llround(tk.t * 1e9);
        tk.log_ns = tk.stamp_ns - offset_ns;
        ticks.push_back(tk);
    }

    // --- Camera -----------------------------------------------------------------
    Intrinsics k = Intrinsics::from_hfov(o.width, o.height, o.hfov);
    if (o.intrinsics) {
        k.fx = (*o.intrinsics)[0];
        k.fy = (*o.intrinsics)[1];
        k.cx = (*o.intrinsics)[2];
        k.cy = (*o.intrinsics)[3];
    }
    k.d = o.distortion;
    const DistortionMap map = make_distortion_map(k);
    if (map.render_width < 2 || map.render_height < 2) throw std::runtime_error("the lens model leaves no image");
    const CameraAxes axes = mount_axes(o.mount);
    const Quat optical_to_body = optical_rotation(axes);
    std::printf("camera: %dx%d, fx %.2f fy %.2f cx %.2f cy %.2f, tilt %.1f pan %.1f roll %.1f, %.3g Hz\n", k.width, k.height,
                k.fx, k.fy, k.cx, k.cy, o.mount.tilt_deg, o.mount.pan_deg, o.mount.roll_deg, o.rate);
    if (!map.identity) {
        std::printf("  rendered as a %dx%d pinhole image and resampled through the lens model\n", map.render_width,
                    map.render_height);
        if (map.outside) std::printf("  %zu pixels are outside what the lens model can show and stay black\n", map.outside);
    }

    // --- Terrain and OpenGL -------------------------------------------------------
    TerrainOptions terrain_options;
    terrain_options.selection.max_screen_error = o.screen_error;
    Terrain terrain(o.cdb, terrain_options);
    if (!terrain.index().extent()) throw std::runtime_error("no Elevation or Imagery tiles in " + o.cdb.string());
    const int res = terrain.options().mesh.resolution;
    GlContext context;
    std::printf("OpenGL %s, %s\n", reinterpret_cast<const char*>(gl::GetString(GL_VERSION)),
                reinterpret_cast<const char*>(gl::GetString(GL_RENDERER)));
    Renderer renderer;
    RenderSettings settings;
    settings.lighting = false;   // the imagery has the light of its day; no second shading
    settings.sea = false;
    settings.graticule = false;
    settings.waves = false;
    Framebuffer colour(map.render_width, map.render_height, Framebuffer::Format::Rgba8);
    std::unique_ptr<Framebuffer> positions;
    if (o.truth_every > 0) {
        positions = std::make_unique<Framebuffer>(map.render_width, map.render_height, Framebuffer::Format::Rgba32f);
    }

    // --- Output folder ---------------------------------------------------------
    fs::path out = fs::absolute(o.out).lexically_normal();
    if (!out.has_filename()) out = out.parent_path();
    const std::string name = out.filename().string();
    const fs::path truth_dir = out / "truth", mcap_file = out / (name + "_0.mcap");
    std::error_code ec;
    if (fs::exists(out, ec) && fs::exists(o.bag, ec) && fs::equivalent(out, o.bag, ec)) {
        throw std::runtime_error("the output folder must not be the input bag");
    }
    if (fs::exists(out, ec) && !fs::is_empty(out, ec)) {
        if (!o.overwrite) throw std::runtime_error(out.string() + " is not empty; pick another folder or pass --overwrite");
        // Remove what an earlier run wrote, and nothing else.
        fs::remove(out / "metadata.yaml", ec);
        fs::remove(mcap_file, ec);
        if (fs::is_directory(truth_dir, ec)) {
            std::vector<fs::path> old;
            for (const auto& e : fs::directory_iterator(truth_dir, ec)) {
                const std::string ext = e.path().extension().string();
                if (ext == ".npy" || ext == ".jpg" || ext == ".csv" || ext == ".json") old.push_back(e.path());
            }
            for (const fs::path& p : old) fs::remove(p, ec);
        }
    }
    fs::create_directories(truth_dir);

    // --- Heights -------------------------------------------------------------------
    // A simulated flight often starts at a height that is not the terrain's:
    // by default a flight that starts at rest is shifted onto the terrain.
    const FlightPose first = track.at(track.start());
    const std::optional<double> start_ground =
        terrain_height(terrain, first.geodetic.lat, first.geodetic.lon, first.geodetic.height);
    double altitude_offset = 0.0;
    std::string altitude_mode = o.altitude;
    if (altitude_mode == "auto") {
        altitude_mode = first.ground_speed < 0.5 && std::fabs(first.climb) < 0.5 ? "ground" : "recorded";
    }
    if (altitude_mode == "ground") {
        if (!start_ground) throw std::runtime_error("no terrain under the start of the flight; use --altitude recorded or metres");
        altitude_offset = *start_ground - first.geodetic.height;
    } else if (altitude_mode != "recorded") {
        altitude_offset = std::atof(altitude_mode.c_str());
    }
    std::printf("heights: %s, %+.2f m added (flight starts at %.2f m, terrain there %s)\n", altitude_mode.c_str(),
                altitude_offset, first.geodetic.height,
                start_ground ? (num(*start_ground, 7) + " m").c_str() : "unknown");
    auto pose_at = [&](double t) {
        FlightPose p = track.at(t);
        p.position += ellipsoid_up(p.geodetic.lat, p.geodetic.lon) * altitude_offset;
        p.geodetic.height += altitude_offset;
        return p;
    };

    // The local frame of the truth files: east, north, up at the start of the flight.
    const FlightPose start_pose = pose_at(track.start());
    const Geodetic origin_g = start_pose.geodetic;
    const Vec3d origin = geodetic_to_ecef(origin_g);
    const Vec3d e0 = local_east(origin_g.lat, origin_g.lon), n0 = local_north(origin_g.lat, origin_g.lon),
                u0 = ellipsoid_up(origin_g.lat, origin_g.lon);
    auto to_local = [&](Vec3d v) { return Vec3d{dot(v, e0), dot(v, n0), dot(v, u0)}; };

    // --- The new bag -----------------------------------------------------------------
    McapWriterOptions writer_options;
    writer_options.compression = o.compression;
    McapWriter writer(mcap_file, writer_options);
    const std::string tf_topic = "/tf_static";
    // New topics follow the bag's own format: its QoS text, and type hashes if it has them.
    std::string qos_like;
    bool hashes = false;
    for (const BagTopic& t : bag->topics()) {
        if (qos_like.empty()) qos_like = metadata_value(t, "offered_qos_profiles");
        hashes = hashes || !metadata_value(t, "topic_type_hash").empty();
    }
    std::map<const BagTopic*, std::uint16_t> channel_of;
    std::optional<std::uint16_t> tf_channel;
    for (const BagTopic& t : bag->topics()) {
        const bool dropped = std::find(o.drop.begin(), o.drop.end(), t.name) != o.drop.end();
        const bool replaced = t.name == o.image_topic || t.name == o.info_topic || (o.prior && t.name == o.prior_topic);
        if (dropped || replaced) {
            std::printf("  leaving out %s%s\n", t.name.c_str(), replaced ? " (written anew)" : "");
            continue;
        }
        channel_of[&t] = writer.add_topic(t);
        if (t.name == tf_topic && t.type == "tf2_msgs/msg/TFMessage") tf_channel = channel_of[&t];
    }
    for (const std::string& d : o.drop) {
        if (!bag->topic(d)) std::printf("  note: --drop %s: the bag has no such topic\n", d.c_str());
    }
    auto new_topic = [&](const std::string& topic, const std::string& type, bool transient_local, bool estimate = false) {
        BagTopic t;
        t.name = topic;
        t.type = type;
        t.encoding = "cdr";
        t.schema_encoding = "ros2msg";
        t.schema = message_definition(type);
        // Reliable: a player then serves reliable and best-effort subscribers alike.
        t.metadata.emplace_back("offered_qos_profiles", qos_profile(true, transient_local, qos_like));
        if (hashes) t.metadata.emplace_back("topic_type_hash", type_hash(type));
        // So that nothing takes the prior for the flight's position when this bag is read again.
        if (estimate) t.metadata.emplace_back(kRoleKey, kRoleEstimate);
        return writer.add_topic(t);
    };
    const std::uint16_t image_channel = new_topic(o.image_topic, "sensor_msgs/msg/CompressedImage", false);
    const std::uint16_t info_channel = new_topic(o.info_topic, "sensor_msgs/msg/CameraInfo", false);
    if (!tf_channel) tf_channel = new_topic(tf_topic, "tf2_msgs/msg/TFMessage", true);
    std::optional<std::uint16_t> prior_channel;
    if (o.prior) prior_channel = new_topic(o.prior_topic, "geographic_msgs/msg/GeoPoseStamped", false, true);

    {
        // The mount, once, before everything else.
        Transform tf;
        const std::int64_t log_ns = std::min(bag->start_ns(), ticks.front().log_ns);
        tf.header = {log_ns + offset_ns, o.body_frame};
        tf.child_frame_id = o.camera_frame;
        tf.x = o.mount.offset.x;
        tf.y = o.mount.offset.y;
        tf.z = o.mount.offset.z;
        tf.rotation = optical_to_body;
        const std::vector<std::uint8_t> msg = encode_tf_message({tf});
        writer.write(*tf_channel, log_ns, msg.data(), msg.size());
    }

    std::FILE* csv = std::fopen((truth_dir / "frames.csv").string().c_str(), "w");
    if (!csv) throw std::runtime_error("cannot write " + (truth_dir / "frames.csv").string());
    std::fputs("frame,t,stamp_ns,image,xyz,lat,lon,height,agl,qx,qy,qz,qw,cam_e,cam_n,cam_u,cam_qx,cam_qy,cam_qz,cam_qw,"
               "prior_lat,prior_lon,prior_height,prior_qx,prior_qy,prior_qz,prior_qw,"
               "prior_err_e,prior_err_n,prior_err_u,prior_err_yaw\n",
               csv);

    // --- Frames -------------------------------------------------------------------------
    CameraInfo camera_info = k.camera_info();
    Drift drift(o.drift);
    double ground = start_ground ? *start_ground : start_pose.geodetic.height - 50.0;
    std::size_t images = 0, low = 0, xyz_files = 0, no_terrain = 0;
    std::uint64_t image_bytes = 0, xyz_bytes = 0;
    const auto wall_start = std::chrono::steady_clock::now();
    auto last_report = wall_start;

    auto render_tick = [&](std::size_t index) {
        const Tick& tk = ticks[index];
        const FlightPose pose = pose_at(tk.t);
        auto to_ecef = [&](Vec3d b) { return pose.forward * b.x + pose.left * b.y + pose.up * b.z; };
        const Vec3d eye = pose.position + to_ecef(o.mount.offset);
        const Vec3d forward = to_ecef(axes.forward), up = to_ecef(axes.up), right = to_ecef(axes.right);
        const Camera camera = make_camera(eye, forward, up, map.render_fov_y_deg, map.render_width, map.render_height,
                                          ground, 5.0e4);
        const std::vector<DrawTile>& draw = settle(terrain, camera);
        const Geodetic eye_g = ecef_to_geodetic(eye);
        std::optional<double> agl;
        if (const auto g = ground_height(draw, eye_g.lat, eye_g.lon, res)) {
            ground = *g;
            agl = eye_g.height - *g;
        } else {
            ++no_terrain;
        }

        // The prior: where a navigation system without GPS would believe it is.
        const DriftSample error = drift.at(tk.t - ticks.front().t);
        const Vec3d east = local_east(pose.geodetic.lat, pose.geodetic.lon),
                    north = local_north(pose.geodetic.lat, pose.geodetic.lon),
                    zenith = ellipsoid_up(pose.geodetic.lat, pose.geodetic.lon);
        GeoPose prior;
        const Geodetic prior_g = ecef_to_geodetic(pose.position + east * error.east + north * error.north + zenith * error.up);
        prior.header = {tk.stamp_ns, "wgs84"};
        prior.latitude = prior_g.lat;
        prior.longitude = prior_g.lon;
        prior.altitude = prior_g.height;
        prior.orientation = apply_drift(pose.attitude, error);
        if (prior_channel) {
            const std::vector<std::uint8_t> msg = encode_geopose(prior);
            writer.write(*prior_channel, tk.log_ns, msg.data(), msg.size());
        }

        bool image_written = false;
        std::string xyz_name;
        if (agl && *agl < o.min_agl) {
            ++low;
        } else {
            colour.bind();
            renderer.draw(draw, camera, map.render_width, map.render_height, settings);
            RgbImage image = colour.read_rgb();
            if (!map.identity) image = remap_rgb(image, map, k.width, k.height);
            apply_realism(image, o.realism, o.drift.seed * 1000003ull + index);
            const std::vector<std::uint8_t> jpeg = encode_jpeg(image, o.realism.jpeg_quality);
            if (jpeg.empty()) throw std::runtime_error("JPEG encoding failed");
            const Stamp stamp{tk.stamp_ns, o.camera_frame};
            const std::vector<std::uint8_t> msg = encode_compressed_image(stamp, "jpeg", jpeg.data(), jpeg.size());
            writer.write(image_channel, tk.log_ns, msg.data(), msg.size());
            camera_info.header = stamp;
            const std::vector<std::uint8_t> info_msg = encode_camera_info(camera_info);
            writer.write(info_channel, tk.log_ns, info_msg.data(), info_msg.size());
            image_written = true;
            image_bytes += jpeg.size();

            if (positions && images % static_cast<std::size_t>(o.truth_every) == 0) {
                // The ground point behind every pixel, through the same lens.
                positions->bind();
                renderer.draw_positions(draw, camera, map.render_width, map.render_height, origin, e0, n0, u0);
                std::vector<float> p = positions->read_rgba32f();
                if (!map.identity) p = remap_positions(p, map, k.width, k.height);
                const std::size_t n = static_cast<std::size_t>(k.width) * k.height;
                std::vector<float> xyz(n * 3, std::numeric_limits<float>::quiet_NaN());
                for (std::size_t i = 0; i < n; ++i) {
                    if (p[i * 4 + 3] > 0.0f) std::copy_n(&p[i * 4], 3, &xyz[i * 3]);  // w = 0: no terrain there
                }
                char stem[32];
                std::snprintf(stem, sizeof stem, "%06zu", index);
                xyz_name = std::string(stem) + ".npy";
                if (!write_npy((truth_dir / xyz_name).string(), xyz.data(),
                               {static_cast<std::size_t>(k.height), static_cast<std::size_t>(k.width), 3})) {
                    throw std::runtime_error("cannot write " + (truth_dir / xyz_name).string());
                }
                std::ofstream jpg(truth_dir / (std::string(stem) + ".jpg"), std::ios::binary | std::ios::trunc);
                jpg.write(reinterpret_cast<const char*>(jpeg.data()), static_cast<std::streamsize>(jpeg.size()));
                ++xyz_files;
                xyz_bytes += xyz.size() * sizeof(float);
            }
            ++images;
        }

        // The truth of this tick: the vehicle, and the camera in the local frame.
        const Vec3d c = to_local(eye - origin);
        const Quat cq = from_columns(to_local(right), to_local(-up), to_local(forward));
        std::fprintf(csv,
                     "%zu,%.4f,%lld,%d,%s,%.9f,%.9f,%.4f,%s,%.9f,%.9f,%.9f,%.9f,%.4f,%.4f,%.4f,%.9f,%.9f,%.9f,%.9f,"
                     "%.9f,%.9f,%.4f,%.9f,%.9f,%.9f,%.9f,%.4f,%.4f,%.4f,%.5f\n",
                     index, tk.t, static_cast<long long>(tk.stamp_ns), image_written ? 1 : 0, xyz_name.c_str(),
                     pose.geodetic.lat, pose.geodetic.lon, pose.geodetic.height, agl ? num(*agl, 7).c_str() : "",
                     pose.attitude.x, pose.attitude.y, pose.attitude.z, pose.attitude.w, c.x, c.y, c.z, cq.x, cq.y, cq.z,
                     cq.w, prior.latitude, prior.longitude, prior.altitude, prior.orientation.x, prior.orientation.y,
                     prior.orientation.z, prior.orientation.w, error.east, error.north, error.up, error.yaw_deg);

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report > std::chrono::seconds(5) || index + 1 == ticks.size()) {
            last_report = now;
            const double elapsed = std::chrono::duration<double>(now - wall_start).count();
            std::printf("  %zu / %zu  t %.1f s  %s  %.1f frames/s\n", index + 1, ticks.size(), tk.t,
                        agl ? ("AGL " + num(*agl, 4) + " m").c_str() : "no terrain below",
                        (index + 1) / std::max(elapsed, 1e-3));
            std::fflush(stdout);
        }
    };

    // Copy the bag in its order, rendering each frame when its time comes.
    std::size_t next = 0;
    auto render_until = [&](std::int64_t log_ns) {
        while (next < ticks.size() && ticks[next].log_ns <= log_ns) render_tick(next++);
    };
    bag->read([&](const BagMessage& m) {
        const auto it = channel_of.find(m.topic);
        if (it == channel_of.end()) return;
        render_until(m.log_time_ns);
        writer.write(it->second, m.log_time_ns, m.data, m.size, m.publish_time_ns);
    });
    render_until(std::numeric_limits<std::int64_t>::max());
    std::fclose(csv);
    writer.close();
    Framebuffer::unbind();

    // --- Bag folder and truth description ----------------------------------------------
    const bool old_qos = qos_profile(true, false, qos_like).find("reliability: 1") != std::string::npos;
    if (!old_qos) {
        std::string distro = input_distro(o.bag);
        if (distro.empty()) distro = "jazzy";
        write_bag_metadata(out, mcap_file.filename().string(), writer.topics(), writer.start_ns(), writer.end_ns(), distro);
    } else {
        std::printf("note: the input is in an older bag format; no metadata.yaml written (the .mcap plays by itself,\n"
                    "      or `ros2 bag reindex` makes one)\n");
    }

    std::ofstream js(truth_dir / "camera.json", std::ios::trunc);
    auto vec = [](const std::vector<double>& v) {
        std::string s = "[";
        for (std::size_t i = 0; i < v.size(); ++i) s += (i ? ", " : "") + num(v[i], 12);
        return s + "]";
    };
    js << "{\n"
       << "  \"input_bag\": " << json_string(o.bag.string()) << ",\n"
       << "  \"cdb\": " << json_string(o.cdb.string()) << ",\n"
       << "  \"bag_file\": " << json_string(mcap_file.filename().string()) << ",\n"
       << "  \"topics\": {\"image\": " << json_string(o.image_topic) << ", \"camera_info\": " << json_string(o.info_topic)
       << ", \"mount\": " << json_string(tf_topic) << ", \"prior\": " << json_string(o.prior ? o.prior_topic : "") << "},\n"
       << "  \"frame_ids\": {\"camera\": " << json_string(o.camera_frame) << ", \"body\": " << json_string(o.body_frame) << "},\n"
       << "  \"camera\": {\"width\": " << k.width << ", \"height\": " << k.height << ", \"fx\": " << num(k.fx, 12)
       << ", \"fy\": " << num(k.fy, 12) << ", \"cx\": " << num(k.cx, 12) << ", \"cy\": " << num(k.cy, 12)
       << ", \"distortion_model\": \"plumb_bob\", \"d\": " << vec({k.d.begin(), k.d.end()}) << ", \"rate_hz\": "
       << num(o.rate) << "},\n"
       << "  \"mount\": {\"tilt_deg\": " << num(o.mount.tilt_deg) << ", \"pan_deg\": " << num(o.mount.pan_deg)
       << ", \"roll_deg\": " << num(o.mount.roll_deg) << ", \"offset_flu_m\": "
       << vec({o.mount.offset.x, o.mount.offset.y, o.mount.offset.z}) << ", \"optical_to_body_xyzw\": "
       << vec({optical_to_body.x, optical_to_body.y, optical_to_body.z, optical_to_body.w}) << "},\n"
       << "  \"local_frame\": {\"lat\": " << num(origin_g.lat, 12) << ", \"lon\": " << num(origin_g.lon, 12)
       << ", \"height\": " << num(origin_g.height, 9)
       << ", \"axes\": \"east, north, up; metres; tangent to the WGS-84 ellipsoid at this point\"},\n"
       << "  \"heights\": {\"mode\": " << json_string(altitude_mode) << ", \"added_m\": " << num(altitude_offset)
       << ", \"reference\": \"WGS-84 ellipsoid\"},\n"
       << "  \"clock\": {\"header_minus_log_ns\": " << offset_ns << "},\n"
       << "  \"realism\": {\"blur_sigma\": " << num(o.realism.blur_sigma) << ", \"vignette\": " << num(o.realism.vignette)
       << ", \"gains\": " << vec({o.realism.gains.begin(), o.realism.gains.end()}) << ", \"brightness\": "
       << num(o.realism.brightness) << ", \"contrast\": " << num(o.realism.contrast) << ", \"saturation\": "
       << num(o.realism.saturation) << ", \"gamma\": " << num(o.realism.gamma) << ", \"noise_sigma\": "
       << num(o.realism.noise_sigma) << ", \"jpeg_quality\": " << o.realism.jpeg_quality << "},\n"
       << "  \"prior_drift\": {\"velocity_walk\": " << num(o.drift.velocity_walk) << ", \"height_walk\": "
       << num(o.drift.height_walk) << ", \"yaw_walk\": " << num(o.drift.yaw_walk) << ", \"yaw_bias\": "
       << num(o.drift.yaw_bias) << ", \"tilt_noise\": " << num(o.drift.tilt_noise) << ", \"start_error\": "
       << num(o.drift.start_error) << ", \"seed\": " << o.drift.seed << "},\n"
       << "  \"counts\": {\"ticks\": " << ticks.size() << ", \"images\": " << images << ", \"below_min_agl\": " << low
       << ", \"xyz_files\": " << xyz_files << "}\n"
       << "}\n";
    js.close();
    if (!js) throw std::runtime_error("cannot write " + (truth_dir / "camera.json").string());

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    std::printf("wrote %s: %llu messages, %zu images (%.1f MB of JPEG) in %.0f s\n", out.string().c_str(),
                static_cast<unsigned long long>(writer.message_count()), images, image_bytes / 1048576.0, elapsed);
    if (low) std::printf("  %zu frames left out below %.1f m above the terrain\n", low, o.min_agl);
    if (no_terrain) std::printf("  %zu frames had no terrain under the vehicle\n", no_terrain);
    std::printf("  truth: %s (frames.csv, camera.json, %zu ground-point files, %.1f MB)\n", truth_dir.string().c_str(),
                xyz_files, xyz_bytes / 1048576.0);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const std::optional<Options> options = parse(argc, argv);
    if (!options) {
        usage();
        return 2;
    }
    try {
        return run(*options);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ogccdb-bag-render: %s\n", e.what());
        return 1;
    }
}
