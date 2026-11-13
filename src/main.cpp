// ogccdb-viewer: a 3D viewer for OGC CDB 1.3 Elevation and Imagery.
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "file_browser.hpp"
#include "flight_player.hpp"
#include "fly_camera.hpp"
#include "geoid.hpp"
#include "globe_camera.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ogccdb/cdb13/lod.hpp"
#include "ogccdb/render/camera.hpp"
#include "ogccdb/render/gl.hpp"
#include "ogccdb/render/image.hpp"
#include "ogccdb/render/renderer.hpp"
#include "ogccdb/runtime/geodesy.hpp"
#include "ogccdb/runtime/terrain.hpp"

using namespace ogccdb;
using namespace ogccdb::runtime;
using namespace ogccdb::render;
using ogccdb::flight::FlightPose;
using ogccdb::flight::camera_axes;

namespace {

struct Args {
    std::filesystem::path cdb;
    std::optional<double> lat, lon, distance, heading, pitch, eye_height, fov;
    bool fly = false;
    int width = 1600, height = 1000;
    std::optional<std::string> screenshot;
    float exaggeration = 1.0f;
    double max_screen_error = 1.5;
    bool lod_colors = false;
    bool hide_ui = false;
    std::string sea = "auto";
    double sea_offset = 0.0;
    std::optional<double> wave_height, wind_from;
    int frames = 1;
    std::optional<std::string> bag;
    std::optional<double> bag_time, play_speed, mount_pitch;
    bool play = false, no_drone_camera = false;
};

void usage() {
    std::puts(
        "usage: ogccdb-viewer <cdb-root> [options]\n"
        "  --lat <deg> --lon <deg>     view target (default: centre of the store)\n"
        "  --distance <m>              eye distance from the target\n"
        "  --heading <deg>             0 = looking north\n"
        "  --pitch <deg>               -90 = straight down (default -60)\n"
        "  --fov <deg>                 vertical field of view (default 45)\n"
        "  --fly                       start in free-fly mode (at the orbit eye, or at --lat --lon --height)\n"
        "  --height <m>                free-fly eye height above the ellipsoid\n"
        "  --exaggeration <k>          vertical exaggeration (default 1)\n"
        "  --screen-error <px>         LOD refinement threshold (default 1.5)\n"
        "  --lod-colors                tint tiles by LOD\n"
        "  --sea <ref>                 sea surface: auto (best installed geoid, default),\n"
        "                              egm2008, egm96, ellipsoid or off\n"
        "  --sea-offset <m>            raise (or lower) the sea above its reference\n"
        "  --waves <m>                 wave height, crest to trough (default 1.5; 0 = calm)\n"
        "  --wind <deg>                direction the wind blows from (default 240)\n"
        "  --size <w>x<h>              window size (default 1600x1000)\n"
        "  --screenshot <file.png>     render until all tiles are loaded, save, exit\n"
        "  --frames <n>                with --screenshot: save n frames at 30 fps as file_0000.png ...\n"
        "  --no-ui                     hide the panels\n"
        "orbit:    left drag pan, right drag rotate/tilt, wheel zoom,\n"
        "          WASD pan, Q/E rotate, R/F tilt, +/- zoom\n"
        "free fly: mouse look (click or Tab captures, Esc/Tab releases), WASD move,\n"
        "          E/Space up, Q/C down, Shift faster, Ctrl slower, wheel speed,\n"
        "          arrows look, G walks on the ground, hold right button to zoom\n"
        "[ ] narrow / widen the field of view\n"
        "V switches orbit / free fly, H toggles the panels\n"
        "flight playback (ROS 2 bag, MCAP or SQLite storage):\n"
        "  --bag <folder|.mcap|.db3|.tar>  open a bag; a folder of bags opens the bag browser\n"
        "  --bag-time <s>              start at this many seconds into the bag\n"
        "  --play [--speed <k>]        start playing (k times real time)\n"
        "  --camera-tilt <deg>         drone camera tilt from the body's forward (default -90, down)\n"
        "  --no-drone-camera           start with the orbit / fly camera instead\n"
        "  keys: P play/pause, B drone camera on/off, Left/Right seek 5 s in the drone camera");
}

std::optional<Args> parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string s = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        auto number = [&](std::optional<double>& out) {
            const char* v = next();
            if (!v) return false;
            out = std::atof(v);
            return true;
        };
        if (s == "-h" || s == "--help") return std::nullopt;
        if (s == "--lat") { if (!number(a.lat)) return std::nullopt; }
        else if (s == "--lon") { if (!number(a.lon)) return std::nullopt; }
        else if (s == "--distance") { if (!number(a.distance)) return std::nullopt; }
        else if (s == "--heading") { if (!number(a.heading)) return std::nullopt; }
        else if (s == "--pitch") { if (!number(a.pitch)) return std::nullopt; }
        else if (s == "--fov") { if (!number(a.fov)) return std::nullopt; }
        else if (s == "--height") { if (!number(a.eye_height)) return std::nullopt; }
        else if (s == "--fly") a.fly = true;
        else if (s == "--bag") { const char* v = next(); if (!v) return std::nullopt; a.bag = v; }
        else if (s == "--bag-time") { if (!number(a.bag_time)) return std::nullopt; }
        else if (s == "--speed") { if (!number(a.play_speed)) return std::nullopt; }
        else if (s == "--camera-tilt") { if (!number(a.mount_pitch)) return std::nullopt; }
        else if (s == "--play") a.play = true;
        else if (s == "--no-drone-camera") a.no_drone_camera = true;
        else if (s == "--exaggeration") { const char* v = next(); if (!v) return std::nullopt; a.exaggeration = static_cast<float>(std::atof(v)); }
        else if (s == "--screen-error") { const char* v = next(); if (!v) return std::nullopt; a.max_screen_error = std::atof(v); }
        else if (s == "--lod-colors") a.lod_colors = true;
        else if (s == "--no-ui") a.hide_ui = true;
        else if (s == "--sea") {
            const char* v = next();
            if (!v) return std::nullopt;
            a.sea = v;
            if (a.sea != "auto" && a.sea != "egm2008" && a.sea != "egm96" && a.sea != "ellipsoid" && a.sea != "off") {
                std::fprintf(stderr, "--sea takes auto, egm2008, egm96, ellipsoid or off\n");
                return std::nullopt;
            }
        } else if (s == "--sea-offset") { const char* v = next(); if (!v) return std::nullopt; a.sea_offset = std::atof(v); }
        else if (s == "--waves") { if (!number(a.wave_height)) return std::nullopt; }
        else if (s == "--wind") { if (!number(a.wind_from)) return std::nullopt; }
        else if (s == "--frames") { const char* v = next(); if (!v) return std::nullopt; a.frames = std::max(1, std::atoi(v)); }
        else if (s == "--size") {
            const char* v = next();
            if (!v || std::sscanf(v, "%dx%d", &a.width, &a.height) != 2) return std::nullopt;
        } else if (s == "--screenshot") {
            const char* v = next();
            if (!v) return std::nullopt;
            a.screenshot = v;
        } else if (!s.empty() && s[0] == '-') {
            std::fprintf(stderr, "unknown option %s\n", s.c_str());
            return std::nullopt;
        } else if (a.cdb.empty()) {
            a.cdb = s;
        } else {
            return std::nullopt;
        }
    }
    if (a.cdb.empty()) return std::nullopt;
    return a;
}

void fit_store(GlobeCamera& cam, const TileIndex& index) {
    const auto e = index.extent();
    if (!e) return;
    cam.lat = (e->south + e->north) / 2.0;
    cam.lon = (e->west + e->east) / 2.0;
    const double size = std::max(e->north - e->south, (e->east - e->west) * std::cos(cam.lat * kDegToRad)) * kDegToRad *
                        kMeanEarthRadius;
    cam.distance = size / (2.0 * std::tan(cam.fov_y_deg * kDegToRad / 2.0)) * 1.3;
    cam.heading = 0.0;
    cam.pitch = -60.0;
}

/// The imaginary sea: a reference surface (geoid or ellipsoid) that runs on
/// under the terrain, like mean sea level in geodesy, plus an offset.
struct Sea {
    enum Reference { Auto, Egm2008, Egm96, Ellipsoid };
    Reference reference = Auto;
    float offset = 0.0f;  ///< metres above the reference (raise it to flood)
    Geoid egm2008{3855}, egm96{5773};

    /// The geoid in use, or nullptr for the ellipsoid.
    Geoid* geoid() {
        switch (reference) {
            case Auto: return egm2008.available() ? &egm2008 : egm96.available() ? &egm96 : nullptr;
            case Egm2008: return egm2008.available() ? &egm2008 : nullptr;
            case Egm96: return egm96.available() ? &egm96 : nullptr;
            case Ellipsoid: return nullptr;
        }
        return nullptr;
    }
    const char* name(const Geoid* g) const {
        return g == &egm2008 ? "geoid EGM2008" : g == &egm96 ? "geoid EGM96" : "ellipsoid";
    }
    /// Sea height above the ellipsoid in metres.
    static double height(Geoid* g, float offset, double lat, double lon) {
        double n = 0.0;
        if (g) {
            if (const auto h = g->height(lat, lon)) n = *h;
        }
        return n + offset;
    }
};

/// Both cameras; one of them drives the view.
struct Views {
    GlobeCamera orbit;
    FlyCamera fly;
    bool flying = false;
    double ground = 0.0;      ///< terrain height under the orbit target
    double eye_ground = 0.0;  ///< terrain height under the free-fly eye
    // Free-fly controls.
    bool mouse_captured = false;  ///< mouse look: cursor hidden and locked to the window
    bool walk = false;            ///< keep the eye at walk_height above the ground
    float walk_height = 1.8f;     ///< metres
    float sensitivity = 0.10f;    ///< degrees per mouse count
    bool invert_y = false;
};

void set_fov(Views& v, double deg) {
    deg = std::clamp(deg, 5.0, 120.0);
    v.orbit.fov_y_deg = deg;
    v.fly.fov_y_deg = deg;
}

void orbit_to_fly(Views& v) {
    v.fly.position = v.orbit.eye();
    v.fly.heading = v.orbit.heading;
    v.fly.pitch = v.orbit.pitch;
    v.fly.fov_y_deg = v.orbit.fov_y_deg;
    v.fly.clamp();
}

/// Orbit around the ground point the free-fly view looks at, or around the
/// point below the eye when it looks up or is underground.
void fly_to_orbit(Views& v) {
    GlobeCamera& o = v.orbit;
    const FlyCamera& f = v.fly;
    const Geodetic g = ecef_to_geodetic(f.position);
    const double above = g.height - v.eye_ground;
    o.heading = f.heading;
    o.fov_y_deg = f.fov_y_deg;
    if (f.pitch < -5.0 && above > 1.0) {
        const double d = above / std::sin(-f.pitch * kDegToRad);
        const Geodetic t = ecef_to_geodetic(f.position + f.forward() * d);
        o.lat = t.lat;
        o.lon = t.lon;
        o.height = t.height;
        o.distance = d;
        o.pitch = f.pitch;
    } else {
        o.lat = g.lat;
        o.lon = g.lon;
        o.height = v.eye_ground;
        o.distance = std::max(std::fabs(above), 50.0) * 1.5;
        o.pitch = -45.0;
    }
    o.clamp();
}

struct Input {
    bool dragging_left = false, dragging_right = false;
    double last_x = 0, last_y = 0;
    double scroll = 0;
};
Input g_input;

void scroll_callback(GLFWwindow* w, double xoff, double yoff) {
    ImGui_ImplGlfw_ScrollCallback(w, xoff, yoff);
    if (!ImGui::GetIO().WantCaptureMouse) g_input.scroll += yoff;
}

void ui(Views& v, Terrain& terrain, Renderer& renderer, RenderSettings& settings, SelectionOptions& sel, Sea& sea,
        FlightPlayer& player, FileBrowser& browser, std::optional<double> agl, const Args& args, double fps,
        bool& show_ui) {
    if (!show_ui) return;
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("ogccdb-viewer", &show_ui);
    ImGui::TextWrapped("%s", args.cdb.string().c_str());
    const TileIndex& idx = terrain.index();
    ImGui::Text("Elevation tiles %zu, Imagery tiles %zu", idx.elevation_count(), idx.imagery_count());
    if (const auto e = idx.extent()) ImGui::Text("Extent %.0f..%.0f N, %.0f..%.0f E", e->south, e->north, e->west, e->east);

    flight_ui(player, browser, agl);
    if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Field of view, shared by both cameras. The runtime picks LODs for it,
        // so a narrow view (telephoto) loads finer tiles far away.
        float fov = static_cast<float>(v.flying ? v.fly.fov_y_deg : v.orbit.fov_y_deg);
        if (ImGui::SliderFloat("FOV", &fov, 5.0f, 120.0f, "%.0f deg vertical")) set_fov(v, fov);
        ImGui::SameLine();
        if (ImGui::SmallButton("45")) set_fov(v, 45.0);
        {
            const ImVec2 ds = ImGui::GetIO().DisplaySize;
            const double hfov = 2.0 * std::atan(std::tan(v.fly.fov_y_deg * kDegToRad / 2.0) * ds.x / std::max(ds.y, 1.0f)) * kRadToDeg;
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ImGui::TextWrapped("%.0f deg horizontal. [ ] change it; in free fly hold the right button to zoom.", hfov);
            ImGui::PopStyleColor();
        }
        int mode = v.flying ? 1 : 0;
        ImGui::RadioButton("Orbit", &mode, 0);
        ImGui::SameLine();
        ImGui::RadioButton("Free fly", &mode, 1);
        ImGui::SameLine();
        ImGui::TextDisabled("(V)");
        if (mode == 1 && !v.flying) {
            orbit_to_fly(v);
            v.flying = true;
        } else if (mode == 0 && v.flying) {
            fly_to_orbit(v);
            v.flying = false;
        }
        if (!v.flying) {
            GlobeCamera& cam = v.orbit;
            const Geodetic eye = ecef_to_geodetic(cam.eye());
            ImGui::Text("Target %.5f, %.5f  ground %.0f m", cam.lat, cam.lon, v.ground);
            ImGui::Text("Eye height %.0f m, distance %.0f m", eye.height, cam.distance);
            float heading = static_cast<float>(cam.heading), pitch = static_cast<float>(cam.pitch);
            if (ImGui::SliderFloat("Heading", &heading, 0.0f, 360.0f, "%.0f deg")) cam.heading = heading;
            if (ImGui::SliderFloat("Pitch", &pitch, -89.9f, -1.0f, "%.0f deg")) cam.pitch = pitch;
            if (ImGui::Button("Fit store")) fit_store(cam, idx);
            ImGui::SameLine();
            if (ImGui::Button("Top view")) cam.pitch = -89.9;
            ImGui::SameLine();
            if (ImGui::Button("North up")) cam.heading = 0.0;
        } else {
            FlyCamera& cam = v.fly;
            const Geodetic eye = ecef_to_geodetic(cam.position);
            const double above = eye.height - v.eye_ground;
            ImGui::Text("Eye %.5f, %.5f", eye.lat, eye.lon);
            ImGui::Text("Height %.1f m, %s ground by %.1f m", eye.height, above >= 0 ? "above" : "BELOW", std::fabs(above));
            ImGui::Text("Speed %.0f m/s", cam.speed(above));
            float heading = static_cast<float>(cam.heading), pitch = static_cast<float>(cam.pitch);
            float speed = static_cast<float>(cam.speed_factor);
            if (ImGui::SliderFloat("Heading", &heading, 0.0f, 360.0f, "%.0f deg")) cam.heading = heading;
            if (ImGui::SliderFloat("Pitch", &pitch, -89.9f, 89.9f, "%.0f deg")) cam.pitch = pitch;
            if (ImGui::SliderFloat("Speed", &speed, 0.01f, 100.0f, "%.2fx", ImGuiSliderFlags_Logarithmic)) {
                cam.speed_factor = speed;
            }
            ImGui::Checkbox("Walk on ground (G)", &v.walk);
            if (v.walk) ImGui::SliderFloat("Eye height", &v.walk_height, 0.5f, 50.0f, "%.1f m", ImGuiSliderFlags_Logarithmic);
            ImGui::SliderFloat("Mouse sensitivity", &v.sensitivity, 0.02f, 0.5f, "%.2f", ImGuiSliderFlags_Logarithmic);
            ImGui::Checkbox("Invert Y", &v.invert_y);
            if (ImGui::Button("Level")) cam.pitch = 0.0;
            ImGui::SameLine();
            if (ImGui::Button("Look up")) cam.pitch = 60.0;
            ImGui::SameLine();
            if (ImGui::Button("Go 100 m below ground")) {
                cam.position = geodetic_to_ecef({eye.lat, eye.lon, v.eye_ground - 100.0});
                cam.pitch = 30.0;
                v.walk = false;
            }
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ImGui::TextWrapped("Click the view or press Tab for mouse look; Esc or Tab releases the mouse.");
            ImGui::PopStyleColor();
        }
    }
    if (ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Imagery", &settings.imagery);
        ImGui::SameLine();
        ImGui::Checkbox("Lighting", &settings.lighting);
        ImGui::SameLine();
        ImGui::Checkbox("Haze", &settings.haze);
        ImGui::Checkbox("Wireframe", &settings.wireframe);
        ImGui::SameLine();
        ImGui::Checkbox("LOD colours", &settings.lod_colors);
        ImGui::Checkbox("Sea", &settings.sea);
        ImGui::SameLine();
        ImGui::Checkbox("Graticule", &settings.graticule);
        if (settings.sea) {
            int ref = sea.reference;
            ImGui::Combo("Sea reference", &ref, "Geoid, best installed\0Geoid EGM2008\0Geoid EGM96\0Ellipsoid\0");
            sea.reference = static_cast<Sea::Reference>(ref);
            Geoid* g = sea.geoid();
            const Geodetic at = v.flying ? ecef_to_geodetic(v.fly.position) : Geodetic{v.orbit.lat, v.orbit.lon, 0.0};
            const double h = Sea::height(g, sea.offset, at.lat, at.lon);
            ImGui::Text("Using the %s: sea at %.1f m here", sea.name(g), h);
            if ((sea.reference == Sea::Egm2008 || sea.reference == Sea::Auto) && !sea.egm2008.available()) {
                ImGui::TextDisabled("EGM2008: %s", sea.egm2008.error().c_str());
            }
            if (sea.reference == Sea::Egm96 && !sea.egm96.available()) {
                ImGui::TextDisabled("EGM96: %s", sea.egm96.error().c_str());
            }
            ImGui::DragFloat("Sea offset", &sea.offset, 1.0f, -1000.0f, 9000.0f, "%+.0f m");
            ImGui::SameLine();
            if (ImGui::SmallButton("0")) sea.offset = 0.0f;
            ImGui::Checkbox("Waves", &settings.waves);
            if (settings.waves) {
                ImGui::SliderFloat("Wave height", &settings.wave_height, 0.1f, 8.0f, "%.1f m", ImGuiSliderFlags_Logarithmic);
                ImGui::SliderFloat("Wind from", &settings.wind_from, 0.0f, 360.0f, "%.0f deg");
            }
            ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyle().Colors[ImGuiCol_TextDisabled]);
            ImGui::TextWrapped("Use the geoid if the store holds ellipsoid heights, the ellipsoid if it holds "
                               "heights above sea level. Raise the offset to flood the terrain.");
            ImGui::PopStyleColor();
        }
        ImGui::SliderFloat("Exaggeration", &settings.exaggeration, 1.0f, 5.0f, "%.1fx");
        float err = static_cast<float>(sel.max_screen_error);
        if (ImGui::SliderFloat("Screen error", &err, 0.5f, 16.0f, "%.1f px", ImGuiSliderFlags_Logarithmic)) {
            sel.max_screen_error = err;
        }
        ImGui::SliderInt("Max LOD", &sel.max_lod, cdb13::kMinLod, cdb13::kMaxLod);
    }
    if (ImGui::CollapsingHeader("Statistics", ImGuiTreeNodeFlags_DefaultOpen)) {
        const TerrainStats s = terrain.stats();
        const RenderStats& r = renderer.stats();
        ImGui::Text("%.0f fps, %zu tiles, %.1fk triangles", fps, s.drawn, r.triangles / 1000.0);
        std::string lods;
        for (std::size_t i = 0; i < s.drawn_per_lod.size(); ++i) {
            if (s.drawn_per_lod[i]) lods += "L" + std::to_string(static_cast<int>(i) + cdb13::kMinLod) + ":" +
                                            std::to_string(s.drawn_per_lod[i]) + " ";
        }
        ImGui::TextWrapped("Drawn per LOD: %s", lods.c_str());
        ImGui::Text("Queued %zu, loading %zu, culled %zu", s.queued, s.loading, s.culled);
        ImGui::Text("Loaded %zu, failed %zu", s.loaded_total, s.failed_total);
        ImGui::Text("CPU tiles %zu (%.0f MB), sources %.0f MB", s.resident, s.tile_bytes / 1048576.0,
                    s.source_bytes / 1048576.0);
        ImGui::Text("GPU tiles %zu (%.0f MB), uploads %zu", r.gpu_tiles, r.gpu_bytes / 1048576.0, r.uploads);
        if (!s.last_error.empty()) ImGui::TextWrapped("Last error: %s", s.last_error.c_str());
    }
    if (ImGui::CollapsingHeader("Help")) {
        ImGui::TextWrapped(
            "Orbit: left drag pan, right drag rotate and tilt, wheel zoom, "
            "WASD pan, Q/E rotate, R/F tilt, +/- zoom.\n"
            "Free fly: mouse look (click the view or Tab to capture, Esc/Tab to release), "
            "WASD move, E/Space up, Q/C down, Shift faster, Ctrl slower, wheel speed, "
            "arrows look, G walk on ground.\n"
            "V: orbit / free fly. H: hide panels.\n"
            "Flight: P play/pause, B drone camera, Left/Right seek 5 s (drone camera).");
    }
    ImGui::End();
}

/// Flight path (flown part bright, the rest faint) and, outside the drone
/// camera, the vehicle: its arms, nose and the camera's view on the ground.
void draw_flight(Renderer& renderer, FlightPlayer& player, const FlightPose& p, bool drone_view, const Camera& rc,
                 double ground_below) {
    if (player.show_path) {
        const auto& path = player.flight_path();
        const auto& times = player.track().path_times();
        const std::size_t split =
            static_cast<std::size_t>(std::upper_bound(times.begin(), times.end(), player.time) - times.begin());
        const float done[4] = {1.0f, 0.75f, 0.15f, 1.0f}, todo[4] = {0.9f, 0.9f, 0.95f, 0.45f};
        if (split >= 1) {
            std::vector<Vec3d> a(path.begin(), path.begin() + static_cast<std::ptrdiff_t>(split));
            a.push_back(p.position);
            renderer.draw_lines(a, GL_LINE_STRIP, done, rc);
        }
        if (split < path.size()) {
            std::vector<Vec3d> b{p.position};
            b.insert(b.end(), path.begin() + static_cast<std::ptrdiff_t>(split), path.end());
            renderer.draw_lines(b, GL_LINE_STRIP, todo, rc);
        }
    }
    if (drone_view) return;
    // The vehicle, drawn large enough to see from afar.
    const double size = std::max(1.0, length(p.position - rc.position) * 0.03);
    const Vec3d c = p.position;
    const Vec3d f = p.forward * size, l = p.left * size;
    const float white[4] = {1, 1, 1, 1}, red[4] = {1.0f, 0.25f, 0.2f, 1}, cyan[4] = {0.3f, 0.9f, 1.0f, 0.9f};
    renderer.draw_lines({c + (f + l) * 0.7, c - (f + l) * 0.7, c + (f - l) * 0.7, c - (f - l) * 0.7}, GL_LINES, white, rc);
    renderer.draw_lines({c, c + f * 1.6}, GL_LINES, red, rc);
    // Camera footprint: the view's corner rays down to the ground below the vehicle.
    Vec3d cf, cu;
    camera_axes(p, player.mount_pitch, cf, cu);
    const Vec3d cr = normalize(cross(cf, cu));
    const double ty = std::tan(rc.fov_y / 2.0), tx = ty * rc.aspect;
    const Vec3d up = ellipsoid_up(p.geodetic.lat, p.geodetic.lon);
    const double above = p.geodetic.height - ground_below;
    std::vector<Vec3d> rays, frame;
    for (const auto& k : {std::array<double, 2>{-1, -1}, {1, -1}, {1, 1}, {-1, 1}}) {
        const Vec3d d = normalize(cf + cr * (k[0] * tx) + cu * (k[1] * ty));
        const double down = -dot(d, up);
        const double reach = down > 0.05 && above > 0 ? std::min(above / down, above * 20.0) : size * 3.0;
        rays.push_back(c);
        rays.push_back(c + d * reach);
        frame.push_back(c + d * reach);
    }
    frame.push_back(frame.front());
    renderer.draw_lines(rays, GL_LINES, cyan, rc, 0.15f);
    renderer.draw_lines(frame, GL_LINE_STRIP, cyan, rc, 0.15f);
}

}  // namespace

int main(int argc, char** argv) {
    const std::optional<Args> parsed = parse(argc, argv);
    if (!parsed) {
        usage();
        return 2;
    }
    const Args& args = *parsed;

    std::unique_ptr<Terrain> terrain;
    try {
        TerrainOptions o;
        o.selection.max_screen_error = args.max_screen_error;
        terrain = std::make_unique<Terrain>(args.cdb, o);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ogccdb-viewer: %s\n", e.what());
        return 1;
    }
    if (!terrain->index().extent()) {
        std::fprintf(stderr, "ogccdb-viewer: no Elevation or Imagery tiles in %s\n", args.cdb.string().c_str());
        return 1;
    }

    if (!glfwInit()) {
        std::fprintf(stderr, "ogccdb-viewer: cannot initialise GLFW\n");
        return 1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);
    glfwWindowHint(GLFW_SAMPLES, args.screenshot ? 0 : 4);
    GLFWwindow* window = glfwCreateWindow(args.width, args.height, "ogccdb-viewer", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "ogccdb-viewer: cannot create an OpenGL 3.3 core window\n");
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(args.screenshot ? 0 : 1);
    if (const char* missing = gl::load(reinterpret_cast<gl::LoadFn>(glfwGetProcAddress))) {
        std::fprintf(stderr, "ogccdb-viewer: OpenGL function %s is missing\n", missing);
        return 1;
    }
    std::printf("OpenGL %s, %s\n", reinterpret_cast<const char*>(gl::GetString(GL_VERSION)),
                reinterpret_cast<const char*>(gl::GetString(GL_RENDERER)));
    if (!args.screenshot) gl::Enable(GL_MULTISAMPLE);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::StyleColorsDark();
    float xscale = 1.0f, yscale = 1.0f;
    glfwGetWindowContentScale(window, &xscale, &yscale);
    if (xscale > 1.0f) {
        ImGui::GetStyle().ScaleAllSizes(xscale);
        ImGui::GetIO().FontGlobalScale = xscale;
    }
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    glfwSetScrollCallback(window, scroll_callback);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    int result = 0;
    {
        Renderer renderer;
        RenderSettings settings;
        settings.exaggeration = args.exaggeration;
        settings.lod_colors = args.lod_colors;
        Sea sea;
        settings.sea = args.sea != "off";
        if (args.sea == "egm2008") sea.reference = Sea::Egm2008;
        if (args.sea == "egm96") sea.reference = Sea::Egm96;
        if (args.sea == "ellipsoid") sea.reference = Sea::Ellipsoid;
        sea.offset = static_cast<float>(args.sea_offset);
        if (args.wave_height) {
            settings.wave_height = static_cast<float>(std::max(*args.wave_height, 0.0));
            settings.waves = *args.wave_height > 0.0;
        }
        if (args.wind_from) settings.wind_from = static_cast<float>(*args.wind_from);
        if (args.screenshot) renderer.set_clock(0.0);  // reproducible waves
        int saved_frames = 0;
        if (settings.sea) {
            Geoid* g = sea.geoid();
            std::printf("sea: %s%s\n", sea.name(g),
                        sea.reference != Sea::Ellipsoid && !g ? " (no geoid grid installed)" : "");
        }
        SelectionOptions selection = terrain->options().selection;

        Views views;
        GlobeCamera& cam = views.orbit;
        fit_store(cam, terrain->index());
        if (args.lat) cam.lat = *args.lat;
        if (args.lon) cam.lon = *args.lon;
        if (args.distance) cam.distance = *args.distance;
        if (args.heading) cam.heading = *args.heading;
        if (args.pitch) cam.pitch = *args.pitch;
        if (args.fov) set_fov(views, *args.fov);
        cam.clamp();
        if (args.fly) {
            orbit_to_fly(views);
            if (args.eye_height) views.fly.position = geodetic_to_ecef({cam.lat, cam.lon, *args.eye_height});
            if (args.pitch) views.fly.pitch = *args.pitch;
            views.fly.clamp();
            views.flying = true;
        }
        FlyCamera& fly = views.fly;

        bool show_ui = !args.hide_ui;
        bool h_was_down = false, v_was_down = false, tab_was_down = false, g_was_down = false,
             esc_was_down = false;
        bool was_flying = false;
        if (glfwRawMouseMotionSupported()) glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);
        // Mouse look locks and hides the cursor; the panel gets no mouse meanwhile.
        auto set_capture = [&](bool on) {
            if (args.screenshot) on = false;
            views.mouse_captured = on;
            glfwSetInputMode(window, GLFW_CURSOR, on ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
            ImGuiIO& gio = ImGui::GetIO();
            if (on) {
                gio.ConfigFlags |= ImGuiConfigFlags_NoMouse | ImGuiConfigFlags_NoMouseCursorChange;
            } else {
                gio.ConfigFlags &= ~(ImGuiConfigFlags_NoMouse | ImGuiConfigFlags_NoMouseCursorChange);
            }
            glfwGetCursorPos(window, &g_input.last_x, &g_input.last_y);
        };
        double& ground = views.ground;
        bool ground_known = false;

        FlightPlayer player;
        FileBrowser browser;
        // A newly opened flight: the orbit camera looks at where it starts.
        auto on_loaded = [&]() {
            const Geodetic g = player.start();
            cam.lat = g.lat;
            cam.lon = g.lon;
            cam.height = g.height;
            cam.distance = 1500.0;
            cam.pitch = -45.0;
            cam.clamp();
            ground_known = false;
            std::printf("flight: %s, %.1f s, position from %s\n", player.path().string().c_str(),
                        player.track().duration(), player.track().info().position_source.c_str());
        };
        if (args.bag && std::filesystem::is_directory(*args.bag) && !ogccdb::flight::is_bag(*args.bag)) {
            browser.open(*args.bag);  // a folder of bags: let the user pick one
        } else if (args.bag) {
            if (player.load(*args.bag)) {
                on_loaded();
                // Explicit view options win over the default view of the flight.
                if (args.lat) cam.lat = *args.lat;
                if (args.lon) cam.lon = *args.lon;
                if (args.distance) cam.distance = *args.distance;
                if (args.heading) cam.heading = *args.heading;
                if (args.pitch) cam.pitch = *args.pitch;
                cam.clamp();
                if (args.bag_time) player.seek(player.track().start() + *args.bag_time);
                if (args.play_speed) player.speed = static_cast<float>(*args.play_speed);
                if (args.mount_pitch) player.mount_pitch = static_cast<float>(std::clamp(*args.mount_pitch, -90.0, 0.0));
                player.playing = args.play;
                player.drone_camera = !args.no_drone_camera;
            } else {
                std::fprintf(stderr, "ogccdb-viewer: %s\n", player.error().c_str());
            }
        }
        bool p_was_down = false, b_was_down = false, left_was_down = false, right_was_down = false;
        int stable_frames = 0;
        int frame = 0;
        auto last_time = std::chrono::steady_clock::now();
        double fps = 0.0;

        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration<double>(now - last_time).count();
            last_time = now;
            if (dt > 0) fps = fps * 0.9 + 0.1 / dt;
            // Flight clock: real time, or 1/30 s per saved frame for --frames.
            player.update(args.screenshot ? (saved_frames > 0 ? 1.0 / 30.0 : 0.0) : dt);
            const bool drone_view = player.loaded() && player.drone_camera;
            if (drone_view && views.mouse_captured) set_capture(false);

            int fb_w = 0, fb_h = 0, win_w = 0, win_h = 0;
            glfwGetFramebufferSize(window, &fb_w, &fb_h);
            glfwGetWindowSize(window, &win_w, &win_h);
            if (fb_w == 0 || fb_h == 0) {
                glfwWaitEvents();
                continue;
            }

            // Input.
            const ImGuiIO& io = ImGui::GetIO();
            double mx = 0, my = 0;
            glfwGetCursorPos(window, &mx, &my);
            const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
                               glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
            if (views.flying != was_flying) {  // switched by V or the panel
                set_capture(views.flying);
                was_flying = views.flying;
            }
            const double fly_above = ecef_to_geodetic(fly.position).height - views.eye_ground;
            if (drone_view) {
                g_input.dragging_left = g_input.dragging_right = false;
            } else if (views.mouse_captured) {
                const double dx = mx - g_input.last_x, dy = my - g_input.last_y;
                // Holding the right button zooms 4x; look slows down with it.
                const double sens = views.sensitivity * (right ? 0.25 : 1.0);
                fly.look(dx * sens, (views.invert_y ? dy : -dy) * sens);
            } else if (!io.WantCaptureMouse) {
                const double dx = mx - g_input.last_x, dy = my - g_input.last_y;
                if (views.flying) {
                    if (left && !g_input.dragging_left) set_capture(true);  // click the view: mouse look
                } else {
                    if (left && g_input.dragging_left) cam.pan_pixels(dx, dy, win_h);
                    if (right && g_input.dragging_right) cam.rotate(dx * 0.25, -dy * 0.25);
                }
                g_input.dragging_left = left;
                g_input.dragging_right = right;
            } else {
                g_input.dragging_left = g_input.dragging_right = false;
            }
            if (views.mouse_captured) glfwGetCursorPos(window, &mx, &my);  // set_capture may have reset it
            g_input.last_x = mx;
            g_input.last_y = my;
            if (g_input.scroll != 0.0) {
                if (views.flying) {
                    fly.speed_factor *= std::pow(1.25, g_input.scroll);
                    fly.clamp();
                } else {
                    cam.zoom(g_input.scroll);
                }
                g_input.scroll = 0.0;
            }
            if (!io.WantCaptureKeyboard) {
                auto down = [&](int key) { return glfwGetKey(window, key) == GLFW_PRESS; };
                const bool pk = down(GLFW_KEY_P), bk = down(GLFW_KEY_B), lk = down(GLFW_KEY_LEFT), rk = down(GLFW_KEY_RIGHT);
                if (player.loaded()) {
                    if (pk && !p_was_down) {
                        if (!player.playing && player.time >= player.track().end()) player.seek(player.track().start());
                        player.playing = !player.playing;
                    }
                    if (bk && !b_was_down) {
                        if (player.drone_camera) {  // leave the drone camera where the drone is
                            const FlightPose p = player.pose();
                            cam.lat = p.geodetic.lat;
                            cam.lon = p.geodetic.lon;
                            cam.height = p.geodetic.height;
                            cam.distance = 300.0;
                            cam.pitch = -35.0;
                            cam.heading = p.yaw;
                            cam.clamp();
                            views.flying = false;
                        }
                        player.drone_camera = !player.drone_camera;
                    }
                    if (drone_view && lk && !left_was_down) player.seek(player.time - 5.0);
                    if (drone_view && rk && !right_was_down) player.seek(player.time + 5.0);
                }
                p_was_down = pk;
                b_was_down = bk;
                left_was_down = lk;
                right_was_down = rk;
                if (drone_view) {
                    // The flight drives the camera.
                } else if (views.flying) {
                    double speed = fly.speed(fly_above);
                    if (down(GLFW_KEY_LEFT_SHIFT) || down(GLFW_KEY_RIGHT_SHIFT)) speed *= 5.0;
                    if (down(GLFW_KEY_LEFT_CONTROL) || down(GLFW_KEY_RIGHT_CONTROL)) speed *= 0.2;
                    const double ahead = (down(GLFW_KEY_W) ? 1.0 : 0.0) - (down(GLFW_KEY_S) ? 1.0 : 0.0);
                    const double side = (down(GLFW_KEY_D) ? 1.0 : 0.0) - (down(GLFW_KEY_A) ? 1.0 : 0.0);
                    const double rise = (down(GLFW_KEY_E) || down(GLFW_KEY_SPACE) ? 1.0 : 0.0) -
                                        (down(GLFW_KEY_Q) || down(GLFW_KEY_C) ? 1.0 : 0.0);
                    if (views.walk) {
                        if (ahead != 0.0 || side != 0.0) fly.move(ahead, side, 0.0, speed, dt, true);
                    } else if (ahead != 0.0 || side != 0.0 || rise != 0.0) {
                        fly.move(ahead, side, rise, speed, dt);
                    }
                    const double turn = 60.0 * dt;
                    if (down(GLFW_KEY_LEFT)) fly.look(-turn, 0);
                    if (down(GLFW_KEY_RIGHT)) fly.look(turn, 0);
                    if (down(GLFW_KEY_UP)) fly.look(0, turn);
                    if (down(GLFW_KEY_DOWN)) fly.look(0, -turn);
                } else {
                    const double step = 600.0 * dt;
                    if (down(GLFW_KEY_W)) cam.pan_pixels(0, step, win_h);
                    if (down(GLFW_KEY_S)) cam.pan_pixels(0, -step, win_h);
                    if (down(GLFW_KEY_A)) cam.pan_pixels(step, 0, win_h);
                    if (down(GLFW_KEY_D)) cam.pan_pixels(-step, 0, win_h);
                    if (down(GLFW_KEY_Q)) cam.rotate(-60.0 * dt, 0);
                    if (down(GLFW_KEY_E)) cam.rotate(60.0 * dt, 0);
                    if (down(GLFW_KEY_R)) cam.rotate(0, 40.0 * dt);
                    if (down(GLFW_KEY_F)) cam.rotate(0, -40.0 * dt);
                    if (down(GLFW_KEY_EQUAL) || down(GLFW_KEY_KP_ADD)) cam.zoom(4.0 * dt);
                    if (down(GLFW_KEY_MINUS) || down(GLFW_KEY_KP_SUBTRACT)) cam.zoom(-4.0 * dt);
                }
                const bool vk = down(GLFW_KEY_V);
                if (vk && !v_was_down) {
                    if (views.flying) fly_to_orbit(views); else orbit_to_fly(views);
                    views.flying = !views.flying;
                }
                v_was_down = vk;
                const bool h = down(GLFW_KEY_H);
                if (h && !h_was_down) show_ui = !show_ui;
                h_was_down = h;
                if (down(GLFW_KEY_LEFT_BRACKET)) set_fov(views, fly.fov_y_deg - 30.0 * dt);
                if (down(GLFW_KEY_RIGHT_BRACKET)) set_fov(views, fly.fov_y_deg + 30.0 * dt);
                const bool tab = down(GLFW_KEY_TAB), gk = down(GLFW_KEY_G), esc = down(GLFW_KEY_ESCAPE);
                if (views.flying && tab && !tab_was_down) set_capture(!views.mouse_captured);
                if (views.flying && gk && !g_was_down) views.walk = !views.walk;
                if (esc && !esc_was_down) {
                    if (views.mouse_captured) {
                        set_capture(false);
                    } else if (!views.flying) {
                        glfwSetWindowShouldClose(window, GLFW_TRUE);
                    }
                }
                tab_was_down = tab;
                g_was_down = gk;
                esc_was_down = esc;
            }

            // Terrain.
            terrain->set_selection_options(selection);
            std::optional<FlightPose> drone;
            if (player.loaded()) {
                drone = player.pose();
                if (player.follow && !drone_view && !views.flying) {
                    cam.lat = drone->geodetic.lat;
                    cam.lon = drone->geodetic.lon;
                }
            }
            Vec3d drone_fwd, drone_up;
            if (drone) camera_axes(*drone, player.mount_pitch, drone_fwd, drone_up);
            const Camera rc = drone_view ? make_camera(drone->position, drone_fwd, drone_up, fly.fov_y_deg, fb_w, fb_h,
                                                       views.eye_ground, 5.0e4)
                              : views.flying
                                  ? make_camera(fly.position, fly.forward(), fly.up(),
                                                fly.fov_y_deg * (views.mouse_captured && right ? 0.25 : 1.0), fb_w, fb_h,
                                                views.eye_ground, 5.0e4)
                                  : cam.to_runtime(fb_w, fb_h, ground_known ? ground : cam.height);
            const auto& draw = terrain->update(rc);
            const int res = terrain->options().mesh.resolution;
            Geoid* const sea_geoid = sea.geoid();
            const float sea_offset = sea.offset;
            settings.sea_height = [sea_geoid, sea_offset](double lat, double lon) {
                return Sea::height(sea_geoid, sea_offset, lat, lon);
            };
            // Ground: the terrain, or the sea where it is higher or there is no terrain.
            auto surface = [&](double lat, double lon) -> std::optional<double> {
                std::optional<double> h = ground_height(draw, lat, lon, res);
                if (settings.sea) h = std::max(h.value_or(-1e9), settings.sea_height(lat, lon));
                if (h) *h *= settings.exaggeration;
                return h;
            };
            if (!views.flying) {
                if (player.loaded() && player.follow && !drone_view) {
                    cam.height = drone->geodetic.height;  // orbit the vehicle itself
                    ground_known = true;
                } else if (const auto g = surface(cam.lat, cam.lon)) {
                    ground = *g;
                    cam.height = ground_known ? cam.height + (ground - cam.height) * std::min(1.0, 10.0 * dt) : ground;
                    ground_known = true;
                }
            }
            std::optional<double> drone_agl;
            if (drone) {
                // Put a flight that starts on the ground on the terrain, once the
                // tiles there are loaded (a finer tile improves it).
                const TerrainStats ts = terrain->stats();
                if (player.altitude == FlightPlayer::Altitude::StartOnGround && ts.queued == 0 && ts.loading == 0) {
                    const Geodetic s0 = player.start();
                    int lod = 0;
                    if (const auto g = ground_height(draw, s0.lat, s0.lon, res, &lod)) player.offer_ground(*g, lod);
                }
                if (const auto g = surface(drone->geodetic.lat, drone->geodetic.lon)) drone_agl = drone->geodetic.height - *g;
            }
            const Geodetic eye_geo = ecef_to_geodetic(rc.position);
            if (const auto g = surface(eye_geo.lat, eye_geo.lon)) {
                views.eye_ground = *g;
                if (views.flying && views.walk) {
                    // Follow the ground quickly but smoothly (tiles refine under the eye).
                    const double target = views.eye_ground + views.walk_height;
                    fly.set_height(eye_geo.height + (target - eye_geo.height) * std::min(1.0, 15.0 * dt));
                }
            }

            renderer.draw(draw, rc, fb_w, fb_h, settings);
            if (drone) {
                draw_flight(renderer, player, *drone, drone_view, rc, views.eye_ground);
            }

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            ui(views, *terrain, renderer, settings, selection, sea, player, browser, drone_agl, args, fps, show_ui);
            if (const auto chosen = browser.draw()) {
                if (player.load(*chosen)) on_loaded();
            }
            if (drone_view && !(args.screenshot && args.hide_ui)) {
                ImDrawList* fg = ImGui::GetForegroundDrawList();
                char hud[256];
                const double t = player.time - player.track().start();
                std::snprintf(hud, sizeof hud,
                              "Drone camera  %d:%04.1f %s %.1fx   AGL %.0f m   hdg %03.0f   %.1f m/s   "
                              "(P play, Left/Right seek, B leave)",
                              static_cast<int>(t / 60), std::fmod(t, 60.0), player.playing ? ">" : "||", player.speed,
                              drone_agl.value_or(0.0), drone->yaw, drone->ground_speed);
                const ImVec2 ts = ImGui::CalcTextSize(hud);
                const float cx = io.DisplaySize.x * 0.5f;
                fg->AddRectFilled(ImVec2(cx - ts.x * 0.5f - 8, 8), ImVec2(cx + ts.x * 0.5f + 8, 16 + ts.y),
                                  IM_COL32(0, 0, 0, 140), 4.0f);
                fg->AddText(ImVec2(cx - ts.x * 0.5f, 12), IM_COL32(255, 255, 255, 230), hud);
            } else if (views.flying) {
                ImDrawList* fg = ImGui::GetForegroundDrawList();
                const ImVec2 c(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f);
                const char* hint = views.mouse_captured
                                       ? (views.walk ? "Walking - WASD move, Shift run, G fly, Esc/Tab release mouse"
                                                     : "Free fly - WASD move, E/Q up/down, G walk, Esc/Tab release mouse")
                                       : "Click the view or press Tab for mouse look";
                const ImVec2 ts = ImGui::CalcTextSize(hint);
                if (!(args.screenshot && args.hide_ui)) {  // clean frames for --screenshot --no-ui
                    fg->AddRectFilled(ImVec2(c.x - ts.x * 0.5f - 8, 8), ImVec2(c.x + ts.x * 0.5f + 8, 16 + ts.y),
                                      IM_COL32(0, 0, 0, 140), 4.0f);
                    fg->AddText(ImVec2(c.x - ts.x * 0.5f, 12), IM_COL32(255, 255, 255, 230), hint);
                }
                if (views.mouse_captured) {  // crosshair
                    const ImU32 col = IM_COL32(255, 255, 255, 200);
                    fg->AddLine(ImVec2(c.x - 9, c.y), ImVec2(c.x - 3, c.y), col, 1.5f);
                    fg->AddLine(ImVec2(c.x + 3, c.y), ImVec2(c.x + 9, c.y), col, 1.5f);
                    fg->AddLine(ImVec2(c.x, c.y - 9), ImVec2(c.x, c.y - 3), col, 1.5f);
                    fg->AddLine(ImVec2(c.x, c.y + 3), ImVec2(c.x, c.y + 9), col, 1.5f);
                }
            }
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            ++frame;
            if (args.screenshot) {
                // Done once nothing is queued or loading and the view has stopped changing.
                const TerrainStats s = terrain->stats();
                stable_frames = (s.queued == 0 && s.loading == 0 && renderer.stats().uploads == 0) ? stable_frames + 1 : 0;
                if (stable_frames >= 5 || frame > 3000 || saved_frames > 0) {
                    gl::Finish();
                    std::string file = *args.screenshot;
                    if (args.frames > 1) {
                        char num[16];
                        std::snprintf(num, sizeof num, "_%04d", saved_frames);
                        const std::filesystem::path p(file);
                        file = (p.parent_path() / (p.stem().string() + num + p.extension().string())).string();
                    }
                    if (!save_screenshot(file, fb_w, fb_h)) {
                        std::fprintf(stderr, "ogccdb-viewer: cannot write %s\n", file.c_str());
                        result = 1;
                        break;
                    }
                    if (++saved_frames >= args.frames) {
                        std::printf("saved %s after %d frames (%zu tiles)\n", file.c_str(), frame, s.drawn);
                        break;
                    }
                    renderer.set_clock(saved_frames / 30.0);
                }
                if (s.queued || s.loading) terrain->wait_idle();
            }
            glfwSwapBuffers(window);
        }
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return result;
}
