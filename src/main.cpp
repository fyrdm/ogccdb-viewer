// ogccdb-viewer: a 3D viewer for OGC CDB 1.3 Elevation and Imagery.
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "gl.hpp"
#include "globe_camera.hpp"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "ogccdb/cdb13/lod.hpp"
#include "ogccdb/runtime/geodesy.hpp"
#include "ogccdb/runtime/terrain.hpp"
#include "renderer.hpp"
#include "screenshot.hpp"

using namespace ogccdb;
using namespace ogccdb::runtime;

namespace {

struct Args {
    std::filesystem::path cdb;
    std::optional<double> lat, lon, distance, heading, pitch;
    int width = 1600, height = 1000;
    std::optional<std::string> screenshot;
    float exaggeration = 1.0f;
    double max_screen_error = 1.5;
    bool lod_colors = false;
    bool hide_ui = false;
    bool skirts = true;
    int mesh_resolution = 32;
};

void usage() {
    std::puts(
        "usage: ogccdb-viewer <cdb-root> [options]\n"
        "  --lat <deg> --lon <deg>     view target (default: centre of the store)\n"
        "  --distance <m>              eye distance from the target\n"
        "  --heading <deg>             0 = looking north\n"
        "  --pitch <deg>               -90 = straight down (default -60)\n"
        "  --exaggeration <k>          vertical exaggeration (default 1)\n"
        "  --screen-error <px>         LOD refinement threshold (default 1.5)\n"
        "  --lod-colors                tint tiles by LOD\n"
        "  --size <w>x<h>              window size (default 1600x1000)\n"
        "  --screenshot <file.png>     render until all tiles are loaded, save, exit\n"
        "  --no-ui                     hide the panels\n"
        "  --no-skirts                 draw tiles without skirts (shows cracks)\n"
        "  --mesh-resolution <n>       grid cells per tile side (default 32)\n"
        "controls: left drag pan, right drag rotate/tilt, wheel zoom,\n"
        "          WASD pan, Q/E rotate, R/F tilt, +/- zoom, H toggles the panels");
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
        else if (s == "--exaggeration") { const char* v = next(); if (!v) return std::nullopt; a.exaggeration = static_cast<float>(std::atof(v)); }
        else if (s == "--screen-error") { const char* v = next(); if (!v) return std::nullopt; a.max_screen_error = std::atof(v); }
        else if (s == "--lod-colors") a.lod_colors = true;
        else if (s == "--no-ui") a.hide_ui = true;
        else if (s == "--no-skirts") a.skirts = false;
        else if (s == "--mesh-resolution") { const char* v = next(); if (!v) return std::nullopt; a.mesh_resolution = std::atoi(v); }
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

/// Terrain height at (lat, lon) from the finest drawn tile over it, by bilinear
/// interpolation of its mesh grid; nullopt if no drawn tile covers it.
std::optional<double> ground_height(const std::vector<DrawTile>& tiles, double lat, double lon, int resolution) {
    const DrawTile* best = nullptr;
    for (const DrawTile& d : tiles) {
        const GeoExtent e = cdb13::extent(d.tile);
        if (lat >= e.south && lat <= e.north && lon >= e.west && lon <= e.east && (!best || d.tile.lod > best->tile.lod)) {
            best = &d;
        }
    }
    if (!best) return std::nullopt;
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

void ui(GlobeCamera& cam, Terrain& terrain, Renderer& renderer, RenderSettings& settings, SelectionOptions& sel,
        const Args& args, double fps, double ground, bool& show_ui) {
    if (!show_ui) return;
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(360, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("ogccdb-viewer", &show_ui);
    ImGui::TextWrapped("%s", args.cdb.string().c_str());
    const TileIndex& idx = terrain.index();
    ImGui::Text("Elevation tiles %zu, Imagery tiles %zu", idx.elevation_count(), idx.imagery_count());
    if (const auto e = idx.extent()) ImGui::Text("Extent %.0f..%.0f N, %.0f..%.0f E", e->south, e->north, e->west, e->east);

    if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) {
        const Geodetic eye = ecef_to_geodetic(cam.eye());
        ImGui::Text("Target %.5f, %.5f  ground %.0f m", cam.lat, cam.lon, ground);
        ImGui::Text("Eye height %.0f m, distance %.0f m", eye.height, cam.distance);
        float heading = static_cast<float>(cam.heading), pitch = static_cast<float>(cam.pitch);
        if (ImGui::SliderFloat("Heading", &heading, 0.0f, 360.0f, "%.0f deg")) cam.heading = heading;
        if (ImGui::SliderFloat("Pitch", &pitch, -89.9f, -1.0f, "%.0f deg")) cam.pitch = pitch;
        if (ImGui::Button("Fit store")) fit_store(cam, idx);
        ImGui::SameLine();
        if (ImGui::Button("Top view")) cam.pitch = -89.9;
        ImGui::SameLine();
        if (ImGui::Button("North up")) cam.heading = 0.0;
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
        ImGui::SameLine();
        ImGui::Checkbox("Globe", &settings.globe);
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
            "Left drag: pan. Right drag: rotate and tilt. Wheel: zoom.\n"
            "WASD: pan, Q/E: rotate, R/F: tilt, +/-: zoom, H: hide panels.");
    }
    ImGui::End();
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
        o.mesh.skirts = args.skirts;
        o.mesh.resolution = args.mesh_resolution;
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
        SelectionOptions selection = terrain->options().selection;

        GlobeCamera cam;
        fit_store(cam, terrain->index());
        if (args.lat) cam.lat = *args.lat;
        if (args.lon) cam.lon = *args.lon;
        if (args.distance) cam.distance = *args.distance;
        if (args.heading) cam.heading = *args.heading;
        if (args.pitch) cam.pitch = *args.pitch;
        cam.clamp();

        bool show_ui = !args.hide_ui;
        bool h_was_down = false;
        double ground = 0.0;
        bool ground_known = false;
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
            if (!io.WantCaptureMouse) {
                if (left && g_input.dragging_left) cam.pan_pixels(mx - g_input.last_x, my - g_input.last_y, win_h);
                if (right && g_input.dragging_right) cam.rotate((mx - g_input.last_x) * 0.25, -(my - g_input.last_y) * 0.25);
                g_input.dragging_left = left;
                g_input.dragging_right = right;
            } else {
                g_input.dragging_left = g_input.dragging_right = false;
            }
            g_input.last_x = mx;
            g_input.last_y = my;
            if (g_input.scroll != 0.0) {
                cam.zoom(g_input.scroll);
                g_input.scroll = 0.0;
            }
            if (!io.WantCaptureKeyboard) {
                auto down = [&](int key) { return glfwGetKey(window, key) == GLFW_PRESS; };
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
                const bool h = down(GLFW_KEY_H);
                if (h && !h_was_down) show_ui = !show_ui;
                h_was_down = h;
                if (down(GLFW_KEY_ESCAPE)) glfwSetWindowShouldClose(window, GLFW_TRUE);
            }

            // Terrain.
            terrain->set_selection_options(selection);
            const Camera rc = cam.to_runtime(fb_w, fb_h, ground_known ? ground : cam.height);
            const auto& draw = terrain->update(rc);
            if (const auto g = ground_height(draw, cam.lat, cam.lon, terrain->options().mesh.resolution)) {
                ground = *g * settings.exaggeration;
                cam.height = ground_known ? cam.height + (ground - cam.height) * std::min(1.0, 10.0 * dt) : ground;
                ground_known = true;
            }

            renderer.draw(draw, rc, fb_w, fb_h, settings);

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
            ui(cam, *terrain, renderer, settings, selection, args, fps, ground, show_ui);
            ImGui::Render();
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            ++frame;
            if (args.screenshot) {
                // Done once nothing is queued or loading and the view has stopped changing.
                const TerrainStats s = terrain->stats();
                stable_frames = (s.queued == 0 && s.loading == 0 && renderer.stats().uploads == 0) ? stable_frames + 1 : 0;
                if (stable_frames >= 5 || frame > 3000) {
                    gl::Finish();
                    if (!save_screenshot(*args.screenshot, fb_w, fb_h)) {
                        std::fprintf(stderr, "ogccdb-viewer: cannot write %s\n", args.screenshot->c_str());
                        result = 1;
                    } else {
                        std::printf("saved %s after %d frames (%zu tiles)\n", args.screenshot->c_str(), frame, s.drawn);
                    }
                    break;
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
