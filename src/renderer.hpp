#pragma once
// Draws the runtime's terrain tiles and a sea-level reference surface with OpenGL 3.3 core.
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

#include "gl.hpp"
#include "ogccdb/runtime/terrain.hpp"

struct RenderSettings {
    bool imagery = true;
    bool lighting = true;
    bool wireframe = false;
    bool lod_colors = false;
    bool haze = true;
    /// Reference surface under the terrain: a sea whose height above the
    /// ellipsoid at (lat, lon) is `sea_height` (the geoid for mean sea level;
    /// 0 m if unset). Off: a dark globe drawn as background only.
    bool sea = true;
    bool graticule = true;
    std::function<double(double lat, double lon)> sea_height;
    /// Moving waves on the sea: wave height (crest to trough, metres) and the
    /// direction the wind blows from (degrees clockwise from north).
    bool waves = true;
    float wave_height = 1.5f;
    float wind_from = 240.0f;
    float exaggeration = 1.0f;
};

struct RenderStats {
    std::size_t gpu_tiles = 0;
    std::size_t uploads = 0;
    std::size_t gpu_bytes = 0;
    std::size_t triangles = 0;
};

class Renderer {
public:
    Renderer();
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void draw(const std::vector<ogccdb::runtime::DrawTile>& tiles, const ogccdb::runtime::Camera& camera,
              int width, int height, const RenderSettings& settings);

    const RenderStats& stats() const { return stats_; }
    /// Fix the wave clock at `seconds` (for reproducible screenshots and frame
    /// sequences); by default it runs in real time.
    void set_clock(double seconds) { fixed_time_ = seconds; }

private:
    struct GpuTile {
        GLuint vao = 0, vbo = 0, ibo = 0, texture = 0;
        GLsizei index_count = 0;
        std::size_t bytes = 0;
        std::uint64_t last_used = 0;
        const ogccdb::runtime::TerrainTile* source = nullptr;
    };

    GpuTile upload(const ogccdb::runtime::TerrainTile& t);
    void release(GpuTile& g);
    void draw_surface(const ogccdb::runtime::Camera& camera, const RenderSettings& settings, const float sky[3],
                      double haze, ogccdb::runtime::Vec3d light);

    void set_wave_anchor(double lat, double lon);
    std::vector<float> wave_uniforms(const RenderSettings& settings) const;

    GLuint terrain_program_ = 0;
    std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
    std::optional<double> fixed_time_;
    bool wave_anchor_set_ = false;
    ogccdb::runtime::Vec3d wave_anchor_, wave_east_, wave_north_;
    GLuint surface_program_ = 0;
    GLuint surface_vao_ = 0, surface_vbo_ = 0, surface_ibo_ = 0;
    GLsizei surface_index_count_ = 0;
    std::vector<float> surface_vertices_;
    float max_anisotropy_ = 0.0f;
    std::uint64_t frame_ = 0;
    std::unordered_map<ogccdb::runtime::TileAddress, GpuTile, ogccdb::runtime::TileAddressHash> tiles_;
    RenderStats stats_;
};
