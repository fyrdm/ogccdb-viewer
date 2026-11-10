#pragma once
// Draws the runtime's terrain tiles and a background globe with OpenGL 3.3 core.
#include <cstdint>
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
    bool globe = true;
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
    void draw_globe(const ogccdb::runtime::Camera& camera, const RenderSettings& settings);

    GLuint terrain_program_ = 0;
    GLuint globe_program_ = 0;
    GLuint globe_vao_ = 0, globe_vbo_ = 0, globe_ibo_ = 0;
    GLsizei globe_index_count_ = 0;
    float max_anisotropy_ = 0.0f;
    std::uint64_t frame_ = 0;
    std::unordered_map<ogccdb::runtime::TileAddress, GpuTile, ogccdb::runtime::TileAddressHash> tiles_;
    RenderStats stats_;
};
