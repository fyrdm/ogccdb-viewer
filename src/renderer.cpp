#include "renderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "ogccdb/runtime/geodesy.hpp"
#include "shader.hpp"

using namespace ogccdb::runtime;

namespace {

constexpr GLenum kTextureMaxAnisotropy = 0x84FE;      // GL 4.6 / EXT_texture_filter_anisotropic
constexpr GLenum kMaxTextureMaxAnisotropy = 0x84FF;

const char* kTerrainVs = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_uv;
layout(location = 3) in float a_height;
uniform mat4 u_mvp;
uniform mat4 u_model_view;
uniform vec3 u_up;
uniform float u_exaggeration;
out vec3 v_normal;
out vec2 v_uv;
out float v_height;
out float v_distance;
void main() {
    vec3 p = a_position + u_up * (a_height * (u_exaggeration - 1.0));
    gl_Position = u_mvp * vec4(p, 1.0);
    v_normal = a_normal;
    v_uv = a_uv;
    v_height = a_height;
    v_distance = length((u_model_view * vec4(p, 1.0)).xyz);
}
)";

const char* kTerrainFs = R"(#version 330 core
in vec3 v_normal;
in vec2 v_uv;
in float v_height;
in float v_distance;
uniform sampler2D u_texture;
uniform int u_has_texture;
uniform int u_lighting;
uniform vec3 u_light;
uniform vec4 u_tint;
uniform vec3 u_haze_color;
uniform float u_haze_distance;
out vec4 frag;
vec3 height_ramp(float h) {
    float t = clamp(h / 3000.0, 0.0, 1.0);
    vec3 low = vec3(0.20, 0.42, 0.22), mid = vec3(0.62, 0.55, 0.38), high = vec3(0.95, 0.95, 0.95);
    return t < 0.5 ? mix(low, mid, t * 2.0) : mix(mid, high, t * 2.0 - 1.0);
}
void main() {
    vec3 c = u_has_texture != 0 ? texture(u_texture, v_uv).rgb : height_ramp(v_height);
    if (u_lighting != 0) {
        float d = max(dot(normalize(v_normal), u_light), 0.0);
        c *= 0.55 + 0.6 * d;
    }
    c = mix(c, u_tint.rgb, u_tint.a);
    if (u_haze_distance > 0.0) {
        float f = 1.0 - exp(-v_distance / u_haze_distance);
        c = mix(c, u_haze_color, clamp(f, 0.0, 0.85));
    }
    frag = vec4(c, 1.0);
}
)";

const char* kGlobeVs = R"(#version 330 core
layout(location = 0) in vec3 a_position;
layout(location = 1) in vec2 a_latlon;
uniform mat4 u_mvp;
out vec2 v_latlon;
void main() {
    gl_Position = u_mvp * vec4(a_position, 1.0);
    v_latlon = a_latlon;
}
)";

const char* kGlobeFs = R"(#version 330 core
in vec2 v_latlon;
uniform vec3 u_color;
uniform vec3 u_line_color;
out vec4 frag;
float grid(vec2 coord, float spacing) {
    vec2 g = abs(fract(coord / spacing - 0.5) - 0.5) / fwidth(coord / spacing);
    return 1.0 - min(min(g.x, g.y), 1.0);
}
void main() {
    // 1-degree lines fade out where they would crowd the screen.
    float fine = 1.0 - smoothstep(0.05, 0.15, max(fwidth(v_latlon.x), fwidth(v_latlon.y)));
    float l = max(grid(v_latlon, 10.0), 0.5 * fine * grid(v_latlon, 1.0));
    frag = vec4(mix(u_color, u_line_color, l), 1.0);
}
)";

std::array<float, 16> to_float(const Mat4d& m) {
    std::array<float, 16> f{};
    for (std::size_t i = 0; i < 16; ++i) f[i] = static_cast<float>(m.m[i]);
    return f;
}

Mat4d translation(Vec3d t) {
    Mat4d m;
    m(0, 3) = t.x;
    m(1, 3) = t.y;
    m(2, 3) = t.z;
    return m;
}

std::array<float, 3> lod_color(int lod) {
    static const std::array<std::array<float, 3>, 8> palette = {{{0.90f, 0.30f, 0.25f},
                                                                  {0.95f, 0.65f, 0.20f},
                                                                  {0.95f, 0.90f, 0.25f},
                                                                  {0.45f, 0.80f, 0.30f},
                                                                  {0.25f, 0.75f, 0.75f},
                                                                  {0.30f, 0.50f, 0.95f},
                                                                  {0.60f, 0.40f, 0.90f},
                                                                  {0.90f, 0.40f, 0.75f}}};
    return palette[static_cast<std::size_t>((lod + 10) % 8)];
}

}  // namespace

Renderer::Renderer() {
    terrain_program_ = make_program(kTerrainVs, kTerrainFs);
    globe_program_ = make_program(kGlobeVs, kGlobeFs);

    if (gl::has_extension("GL_EXT_texture_filter_anisotropic") || gl::has_extension("GL_ARB_texture_filter_anisotropic")) {
        gl::GetFloatv(kMaxTextureMaxAnisotropy, &max_anisotropy_);
    }

    // Background globe: the WGS-84 ellipsoid 50 m down, so terrain at sea level stays on top.
    const int stacks = 90, slices = 180;
    std::vector<float> v;
    std::vector<std::uint32_t> idx;
    for (int i = 0; i <= stacks; ++i) {
        const double lat = -90.0 + 180.0 * i / stacks;
        for (int j = 0; j <= slices; ++j) {
            const double lon = -180.0 + 360.0 * j / slices;
            const Vec3d p = geodetic_to_ecef({lat, lon, -50.0});
            v.insert(v.end(), {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                               static_cast<float>(lat), static_cast<float>(lon)});
        }
    }
    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            const std::uint32_t a = static_cast<std::uint32_t>(i * (slices + 1) + j);
            const std::uint32_t b = a + 1, c = a + static_cast<std::uint32_t>(slices + 1), d = c + 1;
            idx.insert(idx.end(), {a, b, c, b, d, c});
        }
    }
    globe_index_count_ = static_cast<GLsizei>(idx.size());
    gl::GenVertexArrays(1, &globe_vao_);
    gl::BindVertexArray(globe_vao_);
    gl::GenBuffers(1, &globe_vbo_);
    gl::BindBuffer(GL_ARRAY_BUFFER, globe_vbo_);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(), GL_STATIC_DRAW);
    gl::GenBuffers(1, &globe_ibo_);
    gl::BindBuffer(GL_ELEMENT_ARRAY_BUFFER, globe_ibo_);
    gl::BufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(idx.size() * sizeof(std::uint32_t)), idx.data(),
                   GL_STATIC_DRAW);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), nullptr);
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
    gl::BindVertexArray(0);
}

Renderer::~Renderer() {
    for (auto& [t, g] : tiles_) release(g);
    gl::DeleteVertexArrays(1, &globe_vao_);
    gl::DeleteBuffers(1, &globe_vbo_);
    gl::DeleteBuffers(1, &globe_ibo_);
    gl::DeleteProgram(terrain_program_);
    gl::DeleteProgram(globe_program_);
}

Renderer::GpuTile Renderer::upload(const TerrainTile& t) {
    const TerrainMesh& m = t.mesh;
    const std::size_t n = m.vertex_count();
    // Interleaved: position 3, normal 3, uv 2, height 1.
    std::vector<float> v(n * 9);
    for (std::size_t i = 0; i < n; ++i) {
        float* o = &v[i * 9];
        o[0] = m.positions[i * 3];
        o[1] = m.positions[i * 3 + 1];
        o[2] = m.positions[i * 3 + 2];
        o[3] = m.normals[i * 3];
        o[4] = m.normals[i * 3 + 1];
        o[5] = m.normals[i * 3 + 2];
        o[6] = m.uvs[i * 2];
        o[7] = m.uvs[i * 2 + 1];
        o[8] = m.heights[i];
    }
    GpuTile g;
    g.source = &t;
    gl::GenVertexArrays(1, &g.vao);
    gl::BindVertexArray(g.vao);
    gl::GenBuffers(1, &g.vbo);
    gl::BindBuffer(GL_ARRAY_BUFFER, g.vbo);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(v.size() * sizeof(float)), v.data(), GL_STATIC_DRAW);
    gl::GenBuffers(1, &g.ibo);
    gl::BindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.ibo);
    gl::BufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(m.indices.size() * sizeof(std::uint32_t)),
                   m.indices.data(), GL_STATIC_DRAW);
    const GLsizei stride = 9 * sizeof(float);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    gl::EnableVertexAttribArray(2);
    gl::VertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
    gl::EnableVertexAttribArray(3);
    gl::VertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(8 * sizeof(float)));
    gl::BindVertexArray(0);
    g.index_count = static_cast<GLsizei>(m.indices.size());
    g.bytes = v.size() * sizeof(float) + m.indices.size() * sizeof(std::uint32_t);

    if (!t.image.empty()) {
        gl::GenTextures(1, &g.texture);
        gl::BindTexture(GL_TEXTURE_2D, g.texture);
        gl::PixelStorei(GL_UNPACK_ALIGNMENT, 1);
        // Row 0 of the image is the north edge, which is texture coordinate v = 0.
        gl::TexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, t.image.width, t.image.height, 0, GL_RGB, GL_UNSIGNED_BYTE,
                       t.image.rgb.data());
        gl::GenerateMipmap(GL_TEXTURE_2D);
        gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        if (max_anisotropy_ > 1.0f) gl::TexParameterf(GL_TEXTURE_2D, kTextureMaxAnisotropy, std::min(max_anisotropy_, 8.0f));
        g.bytes += t.image.rgb.size() * 4 / 3;
    }
    return g;
}

void Renderer::release(GpuTile& g) {
    if (g.texture) gl::DeleteTextures(1, &g.texture);
    gl::DeleteBuffers(1, &g.vbo);
    gl::DeleteBuffers(1, &g.ibo);
    gl::DeleteVertexArrays(1, &g.vao);
    g = GpuTile{};
}

void Renderer::draw_globe(const Camera& camera, const RenderSettings& settings) {
    if (!settings.globe) return;
    const Mat4d rot = look_at_direction({0, 0, 0}, camera.direction, camera.up);
    const auto mvp = to_float(camera.projection() * rot * translation(-camera.position));
    gl::UseProgram(globe_program_);
    gl::UniformMatrix4fv(gl::GetUniformLocation(globe_program_, "u_mvp"), 1, GL_FALSE, mvp.data());
    gl::Uniform3f(gl::GetUniformLocation(globe_program_, "u_color"), 0.10f, 0.16f, 0.24f);
    gl::Uniform3f(gl::GetUniformLocation(globe_program_, "u_line_color"), 0.30f, 0.40f, 0.52f);
    gl::DepthMask(GL_FALSE);  // background only: terrain always draws over it
    gl::BindVertexArray(globe_vao_);
    gl::DrawElements(GL_TRIANGLES, globe_index_count_, GL_UNSIGNED_INT, nullptr);
    gl::DepthMask(GL_TRUE);
}

void Renderer::draw(const std::vector<DrawTile>& tiles, const Camera& camera, int width, int height,
                    const RenderSettings& settings) {
    ++frame_;
    stats_.uploads = 0;
    stats_.triangles = 0;

    gl::Viewport(0, 0, width, height);
    const float sky[3] = {0.55f, 0.68f, 0.82f};
    const double eye_height = ecef_to_geodetic(camera.position).height;
    // Sky fades to space with height.
    const float space = static_cast<float>(std::clamp((eye_height - 5.0e4) / 1.5e6, 0.0, 1.0));
    gl::ClearColor(sky[0] * (1 - space) + 0.02f * space, sky[1] * (1 - space) + 0.03f * space,
                   sky[2] * (1 - space) + 0.06f * space, 1.0f);
    gl::ClearDepth(1.0);
    gl::Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    gl::Enable(GL_DEPTH_TEST);
    gl::DepthFunc(GL_LEQUAL);
    gl::Disable(GL_CULL_FACE);

    draw_globe(camera, settings);

    const Mat4d rot = look_at_direction({0, 0, 0}, camera.direction, camera.up);
    const Mat4d proj = camera.projection();
    // Light from the north-west, high above the view centre.
    const Geodetic g = ecef_to_geodetic(camera.position);
    const Vec3d light = normalize(ellipsoid_up(g.lat, g.lon) * 1.2 + local_north(g.lat, g.lon) * 0.5 -
                                  local_east(g.lat, g.lon) * 0.5);

    gl::UseProgram(terrain_program_);
    const GLint loc_mvp = gl::GetUniformLocation(terrain_program_, "u_mvp");
    const GLint loc_mv = gl::GetUniformLocation(terrain_program_, "u_model_view");
    const GLint loc_up = gl::GetUniformLocation(terrain_program_, "u_up");
    const GLint loc_tint = gl::GetUniformLocation(terrain_program_, "u_tint");
    const GLint loc_has_tex = gl::GetUniformLocation(terrain_program_, "u_has_texture");
    gl::Uniform1i(gl::GetUniformLocation(terrain_program_, "u_texture"), 0);
    gl::Uniform1i(gl::GetUniformLocation(terrain_program_, "u_lighting"), settings.lighting ? 1 : 0);
    gl::Uniform3f(gl::GetUniformLocation(terrain_program_, "u_light"), static_cast<float>(light.x),
                  static_cast<float>(light.y), static_cast<float>(light.z));
    gl::Uniform1f(gl::GetUniformLocation(terrain_program_, "u_exaggeration"), settings.exaggeration);
    gl::Uniform3f(gl::GetUniformLocation(terrain_program_, "u_haze_color"), sky[0], sky[1], sky[2]);
    const double haze = settings.haze ? std::max(eye_height, 500.0) * 60.0 + 2.0e4 : 0.0;
    gl::Uniform1f(gl::GetUniformLocation(terrain_program_, "u_haze_distance"), static_cast<float>(haze));
    gl::ActiveTexture(GL_TEXTURE0);
    if (settings.wireframe) gl::PolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    for (const DrawTile& d : tiles) {
        auto it = tiles_.find(d.tile);
        if (it != tiles_.end() && it->second.source != d.data.get()) {
            release(it->second);  // rebuilt by the runtime: upload the new data
            tiles_.erase(it);
            it = tiles_.end();
        }
        if (it == tiles_.end()) {
            it = tiles_.emplace(d.tile, upload(*d.data)).first;
            ++stats_.uploads;
        }
        GpuTile& gt = it->second;
        gt.last_used = frame_;

        const Mat4d mv = rot * translation(d.data->mesh.center - camera.position);
        const auto f_mvp = to_float(proj * mv);
        const auto f_mv = to_float(mv);
        gl::UniformMatrix4fv(loc_mvp, 1, GL_FALSE, f_mvp.data());
        gl::UniformMatrix4fv(loc_mv, 1, GL_FALSE, f_mv.data());
        const Vec3d up = d.data->mesh.up;
        gl::Uniform3f(loc_up, static_cast<float>(up.x), static_cast<float>(up.y), static_cast<float>(up.z));
        const bool textured = settings.imagery && gt.texture != 0;
        gl::Uniform1i(loc_has_tex, textured ? 1 : 0);
        if (textured) gl::BindTexture(GL_TEXTURE_2D, gt.texture);
        if (settings.lod_colors) {
            const auto c = lod_color(d.tile.lod);
            gl::Uniform4f(loc_tint, c[0], c[1], c[2], 0.45f);
        } else {
            gl::Uniform4f(loc_tint, 0, 0, 0, 0);
        }
        gl::BindVertexArray(gt.vao);
        gl::DrawElements(GL_TRIANGLES, gt.index_count, GL_UNSIGNED_INT, nullptr);
        stats_.triangles += static_cast<std::size_t>(gt.index_count / 3);
    }
    if (settings.wireframe) gl::PolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gl::BindVertexArray(0);

    // Drop GPU tiles not drawn for a few seconds.
    stats_.gpu_bytes = 0;
    for (auto it = tiles_.begin(); it != tiles_.end();) {
        if (it->second.last_used + 300 < frame_) {
            release(it->second);
            it = tiles_.erase(it);
        } else {
            stats_.gpu_bytes += it->second.bytes;
            ++it;
        }
    }
    stats_.gpu_tiles = tiles_.size();
}
