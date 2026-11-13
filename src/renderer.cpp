#include "renderer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
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
    // Seen from below (underground): darker and warmer, so the underside is obvious.
    if (!gl_FrontFacing) c = mix(c * 0.45, vec3(0.45, 0.25, 0.20), 0.25);
    if (u_haze_distance > 0.0) {
        float f = 1.0 - exp(-v_distance / u_haze_distance);
        c = mix(c, u_haze_color, clamp(f, 0.0, 0.85));
    }
    frag = vec4(c, 1.0);
}
)";

// Reference surface: vertices are already relative to the eye (computed in
// double precision on the CPU), so it is exact near the ground too.
// Waves: a sum of travelling sine waves on a local plane around the eye
// (a_wave, metres east and north of an anchor). The longer ones lift the
// vertices near the eye; all of them shape the normals, each fading out
// where it would be smaller than a pixel.
const char* kSurfaceVs = R"(#version 330 core
layout(location = 0) in vec3 a_rel;
layout(location = 1) in vec3 a_up;
layout(location = 2) in vec3 a_latlon;  // latitude (deg), cos and sin of longitude
layout(location = 3) in vec2 a_wave;
uniform mat4 u_vp;
uniform int u_wave_count;
uniform int u_vertex_waves;
uniform vec4 u_wave[24];  // wave number east, north (rad/m); amplitude (m); phase (rad)
out vec3 v_rel;
out vec3 v_up;
out vec3 v_latlon;
out vec2 v_wave;
void main() {
    vec3 p = a_rel;
    // Ring spacing is about a tenth of the distance: lift only by waves the mesh can hold.
    float spacing = 0.11 * length(a_rel);
    float h = 0.0;
    for (int i = 0; i < u_vertex_waves && i < u_wave_count; ++i) {
        float wavelength = 6.2831853 / length(u_wave[i].xy);
        float w = 1.0 - smoothstep(0.08, 0.2, spacing / wavelength);
        // exp(sin - 1): sharp crests, wide troughs, like real wind waves.
        h += w * u_wave[i].z * (2.0 * exp(sin(dot(u_wave[i].xy, a_wave) - u_wave[i].w) - 1.0) - 1.0);
    }
    p += a_up * h;
    gl_Position = u_vp * vec4(p, 1.0);
    v_rel = p;
    v_up = a_up;
    v_latlon = a_latlon;
    v_wave = a_wave;
}
)";

const char* kSurfaceFs = R"(#version 330 core
in vec3 v_rel;
in vec3 v_up;
in vec3 v_latlon;
in vec2 v_wave;
uniform int u_sea;
uniform int u_graticule;
uniform vec3 u_light;
uniform vec3 u_sky;
uniform float u_haze_distance;
uniform int u_wave_count;
uniform vec4 u_wave[24];
uniform vec3 u_east;
uniform vec3 u_north;
uniform float u_wave_height;
out vec4 frag;
// Longitude jumps by 360 degrees at the antimeridian: take its change across
// a pixel the short way round, so the seam doesn't turn into a wide band.
float lon_width(float lon) {
    float dx = dFdx(lon), dy = dFdy(lon);
    dx -= 360.0 * floor(dx / 360.0 + 0.5);
    dy -= 360.0 * floor(dy / 360.0 + 0.5);
    return abs(dx) + abs(dy);
}
float grid(vec2 coord, vec2 width, float spacing) {
    vec2 g = abs(fract(coord / spacing - 0.5) - 0.5) / max(width / spacing, vec2(1e-6));
    return 1.0 - min(min(g.x, g.y), 1.0);
}
void main() {
    vec3 up = normalize(v_up);
    float dist = length(v_rel);
    vec3 c;
    if (u_sea != 0) {
        // Pixel footprint on the water, in metres (its longer side, so waves fade
        // early at grazing angles instead of turning into streaks).
        float footprint = max(length(dFdx(v_wave)), length(dFdy(v_wave)));
        vec2 grad = vec2(0.0);
        float h = 0.0, lost = 0.0, fine = 0.0;
        for (int i = 0; i < u_wave_count; ++i) {
            vec2 k = u_wave[i].xy;
            float a = u_wave[i].z;
            float ph = dot(k, v_wave) - u_wave[i].w;
            // Phase change across one pixel: fade the wave before it aliases.
            float w = 1.0 - smoothstep(0.3, 0.8, length(k) * footprint);
            float e = exp(sin(ph) - 1.0);  // same wave shape as the vertex shader
            grad += w * a * k * 2.0 * e * cos(ph);
            h += w * a * (2.0 * e - 1.0);
            if (i >= u_wave_count / 2) fine += w * sin(ph);  // small waves: break up the foam
            float sl = (1.0 - w) * a * length(k);  // slope too fine to see: roughness
            lost += sl * sl;
        }
        float rough = smoothstep(0.0, 0.35, sqrt(lost));  // RMS of the slopes faded out
        // Far away the visible waves are few: soften them with the lost detail.
        vec3 n = normalize(up - (u_east * grad.x + u_north * grad.y) * (1.0 - 0.6 * rough));
        vec3 view = normalize(-v_rel);
        if (dot(n, view) < 0.02) n = normalize(n + view * (0.02 - dot(n, view)));
        float fres = 0.02 + 0.98 * pow(1.0 - max(dot(n, view), 0.0), 5.0);
        vec3 deep = vec3(0.02, 0.10, 0.17);
        float crest = u_wave_height > 1e-3 ? clamp(h / (0.5 * u_wave_height), -1.5, 1.5) : 0.0;
        deep += vec3(0.0, 0.035, 0.035) * clamp(crest, 0.0, 1.0);  // light through the crests
        // Reflected sky: bright at the horizon, deeper blue overhead, so each
        // wave facet reflects a different part of it.
        vec3 r = reflect(-view, n);
        float ru = dot(r, up);
        vec3 zenith = vec3(0.22, 0.40, 0.70);
        vec3 reflected = ru > 0.0 ? mix(u_sky, zenith, pow(ru, 0.6)) : u_sky * 0.8;
        c = mix(deep, reflected, clamp(fres, 0.0, 1.0));
        vec3 hv = normalize(u_light + view);
        float power = mix(400.0, 25.0, rough);
        c += vec3(1.0, 0.96, 0.88) * pow(max(dot(n, hv), 0.0), power) * mix(1.2, 0.25, rough);
        // Foam on the highest crests in a rough sea.
        float foam = smoothstep(0.7, 1.2, crest) * smoothstep(0.0, 1.5, fine) *
                     clamp((u_wave_height - 1.0) / 3.0, 0.0, 0.7) * (1.0 - rough);
        c = mix(c, vec3(0.85, 0.9, 0.92), foam);
    } else {
        c = vec3(0.10, 0.16, 0.24);
    }
    if (u_graticule != 0) {
        // 1-degree lines fade out where they would crowd the screen.
        // Longitude from its interpolated cosine and sine: no seam to interpolate across.
        vec2 ll = vec2(v_latlon.x, degrees(atan(v_latlon.z, v_latlon.y)));
        vec2 width = vec2(fwidth(ll.x), lon_width(ll.y));
        float fine = 1.0 - smoothstep(0.05, 0.15, max(width.x, width.y));
        float l = max(grid(ll, width, 10.0), 0.5 * fine * grid(ll, width, 1.0));
        c = mix(c, vec3(0.55, 0.65, 0.75), l * (u_sea != 0 ? 0.35 : 1.0));
    }
    if (u_haze_distance > 0.0) {
        float f = 1.0 - exp(-dist / u_haze_distance);
        c = mix(c, u_sky, clamp(f, 0.0, 0.85));
    }
    frag = vec4(c, 1.0);
}
)";

// Lines (flight path, vehicle): eye-relative positions, one colour.
const char* kLineVs = R"(#version 330 core
layout(location = 0) in vec3 a_rel;
uniform mat4 u_vp;
void main() { gl_Position = u_vp * vec4(a_rel, 1.0); }
)";
const char* kLineFs = R"(#version 330 core
uniform vec4 u_color;
out vec4 frag;
void main() { frag = u_color; }
)";

constexpr int kSurfaceRings = 96;
constexpr int kSurfaceSegments = 160;
constexpr int kSurfaceFloats = 11;  // eye-relative position 3, up 3, lat + cos/sin lon 3, wave plane 2
constexpr int kWaves = 24;
constexpr int kVertexWaves = 6;
// Wave numbers are whole multiples of 2 pi / kWavePeriod, so the wave field
// repeats every kWavePeriod metres and the anchor can move by that much unseen.
constexpr double kWavePeriod = 2048.0;

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
    surface_program_ = make_program(kSurfaceVs, kSurfaceFs);
    line_program_ = make_program(kLineVs, kLineFs);
    gl::GenVertexArrays(1, &line_vao_);
    gl::BindVertexArray(line_vao_);
    gl::GenBuffers(1, &line_vbo_);
    gl::BindBuffer(GL_ARRAY_BUFFER, line_vbo_);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    gl::BindVertexArray(0);

    if (gl::has_extension("GL_EXT_texture_filter_anisotropic") || gl::has_extension("GL_ARB_texture_filter_anisotropic")) {
        gl::GetFloatv(kMaxTextureMaxAnisotropy, &max_anisotropy_);
    }

    // Reference surface: a centre vertex below the eye and rings around it out
    // to the horizon. Positions change every frame; the triangles do not.
    std::vector<std::uint32_t> idx;
    const std::uint32_t segs = kSurfaceSegments;
    for (std::uint32_t j = 0; j < segs; ++j) idx.insert(idx.end(), {0u, 1 + j, 1 + (j + 1) % segs});
    for (std::uint32_t i = 0; i + 1 < static_cast<std::uint32_t>(kSurfaceRings); ++i) {
        for (std::uint32_t j = 0; j < segs; ++j) {
            const std::uint32_t a0 = 1 + i * segs + j, a1 = 1 + i * segs + (j + 1) % segs;
            const std::uint32_t b0 = a0 + segs, b1 = a1 + segs;
            idx.insert(idx.end(), {a0, b0, a1, a1, b0, b1});
        }
    }
    surface_index_count_ = static_cast<GLsizei>(idx.size());
    gl::GenVertexArrays(1, &surface_vao_);
    gl::BindVertexArray(surface_vao_);
    gl::GenBuffers(1, &surface_vbo_);
    gl::BindBuffer(GL_ARRAY_BUFFER, surface_vbo_);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>((1 + kSurfaceRings * kSurfaceSegments) * kSurfaceFloats * sizeof(float)),
                   nullptr, GL_STREAM_DRAW);
    gl::GenBuffers(1, &surface_ibo_);
    gl::BindBuffer(GL_ELEMENT_ARRAY_BUFFER, surface_ibo_);
    gl::BufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(idx.size() * sizeof(std::uint32_t)), idx.data(),
                   GL_STATIC_DRAW);
    const GLsizei stride = kSurfaceFloats * sizeof(float);
    gl::EnableVertexAttribArray(0);
    gl::VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, nullptr);
    gl::EnableVertexAttribArray(1);
    gl::VertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(3 * sizeof(float)));
    gl::EnableVertexAttribArray(2);
    gl::VertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(6 * sizeof(float)));
    gl::EnableVertexAttribArray(3);
    gl::VertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<void*>(9 * sizeof(float)));
    gl::BindVertexArray(0);
}

Renderer::~Renderer() {
    for (auto& [t, g] : tiles_) release(g);
    gl::DeleteVertexArrays(1, &surface_vao_);
    gl::DeleteBuffers(1, &surface_vbo_);
    gl::DeleteBuffers(1, &surface_ibo_);
    gl::DeleteProgram(terrain_program_);
    gl::DeleteProgram(surface_program_);
    gl::DeleteVertexArrays(1, &line_vao_);
    gl::DeleteBuffers(1, &line_vbo_);
    gl::DeleteProgram(line_program_);
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

void Renderer::draw_surface(const Camera& camera, const RenderSettings& settings, const float sky[3], double haze,
                            Vec3d light) {
    // Without the sea, a dark background globe 50 m down (no depth), as before.
    const double k = settings.exaggeration;
    auto level_at = [&](double lat, double lon) {
        if (!settings.sea) return -50.0;
        return settings.sea_height ? settings.sea_height(lat, lon) * k : 0.0;
    };
    const Geodetic eye = ecef_to_geodetic(camera.position);
    const double level = level_at(eye.lat, eye.lon);
    const double alt = std::max(eye.height - level, 1.0);
    const double r = kWgs84A;
    // Rings by angle from the point below the eye, spaced geometrically out to
    // just past the horizon, so spacing grows with distance like the detail needed.
    const double horizon = std::acos(r / (r + alt));
    const double t_max = std::min(horizon * 1.05 + 1e-4, 1.45);
    const double t_min = std::clamp(alt * 0.02 / r, 1e-9, t_max / 50.0);
    const Vec3d e = local_east(eye.lat, eye.lon), n = local_north(eye.lat, eye.lon);
    const Vec3d nadir = geodetic_to_ecef({eye.lat, eye.lon, level});

    // Wave plane: metres east and north of an anchor near the eye. The anchor
    // moves by whole wave periods, so the waves do not jump when it does.
    if (!wave_anchor_set_) set_wave_anchor(eye.lat, eye.lon);
    {
        const Vec3d d = nadir - wave_anchor_;
        const double u = dot(d, wave_east_), v = dot(d, wave_north_);
        if (std::max(std::fabs(u), std::fabs(v)) > 4.0 * kWavePeriod) {
            const double su = std::round(u / kWavePeriod) * kWavePeriod, sv = std::round(v / kWavePeriod) * kWavePeriod;
            const Geodetic a = ecef_to_geodetic(wave_anchor_ + wave_east_ * su + wave_north_ * sv);
            set_wave_anchor(a.lat, a.lon);
        }
    }

    surface_vertices_.clear();
    surface_vertices_.reserve(static_cast<std::size_t>(1 + kSurfaceRings * kSurfaceSegments) * kSurfaceFloats);
    auto push = [&](double lat, double lon) {
        const Vec3d w = geodetic_to_ecef({lat, lon, level_at(lat, lon)});
        const Vec3d p = w - camera.position;
        const Vec3d up = ellipsoid_up(lat, lon);
        const Vec3d a = w - wave_anchor_;
        surface_vertices_.insert(surface_vertices_.end(),
                                 {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z),
                                  static_cast<float>(up.x), static_cast<float>(up.y), static_cast<float>(up.z),
                                  static_cast<float>(lat), static_cast<float>(std::cos(lon * kPi / 180.0)),
                                  static_cast<float>(std::sin(lon * kPi / 180.0)),
                                  static_cast<float>(dot(a, wave_east_)), static_cast<float>(dot(a, wave_north_))});
    };
    push(eye.lat, eye.lon);
    for (int i = 0; i < kSurfaceRings; ++i) {
        const double t = t_min * std::pow(t_max / t_min, static_cast<double>(i) / (kSurfaceRings - 1));
        const double d = r * std::tan(t);  // on the tangent plane below the eye
        for (int j = 0; j < kSurfaceSegments; ++j) {
            const double az = 2.0 * kPi * j / kSurfaceSegments;
            const Geodetic g = ecef_to_geodetic(nadir + (e * std::cos(az) + n * std::sin(az)) * d);
            push(g.lat, g.lon);
        }
    }

    const Mat4d rot = look_at_direction({0, 0, 0}, camera.direction, camera.up);
    const auto vp = to_float(camera.projection() * rot);
    gl::UseProgram(surface_program_);
    gl::UniformMatrix4fv(gl::GetUniformLocation(surface_program_, "u_vp"), 1, GL_FALSE, vp.data());
    gl::Uniform1i(gl::GetUniformLocation(surface_program_, "u_sea"), settings.sea ? 1 : 0);
    gl::Uniform1i(gl::GetUniformLocation(surface_program_, "u_graticule"), settings.graticule ? 1 : 0);
    gl::Uniform3f(gl::GetUniformLocation(surface_program_, "u_light"), static_cast<float>(light.x),
                  static_cast<float>(light.y), static_cast<float>(light.z));
    gl::Uniform3f(gl::GetUniformLocation(surface_program_, "u_sky"), sky[0], sky[1], sky[2]);
    gl::Uniform1f(gl::GetUniformLocation(surface_program_, "u_haze_distance"), static_cast<float>(haze));
    const auto waves = wave_uniforms(settings);
    const int wave_count = settings.sea && settings.waves && settings.wave_height > 0.0f ? kWaves : 0;
    gl::Uniform1i(gl::GetUniformLocation(surface_program_, "u_wave_count"), wave_count);
    gl::Uniform1i(gl::GetUniformLocation(surface_program_, "u_vertex_waves"), kVertexWaves);
    gl::Uniform4fv(gl::GetUniformLocation(surface_program_, "u_wave"), kWaves, waves.data());
    gl::Uniform3f(gl::GetUniformLocation(surface_program_, "u_east"), static_cast<float>(wave_east_.x),
                  static_cast<float>(wave_east_.y), static_cast<float>(wave_east_.z));
    gl::Uniform3f(gl::GetUniformLocation(surface_program_, "u_north"), static_cast<float>(wave_north_.x),
                  static_cast<float>(wave_north_.y), static_cast<float>(wave_north_.z));
    gl::Uniform1f(gl::GetUniformLocation(surface_program_, "u_wave_height"), settings.wave_height);
    gl::BindVertexArray(surface_vao_);
    gl::BindBuffer(GL_ARRAY_BUFFER, surface_vbo_);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(surface_vertices_.size() * sizeof(float)),
                   surface_vertices_.data(), GL_STREAM_DRAW);
    // The sea hides terrain below it; the background globe never does.
    if (!settings.sea) gl::DepthMask(GL_FALSE);
    gl::DrawElements(GL_TRIANGLES, surface_index_count_, GL_UNSIGNED_INT, nullptr);
    gl::DepthMask(GL_TRUE);
    gl::BindVertexArray(0);
}

void Renderer::draw_lines(const std::vector<Vec3d>& points, GLenum mode, const float rgba[4], const Camera& camera,
                          float hidden_alpha) {
    if (points.size() < 2) return;
    line_vertices_.resize(points.size() * 3);
    for (std::size_t i = 0; i < points.size(); ++i) {
        const Vec3d r = points[i] - camera.position;  // eye-relative in double, then float
        line_vertices_[i * 3] = static_cast<float>(r.x);
        line_vertices_[i * 3 + 1] = static_cast<float>(r.y);
        line_vertices_[i * 3 + 2] = static_cast<float>(r.z);
    }
    const auto vp = to_float(camera.projection() * look_at_direction({0, 0, 0}, camera.direction, camera.up));
    gl::UseProgram(line_program_);
    gl::UniformMatrix4fv(gl::GetUniformLocation(line_program_, "u_vp"), 1, GL_FALSE, vp.data());
    const GLint color = gl::GetUniformLocation(line_program_, "u_color");
    gl::BindVertexArray(line_vao_);
    gl::BindBuffer(GL_ARRAY_BUFFER, line_vbo_);
    gl::BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(line_vertices_.size() * sizeof(float)),
                   line_vertices_.data(), GL_STREAM_DRAW);
    gl::Enable(GL_BLEND);
    gl::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    gl::Enable(GL_DEPTH_TEST);
    gl::DepthMask(GL_FALSE);
    const auto n = static_cast<GLsizei>(points.size());
    if (hidden_alpha > 0.0f) {  // the parts behind terrain, faint
        gl::DepthFunc(GL_GREATER);
        gl::Uniform4f(color, rgba[0], rgba[1], rgba[2], rgba[3] * hidden_alpha);
        gl::DrawArrays(mode, 0, n);
    }
    gl::DepthFunc(GL_LEQUAL);
    gl::Uniform4f(color, rgba[0], rgba[1], rgba[2], rgba[3]);
    gl::DrawArrays(mode, 0, n);
    gl::DepthMask(GL_TRUE);
    gl::Disable(GL_BLEND);
    gl::BindVertexArray(0);
}

void Renderer::set_wave_anchor(double lat, double lon) {
    wave_anchor_ = geodetic_to_ecef({lat, lon, 0.0});
    wave_east_ = local_east(lat, lon);
    wave_north_ = local_north(lat, lon);
    wave_anchor_set_ = true;
}

std::vector<float> Renderer::wave_uniforms(const RenderSettings& settings) const {
    // A wind sea: the dominant wave is 0.4 H high (crest) with steepness
    // k a = 0.1, so its length grows with H; shorter waves down to a metre are
    // slightly less steep. They spread around the downwind direction.
    // Fixed pseudo-random jitter in length, direction and phase keeps the
    // crests short and irregular instead of a regular lattice.
    auto jitter = [](int i, double salt) {  // in [-1, 1)
        const double x = std::sin(i * 12.9898 + salt * 78.233) * 43758.5453;
        return 2.0 * (x - std::floor(x)) - 1.0;
    };
    constexpr double g = 9.81;
    const double height = std::max(static_cast<double>(settings.wave_height), 0.0);
    const double a0 = 0.4 * height;
    const double longest = std::clamp(2.0 * kPi * a0 / 0.1, 6.0, 400.0);
    const double ratio = std::pow(1.0 / longest, 1.0 / (kWaves - 1));
    const double t = fixed_time_ ? *fixed_time_
                                 : std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    const double unit = 2.0 * kPi / kWavePeriod;
    std::vector<float> w(static_cast<std::size_t>(kWaves) * 4);
    for (int i = 0; i < kWaves; ++i) {
        const double wavelength = longest * std::pow(ratio, i + 0.4 * jitter(i, 1.0));
        // Waves travel downwind, within +-70 degrees, most of them close to it.
        const double spread = 70.0 * jitter(i, 2.0) * std::fabs(jitter(i, 3.0));
        const double dir = (settings.wind_from + 180.0 + spread) * kDegToRad;
        const double k = 2.0 * kPi / wavelength;
        const double kx = std::round(k * std::sin(dir) / unit) * unit, ky = std::round(k * std::cos(dir) / unit) * unit;
        const double omega = std::sqrt(g * std::hypot(kx, ky));  // deep water: c = sqrt(g / k)
        const double phase0 = kPi * jitter(i, 4.0);
        w[i * 4] = static_cast<float>(kx);
        w[i * 4 + 1] = static_cast<float>(ky);
        w[i * 4 + 2] = static_cast<float>(a0 * std::pow(wavelength / longest, 1.2));
        w[i * 4 + 3] = static_cast<float>(std::fmod(omega * t + phase0, 2.0 * kPi));
    }
    return w;
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
    if (!settings.sea) draw_surface(camera, settings, sky, haze, light);  // background first
    gl::UseProgram(terrain_program_);
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
    if (settings.sea) draw_surface(camera, settings, sky, haze, light);  // hides terrain below sea level
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
