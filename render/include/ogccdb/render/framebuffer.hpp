#pragma once
// An offscreen render target: a colour texture and a depth buffer, for
// drawing without a visible window (or at another size than the window).
#include <vector>

#include "ogccdb/render/gl.hpp"
#include "ogccdb/render/image.hpp"

namespace ogccdb::render {

class Framebuffer {
public:
    enum class Format {
        Rgba8,    ///< colour, as on screen
        Rgba32f,  ///< four floats a pixel (e.g. Renderer::draw_positions)
    };

    /// Needs a current OpenGL context; throws std::runtime_error if the
    /// driver cannot make this target.
    Framebuffer(int width, int height, Format format);
    ~Framebuffer();
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    int width() const { return width_; }
    int height() const { return height_; }

    /// Draws and reads go to this target from now on.
    void bind() const;
    /// Back to the window's own framebuffer.
    static void unbind();

    /// Colour of an Rgba8 target, rows from the top.
    RgbImage read_rgb() const;
    /// Pixels of an Rgba32f target, four floats each, rows from the top.
    std::vector<float> read_rgba32f() const;

private:
    int width_, height_;
    Format format_;
    GLuint fbo_ = 0, color_ = 0, depth_ = 0;
};

}  // namespace ogccdb::render
