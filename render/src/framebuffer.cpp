#include "ogccdb/render/framebuffer.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace ogccdb::render {

Framebuffer::Framebuffer(int width, int height, Format format) : width_(width), height_(height), format_(format) {
    if (width <= 0 || height <= 0) throw std::runtime_error("framebuffer size must be positive");
    gl::GenTextures(1, &color_);
    gl::BindTexture(GL_TEXTURE_2D, color_);
    if (format == Format::Rgba32f) {
        gl::TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
    } else {
        gl::TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    }
    gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl::TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl::GenRenderbuffers(1, &depth_);
    gl::BindRenderbuffer(GL_RENDERBUFFER, depth_);
    gl::RenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width, height);
    gl::GenFramebuffers(1, &fbo_);
    gl::BindFramebuffer(GL_FRAMEBUFFER, fbo_);
    gl::FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_, 0);
    gl::FramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depth_);
    const GLenum status = gl::CheckFramebufferStatus(GL_FRAMEBUFFER);
    gl::BindFramebuffer(GL_FRAMEBUFFER, 0);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        gl::DeleteFramebuffers(1, &fbo_);
        gl::DeleteRenderbuffers(1, &depth_);
        gl::DeleteTextures(1, &color_);
        throw std::runtime_error("offscreen framebuffer incomplete (status " + std::to_string(status) + ")");
    }
}

Framebuffer::~Framebuffer() {
    gl::DeleteFramebuffers(1, &fbo_);
    gl::DeleteRenderbuffers(1, &depth_);
    gl::DeleteTextures(1, &color_);
}

void Framebuffer::bind() const {
    gl::BindFramebuffer(GL_FRAMEBUFFER, fbo_);
    gl::Viewport(0, 0, width_, height_);
}

void Framebuffer::unbind() { gl::BindFramebuffer(GL_FRAMEBUFFER, 0); }

RgbImage Framebuffer::read_rgb() const {
    gl::BindFramebuffer(GL_FRAMEBUFFER, fbo_);
    return render::read_rgb(width_, height_);
}

std::vector<float> Framebuffer::read_rgba32f() const {
    gl::BindFramebuffer(GL_FRAMEBUFFER, fbo_);
    std::vector<float> rows(static_cast<std::size_t>(width_) * height_ * 4), out(rows.size());
    gl::PixelStorei(GL_PACK_ALIGNMENT, 4);
    gl::ReadPixels(0, 0, width_, height_, GL_RGBA, GL_FLOAT, rows.data());
    const std::size_t stride = static_cast<std::size_t>(width_) * 4;
    for (int y = 0; y < height_; ++y) {  // OpenGL rows start at the bottom
        std::copy_n(rows.data() + static_cast<std::size_t>(height_ - 1 - y) * stride, stride,
                    out.data() + static_cast<std::size_t>(y) * stride);
    }
    return out;
}

}  // namespace ogccdb::render
