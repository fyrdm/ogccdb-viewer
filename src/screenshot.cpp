#include "screenshot.hpp"

#include <cstdint>
#include <vector>

#include "gl.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

bool save_screenshot(const std::string& path, int width, int height) {
    std::vector<std::uint8_t> px(static_cast<std::size_t>(width) * height * 3);
    gl::PixelStorei(GL_PACK_ALIGNMENT, 1);
    gl::ReadBuffer(GL_BACK);
    gl::ReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, px.data());
    stbi_flip_vertically_on_write(1);  // OpenGL rows start at the bottom
    return stbi_write_png(path.c_str(), width, height, 3, px.data(), width * 3) != 0;
}
