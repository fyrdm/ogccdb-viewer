#include "ogccdb/render/image.hpp"

#include <algorithm>

#include "ogccdb/render/gl.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace ogccdb::render {

RgbImage read_rgb(int width, int height) {
    RgbImage img;
    img.width = width;
    img.height = height;
    img.rgb.resize(static_cast<std::size_t>(width) * height * 3);
    std::vector<std::uint8_t> rows(img.rgb.size());
    gl::PixelStorei(GL_PACK_ALIGNMENT, 1);
    gl::ReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, rows.data());
    // OpenGL rows start at the bottom.
    const std::size_t stride = static_cast<std::size_t>(width) * 3;
    for (int y = 0; y < height; ++y) {
        std::copy_n(rows.data() + static_cast<std::size_t>(height - 1 - y) * stride, stride,
                    img.rgb.data() + static_cast<std::size_t>(y) * stride);
    }
    return img;
}

bool save_screenshot(const std::string& path, int width, int height) {
    gl::ReadBuffer(GL_BACK);
    return write_png(path, read_rgb(width, height));
}

bool write_png(const std::string& path, const RgbImage& image) {
    return stbi_write_png(path.c_str(), image.width, image.height, 3, image.rgb.data(), image.width * 3) != 0;
}

std::vector<std::uint8_t> encode_jpeg(const RgbImage& image, int quality) {
    std::vector<std::uint8_t> out;
    auto sink = [](void* ctx, void* data, int size) {
        auto* v = static_cast<std::vector<std::uint8_t>*>(ctx);
        const auto* d = static_cast<const std::uint8_t*>(data);
        v->insert(v->end(), d, d + size);
    };
    if (!stbi_write_jpg_to_func(sink, &out, image.width, image.height, 3, image.rgb.data(), quality)) out.clear();
    return out;
}

}  // namespace ogccdb::render
