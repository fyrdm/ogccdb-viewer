#pragma once
// Reading rendered pixels back and writing them as PNG or JPEG.
#include <cstdint>
#include <string>
#include <vector>

namespace ogccdb::render {

/// 8-bit RGB pixels, rows from the top.
struct RgbImage {
    int width = 0, height = 0;
    std::vector<std::uint8_t> rgb;
};

/// Reads the colour of the bound read framebuffer (the back buffer of a
/// window, or a Framebuffer), rows from the top.
RgbImage read_rgb(int width, int height);

/// Reads the current back buffer and writes it as PNG. Returns false on failure.
bool save_screenshot(const std::string& path, int width, int height);

bool write_png(const std::string& path, const RgbImage& image);
/// JPEG at `quality` 1..100; empty on failure.
std::vector<std::uint8_t> encode_jpeg(const RgbImage& image, int quality);

}  // namespace ogccdb::render
