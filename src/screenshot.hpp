#pragma once
#include <string>

/// Reads the current back buffer and writes it as PNG. Returns false on failure.
bool save_screenshot(const std::string& path, int width, int height);
