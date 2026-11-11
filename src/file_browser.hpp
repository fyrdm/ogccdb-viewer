#pragma once
// A small file browser window (Dear ImGui) for choosing a ROS 2 bag: a bag
// folder (metadata.yaml and its storage files) or an .mcap or .db3 file.
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

class FileBrowser {
public:
    void open(const std::filesystem::path& start);
    bool is_open() const { return open_; }
    /// Draws the window; returns the chosen bag once, when the user opens one.
    std::optional<std::filesystem::path> draw();

private:
    struct Entry {
        std::filesystem::path path;
        bool dir = false;
        bool bag = false;
    };
    void list(const std::filesystem::path& dir);

    bool open_ = false;
    std::filesystem::path dir_;
    std::vector<Entry> entries_;
    int selected_ = -1;
    char path_buf_[1024] = {};
    std::string error_;
};
