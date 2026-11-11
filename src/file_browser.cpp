#include "file_browser.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "imgui.h"
#include "ogccdb/flight/bag.hpp"

namespace fs = std::filesystem;

void FileBrowser::open(const fs::path& start) {
    open_ = true;
    std::error_code ec;
    fs::path dir = start;
    if (dir.empty()) {
        const char* home = std::getenv("HOME");
        dir = home ? fs::path(home) : fs::current_path(ec);
    }
    if (!fs::is_directory(dir, ec)) dir = dir.parent_path();
    list(dir);
}

void FileBrowser::list(const fs::path& dir) {
    std::error_code ec;
    dir_ = fs::absolute(dir, ec).lexically_normal();
    entries_.clear();
    selected_ = -1;
    error_.clear();
    std::snprintf(path_buf_, sizeof path_buf_, "%s", dir_.string().c_str());
    for (fs::directory_iterator it(dir_, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end;
         it.increment(ec)) {
        const fs::path& p = it->path();
        const std::string name = p.filename().string();
        if (name.empty() || name[0] == '.') continue;
        Entry e;
        e.path = p;
        e.dir = it->is_directory(ec);
        e.bag = ogccdb::flight::is_bag(p);
        if (e.dir || e.bag) entries_.push_back(e);
    }
    if (ec) error_ = ec.message();
    std::sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) {
        if (a.dir != b.dir) return a.dir;  // folders first
        return a.path.filename().string() < b.path.filename().string();
    });
}

std::optional<fs::path> FileBrowser::draw() {
    if (!open_) return std::nullopt;
    std::optional<fs::path> chosen;
    ImGui::SetNextWindowSize(ImVec2(620, 440), ImGuiCond_FirstUseEver);
    const ImVec2 ds = ImGui::GetIO().DisplaySize;
    ImGui::SetNextWindowPos(ImVec2(ds.x * 0.5f, ds.y * 0.5f), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
    if (!ImGui::Begin("Open ROS 2 bag", &open_)) {
        ImGui::End();
        return std::nullopt;
    }
    if (ImGui::Button("Up")) list(dir_.parent_path());
    ImGui::SameLine();
    if (ImGui::Button("Home")) {
        if (const char* home = std::getenv("HOME")) list(home);
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputText("##path", path_buf_, sizeof path_buf_, ImGuiInputTextFlags_EnterReturnsTrue)) {
        const fs::path typed(path_buf_);
        std::error_code ec;
        if (ogccdb::flight::is_bag(typed)) {
            chosen = typed;
        } else if (fs::is_directory(typed, ec)) {
            list(typed);
        } else {
            error_ = "no such folder or bag";
        }
    }
    ImGui::TextDisabled("A bag is a folder with metadata.yaml and .mcap or .db3 files, one such file, or a .tar of a bag.");
    if (!error_.empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", error_.c_str());

    const float footer = ImGui::GetFrameHeightWithSpacing() + 4;
    if (ImGui::BeginChild("entries", ImVec2(0, -footer), ImGuiChildFlags_Borders)) {
        for (int i = 0; i < static_cast<int>(entries_.size()); ++i) {
            const Entry& e = entries_[i];
            std::string label = (e.bag ? "[bag] " : e.dir ? "[dir] " : "      ") + e.path.filename().string();
            if (ImGui::Selectable(label.c_str(), selected_ == i, ImGuiSelectableFlags_AllowDoubleClick)) {
                selected_ = i;
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    if (e.bag) {
                        chosen = e.path;
                    } else if (e.dir) {
                        list(e.path);
                        break;
                    }
                }
            }
        }
    }
    ImGui::EndChild();
    const bool can_open = selected_ >= 0 && selected_ < static_cast<int>(entries_.size()) && entries_[selected_].bag;
    ImGui::BeginDisabled(!can_open);
    if (ImGui::Button("Open bag") && can_open) chosen = entries_[selected_].path;
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel")) open_ = false;
    ImGui::SameLine();
    ImGui::TextDisabled("double-click a folder to enter it, a bag to open it");
    ImGui::End();
    if (chosen) open_ = false;
    return chosen;
}
