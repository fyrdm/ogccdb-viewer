#include "flight_player.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

#include "imgui.h"

using namespace ogccdb::flight;
using namespace ogccdb::runtime;

bool FlightPlayer::load(const std::filesystem::path& path) {
    try {
        auto bag = Bag::open(path);
        FlightTrack track = FlightTrack::build(*bag);
        bag_ = std::move(bag);
        track_ = std::move(track);
        path_ = path;
        error_.clear();
        time = track_.start();
        playing = false;
        ground_offset_ = 0.0;
        ground_lod_ = -100;
        path_cache_offset_ = 1e300;
        // A flight that starts at rest starts on the ground: put it there, as
        // GNSS and barometric heights are often off by metres or more.
        const FlightPose p0 = track_.at(track_.start());
        altitude = (p0.ground_speed < 0.5 && std::fabs(p0.climb) < 0.5) ? Altitude::StartOnGround : Altitude::AsRecorded;
        return true;
    } catch (const std::exception& e) {
        error_ = e.what();
        return false;
    }
}

void FlightPlayer::update(double dt) {
    if (!loaded() || !playing) return;
    time += dt * speed;
    const double end = track_.end(), begin = track_.start();
    if (time > end) {
        if (loop) {
            time = begin;
        } else {
            time = end;
            playing = false;
        }
    }
}

void FlightPlayer::seek(double t) {
    if (loaded()) time = std::clamp(t, track_.start(), track_.end());
}

double FlightPlayer::offset() const {
    switch (altitude) {
        case Altitude::AsRecorded: return 0.0;
        case Altitude::StartOnGround: return ground_offset_;
        case Altitude::Manual: return manual_offset;
    }
    return 0.0;
}

Geodetic FlightPlayer::start() const { return track_.at(track_.start()).geodetic; }

void FlightPlayer::offer_ground(double terrain_height, int lod) {
    if (lod < ground_lod_) return;
    ground_lod_ = lod;
    ground_offset_ = terrain_height - start().height;
}

FlightPose FlightPlayer::pose() const {
    FlightPose p = track_.at(time);
    const double off = offset();
    p.position += ellipsoid_up(p.geodetic.lat, p.geodetic.lon) * off;
    p.geodetic.height += off;
    return p;
}

const std::vector<Vec3d>& FlightPlayer::flight_path() {
    const double off = offset();
    if (off != path_cache_offset_) {
        path_cache_ = track_.path();
        for (Vec3d& v : path_cache_) {
            const Geodetic g = ecef_to_geodetic(v);
            v += ellipsoid_up(g.lat, g.lon) * off;
        }
        path_cache_offset_ = off;
    }
    return path_cache_;
}

namespace {

std::string clock(double s) {
    char b[32];
    const int m = static_cast<int>(s / 60.0);
    std::snprintf(b, sizeof b, "%d:%04.1f", m, s - m * 60.0);
    return b;
}

}  // namespace

void flight_ui(FlightPlayer& pl, FileBrowser& browser, std::optional<double> agl) {
    if (!ImGui::CollapsingHeader("Flight (ROS 2 bag)", ImGuiTreeNodeFlags_DefaultOpen)) return;
    if (ImGui::Button("Open bag...")) browser.open(pl.loaded() ? pl.path().parent_path() : std::filesystem::path());
    if (pl.loaded()) {
        ImGui::SameLine();
        ImGui::TextUnformatted(pl.path().filename().string().c_str());
    }
    if (!pl.error().empty()) ImGui::TextColored(ImVec4(1, 0.5f, 0.4f, 1), "%s", pl.error().c_str());
    if (!pl.loaded()) {
        ImGui::TextDisabled("Choose a bag with a NavSatFix (and a local pose for attitude).");
        return;
    }
    const FlightTrack& tr = pl.track();
    const TrackInfo& info = tr.info();
    Bag& bag = pl.bag();

    // Transport.
    if (ImGui::Button("|<")) pl.seek(tr.start());
    ImGui::SameLine();
    if (ImGui::Button(pl.playing ? "Pause (P)" : "Play (P)")) {
        if (!pl.playing && pl.time >= tr.end()) pl.seek(tr.start());
        pl.playing = !pl.playing;
    }
    ImGui::SameLine();
    if (ImGui::Button(">|")) pl.seek(tr.end());
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &pl.loop);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    ImGui::SliderFloat("##speed", &pl.speed, 0.1f, 32.0f, "%.1fx", ImGuiSliderFlags_Logarithmic);
    float t = static_cast<float>(pl.time);
    const std::string label = clock(pl.time - tr.start()) + " / " + clock(tr.duration());
    ImGui::SetNextItemWidth(-1);
    if (ImGui::SliderFloat("##time", &t, static_cast<float>(tr.start()), static_cast<float>(tr.end()), label.c_str())) {
        pl.seek(t);
    }

    // Camera.
    ImGui::Checkbox("Drone camera (B)", &pl.drone_camera);
    ImGui::SameLine();
    ImGui::Checkbox("Path", &pl.show_path);
    if (!pl.drone_camera) {
        ImGui::SameLine();
        ImGui::Checkbox("Follow", &pl.follow);
    }
    ImGui::SliderFloat("Camera tilt", &pl.mount_pitch, -90.0f, 0.0f, "%.0f deg");
    ImGui::SameLine();
    if (ImGui::SmallButton("down")) pl.mount_pitch = -90.0f;

    // Altitude.
    int mode = static_cast<int>(pl.altitude);
    ImGui::RadioButton("As recorded", &mode, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Start on ground", &mode, 1);
    ImGui::SameLine();
    ImGui::RadioButton("Offset", &mode, 2);
    pl.altitude = static_cast<FlightPlayer::Altitude>(mode);
    if (pl.altitude == FlightPlayer::Altitude::Manual) {
        ImGui::DragFloat("Height offset", &pl.manual_offset, 0.5f, -5000.0f, 5000.0f, "%+.1f m");
    } else if (pl.altitude == FlightPlayer::Altitude::StartOnGround) {
        ImGui::TextDisabled("Heights shifted by %+.1f m to put the start on the terrain", pl.offset());
    }

    // Live values.
    const FlightPose p = pl.pose();
    ImGui::Text("%.7f, %.7f  h %.1f m", p.geodetic.lat, p.geodetic.lon, p.geodetic.height);
    if (agl) {
        ImGui::SameLine();
        ImGui::Text(" AGL %.1f m", *agl);
    }
    ImGui::Text("Heading %5.1f  pitch %+5.1f  roll %+5.1f deg", p.yaw, p.pitch, p.roll);
    ImGui::Text("Ground speed %.1f m/s, climb %+.1f m/s", p.ground_speed, p.climb);

    if (ImGui::TreeNode("Bag")) {
        ImGui::Text("%s storage, %.1f s, %llu messages", bag.storage().c_str(), tr.bag_duration(),
                    static_cast<unsigned long long>(bag.message_count()));
        ImGui::TextWrapped("Position: %s (%s%s%s)", info.position_source.c_str(), info.fix_topic.c_str(),
                           info.local_topic.empty() ? "" : ", ", info.local_topic.c_str());
        if (!info.local_topic.empty()) {
            ImGui::Text("Local origin %.6f, %.6f; fixes vs track %.2f m RMS", info.origin.lat, info.origin.lon,
                        info.anchor_rms);
        }
        ImGui::TextWrapped("Attitude: %s", info.attitude_topic.empty() ? "none (level, nose along the track)"
                                                                        : info.attitude_topic.c_str());
        ImGui::TreePop();
    }
    // The bag's topics as they are played: messages so far and their rate.
    if (ImGui::BeginTable("topics", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Topic");
        ImGui::TableSetupColumn("Played");
        ImGui::TableSetupColumn("Rate");
        ImGui::TableHeadersRow();
        const auto& times = tr.topic_times();
        for (std::size_t i = 0; i < bag.topics().size() && i < times.size(); ++i) {
            const auto& tt = times[i];
            const auto now = std::upper_bound(tt.begin(), tt.end(), pl.time);
            const auto before = std::upper_bound(tt.begin(), tt.end(), pl.time - 1.0);
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(bag.topics()[i].name.c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", bag.topics()[i].type.c_str());
            ImGui::TableNextColumn();
            ImGui::Text("%zu / %zu", static_cast<std::size_t>(now - tt.begin()), tt.size());
            ImGui::TableNextColumn();
            ImGui::Text("%.0f Hz", static_cast<double>(now - before));
        }
        ImGui::EndTable();
    }
}
