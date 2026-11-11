#pragma once
// Plays a vehicle's flight from a ROS 2 bag: a clock over the bag's time,
// the vehicle's pose at that time, and the panel that controls it.
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "file_browser.hpp"
#include "ogccdb/flight/bag.hpp"
#include "ogccdb/flight/track.hpp"

class FlightPlayer {
public:
    enum class Altitude { AsRecorded, StartOnGround, Manual };

    /// Opens a bag and builds its track; on failure keeps the current one and
    /// sets error().
    bool load(const std::filesystem::path& path);
    bool loaded() const { return bag_ != nullptr; }
    const std::string& error() const { return error_; }
    const std::filesystem::path& path() const { return path_; }
    ogccdb::flight::Bag& bag() { return *bag_; }
    const ogccdb::flight::FlightTrack& track() const { return track_; }

    /// Advances the clock by `dt` seconds of wall time while playing.
    void update(double dt);
    void seek(double t);
    /// The vehicle at the current time, with the altitude offset applied.
    ogccdb::flight::FlightPose pose() const;
    /// The flight path with the altitude offset applied (ECEF).
    const std::vector<ogccdb::runtime::Vec3d>& flight_path();
    /// Where the track starts (for aligning it with the ground).
    ogccdb::flight::Geodetic start() const;
    /// Vertical offset applied to the track, metres.
    double offset() const;
    /// Offers the terrain height under the track's start, from a tile of `lod`;
    /// used by Altitude::StartOnGround (a finer tile replaces a coarser one).
    void offer_ground(double terrain_height, int lod);

    // Playback state, edited by the panel and the keys.
    double time = 0.0;  ///< seconds from the start of the bag
    bool playing = false;
    float speed = 1.0f;
    bool loop = true;
    // View.
    bool drone_camera = true;    ///< the view is the vehicle's camera
    float mount_pitch = -90.0f;  ///< camera tilt from the body's forward; -90 = straight down
    bool show_path = true;
    bool follow = false;         ///< the orbit camera follows the vehicle
    Altitude altitude = Altitude::AsRecorded;
    float manual_offset = 0.0f;

private:
    std::unique_ptr<ogccdb::flight::Bag> bag_;
    ogccdb::flight::FlightTrack track_;
    std::filesystem::path path_;
    std::string error_;
    double ground_offset_ = 0.0;
    int ground_lod_ = -100;
    std::vector<ogccdb::runtime::Vec3d> path_cache_;
    double path_cache_offset_ = 1e300;
};

/// The "Flight" panel section: open a bag, transport, camera, live values and
/// the topics being played. `agl` is the vehicle's height above the terrain.
void flight_ui(FlightPlayer& player, FileBrowser& browser, std::optional<double> agl);
