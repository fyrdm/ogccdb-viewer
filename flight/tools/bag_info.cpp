// ogccdb-bag-info: lists a ROS 2 bag's topics and the flight track built from it.
#include <cstdio>
#include <cstdlib>
#include <exception>

#include "ogccdb/flight/bag.hpp"
#include "ogccdb/flight/track.hpp"

using namespace ogccdb::flight;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::puts("usage: ogccdb-bag-info <bag folder | .mcap | .db3> [seconds...]");
        return 2;
    }
    try {
        auto bag = Bag::open(argv[1]);
        std::printf("%s: %s storage, %.1f s, %llu messages\n", argv[1], bag->storage().c_str(),
                    (bag->end_ns() - bag->start_ns()) * 1e-9, static_cast<unsigned long long>(bag->message_count()));
        for (const BagTopic& t : bag->topics()) {
            std::printf("  %-40s %-40s %8llu\n", t.name.c_str(), t.type.c_str(), static_cast<unsigned long long>(t.count));
        }
        const FlightTrack track = FlightTrack::build(*bag);
        const TrackInfo& i = track.info();
        std::printf("track: %.1f..%.1f s, position from %s\n", track.start(), track.end(), i.position_source.c_str());
        std::printf("  fixes %s (%zu), local %s (%zu), attitude %s (%zu)\n", i.fix_topic.c_str(), i.fixes,
                    i.local_topic.c_str(), i.local_poses, i.attitude_topic.c_str(), i.attitudes);
        if (!i.local_topic.empty()) {
            std::printf("  local origin %.7f, %.7f, %.2f m; fixes vs local track RMS %.2f m\n", i.origin.lat,
                        i.origin.lon, i.origin.height, i.anchor_rms);
        }
        for (int a = 2; a < argc || a == 2; ++a) {
            const int steps = 10;
            if (argc > 2) {
                const FlightPose p = track.at(std::atof(argv[a]));
                std::printf("  t %6.1f  %.7f %.7f %8.2f m  hdg %5.1f pitch %5.1f roll %5.1f  gs %4.1f climb %5.1f\n", p.t,
                            p.geodetic.lat, p.geodetic.lon, p.geodetic.height, p.yaw, p.pitch, p.roll, p.ground_speed, p.climb);
                continue;
            }
            for (int s = 0; s <= steps; ++s) {
                const FlightPose p = track.at(track.start() + track.duration() * s / steps);
                std::printf("  t %6.1f  %.7f %.7f %8.2f m  hdg %5.1f pitch %5.1f roll %5.1f  gs %4.1f climb %5.1f\n", p.t,
                            p.geodetic.lat, p.geodetic.lon, p.geodetic.height, p.yaw, p.pitch, p.roll, p.ground_speed, p.climb);
            }
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ogccdb-bag-info: %s\n", e.what());
        return 1;
    }
    return 0;
}
