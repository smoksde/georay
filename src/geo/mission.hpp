#pragma once

// Flight plans as QGroundControl and a Pixhawk actually exchange them.
//
// Two formats, because both are in daily use:
//
//   .plan       QGroundControl's native JSON. Carries the planned home
//               position, cruise speed, and a list of mission items.
//   .waypoints  The older "QGC WPL 110" tab-separated text, which Mission
//               Planner, MAVProxy and QGC all still read and write, and which
//               is what a MAVLink MISSION_ITEM stream looks like written down.
//
// The part that repays attention is the altitude frame. A waypoint's z is
// meaningless without knowing what it is measured from, and the three frames
// in common use differ by tens of metres over any real terrain:
//
//   MAV_FRAME_GLOBAL (0)              above mean sea level
//   MAV_FRAME_GLOBAL_RELATIVE_ALT (3) above the home position - the default
//                                     in QGC, and what most plans contain
//   MAV_FRAME_GLOBAL_TERRAIN_ALT (10) above the ground directly below
//
// Reading a relative altitude as AMSL puts the aircraft at the home
// elevation plus the height, which over Berlin is a 38 m error and over an
// alpine site a lethal one. resolveAltitude() below is where that is handled.

#include <string>
#include <vector>

#include "geo/geodetic.hpp"
#include "geo/terrain.hpp"

namespace georay {

// The MAVLink command ids this tool cares about. Others are kept but ignored
// for navigation.
namespace mav_cmd {
constexpr int waypoint = 16;       // NAV_WAYPOINT
constexpr int loiter_unlimited = 17;
constexpr int loiter_turns = 18;
constexpr int loiter_time = 19;
constexpr int return_to_launch = 20;
constexpr int land = 21;           // NAV_LAND
constexpr int takeoff = 22;        // NAV_TAKEOFF
}  // namespace mav_cmd

namespace mav_frame {
constexpr int global = 0;
constexpr int global_relative_alt = 3;
constexpr int global_terrain_alt = 10;
constexpr int global_int = 5;
constexpr int global_relative_alt_int = 6;
constexpr int global_terrain_alt_int = 11;
}  // namespace mav_frame

struct Waypoint {
    int sequence = 0;
    int command = mav_cmd::waypoint;
    int frame = mav_frame::global_relative_alt;
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;   // as written in the file, in `frame`
    double params[4] = {0, 0, 0, 0};
    bool autocontinue = true;

    // Whether this item moves the aircraft somewhere. A mission is full of
    // items that do not - camera triggers, speed changes, jumps - and
    // treating one of those as a position flies to latitude 0, longitude 0.
    bool isNavigation() const;
    // Short label for the HUD.
    const char* commandName() const;
};

struct Mission {
    std::string name;
    std::string source_path;
    bool has_home = false;
    Geodetic home;             // planned home position; altitude is AMSL
    double cruise_speed_mps = 0.0;
    std::vector<Waypoint> items;

    // Navigation items only, in order.
    std::vector<Waypoint> navigationItems() const;

    // The waypoint's position with its altitude resolved to a real
    // ellipsoidal height, given the ground model. This is the function that
    // turns "50" into a height above the ellipsoid, and it needs both the
    // frame and the terrain to do it.
    Geodetic resolveAltitude(const Waypoint& waypoint, const Terrain& terrain,
                             const EnuFrame& frame, double geoid_separation_m) const;

    // Bounding box of the navigation items, for framing a scene around a
    // loaded mission.
    bool bounds(double& south, double& north, double& west, double& east) const;

    bool empty() const { return items.empty(); }
};

// Loads either format, chosen by content rather than by extension: a .plan
// renamed to .waypoints should still work, and guessing from the name is how
// a tool ends up reporting "malformed JSON" about a tab-separated file.
bool loadMission(const std::string& path, Mission& out, std::string& error);

// Exposed separately so the tests can drive them directly.
bool parseQgcPlan(const std::string& text, Mission& out, std::string& error);
bool parseWaypointList(const std::string& text, Mission& out, std::string& error);

}  // namespace georay
