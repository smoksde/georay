#include "geo/mission.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "geo/json.hpp"

namespace georay {
namespace {

// A plausible-position check. A mission file with latitude 0, longitude 0 is
// almost always an unset field rather than a sortie in the Gulf of Guinea,
// and flying to it would silently relocate the whole scene.
bool plausiblePosition(double lat, double lon) {
    if (!std::isfinite(lat) || !std::isfinite(lon)) return false;
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return false;
    return std::fabs(lat) > 1e-7 || std::fabs(lon) > 1e-7;
}

bool frameIsRelative(int frame) {
    return frame == mav_frame::global_relative_alt || frame == mav_frame::global_relative_alt_int;
}
bool frameIsTerrain(int frame) {
    return frame == mav_frame::global_terrain_alt || frame == mav_frame::global_terrain_alt_int;
}

}  // namespace

bool Waypoint::isNavigation() const {
    switch (command) {
        case mav_cmd::waypoint:
        case mav_cmd::loiter_unlimited:
        case mav_cmd::loiter_turns:
        case mav_cmd::loiter_time:
        case mav_cmd::land:
        case mav_cmd::takeoff:
            return plausiblePosition(latitude_deg, longitude_deg);
        default:
            // Everything else - camera triggers, speed changes, jumps,
            // return-to-launch - carries no usable position.
            return false;
    }
}

const char* Waypoint::commandName() const {
    switch (command) {
        case mav_cmd::waypoint: return "WAYPOINT";
        case mav_cmd::loiter_unlimited: return "LOITER";
        case mav_cmd::loiter_turns: return "LOITER TURNS";
        case mav_cmd::loiter_time: return "LOITER TIME";
        case mav_cmd::return_to_launch: return "RTL";
        case mav_cmd::land: return "LAND";
        case mav_cmd::takeoff: return "TAKEOFF";
        default: return "OTHER";
    }
}

std::vector<Waypoint> Mission::navigationItems() const {
    std::vector<Waypoint> out;
    for (const Waypoint& item : items) {
        if (item.isNavigation()) out.push_back(item);
    }
    return out;
}

Geodetic Mission::resolveAltitude(const Waypoint& waypoint, const Terrain& terrain,
                                  const EnuFrame& frame, double geoid_separation_m) const {
    Geodetic out(waypoint.latitude_deg, waypoint.longitude_deg, 0.0);

    if (frameIsTerrain(waypoint.frame)) {
        // Height above the ground directly below. The terrain is in ENU, so
        // the sum is done there and converted once.
        const glm::dvec3 here = frame.fromGeodetic(Geodetic(out.latitude_deg,
                                                            out.longitude_deg, 0.0));
        const double ground_enu = terrain.sampleHeight(here.x, here.y);
        const glm::dvec3 target(here.x, here.y, ground_enu + waypoint.altitude_m);
        return frame.toGeodetic(target);
    }

    if (frameIsRelative(waypoint.frame)) {
        // Height above the home position. The home altitude in a .plan is
        // AMSL, so the geoid separation turns it into the ellipsoidal height
        // the rest of this tool works in.
        const double home_ellipsoidal =
            has_home ? home.altitude_m + geoid_separation_m : frame.origin().altitude_m;
        out.altitude_m = home_ellipsoidal + waypoint.altitude_m;
        return out;
    }

    // MAV_FRAME_GLOBAL: above mean sea level, i.e. orthometric.
    out.altitude_m = waypoint.altitude_m + geoid_separation_m;
    return out;
}

bool Mission::bounds(double& south, double& north, double& west, double& east) const {
    bool any = false;
    for (const Waypoint& item : items) {
        if (!item.isNavigation()) continue;
        if (!any) {
            south = north = item.latitude_deg;
            west = east = item.longitude_deg;
            any = true;
            continue;
        }
        south = std::min(south, item.latitude_deg);
        north = std::max(north, item.latitude_deg);
        west = std::min(west, item.longitude_deg);
        east = std::max(east, item.longitude_deg);
    }
    return any;
}

bool parseQgcPlan(const std::string& text, Mission& out, std::string& error) {
    JsonValue root;
    if (!JsonValue::parse(text, root, error)) return false;

    if (!root.isObject()) {
        error = "not a QGroundControl plan (top level is not an object)";
        return false;
    }
    // fileType is how QGC identifies its own files. Checking it means a JSON
    // document of some other kind fails with a useful message rather than
    // producing an empty mission.
    if (root.has("fileType") && root["fileType"].string() != "Plan") {
        error = "JSON file is not a QGroundControl Plan (fileType is '" +
                root["fileType"].string() + "')";
        return false;
    }

    const JsonValue& mission = root["mission"];
    if (!mission.isObject()) {
        error = "plan has no mission section";
        return false;
    }

    Mission parsed;
    parsed.cruise_speed_mps = mission["cruiseSpeed"].number(0.0);

    const JsonValue& home = mission["plannedHomePosition"];
    if (home.isArray() && home.size() >= 3) {
        const double lat = home.at(0).number();
        const double lon = home.at(1).number();
        if (plausiblePosition(lat, lon)) {
            parsed.has_home = true;
            parsed.home = Geodetic(lat, lon, home.at(2).number());
        }
    }

    const JsonValue& items = mission["items"];
    if (!items.isArray()) {
        error = "plan's mission has no item list";
        return false;
    }

    for (size_t i = 0; i < items.size(); ++i) {
        const JsonValue& item = items.at(i);
        if (!item.isObject()) continue;

        // A survey or corridor scan appears as a ComplexItem, whose expanded
        // waypoints QGC computes itself. Skipping it with a note is honest;
        // silently dropping it would make a plan look shorter than it is.
        const std::string type = item["type"].string();
        if (type == "ComplexItem") {
            parsed.name += parsed.name.empty() ? "" : " ";
            continue;
        }

        Waypoint waypoint;
        waypoint.sequence = int(item["doJumpId"].number(double(i)));
        waypoint.command = int(item["command"].number(double(mav_cmd::waypoint)));
        waypoint.frame = int(item["frame"].number(double(mav_frame::global_relative_alt)));
        waypoint.autocontinue = item["autoContinue"].boolean(true);

        const JsonValue& params = item["params"];
        // MAVLink's seven params: 1-4 are command-specific, 5-7 are
        // latitude, longitude and altitude. QGC writes nulls for unused ones,
        // which read back as 0 here, which is what they mean.
        for (int p = 0; p < 4; ++p) waypoint.params[p] = params.at(size_t(p)).number(0.0);
        waypoint.latitude_deg = params.at(4).number(0.0);
        waypoint.longitude_deg = params.at(5).number(0.0);
        waypoint.altitude_m = params.at(6).number(0.0);

        // QGC also stores Altitude/AltitudeMode alongside params; when the
        // params altitude is absent, fall back to it.
        if (waypoint.altitude_m == 0.0 && item.has("Altitude")) {
            waypoint.altitude_m = item["Altitude"].number(0.0);
        }

        parsed.items.push_back(waypoint);
    }

    if (parsed.navigationItems().empty()) {
        error = "plan contains no navigable waypoints";
        return false;
    }

    out = std::move(parsed);
    return true;
}

bool parseWaypointList(const std::string& text, Mission& out, std::string& error) {
    std::istringstream stream(text);
    std::string line;

    if (!std::getline(stream, line)) {
        error = "empty waypoint file";
        return false;
    }
    // The magic line is "QGC WPL <version>". Versions 110 and 120 exist and
    // have the same column layout.
    if (line.rfind("QGC WPL", 0) != 0) {
        error = "not a QGC waypoint list (missing the 'QGC WPL' header)";
        return false;
    }

    Mission parsed;
    int line_number = 1;
    while (std::getline(stream, line)) {
        ++line_number;
        if (line.empty() || line[0] == '#') continue;

        // Columns: seq, current, frame, command, p1..p4, x, y, z, autocontinue.
        // Whitespace-separated rather than strictly tab-separated, because
        // files edited by hand routinely end up with spaces.
        std::istringstream fields(line);
        double column[12];
        int read = 0;
        for (; read < 12; ++read) {
            if (!(fields >> column[read])) break;
        }
        if (read < 12) {
            error = "line " + std::to_string(line_number) + ": expected 12 columns, found " +
                    std::to_string(read);
            return false;
        }

        Waypoint waypoint;
        waypoint.sequence = int(column[0]);
        waypoint.frame = int(column[2]);
        waypoint.command = int(column[3]);
        for (int p = 0; p < 4; ++p) waypoint.params[p] = column[4 + p];
        waypoint.latitude_deg = column[8];
        waypoint.longitude_deg = column[9];
        waypoint.altitude_m = column[10];
        waypoint.autocontinue = column[11] != 0.0;

        // Sequence 0 is the home position by convention, not a waypoint to
        // fly to. It is kept as the home fix and left out of the route.
        if (waypoint.sequence == 0 && plausiblePosition(waypoint.latitude_deg,
                                                        waypoint.longitude_deg)) {
            parsed.has_home = true;
            parsed.home = Geodetic(waypoint.latitude_deg, waypoint.longitude_deg,
                                   waypoint.altitude_m);
            continue;
        }
        parsed.items.push_back(waypoint);
    }

    if (parsed.navigationItems().empty()) {
        error = "waypoint file contains no navigable waypoints";
        return false;
    }

    out = std::move(parsed);
    return true;
}

bool loadMission(const std::string& path, Mission& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open mission " + path;
        return false;
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    // Chosen by content, not by extension: a .plan renamed to .waypoints
    // should still load, and guessing from the name is how a tool ends up
    // reporting "malformed JSON" about a tab-separated file.
    const size_t first = text.find_first_not_of(" \t\r\n");
    const bool looks_like_json = first != std::string::npos && text[first] == '{';

    const bool ok = looks_like_json ? parseQgcPlan(text, out, error)
                                    : parseWaypointList(text, out, error);
    if (!ok) {
        error = path + ": " + error;
        return false;
    }

    out.source_path = path;
    if (out.name.empty()) {
        const size_t slash = path.find_last_of("/\\");
        out.name = slash == std::string::npos ? path : path.substr(slash + 1);
    }
    return true;
}

}  // namespace georay
