// The two file formats georay reads: the hand-written JSON parser (built for
// this rather than a library, because the subset needed is a handful of
// numbers), and QGroundControl/Pixhawk mission files - where the altitude
// frame is the part that is silently wrong if misread.

#include "selftest_support.hpp"

#include <algorithm>
#include <cmath>

#include "geo/geolocate.hpp"
#include "geo/json.hpp"
#include "geo/mission.hpp"

namespace georay {

void testJson() {
    std::printf("json reader\n");

    JsonValue value;
    std::string error;

    // Scalars and the basic shapes.
    check(JsonValue::parse("42", value, error) && value.isNumber(), "a bare number parses");
    checkNear(value.number(), 42.0, 1e-12, "...with the right value");
    check(JsonValue::parse("-1.5e3", value, error), "exponent notation parses");
    checkNear(value.number(), -1500.0, 1e-9, "...with the right value");
    check(JsonValue::parse("\"hi\"", value, error) && value.string() == "hi", "a string parses");
    check(JsonValue::parse("true", value, error) && value.boolean(), "true parses");
    check(JsonValue::parse("null", value, error) && value.isNull(), "null parses");
    check(JsonValue::parse("[]", value, error) && value.isArray() && value.size() == 0,
          "an empty array parses");
    check(JsonValue::parse("{}", value, error) && value.isObject() && value.size() == 0,
          "an empty object parses");

    // Nesting, mixed types, whitespace.
    check(JsonValue::parse(" { \"a\" : [1, 2, {\"b\": null}] , \"c\":\"x\" } ", value, error),
          "a nested document parses: " + error);
    check(value["a"].isArray() && value["a"].size() == 3, "array member has the right length");
    checkNear(value["a"].at(1).number(), 2.0, 1e-12, "array indexing");
    check(value["a"].at(2)["b"].isNull(), "nested object member");
    check(value["c"].string() == "x", "string member");

    // Missing things yield defaults rather than exceptions, because a
    // mission file with an unexpected shape should produce a clear message,
    // not a crash three layers down.
    check(value["nope"].isNull(), "a missing key is null");
    check(value["nope"]["deeper"].isNull(), "...and can be indexed into safely");
    checkNear(value["nope"].number(7.0), 7.0, 1e-12, "...and falls back as asked");
    check(value["a"].at(99).isNull(), "an out-of-range index is null");
    checkNear(value["c"].number(-1.0), -1.0, 1e-12, "a string read as a number falls back");

    // Escapes.
    check(JsonValue::parse("\"a\\nb\\t\\\"c\\\"\"", value, error),
          "escapes parse: " + error);
    check(value.string() == "a\nb\t\"c\"", "...and decode correctly");
    check(JsonValue::parse("\"\\u00e4\"", value, error) && value.string() == "\xc3\xa4",
          "a \\u escape becomes UTF-8");

    // Malformed input must be refused. Each of these is something a real
    // half-written or truncated file produces, and each must be an error
    // rather than a partially-built value.
    const char* bad[] = {
        "",  "{",  "}",  "[",  "[1,",  "[1,]",  "{\"a\"}",  "{\"a\":}",  "{a:1}",
        "01",  "-",  "1.",  ".5",  "1e",  "tru",  "\"unterminated",
        "{} trailing",  "[1] [2]",  "\"\\q\"",  "{\"a\":1,}",
    };
    for (const char* text : bad) {
        JsonValue rejected;
        std::string message;
        check(!JsonValue::parse(text, rejected, message),
              std::string("refuses malformed input: '") + text + "'");
        check(!message.empty(), "...with a message");
    }

    // Deep nesting must hit the depth cap rather than the stack.
    {
        std::string deep(500, '[');
        JsonValue rejected;
        std::string message;
        check(!JsonValue::parse(deep, rejected, message), "deeply nested input is refused");
    }
}

void testMission() {
    std::printf("qgroundcontrol / pixhawk missions\n");

    // A QGC .plan as the application actually writes one: relative altitudes,
    // nulls in the unused MAVLink params, a takeoff and a land.
    const std::string plan = R"JSON({
        "fileType": "Plan",
        "geoFence": { "circles": [], "polygons": [], "version": 2 },
        "groundStation": "QGroundControl",
        "mission": {
            "cruiseSpeed": 16,
            "firmwareType": 12,
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "items": [
                { "AMSLAltAboveTerrain": null, "Altitude": 50, "AltitudeMode": 1,
                  "autoContinue": true, "command": 22, "doJumpId": 1, "frame": 3,
                  "params": [15, 0, 0, null, 52.5163, 13.3777, 50], "type": "SimpleItem" },
                { "Altitude": 80, "AltitudeMode": 1, "autoContinue": true,
                  "command": 16, "doJumpId": 2, "frame": 3,
                  "params": [0, 0, 0, null, 52.5200, 13.3800, 80], "type": "SimpleItem" },
                { "Altitude": 80, "AltitudeMode": 1, "autoContinue": true,
                  "command": 16, "doJumpId": 3, "frame": 3,
                  "params": [0, 0, 0, null, 52.5200, 13.3900, 80], "type": "SimpleItem" },
                { "autoContinue": true, "command": 178, "doJumpId": 4, "frame": 2,
                  "params": [1, 12, -1, 0, 0, 0, 0], "type": "SimpleItem" },
                { "Altitude": 0, "AltitudeMode": 1, "autoContinue": true,
                  "command": 21, "doJumpId": 5, "frame": 3,
                  "params": [0, 0, 0, null, 52.5163, 13.3777, 0], "type": "SimpleItem" }
            ],
            "plannedHomePosition": [52.5163, 13.3777, 38.5],
            "vehicleType": 2,
            "version": 2
        },
        "rallyPoints": { "points": [], "version": 2 },
        "version": 1
    })JSON";

    Mission mission;
    std::string error;
    check(parseQgcPlan(plan, mission, error), "a QGC .plan parses: " + error);
    check(mission.items.size() == 5, "every mission item is kept, navigable or not");
    checkNear(mission.cruise_speed_mps, 16.0, 1e-12, "cruise speed is read");
    check(mission.has_home, "the planned home position is read");
    checkNear(mission.home.latitude_deg, 52.5163, 1e-12, "home latitude");
    checkNear(mission.home.altitude_m, 38.5, 1e-12, "home altitude");

    // The speed-change item (command 178) carries no position, and treating
    // it as one would fly the aircraft to latitude 0, longitude 0.
    const std::vector<Waypoint> navigation = mission.navigationItems();
    check(navigation.size() == 4, "non-navigation items are excluded from the route");
    check(navigation[0].command == mav_cmd::takeoff, "takeoff is a navigation item");
    check(navigation[3].command == mav_cmd::land, "land is a navigation item");
    checkNear(navigation[1].latitude_deg, 52.5200, 1e-12, "waypoint latitude from params[4]");
    checkNear(navigation[1].longitude_deg, 13.3800, 1e-12, "waypoint longitude from params[5]");
    checkNear(navigation[1].altitude_m, 80.0, 1e-12, "waypoint altitude from params[6]");

    // Altitude frames. This is the part that is silently wrong if ignored.
    {
        const Geodetic origin(52.5163, 13.3777, 38.5 + 39.5);
        const EnuFrame frame(origin);
        Terrain terrain(Terrain::Kind::Flat, 5000.0, 33, 0.0);
        const double separation = 39.5;

        // MAV_FRAME_GLOBAL_RELATIVE_ALT: 80 m above the home position, whose
        // AMSL altitude is 38.5, so 118.5 AMSL = 158.0 ellipsoidal.
        Waypoint relative = navigation[1];
        relative.frame = mav_frame::global_relative_alt;
        checkNear(mission.resolveAltitude(relative, terrain, frame, separation).altitude_m,
                  38.5 + 39.5 + 80.0, 1e-9, "relative altitude is measured from home");

        // MAV_FRAME_GLOBAL: 80 m above mean sea level, so 119.5 ellipsoidal.
        Waypoint amsl = relative;
        amsl.frame = mav_frame::global;
        checkNear(mission.resolveAltitude(amsl, terrain, frame, separation).altitude_m,
                  80.0 + 39.5, 1e-9, "AMSL altitude only adds the geoid separation");

        // The two differ by the home elevation. Reading one as the other is a
        // 38 m error over Berlin and a lethal one in the mountains.
        check(std::fabs(mission.resolveAltitude(relative, terrain, frame, separation).altitude_m -
                        mission.resolveAltitude(amsl, terrain, frame, separation).altitude_m) >
                  38.0,
              "relative and AMSL frames are not interchangeable");

        // MAV_FRAME_GLOBAL_TERRAIN_ALT: 80 m above the ground below. On flat
        // terrain at ENU 0 that is the frame origin's height plus 80.
        Waypoint agl = relative;
        agl.frame = mav_frame::global_terrain_alt;
        const double got =
            mission.resolveAltitude(agl, terrain, frame, separation).altitude_m;
        checkNear(got, origin.altitude_m + 80.0, 0.1,
                  "terrain-relative altitude is measured from the ground below");
    }

    // The older tab-separated format, which Mission Planner and MAVProxy use
    // and which is what a MAVLink mission stream looks like written down.
    const std::string wpl =
        "QGC WPL 110\n"
        "0\t1\t0\t16\t0\t0\t0\t0\t52.516300\t13.377700\t38.500000\t1\n"
        "1\t0\t3\t22\t15\t0\t0\t0\t52.516300\t13.377700\t50.000000\t1\n"
        "2\t0\t3\t16\t0\t0\t0\t0\t52.520000\t13.380000\t80.000000\t1\n"
        "3\t0\t3\t16\t0\t0\t0\t0\t52.520000\t13.390000\t80.000000\t1\n"
        "4\t0\t3\t21\t0\t0\t0\t0\t52.516300\t13.377700\t0.000000\t1\n";
    Mission wpl_mission;
    check(parseWaypointList(wpl, wpl_mission, error), "a QGC WPL list parses: " + error);
    check(wpl_mission.has_home, "sequence 0 is taken as home, not as a waypoint");
    checkNear(wpl_mission.home.altitude_m, 38.5, 1e-9, "home altitude from sequence 0");
    check(wpl_mission.navigationItems().size() == 4,
          "the remaining items are the route");
    checkNear(wpl_mission.navigationItems()[1].latitude_deg, 52.52, 1e-9,
              "waypoint latitude from column 9");
    checkNear(wpl_mission.navigationItems()[2].longitude_deg, 13.39, 1e-9,
              "waypoint longitude from column 10");

    // Both formats must describe the same flight, which is the strongest
    // check that neither has a column or index off by one.
    const std::vector<Waypoint> a = mission.navigationItems();
    const std::vector<Waypoint> b = wpl_mission.navigationItems();
    check(a.size() == b.size(), "both formats yield the same number of waypoints");
    if (a.size() == b.size()) {
        double worst = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            worst = std::max(worst, std::fabs(a[i].latitude_deg - b[i].latitude_deg));
            worst = std::max(worst, std::fabs(a[i].longitude_deg - b[i].longitude_deg));
            worst = std::max(worst, std::fabs(a[i].altitude_m - b[i].altitude_m));
            check(a[i].command == b[i].command, "...and the same commands");
        }
        checkNear(worst, 0.0, 1e-9, ".plan and .waypoints agree on every position");
    }

    // Bounds, used to frame a scene around a loaded mission.
    double south = 0, north = 0, west = 0, east = 0;
    check(mission.bounds(south, north, west, east), "mission bounds are computed");
    checkNear(south, 52.5163, 1e-9, "southern bound");
    checkNear(north, 52.5200, 1e-9, "northern bound");
    checkNear(east, 13.3900, 1e-9, "eastern bound");

    // Rejections.
    Mission rejected;
    check(!parseQgcPlan("{\"fileType\":\"Fence\"}", rejected, error),
          "a JSON file that is not a Plan is refused");
    check(!parseQgcPlan("{\"fileType\":\"Plan\"}", rejected, error),
          "a Plan with no mission section is refused");
    check(!parseWaypointList("1 2 3\n", rejected, error),
          "a text file without the QGC WPL header is refused");
    check(!parseWaypointList("QGC WPL 110\n1\t0\t3\n", rejected, error),
          "a waypoint line with too few columns is refused");
    // A plan whose only items carry no position is not a flight.
    check(!parseQgcPlan("{\"fileType\":\"Plan\",\"mission\":{\"items\":["
                        "{\"command\":178,\"frame\":2,\"params\":[1,12,-1,0,0,0,0]}]}}",
                        rejected, error),
          "a plan with no navigable waypoints is refused");
}

// writeQgcPlan is the other half of parseQgcPlan, and the one check that
// matters is that the two agree: whatever is written must read back as
// exactly what went in, including the takeoff/land commands the planner
// produces and lat/lon precision tight enough to matter at this tool's scale.
void testWriteQgcPlan() {
    std::printf("writing qgroundcontrol plans\n");

    Mission mission;
    mission.cruise_speed_mps = 12.5;
    mission.has_home = true;
    mission.home = Geodetic(52.516300, 13.377700, 38.5);

    auto item = [](int command, double lat, double lon, double alt_m) {
        Waypoint w;
        w.command = command;
        w.frame = mav_frame::global_relative_alt;
        w.latitude_deg = lat;
        w.longitude_deg = lon;
        w.altitude_m = alt_m;
        w.autocontinue = true;
        return w;
    };
    mission.items.push_back(item(mav_cmd::takeoff, 52.516300, 13.377700, 50.0));
    mission.items.push_back(item(mav_cmd::waypoint, 52.520000, 13.380000, 60.0));
    mission.items.push_back(item(mav_cmd::waypoint, 52.520000, 13.390000, 60.0));
    mission.items.push_back(item(mav_cmd::land, 52.516300, 13.377700, 0.0));

    const std::string directory = makeTempDir();
    check(!directory.empty(), "temporary directory for the plan-writing test");
    if (directory.empty()) return;
    const std::string path = directory + "/round_trip.plan";

    std::string error;
    check(writeQgcPlan(path, mission, error), "a built mission writes: " + error);

    Mission read_back;
    check(loadMission(path, read_back, error), "the written plan loads back: " + error);
    check(read_back.items.size() == mission.items.size(),
          "every item survives the round trip");
    checkNear(read_back.cruise_speed_mps, mission.cruise_speed_mps, 1e-6,
              "cruise speed round-trips");
    check(read_back.has_home, "home position is marked present");
    checkNear(read_back.home.latitude_deg, mission.home.latitude_deg, 1e-7,
              "home latitude round-trips");
    checkNear(read_back.home.altitude_m, mission.home.altitude_m, 1e-6,
              "home altitude round-trips");

    if (read_back.items.size() == mission.items.size()) {
        for (size_t i = 0; i < mission.items.size(); ++i) {
            const Waypoint& a = mission.items[i];
            const Waypoint& b = read_back.items[i];
            check(a.command == b.command, "command round-trips, item " + std::to_string(i));
            check(a.frame == b.frame, "frame round-trips, item " + std::to_string(i));
            checkNear(a.latitude_deg, b.latitude_deg, 1e-7,
                      "latitude round-trips, item " + std::to_string(i));
            checkNear(a.longitude_deg, b.longitude_deg, 1e-7,
                      "longitude round-trips, item " + std::to_string(i));
            checkNear(a.altitude_m, b.altitude_m, 1e-6,
                      "altitude round-trips, item " + std::to_string(i));
        }
    }
    check(read_back.items.front().command == mav_cmd::takeoff,
          "the takeoff item is still a takeoff after the round trip");
    check(read_back.items.back().command == mav_cmd::land,
          "the land item is still a land after the round trip");

    // Writing with nothing to write is refused rather than producing a plan
    // with no route - a mission file that loads but goes nowhere.
    Mission empty_mission;
    check(!writeQgcPlan(directory + "/empty.plan", empty_mission, error),
          "writing an empty mission is refused");
}

}  // namespace georay
