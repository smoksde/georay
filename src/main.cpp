// georay - a prototype for locating where on Earth a UAV camera pixel lands.
//
// See README.md for what it does and the controls; src/geo/geolocate.hpp for
// the pipeline this is all built around.

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "app.hpp"
#include "geo/dem.hpp"
#include "selftest.hpp"

namespace {

void printUsage() {
    std::cout <<
        "georay - UAV camera pixel geolocation prototype\n"
        "\n"
        "  georay                        open the simulation window\n"
        "  georay --selftest             run the numeric verification suite and exit\n"
        "  georay --screenshot FILE      render a few frames, save a BMP, exit\n"
        "  georay --size WxH             window size (default 1600x900)\n"
        "\n"
        "Scene:\n"
        "  --scene NAME                  load scenes/NAME.scene. With no scene and no\n"
        "                                data flags, scenes/default.scene is used when its\n"
        "                                data is present. Try: berlin-satellite, berlin,\n"
        "                                munich, zugspitze. Later flags override a scene.\n"
        "  --origin LAT,LON              scene centre (default 48.137554,11.575497)\n"
        "  --extent METRES               half-extent of the terrain (default 3000)\n"
        "  --map SOURCE                  tile source for the satellite layer\n"
        "  --street-map SOURCE           tile source for the drawn-map layer; M switches\n"
        "  --mission FILE                a QGroundControl .plan or QGC WPL .waypoints\n"
        "                                list to fly\n"
        "  --map-mode MODE               satellite (default), street, or off\n"
        "  --tiles DIR                   tile cache directory (default data/tiles)\n"
        "  --online                      fetch missing tiles while flying\n"
        "  --advance SECONDS             run the simulation forward before drawing\n"
        "\n"
        "Real elevation data (optional; synthetic terrain is used without it):\n"
        "  --dem DIR                     directory of SRTM .hgt tiles\n"
        "  --geoid-separation METRES     geoid height for the scene, e.g. 47 in\n"
        "                                southern Germany. SRTM heights are above the\n"
        "                                geoid and georay works above the ellipsoid, so\n"
        "                                without this every elevation is off by that much.\n"
        "  --geoid FILE                  a geoid grid from scripts/fetch_dem.py, instead\n"
        "                                of a single separation\n"
        "\n"
        "Fetch data for a scene with:\n"
        "  scripts/fetch_dem.py --origin 52.5163,13.3777 --extent 2000\n"
        "  scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 --zoom 17\n"
        "  ./build/georay --scene berlin\n"
        "\n"
        "  georay --help                 this message\n";
}

// Whether a scene's data files are actually on disk. Used to decide if the
// default scene can be opened, so a checkout that has never run a fetch
// script still starts rather than erroring on a missing file.
bool dataPresent(const georay::App::Options& options) {
    // Only elevation has to be on disk. Map tiles are optional - they come
    // from the cache or, with --online, from the network - and a scene with
    // terrain and no map is still a usable scene.
    if (options.dem_directory.empty()) return true;
    // A directory with no tile covering the origin is as good as absent.
    const std::string tile =
        georay::HgtTile::tileNameFor(options.origin_latitude_deg, options.origin_longitude_deg);
    return std::ifstream(options.dem_directory + "/" + tile + ".hgt").good();
}

// Parses "LAT,LON". Returns false on anything it cannot read, rather than
// quietly defaulting - a mistyped origin that silently becomes 0,0 would put
// the scene in the Atlantic with no explanation.
bool parseOrigin(const char* text, double& latitude, double& longitude) {
    char extra = 0;
    if (std::sscanf(text, "%lf,%lf%c", &latitude, &longitude, &extra) != 2) return false;
    return latitude >= -90.0 && latitude <= 90.0 && longitude >= -180.0 && longitude <= 180.0;
}

}  // namespace

int main(int argc, char** argv) {
    georay::App::Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
        if (arg == "--selftest") {
            // Deliberately before any window or GL setup: the maths must be
            // testable on a machine with no display at all.
            return georay::runSelfTest();
        }
        if (arg == "--screenshot" && i + 1 < argc) {
            options.screenshot_path = argv[++i];
            continue;
        }
        if (arg == "--scene" && i + 1 < argc) {
            // Resolved relative to the project's scenes/ directory, which is
            // baked in at build time for the same reason the shaders are: a
            // build directory anywhere still finds them.
            const std::string name = argv[++i];
            const std::string path = name.find('/') != std::string::npos
                                         ? name
                                         : std::string(GEORAY_SCENE_DIR) + "/" + name + ".scene";
            std::string error;
            if (!georay::App::Options::loadScene(path, options, error)) {
                std::cerr << "georay: " << error << "\n";
                return 2;
            }
            continue;
        }
        if (arg == "--map" && i + 1 < argc) {
            options.satellite_source = argv[++i];
            continue;
        }
        if (arg == "--street-map" && i + 1 < argc) {
            options.street_source = argv[++i];
            continue;
        }
        if (arg == "--mission" && i + 1 < argc) {
            options.mission_path = argv[++i];
            continue;
        }
        if (arg == "--advance" && i + 1 < argc) {
            char extra = 0;
            if (std::sscanf(argv[++i], "%lf%c", &options.advance_seconds, &extra) != 1 ||
                options.advance_seconds < 0.0 || options.advance_seconds > 86400.0) {
                std::cerr << "georay: --advance wants seconds (0 to 86400)\n";
                return 2;
            }
            continue;
        }
        if (arg == "--online") {
            options.online = true;
            continue;
        }
        if (arg == "--tile-limit" && i + 1 < argc) {
            char extra = 0;
            if (std::sscanf(argv[++i], "%d%c", &options.online_tile_limit, &extra) != 1 ||
                options.online_tile_limit < 1) {
                std::cerr << "georay: --tile-limit wants a positive count\n";
                return 2;
            }
            continue;
        }
        if (arg == "--tiles" && i + 1 < argc) {
            options.tile_directory = argv[++i];
            continue;
        }
        if (arg == "--map-mode" && i + 1 < argc) {
            options.map_mode = argv[++i];
            if (options.map_mode != "satellite" && options.map_mode != "street" &&
                options.map_mode != "off") {
                std::cerr << "georay: --map-mode wants satellite, street or off\n";
                return 2;
            }
            continue;
        }
        if (arg == "--dem" && i + 1 < argc) {
            options.dem_directory = argv[++i];
            continue;
        }
        if (arg == "--geoid" && i + 1 < argc) {
            options.geoid_grid_path = argv[++i];
            continue;
        }
        if (arg == "--geoid-separation" && i + 1 < argc) {
            char extra = 0;
            if (std::sscanf(argv[++i], "%lf%c", &options.geoid_separation_m, &extra) != 1) {
                std::cerr << "georay: --geoid-separation wants a number of metres\n";
                return 2;
            }
            options.geoid_separation_given = true;
            continue;
        }
        if (arg == "--origin" && i + 1 < argc) {
            if (!parseOrigin(argv[++i], options.origin_latitude_deg,
                             options.origin_longitude_deg)) {
                std::cerr << "georay: --origin wants LAT,LON, e.g. 48.1375,11.5755\n";
                return 2;
            }
            continue;
        }
        if (arg == "--extent" && i + 1 < argc) {
            char extra = 0;
            double extent = 0.0;
            if (std::sscanf(argv[++i], "%lf%c", &extent, &extra) != 1 || extent < 100.0 ||
                extent > 200000.0) {
                std::cerr << "georay: --extent wants a half-extent in metres (100 to 200000)\n";
                return 2;
            }
            options.half_extent_m = extent;
            continue;
        }
        if (arg == "--size" && i + 1 < argc) {
            int w = 0, h = 0;
            if (std::sscanf(argv[++i], "%dx%d", &w, &h) == 2 && w > 320 && h > 240) {
                options.window_width = w;
                options.window_height = h;
            } else {
                std::cerr << "georay: --size wants WxH, e.g. 1920x1080\n";
                return 2;
            }
            continue;
        }

        std::cerr << "georay: unknown argument '" << arg << "'\n\n";
        printUsage();
        return 2;
    }

    // With nothing asked for, open the default scene rather than synthetic
    // terrain. Real data being opt-in behind four flags meant that running
    // georay showed generated hills and nothing said otherwise - a tool whose
    // whole point is real coordinates should not open on a fiction.
    //
    // Checked for existence first, so a fresh checkout with no data still
    // runs instead of failing on a file nobody has fetched yet.
    if (options.dem_directory.empty() &&
        options.scene_name.empty()) {
        const std::string path = std::string(GEORAY_SCENE_DIR) + "/default.scene";
        std::string error;
        georay::App::Options candidate = options;
        if (georay::App::Options::loadScene(path, candidate, error) &&
            dataPresent(candidate)) {
            options = candidate;
        } else {
            // Flushed explicitly: stdout is fully buffered when piped, and
            // this is precisely the line someone who has just built the thing
            // needs to see.
            std::cout << "georay: no scene data found, using synthetic terrain.\n"
                         "        For real terrain and satellite imagery:\n"
                         "          scripts/fetch_dem.py --origin 52.5163,13.3777 "
                         "--extent 1200\n"
                         "        or just add --online and georay will fetch the map "
                         "itself.\n"
                      << std::flush;
        }
    }

    georay::App app;
    if (!app.initialise(options)) return 1;
    return app.run();
}
