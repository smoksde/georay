// georay - a prototype for locating where on Earth a UAV camera pixel lands.
//
// See README.md for what it does and the controls; src/geo/geolocate.hpp for
// the pipeline this is all built around.

#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "app.hpp"
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
        "  --scene NAME                  load scenes/NAME.scene (try: berlin, munich,\n"
        "                                zugspitze). Flags after it override its settings.\n"
        "  --origin LAT,LON              scene centre (default 48.137554,11.575497)\n"
        "  --extent METRES               half-extent of the terrain (default 3000)\n"
        "  --map FILE                    map raster from scripts/fetch_map.py\n"
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
        "  scripts/fetch_map.py --origin 52.5163,13.3777 --extent 2000 "
        "--out data/berlin.map\n"
        "  ./build/georay --scene berlin\n"
        "\n"
        "  georay --help                 this message\n";
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
            options.map_path = argv[++i];
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

    // The complaint this answers: running georay bare shows synthetic terrain,
    // and nothing on screen tells you real data is even possible. Say so once,
    // here, where someone who has just built it will see it.
    if (options.dem_directory.empty() && options.map_path.empty()) {
        // Flushed explicitly: stdout is fully buffered when piped, and this
        // is precisely the line someone who has just built the thing needs
        // to see - losing it to a buffer when the window is closed would
        // defeat the point of printing it.
        std::cout << "georay: synthetic terrain. For real terrain and maps try "
                     "--scene berlin\n"
                     "        (or --help; scenes/ lists what is available).\n"
                  << std::flush;
    }

    georay::App app;
    if (!app.initialise(options)) return 1;
    return app.run();
}
