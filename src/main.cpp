// georay - a prototype for locating where on Earth a UAV camera pixel lands.
//
// See README.md for what it does and the controls; src/geo/geolocate.hpp for
// the pipeline this is all built around.

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
        "  georay --help                 this message\n";
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

    georay::App app;
    if (!app.initialise(options)) return 1;
    return app.run();
}
