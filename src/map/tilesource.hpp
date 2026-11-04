#pragma once

// Where map tiles come from, and under what terms - kept together so a URL
// change cannot drift from its credit, which georay displays as given here.

#include <string>

namespace georay {

struct TileSource {
    std::string name;          // short id used on the command line and in paths
    std::string url_template;  // with {z}, {x}, {y} placeholders
    std::string attribution;   // displayed whenever the layer is visible
    int min_zoom = 0;
    int max_zoom = 19;
    bool imagery = false;      // a photograph rather than drawn cartography

    bool valid() const { return !url_template.empty(); }
    // Substitutes the placeholders. Note that sources differ in their
    // ordering - ESRI serves z/y/x - which is why the template uses names.
    std::string urlFor(int zoom, int x, int y) const;
};

// The sources georay knows by name. Matches scripts/fetch_map.py, so the
// disk cache is shared between the script and the live fetcher.
const TileSource* findTileSource(const std::string& name);
// All of them, for --help and error messages.
std::string tileSourceNames();

}  // namespace georay
