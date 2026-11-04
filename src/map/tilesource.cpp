#include "map/tilesource.hpp"

#include <vector>

namespace georay {
namespace {

// Zoom ceilings are what each service actually serves, not what it would be
// nice to have. Asking past them returns 404s, which look like a network
// fault rather than the end of the data.
const std::vector<TileSource>& sources() {
    static const std::vector<TileSource> list = {
        {"esri",
         "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery"
         "/MapServer/tile/{z}/{y}/{x}",
         "IMAGERY (C) ESRI, MAXAR, EARTHSTAR GEOGRAPHICS", 0, 19, true},
        {"basemapde",
         "https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0"
         "/de_basemapde_web_raster_grau/default/GLOBAL_WEBMERCATOR/{z}/{y}/{x}.png",
         "(C) GEOBASIS-DE / BKG (DL-DE/BY-2-0)", 0, 18, false},
        {"basemapde-colour",
         "https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0"
         "/de_basemapde_web_raster_farbe/default/GLOBAL_WEBMERCATOR/{z}/{y}/{x}.png",
         "(C) GEOBASIS-DE / BKG (DL-DE/BY-2-0)", 0, 18, false},
        {"osm", "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
         "(C) OPENSTREETMAP CONTRIBUTORS", 0, 19, false},
    };
    return list;
}

void substitute(std::string& text, const std::string& token, int value) {
    const size_t at = text.find(token);
    if (at != std::string::npos) text.replace(at, token.size(), std::to_string(value));
}

}  // namespace

std::string TileSource::urlFor(int zoom, int x, int y) const {
    std::string url = url_template;
    substitute(url, "{z}", zoom);
    substitute(url, "{x}", x);
    substitute(url, "{y}", y);
    return url;
}

const TileSource* findTileSource(const std::string& name) {
    for (const TileSource& source : sources()) {
        if (source.name == name) return &source;
    }
    return nullptr;
}

std::string tileSourceNames() {
    std::string out;
    for (const TileSource& source : sources()) {
        if (!out.empty()) out += ", ";
        out += source.name;
    }
    return out;
}

}  // namespace georay
