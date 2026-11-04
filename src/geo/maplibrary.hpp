#pragma once

// A directory of map rasters, streamed by position.
//
// One stitched raster cannot cover a region: Brandenburg at the resolution a
// UAV camera actually resolves is millions of tiles and over a hundred
// gigabytes, and a single GPU texture over that area tops out around 15 m per
// pixel however large it is. So the imagery is cut into windows a few
// kilometres across, and georay holds the ones near the aircraft.
//
// Windows are plain .map files written by scripts/fetch_map.py. Their
// geographic bounds come from the files themselves, so a library is just a
// directory - add a window by fetching one, remove it by deleting it.
//
// What this does not do is make the whole region appear. Imagery has to have
// been fetched to be shown, and fetching a whole Land tile-by-tile is both
// enormous and abusive to a tile service. The practical workflow is to fetch
// the corridor a flight actually covers (see fetch_map.py --along), which is
// a thousandth of the data for the same sortie.

#include <memory>
#include <string>
#include <vector>

#include "geo/geodetic.hpp"
#include "geo/maptile.hpp"

namespace georay {

class MapLibrary {
public:
    // Scans `directory` for .map files and reads each one's header. Only the
    // headers: a window is a few tens of megabytes and a library may hold
    // dozens, so the pixels wait until something needs them.
    bool open(const std::string& directory, std::string& error);

    bool isOpen() const { return !directory_.empty(); }
    const std::string& directory() const { return directory_; }
    size_t windowCount() const { return windows_.size(); }

    // The window covering a position, loading it if necessary, or null if
    // none does. The returned raster stays valid until a later call evicts
    // it, which is why callers hold it only for as long as they are using it.
    const MapRaster* windowFor(double lat_deg, double lon_deg);

    // Name of the window currently resident, for the HUD.
    const std::string& residentName() const { return resident_name_; }
    const std::string& lastError() const { return last_error_; }

    // How many windows have been read from disk since opening. A diagnostic:
    // if this climbs every few seconds, the windows are too small for the
    // speed being flown.
    int loadCount() const { return load_count_; }

private:
    struct Window {
        std::string path;
        std::string name;
        // Bounds read from the header, so a position can be matched without
        // touching the pixels.
        double south = 0.0, north = 0.0, west = 0.0, east = 0.0;
        bool contains(double lat, double lon) const {
            return lat >= south && lat <= north && lon >= west && lon <= east;
        }
        // Squared degrees from the window's centre, for picking the best
        // match when windows overlap - the one whose middle you are nearest
        // keeps the most context around you.
        double centreOffsetSq(double lat, double lon) const {
            const double dy = lat - 0.5 * (south + north);
            const double dx = lon - 0.5 * (west + east);
            return dy * dy + dx * dx;
        }
    };

    std::string directory_;
    std::vector<Window> windows_;

    // Exactly one window resident. More would mean an atlas or a texture
    // array and a redirect table in the shader; one window plus a re-anchor
    // that rebuilds the terrain mesh anyway is the same machinery for a
    // fraction of the complexity.
    std::unique_ptr<MapRaster> resident_;
    std::string resident_path_;
    std::string resident_name_;
    std::string last_error_;
    int load_count_ = 0;
};

}  // namespace georay
