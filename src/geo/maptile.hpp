#pragma once

// A map raster to drape over the terrain: real streets and buildings under
// the simulated camera, so a geolocated pixel can be checked against a place
// you can name.
//
// Written by scripts/fetch_map.py, which does the HTTP, the PNG decoding and
// the stitching. What lands here is one file: a header and a block of RGB
// bytes. That is the same split the geoid converter uses - messy third-party
// formats are Python's problem at fetch time, and the C++ side reads
// something it cannot get wrong.
//
// The one piece of real geometry this file does own is Web Mercator. Map
// tiles are not in latitude and longitude; their y axis is the Mercator
// projection, which is decidedly non-linear in latitude. Interpolating in
// latitude instead would misplace the map by over a metre on the ground
// across a 4 km scene - small enough to look fine and large enough to matter
// for a tool that reports positions in metres.

#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace georay {

// Everything a raster's header carries, without its pixels.
//
// A library of windows needs each one's bounds to decide which covers a
// position, and nothing else. Reading the pixels for that would mean a
// gigabyte of disk traffic at start-up over a region-sized library, to
// answer a question the first 172 bytes of each file already contain.
struct MapRasterInfo {
    int zoom = 0;
    int tile_x = 0, tile_y = 0, tiles_x = 0, tiles_y = 0;
    int width = 0, height = 0;
    std::string attribution;

    double northEdge() const;
    double southEdge() const;
    double westEdge() const;
    double eastEdge() const;
};

class MapRaster {
public:
    static bool load(const std::string& path, MapRaster& out, std::string& error);
    // Header only: bounds, zoom and attribution, without touching the pixels.
    static bool readHeader(const std::string& path, MapRasterInfo& out, std::string& error);

    bool valid() const { return width_ > 0 && height_ > 0; }
    int width() const { return width_; }
    int height() const { return height_; }
    const std::vector<unsigned char>& pixels() const { return pixels_; }  // RGB, row 0 north

    // Credit line the tile source requires. Carried inside the raster rather
    // than hard-coded, because whoever fetched it chose the source and the
    // terms came with it; georay displays whatever this says.
    const std::string& attribution() const { return attribution_; }

    // Texture coordinates for a geodetic position. u runs west to east, v
    // north to south, matching the raster's row order. Values outside 0..1
    // mean the position is off the map, which the caller may still want -
    // clamping here would silently smear the edge pixels across the
    // surrounding terrain.
    glm::dvec2 uvFor(double lat_deg, double lon_deg) const;

    bool covers(double lat_deg, double lon_deg) const {
        const glm::dvec2 uv = uvFor(lat_deg, lon_deg);
        return uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0;
    }

    // Ground resolution at a latitude, for deciding whether the map can
    // usefully support the camera's own ground sample distance.
    double metresPerPixel(double lat_deg) const;

    int zoom() const { return zoom_; }
    // Geographic bounds, for reporting. The latitude edges come back through
    // the inverse Mercator transform, so they are exact rather than assumed.
    double northEdge() const;
    double southEdge() const;
    double westEdge() const;
    double eastEdge() const;

private:
    int zoom_ = 0;
    int tile_x_ = 0, tile_y_ = 0, tiles_x_ = 0, tiles_y_ = 0;
    int width_ = 0, height_ = 0;
    std::string attribution_;
    std::vector<unsigned char> pixels_;
};

}  // namespace georay
