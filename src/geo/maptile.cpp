#include "geo/maptile.hpp"

#include <cmath>
#include <cstring>
#include <fstream>

#include "geo/geodetic.hpp"

namespace georay {
namespace {

constexpr char kMagic[8] = {'G', 'E', 'O', 'R', 'A', 'Y', 'M', '1'};
constexpr int kAttributionBytes = 128;
// Web Mercator cannot represent the poles; the standard cut-off is the
// latitude at which the projected square closes.
constexpr double kMercatorLimit = 85.05112878;

template <typename T>
bool readRaw(std::istream& in, T& value) {
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return bool(in);
}

// Fractional slippy-map tile coordinates: the forward Web Mercator transform,
// identical to the one scripts/fetch_map.py used to lay the raster out.
double tileX(double lon_deg, double n) { return (lon_deg + 180.0) / 360.0 * n; }

double tileY(double lat_deg, double n) {
    const double clamped = glm::clamp(lat_deg, -kMercatorLimit, kMercatorLimit);
    const double lat = deg2rad(clamped);
    return (1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / 3.14159265358979323846) / 2.0 * n;
}

// Inverse, for reporting the raster's latitude bounds.
double latForTileY(double y, double n) {
    const double t = 3.14159265358979323846 * (1.0 - 2.0 * y / n);
    return rad2deg(std::atan(std::sinh(t)));
}

}  // namespace

bool MapRaster::load(const std::string& path, MapRaster& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open map raster " + path;
        return false;
    }

    char magic[8] = {};
    file.read(magic, sizeof(magic));
    if (!file || std::memcmp(magic, kMagic, sizeof(magic)) != 0) {
        error = path + ": not a georay map raster (bad magic)";
        return false;
    }

    MapRaster m;
    int32_t zoom = 0, tile_x = 0, tile_y = 0, tiles_x = 0, tiles_y = 0, width = 0, height = 0;
    if (!readRaw(file, zoom) || !readRaw(file, tile_x) || !readRaw(file, tile_y) ||
        !readRaw(file, tiles_x) || !readRaw(file, tiles_y) || !readRaw(file, width) ||
        !readRaw(file, height)) {
        error = path + ": truncated header";
        return false;
    }

    // Bounds before allocating: a corrupt header must not be able to ask for
    // an arbitrary amount of memory.
    if (zoom < 1 || zoom > 24 || tiles_x < 1 || tiles_y < 1 || width < 1 || height < 1 ||
        width > 32768 || height > 32768) {
        error = path + ": implausible raster dimensions";
        return false;
    }

    std::vector<char> attribution(kAttributionBytes);
    file.read(attribution.data(), kAttributionBytes);
    if (file.gcount() != kAttributionBytes) {
        error = path + ": truncated attribution";
        return false;
    }
    // The field is null-padded; stop at the first terminator rather than
    // carrying the padding into the HUD.
    m.attribution_.assign(attribution.data(),
                          strnlen(attribution.data(), kAttributionBytes));

    const size_t bytes = size_t(width) * size_t(height) * 3;
    m.pixels_.resize(bytes);
    file.read(reinterpret_cast<char*>(m.pixels_.data()), std::streamsize(bytes));
    if (file.gcount() != std::streamsize(bytes)) {
        error = path + ": truncated pixel data";
        return false;
    }

    m.zoom_ = zoom;
    m.tile_x_ = tile_x;
    m.tile_y_ = tile_y;
    m.tiles_x_ = tiles_x;
    m.tiles_y_ = tiles_y;
    m.width_ = width;
    m.height_ = height;
    out = std::move(m);
    return true;
}

glm::dvec2 MapRaster::uvFor(double lat_deg, double lon_deg) const {
    if (!valid()) return {0.0, 0.0};
    const double n = std::pow(2.0, double(zoom_));
    return {(tileX(lon_deg, n) - double(tile_x_)) / double(tiles_x_),
            (tileY(lat_deg, n) - double(tile_y_)) / double(tiles_y_)};
}

double MapRaster::metresPerPixel(double lat_deg) const {
    if (!valid()) return 0.0;
    // Web Mercator's scale factor is 1/cos(latitude), so a pixel covers less
    // ground the further from the equator it is.
    const double native = 156543.03392 * std::cos(deg2rad(lat_deg)) / std::pow(2.0, double(zoom_));

    // ...and the raster is not necessarily at its tiles' native 256 pixels
    // each: fetch_map.py scales the stitched image down to fit --max-pixels.
    // Reporting the zoom level's resolution regardless would claim detail the
    // file does not contain - which matters here, because this number is what
    // says whether flying lower can show you anything.
    const double native_width = double(tiles_x_) * 256.0;
    return native * (native_width / double(width_));
}

double MapRaster::northEdge() const {
    return latForTileY(double(tile_y_), std::pow(2.0, double(zoom_)));
}
double MapRaster::southEdge() const {
    return latForTileY(double(tile_y_ + tiles_y_), std::pow(2.0, double(zoom_)));
}
double MapRaster::westEdge() const {
    return double(tile_x_) / std::pow(2.0, double(zoom_)) * 360.0 - 180.0;
}
double MapRaster::eastEdge() const {
    return double(tile_x_ + tiles_x_) / std::pow(2.0, double(zoom_)) * 360.0 - 180.0;
}

}  // namespace georay
