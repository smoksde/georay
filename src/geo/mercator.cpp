#include "geo/mercator.hpp"

#include <algorithm>
#include <cmath>

#include "geo/geodetic.hpp"

namespace georay {
namespace {

constexpr double kPi = 3.14159265358979323846;
// Web Mercator cannot represent the poles; this is the latitude at which the
// projected world closes into a square.
constexpr double kLimit = 85.05112878;

}  // namespace

glm::dvec2 mercatorTile(double lat_deg, double lon_deg, int zoom) {
    const double n = std::pow(2.0, double(zoom));
    const double lat = deg2rad(glm::clamp(lat_deg, -kLimit, kLimit));
    return {(lon_deg + 180.0) / 360.0 * n,
            (1.0 - std::log(std::tan(lat) + 1.0 / std::cos(lat)) / kPi) / 2.0 * n};
}

double mercatorLatitude(double tile_y, int zoom) {
    const double n = std::pow(2.0, double(zoom));
    return rad2deg(std::atan(std::sinh(kPi * (1.0 - 2.0 * tile_y / n))));
}

double mercatorLongitude(double tile_x, int zoom) {
    return tile_x / std::pow(2.0, double(zoom)) * 360.0 - 180.0;
}

double mercatorMetresPerPixel(double lat_deg, int zoom) {
    return 156543.03392 * std::cos(deg2rad(glm::clamp(lat_deg, -kLimit, kLimit))) /
           std::pow(2.0, double(zoom));
}

int zoomForResolution(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom) {
    if (!(metres_per_pixel > 0.0)) return max_zoom;
    // Solve 156543 * cos(lat) / 2^z = wanted, then round up: a zoom whose
    // pixels are finer than asked for is detail going to waste, one whose
    // pixels are coarser is the blur this function exists to avoid.
    const double finest = mercatorMetresPerPixel(lat_deg, 0);
    const double exact = std::log2(finest / metres_per_pixel);
    const int zoom = int(std::ceil(exact - 1e-9));
    return std::min(std::max(zoom, min_zoom), max_zoom);
}

int zoomNotFinerThan(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom) {
    if (!(metres_per_pixel > 0.0)) return min_zoom;
    const double finest = mercatorMetresPerPixel(lat_deg, 0);
    const double exact = std::log2(finest / metres_per_pixel);
    const int zoom = int(std::floor(exact + 1e-9));
    return std::min(std::max(zoom, min_zoom), max_zoom);
}

MercatorBox MercatorBox::fromBounds(double south_deg, double north_deg, double west_deg,
                                    double east_deg, int zoom) {
    MercatorBox box;
    box.zoom_ = zoom;
    const glm::dvec2 nw = mercatorTile(north_deg, west_deg, zoom);
    const glm::dvec2 se = mercatorTile(south_deg, east_deg, zoom);
    box.tile_x_ = nw.x;
    box.tile_y_ = nw.y;
    box.tile_width_ = se.x - nw.x;
    box.tile_height_ = se.y - nw.y;
    return box;
}

MercatorBox MercatorBox::around(double lat_deg, double lon_deg, double half_extent_m, int zoom) {
    // Longitude degrees shrink towards the poles, hence the cosine; the guard
    // keeps a polar box from spanning the world.
    const double d_lat = half_extent_m / 110574.0;
    const double d_lon =
        half_extent_m / std::max(111320.0 * std::cos(deg2rad(lat_deg)), 1.0);
    return fromBounds(lat_deg - d_lat, lat_deg + d_lat, lon_deg - d_lon, lon_deg + d_lon, zoom);
}

glm::dvec2 MercatorBox::uvFor(double lat_deg, double lon_deg) const {
    if (!valid()) return {0.0, 0.0};
    const glm::dvec2 t = mercatorTile(lat_deg, lon_deg, zoom_);
    return {(t.x - tile_x_) / tile_width_, (t.y - tile_y_) / tile_height_};
}

bool MercatorBox::contains(double lat_deg, double lon_deg) const {
    const glm::dvec2 uv = uvFor(lat_deg, lon_deg);
    return uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0;
}

double MercatorBox::north() const { return mercatorLatitude(tile_y_, zoom_); }
double MercatorBox::south() const { return mercatorLatitude(tile_y_ + tile_height_, zoom_); }
double MercatorBox::west() const { return mercatorLongitude(tile_x_, zoom_); }
double MercatorBox::east() const { return mercatorLongitude(tile_x_ + tile_width_, zoom_); }

void MercatorBox::tileRange(int zoom, int& x0, int& y0, int& x1, int& y1) const {
    // Rescale this box's tile coordinates into the requested zoom's units.
    const double scale = std::pow(2.0, double(zoom) - double(zoom_));
    const double left = tile_x_ * scale, top = tile_y_ * scale;
    const double right = (tile_x_ + tile_width_) * scale;
    const double bottom = (tile_y_ + tile_height_) * scale;

    const int n = int(std::pow(2.0, double(zoom)));
    x0 = std::max(0, int(std::floor(left)));
    y0 = std::max(0, int(std::floor(top)));
    // The box's far edge sits exactly on a tile boundary when it is a whole
    // number; including that tile would fetch a column of pixels the box does
    // not contain.
    x1 = std::min(n - 1, int(std::ceil(right)) - 1);
    y1 = std::min(n - 1, int(std::ceil(bottom)) - 1);
    x1 = std::max(x1, x0);
    y1 = std::max(y1, y0);
}

}  // namespace georay
