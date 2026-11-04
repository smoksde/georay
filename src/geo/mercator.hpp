#pragma once

// Web Mercator, and a geographic box expressed in it.
//
// This exists so that texture coordinates stop depending on which tiles a map
// happens to be made of. A box is defined by its geographic bounds; the UV of
// a position inside it is a function of those bounds alone. Change the zoom
// the box is filled at - a different tile grid, a different pixel count - and
// every UV stays exactly where it was.
//
// That is what makes zoom-matched detail possible without rebuilding the
// terrain mesh: the mesh bakes UVs once, and the texture underneath can be
// re-stitched at any resolution.
//
// Web Mercator's y axis is not linear in latitude, and at the scales georay
// works at that matters: interpolating latitude instead misplaces the map by
// over a metre across a 4 km scene.

#include <glm/glm.hpp>

namespace georay {

// Fractional slippy-map tile coordinates at a zoom level. x runs west to
// east, y north to south, both 0..2^zoom.
glm::dvec2 mercatorTile(double lat_deg, double lon_deg, int zoom);
// Inverse of the y component.
double mercatorLatitude(double tile_y, int zoom);
// Longitude of a tile x coordinate.
double mercatorLongitude(double tile_x, int zoom);

// Ground resolution of one pixel of a 256-pixel tile at this zoom.
double mercatorMetresPerPixel(double lat_deg, int zoom);

// The zoom whose native tile resolution is closest to - and no coarser than -
// a wanted ground resolution. Clamped to [min_zoom, max_zoom], because a
// source that has no tiles past z15 cannot be asked for z19 however much
// detail the view would like.
int zoomForResolution(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom);

// The finest zoom whose native resolution is still no finer than a limit -
// the opposite rounding to zoomForResolution.
//
// Needed because a stitched texture has a fixed pixel budget. Picking a zoom
// finer than that budget downloads tiles only to average them away: at a
// 4096-pixel cap over a 40 km box, one step too fine is four times the tiles
// for a picture that ends up identical.
int zoomNotFinerThan(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom);

// A geographic box, held in Mercator so that interpolation inside it is
// linear in the projection rather than in latitude.
class MercatorBox {
public:
    MercatorBox() = default;
    // Built from geographic bounds. `zoom` only fixes the units the internal
    // tile coordinates are expressed in; it does not constrain what the box
    // is later filled with.
    static MercatorBox fromBounds(double south_deg, double north_deg, double west_deg,
                                  double east_deg, int zoom = 20);
    // The box covering a square of `half_extent_m` around a position.
    static MercatorBox around(double lat_deg, double lon_deg, double half_extent_m,
                              int zoom = 20);

    bool valid() const { return tile_width_ > 0.0 && tile_height_ > 0.0; }

    // Texture coordinates: u west to east, v north to south, matching a
    // raster whose first row is its northern edge. Outside 0..1 means outside
    // the box, which callers may legitimately want to know.
    glm::dvec2 uvFor(double lat_deg, double lon_deg) const;
    bool contains(double lat_deg, double lon_deg) const;

    double south() const;
    double north() const;
    double west() const;
    double east() const;

    // The tile range covering this box at a zoom, inclusive.
    void tileRange(int zoom, int& x0, int& y0, int& x1, int& y1) const;

    int zoom() const { return zoom_; }

private:
    int zoom_ = 20;
    double tile_x_ = 0.0, tile_y_ = 0.0;        // north-west corner
    double tile_width_ = 0.0, tile_height_ = 0.0;
};

}  // namespace georay
