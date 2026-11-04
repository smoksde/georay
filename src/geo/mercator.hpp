#pragma once

// Web Mercator, and a geographic box expressed in it.
//
// A box is defined by its geographic bounds; a position's UV inside it is a
// function of those bounds alone, independent of the zoom the box happens
// to be filled at. That is what lets the terrain mesh bake UVs once and the
// texture underneath be re-stitched at any resolution.
//
// Web Mercator's y axis is not linear in latitude - interpolating latitude
// instead misplaces the map by over a metre across a 4 km scene.

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

// Rounds up: the zoom no coarser than a wanted ground resolution, clamped to
// [min_zoom, max_zoom] since a source with no tiles past z15 cannot give z19.
int zoomForResolution(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom);

// Rounds down: the finest zoom still no finer than a limit. For a stitched
// texture with a fixed pixel budget - one step too fine at a 4096-pixel cap
// over 40 km is four times the tiles for a picture that ends up identical.
int zoomNotFinerThan(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom);

// Rounds to nearest. The right choice for streamed chunks, where the other
// two are not: rounding up is four times the tiles to be twice as sharp as
// the screen shows; rounding down is visibly soft. Nearest is within a
// factor of 1.4 either way - invisible.
int zoomNearest(double lat_deg, double metres_per_pixel, int min_zoom, int max_zoom);

// A geographic box, held in Mercator so that interpolation inside it is
// linear in the projection rather than in latitude.
class MercatorBox {
public:
    MercatorBox() = default;
    // Built from geographic bounds; `zoom` only fixes the units the internal
    // tile coordinates use, not what the box is later filled with.
    static MercatorBox fromBounds(double south_deg, double north_deg, double west_deg,
                                  double east_deg, int zoom = 20);
    // The box covering a square of `half_extent_m` around a position.
    static MercatorBox around(double lat_deg, double lon_deg, double half_extent_m,
                              int zoom = 20);

    bool valid() const { return tile_width_ > 0.0 && tile_height_ > 0.0; }

    // u west to east, v north to south, matching a raster whose first row is
    // its northern edge. Outside 0..1 means outside the box.
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
