#pragma once

// The map image under the terrain, stitched from tiles at run time.
//
// The image covers a fixed geographic box, so its texture coordinates are a
// function of that box alone - re-stitch at a different zoom and every UV
// already baked into the terrain mesh stays exactly where it was. The map
// gets sharper without the mesh being touched.
//
// Not more detail at every scale: tiles are generalised per zoom, so a
// higher zoom brings finer pixels *and* more features, while magnifying a
// fixed zoom only brings blur. Choosing zoom from viewing distance keeps the
// two matched.

#include <string>
#include <vector>

#include "geo/mercator.hpp"
#include "map/tilecache.hpp"
#include "map/tilesource.hpp"

namespace georay {

class MapTexture {
public:
    // How the stitch went - a map 40% grey holes is a different thing from
    // a complete one, and the difference has to be visible.
    struct Result {
        bool ok = false;
        int zoom = 0;
        int tiles_wanted = 0;
        int tiles_present = 0;
        int width = 0, height = 0;
        double metres_per_pixel = 0.0;
        std::vector<std::pair<int, int>> missing;  // tile x,y that were absent

        double coverage() const {
            return tiles_wanted > 0 ? double(tiles_present) / double(tiles_wanted) : 0.0;
        }
    };

    // Stitches the tiles covering `box` at `zoom`, scaled so the longest side
    // is at most `max_pixels`. Tiles that are absent are left as a flat grey
    // and listed in the result, so the caller can fetch them.
    Result build(TileCache& cache, const TileSource& source, const MercatorBox& box, int zoom,
                 int max_pixels);

    bool valid() const { return width_ > 0 && height_ > 0; }
    int width() const { return width_; }
    int height() const { return height_; }
    const std::vector<unsigned char>& pixels() const { return pixels_; }  // RGB, row 0 north
    const MercatorBox& box() const { return box_; }
    int zoom() const { return zoom_; }
    double metresPerPixel() const { return metres_per_pixel_; }

private:
    MercatorBox box_;
    int zoom_ = 0;
    int width_ = 0, height_ = 0;
    double metres_per_pixel_ = 0.0;
    std::vector<unsigned char> pixels_;
};

}  // namespace georay
