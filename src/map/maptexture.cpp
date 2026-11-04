#include "map/maptexture.hpp"

#include <algorithm>
#include <cmath>

namespace georay {

MapTexture::Result MapTexture::build(TileCache& cache, const TileSource& source,
                                     const MercatorBox& box, int zoom, int max_pixels) {
    Result result;
    if (!box.valid()) return result;

    zoom = std::min(std::max(zoom, source.min_zoom), source.max_zoom);
    result.zoom = zoom;

    int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    box.tileRange(zoom, x0, y0, x1, y1);
    const int tiles_x = x1 - x0 + 1, tiles_y = y1 - y0 + 1;
    result.tiles_wanted = tiles_x * tiles_y;

    // The stitched grid, before cropping to the box. It covers whole tiles,
    // which overhang the box on every side.
    const int grid_w = tiles_x * DecodedTile::kSize;
    const int grid_h = tiles_y * DecodedTile::kSize;
    if (grid_w <= 0 || grid_h <= 0) return result;

    // The box's own position within that grid, in grid pixels. Cropping to it
    // is what makes the texture cover exactly the box - and therefore what
    // makes the UVs independent of which tiles were used.
    const glm::dvec2 nw = mercatorTile(box.north(), box.west(), zoom);
    const glm::dvec2 se = mercatorTile(box.south(), box.east(), zoom);
    const double crop_x = (nw.x - double(x0)) * DecodedTile::kSize;
    const double crop_y = (nw.y - double(y0)) * DecodedTile::kSize;
    const double crop_w = (se.x - nw.x) * DecodedTile::kSize;
    const double crop_h = (se.y - nw.y) * DecodedTile::kSize;
    if (!(crop_w > 0.5) || !(crop_h > 0.5)) return result;

    // Output size: the cropped region at native resolution, capped. Capping
    // rather than always filling max_pixels avoids magnifying a small box
    // into a big blurry texture.
    double scale = 1.0;
    const double longest = std::max(crop_w, crop_h);
    if (max_pixels > 0 && longest > double(max_pixels)) scale = double(max_pixels) / longest;
    width_ = std::max(1, int(std::lround(crop_w * scale)));
    height_ = std::max(1, int(std::lround(crop_h * scale)));

    // Flat grey where no tile is available, so a gap reads as missing data
    // rather than as black ground or as whatever was there before.
    pixels_.assign(size_t(width_) * size_t(height_) * 3, 0x9a);

    for (int ty = y0; ty <= y1; ++ty) {
        for (int tx = x0; tx <= x1; ++tx) {
            const DecodedTile* tile = cache.get(source.name, zoom, tx, ty);
            if (!tile || !tile->valid()) {
                result.missing.emplace_back(tx, ty);
                continue;
            }
            ++result.tiles_present;

            // Where this tile lands in the output, in output pixels.
            const double tile_grid_x = double(tx - x0) * DecodedTile::kSize;
            const double tile_grid_y = double(ty - y0) * DecodedTile::kSize;
            const int out_x0 = std::max(0, int(std::floor((tile_grid_x - crop_x) * scale)));
            const int out_y0 = std::max(0, int(std::floor((tile_grid_y - crop_y) * scale)));
            const int out_x1 = std::min(
                width_, int(std::ceil((tile_grid_x + DecodedTile::kSize - crop_x) * scale)));
            const int out_y1 = std::min(
                height_, int(std::ceil((tile_grid_y + DecodedTile::kSize - crop_y) * scale)));

            // Sampled per output pixel rather than blitted per source pixel,
            // because the output is an arbitrary scale of the input and the
            // inverse mapping is the one that leaves no holes.
            for (int oy = out_y0; oy < out_y1; ++oy) {
                const double grid_y = crop_y + (double(oy) + 0.5) / scale;
                const int sy = int(grid_y - tile_grid_y);
                if (sy < 0 || sy >= DecodedTile::kSize) continue;
                for (int ox = out_x0; ox < out_x1; ++ox) {
                    const double grid_x = crop_x + (double(ox) + 0.5) / scale;
                    const int sx = int(grid_x - tile_grid_x);
                    if (sx < 0 || sx >= DecodedTile::kSize) continue;
                    const unsigned char* src =
                        tile->rgb.data() + (size_t(sy) * DecodedTile::kSize + sx) * 3;
                    unsigned char* dst = pixels_.data() + (size_t(oy) * width_ + ox) * 3;
                    dst[0] = src[0];
                    dst[1] = src[1];
                    dst[2] = src[2];
                }
            }
        }
    }

    box_ = box;
    zoom_ = zoom;
    // The box's own width on the ground divided by the texture's width, so
    // this is what a pixel actually covers rather than what the zoom level
    // would give at native scale.
    const double mid_lat = 0.5 * (box.north() + box.south());
    metres_per_pixel_ = mercatorMetresPerPixel(mid_lat, zoom) / scale;

    result.ok = result.tiles_present > 0;
    result.width = width_;
    result.height = height_;
    result.metres_per_pixel = metres_per_pixel_;
    return result;
}

}  // namespace georay
