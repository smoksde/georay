#pragma once

// Real elevation data: SRTM-style .hgt tiles read straight off disk.
//
// The .hgt format is the reason real terrain costs this project nothing. It
// has no header whatsoever - the file is a square array of big-endian signed
// 16-bit metres, and every other fact about it comes from the filename and
// the file length:
//
//   N48E011.hgt   the south-west corner, in whole degrees
//   25,934,402 B  = 3601 x 3601 x 2, so 1 arc-second posting (SRTM1, ~30 m)
//    2,884,802 B  = 1201 x 1201 x 2, so 3 arc-second posting (SRTM3, ~90 m)
//
// Row 0 is the NORTHERN edge and column 0 the western one, so the array runs
// top-left to bottom-right exactly as the tile would be drawn on a map -
// which is the opposite of the south-to-north order the filename suggests,
// and the single easiest thing to get backwards.
//
// Each tile spans a full degree inclusive of both edges, so neighbouring
// tiles share a row or column of posts. That redundancy is what lets a
// position on a tile boundary be sampled from either tile and give the same
// answer, and sampleOrthometric() relies on it.
//
// Heights are orthometric (above the EGM96 geoid). Converting them for use
// in georay's ellipsoidal world is the GeoidModel's job - see geoid.hpp.
//
// Nothing here touches the network. Tiles are fetched by
// scripts/fetch_dem.py, deliberately, into a directory this reads.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace georay {

// A single .hgt tile held in memory.
class HgtTile {
public:
    // SRTM's void marker. Real gaps exist - steep terrain and water bodies
    // defeat the radar - and a void read as -32768 metres would punch a hole
    // 32 km deep through the terrain.
    static constexpr int16_t kVoid = -32768;

    static bool load(const std::string& path, HgtTile& out, std::string& error);

    // Corner of the tile, in whole degrees (its south-west post).
    int cornerLatitude() const { return corner_lat_; }
    int cornerLongitude() const { return corner_lon_; }
    int samplesPerSide() const { return samples_per_side_; }
    int voidCount() const { return void_count_; }
    const std::string& name() const { return name_; }

    bool contains(double lat_deg, double lon_deg) const;

    // Bilinearly interpolated orthometric height. Returns false where the
    // position is off the tile, or where every surrounding post is a void -
    // which is a different thing from a height of zero, and must not be
    // allowed to become one.
    bool sample(double lat_deg, double lon_deg, double& height_m) const;

    // Raw post, row 0 north. Out-of-range indices return kVoid.
    int16_t post(int row, int col) const;

    // Parses "N48E011" out of a path, with or without a .hgt suffix.
    static bool parseName(const std::string& path, int& lat, int& lon);
    // The tile name covering a position: floor of each coordinate.
    static std::string tileNameFor(double lat_deg, double lon_deg);

private:
    std::string name_;
    int corner_lat_ = 0, corner_lon_ = 0;
    int samples_per_side_ = 0;
    int void_count_ = 0;
    std::vector<int16_t> samples_;
};

// A directory of tiles, loaded as they are needed and evicted when too many
// accumulate.
//
// Lazy rather than eager because a scene's extent rarely lines up with the
// degree grid: a 6 km square near a corner touches four tiles, and loading a
// whole continent's worth up front to find that out would be absurd.
//
// Bounded because the aircraft travels. A one-degree SRTM1 tile is 26 MB in
// memory, so a cache that only ever grows is not a streaming cache - it is a
// slow way to load the whole dataset. A flight across a region touches tiles
// it will not come back to, and those have to go.
class DemSource {
public:
    // Eight tiles is 207 MB and comfortably more than any single terrain
    // window needs - a 40 km window straddling a degree corner touches four -
    // so the working set never thrashes against the limit.
    static constexpr size_t kDefaultTileLimit = 8;

    explicit DemSource(std::string directory, size_t tile_limit = kDefaultTileLimit)
        : directory_(std::move(directory)), tile_limit_(tile_limit > 0 ? tile_limit : 1) {}

    // Tiles read from disk since construction. Climbing steadily means the
    // limit is too low for the area being flown.
    int loadCount() const { return load_count_; }
    size_t residentTiles() const { return tiles_.size(); }

    // Orthometric height at a geodetic position. False means no data: either
    // no tile on disk covers it, or the posts around it are voids.
    bool sampleOrthometric(double lat_deg, double lon_deg, double& height_m) const;

    const std::string& directory() const { return directory_; }
    // Names of the tiles actually used so far, in load order.
    std::vector<std::string> loadedTiles() const;
    // Tiles that were asked for and are not on disk. Reported rather than
    // silently treated as sea, because missing coverage is a gap in the
    // answer, not an answer.
    std::vector<std::string> missingTiles() const;
    const std::string& lastError() const { return last_error_; }

private:
    const HgtTile* tileFor(double lat_deg, double lon_deg) const;

    std::string directory_;
    // Mutable because loading is a caching detail: sampling is logically a
    // const operation on a fixed directory of files.
    mutable std::map<std::string, HgtTile> tiles_;
    // Names in use order, oldest first, so the one to drop is the front.
    mutable std::vector<std::string> recent_;
    mutable std::map<std::string, bool> missing_;
    mutable std::string last_error_;
    size_t tile_limit_ = kDefaultTileLimit;
    mutable int load_count_ = 0;
};

}  // namespace georay
