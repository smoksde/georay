#pragma once

// Map tiles on disk, decoded into pixels.
//
// Shares its directory and naming with scripts/fetch_map.py - "<source>_<z>_
// <x>_<y>.img" - so a tile fetched by either is usable by both. That matters
// more than it sounds: the script remains the way to grab a region in bulk,
// and the live fetcher the way to fill gaps while flying, and neither should
// re-download what the other already has.
//
// Decoding happens here because the map is now stitched at run time rather
// than pre-assembled in Python. That is what buys zoom-matched detail - the
// texture can be rebuilt at any resolution - and it is why georay now carries
// an image decoder at all.

#include <cstdint>
#include <list>
#include <map>
#include <string>
#include <vector>

namespace georay {

// One decoded tile: always 256x256 RGB, whatever the file held.
struct DecodedTile {
    static constexpr int kSize = 256;
    std::vector<unsigned char> rgb;  // kSize * kSize * 3
    bool valid() const { return rgb.size() == size_t(kSize) * kSize * 3; }
};

class TileCache {
public:
    explicit TileCache(std::string directory, size_t memory_limit = 256);

    const std::string& directory() const { return directory_; }
    static std::string pathFor(const std::string& directory, const std::string& source,
                               int zoom, int x, int y);

    // Is this tile already on disk? Cheap - no decode, no read.
    bool onDisk(const std::string& source, int zoom, int x, int y) const;

    // Decoded pixels, or null when the tile is absent or unreadable. A tile
    // that fails to decode is remembered as bad, so a corrupt file is not
    // re-read and re-decoded on every frame.
    const DecodedTile* get(const std::string& source, int zoom, int x, int y);

    void clearMemory();
    size_t residentTiles() const { return resident_.size(); }
    const std::string& lastError() const { return last_error_; }

private:
    std::string key(const std::string& source, int zoom, int x, int y) const;

    std::string directory_;
    size_t memory_limit_;

    // Least-recently-used first. A 256x256 RGB tile is 192 KB, so a few
    // hundred is tens of megabytes - but unbounded it would be the whole
    // region, which is the thing streaming exists to avoid.
    std::map<std::string, DecodedTile> resident_;
    std::list<std::string> recent_;
    std::map<std::string, bool> unreadable_;

    std::string last_error_;
};

}  // namespace georay
