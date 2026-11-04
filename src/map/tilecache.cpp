#include "map/tilecache.hpp"

#include <algorithm>
#include <fstream>

// Only the two formats tile servers actually serve. Every other decoder stb
// offers is switched off, so a malformed tile has a smaller surface to go
// wrong in.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#define STBI_NO_FAILURE_STRINGS
#include "stb_image.h"

namespace georay {
namespace {

// A tile file should be a few tens of kilobytes. The cap is not about disk,
// it is about not handing an arbitrary-length buffer to a decoder because
// something unexpected is sitting in the cache directory.
constexpr size_t kMaxTileBytes = 16u * 1024u * 1024u;

}  // namespace

TileCache::TileCache(std::string directory, size_t memory_limit)
    : directory_(std::move(directory)), memory_limit_(memory_limit > 0 ? memory_limit : 1) {}

std::string TileCache::pathFor(const std::string& directory, const std::string& source,
                               int zoom, int x, int y) {
    return directory + "/" + source + "_" + std::to_string(zoom) + "_" + std::to_string(x) +
           "_" + std::to_string(y) + ".img";
}

std::string TileCache::key(const std::string& source, int zoom, int x, int y) const {
    return source + "/" + std::to_string(zoom) + "/" + std::to_string(x) + "/" +
           std::to_string(y);
}

bool TileCache::onDisk(const std::string& source, int zoom, int x, int y) const {
    return std::ifstream(pathFor(directory_, source, zoom, x, y), std::ios::binary).good();
}

const DecodedTile* TileCache::get(const std::string& source, int zoom, int x, int y) {
    const std::string id = key(source, zoom, x, y);

    const auto found = resident_.find(id);
    if (found != resident_.end()) {
        // Move to the back of the use order, so the front is genuinely the
        // least recently used.
        const auto at = std::find(recent_.begin(), recent_.end(), id);
        if (at != recent_.end()) recent_.erase(at);
        recent_.push_back(id);
        return &found->second;
    }
    // A tile that failed once will fail again; re-reading and re-decoding it
    // every frame is the difference between a missing tile and a stall.
    if (unreadable_.count(id)) return nullptr;

    const std::string path = pathFor(directory_, source, zoom, x, y);
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) return nullptr;  // simply not fetched yet; not an error

    const std::streamsize size = file.tellg();
    if (size <= 0 || size_t(size) > kMaxTileBytes) {
        unreadable_[id] = true;
        ++failure_count_;
        last_error_ = path + ": implausible size";
        return nullptr;
    }
    file.seekg(0);
    // Named, rather than vector<unsigned char> encoded(size_t(size)): that
    // form is a function declaration, not a variable.
    const size_t byte_count = size_t(size);
    std::vector<unsigned char> encoded(byte_count);
    file.read(reinterpret_cast<char*>(encoded.data()), size);
    if (file.gcount() != size) {
        unreadable_[id] = true;
        ++failure_count_;
        last_error_ = path + ": short read";
        return nullptr;
    }

    int width = 0, height = 0, channels = 0;
    unsigned char* pixels = stbi_load_from_memory(encoded.data(), int(encoded.size()), &width,
                                                  &height, &channels, 3);
    if (!pixels) {
        // Most often a half-written file from an interrupted fetch, or an
        // error page saved with a .img name.
        unreadable_[id] = true;
        ++failure_count_;
        last_error_ = path + ": not a decodable image";
        return nullptr;
    }

    DecodedTile tile;
    tile.rgb.resize(size_t(DecodedTile::kSize) * DecodedTile::kSize * 3);
    if (width == DecodedTile::kSize && height == DecodedTile::kSize) {
        std::copy(pixels, pixels + tile.rgb.size(), tile.rgb.begin());
    } else if (width > 0 && height > 0) {
        // Some services serve 512-pixel tiles for the same ground. Nearest
        // resampling to the common size keeps the stitcher's arithmetic
        // uniform; the alternative is a per-tile scale factor threaded
        // through every caller.
        for (int row = 0; row < DecodedTile::kSize; ++row) {
            const int sy = row * height / DecodedTile::kSize;
            for (int col = 0; col < DecodedTile::kSize; ++col) {
                const int sx = col * width / DecodedTile::kSize;
                const unsigned char* src = pixels + (size_t(sy) * width + sx) * 3;
                unsigned char* dst =
                    tile.rgb.data() + (size_t(row) * DecodedTile::kSize + col) * 3;
                dst[0] = src[0];
                dst[1] = src[1];
                dst[2] = src[2];
            }
        }
    }
    stbi_image_free(pixels);

    ++decode_count_;
    resident_[id] = std::move(tile);
    recent_.push_back(id);

    while (resident_.size() > memory_limit_ && !recent_.empty()) {
        const std::string oldest = recent_.front();
        recent_.pop_front();
        if (oldest != id) resident_.erase(oldest);
    }
    return &resident_[id];
}

void TileCache::clearMemory() {
    resident_.clear();
    recent_.clear();
}

}  // namespace georay
