#include "geo/dem.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace georay {
namespace {

// Integer square root of n, or 0 when n is not a perfect square. Used to
// turn a file length into a posting count, which is the only way a headerless
// format can declare its own resolution.
int exactSqrt(long long n) {
    if (n <= 0) return 0;
    long long r = (long long)std::llround(std::sqrt(double(n)));
    for (long long c = std::max(0LL, r - 2); c <= r + 2; ++c) {
        if (c * c == n) return int(c);
    }
    return 0;
}

}  // namespace

bool HgtTile::parseName(const std::string& path, int& lat, int& lon) {
    // Work from the basename, so a directory called "N00E000" upstream of the
    // file cannot be mistaken for the tile.
    const size_t slash = path.find_last_of("/\\");
    std::string base = slash == std::string::npos ? path : path.substr(slash + 1);

    const size_t dot = base.find('.');
    if (dot != std::string::npos) base = base.substr(0, dot);
    if (base.size() < 7) return false;

    const char ns = char(std::toupper(static_cast<unsigned char>(base[0])));
    const char ew = char(std::toupper(static_cast<unsigned char>(base[3])));
    if ((ns != 'N' && ns != 'S') || (ew != 'E' && ew != 'W')) return false;

    for (int i = 1; i <= 2; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(base[i]))) return false;
    }
    for (int i = 4; i <= 6; ++i) {
        if (!std::isdigit(static_cast<unsigned char>(base[i]))) return false;
    }

    lat = std::stoi(base.substr(1, 2)) * (ns == 'N' ? 1 : -1);
    lon = std::stoi(base.substr(4, 3)) * (ew == 'E' ? 1 : -1);
    return lat >= -90 && lat <= 89 && lon >= -180 && lon <= 179;
}

std::string HgtTile::tileNameFor(double lat_deg, double lon_deg) {
    // A NaN would make the conversion below undefined, and an out-of-range
    // coordinate would format a name no tile could ever have. Clamping first
    // means a bad position asks for a real tile and is told there is no data,
    // rather than producing something stranger.
    if (!std::isfinite(lat_deg)) lat_deg = 0.0;
    if (!std::isfinite(lon_deg)) lon_deg = 0.0;

    // Floor, not truncation: at latitude -0.5 the tile is S01, and truncating
    // would call it N00 and read the wrong side of the equator.
    const int lat = std::clamp(int(std::floor(lat_deg)), -90, 89);
    const int lon = std::clamp(int(std::floor(lon_deg)), -180, 179);

    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%c%02d%c%03d", lat < 0 ? 'S' : 'N', std::abs(lat),
                  lon < 0 ? 'W' : 'E', std::abs(lon));
    return buffer;
}

bool HgtTile::load(const std::string& path, HgtTile& out, std::string& error) {
    int lat = 0, lon = 0;
    if (!parseName(path, lat, lon)) {
        error = path + ": filename does not name a tile (expected e.g. N48E011.hgt)";
        return false;
    }

    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        error = "cannot open " + path;
        return false;
    }
    const long long bytes = file.tellg();
    file.seekg(0);

    if (bytes <= 0 || bytes % 2 != 0) {
        error = path + ": not a whole number of 16-bit samples";
        return false;
    }
    const int side = exactSqrt(bytes / 2);
    if (side < 2) {
        error = path + ": length is not a square number of samples";
        return false;
    }

    // Named, rather than vector<unsigned char> raw(size_t(bytes)): that form
    // is a function declaration, not a variable.
    const size_t byte_count = size_t(bytes);
    std::vector<unsigned char> raw(byte_count);
    file.read(reinterpret_cast<char*>(raw.data()), std::streamsize(bytes));
    if (file.gcount() != std::streamsize(bytes)) {
        error = path + ": short read";
        return false;
    }

    out.samples_.resize(size_t(side) * size_t(side));
    out.void_count_ = 0;
    for (size_t i = 0; i < out.samples_.size(); ++i) {
        // Big-endian, always - .hgt predates the convention of writing the
        // host's byte order and has never changed. Assembling the value by
        // hand rather than byte-swapping a loaded int16 keeps it correct on a
        // big-endian host too.
        const int16_t value = int16_t((raw[i * 2] << 8) | raw[i * 2 + 1]);
        out.samples_[i] = value;
        if (value == kVoid) ++out.void_count_;
    }

    out.corner_lat_ = lat;
    out.corner_lon_ = lon;
    out.samples_per_side_ = side;
    const size_t slash = path.find_last_of("/\\");
    out.name_ = slash == std::string::npos ? path : path.substr(slash + 1);
    return true;
}

int16_t HgtTile::post(int row, int col) const {
    if (row < 0 || col < 0 || row >= samples_per_side_ || col >= samples_per_side_) return kVoid;
    return samples_[size_t(row) * size_t(samples_per_side_) + size_t(col)];
}

bool HgtTile::contains(double lat_deg, double lon_deg) const {
    return lat_deg >= double(corner_lat_) && lat_deg <= double(corner_lat_) + 1.0 &&
           lon_deg >= double(corner_lon_) && lon_deg <= double(corner_lon_) + 1.0;
}

bool HgtTile::sample(double lat_deg, double lon_deg, double& height_m) const {
    if (samples_per_side_ < 2 || !contains(lat_deg, lon_deg)) return false;

    const double n = double(samples_per_side_ - 1);
    // Row 0 is the northern edge, so the row coordinate grows southwards.
    const double y = (double(corner_lat_) + 1.0 - lat_deg) * n;
    const double x = (lon_deg - double(corner_lon_)) * n;

    const int row = std::min(std::max(int(std::floor(y)), 0), samples_per_side_ - 2);
    const int col = std::min(std::max(int(std::floor(x)), 0), samples_per_side_ - 2);
    const double ty = std::clamp(y - double(row), 0.0, 1.0);
    const double tx = std::clamp(x - double(col), 0.0, 1.0);

    const int16_t p00 = post(row, col), p01 = post(row, col + 1);
    const int16_t p10 = post(row + 1, col), p11 = post(row + 1, col + 1);

    // Interpolate over whichever corners have data, weighted as usual. A void
    // next to real ground should not drag the surface to zero, and dropping
    // the sample entirely would tear a hole through terrain that is mostly
    // fine. Only an entirely void cell is genuinely no-data.
    const double w00 = (1 - tx) * (1 - ty), w01 = tx * (1 - ty);
    const double w10 = (1 - tx) * ty, w11 = tx * ty;

    double sum = 0.0, weight = 0.0;
    if (p00 != kVoid) { sum += w00 * p00; weight += w00; }
    if (p01 != kVoid) { sum += w01 * p01; weight += w01; }
    if (p10 != kVoid) { sum += w10 * p10; weight += w10; }
    if (p11 != kVoid) { sum += w11 * p11; weight += w11; }

    if (weight <= 1e-12) return false;
    height_m = sum / weight;
    return true;
}

const HgtTile* DemSource::tileFor(double lat_deg, double lon_deg) const {
    const std::string name = HgtTile::tileNameFor(lat_deg, lon_deg);

    const auto found = tiles_.find(name);
    if (found != tiles_.end()) return &found->second;
    // Remembered misses: without this, every sample over a gap re-stats a
    // file that is not there, which over a 263k-post resample is a quarter of
    // a million failed opens.
    if (missing_.count(name)) return nullptr;

    HgtTile tile;
    std::string error;
    const std::string path = directory_ + "/" + name + ".hgt";
    if (!HgtTile::load(path, tile, error)) {
        missing_[name] = true;
        last_error_ = error;
        return nullptr;
    }
    return &tiles_.emplace(name, std::move(tile)).first->second;
}

bool DemSource::sampleOrthometric(double lat_deg, double lon_deg, double& height_m) const {
    const HgtTile* tile = tileFor(lat_deg, lon_deg);
    if (!tile) return false;
    return tile->sample(lat_deg, lon_deg, height_m);
}

std::vector<std::string> DemSource::loadedTiles() const {
    std::vector<std::string> names;
    names.reserve(tiles_.size());
    for (const auto& entry : tiles_) names.push_back(entry.first);
    return names;
}

std::vector<std::string> DemSource::missingTiles() const {
    std::vector<std::string> names;
    names.reserve(missing_.size());
    for (const auto& entry : missing_) names.push_back(entry.first);
    return names;
}

}  // namespace georay
