#include "geo/geoid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace georay {
namespace {

// georay's geoid grid: see scripts/fetch_dem.py, which writes it.
//
//   char    magic[8] = "GEORAYG1"
//   int32   rows, cols
//   double  lat_min, lat_max, lon_min, lon_max   (positions of the edge posts)
//   double  scale, offset                        (metres = raw * scale + offset)
//   int16   samples[rows * cols]                 (row 0 north, col 0 west)
//
// All little-endian, which every platform this builds on is.
constexpr char kMagic[8] = {'G', 'E', 'O', 'R', 'A', 'Y', 'G', '1'};

template <typename T>
bool readRaw(std::istream& in, T& value) {
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return bool(in);
}

}  // namespace

GeoidModel GeoidModel::constant(double separation_m) {
    GeoidModel m;
    m.constant_m_ = separation_m;
    return m;
}

bool GeoidModel::loadGrid(const std::string& path, GeoidModel& out, std::string& error) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        error = "cannot open geoid grid " + path;
        return false;
    }

    char magic[8] = {};
    file.read(magic, sizeof(magic));
    if (!file || std::memcmp(magic, kMagic, sizeof(magic)) != 0) {
        error = path + ": not a georay geoid grid (bad magic)";
        return false;
    }

    GeoidModel m;
    int32_t rows = 0, cols = 0;
    if (!readRaw(file, rows) || !readRaw(file, cols) || !readRaw(file, m.lat_min_) ||
        !readRaw(file, m.lat_max_) || !readRaw(file, m.lon_min_) || !readRaw(file, m.lon_max_) ||
        !readRaw(file, m.scale_) || !readRaw(file, m.offset_)) {
        error = path + ": truncated header";
        return false;
    }
    // Two posts minimum in each direction, or the interpolation below has no
    // cell to work in; and a sanity cap so a corrupt header cannot ask for a
    // terabyte of memory.
    if (rows < 2 || cols < 2 || rows > 100000 || cols > 100000) {
        error = path + ": implausible grid size";
        return false;
    }
    if (!(m.lat_max_ > m.lat_min_) || !(m.lon_max_ > m.lon_min_)) {
        error = path + ": degenerate bounds";
        return false;
    }

    m.rows_ = rows;
    m.cols_ = cols;
    m.samples_.resize(size_t(rows) * size_t(cols));
    file.read(reinterpret_cast<char*>(m.samples_.data()),
              std::streamsize(m.samples_.size() * sizeof(int16_t)));
    if (file.gcount() != std::streamsize(m.samples_.size() * sizeof(int16_t))) {
        error = path + ": truncated sample data";
        return false;
    }

    m.gridded_ = true;
    m.source_ = path.substr(path.find_last_of("/\\") + 1);
    out = std::move(m);
    return true;
}

double GeoidModel::separationAt(double lat_deg, double lon_deg) const {
    if (!gridded_) return constant_m_;

    // Clamp rather than extrapolate. A geoid grid is normally global, so
    // falling off it means the caller asked for somewhere impossible; the
    // edge value is a far better answer than a linear extrapolation of a
    // surface that is not linear.
    const double lat = std::clamp(lat_deg, lat_min_, lat_max_);
    const double lon = std::clamp(lon_deg, lon_min_, lon_max_);

    // Row 0 is the northern edge, so the row index runs with decreasing
    // latitude.
    const double y = (lat_max_ - lat) / (lat_max_ - lat_min_) * double(rows_ - 1);
    const double x = (lon - lon_min_) / (lon_max_ - lon_min_) * double(cols_ - 1);

    const int iy = std::min(int(y), rows_ - 2);
    const int ix = std::min(int(x), cols_ - 2);
    const double ty = y - double(iy);
    const double tx = x - double(ix);

    auto at = [&](int r, int c) {
        return double(samples_[size_t(r) * size_t(cols_) + size_t(c)]) * scale_ + offset_;
    };
    const double n00 = at(iy, ix), n01 = at(iy, ix + 1);
    const double n10 = at(iy + 1, ix), n11 = at(iy + 1, ix + 1);
    return (n00 * (1 - tx) + n01 * tx) * (1 - ty) + (n10 * (1 - tx) + n11 * tx) * ty;
}

std::string GeoidModel::description() const {
    char buffer[160];
    if (gridded_) {
        std::snprintf(buffer, sizeof(buffer), "GRID %s", source_.c_str());
    } else if (constant_m_ != 0.0) {
        std::snprintf(buffer, sizeof(buffer), "CONSTANT %.2f M", constant_m_);
    } else {
        std::snprintf(buffer, sizeof(buffer), "NONE - HEIGHTS ARE ORTHOMETRIC");
    }
    return buffer;
}

}  // namespace georay
