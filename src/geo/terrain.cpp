#include "geo/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace georay {
namespace {

// Value noise with a fixed hash: deterministic across runs and platforms, so
// a geolocation reported in one session can be reproduced in the next. A
// std::mt19937-seeded random field would not be, once the grid resolution
// changed.
double hashNoise(int x, int y) {
    unsigned int h = unsigned(x) * 374761393u + unsigned(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    h ^= h >> 16;
    return double(h) / double(0xFFFFFFFFu) * 2.0 - 1.0;  // [-1, 1]
}

double smoothNoise(double x, double y) {
    const double fx = std::floor(x), fy = std::floor(y);
    const int ix = int(fx), iy = int(fy);
    const double tx = x - fx, ty = y - fy;
    // Smoothstep weights, so octaves summed below have continuous slope and
    // the terrain normals do not show the grid.
    const double wx = tx * tx * (3.0 - 2.0 * tx);
    const double wy = ty * ty * (3.0 - 2.0 * ty);

    const double n00 = hashNoise(ix, iy), n10 = hashNoise(ix + 1, iy);
    const double n01 = hashNoise(ix, iy + 1), n11 = hashNoise(ix + 1, iy + 1);
    return (n00 * (1 - wx) + n10 * wx) * (1 - wy) + (n01 * (1 - wx) + n11 * wx) * wy;
}

double fbm(double x, double y, int octaves, double lacunarity = 2.0, double gain = 0.5) {
    double sum = 0.0, amplitude = 1.0, frequency = 1.0, norm = 0.0;
    for (int i = 0; i < octaves; ++i) {
        sum += amplitude * smoothNoise(x * frequency, y * frequency);
        norm += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }
    return norm > 0.0 ? sum / norm : 0.0;
}

}  // namespace

Terrain::Terrain(Kind kind, double half_extent_m, int resolution, double base_elevation_m)
    : half_extent_m_(half_extent_m > 1.0 ? half_extent_m : 1.0),
      resolution_(resolution > 2 ? resolution : 2),
      base_elevation_m_(base_elevation_m) {
    spacing_m_ = 2.0 * half_extent_m_ / double(resolution_ - 1);
    generate(kind);
}

const char* Terrain::kindName(Kind kind) {
    switch (kind) {
        case Kind::Flat: return "FLAT";
        case Kind::Hills: return "HILLS";
        case Kind::Valley: return "VALLEY";
        case Kind::Mountains: return "MOUNTAINS";
    }
    return "?";
}

void Terrain::generate(Kind kind) {
    kind_ = kind;
    source_name_ = kindName(kind);
    heights_.assign(size_t(resolution_) * size_t(resolution_), base_elevation_m_);

    for (int iy = 0; iy < resolution_; ++iy) {
        for (int ix = 0; ix < resolution_; ++ix) {
            const glm::dvec2 p = gridPosition(ix, iy);
            // Noise coordinates in units of ~500 m, which gives features at
            // a scale a UAV at 100-300 m actually flies over.
            const double nx = p.x / 500.0, ny = p.y / 500.0;
            double h = 0.0;

            switch (kind) {
                case Kind::Flat:
                    h = 0.0;
                    break;
                case Kind::Hills:
                    h = 35.0 * fbm(nx, ny, 4) + 12.0 * fbm(nx * 3.0 + 7.3, ny * 3.0 - 2.1, 3);
                    break;
                case Kind::Valley: {
                    // A valley running roughly north-south: height grows with
                    // distance from the (meandering) channel.
                    const double channel = 400.0 * smoothNoise(ny * 0.6, 11.0);
                    const double across = std::fabs(p.x - channel) / 900.0;
                    h = 150.0 * std::min(1.0, across * across) +
                        25.0 * fbm(nx * 2.0, ny * 2.0, 4);
                    break;
                }
                case Kind::Mountains: {
                    // Ridged noise: folding the field about zero turns smooth
                    // bumps into creases, which is what makes a ray graze a
                    // near ridge instead of reaching the far slope - the case
                    // a flat-earth assumption gets most wrong.
                    const double ridged = 1.0 - std::fabs(fbm(nx * 0.8, ny * 0.8, 5));
                    h = 420.0 * ridged * ridged + 60.0 * fbm(nx * 4.0, ny * 4.0, 4);
                    break;
                }
            }
            heights_[size_t(iy) * resolution_ + ix] = base_elevation_m_ + h;
        }
    }
    updateExtrema();
}

int Terrain::fillGaps(std::vector<bool>& has_data) {
    const size_t count = heights_.size();
    int missing = 0;
    for (size_t i = 0; i < count; ++i) {
        if (!has_data[i]) ++missing;
    }
    if (missing == 0 || missing == int(count)) return missing;

    // Iterative neighbour diffusion. Each pass fills any gap post that has at
    // least one filled neighbour, so a void closes inwards from its rim at
    // one post per pass. The cap keeps a pathological input (a tile missing
    // entirely) from turning into a long loop; what it cannot reach falls
    // back to the mean below.
    std::vector<double> next = heights_;
    std::vector<bool> next_has = has_data;
    for (int pass = 0; pass < 64; ++pass) {
        bool changed = false;
        for (int iy = 0; iy < resolution_; ++iy) {
            for (int ix = 0; ix < resolution_; ++ix) {
                const size_t index = size_t(iy) * size_t(resolution_) + size_t(ix);
                if (has_data[index]) continue;

                double sum = 0.0;
                int n = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int nx = ix + dx, ny = iy + dy;
                        if (nx < 0 || ny < 0 || nx >= resolution_ || ny >= resolution_) continue;
                        const size_t j = size_t(ny) * size_t(resolution_) + size_t(nx);
                        if (!has_data[j]) continue;
                        sum += heights_[j];
                        ++n;
                    }
                }
                if (n > 0) {
                    next[index] = sum / double(n);
                    next_has[index] = true;
                    changed = true;
                }
            }
        }
        heights_ = next;
        has_data = next_has;
        if (!changed) break;
    }

    // Anything still unreached: the mean of what did load, which at least
    // keeps the surface at a sane elevation instead of at zero.
    double sum = 0.0;
    int n = 0;
    for (size_t i = 0; i < count; ++i) {
        if (has_data[i]) { sum += heights_[i]; ++n; }
    }
    const double mean = n > 0 ? sum / double(n) : 0.0;
    for (size_t i = 0; i < count; ++i) {
        if (!has_data[i]) heights_[i] = mean;
    }
    return missing;
}

Terrain::DemLoadReport Terrain::loadFromDem(const DemSource& dem, const GeoidModel& geoid,
                                            const EnuFrame& frame) {
    DemLoadReport report;
    report.posts_total = resolution_ * resolution_;

    std::vector<bool> has_data(heights_.size(), false);

    for (int iy = 0; iy < resolution_; ++iy) {
        for (int ix = 0; ix < resolution_; ++ix) {
            const size_t index = size_t(iy) * size_t(resolution_) + size_t(ix);
            const glm::dvec2 p = gridPosition(ix, iy);

            // Where this ENU post actually is on the ellipsoid. Taking the
            // latitude and longitude from a point on the tangent plane and
            // then converting the sampled height back through the same frame
            // is what carries the curvature term: over a few kilometres the
            // tangent plane rises above the ellipsoid by tens of centimetres,
            // and a flat subtraction would bake that straight into the
            // terrain as a dome.
            const Geodetic here = frame.toGeodetic(glm::dvec3(p.x, p.y, 0.0));

            double orthometric = 0.0;
            if (!dem.sampleOrthometric(here.latitude_deg, here.longitude_deg, orthometric)) {
                continue;  // left as a gap; filled below
            }

            const double ellipsoidal =
                geoid.toEllipsoidal(here.latitude_deg, here.longitude_deg, orthometric);
            heights_[index] =
                frame.fromGeodetic(Geodetic(here.latitude_deg, here.longitude_deg, ellipsoidal)).z;
            has_data[index] = true;
            ++report.posts_from_data;
        }
    }

    report.tiles_used = dem.loadedTiles();
    report.tiles_missing = dem.missingTiles();

    if (report.posts_from_data == 0) {
        // Nothing loaded at all. Leave the previous terrain in place rather
        // than replacing it with a flat sheet of invented mean elevation,
        // which would look like a successful load of very boring ground.
        report.error = dem.lastError().empty()
                           ? "no elevation data covers this area in " + dem.directory()
                           : dem.lastError();
        return report;
    }

    report.posts_filled = fillGaps(has_data);
    updateExtrema();

    const Geodetic origin = frame.origin();
    report.origin_separation_m =
        geoid.separationAt(origin.latitude_deg, origin.longitude_deg);
    double origin_height = 0.0;
    if (dem.sampleOrthometric(origin.latitude_deg, origin.longitude_deg, origin_height)) {
        report.origin_orthometric_m = origin_height;
    }

    kind_ = Kind::Flat;  // no synthetic generator produced these
    source_name_ = "SRTM";
    if (!report.tiles_used.empty()) {
        source_name_ += " " + report.tiles_used.front();
        if (report.tiles_used.size() > 1) {
            source_name_ += " +" + std::to_string(report.tiles_used.size() - 1);
        }
    }

    report.ok = true;
    return report;
}

void Terrain::updateExtrema() {
    const auto [lo, hi] = std::minmax_element(heights_.begin(), heights_.end());
    min_height_ = *lo;
    max_height_ = *hi;
}

glm::dvec2 Terrain::gridPosition(int ix, int iy) const {
    return {-half_extent_m_ + ix * spacing_m_, -half_extent_m_ + iy * spacing_m_};
}

bool Terrain::inCoverage(double east, double north) const {
    return std::fabs(east) <= half_extent_m_ && std::fabs(north) <= half_extent_m_;
}

double Terrain::sampleHeight(double east, double north) const {
    const double gx = (east + half_extent_m_) / spacing_m_;
    const double gy = (north + half_extent_m_) / spacing_m_;

    // Clamp to the last full cell, so the interpolation below always has four
    // valid corners and the surface extends flat beyond the modelled square
    // instead of ending in a cliff (or an out-of-bounds read).
    const double max_index = double(resolution_ - 1);
    const double cx = glm::clamp(gx, 0.0, max_index);
    const double cy = glm::clamp(gy, 0.0, max_index);

    const int ix = std::min(int(cx), resolution_ - 2);
    const int iy = std::min(int(cy), resolution_ - 2);
    const double tx = cx - double(ix);
    const double ty = cy - double(iy);

    const double h00 = heightAt(ix, iy), h10 = heightAt(ix + 1, iy);
    const double h01 = heightAt(ix, iy + 1), h11 = heightAt(ix + 1, iy + 1);
    return (h00 * (1 - tx) + h10 * tx) * (1 - ty) + (h01 * (1 - tx) + h11 * tx) * ty;
}

glm::dvec3 Terrain::sampleNormal(double east, double north) const {
    // Central differences one grid spacing apart. Matching the sample spacing
    // rather than using a tiny epsilon keeps the normal consistent with the
    // bilinear surface actually being intersected.
    const double d = spacing_m_;
    const double hx = sampleHeight(east + d, north) - sampleHeight(east - d, north);
    const double hy = sampleHeight(east, north + d) - sampleHeight(east, north - d);
    return glm::normalize(glm::dvec3(-hx / (2.0 * d), -hy / (2.0 * d), 1.0));
}

RayHit Terrain::raycast(const glm::dvec3& origin, const glm::dvec3& direction,
                        double max_distance) const {
    RayHit result;

    const double dir_len = glm::length(direction);
    if (dir_len < 1e-12) return result;
    const glm::dvec3 dir = direction / dir_len;

    // Step size scales with the terrain's own resolution: finer than one grid
    // cell wastes samples on detail the heightfield does not have, coarser
    // can step clean over a ridge and report the far slope instead of the
    // near one. Half a cell is the usual compromise.
    const double step = std::max(spacing_m_ * 0.5, 0.5);

    auto heightAbove = [&](double t) {
        const glm::dvec3 p = origin + dir * t;
        return p.z - sampleHeight(p.x, p.y);
    };

    double t_prev = 0.0;
    double f_prev = heightAbove(0.0);

    // Starting underground means the camera itself is below the surface -
    // physically meaningless here, and the sign-change search would never
    // trigger. Report the origin so the caller sees something sane.
    if (f_prev <= 0.0) {
        result.hit = true;
        result.point = origin;
        result.distance = 0.0;
        result.normal = sampleNormal(origin.x, origin.y);
        return result;
    }

    for (double t = step; t <= max_distance; t += step) {
        ++result.steps;
        const double f = heightAbove(t);

        if (f <= 0.0) {
            // Bisect the bracket [t_prev, t]. Fifty halvings take a
            // kilometre-wide bracket below a nanometre, so this terminates on
            // the iteration count rather than needing a tolerance that would
            // have to be tuned per terrain scale.
            double lo = t_prev, hi = t;
            for (int i = 0; i < 50; ++i) {
                const double mid = 0.5 * (lo + hi);
                if (heightAbove(mid) > 0.0) lo = mid; else hi = mid;
            }
            const double t_hit = 0.5 * (lo + hi);
            result.hit = true;
            result.distance = t_hit;
            result.point = origin + dir * t_hit;
            result.normal = sampleNormal(result.point.x, result.point.y);
            // Snap the reported height onto the surface: the bisection
            // converges to within nanometres, but the caller converts this to
            // a geodetic altitude, and reporting the exact surface value is
            // more honest than reporting the ray's.
            result.point.z = sampleHeight(result.point.x, result.point.y);
            result.left_coverage = !inCoverage(result.point.x, result.point.y);
            return result;
        }

        const glm::dvec3 p = origin + dir * t;
        if (!inCoverage(p.x, p.y)) {
            // Outside the modelled square the surface is a flat extrapolation
            // of the edge, so anything found past here would be fiction.
            result.left_coverage = true;
            return result;
        }

        t_prev = t;
        f_prev = f;
    }

    (void)f_prev;
    return result;  // ran out of range without ever descending to the ground
}

}  // namespace georay
