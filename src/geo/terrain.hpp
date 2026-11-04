#pragma once

// The ground model a camera ray gets intersected against.
//
// A regular grid of heights, bilinearly interpolated between posts -
// deliberately the same shape as a real DEM tile (SRTM, Copernicus, a
// GeoTIFF), so swapping synthetic terrain for surveyed data means filling
// the same array from a file. The analytic generator exists only to give
// synthetic relief something to show parallax against.
//
// Heights and positions are ENU metres: 0 is the frame origin's altitude,
// not sea level. Loading a real DEM means pushing each post through
// EnuFrame::fromGeodetic() rather than copying elevations in directly - that
// is what applies the tangent-plane curvature correction (~8 cm/km from the
// origin, growing quadratically) and adds the orthometric DEM's geoid
// offset. loadFromDem() below does exactly that; a plain subtraction instead
// would drop the curvature term and bake a dome into the terrain.

#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "geo/dem.hpp"
#include "geo/geodetic.hpp"
#include "geo/geoid.hpp"

namespace georay {

// What a ray hit, or why it did not.
struct RayHit {
    bool hit = false;
    glm::dvec3 point{0.0};  // ENU metres
    double distance = 0.0;  // along the ray from its origin, metres
    glm::dvec3 normal{0.0, 0.0, 1.0};
    int steps = 0;          // marching steps used; a diagnostic, not a result

    // Set when the ray left the modelled area rather than missing the
    // ground - an unknown, not an answer, and conflating the two misleads at
    // the edge of coverage.
    bool left_coverage = false;
};

class Terrain {
public:
    enum class Kind {
        Flat,      // a single constant elevation: the classic flat-earth assumption
        Hills,     // smooth rolling relief
        Valley,    // a river valley with flanking ridges, for strong parallax
        Mountains  // steep ridges, where terrain-aware intersection really shows
    };

    // `half_extent_m` is the distance from the centre to each edge, so the
    // modelled square is 2*half_extent on a side. `resolution` is the number
    // of grid posts per side.
    // `base_elevation_m` is the ENU height the relief is added on top of,
    // i.e. the ground level of a Kind::Flat terrain.
    Terrain(Kind kind = Kind::Hills, double half_extent_m = 3000.0, int resolution = 513,
            double base_elevation_m = 0.0);

    void generate(Kind kind);
    Kind kind() const { return kind_; }
    static const char* kindName(Kind kind);

    // What produced the current heights, fit for a HUD row: "HILLS", or
    // "SRTM N48E011". The synthetic Kind alone cannot answer that once real
    // data is in play.
    const std::string& sourceName() const { return source_name_; }

    // How a DEM load went - a fifth of its posts invented is a different
    // claim from none, and the user has to see which they're looking at.
    struct DemLoadReport {
        bool ok = false;
        int posts_total = 0;
        int posts_from_data = 0;   // sampled directly from the DEM
        int posts_filled = 0;      // interpolated across voids or missing tiles
        std::vector<std::string> tiles_used;
        std::vector<std::string> tiles_missing;
        double origin_orthometric_m = 0.0;  // ground at the frame origin, per the DEM
        double origin_separation_m = 0.0;   // geoid separation applied there
        std::string error;

        double gapFraction() const {
            return posts_total > 0 ? double(posts_filled) / double(posts_total) : 0.0;
        }
    };

    // Replace the heights with real elevation data, resampled onto this
    // terrain's existing ENU grid rather than the DEM's own lat/lon grid -
    // so the ray marcher and renderer keep working on the structure they
    // already understand. Costs nothing in fidelity: at a few hundred posts
    // across a few km, georay's grid is finer than SRTM's ~30 m posting.
    DemLoadReport loadFromDem(const DemSource& dem, const GeoidModel& geoid,
                              const EnuFrame& frame);

    // Height at an east/north position, bilinearly interpolated. Positions
    // outside the modelled square clamp to the edge, which keeps the surface
    // closed; use inCoverage() to tell whether an answer is real.
    double sampleHeight(double east, double north) const;
    glm::dvec3 sampleNormal(double east, double north) const;
    bool inCoverage(double east, double north) const;

    // Intersect a ray (origin + t * direction, t >= 0) with the surface:
    // marches in steps and bisects the first sign change of (ray height -
    // ground height). Marching rather than an analytic solve because a
    // sampled heightfield has none; bisection because a marching step coarse
    // enough to cross kilometres without thousands of samples would
    // otherwise quantise the hit to the step size.
    RayHit raycast(const glm::dvec3& origin, const glm::dvec3& direction,
                   double max_distance = 50000.0) const;

    double halfExtent() const { return half_extent_m_; }
    int resolution() const { return resolution_; }
    double minHeight() const { return min_height_; }
    double maxHeight() const { return max_height_; }

    // Raw grid access, for building the render mesh without re-interpolating.
    double heightAt(int ix, int iy) const { return heights_[size_t(iy) * resolution_ + ix]; }
    glm::dvec2 gridPosition(int ix, int iy) const;

private:
    void updateExtrema();

    // Replaces missing posts by diffusing valid neighbours inwards; a post
    // still missing after a few passes takes the mean of whatever loaded.
    int fillGaps(std::vector<bool>& has_data);

    Kind kind_ = Kind::Hills;
    std::string source_name_ = "HILLS";
    double half_extent_m_ = 3000.0;
    int resolution_ = 513;
    double base_elevation_m_ = 0.0;
    double spacing_m_ = 1.0;  // metres between grid posts
    std::vector<double> heights_;
    double min_height_ = 0.0, max_height_ = 0.0;
};

}  // namespace georay
