#pragma once

// The ground model a camera ray gets intersected against.
//
// This is a regular grid of heights with bilinear interpolation between
// posts - deliberately the same shape as a real DEM tile (SRTM, Copernicus,
// a GeoTIFF), so swapping synthetic terrain for surveyed data means filling
// the same array from a file and changing nothing else. The analytic
// generator exists only to have something interesting to look at and to make
// the parallax effects of real relief visible.
//
// Heights and positions are ENU metres, in the same local frame the rest of
// the simulation uses - so a height of 0 is the frame origin's altitude, not
// sea level. Rays are traced in ENU, so keeping the ground model there too
// means the intersection never has to change frames mid-solve.
//
// Loading a real DEM therefore means pushing each post through
// EnuFrame::fromGeodetic() rather than copying raw elevations in. That is
// not busywork: it is what applies the tangent-plane curvature correction
// (about 8 cm per kilometre from the origin, growing quadratically), and it
// is where an orthometric DEM's geoid offset would be added.

#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace georay {

// What a ray hit, or why it did not.
struct RayHit {
    bool hit = false;
    glm::dvec3 point{0.0};  // ENU metres
    double distance = 0.0;  // along the ray from its origin, metres
    glm::dvec3 normal{0.0, 0.0, 1.0};
    int steps = 0;          // marching steps used; a diagnostic, not a result

    // Set when the ray left the modelled area rather than missing the
    // ground. The distinction matters: a ray that exits the DEM tile is an
    // unknown, not an answer, and a tool that reports it as "no ground" will
    // happily mislead you at the edge of coverage.
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

    // Height at an east/north position, bilinearly interpolated. Positions
    // outside the modelled square clamp to the edge, which keeps the surface
    // closed; use inCoverage() to tell whether an answer is real.
    double sampleHeight(double east, double north) const;
    glm::dvec3 sampleNormal(double east, double north) const;
    bool inCoverage(double east, double north) const;

    // Intersect a ray (origin + t * direction, t >= 0) with the surface.
    //
    // Marches in steps and bisects the first sign change of (ray height -
    // ground height). Ray marching rather than an analytic solve because a
    // sampled heightfield has no analytic form - and bisection afterwards
    // because the marching step has to be coarse enough to cross kilometres
    // without thousands of samples, which on its own would quantise the hit
    // to the step size.
    RayHit raycast(const glm::dvec3& origin, const glm::dvec3& direction,
                   double max_distance = 50000.0) const;

    double halfExtent() const { return half_extent_m_; }
    int resolution() const { return resolution_; }
    double baseElevation() const { return base_elevation_m_; }
    double minHeight() const { return min_height_; }
    double maxHeight() const { return max_height_; }

    // Raw grid access, for building the render mesh without re-interpolating.
    double heightAt(int ix, int iy) const { return heights_[size_t(iy) * resolution_ + ix]; }
    glm::dvec2 gridPosition(int ix, int iy) const;

private:
    void updateExtrema();

    Kind kind_ = Kind::Hills;
    double half_extent_m_ = 3000.0;
    int resolution_ = 513;
    double base_elevation_m_ = 0.0;
    double spacing_m_ = 1.0;  // metres between grid posts
    std::vector<double> heights_;
    double min_height_ = 0.0, max_height_ = 0.0;
};

}  // namespace georay
