#pragma once

// WGS84 geodetic <-> ECEF <-> local ENU conversions.
//
// Everything in this file is double precision and stays double precision.
// Single precision has ~7 significant digits, and an ECEF coordinate is
// ~6.4e6 metres, so a float ECEF value quantises to roughly half a metre -
// larger than the geolocation error this tool exists to study. Floats appear
// only where coordinates are handed to OpenGL, and only after they have been
// made local (ENU) so the magnitudes are small.

#include <glm/glm.hpp>

namespace georay {

// WGS84 reference ellipsoid.
namespace wgs84 {
constexpr double semi_major_axis = 6378137.0;            // a, metres
constexpr double flattening      = 1.0 / 298.257223563;  // f
constexpr double semi_minor_axis = semi_major_axis * (1.0 - flattening);
constexpr double eccentricity_sq = flattening * (2.0 - flattening);
}  // namespace wgs84

// A position on/above the ellipsoid. Angles in degrees because that is how
// every GPS receiver, flight log and map UI states them; radians only ever
// appear inside the conversion functions.
struct Geodetic {
    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;
    double altitude_m    = 0.0;  // height above the ellipsoid, not above ground

    Geodetic() = default;
    Geodetic(double lat_deg, double lon_deg, double alt_m)
        : latitude_deg(lat_deg), longitude_deg(lon_deg), altitude_m(alt_m) {}
};

// Earth-centred, Earth-fixed metres.
glm::dvec3 geodeticToEcef(const Geodetic& g);

// Inverse of the above. Uses Bowring's closed form, which is exact enough
// (sub-micrometre) for any altitude this tool deals with and, unlike the
// naive fixed-point iteration, needs no convergence loop or iteration cap.
Geodetic ecefToGeodetic(const glm::dvec3& ecef);

// A local tangent-plane frame: east, north, up in metres from an origin.
//
// The whole simulation runs in one of these. A UAV scene spans a few
// kilometres at most, where the difference between the tangent plane and the
// curved ellipsoid is centimetres, so working locally costs nothing and buys
// coordinates small enough to render and to reason about.
class EnuFrame {
public:
    EnuFrame() = default;
    explicit EnuFrame(const Geodetic& origin);

    const Geodetic& origin() const { return origin_; }

    glm::dvec3 fromEcef(const glm::dvec3& ecef) const;
    glm::dvec3 toEcef(const glm::dvec3& enu) const;

    glm::dvec3 fromGeodetic(const Geodetic& g) const { return fromEcef(geodeticToEcef(g)); }
    Geodetic   toGeodetic(const glm::dvec3& enu) const { return ecefToGeodetic(toEcef(enu)); }

    // Rotation taking ENU vectors to ECEF vectors (no translation).
    const glm::dmat3& rotationEcefFromEnu() const { return ecef_from_enu_; }

private:
    Geodetic   origin_;
    glm::dvec3 origin_ecef_{0.0};
    glm::dmat3 ecef_from_enu_{1.0};
};

// Great-circle-ish horizontal distance between two geodetic points, computed
// through the local ENU frame of the first. Only meaningful at the scales
// this tool works at (kilometres), which is all it is used for - reporting
// how far a geolocated pixel is from the aircraft.
double horizontalDistance(const Geodetic& a, const Geodetic& b);

inline double deg2rad(double d) { return d * (3.14159265358979323846 / 180.0); }
inline double rad2deg(double r) { return r * (180.0 / 3.14159265358979323846); }

}  // namespace georay
