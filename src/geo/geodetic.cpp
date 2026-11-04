#include "geo/geodetic.hpp"

#include <cmath>

namespace georay {

glm::dvec3 geodeticToEcef(const Geodetic& g) {
    const double lat = deg2rad(g.latitude_deg);
    const double lon = deg2rad(g.longitude_deg);
    const double sin_lat = std::sin(lat), cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon), cos_lon = std::cos(lon);

    // Radius of curvature in the prime vertical.
    const double n = wgs84::semi_major_axis /
                     std::sqrt(1.0 - wgs84::eccentricity_sq * sin_lat * sin_lat);

    return {
        (n + g.altitude_m) * cos_lat * cos_lon,
        (n + g.altitude_m) * cos_lat * sin_lon,
        (n * (1.0 - wgs84::eccentricity_sq) + g.altitude_m) * sin_lat,
    };
}

Geodetic ecefToGeodetic(const glm::dvec3& ecef) {
    const double a = wgs84::semi_major_axis;
    const double b = wgs84::semi_minor_axis;
    const double e_sq = wgs84::eccentricity_sq;
    // Second eccentricity squared: e'^2 = (a^2 - b^2) / b^2.
    const double ep_sq = (a * a - b * b) / (b * b);

    const double p = std::hypot(ecef.x, ecef.y);

    // Exactly on the polar axis: longitude is undefined and the formula below
    // divides by p. Pick longitude 0 and take latitude from the sign of z.
    if (p < 1e-9) {
        const double sign = ecef.z >= 0.0 ? 1.0 : -1.0;
        return Geodetic(sign * 90.0, 0.0, std::fabs(ecef.z) - b);
    }

    const double lon = std::atan2(ecef.y, ecef.x);

    // Bowring's closed form from the reduced latitude beta: good to a couple
    // of micrometres near the surface but degrades with altitude, so used as
    // a seed for the refinement below rather than the answer.
    const double beta = std::atan2(a * ecef.z, b * p);
    const double num  = ecef.z + b * ep_sq * std::pow(std::sin(beta), 3);
    const double den  = p - a * e_sq * std::pow(std::cos(beta), 3);
    double lat = std::atan2(num, den);

    // Altitude, from whichever expression is better conditioned: near the
    // equator cos(lat) ~ 1 and the horizontal one is stable, near the poles
    // it tends to zero and the vertical one takes over.
    auto altitudeAt = [&](double latitude, double n) {
        const double cos_lat = std::cos(latitude), sin_lat = std::sin(latitude);
        return std::fabs(cos_lat) > 0.5 ? p / cos_lat - n
                                        : ecef.z / sin_lat - n * (1.0 - e_sq);
    };

    double alt = 0.0;
    // Fixed-point steps on tan(lat) = z / (p (1 - e^2 N/(N+h))): from
    // Bowring's seed this converges to the double-precision floor
    // immediately, removing altitude as a source of error.
    for (int i = 0; i < 3; ++i) {
        const double sin_lat = std::sin(lat);
        const double n = a / std::sqrt(1.0 - e_sq * sin_lat * sin_lat);
        alt = altitudeAt(lat, n);
        lat = std::atan2(ecef.z, p * (1.0 - e_sq * n / (n + alt)));
    }
    {
        const double sin_lat = std::sin(lat);
        const double n = a / std::sqrt(1.0 - e_sq * sin_lat * sin_lat);
        alt = altitudeAt(lat, n);
    }

    return Geodetic(rad2deg(lat), rad2deg(lon), alt);
}

EnuFrame::EnuFrame(const Geodetic& origin) : origin_(origin) {
    origin_ecef_ = geodeticToEcef(origin);

    const double lat = deg2rad(origin.latitude_deg);
    const double lon = deg2rad(origin.longitude_deg);
    const double sin_lat = std::sin(lat), cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon), cos_lon = std::cos(lon);

    // Columns are the ENU basis vectors expressed in ECEF. glm is
    // column-major, so dmat3(c0, c1, c2) builds exactly that.
    const glm::dvec3 east { -sin_lon,            cos_lon,           0.0     };
    const glm::dvec3 north{ -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat };
    const glm::dvec3 up   {  cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat };

    ecef_from_enu_ = glm::dmat3(east, north, up);
}

glm::dvec3 EnuFrame::fromEcef(const glm::dvec3& ecef) const {
    // The inverse of a rotation is its transpose, which is far cheaper and
    // numerically cleaner than a general 3x3 inverse.
    return glm::transpose(ecef_from_enu_) * (ecef - origin_ecef_);
}

glm::dvec3 EnuFrame::toEcef(const glm::dvec3& enu) const {
    return ecef_from_enu_ * enu + origin_ecef_;
}

double horizontalDistance(const Geodetic& a, const Geodetic& b) {
    const EnuFrame frame(a);
    const glm::dvec3 enu = frame.fromGeodetic(b);
    return std::hypot(enu.x, enu.y);
}

}  // namespace georay
