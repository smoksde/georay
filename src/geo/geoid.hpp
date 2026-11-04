#pragma once

// Geoid separation: the gap between the WGS84 ellipsoid and mean sea level.
//
// GPS reports height above the *ellipsoid*. A DEM (SRTM, ASTER, most
// national products) reports height above the *geoid* - "metres above sea
// level" on a map. Near Munich the two differ by about 47 m.
//
// Ignoring the difference looks fine - terrain renders, rays hit it,
// latitude/longitude are correct - but every height is off by the
// separation: slant range, footprint, geolocated elevation, all silently
// wrong, and the aircraft silently flies 47 m lower over the ground than
// commanded.

#include <cstdint>
#include <string>
#include <vector>

namespace georay {

class GeoidModel {
public:
    // No correction. Heights stay orthometric; the HUD says so rather than
    // letting it pass silently.
    GeoidModel() = default;

    // A single separation for the whole scene. The geoid varies about a
    // centimetre per kilometre, so a constant taken at the origin is good to
    // a few centimetres across a scene tens of km wide - well under SRTM's
    // own ~6 m vertical accuracy. The grid below only matters at continental
    // scale.
    static GeoidModel constant(double separation_m);

    // A gridded model in georay's own binary format, written by
    // scripts/fetch_dem.py from a standard geoid file (PGM, NGA ASCII grid,
    // ...) - that parsing happens once in Python so C++ never has to know
    // those formats.
    static bool loadGrid(const std::string& path, GeoidModel& out, std::string& error);

    // Ellipsoidal height of a point whose orthometric height is known.
    double toEllipsoidal(double lat_deg, double lon_deg, double orthometric_m) const {
        return orthometric_m + separationAt(lat_deg, lon_deg);
    }

    // Separation N, positive where the geoid is above the ellipsoid.
    double separationAt(double lat_deg, double lon_deg) const;

    bool isIdentity() const { return !gridded_ && constant_m_ == 0.0; }
    bool isGridded() const { return gridded_; }
    // One line fit for a HUD row.
    std::string description() const;

private:
    bool gridded_ = false;
    double constant_m_ = 0.0;

    // Grid layout: row 0 is the northern edge, column 0 the western one.
    int rows_ = 0, cols_ = 0;
    double lat_min_ = 0.0, lat_max_ = 0.0, lon_min_ = 0.0, lon_max_ = 0.0;
    double scale_ = 1.0, offset_ = 0.0;
    std::vector<int16_t> samples_;
    std::string source_;
};

}  // namespace georay
