#pragma once

// Geoid separation: the gap between the WGS84 ellipsoid and mean sea level.
//
// This exists because the two height systems in play are not the same system.
// A GPS receiver reports height above the *ellipsoid*. A DEM - SRTM, ASTER,
// most national products - reports height above the *geoid*, which is what
// "metres above sea level" means on a map. Near Munich they differ by about
// 47 metres.
//
// Ignore it and nothing looks broken: the terrain renders, the rays hit it,
// the latitudes and longitudes are even correct. Only the heights are wrong,
// by the separation - and since the aircraft's altitude is ellipsoidal while
// the ground is orthometric, the aircraft ends up flying 47 m lower over the
// terrain than commanded, which quietly changes every slant range, every
// footprint and every geolocated elevation.
//
// Two models are offered, and for this tool the simple one is usually the
// right one - see Constant below.

#include <cstdint>
#include <string>
#include <vector>

namespace georay {

class GeoidModel {
public:
    // No correction at all. Heights stay orthometric, which is wrong but at
    // least wrong in a single stated way; the application says so in the HUD
    // rather than letting it pass silently.
    GeoidModel() = default;

    // A single separation for the whole scene.
    //
    // This is not a shortcut. The geoid is a long-wavelength surface: it
    // varies by roughly a centimetre per kilometre, so across a scene tens of
    // kilometres wide a constant taken at the origin is good to a few
    // centimetres - far below the metre-scale errors this tool exists to
    // study, and below the DEM's own vertical accuracy (SRTM is about 6 m).
    // The grid below only earns its keep over continental distances.
    static GeoidModel constant(double separation_m);

    // A gridded model in georay's own format, written by
    // scripts/fetch_dem.py from a standard geoid file.
    //
    // georay reads one dead-simple binary rather than any of the half-dozen
    // formats geoid models actually ship in (PGM with metadata in comments,
    // NGA's ASCII grid, ...). Parsing those is fiddly, version-dependent
    // work that Python does once at fetch time and C++ then never has to
    // know about.
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
