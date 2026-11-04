// The two invariants the streaming ground depends on: a geolocation must not
// change when the floating origin re-anchors underneath it, and a Mercator
// box's UVs must not depend on the zoom used to compute them - that is what
// lets a chunk be re-skinned at a finer zoom without rebuilding its mesh.

#include "selftest_support.hpp"

#include <algorithm>
#include <cmath>

#include "geo/dem.hpp"
#include "geo/geoid.hpp"
#include "geo/geolocate.hpp"
#include "geo/mercator.hpp"

namespace georay {

// The floating origin's correctness condition, and the whole reason it is
// safe: geolocating the same pixel from the same aircraft must give the same
// place on Earth no matter where the ENU frame happens to be anchored.
//
// If that fails, flying across a region silently changes the answers - and
// nothing on screen would show it, because every frame looks locally correct.
void testFloatingOrigin() {
    std::printf("floating origin\n");

    const std::string directory = makeTempDir();
    check(!directory.empty(), "temporary directory for the floating-origin tests");
    if (directory.empty()) return;

    // A DEM with a known smooth slope, so the terrain is the same ground
    // whichever window is resampled out of it.
    const int side = 61;
    std::vector<int16_t> sloped(size_t(side * side));
    for (int r = 0; r < side; ++r) {
        for (int c = 0; c < side; ++c) {
            sloped[size_t(r * side + c)] = int16_t(1700 - 20 * r + 10 * c);
        }
    }
    writeTestHgt(directory, "N48E011", side, sloped);

    const GeoidModel geoid = GeoidModel::constant(47.0);
    const CameraModel cam = CameraModel::fromSensor(12.0, 13.2, 1920, 1080);

    // One aircraft, one pixel, three different frame anchors - the kind of
    // spread a flight would produce between re-anchors.
    // Comfortably above the ground, which at this spot is about 1447 m
    // ellipsoidal. An aircraft below the surface is a separate case: the ray
    // marcher returns the camera's own position immediately, and that answer
    // is frame-independent for trivial reasons - it would satisfy every
    // assertion below while testing none of them.
    UavState state;
    state.position = Geodetic(48.5, 11.5, 1900.0);
    state.attitude = {35.0, 2.0, 4.0};
    state.gimbal = {-20.0, 55.0, 0.0};
    state.camera_offset_body = glm::dvec3(0.25, 0.0, 0.35);
    const glm::dvec2 pixel(1310.0, 690.0);

    const Geodetic anchors[] = {
        Geodetic(48.5, 11.5, 0.0),        // directly beneath the aircraft
        Geodetic(48.492, 11.487, 0.0),    // about 1.5 km away
        Geodetic(48.518, 11.528, 0.0),    // about 2.5 km the other way
    };

    Geodetic first_answer;
    bool have_first = false;
    double worst_shift = 0.0;

    for (const Geodetic& anchor : anchors) {
        Geodetic origin = anchor;
        DemSource dem(directory);
        double ground = 0.0;
        check(dem.sampleOrthometric(origin.latitude_deg, origin.longitude_deg, ground),
              "the anchor is covered by the DEM");
        origin.altitude_m = geoid.toEllipsoidal(origin.latitude_deg, origin.longitude_deg, ground);

        const EnuFrame frame(origin);
        Terrain terrain(Terrain::Kind::Flat, 3000.0, 257, 0.0);
        check(terrain.loadFromDem(dem, geoid, frame).ok, "the window loads at this anchor");

        const GeolocateResult result = geolocatePixel(state, cam, terrain, frame, pixel);
        check(result.valid, "the pixel finds ground at this anchor");
        if (!result.valid) continue;
        // The ray has to have actually travelled. Without this the test can
        // pass on the underground early-out, which is how it was vacuous the
        // first time it was written.
        check(result.slant_range_m > 100.0,
              "the ray travelled a real distance, so this is not the underground case");

        if (!have_first) {
            first_answer = result.ground_geodetic;
            have_first = true;
            continue;
        }
        // Compare as a distance on the ground, which is what actually
        // matters, rather than as degrees.
        worst_shift = std::max(worst_shift, horizontalDistance(first_answer,
                                                               result.ground_geodetic));
        checkNear(result.ground_geodetic.altitude_m - first_answer.altitude_m, 0.0, 0.5,
                  "re-anchoring does not change the reported elevation");
    }

    check(have_first, "the floating-origin comparison actually ran");
    // Not bit-identical, and it cannot be: each anchor resamples the DEM onto
    // a differently-placed grid, so the interpolated surface differs slightly
    // between windows. Half a metre over terrain sloping 1.2 km per degree is
    // that resampling, not a frame error - a genuine frame bug moves the
    // answer by the distance between anchors, which is kilometres.
    checkNear(worst_shift, 0.0, 0.5, "re-anchoring moves a geolocation by less than a metre");

    // And the tangent-plane drop that makes all of this necessary: flat
    // ground falls away as d^2/2R, which is what caps a single frame.
    {
        const Geodetic origin(52.5, 13.4, 100.0);
        const EnuFrame frame(origin);
        for (double distance : {2400.0, 25000.0, 120000.0}) {
            // A point on the ellipsoid at the same height, that far east.
            const double degrees = distance / (111320.0 * std::cos(deg2rad(52.5)));
            const glm::dvec3 enu =
                frame.fromGeodetic(Geodetic(52.5, 13.4 + degrees, origin.altitude_m));
            const double expected = -distance * distance / (2.0 * 6371000.0);
            checkNear(enu.z, expected, std::fabs(expected) * 0.05 + 0.01,
                      "flat ground falls away from the tangent plane as d^2/2R");
        }
    }
}

// The projection the whole map pipeline now rests on.
//
// Texture coordinates come from a geographic box rather than from whichever
// tiles a map was stitched out of. That is what lets the map be re-stitched
// at a finer zoom without rebuilding the terrain mesh - and it only holds if
// a box's UVs really are independent of the zoom it is filled at.
void testMercator() {
    std::printf("web mercator and the map box\n");

    // Identities of the projection itself.
    for (int zoom : {0, 1, 12, 18}) {
        const double n = std::pow(2.0, double(zoom));
        checkNear(mercatorTile(0.0, -180.0, zoom).x, 0.0, 1e-9, "longitude -180 is tile 0");
        checkNear(mercatorTile(0.0, 0.0, zoom).x, n * 0.5, 1e-9, "the prime meridian is halfway");
        checkNear(mercatorTile(0.0, 180.0, zoom).x, n, 1e-9, "longitude 180 is the last tile");
        checkNear(mercatorTile(0.0, 0.0, zoom).y, n * 0.5, 1e-9, "the equator is halfway down");
        // Symmetric about the equator even though it is not linear in latitude.
        for (double lat : {10.0, 52.5, 84.0}) {
            checkNear(mercatorTile(lat, 0.0, zoom).y + mercatorTile(-lat, 0.0, zoom).y, n, 1e-9,
                      "mercator is symmetric about the equator");
        }
        check(mercatorTile(52.5, 0.0, zoom).y < n * 0.5, "northern latitudes are above halfway");
    }

    // Round trip through the inverse.
    Rng rng;
    double worst = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const double lat = rng.range(-84.0, 84.0), lon = rng.range(-179.9, 179.9);
        const glm::dvec2 t = mercatorTile(lat, lon, 14);
        worst = std::max(worst, std::fabs(mercatorLatitude(t.y, 14) - lat));
        worst = std::max(worst, std::fabs(mercatorLongitude(t.x, 14) - lon));
    }
    checkNear(worst, 0.0, 1e-9, "mercator round trip (worst, degrees)");

    // Ground resolution, and the two zoom choosers.
    checkNear(mercatorMetresPerPixel(0.0, 0), 156543.03392, 1e-3, "resolution at the equator");
    checkNear(mercatorMetresPerPixel(52.5163, 12), 23.26, 0.02, "resolution at z12 in Berlin");
    checkNear(mercatorMetresPerPixel(52.5163, 18), mercatorMetresPerPixel(52.5163, 12) / 64.0,
              1e-9, "each zoom halves the ground a pixel covers");

    // zoomForResolution rounds towards finer, zoomNotFinerThan towards
    // coarser. A single rounding would either blur the map or quadruple the
    // tiles fetched for a picture that is then averaged back down.
    for (int zoom : {11, 13, 17}) {
        const double exact = mercatorMetresPerPixel(52.5, zoom);
        checkNear(zoomForResolution(52.5, exact, 0, 22), zoom, 0, "exact resolution is its zoom");
        checkNear(zoomNotFinerThan(52.5, exact, 0, 22), zoom, 0, "...from either direction");
        // Between two zooms the two disagree by exactly one step.
        const double between = exact * 0.7;
        check(zoomForResolution(52.5, between, 0, 22) == zoom + 1,
              "a finer want rounds up to the finer zoom");
        check(zoomNotFinerThan(52.5, between, 0, 22) == zoom,
              "...while the budget rounds down to the coarser one");
    }
    check(zoomForResolution(52.5, 0.01, 0, 15) == 15, "the source's ceiling is respected");
    check(zoomNotFinerThan(52.5, 1e9, 8, 22) == 8, "...and its floor");

    // The box: UVs from bounds alone.
    {
        const MercatorBox box = MercatorBox::around(52.5163, 13.3777, 2000.0);
        check(box.valid(), "a box around a position is valid");
        checkNear(box.uvFor(box.north(), box.west()).x, 0.0, 1e-12, "north-west corner is u=0");
        checkNear(box.uvFor(box.north(), box.west()).y, 0.0, 1e-12, "...and v=0");
        checkNear(box.uvFor(box.south(), box.east()).x, 1.0, 1e-12, "south-east corner is u=1");
        checkNear(box.uvFor(box.south(), box.east()).y, 1.0, 1e-12, "...and v=1");
        check(box.contains(52.5163, 13.3777), "the box contains its own centre");
        check(!box.contains(52.5163, 14.5), "...and not somewhere else");
        check(box.north() > box.south() && box.east() > box.west(), "bounds are the right way up");

        // v is not linear in latitude - the whole reason this is done in
        // Mercator rather than by interpolating degrees.
        const double mid_lat = 0.5 * (box.north() + box.south());
        const double linear_guess = 0.5;
        const double actual = box.uvFor(mid_lat, box.west()).y;
        check(std::fabs(actual - linear_guess) > 1e-7,
              "v is not linear in latitude, even across 4 km");

        // THE invariant: the same box, described at any internal zoom, gives
        // the same UVs. If this failed, re-stitching at a finer zoom would
        // slide the map out from under the terrain mesh.
        const MercatorBox same = MercatorBox::fromBounds(box.south(), box.north(), box.west(),
                                                         box.east(), 12);
        double worst_uv = 0.0;
        for (int i = 0; i < 500; ++i) {
            const double lat = rng.range(box.south(), box.north());
            const double lon = rng.range(box.west(), box.east());
            worst_uv = std::max(worst_uv, glm::length(box.uvFor(lat, lon) - same.uvFor(lat, lon)));
        }
        checkNear(worst_uv, 0.0, 1e-9, "a box's UVs do not depend on the zoom it is held in");
    }

    // Tile ranges: which tiles a box needs, and that the count stays bounded.
    {
        const MercatorBox box = MercatorBox::around(52.5163, 13.3777, 2000.0);
        for (int zoom : {10, 14, 18}) {
            int x0, y0, x1, y1;
            box.tileRange(zoom, x0, y0, x1, y1);
            check(x1 >= x0 && y1 >= y0, "a tile range is never inverted");
            const int n = int(std::pow(2.0, double(zoom)));
            check(x0 >= 0 && y0 >= 0 && x1 < n && y1 < n, "a tile range stays on the map");

            // Every corner of the box must fall inside the range, or the
            // stitched texture would have an uncovered edge.
            const double corners[4][2] = {{box.north(), box.west()},
                                          {box.north(), box.east()},
                                          {box.south(), box.west()},
                                          {box.south(), box.east()}};
            for (const auto& corner : corners) {
                const glm::dvec2 t = mercatorTile(corner[0], corner[1], zoom);
                check(t.x >= double(x0) - 1e-9 && t.x <= double(x1 + 1) + 1e-9 &&
                          t.y >= double(y0) - 1e-9 && t.y <= double(y1 + 1) + 1e-9,
                      "the tile range covers every corner of the box");
            }
        }
    }
}

}  // namespace georay
