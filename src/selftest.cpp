// Numeric verification of the geolocation pipeline, run with --selftest.
//
// This exists because every error in this tool is silent: a wrong sign in a
// rotation, a transposed matrix, a degrees/radians slip - none of them crash
// or even look wrong on screen. They produce a confident latitude and
// longitude that is simply somewhere else. The tests below pin down the
// conventions (which way is yaw, which way is tilt, which axis is north) with
// cases whose answers are known independently of the code under test.

#include "selftest.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "geo/dem.hpp"
#include "geo/geoid.hpp"
#include "geo/geolocate.hpp"

namespace georay {
namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool ok, const std::string& what) {
    ++g_checks;
    if (!ok) {
        ++g_failures;
        std::printf("  FAIL  %s\n", what.c_str());
    }
}

void checkNear(double got, double want, double tol, const std::string& what) {
    ++g_checks;
    const double err = std::fabs(got - want);
    // NaN fails every comparison, including err <= tol, so it lands here
    // rather than silently passing - which is the point, since a NaN
    // latitude is exactly the kind of failure this tool must not ship.
    if (!(err <= tol)) {
        ++g_failures;
        std::printf("  FAIL  %s: got %.9f want %.9f (err %.3g > %.3g)\n", what.c_str(), got,
                    want, err, tol);
    }
}

void checkVecNear(const glm::dvec3& got, const glm::dvec3& want, double tol,
                  const std::string& what) {
    ++g_checks;
    const double err = glm::length(got - want);
    if (!(err <= tol)) {
        ++g_failures;
        std::printf("  FAIL  %s: got (%.6f %.6f %.6f) want (%.6f %.6f %.6f) err %.3g\n",
                    what.c_str(), got.x, got.y, got.z, want.x, want.y, want.z, err);
    }
}

// A deterministic pseudo-random sequence, so a failure reported on one
// machine reproduces exactly on another.
struct Rng {
    unsigned int s = 0x9e3779b9u;
    double next() {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        return double(s) / double(0xFFFFFFFFu);
    }
    double range(double lo, double hi) { return lo + (hi - lo) * next(); }
};

// --- geodetic ------------------------------------------------------------

void testGeodetic() {
    std::printf("geodetic\n");

    // Independently known points on the ellipsoid.
    checkVecNear(geodeticToEcef(Geodetic(0, 0, 0)),
                 glm::dvec3(wgs84::semi_major_axis, 0, 0), 1e-6, "equator/prime meridian");
    checkVecNear(geodeticToEcef(Geodetic(0, 90, 0)),
                 glm::dvec3(0, wgs84::semi_major_axis, 0), 1e-6, "equator/90E");
    checkVecNear(geodeticToEcef(Geodetic(90, 0, 0)),
                 glm::dvec3(0, 0, wgs84::semi_minor_axis), 1e-6, "north pole");

    // Round trip over a spread of latitudes, longitudes and altitudes,
    // including the poles and negative altitude.
    Rng rng;
    double worst_pos = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const Geodetic g(rng.range(-89.999, 89.999), rng.range(-180.0, 180.0),
                         rng.range(-500.0, 12000.0));
        const Geodetic back = ecefToGeodetic(geodeticToEcef(g));
        // Compare as positions, not as angles: a longitude error near the
        // pole is geometrically meaningless, and comparing degrees there
        // would fail a conversion that is actually exact.
        const double err = glm::length(geodeticToEcef(back) - geodeticToEcef(g));
        worst_pos = std::max(worst_pos, err);
    }
    checkNear(worst_pos, 0.0, 1e-6, "ecef/geodetic round trip (worst, metres)");

    // Poles: the longitude is arbitrary but the altitude must still be right.
    checkNear(ecefToGeodetic(glm::dvec3(0, 0, wgs84::semi_minor_axis + 100.0)).altitude_m,
              100.0, 1e-6, "altitude on the polar axis");
    checkNear(ecefToGeodetic(glm::dvec3(0, 0, wgs84::semi_minor_axis + 100.0)).latitude_deg,
              90.0, 1e-9, "latitude on the polar axis");
}

void testEnuFrame() {
    std::printf("enu frame\n");

    const Geodetic origin(48.1372, 11.5756, 520.0);  // Munich, roughly
    const EnuFrame frame(origin);

    checkVecNear(frame.fromGeodetic(origin), glm::dvec3(0, 0, 0), 1e-6, "origin maps to zero");

    // The axes must point where their names say. Going north must increase y
    // and leave x alone; going east must increase x; going up must increase z.
    const glm::dvec3 north = frame.fromGeodetic(Geodetic(origin.latitude_deg + 0.001,
                                                         origin.longitude_deg,
                                                         origin.altitude_m));
    check(north.y > 100.0 && std::fabs(north.x) < 1e-3, "+latitude is +north, not +east");

    const glm::dvec3 east = frame.fromGeodetic(Geodetic(origin.latitude_deg,
                                                        origin.longitude_deg + 0.001,
                                                        origin.altitude_m));
    check(east.x > 50.0 && std::fabs(east.y) < 1.0, "+longitude is +east");

    const glm::dvec3 up = frame.fromGeodetic(Geodetic(origin.latitude_deg,
                                                      origin.longitude_deg,
                                                      origin.altitude_m + 250.0));
    checkVecNear(up, glm::dvec3(0, 0, 250.0), 1e-6, "+altitude is +up by exactly that much");

    // A metre east should be a metre, not 0.99 or 1.01 - the scale of the
    // tangent plane is what turns a pixel offset into a real distance.
    const glm::dvec3 one_metre = frame.fromGeodetic(frame.toGeodetic(glm::dvec3(1, 0, 0)));
    checkVecNear(one_metre, glm::dvec3(1, 0, 0), 1e-6, "enu round trip is metric");

    Rng rng;
    double worst = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const glm::dvec3 p(rng.range(-5000, 5000), rng.range(-5000, 5000), rng.range(-200, 3000));
        worst = std::max(worst, glm::length(frame.fromGeodetic(frame.toGeodetic(p)) - p));
    }
    checkNear(worst, 0.0, 1e-6, "enu/geodetic round trip (worst, metres)");
}

// --- camera --------------------------------------------------------------

void testCameraModel() {
    std::printf("camera model\n");

    CameraModel cam = CameraModel::fromSensor(24.0, 36.0, 1920, 1080);
    checkNear(cam.fx(), 24.0 * 1920.0 / 36.0, 1e-9, "fx from sensor geometry");
    checkNear(cam.fy(), cam.fx(), 1e-9, "square pixels give fy == fx");
    checkNear(cam.focalLengthMm(), 24.0, 1e-9, "focal length round trips through fx");

    // The principal point looks straight down the optical axis.
    checkVecNear(cam.pixelToRay({cam.cx(), cam.cy()}), glm::dvec3(0, 0, 1), 1e-12,
                 "centre pixel is the optical axis");

    // Image +x is optical +x, image +y is optical +y (down). Getting this
    // backwards flips the geolocation across the boresight.
    check(cam.pixelToRay({cam.cx() + 100, cam.cy()}).x > 0, "+u is optical +x");
    check(cam.pixelToRay({cam.cx(), cam.cy() + 100}).y > 0, "+v is optical +y (down)");

    cam.setHorizontalFovDeg(90.0);
    checkNear(cam.horizontalFovDeg(), 90.0, 1e-9, "horizontal fov round trip");
    checkNear(cam.fx(), 1920.0 * 0.5, 1e-6, "90 degree fov means fx == width/2");
    // The vertical field follows from the image aspect, not from the setter.
    checkNear(cam.verticalFovDeg(), rad2deg(2.0 * std::atan(540.0 / cam.fy())), 1e-9,
              "vertical fov follows the aspect ratio");

    // Resizing the image must not change what the camera sees.
    const double fov_before = cam.horizontalFovDeg();
    cam.setImageSize(960, 540);
    checkNear(cam.horizontalFovDeg(), fov_before, 1e-9, "resize preserves field of view");
    checkNear(cam.cx(), 480.0, 1e-9, "resize rescales the principal point");

    // Pixel -> ray -> pixel, everywhere in the frame, undistorted.
    Rng rng;
    double worst = 0.0;
    for (int i = 0; i < 5000; ++i) {
        const glm::dvec2 px(rng.range(0, cam.imageWidth()), rng.range(0, cam.imageHeight()));
        const glm::dvec2 back = cam.projectToPixel(cam.pixelToRay(px));
        worst = std::max(worst, glm::length(back - px));
    }
    checkNear(worst, 0.0, 1e-9, "pixel/ray round trip (worst, pixels)");
}

void testDistortion() {
    std::printf("lens distortion\n");

    CameraModel cam = CameraModel::fromSensor(16.0, 23.5, 1280, 720);
    cam.distortion() = {-0.28, 0.12, -0.02, 0.0009, -0.0012};

    // The inverse is numeric, so it is the round trip that has to be verified
    // rather than any individual value. Sub-thousandth-pixel accuracy over the
    // whole frame means the undistortion iteration is converging, not just
    // stopping.
    Rng rng;
    double worst = 0.0;
    for (int i = 0; i < 5000; ++i) {
        const glm::dvec2 px(rng.range(0, cam.imageWidth()), rng.range(0, cam.imageHeight()));
        const glm::dvec2 back = cam.projectToPixel(cam.pixelToRay(px));
        worst = std::max(worst, glm::length(back - px));
    }
    checkNear(worst, 0.0, 1e-6, "distorted pixel/ray round trip (worst, pixels)");

    // Distortion must actually be doing something, or the round trip above is
    // testing an identity and proves nothing.
    const glm::dvec2 corner{10.0, 10.0};
    CameraModel ideal = cam;
    ideal.distortion() = {};
    check(glm::length(cam.pixelToRay(corner) - ideal.pixelToRay(corner)) > 1e-3,
          "distortion measurably bends a corner ray");
    // ...and must not move the principal point, which is on the axis.
    checkVecNear(cam.pixelToRay({cam.cx(), cam.cy()}), glm::dvec3(0, 0, 1), 1e-12,
                 "distortion leaves the optical axis alone");
}

// --- pose ----------------------------------------------------------------

void testPose() {
    std::printf("pose chain\n");

    UavState s;
    s.attitude = {0, 0, 0};
    s.gimbal = {0, 90, 0};  // straight down
    checkVecNear(boresightEnu(s), glm::dvec3(0, 0, -1), 1e-12, "tilt 90 is nadir");

    s.gimbal = {0, 0, 0};  // straight ahead, nose north
    checkVecNear(boresightEnu(s), glm::dvec3(0, 1, 0), 1e-12, "yaw 0 / tilt 0 looks north");

    s.attitude.yaw_deg = 90.0;
    checkVecNear(boresightEnu(s), glm::dvec3(1, 0, 0), 1e-12, "yaw 90 looks east");

    s.attitude.yaw_deg = 180.0;
    checkVecNear(boresightEnu(s), glm::dvec3(0, -1, 0), 1e-12, "yaw 180 looks south");

    // Gimbal pan is measured from the nose and is positive to the right.
    s.attitude.yaw_deg = 0.0;
    s.gimbal = {90, 0, 0};
    checkVecNear(boresightEnu(s), glm::dvec3(1, 0, 0), 1e-12, "pan 90 looks right of the nose");

    // Pitch is nose-up positive.
    s.gimbal = {0, 0, 0};
    s.attitude = {0, 30, 0};
    check(boresightEnu(s).z > 0.0, "positive pitch raises the boresight");

    // Roll must not move a nadir boresight's azimuth, but must tilt a forward
    // one sideways - the standard way to catch a pitch/roll swap.
    s.attitude = {0, 0, 45};
    s.gimbal = {0, 0, 0};
    const glm::dvec3 rolled = boresightEnu(s);
    checkVecNear(rolled, glm::dvec3(0, 1, 0), 1e-12, "roll does not move a forward boresight");
    s.gimbal = {0, 90, 0};
    const glm::dvec3 rolled_nadir = boresightEnu(s);
    // Counterintuitive but correct, and worth pinning down because the
    // "obvious" expectation is the opposite: lean to your right and your
    // feet-down axis tilts to your LEFT. Nose north, right wing down, so a
    // belly-mounted nadir camera looks down and to the west.
    check(rolled_nadir.x < -0.5, "roll right swings a nadir boresight west");
    checkNear(rolled_nadir.y, 0.0, 1e-12, "...with no north/south component");

    // The optical frame's own axes, in the nadir case: image-right must be
    // the aircraft's right, image-down must be along the direction of flight.
    s.attitude = {0, 0, 0};
    s.gimbal = {0, 90, 0};
    const glm::dmat3 enu_from_optical = rotationEnuFromOptical(s);
    checkVecNear(enu_from_optical * glm::dvec3(1, 0, 0), glm::dvec3(1, 0, 0), 1e-12,
                 "nadir, nose north: image right is east");
    checkVecNear(enu_from_optical * glm::dvec3(0, 1, 0), glm::dvec3(0, -1, 0), 1e-12,
                 "nadir, nose north: image down is south");

    // Every rotation in the chain must be a proper rotation: orthonormal and
    // right-handed. A determinant of -1 would mirror the world, which looks
    // entirely plausible on screen and puts every geolocation on the wrong
    // side of the flight path.
    Rng rng;
    for (int i = 0; i < 500; ++i) {
        UavState t;
        t.attitude = {rng.range(-180, 180), rng.range(-80, 80), rng.range(-60, 60)};
        t.gimbal = {rng.range(-180, 180), rng.range(-20, 95), rng.range(-30, 30)};
        const glm::dmat3 r = rotationEnuFromOptical(t);
        checkNear(glm::determinant(r), 1.0, 1e-12, "chain determinant is +1");
        const glm::dmat3 should_be_identity = glm::transpose(r) * r;
        checkNear(glm::length(should_be_identity[0] - glm::dvec3(1, 0, 0)) +
                      glm::length(should_be_identity[1] - glm::dvec3(0, 1, 0)) +
                      glm::length(should_be_identity[2] - glm::dvec3(0, 0, 1)),
                  0.0, 1e-12, "chain is orthonormal");
    }

    // Aiming is the inverse of the chain, so it has to come back to itself.
    double worst = 0.0;
    for (int i = 0; i < 1000; ++i) {
        UavState t;
        t.attitude = {rng.range(-180, 180), rng.range(-45, 45), rng.range(-45, 45)};
        const glm::dvec3 want = glm::normalize(glm::dvec3(
            rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, -0.05)));
        t.gimbal = aimGimbalAlong(t.attitude, want);
        worst = std::max(worst, glm::length(boresightEnu(t) - want));
    }
    checkNear(worst, 0.0, 1e-9, "aimGimbalAlong inverts the chain (worst, unit vector)");
}

// --- terrain -------------------------------------------------------------

void testTerrain() {
    std::printf("terrain\n");

    const Terrain flat(Terrain::Kind::Flat, 2000.0, 129, 300.0);
    checkNear(flat.sampleHeight(0, 0), 300.0, 1e-12, "flat terrain is its base elevation");
    checkNear(flat.sampleHeight(1234.5, -876.25), 300.0, 1e-12, "flat terrain is flat off-grid");
    checkVecNear(flat.sampleNormal(100, -40), glm::dvec3(0, 0, 1), 1e-12, "flat normal is up");

    // Interpolation must pass exactly through the grid posts it interpolates.
    const Terrain hills(Terrain::Kind::Hills, 2000.0, 129, 300.0);
    double worst = 0.0;
    for (int iy = 0; iy < hills.resolution(); iy += 7) {
        for (int ix = 0; ix < hills.resolution(); ix += 7) {
            const glm::dvec2 p = hills.gridPosition(ix, iy);
            worst = std::max(worst, std::fabs(hills.sampleHeight(p.x, p.y) - hills.heightAt(ix, iy)));
        }
    }
    checkNear(worst, 0.0, 1e-9, "bilinear sampling is exact at grid posts");

    check(hills.maxHeight() - hills.minHeight() > 20.0, "hills have relief to intersect");
    check(hills.inCoverage(0, 0) && !hills.inCoverage(99999, 0), "coverage test");

    // A straight-down ray must land on the surface directly below, at the
    // sampled height - the one case with an answer that needs no raycasting.
    for (int i = 0; i < 50; ++i) {
        const double e = -1500.0 + i * 60.0, n = 900.0 - i * 31.0;
        const double ground = hills.sampleHeight(e, n);
        const RayHit h = hills.raycast({e, n, ground + 500.0}, {0, 0, -1});
        check(h.hit, "nadir ray hits");
        checkNear(h.point.z, ground, 1e-6, "nadir ray lands on the sampled height");
        checkNear(h.distance, 500.0, 1e-6, "nadir slant range equals the height above ground");
    }

    // Upward rays must miss rather than find something behind the camera: a
    // raycaster that solves for t without checking its sign reports a hit
    // behind you as a hit in front, and the result looks entirely normal.
    check(!hills.raycast({0, 0, 1000.0}, {0, 0, 1}).hit, "straight up misses");
    check(!hills.raycast({0, 0, 1000.0}, glm::normalize(glm::dvec3(0.2, 0.1, 1.0))).hit,
          "upward oblique misses");

    // The hit must be the FIRST crossing. On ridged terrain a step that is
    // too coarse jumps a near ridge and reports the far slope, which is both
    // wrong and completely believable on screen.
    const Terrain mountains(Terrain::Kind::Mountains, 3000.0, 513, 200.0);
    Rng rng;
    int tested = 0, occluded = 0;
    for (int i = 0; i < 200; ++i) {
        const glm::dvec3 origin(rng.range(-2000, 2000), rng.range(-2000, 2000),
                                mountains.maxHeight() + rng.range(50, 400));
        const glm::dvec3 dir = glm::normalize(glm::dvec3(
            rng.range(-1, 1), rng.range(-1, 1), rng.range(-1.0, -0.15)));
        const RayHit h = mountains.raycast(origin, dir);
        if (!h.hit) continue;
        ++tested;
        // Nothing along the ray before the hit may be underground.
        bool clean = true;
        for (int s = 1; s < 400; ++s) {
            const double t = h.distance * double(s) / 400.0;
            const glm::dvec3 p = origin + dir * t;
            if (p.z < mountains.sampleHeight(p.x, p.y) - 0.5) { clean = false; break; }
            if (p.z < mountains.sampleHeight(p.x, p.y) + 2.0) ++occluded;
        }
        check(clean, "raycast returns the first surface crossing, not a later one");
    }
    check(tested > 100, "the first-crossing test actually ran");
    (void)occluded;
}

// --- the pipeline --------------------------------------------------------

// Flat ground has a closed-form answer, so the full pipeline can be checked
// against arithmetic instead of against itself.
void testGeolocateFlatClosedForm() {
    std::printf("geolocate: flat ground, closed form\n");

    const Geodetic origin(48.1372, 11.5756, 500.0);
    const EnuFrame frame(origin);
    // Terrain heights are ENU, so ground level 0 means "the frame origin's
    // altitude", which here is 500 m.
    const Terrain flat(Terrain::Kind::Flat, 20000.0, 129, 0.0);
    const CameraModel cam = CameraModel::fromSensor(24.0, 36.0, 1280, 720);

    UavState s;
    s.position = Geodetic(origin.latitude_deg, origin.longitude_deg, 500.0 + 300.0);
    s.attitude = {0, 0, 0};
    s.gimbal = {0, 90, 0};
    s.camera_offset_body = glm::dvec3(0.0);  // keep the arithmetic exact

    // Nadir centre pixel: 300 m below the aircraft, at the aircraft's own
    // latitude and longitude.
    GeolocateResult r = geolocatePixel(s, cam, flat, frame, {cam.cx(), cam.cy()});
    check(r.valid, "nadir centre pixel finds ground");
    checkNear(r.slant_range_m, 300.0, 1e-6, "nadir slant range is the height above ground");
    checkNear(r.ground_range_m, 0.0, 1e-6, "nadir has no ground range");
    checkNear(r.depression_angle_deg, 90.0, 1e-9, "nadir depression is 90 degrees");
    checkNear(r.ground_geodetic.latitude_deg, s.position.latitude_deg, 1e-12,
              "nadir hit keeps the aircraft latitude");
    checkNear(r.ground_geodetic.longitude_deg, s.position.longitude_deg, 1e-12,
              "nadir hit keeps the aircraft longitude");
    checkNear(r.ground_geodetic.altitude_m, 500.0, 1e-6, "nadir hit is at ground elevation");
    // GSD at nadir is height/focal-length, exactly.
    checkNear(r.gsd_m, 300.0 / cam.fx(), 1e-9, "nadir gsd is range over focal length");

    // Off-centre pixel at nadir: offset on the ground is (u - cx) * H / fx,
    // and image +x is east with the nose north, image +y (down) is south.
    const glm::dvec2 off{cam.cx() + 200.0, cam.cy() + 100.0};
    r = geolocatePixel(s, cam, flat, frame, off);
    check(r.valid, "off-centre nadir pixel finds ground");
    checkNear(r.ground_enu.x, 200.0 * 300.0 / cam.fx(), 1e-6, "image +x maps east by u*H/fx");
    checkNear(r.ground_enu.y, -100.0 * 300.0 / cam.fy(), 1e-6, "image +y maps south by v*H/fy");

    // A 45 degree depression puts the target exactly one height away.
    s.gimbal = {0, 45, 0};
    r = geolocatePixel(s, cam, flat, frame, {cam.cx(), cam.cy()});
    check(r.valid, "45 degree tilt finds ground");
    checkNear(r.ground_range_m, 300.0, 1e-6, "45 degree depression: ground range == height");
    checkNear(r.ground_enu.y, 300.0, 1e-6, "...and it is 300 m north of the aircraft");
    checkNear(r.slant_range_m, 300.0 * std::sqrt(2.0), 1e-6, "...with a sqrt(2) slant range");
    checkNear(r.depression_angle_deg, 45.0, 1e-9, "depression angle is reported as 45");

    // Arbitrary poses against the analytic plane intersection.
    Rng rng;
    double worst = 0.0;
    int cases = 0;
    for (int i = 0; i < 3000; ++i) {
        UavState t;
        t.position = Geodetic(origin.latitude_deg, origin.longitude_deg,
                              500.0 + rng.range(60.0, 600.0));
        t.attitude = {rng.range(-180, 180), rng.range(-20, 20), rng.range(-20, 20)};
        t.gimbal = {rng.range(-90, 90), rng.range(25, 90), rng.range(-15, 15)};
        t.camera_offset_body = glm::dvec3(rng.range(-1, 1), rng.range(-1, 1), rng.range(0, 1));

        const glm::dvec2 px(rng.range(0, cam.imageWidth()), rng.range(0, cam.imageHeight()));
        const GeolocateResult got = geolocatePixel(t, cam, flat, frame, px);
        if (!got.valid) continue;
        ++cases;

        // Closed form: intersect the same ray with the plane z = 500.
        const glm::dvec3 o = cameraPositionEnu(t, frame);
        const glm::dvec3 d = glm::normalize(rotationEnuFromOptical(t) * cam.pixelToRay(px));
        if (d.z >= -1e-6) continue;
        const glm::dvec3 want = o + d * ((0.0 - o.z) / d.z);
        worst = std::max(worst, glm::length(got.ground_enu - want));
    }
    check(cases > 1500, "the closed-form comparison actually ran");
    checkNear(worst, 0.0, 1e-6, "pipeline matches plane intersection (worst, metres)");
}

// On real relief there is no closed form, so the check is the round trip:
// a pixel's ground point, projected back through the camera, must land on
// the pixel it came from. That catches any disagreement between the forward
// and inverse chains, which is where convention errors live.
void testGeolocateRoundTrip() {
    std::printf("geolocate: round trip over terrain\n");

    const Geodetic origin(47.3769, 8.5417, 400.0);
    const EnuFrame frame(origin);
    const CameraModel plain = CameraModel::fromSensor(18.0, 23.5, 1920, 1080);

    CameraModel distorted = plain;
    distorted.distortion() = {-0.21, 0.08, -0.01, 0.0005, -0.0007};

    Rng rng;
    for (const Terrain::Kind kind :
         {Terrain::Kind::Flat, Terrain::Kind::Hills, Terrain::Kind::Valley,
          Terrain::Kind::Mountains}) {
        const Terrain terrain(kind, 4000.0, 513, 0.0);

        const CameraModel* const cams[2] = {&plain, &distorted};
        for (const CameraModel* cam : cams) {
            double worst_px = 0.0;
            int hits = 0, misses = 0;
            for (int i = 0; i < 400; ++i) {
                UavState s;
                // Comfortably above the highest ground, so the camera is
                // never inside the terrain - that is a separate case, and
                // mixing it in here would mask a real round-trip failure.
                s.position = Geodetic(origin.latitude_deg + rng.range(-0.004, 0.004),
                                      origin.longitude_deg + rng.range(-0.004, 0.004),
                                      origin.altitude_m + terrain.maxHeight() +
                                          rng.range(80.0, 500.0));
                s.attitude = {rng.range(-180, 180), rng.range(-12, 12), rng.range(-12, 12)};
                s.gimbal = {rng.range(-60, 60), rng.range(35, 90), rng.range(-10, 10)};

                const glm::dvec2 px(rng.range(0, cam->imageWidth()),
                                    rng.range(0, cam->imageHeight()));
                const GeolocateResult g = geolocatePixel(s, *cam, terrain, frame, px);
                if (!g.valid) { ++misses; continue; }
                ++hits;

                const ProjectResult p = projectGeodetic(s, *cam, frame, g.ground_geodetic);
                check(p.in_front, "a geolocated point is in front of the camera");
                worst_px = std::max(worst_px, glm::length(p.pixel - px));
            }
            const std::string label =
                std::string(Terrain::kindName(kind)) +
                (cam == &distorted ? " (distorted)" : " (ideal)");
            check(hits > 300, label + ": most rays found ground");
            // A tenth of a pixel is far below anything a user could see, and
            // the error here is dominated by the geodetic round trip through
            // ECEF rather than by the ray maths.
            checkNear(worst_px, 0.0, 0.1, label + ": pixel round trip (worst, pixels)");
            (void)misses;
        }
    }
}

void testGeolocateFailureModes() {
    std::printf("geolocate: failure modes\n");

    const Geodetic origin(0.0, 0.0, 0.0);
    const EnuFrame frame(origin);
    const Terrain flat(Terrain::Kind::Flat, 1000.0, 65, 0.0);
    const CameraModel cam = CameraModel::fromSensor(24.0, 36.0, 640, 480);

    UavState s;
    s.position = Geodetic(0.0, 0.0, 200.0);

    // Above the horizon: no ground, and the tool must say which kind of
    // no-ground this is rather than returning a quiet zero.
    s.gimbal = {0, -30, 0};  // tilted up
    GeolocateResult r = geolocatePixel(s, cam, flat, frame, {cam.cx(), cam.cy()});
    check(!r.valid, "a ray above the horizon finds no ground");
    check(r.pointed_at_sky, "...and is reported as pointing at the sky");
    check(!r.left_coverage, "...not as leaving coverage");

    // Shallow but downward, aimed past the edge of the modelled square: that
    // is missing data, not missing ground, and conflating the two is how a
    // tool quietly lies at the edge of its DEM.
    s.gimbal = {0, 2.0, 0};
    r = geolocatePixel(s, cam, flat, frame, {cam.cx(), cam.cy()});
    check(!r.valid, "a ray leaving the tile finds no ground");
    check(r.left_coverage, "...and is reported as leaving coverage");
    check(!r.pointed_at_sky, "...not as pointing at the sky");

    // A point behind the camera must never be reported as being in the image.
    s.gimbal = {0, 90, 0};
    const ProjectResult behind = projectEnu(s, cam, frame, glm::dvec3(0, 0, 400.0));
    check(!behind.in_front, "a point behind the camera is not in front");
    check(!behind.in_image, "...and is not in the image");
}

void testFootprint() {
    std::printf("footprint\n");

    const Geodetic origin(48.0, 11.0, 0.0);
    const EnuFrame frame(origin);
    const Terrain flat(Terrain::Kind::Flat, 20000.0, 65, 0.0);
    const CameraModel cam = CameraModel::fromSensor(24.0, 36.0, 1280, 720);

    UavState s;
    s.position = Geodetic(48.0, 11.0, 400.0);
    s.gimbal = {0, 90, 0};
    s.camera_offset_body = glm::dvec3(0.0);

    const Footprint f = computeFootprint(s, cam, flat, frame, 8);
    check(f.complete, "a nadir footprint over flat ground is complete");
    // Nadir over flat ground, the footprint is a rectangle of
    // (width * H / fx) by (height * H / fy).
    const double want = (1280.0 * 400.0 / cam.fx()) * (720.0 * 400.0 / cam.fy());
    checkNear(f.area_m2, want, want * 1e-9, "nadir footprint area matches the rectangle");
    for (int i = 0; i < 4; ++i) check(f.corners[i].valid, "every footprint corner found ground");

    // Pointed at the sky, the footprint must be reported as incomplete and
    // must not invent an area.
    s.gimbal = {0, -45, 0};
    const Footprint sky = computeFootprint(s, cam, flat, frame, 4);
    check(!sky.complete, "a skyward footprint is incomplete");
    checkNear(sky.area_m2, 0.0, 0.0, "...and reports no area rather than a wrong one");
}

void testGlProjectionMatchesModel() {
    std::printf("gl projection agrees with the pinhole model\n");

    // The simulated camera view is drawn with glProjectionMatrix() while the
    // picking maths uses pixelToRay(). If those two disagree, clicking a
    // feature in the rendered view geolocates a slightly different feature,
    // and nothing on screen reveals it. So: push points through both and
    // require the same pixel.
    CameraModel cam = CameraModel::fromSensor(20.0, 36.0, 1600, 900);
    cam.setIntrinsics(cam.fx(), cam.fy() * 1.02, cam.cx() - 23.0, cam.cy() + 17.0);  // off-centre

    const glm::mat4 proj = cam.glProjectionMatrix(1.0, 10000.0);

    Rng rng;
    double worst = 0.0;
    for (int i = 0; i < 4000; ++i) {
        // A point in the optical frame, in front of the lens.
        const glm::dvec3 optical(rng.range(-400, 400), rng.range(-400, 400),
                                 rng.range(5.0, 3000.0));
        const glm::dvec2 want = cam.projectToPixel(optical);

        // The view matrix flips y and z to get GL eye space.
        const glm::vec4 eye(float(optical.x), float(-optical.y), float(-optical.z), 1.0f);
        const glm::vec4 clip = proj * eye;
        if (clip.w <= 0.0f) continue;
        const glm::vec3 ndc = glm::vec3(clip) / clip.w;

        // NDC back to pixels: x right, y up in NDC but down in image space.
        const glm::dvec2 got{(ndc.x * 0.5 + 0.5) * cam.imageWidth(),
                             (0.5 - ndc.y * 0.5) * cam.imageHeight()};
        worst = std::max(worst, glm::length(got - want));
    }
    // Loose compared to the double-precision tests above because the matrix
    // itself is float, as GL requires; a twentieth of a pixel is pure float
    // rounding, not a modelling difference.
    checkNear(worst, 0.0, 0.05, "gl matrix and pinhole model agree (worst, pixels)");
}

// --- real elevation data -------------------------------------------------

// Writes a .hgt file whose height at every post is a known function of the
// post indices, so what comes back out can be compared against arithmetic.
//
// Synthetic rather than a downloaded tile on purpose: this has to run in CI
// and on a machine with no network, and a test that needs 7 MB fetched from
// AWS is a test that gets skipped. What it verifies is the format handling -
// byte order, row order, voids, tile selection - which is exactly where the
// bugs in a headerless format live.
std::string writeTestHgt(const std::string& directory, const std::string& name, int side,
                         const std::vector<int16_t>& values) {
    const std::string path = directory + "/" + name + ".hgt";
    std::ofstream file(path, std::ios::binary);
    for (int i = 0; i < side * side; ++i) {
        const int16_t v = values[size_t(i)];
        // Big-endian, as the format requires. Written byte by byte so the
        // test does not depend on the host's byte order either.
        const unsigned char bytes[2] = {(unsigned char)((uint16_t(v) >> 8) & 0xFF),
                                        (unsigned char)(uint16_t(v) & 0xFF)};
        file.write(reinterpret_cast<const char*>(bytes), 2);
    }
    return path;
}

std::string makeTempDir() {
    // TMPDIR when the environment sets one (the sandbox here does), so the
    // test writes where it is allowed to.
    const char* base = std::getenv("TMPDIR");
    std::string directory = std::string(base && *base ? base : "/tmp") + "/georay_selftest_dem";
    const std::string command = "rm -rf '" + directory + "' && mkdir -p '" + directory + "'";
    if (std::system(command.c_str()) != 0) return {};
    return directory;
}

void testHgtTile() {
    std::printf("srtm .hgt tiles\n");

    checkNear(0.0, 0.0, 0.0, "");  // keeps the check counter honest if we bail
    const std::string directory = makeTempDir();
    check(!directory.empty(), "temporary directory for the DEM tests");
    if (directory.empty()) return;

    // Tile names parse, including the hemispheres and the floor behaviour
    // that truncation gets wrong just south of the equator or west of
    // Greenwich.
    check(HgtTile::tileNameFor(48.137, 11.575) == "N48E011", "tile name, northern/eastern");
    check(HgtTile::tileNameFor(-0.5, -0.5) == "S01W001", "tile name floors, not truncates");
    check(HgtTile::tileNameFor(-33.9, 18.4) == "S34E018", "tile name, southern/eastern");
    check(HgtTile::tileNameFor(48.0, 11.0) == "N48E011", "tile name on an exact corner");
    // Garbage in must not produce a name no tile could have, or undefined
    // behaviour from converting a NaN to an int.
    check(HgtTile::tileNameFor(std::nan(""), 11.0) == "N00E011", "a NaN latitude is handled");
    check(HgtTile::tileNameFor(1e9, 1e9) == "N89E179", "an absurd position clamps to a real tile");

    int lat = 0, lon = 0;
    check(HgtTile::parseName("/data/dem/S34E018.hgt", lat, lon) && lat == -34 && lon == 18,
          "parse a path with directories and a suffix");
    check(!HgtTile::parseName("rubbish.hgt", lat, lon), "reject a name that is not a tile");
    check(!HgtTile::parseName("N48E011/x.bin", lat, lon),
          "a directory named like a tile does not make the file one");

    // A 3x3 tile covering N48E011, with a height that encodes its own post
    // indices: h = 1000 + 10*row + col, row 0 being the NORTH edge.
    const int side = 3;
    std::vector<int16_t> values(size_t(side * side));
    for (int r = 0; r < side; ++r) {
        for (int c = 0; c < side; ++c) values[size_t(r * side + c)] = int16_t(1000 + 10 * r + c);
    }
    const std::string path = writeTestHgt(directory, "N48E011", side, values);

    HgtTile tile;
    std::string error;
    check(HgtTile::load(path, tile, error), "load a tile: " + error);
    check(tile.samplesPerSide() == 3, "posting count comes from the file length");
    check(tile.cornerLatitude() == 48 && tile.cornerLongitude() == 11, "corner from the name");

    double h = 0.0;
    // The four corners. These are the assertions that catch a flipped row
    // order, which is otherwise invisible - the terrain still looks like
    // terrain, just mirrored north-south.
    check(tile.sample(49.0, 11.0, h), "sample the north-west corner");
    checkNear(h, 1000.0, 1e-9, "north-west corner is row 0, col 0");
    check(tile.sample(49.0, 12.0, h), "sample the north-east corner");
    checkNear(h, 1002.0, 1e-9, "north-east corner is row 0, last col");
    check(tile.sample(48.0, 11.0, h), "sample the south-west corner");
    checkNear(h, 1020.0, 1e-9, "south-west corner is the LAST row, col 0");
    check(tile.sample(48.0, 12.0, h), "sample the south-east corner");
    checkNear(h, 1022.0, 1e-9, "south-east corner is the last row, last col");

    // Interpolation between them.
    check(tile.sample(48.5, 11.5, h), "sample the tile centre");
    checkNear(h, 1011.0, 1e-9, "centre interpolates to the middle post");
    check(tile.sample(48.75, 11.25, h), "sample a quarter point");
    checkNear(h, 1005.5, 1e-9, "quarter point interpolates linearly");

    check(!tile.sample(47.5, 11.5, h), "a position south of the tile is off it");
    check(!tile.sample(48.5, 13.5, h), "a position east of the tile is off it");

    // Voids: a corner marked -32768 must not drag the surface 32 km down.
    std::vector<int16_t> voided = values;
    voided[0] = HgtTile::kVoid;  // north-west post
    writeTestHgt(directory, "N49E011", side, voided);
    HgtTile void_tile;
    check(HgtTile::load(directory + "/N49E011.hgt", void_tile, error), "load the voided tile");
    check(void_tile.voidCount() == 1, "voids are counted at load");
    check(void_tile.sample(49.5, 11.5, h), "a cell with one void still samples");
    check(h > 900.0 && h < 1100.0,
          "a void is excluded from the interpolation, not read as -32768 metres");

    // An entirely void cell is genuinely no-data and must say so.
    std::vector<int16_t> all_void(size_t(side * side), HgtTile::kVoid);
    writeTestHgt(directory, "N50E011", side, all_void);
    HgtTile empty_tile;
    check(HgtTile::load(directory + "/N50E011.hgt", empty_tile, error), "load an all-void tile");
    check(!empty_tile.sample(50.5, 11.5, h), "an all-void cell reports no data");

    // A file whose length is not a square number of samples is not a tile.
    {
        std::ofstream bad(directory + "/N51E011.hgt", std::ios::binary);
        const char junk[10] = {};
        bad.write(junk, sizeof(junk));
    }
    HgtTile bad_tile;
    check(!HgtTile::load(directory + "/N51E011.hgt", bad_tile, error),
          "reject a file that is not a square array");

    // The source picks the right tile per position, and reports what it could
    // not find rather than treating it as sea level.
    DemSource source(directory);
    check(source.sampleOrthometric(48.5, 11.5, h), "source finds N48E011");
    checkNear(h, 1011.0, 1e-9, "...and samples it correctly");
    check(source.sampleOrthometric(49.5, 11.5, h), "source finds N49E011");
    check(!source.sampleOrthometric(48.5, 20.5, h), "source reports no data where no tile is");
    check(source.loadedTiles().size() == 2, "only the tiles actually needed are loaded");
    check(source.missingTiles().size() == 1, "the absent tile is reported as missing");

    // Neighbouring tiles share their edge posts, so a position on the seam
    // must give the same height whichever side it is sampled from. Getting
    // this wrong puts a one-post cliff along every degree line.
    std::vector<int16_t> south(size_t(side * side));
    for (int r = 0; r < side; ++r) {
        for (int c = 0; c < side; ++c) south[size_t(r * side + c)] = int16_t(2000 + 10 * r + c);
    }
    // Make N47E011's north edge equal N48E011's south edge, as real tiles do.
    for (int c = 0; c < side; ++c) south[size_t(c)] = int16_t(1020 + c);
    writeTestHgt(directory, "N47E011", side, south);

    HgtTile north_tile, south_tile;
    check(HgtTile::load(directory + "/N48E011.hgt", north_tile, error), "load the north tile");
    check(HgtTile::load(directory + "/N47E011.hgt", south_tile, error), "load the south tile");
    double from_north = 0.0, from_south = 0.0;
    check(north_tile.sample(48.0, 11.5, from_north), "seam from the north tile");
    check(south_tile.sample(48.0, 11.5, from_south), "seam from the south tile");
    checkNear(from_north, from_south, 1e-9, "shared edge posts agree across the seam");
}

void testGeoid() {
    std::printf("geoid separation\n");

    const GeoidModel none;
    check(none.isIdentity(), "the default model applies no correction");
    checkNear(none.separationAt(48.0, 11.0), 0.0, 0.0, "...and reports zero separation");

    const GeoidModel flat = GeoidModel::constant(47.3);
    check(!flat.isIdentity(), "a constant model is not the identity");
    checkNear(flat.separationAt(48.0, 11.0), 47.3, 1e-12, "constant separation anywhere");
    checkNear(flat.separationAt(-33.9, 18.4), 47.3, 1e-12, "...really anywhere");
    // The conversion the whole model exists for: orthometric to ellipsoidal.
    checkNear(flat.toEllipsoidal(48.0, 11.0, 515.0), 562.3, 1e-12,
              "orthometric height plus separation is ellipsoidal height");
}

// Loading real elevation data end to end: DEM posts in, ENU heights out.
//
// The hard part to verify is that this cannot be checked by re-running the
// code under test, so every assertion below is against something known
// independently - a closed-form curvature drop, a gradient computed from
// metres-per-degree, an exact identity at the frame origin.
void testDemTerrain() {
    std::printf("dem -> enu terrain\n");

    const std::string directory = makeTempDir();
    check(!directory.empty(), "temporary directory for the terrain tests");
    if (directory.empty()) return;

    const int side = 61;  // 1/60 degree posting, so the values below are integers

    // --- a DEM of constant orthometric height -----------------------------
    //
    // Flat ground on the ellipsoid is NOT flat in a local tangent plane: it
    // curves away below it. That drop is d^2 / 2R, which is independently
    // computable, and it is the one effect that proves the conversion really
    // goes through the ellipsoid rather than just subtracting a number.
    {
        const std::vector<int16_t> flat(size_t(side * side), 500);
        writeTestHgt(directory, "N48E011", side, flat);

        const double separation = 47.0;
        const GeoidModel geoid = GeoidModel::constant(separation);
        // The origin's ellipsoidal height set to match the ground exactly, so
        // the terrain's ENU height at the origin must come out at zero.
        const Geodetic origin(48.5, 11.5, 500.0 + separation);
        const EnuFrame frame(origin);

        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 3000.0, 65, 0.0);
        const Terrain::DemLoadReport report = terrain.loadFromDem(dem, geoid, frame);

        check(report.ok, "a covered area loads: " + report.error);
        check(report.posts_from_data == report.posts_total, "every post came from the data");
        check(report.posts_filled == 0, "nothing had to be invented");
        check(report.tiles_used.size() == 1 && report.tiles_used.front() == "N48E011",
              "exactly the one covering tile was loaded");
        checkNear(report.origin_orthometric_m, 500.0, 1e-9, "origin ground height is reported");
        checkNear(report.origin_separation_m, separation, 1e-12, "separation is reported");

        checkNear(terrain.sampleHeight(0.0, 0.0), 0.0, 1e-6,
                  "ground at the frame origin sits at ENU zero");

        // d^2 / 2R, with R the local radius of curvature (~6.37e6 m at this
        // latitude): 3 km out, flat ground falls about 0.71 m below the
        // tangent plane. Zero here would mean the curvature term was dropped.
        const double expected_drop = -3000.0 * 3000.0 / (2.0 * 6.374e6);
        checkNear(terrain.sampleHeight(3000.0, 0.0), expected_drop, 0.02,
                  "flat ground curves away from the tangent plane to the east");
        checkNear(terrain.sampleHeight(0.0, 3000.0), expected_drop, 0.02,
                  "...and to the north by the same amount");
        check(expected_drop < -0.6, "the curvature drop this test checks for is non-trivial");

        // Changing the geoid separation must move the whole surface rigidly.
        Terrain shifted(Terrain::Kind::Flat, 3000.0, 65, 0.0);
        DemSource dem2(directory);
        shifted.loadFromDem(dem2, GeoidModel::constant(separation + 10.0), frame);
        checkNear(shifted.sampleHeight(0.0, 0.0) - terrain.sampleHeight(0.0, 0.0), 10.0, 1e-6,
                  "a 10 m larger separation raises the ground by exactly 10 m");
    }

    // --- a DEM with a known slope ----------------------------------------
    //
    // h = 500 + 1200*(lat - 48) + 600*(lon - 11), chosen so every post is a
    // whole number of metres (the format stores integers) and so the
    // north and east gradients differ enough that swapping them is obvious.
    {
        std::vector<int16_t> sloped(size_t(side * side));
        for (int r = 0; r < side; ++r) {
            for (int c = 0; c < side; ++c) {
                // Row 0 is the north edge, i.e. lat = 49.
                sloped[size_t(r * side + c)] = int16_t(1700 - 20 * r + 10 * c);
            }
        }
        writeTestHgt(directory, "N48E011", side, sloped);

        const Geodetic origin(48.5, 11.5, 1000.0);
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 2000.0, 65, 0.0);
        check(terrain.loadFromDem(dem, GeoidModel::constant(47.0), frame).ok,
              "the sloped DEM loads");

        // Central differences over 2 km, against metres-per-degree at 48.5N.
        // These two numbers differ by a third, so an east/north swap cannot
        // slip through, and a sign error flips them negative.
        const double dz_dn =
            (terrain.sampleHeight(0.0, 1000.0) - terrain.sampleHeight(0.0, -1000.0)) / 2000.0;
        const double dz_de =
            (terrain.sampleHeight(1000.0, 0.0) - terrain.sampleHeight(-1000.0, 0.0)) / 2000.0;

        const double metres_per_degree_lat = 111200.0;  // at 48.5 degrees
        const double metres_per_degree_lon = 73900.0;
        checkNear(dz_dn, 1200.0 / metres_per_degree_lat, 2e-4,
                  "northward slope matches 1200 m per degree of latitude");
        checkNear(dz_de, 600.0 / metres_per_degree_lon, 2e-4,
                  "eastward slope matches 600 m per degree of longitude");
        check(dz_dn > dz_de * 1.2, "north and east gradients are not interchangeable");
    }

    // --- across a tile seam ----------------------------------------------
    //
    // An origin on a degree corner needs four tiles. If tile selection or the
    // shared edge posts were wrong, a one-post cliff would run through the
    // middle of the scene.
    {
        const std::vector<int16_t> flat(size_t(side * side), 800);
        for (const char* name : {"N47E010", "N47E011", "N48E010", "N48E011"}) {
            writeTestHgt(directory, name, side, flat);
        }

        const Geodetic origin(48.0, 11.0, 800.0 + 47.0);
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 2000.0, 65, 0.0);
        const Terrain::DemLoadReport report =
            terrain.loadFromDem(dem, GeoidModel::constant(47.0), frame);

        check(report.ok, "a scene straddling four tiles loads");
        check(report.tiles_used.size() == 4, "all four tiles were used");
        check(report.posts_filled == 0, "no gaps across the seams");

        // Nothing but the smooth curvature drop may appear: step across the
        // seam in 50 m increments and require the surface to stay continuous.
        double worst_step = 0.0;
        double previous = terrain.sampleHeight(-500.0, 0.0);
        for (double e = -450.0; e <= 500.0; e += 50.0) {
            const double here = terrain.sampleHeight(e, 0.0);
            worst_step = std::max(worst_step, std::fabs(here - previous));
            previous = here;
        }
        checkNear(worst_step, 0.0, 0.01, "no cliff where tiles meet (worst step, metres)");
    }

    // --- missing coverage -------------------------------------------------
    {
        const Geodetic origin(0.0, 0.0, 0.0);  // nothing on disk covers this
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Hills, 2000.0, 33, 0.0);
        const double before = terrain.sampleHeight(0.0, 0.0);

        const Terrain::DemLoadReport report = terrain.loadFromDem(dem, GeoidModel(), frame);
        check(!report.ok, "a load with no coverage at all fails");
        check(!report.error.empty(), "...and says why");
        checkNear(terrain.sampleHeight(0.0, 0.0), before, 1e-12,
                  "...and leaves the previous terrain alone rather than flattening it");
        check(terrain.sourceName() == "HILLS", "...including what the terrain says it is");
    }

    // --- partial coverage -------------------------------------------------
    //
    // Half a scene of real data is still useful, but the invented half has to
    // be declared. A gap filled silently is a fabricated elevation.
    {
        const std::vector<int16_t> flat(size_t(side * side), 300);
        writeTestHgt(directory, "N20E030", side, flat);
        // The origin sits near the tile's western edge, so the scene runs off
        // it into a neighbour that does not exist.
        const Geodetic origin(20.5, 30.002, 347.0);
        const EnuFrame frame(origin);
        DemSource dem(directory);
        Terrain terrain(Terrain::Kind::Flat, 2000.0, 65, 0.0);
        const Terrain::DemLoadReport report =
            terrain.loadFromDem(dem, GeoidModel::constant(47.0), frame);

        check(report.ok, "a partly covered scene still loads");
        check(report.posts_filled > 0, "the uncovered posts are counted as filled");
        check(report.gapFraction() > 0.0 && report.gapFraction() < 1.0,
              "the gap fraction is reported for the user to judge");
        check(!report.tiles_missing.empty(), "the absent tile is named");
        // Filled posts must be plausible, not zero or a void marker.
        check(terrain.minHeight() > -100.0 && terrain.maxHeight() < 100.0,
              "filled posts take sane values rather than zero or -32768");
    }
}

// The rendered simulated-camera view and the geolocation maths must be the
// same camera. If they are not, the user clicks a feature in one image and
// gets the coordinates of a slightly different feature - a failure with no
// visible symptom at all, since both the picture and the number look fine.
void testRenderMatchesGeolocation() {
    std::printf("rendered view agrees with the geolocation maths\n");

    const Geodetic origin(48.137554, 11.575497, 515.0);
    const EnuFrame frame(origin);
    // Distortion deliberately off: a projection matrix is linear and cannot
    // express it, so this is a test of the frames and the matrices, which is
    // what it can actually prove. The disagreement a distortion model causes
    // is a documented property of the tool, not a bug for this test to hide.
    CameraModel cam = CameraModel::fromSensor(12.0, 13.2, 1920, 1080);
    // An off-centre principal point and non-square pixels, because a centred
    // square-pixel camera would pass even with the terms that handle those
    // dropped entirely.
    cam.setIntrinsics(cam.fx(), cam.fy() * 1.03, cam.cx() - 31.0, cam.cy() + 19.0);

    const glm::mat4 projection = cam.glProjectionMatrix(1.0, 60000.0);

    Rng rng;
    double worst = 0.0;
    int cases = 0;
    for (int i = 0; i < 2000; ++i) {
        UavState s;
        s.position = Geodetic(origin.latitude_deg + rng.range(-0.003, 0.003),
                              origin.longitude_deg + rng.range(-0.003, 0.003),
                              origin.altitude_m + rng.range(80.0, 600.0));
        s.attitude = {rng.range(-180, 180), rng.range(-15, 15), rng.range(-30, 30)};
        s.gimbal = {rng.range(-120, 120), rng.range(10, 95), rng.range(-20, 20)};
        s.camera_offset_body = glm::dvec3(0.25, 0.0, 0.35);

        // Build the test point from a pixel and a range rather than
        // scattering points in space. Two reasons: it guarantees the point
        // is in the image, which is the only place the comparison means
        // anything, and the expected pixel is then an input rather than
        // something computed by one of the two things being compared.
        //
        // Scattering points instead puts most of them far outside the frame,
        // including near-90-degree grazing cases that project to pixel
        // coordinates in the millions - where float and double disagree
        // wildly and no user can click anyway.
        const glm::dvec2 want_pixel(rng.range(0, cam.imageWidth()),
                                    rng.range(0, cam.imageHeight()));
        const double range = rng.range(50.0, 3000.0);
        const glm::dvec3 point = cameraPositionEnu(s, frame) +
                                 rotationEnuFromOptical(s) * cam.pixelToRay(want_pixel) * range;

        // The maths half, for good measure: it must agree with its own input.
        const ProjectResult round_trip = projectEnu(s, cam, frame, point);
        check(round_trip.in_front, "a point built along a ray is in front of the camera");
        checkNear(glm::length(round_trip.pixel - want_pixel), 0.0, 1e-6,
                  "projectEnu inverts the ray construction");

        const glm::vec4 clip = projection * glViewMatrix(s, frame) *
                               glm::vec4(float(point.x), float(point.y), float(point.z), 1.0f);
        if (clip.w <= 1e-4f) continue;
        const glm::vec3 ndc = glm::vec3(clip) / clip.w;

        // NDC (+y up) back to image pixels (+y down).
        const glm::dvec2 got{(ndc.x * 0.5 + 0.5) * cam.imageWidth(),
                             (0.5 - ndc.y * 0.5) * cam.imageHeight()};
        ++cases;
        worst = std::max(worst, glm::length(got - want_pixel));
    }
    check(cases > 1800, "the render/maths comparison actually ran");
    // A twentieth of a pixel on a 1920-wide image. The matrices are float and
    // the points are kilometres out, so this is float rounding on a large
    // magnitude - not a modelling difference.
    checkNear(worst, 0.0, 0.05, "render and maths put a point on the same pixel");
}

}  // namespace

int runSelfTest() {
    g_failures = 0;
    g_checks = 0;
    std::printf("georay selftest\n\n");

    testGeodetic();
    testEnuFrame();
    testCameraModel();
    testDistortion();
    testPose();
    testTerrain();
    testGeolocateFlatClosedForm();
    testGeolocateRoundTrip();
    testGeolocateFailureModes();
    testFootprint();
    testGlProjectionMatchesModel();
    testRenderMatchesGeolocation();
    testHgtTile();
    testGeoid();
    testDemTerrain();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

}  // namespace georay
