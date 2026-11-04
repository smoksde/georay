// Geodesy fundamentals: the ellipsoid, the ENU tangent plane, the camera's
// own pixel/ray model, lens distortion, and the full attitude/gimbal
// rotation chain. These are the conventions pose.hpp's header documents -
// every one pinned down here independently of the code under test.

#include "selftest_support.hpp"

#include <algorithm>
#include <cmath>

#include "geo/geolocate.hpp"

namespace georay {

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

}  // namespace georay
