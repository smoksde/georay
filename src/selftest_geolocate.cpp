// Terrain intersection and the geolocation pipeline built on it: the flat-
// ground closed form, the round trip over real relief, the failure modes a
// ray can have, the footprint, and the two places a render-time matrix must
// agree with the pixel/ray maths it is supposed to match.

#include "selftest_support.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "geo/geolocate.hpp"

namespace georay {

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

}  // namespace georay
