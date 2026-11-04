#pragma once

// The point of the whole project: given a pixel in a UAV camera image, the
// camera's intrinsics and extrinsics, the aircraft's GPS fix and attitude,
// and a ground model - where on Earth is that pixel looking?
//
// The pipeline, in order:
//   1. pixel -> undistorted ray in the optical frame      (CameraModel)
//   2. optical -> ENU rotation from attitude and gimbal    (pose.hpp)
//   3. ray origin from GPS fix plus camera lever arm       (pose.hpp)
//   4. intersect the ENU ray with the terrain              (Terrain)
//   5. hit point ENU -> ECEF -> geodetic latitude/longitude/altitude
//
// Steps 1-3 are exact given the inputs. Step 4 is where the accuracy
// actually lives, which is why the result carries enough diagnostics to tell
// a confident answer from a grazing one.

#include <array>
#include <vector>

#include <glm/glm.hpp>

#include "geo/camera_model.hpp"
#include "geo/geodetic.hpp"
#include "geo/pose.hpp"
#include "geo/terrain.hpp"

namespace georay {

struct GeolocateResult {
    bool valid = false;  // a ground point was found

    // The ray that was traced, in the ENU frame used for the query.
    glm::dvec3 ray_origin_enu{0.0};
    glm::dvec3 ray_direction_enu{0.0};

    // Where it landed.
    glm::dvec3 ground_enu{0.0};
    Geodetic ground_geodetic;

    double slant_range_m = 0.0;    // camera to ground point, along the ray
    double ground_range_m = 0.0;   // horizontal camera-to-target distance
    double ground_elevation_m = 0.0;
    double depression_angle_deg = 0.0;  // below the local horizontal; 90 = nadir

    // Ground sample distance at the hit: how wide a single pixel is on the
    // ground there. The honest measure of how precise an answer can even be,
    // and it varies hugely across one oblique frame.
    double gsd_m = 0.0;

    // Why there is no answer, when valid is false.
    bool pointed_at_sky = false;   // the ray rises and never comes back down
    bool left_coverage = false;    // ran out of terrain data before hitting

    int march_steps = 0;
};

// Geolocate one pixel. `frame` fixes which ENU origin the returned ENU
// coordinates are expressed in; the geodetic result is independent of it.
GeolocateResult geolocatePixel(const UavState& state, const CameraModel& camera,
                               const Terrain& terrain, const EnuFrame& frame,
                               const glm::dvec2& pixel);

// Inverse direction: where does a known world point appear in the image?
// Used to verify the forward pipeline by round-tripping, and to overlay
// ground markers onto the simulated camera view.
struct ProjectResult {
    bool in_front = false;   // in front of the lens at all
    bool in_image = false;   // and inside the image bounds
    glm::dvec2 pixel{0.0};
    double range_m = 0.0;
};

ProjectResult projectGeodetic(const UavState& state, const CameraModel& camera,
                              const EnuFrame& frame, const Geodetic& target);
ProjectResult projectEnu(const UavState& state, const CameraModel& camera,
                         const EnuFrame& frame, const glm::dvec3& point_enu);

// View matrix taking ENU world coordinates into the OpenGL eye space of the
// simulated camera. Lives here rather than in the renderer because, combined
// with CameraModel::glProjectionMatrix(), it must put a world point on
// exactly the pixel projectEnu() says it lands on - a numeric claim, tested
// here without a GL context.
//
// Cannot reproduce lens distortion: a projection matrix is linear and
// distortion is not, so with a distortion model active the render and the
// maths disagree towards the frame edges by precisely the distortion's size.
glm::mat4 glViewMatrix(const UavState& state, const EnuFrame& frame);

// Ground footprint of the whole image: the terrain hits of the four image
// corners, plus a densified outline. `samples_per_edge` controls how finely
// each edge is traced - an oblique view's footprint is curved by both
// perspective and relief, so a straight line between corners would lie.
// One traced point along the border. Misses stay in the list rather than
// being dropped, so a consumer can tell adjacent samples from ones with a
// gap between them instead of drawing a straight line across it.
struct FootprintSample {
    bool valid = false;
    glm::dvec3 enu{0.0};
};

struct Footprint {
    bool complete = false;  // every sampled point found ground
    std::array<GeolocateResult, 4> corners;  // TL, TR, BR, BL
    std::vector<FootprintSample> outline;    // in border order, misses included
    double area_m2 = 0.0;                    // 0 when incomplete
};

Footprint computeFootprint(const UavState& state, const CameraModel& camera,
                           const Terrain& terrain, const EnuFrame& frame,
                           int samples_per_edge = 12);

}  // namespace georay
