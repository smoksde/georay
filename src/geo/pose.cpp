#include "geo/pose.hpp"

#include <cmath>

namespace georay {
namespace {

// Right-handed rotations about the three axes of whichever frame they are
// applied in. Written out rather than taken from glm::rotate so the column
// order is visible at the call sites that depend on it.
glm::dmat3 rotX(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return glm::dmat3(glm::dvec3(1, 0, 0), glm::dvec3(0, c, s), glm::dvec3(0, -s, c));
}
glm::dmat3 rotY(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return glm::dmat3(glm::dvec3(c, 0, -s), glm::dvec3(0, 1, 0), glm::dvec3(s, 0, c));
}
glm::dmat3 rotZ(double a) {
    const double c = std::cos(a), s = std::sin(a);
    return glm::dmat3(glm::dvec3(c, s, 0), glm::dvec3(-s, c, 0), glm::dvec3(0, 0, 1));
}

}  // namespace

glm::dmat3 rotationNedFromBody(const Attitude& a) {
    // Aerospace order: yaw about down, then pitch about the new right axis,
    // then roll about the new forward axis. As matrices acting on a body
    // vector that composes right-to-left as Rz * Ry * Rx.
    return rotZ(deg2rad(a.yaw_deg)) * rotY(deg2rad(a.pitch_deg)) * rotX(deg2rad(a.roll_deg));
}

glm::dmat3 rotationBodyFromGimbal(const GimbalAngles& g) {
    // Tilt enters negated: positive tilt means pointing down, while a
    // positive rotation about the body's +y (right) axis pitches the nose up.
    return rotZ(deg2rad(g.pan_deg)) * rotY(-deg2rad(g.tilt_deg)) * rotX(deg2rad(g.roll_deg));
}

glm::dmat3 rotationGimbalFromOptical() {
    // Columns are the optical basis vectors written in gimbal coordinates:
    // image-right is gimbal-right, image-down is gimbal-down, and the view
    // axis is gimbal-forward.
    return glm::dmat3(glm::dvec3(0, 1, 0),   // optical +x -> body/gimbal +y
                      glm::dvec3(0, 0, 1),   // optical +y -> body/gimbal +z
                      glm::dvec3(1, 0, 0));  // optical +z -> body/gimbal +x
}

glm::dmat3 rotationEnuFromNed() {
    return glm::dmat3(glm::dvec3(0, 1, 0),    // north -> +y
                      glm::dvec3(1, 0, 0),    // east  -> +x
                      glm::dvec3(0, 0, -1));  // down  -> -z
}

glm::dmat3 rotationEnuFromOptical(const UavState& s) {
    return rotationEnuFromNed() * rotationNedFromBody(s.attitude) *
           rotationBodyFromGimbal(s.gimbal) * rotationGimbalFromOptical();
}

glm::dvec3 cameraPositionEnu(const UavState& s, const EnuFrame& frame) {
    const glm::dvec3 gps_enu = frame.fromGeodetic(s.position);
    const glm::dvec3 lever_enu =
        rotationEnuFromNed() * rotationNedFromBody(s.attitude) * s.camera_offset_body;
    return gps_enu + lever_enu;
}

glm::dvec3 boresightEnu(const UavState& s) {
    return rotationEnuFromOptical(s) * glm::dvec3(0.0, 0.0, 1.0);
}

GimbalAngles aimGimbalAlong(const Attitude& attitude, const glm::dvec3& direction_enu,
                            double keep_roll_deg) {
    GimbalAngles out;
    out.roll_deg = keep_roll_deg;

    const double len = glm::length(direction_enu);
    if (len < 1e-12) return out;  // no direction to aim along; leave the gimbal be

    // Bring the target direction into the body frame, where pan and tilt are
    // just its azimuth and elevation. Rotations are orthonormal, so the
    // transposes below are the inverses.
    const glm::dvec3 dir_body = glm::transpose(rotationNedFromBody(attitude)) *
                                glm::transpose(rotationEnuFromNed()) * (direction_enu / len);

    // Forward chain gives boresight_body = (cosPan cosTilt, sinPan cosTilt, sinTilt).
    out.tilt_deg = rad2deg(std::asin(glm::clamp(dir_body.z, -1.0, 1.0)));
    out.pan_deg = rad2deg(std::atan2(dir_body.y, dir_body.x));
    return out;
}

}  // namespace georay
