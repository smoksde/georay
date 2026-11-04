#pragma once

// The camera's extrinsics: where the lens is and which way it points.
//
// Four frames, mixed up easily and silently if not spelled out:
//
//   ENU     world frame: +x east, +y north, +z up. Everything renders here.
//   NED     navigation frame: +x north, +y east, +z down. Yaw/pitch/roll are
//           defined against this, because that is what an autopilot reports.
//   body    aircraft frame: +x forward (nose), +y right (starboard wing),
//           +z down. Equals NED when yaw=pitch=roll=0.
//   gimbal  payload frame: same axis convention as body, rotated by the
//           gimbal's own pan/tilt/roll.
//   optical camera frame: +x right in image, +y down in image, +z view axis.
//           This is the frame CameraModel works in.
//
// The full chain a pixel's ray travels is
//   optical -> gimbal -> body -> NED -> ENU
// and this file builds it as one rotation matrix.

#include <glm/glm.hpp>

#include "geo/geodetic.hpp"

namespace georay {

// Aircraft attitude as an autopilot reports it, in degrees.
struct Attitude {
    // Heading: 0 = north, 90 = east. Clockwise seen from above, i.e. a
    // right-handed rotation about NED's +z (down).
    double yaw_deg = 0.0;
    // Nose up is positive.
    double pitch_deg = 0.0;
    // Right wing down is positive.
    double roll_deg = 0.0;
};

// Gimbal angles relative to the airframe, in degrees. A three-axis gimbal
// usually stabilises roll to zero, but it is kept here so a fixed, tilted
// camera mount can be modelled by the same struct.
struct GimbalAngles {
    // Pan relative to the nose: 0 = straight ahead, positive to the right.
    double pan_deg = 0.0;
    // Tilt below the airframe's horizontal: 0 = forward, 90 = straight down
    // (nadir), positive down.
    double tilt_deg = 90.0;
    double roll_deg = 0.0;
};

// Everything needed to turn a pixel into a world ray: where the aircraft is,
// how it is oriented, and how the camera is mounted on it.
struct UavState {
    Geodetic position;            // GPS fix of the aircraft, altitude above ellipsoid
    Attitude attitude;
    GimbalAngles gimbal;

    // Lens offset from the GPS antenna, in the body frame (metres, +x nose,
    // +y right, +z down).
    glm::dvec3 camera_offset_body{0.0, 0.0, 0.2};
};

// Rotation taking body-frame vectors to NED, from yaw-pitch-roll applied in
// the standard aerospace order (yaw about down, then pitch, then roll).
glm::dmat3 rotationNedFromBody(const Attitude& a);

// Rotation taking gimbal-frame vectors to body frame.
glm::dmat3 rotationBodyFromGimbal(const GimbalAngles& g);

// Constant change of basis from the optical frame to the gimbal frame.
glm::dmat3 rotationGimbalFromOptical();

// Constant change of basis from NED to ENU. Its own inverse.
glm::dmat3 rotationEnuFromNed();

// The whole chain: optical-frame direction -> ENU direction.
glm::dmat3 rotationEnuFromOptical(const UavState& state);

// Where the lens sits in the given ENU frame, GPS fix and lever arm included.
glm::dvec3 cameraPositionEnu(const UavState& state, const EnuFrame& frame);

// Boresight (optical +z) as an ENU unit vector - the direction the centre of
// the image looks. Convenience for HUD readouts and for aiming the gimbal.
glm::dvec3 boresightEnu(const UavState& state);

// Gimbal pan/tilt that points the boresight along a given ENU direction, at
// the aircraft's current attitude. Inverse of the chain above.
GimbalAngles aimGimbalAlong(const Attitude& attitude, const glm::dvec3& direction_enu,
                            double keep_roll_deg = 0.0);

}  // namespace georay
