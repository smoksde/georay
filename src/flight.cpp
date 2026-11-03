#include "flight.hpp"

#include <algorithm>
#include <cmath>

namespace georay {
namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

const char* FlightSimulator::patternName(Pattern p) {
    switch (p) {
        case Pattern::Hold: return "HOLD";
        case Pattern::Orbit: return "ORBIT";
        case Pattern::Lawnmower: return "LAWNMOWER";
        case Pattern::Transit: return "TRANSIT";
    }
    return "?";
}

const char* FlightSimulator::gimbalModeName(GimbalMode m) {
    switch (m) {
        case GimbalMode::Fixed: return "MANUAL";
        case GimbalMode::Nadir: return "NADIR LOCK";
        case GimbalMode::TrackPoint: return "TRACK POINT";
    }
    return "?";
}

void FlightSimulator::configure(Pattern pattern, const glm::dvec2& centre_enu) {
    pattern_ = pattern;
    centre_enu_ = centre_enu;
    track_target_enu_ = centre_enu;
    elapsed_s_ = 0.0;
}

void FlightSimulator::setSpeed(double v) { speed_mps_ = std::clamp(v, 0.0, 80.0); }
void FlightSimulator::setRadius(double r) { radius_m_ = std::clamp(r, 20.0, 2500.0); }
void FlightSimulator::setHeightAboveGround(double h) {
    height_agl_m_ = std::clamp(h, 10.0, 4000.0);
}

Attitude FlightSimulator::attitudeFor(const glm::dvec2& velocity_enu,
                                      double turn_rate_rad_s) const {
    Attitude a;
    if (glm::length(velocity_enu) > 1e-6) {
        // Heading is measured from north, clockwise: atan2(east, north).
        a.yaw_deg = rad2deg(std::atan2(velocity_enu.x, velocity_enu.y));
    }

    // Coordinated-turn bank angle: tan(roll) = v * omega / g. The same
    // relation an autopilot uses, so a turning aircraft rolls by a realistic
    // amount rather than a made-up one - and the gimbal has to work against
    // it, which is the point.
    const double speed = glm::length(velocity_enu);
    if (speed > 0.5 && std::fabs(turn_rate_rad_s) > 1e-9) {
        a.roll_deg = rad2deg(std::atan(speed * turn_rate_rad_s / 9.80665));
        a.roll_deg = std::clamp(a.roll_deg, -45.0, 45.0);
    }

    // A slight nose-up trim, as a fixed-wing aircraft flies. Small, but it
    // means the pitch term in the chain is never exactly zero - which is how
    // a pitch handled incorrectly stays hidden.
    a.pitch_deg = 2.0;
    return a;
}

void FlightSimulator::update(double dt, const Terrain& terrain, const EnuFrame& frame,
                             UavState& state) {
    elapsed_s_ += dt;

    glm::dvec2 position_enu = centre_enu_;
    glm::dvec2 velocity_enu(0.0);
    double turn_rate = 0.0;

    switch (pattern_) {
        case Pattern::Hold:
            position_enu = centre_enu_ + glm::dvec2(radius_m_, 0.0);
            break;

        case Pattern::Orbit: {
            // Angular rate from the commanded ground speed, so changing speed
            // or radius keeps the two consistent instead of letting the
            // aircraft fly an impossible corner.
            turn_rate = speed_mps_ / std::max(radius_m_, 1.0);
            const double angle = elapsed_s_ * turn_rate;
            position_enu = centre_enu_ + radius_m_ * glm::dvec2(std::cos(angle), std::sin(angle));
            velocity_enu = speed_mps_ * glm::dvec2(-std::sin(angle), std::cos(angle));
            break;
        }

        case Pattern::Lawnmower: {
            // Legs run east-west, stepping north between them. The turns at
            // the leg ends are instantaneous in heading, which is unrealistic
            // but keeps the pattern readable; the bank angle is left at zero
            // there rather than spiking.
            const double leg_time = leg_length_m_ / std::max(speed_mps_, 0.1);
            const int leg = int(elapsed_s_ / std::max(leg_time, 1e-6));
            const double t_in_leg = elapsed_s_ - leg * leg_time;
            const double along = (t_in_leg / std::max(leg_time, 1e-6)) * leg_length_m_;
            const double direction = (leg % 2 == 0) ? 1.0 : -1.0;

            position_enu.x = centre_enu_.x - leg_length_m_ * 0.5 +
                             (direction > 0 ? along : leg_length_m_ - along);
            position_enu.y = centre_enu_.y + (leg - 2) * leg_spacing_m_;
            velocity_enu = glm::dvec2(direction * speed_mps_, 0.0);
            break;
        }

        case Pattern::Transit: {
            const double heading = deg2rad(transit_heading_deg_);
            const glm::dvec2 forward(std::sin(heading), std::cos(heading));
            // Reverse every few minutes so the aircraft stays on the modelled
            // terrain instead of flying off the edge and leaving the view
            // showing nothing.
            const double leg_time = 120.0;
            const double phase = std::fmod(elapsed_s_, 2.0 * leg_time);
            const double along = phase < leg_time ? phase : 2.0 * leg_time - phase;
            const double direction = phase < leg_time ? 1.0 : -1.0;
            position_enu = centre_enu_ + forward * (along * speed_mps_ - 1200.0);
            velocity_enu = forward * (speed_mps_ * direction);
            break;
        }
    }

    // Terrain-following altitude: the aircraft holds a height above the
    // ground below it, not a constant altitude. That keeps the simulated
    // imagery at a usable scale over relief, and it is what an AGL-holding
    // autopilot does.
    const double ground = terrain.sampleHeight(position_enu.x, position_enu.y);
    const glm::dvec3 enu(position_enu.x, position_enu.y, ground + height_agl_m_);

    state.position = frame.toGeodetic(enu);
    state.attitude = attitudeFor(velocity_enu, turn_rate);

    switch (gimbal_mode_) {
        case GimbalMode::Fixed:
            // Left exactly as the user set it, which is what makes the
            // manual-control keys meaningful.
            break;

        case GimbalMode::Nadir:
            // Straight down in the *world*, which means cancelling the
            // aircraft's own attitude - exactly what a stabilised gimbal
            // does, and a good check that the inverse chain is right.
            state.gimbal = aimGimbalAlong(state.attitude, glm::dvec3(0.0, 0.0, -1.0));
            break;

        case GimbalMode::TrackPoint: {
            const double target_z =
                terrain.sampleHeight(track_target_enu_.x, track_target_enu_.y);
            const glm::dvec3 target(track_target_enu_.x, track_target_enu_.y, target_z);
            const glm::dvec3 to_target = target - cameraPositionEnu(state, frame);
            state.gimbal = aimGimbalAlong(state.attitude, to_target);
            break;
        }
    }
}

}  // namespace georay
