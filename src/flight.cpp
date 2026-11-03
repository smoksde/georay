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
        case Pattern::Mission: return "MISSION";
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

void FlightSimulator::setMission(const georay::Mission& mission, const Terrain& terrain,
                                 const EnuFrame& frame, double geoid_separation_m) {
    mission_ = mission;
    route_.clear();
    leg_start_m_.clear();
    route_length_m_ = 0.0;
    along_route_m_ = 0.0;
    current_leg_ = 0;

    for (const Waypoint& item : mission.navigationItems()) {
        const Geodetic resolved =
            mission.resolveAltitude(item, terrain, frame, geoid_separation_m);
        route_.push_back(frame.fromGeodetic(resolved));
    }

    // Cumulative distance along the route, so a position can be found by
    // walking a single scalar rather than by integrating per frame - which
    // would drift, and would make the flight depend on the frame rate.
    leg_start_m_.push_back(0.0);
    for (size_t i = 1; i < route_.size(); ++i) {
        route_length_m_ += glm::length(route_[i] - route_[i - 1]);
        leg_start_m_.push_back(route_length_m_);
    }

    if (mission.cruise_speed_mps > 0.5) speed_mps_ = mission.cruise_speed_mps;
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

        case Pattern::Mission: {
            if (route_.size() < 2) {
                // A plan with one usable waypoint is a position, not a
                // route. Hold over it rather than dividing by a zero length.
                if (!route_.empty()) position_enu = glm::dvec2(route_.front());
                break;
            }
            along_route_m_ += speed_mps_ * dt;
            // Loop rather than stop at the end: the point is to watch the
            // geolocation work along the whole plan, repeatedly.
            if (route_length_m_ > 1e-6) {
                along_route_m_ = std::fmod(along_route_m_, route_length_m_);
                if (along_route_m_ < 0.0) along_route_m_ += route_length_m_;
            }

            // Find the leg containing this distance.
            size_t leg = 0;
            while (leg + 2 < route_.size() && leg_start_m_[leg + 1] <= along_route_m_) ++leg;
            current_leg_ = int(leg);

            const glm::dvec3 a = route_[leg];
            const glm::dvec3 b = route_[leg + 1];
            const double leg_length = glm::length(b - a);
            const double t = leg_length > 1e-6
                                 ? (along_route_m_ - leg_start_m_[leg]) / leg_length
                                 : 0.0;
            const glm::dvec3 here = a + (b - a) * glm::clamp(t, 0.0, 1.0);

            position_enu = glm::dvec2(here);
            mission_altitude_enu_ = here.z;
            const glm::dvec3 along = b - a;
            velocity_enu = glm::length(glm::dvec2(along)) > 1e-9
                               ? glm::normalize(glm::dvec2(along)) * speed_mps_
                               : glm::dvec2(0.0);

            // Bank into the corner at each waypoint, approximated from the
            // heading change across the next one. Without it the aircraft
            // pivots flat, and a gimbal that has to work against bank is half
            // the reason this simulation exists.
            if (leg + 2 < route_.size()) {
                const glm::dvec2 next = glm::dvec2(route_[leg + 2] - b);
                const glm::dvec2 current = glm::dvec2(along);
                if (glm::length(next) > 1e-6 && glm::length(current) > 1e-6) {
                    const glm::dvec2 u = glm::normalize(current), v = glm::normalize(next);
                    const double turn = std::atan2(u.x * v.y - u.y * v.x, glm::dot(u, v));
                    // Spread the turn over the last 80 m of the leg.
                    const double distance_to_corner = leg_length * (1.0 - t);
                    const double blend = glm::clamp(1.0 - distance_to_corner / 80.0, 0.0, 1.0);
                    turn_rate = -turn * blend * speed_mps_ / 80.0;
                }
            }
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
    // A mission carries its own altitudes, already resolved against whatever
    // frame each waypoint was written in; overriding them with the AGL knob
    // would discard the one thing the plan actually specifies.
    glm::dvec3 enu;
    if (pattern_ == Pattern::Mission && !route_.empty()) {
        enu = glm::dvec3(position_enu.x, position_enu.y, mission_altitude_enu_);
    } else {
        const double ground = terrain.sampleHeight(position_enu.x, position_enu.y);
        enu = glm::dvec3(position_enu.x, position_enu.y, ground + height_agl_m_);
    }

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
