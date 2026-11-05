#pragma once

// Flies the aircraft, so the tool is a simulation rather than a static
// diagram.
//
// A stationary camera makes a pose-chain error impossible to spot - any
// fixed rotation still produces a plausible-looking ground point. Put the
// aircraft on a known path with the gimbal locked onto a fixed target, and
// a wrong sign anywhere in the chain shows up immediately as a geolocated
// point that drifts, swings the wrong way, or circles the target instead of
// sitting on it.

#include <string>

#include "geo/mission.hpp"
#include "geo/pose.hpp"
#include "geo/terrain.hpp"

namespace georay {

class FlightSimulator {
public:
    enum class Pattern {
        Hold,       // stationary hover: the baseline for reading the HUD
        Orbit,      // circle a fixed point, which is the strongest pose test
        Lawnmower,  // parallel survey lines, as a mapping sortie actually flies
        Transit,    // straight and level on a constant heading
        Mission     // fly a loaded QGroundControl / Pixhawk plan
    };

    enum class GimbalMode {
        Fixed,      // whatever the user dialled in; the aircraft moves under it
        Nadir,      // straight down, stabilised against pitch and roll
        TrackPoint  // locked onto a ground target, as a real operator would
    };

    static const char* patternName(Pattern p);
    static const char* gimbalModeName(GimbalMode m);

    // `centre` is the pattern's reference point in the ENU frame.
    void configure(Pattern pattern, const glm::dvec2& centre_enu);

    void setPattern(Pattern p) { pattern_ = p; }
    Pattern pattern() const { return pattern_; }

    // The pattern's reference point, in ENU. Exposed so the caller can hold
    // the geodetic truth and restore it after a re-anchor - otherwise the
    // orbit centre quietly follows the aircraft and the flight runs away
    // from its own scene.
    void setCentreEnu(const glm::dvec2& centre) { centre_enu_ = centre; }
    const glm::dvec2& centreEnu() const { return centre_enu_; }
    void setGimbalMode(GimbalMode m) { gimbal_mode_ = m; }
    GimbalMode gimbalMode() const { return gimbal_mode_; }

    void setTrackTargetEnu(const glm::dvec2& target) { track_target_enu_ = target; }
    const glm::dvec2& trackTargetEnu() const { return track_target_enu_; }

    void setSpeed(double metres_per_second);
    double speed() const { return speed_mps_; }
    void setRadius(double metres);
    double radius() const { return radius_m_; }
    // Height above the ground directly below, what an operator thinks in;
    // UavState's geodetic altitude is derived from this and the terrain, so
    // the aircraft follows the relief.
    void setHeightAboveGround(double metres);
    double heightAboveGround() const { return height_agl_m_; }

    // Loads a flight plan for Pattern::Mission. Waypoint altitudes are
    // resolved against the terrain here, once, rather than every frame -
    // re-deriving them per frame would make altitude depend on current
    // position. `keep_progress` re-derives the route in a new frame without
    // resetting to waypoint one, for a re-anchor (never for a freshly
    // loaded plan).
    void setMission(const georay::Mission& mission, const Terrain& terrain,
                    const EnuFrame& frame, double geoid_separation_m,
                    bool keep_progress = false);
    const georay::Mission& mission() const { return mission_; }
    bool hasMission() const { return !route_.empty(); }
    // The route in ENU, for drawing it.
    const std::vector<glm::dvec3>& routeEnu() const { return route_; }
    int currentLeg() const { return current_leg_; }
    double routeLength() const { return route_length_m_; }

    void reset() { elapsed_s_ = 0.0; along_route_m_ = 0.0; current_leg_ = 0; }

    // Advances the pattern and writes position, attitude and (unless the
    // gimbal is Fixed) gimbal angles into `state`.
    void update(double dt, const Terrain& terrain, const EnuFrame& frame, UavState& state);

private:
    // Attitude the pattern implies: a turning aircraft banks, and that bank
    // rotates the camera - the difference between a toy and numbers that
    // mean anything.
    Attitude attitudeFor(const glm::dvec2& velocity_enu, double turn_rate_rad_s) const;

    Pattern pattern_ = Pattern::Orbit;
    GimbalMode gimbal_mode_ = GimbalMode::TrackPoint;

    glm::dvec2 centre_enu_{0.0, 0.0};
    glm::dvec2 track_target_enu_{0.0, 0.0};

    double speed_mps_ = 18.0;
    double radius_m_ = 450.0;
    double height_agl_m_ = 180.0;
    double transit_heading_deg_ = 45.0;
    double elapsed_s_ = 0.0;

    // Lawnmower geometry: leg length and the spacing between legs.
    double leg_length_m_ = 900.0;
    double leg_spacing_m_ = 220.0;

    // Mission state: the route is the navigation waypoints in ENU with
    // altitudes already resolved; the aircraft walks it at the commanded
    // ground speed.
    georay::Mission mission_;
    std::vector<glm::dvec3> route_;
    std::vector<double> leg_start_m_;  // cumulative distance at each waypoint
    double route_length_m_ = 0.0;
    double along_route_m_ = 0.0;
    int current_leg_ = 0;
    double mission_altitude_enu_ = 0.0;
};

}  // namespace georay
