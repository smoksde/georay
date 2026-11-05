// SDL event handling: the keyboard and mouse bindings, and the dispatch loop
// that reads them each frame. Split out of app.cpp because this is purely
// "what does an input event change", with no rendering or simulation logic
// of its own.

#include "app.hpp"

#include <algorithm>

namespace georay {

void App::handleEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                running_ = false;
                break;

            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    width_ = event.window.data1;
                    height_ = event.window.data2;
                }
                break;

            case SDL_KEYDOWN:
                onKeyDown(event.key.keysym);
                break;

            case SDL_MOUSEBUTTONDOWN:
                onMouseButtonDown(event.button);
                break;

            case SDL_MOUSEBUTTONUP:
                if (event.button.button == SDL_BUTTON_LEFT) dragging_orbit_ = false;
                if (event.button.button == SDL_BUTTON_RIGHT ||
                    event.button.button == SDL_BUTTON_MIDDLE)
                    dragging_pan_ = false;
                break;

            case SDL_MOUSEMOTION:
                onMouseMotion(event.motion);
                break;

            case SDL_MOUSEWHEEL:
                view_camera_.zoom(float(event.wheel.y));
                break;

            default:
                break;
        }
    }
}

void App::onKeyDown(const SDL_Keysym& key) {
    // Manual gimbal control implies the user wants manual gimbal control, so
    // the arrow keys switch the mode rather than fighting the autopilot.
    auto takeManualGimbal = [&]() {
        flight_.setGimbalMode(FlightSimulator::GimbalMode::Fixed);
    };

    switch (key.sym) {
        case SDLK_ESCAPE:
        case SDLK_q:
            running_ = false;
            break;

        case SDLK_SPACE:
            paused_ = !paused_;
            break;

        case SDLK_h:
            show_help_ = !show_help_;
            break;
        case SDLK_f:
            show_footprint_ = !show_footprint_;
            break;
        case SDLK_v:
            large_inset_ = !large_inset_;
            break;
        case SDLK_m:
            cycleGroundMode();
            break;
        case SDLK_n:
            dark_ground_ = !dark_ground_;
            break;
        case SDLK_b:
            follow_aircraft_ = !follow_aircraft_;
            break;

        case SDLK_l:
            togglePlanningMode();
            break;
        case SDLK_z:
            undoPlannedWaypoint();
            break;
        case SDLK_c:
            clearPlannedMission();
            break;
        case SDLK_x:
            markPlannedLanding();
            break;
        case SDLK_s:
            savePlannedMission();
            break;

        case SDLK_1:
        case SDLK_2:
        case SDLK_3:
        case SDLK_4: {
            const Terrain::Kind kinds[] = {Terrain::Kind::Flat, Terrain::Kind::Hills,
                                           Terrain::Kind::Valley, Terrain::Kind::Mountains};
            terrain_->generate(kinds[key.sym - SDLK_1]);
            using_dem_ = false;
            // The pick refers to the old surface, so it is no longer an
            // answer to anything.
            have_pick_ = false;
            break;
        }

        case SDLK_0:
            // Back to the real data, if there is any. Switching between
            // measured and synthetic ground on the same scene is the most
            // direct way to see how much the terrain model actually matters.
            if (dem_ && loadDemTerrain()) {
                have_pick_ = false;
            }
            break;

        case SDLK_p: {
            const int next = (int(flight_.pattern()) + 1) % 4;
            flight_.setPattern(FlightSimulator::Pattern(next));
            break;
        }
        case SDLK_g: {
            const int next = (int(flight_.gimbalMode()) + 1) % 3;
            flight_.setGimbalMode(FlightSimulator::GimbalMode(next));
            break;
        }
        case SDLK_t:
            // Point the gimbal at whatever was last geolocated, which is the
            // natural "track that" gesture once you have clicked something.
            if (have_pick_ && pick_result_.valid) {
                flight_.setTrackTargetEnu(glm::dvec2(pick_result_.ground_enu));
                track_target_geodetic_ = pick_result_.ground_geodetic;
                flight_.setGimbalMode(FlightSimulator::GimbalMode::TrackPoint);
            }
            break;

        case SDLK_r:
            resetSimulation();
            break;

        case SDLK_LEFTBRACKET:
            camera_.setHorizontalFovDeg(camera_.horizontalFovDeg() - 2.0);
            break;
        case SDLK_RIGHTBRACKET:
            camera_.setHorizontalFovDeg(camera_.horizontalFovDeg() + 2.0);
            break;

        case SDLK_COMMA:
            flight_.setHeightAboveGround(flight_.heightAboveGround() - 10.0);
            break;
        case SDLK_PERIOD:
            flight_.setHeightAboveGround(flight_.heightAboveGround() + 10.0);
            break;

        case SDLK_MINUS:
            flight_.setSpeed(flight_.speed() - 2.0);
            break;
        case SDLK_EQUALS:
            flight_.setSpeed(flight_.speed() + 2.0);
            break;

        case SDLK_SEMICOLON:
            flight_.setRadius(flight_.radius() - 25.0);
            break;
        case SDLK_QUOTE:
            flight_.setRadius(flight_.radius() + 25.0);
            break;

        case SDLK_LEFT:
            takeManualGimbal();
            uav_.gimbal.pan_deg -= 2.0;
            break;
        case SDLK_RIGHT:
            takeManualGimbal();
            uav_.gimbal.pan_deg += 2.0;
            break;
        case SDLK_UP:
            takeManualGimbal();
            uav_.gimbal.tilt_deg = std::min(95.0, uav_.gimbal.tilt_deg + 2.0);
            break;
        case SDLK_DOWN:
            takeManualGimbal();
            uav_.gimbal.tilt_deg = std::max(-20.0, uav_.gimbal.tilt_deg - 2.0);
            break;

        case SDLK_PAGEUP:
            sim_rate_ = std::min(8.0, sim_rate_ * 2.0);
            break;
        case SDLK_PAGEDOWN:
            sim_rate_ = std::max(0.125, sim_rate_ * 0.5);
            break;

        case SDLK_F12:
            writeScreenshot("georay.bmp");
            break;

        default:
            break;
    }
}

void App::onMouseButtonDown(const SDL_MouseButtonEvent& event) {
    glm::dvec2 image_pixel;
    if (event.button == SDL_BUTTON_LEFT && windowToImagePixel(event.x, event.y, image_pixel)) {
        pickPixel(image_pixel);
        return;  // a click in the inset picks; it must not also start an orbit
    }

    // Planning claims left clicks in the main view for placing waypoints,
    // same as the inset claims them for picking - either way, not an orbit.
    if (event.button == SDL_BUTTON_LEFT && planning_mode_) {
        addPlannedWaypoint(event.x, event.y);
        return;
    }

    if (event.button == SDL_BUTTON_LEFT) dragging_orbit_ = true;
    if (event.button == SDL_BUTTON_RIGHT || event.button == SDL_BUTTON_MIDDLE)
        dragging_pan_ = true;
}

void App::onMouseMotion(const SDL_MouseMotionEvent& event) {
    // Orbiting changes the viewing angle, not the subject, so it deliberately
    // leaves follow mode alone; panning below does turn it off.
    if (dragging_orbit_) {
        view_camera_.orbit(float(event.xrel) * 0.3f, float(-event.yrel) * 0.3f);
    }
    if (dragging_pan_) {
        // Panning is an explicit statement that the user wants to look
        // somewhere other than at the aircraft.
        follow_aircraft_ = false;
        view_camera_.pan(glm::vec2(float(event.xrel), float(event.yrel)));
    }

    glm::dvec2 image_pixel;
    hover_valid_ = windowToImagePixel(event.x, event.y, image_pixel);
    if (hover_valid_) hover_pixel_ = image_pixel;
}

}  // namespace georay
