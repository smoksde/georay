#include "app.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <vector>

namespace georay {
namespace {

glm::vec3 toFloat(const glm::dvec3& v) { return glm::vec3(float(v.x), float(v.y), float(v.z)); }

// Fixed-decimal formatting. std::to_string always gives six decimals and
// std::ostringstream needs three lines per number; this is one call, and the
// HUD is nothing but numbers.
std::string num(double value, int decimals) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return buffer;
}

// Latitude and longitude with a hemisphere letter rather than a sign. Six
// decimal places is about 11 cm at the equator - one more digit than this
// tool can justify, and one fewer would round away differences it is
// supposed to show.
std::string formatLatitude(double deg) {
    return num(std::fabs(deg), 6) + "^ " + (deg >= 0.0 ? "N" : "S");
}
std::string formatLongitude(double deg) {
    return num(std::fabs(deg), 6) + "^ " + (deg >= 0.0 ? "E" : "W");
}

// Compass point for a heading, which is far easier to sanity-check at a
// glance than three digits.
const char* compass(double heading_deg) {
    static const char* names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    double h = std::fmod(heading_deg, 360.0);
    if (h < 0.0) h += 360.0;
    return names[int(std::floor(h / 45.0 + 0.5)) % 8];
}

std::string formatDistance(double metres) {
    if (std::fabs(metres) < 1000.0) return num(metres, 1) + " M";
    return num(metres / 1000.0, 3) + " KM";
}

}  // namespace

App::App() : frame_(frame_origin_) {}

App::~App() {
    scene_.reset();
    if (gl_context_) SDL_GL_DeleteContext(gl_context_);
    if (window_) SDL_DestroyWindow(window_);
    SDL_Quit();
}

bool App::initialise(const Options& options) {
    options_ = options;
    width_ = options.window_width;
    height_ = options.window_height;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "georay: SDL_Init failed: " << SDL_GetError() << "\n";
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    // 4x multisampling: this view is mostly thin lines, and aliased rays are
    // genuinely harder to judge against the feature they are supposed to hit.
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    window_ = SDL_CreateWindow("georay - UAV pixel geolocation prototype",
                               SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width_, height_,
                               SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (!window_) {
        std::cerr << "georay: SDL_CreateWindow failed: " << SDL_GetError() << "\n";
        return false;
    }

    gl_context_ = SDL_GL_CreateContext(window_);
    if (!gl_context_) {
        std::cerr << "georay: SDL_GL_CreateContext failed: " << SDL_GetError() << "\n";
        return false;
    }
    SDL_GL_MakeCurrent(window_, gl_context_);
    SDL_GL_SetSwapInterval(1);

    glewExperimental = GL_TRUE;
    const GLenum glew_status = glewInit();
    if (glew_status != GLEW_OK) {
        std::cerr << "georay: glewInit failed: " << glewGetErrorString(glew_status) << "\n";
        return false;
    }
    // GLEW's initialisation is known to leave a spurious GL_INVALID_ENUM
    // behind on core profiles. Clearing it here means the first real error
    // this tool hits is not mistaken for that one.
    glGetError();

    scene_ = std::make_unique<Scene>();
    if (!scene_->initialise()) {
        std::cerr << "georay: shader setup failed: " << scene_->error() << "\n";
        return false;
    }

    if (!buildWorld()) return false;

    // A payload in the class of a small UAV mapping camera: 1" sensor,
    // 1920x1080 video out, a mild wide angle.
    camera_ = CameraModel::fromSensor(12.0, 13.2, 1920, 1080);
    // Distortion starts off, and that is a correctness decision rather than a
    // simplification. The simulated view is drawn with a projection matrix,
    // which is linear and so cannot render distortion; the geolocation maths
    // applies it. With a distortion model active the two are therefore
    // different cameras, and a pixel clicked in the rendered image is not the
    // pixel the maths geolocates. Pressing D turns it on precisely to show
    // that divergence - the HUD says so when it is active - but the default
    // state is the one where what you see is what is computed.
    camera_.distortion() = {};

    buildStructures();
    resetSimulation();

    running_ = true;
    return true;
}

bool App::buildWorld() {
    // Geoid model first: the frame's own origin altitude depends on it.
    if (!options_.geoid_grid_path.empty()) {
        std::string error;
        if (!GeoidModel::loadGrid(options_.geoid_grid_path, geoid_, error)) {
            std::cerr << "georay: " << error << "\n";
            return false;
        }
    } else if (options_.geoid_separation_given) {
        geoid_ = GeoidModel::constant(options_.geoid_separation_m);
    }

    // A provisional frame at sea level, only so the DEM can be asked what the
    // ground height at the origin is. The real frame is built from the
    // answer, below: putting the origin at ground level is what keeps ENU z
    // near zero across the scene, which is both easier to read and kinder to
    // float precision in the renderer.
    frame_origin_ = Geodetic(options_.origin_latitude_deg, options_.origin_longitude_deg, 0.0);
    frame_ = EnuFrame(frame_origin_);

    // 513 posts across whatever extent was asked for - about 12 m at the
    // default 6 km, finer than SRTM's 30 m posting, and a surface mesh of
    // around 260k triangles, which draws comfortably.
    terrain_ = std::make_unique<Terrain>(Terrain::Kind::Hills, options_.half_extent_m, 513, 0.0);
    // The flat-earth reference deliberately spans far more than the scene:
    // its whole job is to answer "where would a terrain-unaware system have
    // put this", including for rays that leave the real terrain's coverage.
    flat_reference_ =
        std::make_unique<Terrain>(Terrain::Kind::Flat, options_.half_extent_m * 10.0, 17, 0.0);

    if (!options_.dem_directory.empty()) {
        dem_ = std::make_unique<DemSource>(options_.dem_directory);

        double ground_orthometric = 0.0;
        if (dem_->sampleOrthometric(frame_origin_.latitude_deg, frame_origin_.longitude_deg,
                                    ground_orthometric)) {
            frame_origin_.altitude_m = geoid_.toEllipsoidal(
                frame_origin_.latitude_deg, frame_origin_.longitude_deg, ground_orthometric);
            frame_ = EnuFrame(frame_origin_);
        }

        if (!loadDemTerrain()) {
            // Fall back to synthetic rather than exiting. The tool is still
            // completely usable without real data, and a hard failure here
            // would turn a missing file into "georay does not start".
            std::cerr << "georay: " << dem_report_.error << "\n"
                      << "georay: falling back to synthetic terrain; "
                         "run scripts/fetch_dem.py to get tiles for this area\n";
            terrain_->generate(Terrain::Kind::Hills);
        }
    }

    if (geoid_.isIdentity() && using_dem_) {
        // Worth saying out loud: without it every elevation this tool reports
        // is wrong by the separation, which is tens of metres in most of the
        // populated world.
        std::cerr << "georay: no geoid separation given, so DEM heights stay orthometric.\n"
                  << "georay: pass --geoid-separation (about 47 for southern Germany) "
                     "or --geoid.\n";
    }

    scene_->buildTerrain(*terrain_);
    return true;
}

bool App::loadDemTerrain() {
    if (!dem_) return false;
    dem_report_ = terrain_->loadFromDem(*dem_, geoid_, frame_);
    using_dem_ = dem_report_.ok;
    if (using_dem_) {
        std::cout << "georay: loaded " << dem_report_.tiles_used.size() << " tile(s), "
                  << dem_report_.posts_from_data << "/" << dem_report_.posts_total
                  << " posts from data";
        if (dem_report_.posts_filled > 0) {
            std::cout << ", " << dem_report_.posts_filled << " interpolated across gaps";
        }
        std::cout << "\ngeoray: ground at the origin " << dem_report_.origin_orthometric_m
                  << " m orthometric, geoid " << geoid_.description() << "\n";
    }
    return using_dem_;
}

void App::buildStructures() {
    // Placed deterministically, spread across the middle of the terrain where
    // the aircraft actually flies. Heights vary so the parallax between them
    // is obvious as the camera moves.
    const double offsets[][3] = {
        {-520.0, 180.0, 42.0},  {-180.0, -420.0, 28.0}, {260.0, 90.0, 35.0},
        {620.0, -260.0, 22.0},  {-60.0, 520.0, 48.0},   {420.0, 640.0, 18.0},
        {-700.0, -640.0, 31.0}, {840.0, 320.0, 26.0},   {120.0, -760.0, 38.0},
    };
    // Positions are given for a 3 km half-extent and scaled to whatever the
    // scene actually is, so they stay inside the terrain at any extent.
    const double scale = terrain_ ? terrain_->halfExtent() / 3000.0 : 1.0;
    structures_.clear();
    for (const auto& o : offsets) {
        Structure s;
        s.position_enu = glm::dvec2(o[0], o[1]) * scale;
        s.height_m = o[2];
        s.half_width_m = 6.0 + o[2] * 0.12;
        structures_.push_back(s);
    }
}

void App::resetSimulation() {
    flight_.configure(FlightSimulator::Pattern::Orbit, glm::dvec2(0.0, 0.0));
    flight_.setGimbalMode(FlightSimulator::GimbalMode::TrackPoint);
    flight_.setTrackTargetEnu(glm::dvec2(0.0, 0.0));
    flight_.setHeightAboveGround(220.0);
    flight_.setRadius(520.0);
    flight_.setSpeed(20.0);

    uav_ = UavState();
    uav_.camera_offset_body = glm::dvec3(0.25, 0.0, 0.35);
    // One update with no time step, so the first frame already has a valid
    // pose rather than the default one at the origin.
    flight_.update(0.0, *terrain_, frame_, uav_);

    view_camera_.setTarget(glm::vec3(0.0f, 0.0f, float(terrain_->sampleHeight(0, 0))));
    // Back off far enough to frame the terrain's relief as well as its
    // extent. Scaling by the horizontal size alone is fine on a plain and
    // puts the camera inside the mountain on real alpine terrain, where the
    // scene is 2 km tall and only 6 km wide.
    const float relief = float(terrain_->maxHeight() - terrain_->minHeight());
    view_camera_.setDistance(std::max(float(terrain_->halfExtent()) * 0.5f, relief * 1.5f));

    have_pick_ = false;
    hover_valid_ = false;
}

// --------------------------------------------------------------------------
// Input
// --------------------------------------------------------------------------

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
        case SDLK_c:
            show_contours_ = !show_contours_;
            break;
        case SDLK_x:
            show_flat_comparison_ = !show_flat_comparison_;
            break;
        case SDLK_v:
            large_inset_ = !large_inset_;
            break;
        case SDLK_b:
            follow_aircraft_ = !follow_aircraft_;
            break;

        case SDLK_d:
            // Toggling distortion is a live demonstration of how much the
            // lens model is worth: at the frame edges it moves the answer by
            // metres. See the comment in initialise() for why it starts off.
            if (camera_.distortion().isIdentity()) {
                camera_.distortion() = {-0.14, 0.035, -0.004, 0.0004, -0.0003};
            } else {
                camera_.distortion() = {};
            }
            break;

        case SDLK_1:
        case SDLK_2:
        case SDLK_3:
        case SDLK_4: {
            const Terrain::Kind kinds[] = {Terrain::Kind::Flat, Terrain::Kind::Hills,
                                           Terrain::Kind::Valley, Terrain::Kind::Mountains};
            terrain_->generate(kinds[key.sym - SDLK_1]);
            using_dem_ = false;
            scene_->buildTerrain(*terrain_);
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
                scene_->buildTerrain(*terrain_);
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

// --------------------------------------------------------------------------
// Geometry helpers
// --------------------------------------------------------------------------

glm::vec4 App::insetRect() const {
    const float margin = 16.0f;
    const float fraction = large_inset_ ? 0.52f : 0.30f;
    float h = float(height_) * fraction;
    float w = h * float(camera_.aspectRatio());
    // Never let the inset eat the window: on a narrow window the width is the
    // binding constraint, not the height.
    const float max_w = float(width_) - 2.0f * margin;
    if (w > max_w) {
        w = max_w;
        h = w / float(camera_.aspectRatio());
    }
    return glm::vec4(float(width_) - w - margin, float(height_) - h - margin, w, h);
}

bool App::windowToImagePixel(int x, int y, glm::dvec2& out) const {
    const glm::vec4 rect = insetRect();
    if (x < rect.x || x > rect.x + rect.z || y < rect.y || y > rect.y + rect.w) return false;
    out.x = double(x - rect.x) / double(rect.z) * double(camera_.imageWidth());
    out.y = double(y - rect.y) / double(rect.w) * double(camera_.imageHeight());
    return true;
}

glm::mat4 App::cameraViewMatrix() const { return glViewMatrix(uav_, frame_); }

void App::pickPixel(const glm::dvec2& image_pixel) {
    have_pick_ = true;
    pick_pixel_ = image_pixel;
    pick_result_ = geolocatePixel(uav_, camera_, *terrain_, frame_, image_pixel);

    // What the same pixel would have given on the flat-earth assumption, for
    // the error readout.
    pick_flat_result_ = geolocatePixel(uav_, camera_, *flat_reference_, frame_, image_pixel);

    if (!pick_result_.valid) {
        pick_reprojection_px_ = 0.0;
        sensitivity_tilt_m_per_deg_ = 0.0;
        sensitivity_yaw_m_per_deg_ = 0.0;
        sensitivity_alt_m_per_m_ = 0.0;
        return;
    }

    const ProjectResult back = projectGeodetic(uav_, camera_, frame_, pick_result_.ground_geodetic);
    pick_reprojection_px_ = glm::length(back.pixel - image_pixel);

    // Error budget by finite difference: perturb one input, re-geolocate, and
    // measure how far the answer moved. These are the numbers that decide
    // whether a geolocation is worth anything - at a shallow depression angle
    // one degree of gimbal error is tens of metres on the ground, and no
    // amount of GPS accuracy helps.
    auto displacement = [&](const UavState& perturbed) {
        const GeolocateResult r =
            geolocatePixel(perturbed, camera_, *terrain_, frame_, image_pixel);
        if (!r.valid) return 0.0;
        return glm::length(glm::dvec2(r.ground_enu) - glm::dvec2(pick_result_.ground_enu));
    };

    UavState perturbed = uav_;
    perturbed.gimbal.tilt_deg += 1.0;
    sensitivity_tilt_m_per_deg_ = displacement(perturbed);

    perturbed = uav_;
    perturbed.attitude.yaw_deg += 1.0;
    sensitivity_yaw_m_per_deg_ = displacement(perturbed);

    perturbed = uav_;
    perturbed.position.altitude_m += 1.0;
    sensitivity_alt_m_per_m_ = displacement(perturbed);
}

// --------------------------------------------------------------------------
// Simulation
// --------------------------------------------------------------------------

void App::update(double dt) {
    if (!paused_) flight_.update(dt * sim_rate_, *terrain_, frame_, uav_);

    if (follow_aircraft_) {
        // Track the point the camera is looking at rather than the aircraft
        // itself: that is the subject, and keeping it centred is what makes
        // the footprint and the rays readable while the aircraft moves.
        const glm::dvec2 target = flight_.trackTargetEnu();
        view_camera_.setTarget(glm::vec3(float(target.x), float(target.y),
                                         float(terrain_->sampleHeight(target.x, target.y))));
    }

    if (show_footprint_) {
        footprint_ = computeFootprint(uav_, camera_, *terrain_, frame_, 10);
    }

    if (hover_valid_) {
        hover_result_ = geolocatePixel(uav_, camera_, *terrain_, frame_, hover_pixel_);
    }

    // The picked pixel is a pixel, not a place: as the aircraft moves, that
    // pixel looks somewhere else. Re-running it every frame is what makes
    // the simulation a test of the pipeline rather than a snapshot of it.
    if (have_pick_ && !paused_) pickPixel(pick_pixel_);
}

// --------------------------------------------------------------------------
// Rendering
// --------------------------------------------------------------------------

void App::render() {
    glViewport(0, 0, width_, height_);
    glClearColor(0.07f, 0.09f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    renderWorldView();
    renderCameraInset();
    renderHud();

    SDL_GL_SwapWindow(window_);
    ++frames_rendered_;
}

void App::renderWorldView() {
    view_camera_.setAspect(float(width_) / float(height_));
    const glm::mat4 view_projection = view_camera_.viewProjection();
    const glm::vec3 eye = view_camera_.position();

    scene_->beginWorld(view_projection, eye, *terrain_, show_contours_ ? 25.0f : 0.0f);
    scene_->drawTerrain();

    PrimitiveBatch& lines = scene_->world();
    scene_->addGroundGrid(*terrain_, 200.0);
    scene_->addFrameAxes(300.0);

    // Structures, as wireframe boxes standing on the surface.
    for (const Structure& s : structures_) {
        const double base = terrain_->sampleHeight(s.position_enu.x, s.position_enu.y);
        const glm::vec3 centre(float(s.position_enu.x), float(s.position_enu.y),
                               float(base + s.height_m * 0.5));
        lines.addBox(centre,
                     glm::vec3(float(s.half_width_m), float(s.half_width_m),
                               float(s.height_m * 0.5)),
                     colours::landmark);
    }

    scene_->addAircraft(uav_, frame_, *terrain_, 28.0);
    scene_->addCameraFrustum(uav_, camera_, frame_, footprint_, 2500.0);
    if (show_footprint_) scene_->addFootprint(footprint_, *terrain_);

    if (hover_valid_) scene_->addPickRay(hover_result_, *terrain_, 2500.0);
    if (have_pick_) {
        scene_->addPickRay(pick_result_, *terrain_, 2500.0);
        if (show_flat_comparison_ && pick_result_.valid && pick_flat_result_.valid) {
            // The flat-earth answer, and a line joining it to the real one.
            // The length of that line is the error a terrain-unaware system
            // would have made, drawn at the scale it actually happens.
            scene_->addGroundMarker(pick_flat_result_.ground_enu,
                                    glm::vec4(0.95f, 0.35f, 0.45f, 1.0f), 18.0);
            lines.addSegment(toFloat(pick_result_.ground_enu),
                             toFloat(pick_flat_result_.ground_enu),
                             glm::vec4(0.95f, 0.35f, 0.45f, 0.9f));
        }
    }

    // The gimbal's tracking target, so it is clear what the camera is locked
    // onto even when the boresight is off the bottom of the frustum.
    if (flight_.gimbalMode() == FlightSimulator::GimbalMode::TrackPoint) {
        const glm::dvec2 t = flight_.trackTargetEnu();
        scene_->addGroundMarker(glm::dvec3(t.x, t.y, terrain_->sampleHeight(t.x, t.y)),
                                colours::boresight, 14.0);
    }

    scene_->endWorld();
}

void App::renderCameraInset() {
    const glm::vec4 rect = insetRect();
    // GL's viewport origin is the bottom-left of the window; the rect is in
    // top-left pixel coordinates.
    const GLint gl_x = GLint(rect.x);
    const GLint gl_y = GLint(float(height_) - rect.y - rect.w);
    const GLsizei gl_w = GLsizei(rect.z);
    const GLsizei gl_h = GLsizei(rect.w);

    glEnable(GL_SCISSOR_TEST);
    glScissor(gl_x, gl_y, gl_w, gl_h);
    glViewport(gl_x, gl_y, gl_w, gl_h);
    glClearColor(0.55f, 0.66f, 0.78f, 1.0f);  // sky, for the part above the horizon
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // The projection comes from the camera model, not from a convenient
    // perspective call: that is what makes this inset the camera's actual
    // image rather than an approximation of it.
    const glm::mat4 view_projection =
        camera_.glProjectionMatrix(1.0, 60000.0) * cameraViewMatrix();
    const glm::vec3 eye = toFloat(cameraPositionEnu(uav_, frame_));

    scene_->beginWorld(view_projection, eye, *terrain_, show_contours_ ? 25.0f : 0.0f);
    scene_->drawTerrain();

    PrimitiveBatch& lines = scene_->world();
    scene_->addGroundGrid(*terrain_, 200.0);
    for (const Structure& s : structures_) {
        const double base = terrain_->sampleHeight(s.position_enu.x, s.position_enu.y);
        const glm::vec3 centre(float(s.position_enu.x), float(s.position_enu.y),
                               float(base + s.height_m * 0.5));
        lines.addBox(centre,
                     glm::vec3(float(s.half_width_m), float(s.half_width_m),
                               float(s.height_m * 0.5)),
                     colours::landmark);
    }
    if (have_pick_ && pick_result_.valid) {
        scene_->addGroundMarker(pick_result_.ground_enu, colours::pick_hit, 12.0);
    }
    scene_->endWorld();

    glDisable(GL_SCISSOR_TEST);
    glViewport(0, 0, width_, height_);
}

void App::renderHud() {
    scene_->beginOverlay(width_, height_);

    PrimitiveBatch& fills = scene_->panels();
    PrimitiveBatch& lines = scene_->overlay();
    TextRenderer& text = scene_->text();

    const float s = 2.0f;             // glyph scale: 6x10 pixels per character
    const float line_h = TextRenderer::lineHeight(s);
    const float pad = 10.0f;

    // A small layout helper, so panels line up without a pile of magic
    // offsets at every call site.
    struct Cursor {
        TextRenderer& text;
        float x, y, scale, line_h;
        void row(const std::string& label, const std::string& value,
                 const glm::vec4& value_colour) {
            text.add(label, {x, y}, scale, colours::text_label);
            // Values at a fixed column, so a changing number does not make
            // the whole row jitter.
            text.add(value, {x + 15.0f * TextRenderer::glyphAdvance(scale), y}, scale,
                     value_colour);
            y += line_h;
        }
        void line(const std::string& s, const glm::vec4& colour) {
            text.add(s, {x, y}, scale, colour);
            y += line_h;
        }
        void blank() { y += line_h * 0.45f; }
    };

    auto panel = [&](float x, float y, float w, float h) {
        fills.addQuad({x, y}, {w, h}, colours::panel);
        // A one-pixel frame: enough to separate the panel from the terrain
        // behind it without drawing attention to itself.
        lines.addSegment({x, y, 0}, {x + w, y, 0}, colours::panel_edge);
        lines.addSegment({x + w, y, 0}, {x + w, y + h, 0}, colours::panel_edge);
        lines.addSegment({x + w, y + h, 0}, {x, y + h, 0}, colours::panel_edge);
        lines.addSegment({x, y + h, 0}, {x, y, 0}, colours::panel_edge);
    };

    // ---- platform and camera state ------------------------------------
    {
        const float w = 400.0f;
        const float h = 20.0f * line_h + 2.0f * pad;
        panel(pad, pad, w, h);
        Cursor c{text, pad * 2.0f, pad * 2.0f, s, line_h};

        c.line("PLATFORM", colours::text_primary);
        c.row("LAT", formatLatitude(uav_.position.latitude_deg), colours::text_value);
        c.row("LON", formatLongitude(uav_.position.longitude_deg), colours::text_value);
        c.row("ALT ELLIPS", num(uav_.position.altitude_m, 1) + " M", colours::text_value);
        // Height above ground has to be worked out entirely in ENU: the
        // terrain's heights are ENU, the geodetic altitude is above the
        // ellipsoid, and subtracting one from the other silently adds the
        // frame origin's elevation to the answer.
        const glm::dvec3 camera_enu = cameraPositionEnu(uav_, frame_);
        const double agl = camera_enu.z - terrain_->sampleHeight(camera_enu.x, camera_enu.y);
        c.row("HEIGHT AGL", num(agl, 1) + " M", colours::text_value);
        c.row("HEADING", num(uav_.attitude.yaw_deg, 1) + "^ " + compass(uav_.attitude.yaw_deg),
              colours::text_value);
        c.row("PITCH/ROLL", num(uav_.attitude.pitch_deg, 1) + "^ / " +
                                num(uav_.attitude.roll_deg, 1) + "^",
              colours::text_value);
        c.blank();
        c.line("GIMBAL", colours::text_primary);
        c.row("MODE", FlightSimulator::gimbalModeName(flight_.gimbalMode()),
              colours::text_good);
        c.row("PAN", num(uav_.gimbal.pan_deg, 2) + "^", colours::text_value);
        c.row("TILT DOWN", num(uav_.gimbal.tilt_deg, 2) + "^", colours::text_value);
        c.blank();
        c.line("CAMERA", colours::text_primary);
        c.row("IMAGE", std::to_string(camera_.imageWidth()) + " X " +
                           std::to_string(camera_.imageHeight()),
              colours::text_value);
        c.row("FOCAL", num(camera_.focalLengthMm(), 2) + " MM", colours::text_value);
        c.row("FOV H/V", num(camera_.horizontalFovDeg(), 1) + "^ / " +
                             num(camera_.verticalFovDeg(), 1) + "^",
              colours::text_value);
        c.row("FX FY", num(camera_.fx(), 1) + " " + num(camera_.fy(), 1), colours::text_dim);
        c.row("CX CY", num(camera_.cx(), 1) + " " + num(camera_.cy(), 1), colours::text_dim);
        if (camera_.distortion().isIdentity()) {
            c.row("DISTORTION", "OFF - VIEW MATCHES MATHS", colours::text_dim);
        } else {
            // Not a decoration: with distortion on, the rendered image is
            // still an ideal pinhole projection, so clicking near a frame
            // edge geolocates a slightly different ray than the one the
            // picture shows. Saying so is the difference between a
            // demonstration and a trap.
            c.row("DISTORTION", "ON - VIEW IS UNDISTORTED", colours::text_warn);
        }
    }

    // ---- simulation state ---------------------------------------------
    {
        const float w = 400.0f;
        float rows = 9.0f;
        if (using_dem_) rows += dem_report_.posts_filled > 0 ? 2.0f : 1.0f;
        const float h = rows * line_h + 2.0f * pad;
        const float y = pad + 20.0f * line_h + 2.0f * pad + pad;
        panel(pad, y, w, h);
        Cursor c{text, pad * 2.0f, y + pad, s, line_h};

        c.line("SIMULATION", colours::text_primary);
        c.row("PATTERN", FlightSimulator::patternName(flight_.pattern()), colours::text_good);
        c.row("STATE", paused_ ? "PAUSED" : "RUNNING",
              paused_ ? colours::text_warn : colours::text_good);
        c.row("RATE", num(sim_rate_, 3) + " X", colours::text_value);
        c.row("GROUND SPEED", num(flight_.speed(), 1) + " M/S", colours::text_value);
        c.row("ORBIT RADIUS", num(flight_.radius(), 0) + " M", colours::text_value);
        c.row("TERRAIN", terrain_->sourceName(),
              using_dem_ ? colours::text_good : colours::text_value);
        if (using_dem_) {
            c.row("GEOID", geoid_.description(),
                  geoid_.isIdentity() ? colours::text_warn : colours::text_value);
            if (dem_report_.posts_filled > 0) {
                // An interpolated post is an invented elevation. It is often
                // the right thing to do, but never the right thing to hide.
                c.row("DEM GAPS", num(dem_report_.gapFraction() * 100.0, 1) + " PCT FILLED",
                      colours::text_warn);
            }
        }
        if (!show_footprint_) {
            c.row("FOOTPRINT", "HIDDEN (F)", colours::text_dim);
        } else if (footprint_.complete) {
            c.row("FOOTPRINT", num(footprint_.area_m2 / 1e6, 3) + " KM2", colours::text_value);
        } else {
            // An incomplete footprint has no area, and printing one anyway
            // would be inventing a number for the most visible field here.
            c.row("FOOTPRINT", "PARTLY OFF GROUND", colours::text_warn);
        }
        c.row("FPS", num(fps_, 0), colours::text_dim);
    }

    // ---- the answer ----------------------------------------------------
    {
        const float w = 560.0f;
        const float h = 16.0f * line_h + 2.0f * pad;
        const float x = pad;
        const float y = float(height_) - h - pad;
        panel(x, y, w, h);
        Cursor c{text, x + pad, y + pad, s, line_h};

        const GeolocateResult& r = have_pick_ ? pick_result_ : hover_result_;
        const bool showing_pick = have_pick_;
        c.line(showing_pick ? "GEOLOCATION - PICKED PIXEL" : "GEOLOCATION - CURSOR",
               colours::text_primary);

        if (!have_pick_ && !hover_valid_) {
            c.blank();
            c.line("MOVE THE CURSOR OVER THE CAMERA VIEW", colours::text_dim);
            c.line("AND CLICK A PIXEL TO GEOLOCATE IT.", colours::text_dim);
        } else if (!r.valid) {
            c.blank();
            if (r.pointed_at_sky) {
                c.line("NO GROUND: RAY IS ABOVE THE HORIZON", colours::text_warn);
                c.line("TILT THE GIMBAL DOWN (ARROW KEYS)", colours::text_dim);
            } else if (r.left_coverage) {
                c.line("NO GROUND: RAY LEFT THE TERRAIN TILE", colours::text_warn);
                c.line("THIS IS MISSING DATA, NOT A RESULT.", colours::text_dim);
            } else {
                c.line("NO GROUND FOUND ALONG THE RAY", colours::text_warn);
            }
        } else {
            const glm::dvec2 px = showing_pick ? pick_pixel_ : hover_pixel_;
            c.row("PIXEL", num(px.x, 1) + " , " + num(px.y, 1), colours::text_dim);
            c.row("LATITUDE", formatLatitude(r.ground_geodetic.latitude_deg),
                  colours::text_value);
            c.row("LONGITUDE", formatLongitude(r.ground_geodetic.longitude_deg),
                  colours::text_value);
            c.row("GROUND ELEV", num(r.ground_geodetic.altitude_m, 2) + " M",
                  colours::text_value);
            c.row("ENU E/N", num(r.ground_enu.x, 1) + " / " + num(r.ground_enu.y, 1),
                  colours::text_dim);
            c.row("SLANT RANGE", formatDistance(r.slant_range_m), colours::text_value);
            c.row("GROUND RANGE", formatDistance(r.ground_range_m), colours::text_value);
            c.row("DEPRESSION", num(r.depression_angle_deg, 1) + "^", colours::text_value);
            c.row("PIXEL GSD", num(r.gsd_m * 100.0, 1) + " CM", colours::text_value);

            if (showing_pick) {
                c.blank();
                // A round trip that does not close means the forward and
                // inverse models disagree, and every number above is suspect.
                const bool clean = pick_reprojection_px_ < 0.05;
                c.row("REPROJECTION", num(pick_reprojection_px_, 4) + " PX",
                      clean ? colours::text_good : colours::text_warn);
                c.row("D/TILT", num(sensitivity_tilt_m_per_deg_, 1) + " M PER DEG",
                      colours::text_value);
                c.row("D/HEADING", num(sensitivity_yaw_m_per_deg_, 1) + " M PER DEG",
                      colours::text_value);
                c.row("D/ALTITUDE", num(sensitivity_alt_m_per_m_, 2) + " M PER M",
                      colours::text_value);
            }
        }
    }

    // ---- flat-earth comparison ----------------------------------------
    if (show_flat_comparison_ && have_pick_) {
        const float w = 440.0f;
        const float h = 6.0f * line_h + 2.0f * pad;
        const float x = pad + 560.0f + pad;
        const float y = float(height_) - h - pad;
        panel(x, y, w, h);
        Cursor c{text, x + pad, y + pad, s, line_h};

        c.line("FLAT-EARTH COMPARISON", colours::text_primary);
        if (pick_result_.valid && pick_flat_result_.valid) {
            const double error = glm::length(glm::dvec2(pick_result_.ground_enu) -
                                             glm::dvec2(pick_flat_result_.ground_enu));
            c.row("TERRAIN AWARE", num(pick_result_.ground_geodetic.altitude_m, 1) + " M",
                  colours::text_value);
            c.row("FLAT ASSUMED", num(pick_flat_result_.ground_geodetic.altitude_m, 1) + " M",
                  colours::text_dim);
            c.row("HORIZ ERROR", formatDistance(error),
                  error > 5.0 ? colours::text_warn : colours::text_good);
            c.line("THE RED MARKER IS WHERE A TERRAIN-", colours::text_dim);
            c.line("UNAWARE SYSTEM WOULD HAVE PUT IT.", colours::text_dim);
        } else {
            c.line("NO COMPARISON FOR THIS PIXEL.", colours::text_dim);
        }
    }

    // ---- the camera view's own frame and reticle ----------------------
    {
        const glm::vec4 rect = insetRect();
        lines.addSegment({rect.x, rect.y, 0}, {rect.x + rect.z, rect.y, 0}, colours::panel_edge);
        lines.addSegment({rect.x + rect.z, rect.y, 0}, {rect.x + rect.z, rect.y + rect.w, 0},
                         colours::panel_edge);
        lines.addSegment({rect.x + rect.z, rect.y + rect.w, 0}, {rect.x, rect.y + rect.w, 0},
                         colours::panel_edge);
        lines.addSegment({rect.x, rect.y + rect.w, 0}, {rect.x, rect.y, 0}, colours::panel_edge);

        // The principal point, which is where the boresight lands - not
        // necessarily the middle of the frame, and drawing it at the middle
        // would hide exactly the off-centre case this model supports.
        const float cx = rect.x + float(camera_.cx() / camera_.imageWidth()) * rect.z;
        const float cy = rect.y + float(camera_.cy() / camera_.imageHeight()) * rect.w;
        const glm::vec4 reticle = colours::boresight;
        lines.addSegment({cx - 12.0f, cy, 0}, {cx - 3.0f, cy, 0}, reticle);
        lines.addSegment({cx + 3.0f, cy, 0}, {cx + 12.0f, cy, 0}, reticle);
        lines.addSegment({cx, cy - 12.0f, 0}, {cx, cy - 3.0f, 0}, reticle);
        lines.addSegment({cx, cy + 3.0f, 0}, {cx, cy + 12.0f, 0}, reticle);

        auto imageToWindow = [&](const glm::dvec2& px) {
            return glm::vec3(rect.x + float(px.x / camera_.imageWidth()) * rect.z,
                             rect.y + float(px.y / camera_.imageHeight()) * rect.w, 0.0f);
        };

        // The picked pixel, marked in the image where it was picked.
        if (have_pick_) {
            const glm::vec3 p = imageToWindow(pick_pixel_);
            const glm::vec4 colour = pick_result_.valid ? colours::pick_hit : colours::text_warn;
            lines.addSegment(p - glm::vec3(9, 9, 0), p - glm::vec3(2, 2, 0), colour);
            lines.addSegment(p + glm::vec3(2, 2, 0), p + glm::vec3(9, 9, 0), colour);
            lines.addSegment(p + glm::vec3(-9, 9, 0), p + glm::vec3(-2, 2, 0), colour);
            lines.addSegment(p + glm::vec3(2, -2, 0), p + glm::vec3(9, -9, 0), colour);
        }

        text.add("SIMULATED UAV CAMERA", {rect.x + 6.0f, rect.y + 6.0f}, s,
                 colours::text_primary);
        if (hover_valid_ && hover_result_.valid) {
            // A live readout pinned to the bottom of the inset, so a value is
            // available without clicking.
            const std::string live = formatLatitude(hover_result_.ground_geodetic.latitude_deg) +
                                     "  " +
                                     formatLongitude(hover_result_.ground_geodetic.longitude_deg);
            text.add(live, {rect.x + 6.0f, rect.y + rect.w - 6.0f - 5.0f * s}, s,
                     colours::text_value);
        }
    }

    // ---- help ---------------------------------------------------------
    if (show_help_) {
        static const char* const keys[] = {
            "CONTROLS                    H TO HIDE",
            "",
            "LMB IN CAMERA VIEW   GEOLOCATE PIXEL",
            "LMB DRAG             ORBIT 3D VIEW",
            "RMB DRAG             PAN 3D VIEW",
            "WHEEL                ZOOM 3D VIEW",
            "",
            "SPACE                PAUSE / RESUME",
            "PGUP PGDN            SIM RATE",
            "P                    FLIGHT PATTERN",
            "G                    GIMBAL MODE",
            "ARROWS               AIM GIMBAL",
            "T                    TRACK PICKED POINT",
            "B                    FOLLOW TARGET",
            "",
            "1 2 3 4              SYNTHETIC TERRAIN",
            "0                    REAL DEM TERRAIN",
            "[ ]                  CAMERA ZOOM",
            ", .                  HEIGHT AGL",
            "- =                  GROUND SPEED",
            "; '                  ORBIT RADIUS",
            "",
            "D                    LENS DISTORTION",
            "F                    FOOTPRINT",
            "C                    CONTOUR LINES",
            "X                    FLAT-EARTH ERROR",
            "V                    CAMERA VIEW SIZE",
            "R                    RESET",
            "F12                  SCREENSHOT",
            "Q / ESC              QUIT",
        };
        const int rows = int(sizeof(keys) / sizeof(keys[0]));
        // Same glyph scale as the rest of the HUD: at scale 1 a 3x5 font is
        // three pixels wide and unreadable, which makes a help panel worse
        // than none.
        const float help_scale = s;
        const float help_line = TextRenderer::lineHeight(help_scale);
        float w = 0.0f;
        for (const char* row : keys) w = std::max(w, TextRenderer::textWidth(row, help_scale));
        w += 2.0f * pad;
        const float h = rows * help_line + 2.0f * pad;
        // Top-right: the camera inset owns the bottom-right corner, and the
        // readouts own the left, so this is the one place a tall panel fits
        // without covering something that matters.
        const float x = float(width_) - w - pad;
        const float y = pad;
        panel(x, y, w, h);
        float ty = y + pad;
        for (int i = 0; i < rows; ++i) {
            const glm::vec4 colour = (i == 0) ? colours::text_primary : colours::text_dim;
            text.add(keys[i], {x + pad, ty}, help_scale, colour);
            ty += help_line;
        }
    }

    scene_->endOverlay();
}

void App::writeScreenshot(const std::string& path) {
    std::vector<unsigned char> pixels(size_t(width_) * size_t(height_) * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());

    // GL reads bottom-up; BMP via SDL wants the rows in the order the surface
    // declares, so flip here rather than relying on a negative pitch.
    const size_t stride = size_t(width_) * 3;
    std::vector<unsigned char> flipped(pixels.size());
    for (int row = 0; row < height_; ++row) {
        std::copy(pixels.begin() + long((height_ - 1 - row) * stride),
                  pixels.begin() + long((height_ - row) * stride),
                  flipped.begin() + long(row * stride));
    }

    SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(
        flipped.data(), width_, height_, 24, int(stride), 0x0000FF, 0x00FF00, 0xFF0000, 0);
    if (!surface) {
        std::cerr << "georay: screenshot surface failed: " << SDL_GetError() << "\n";
        return;
    }
    if (SDL_SaveBMP(surface, path.c_str()) != 0) {
        std::cerr << "georay: screenshot save failed: " << SDL_GetError() << "\n";
    } else {
        std::cout << "georay: wrote " << path << " (" << width_ << "x" << height_ << ")\n";
    }
    SDL_FreeSurface(surface);
}

int App::run() {
    Uint64 previous = SDL_GetPerformanceCounter();
    const double frequency = double(SDL_GetPerformanceFrequency());
    double fps_accumulator = 0.0;
    int fps_frames = 0;

    // In screenshot mode the cursor is parked over the middle of the camera
    // inset and a pixel is picked, so the captured frame exercises the
    // geolocation path rather than just the empty HUD.
    if (!options_.screenshot_path.empty()) {
        const glm::vec4 rect = insetRect();
        glm::dvec2 image_pixel;
        if (windowToImagePixel(int(rect.x + rect.z * 0.42f), int(rect.y + rect.w * 0.62f),
                               image_pixel)) {
            hover_valid_ = true;
            hover_pixel_ = image_pixel;
            pickPixel(image_pixel);
        }
    }

    while (running_) {
        const Uint64 now = SDL_GetPerformanceCounter();
        double dt = double(now - previous) / frequency;
        previous = now;
        // Clamp the step: a stall (a drag of the window, a breakpoint) would
        // otherwise teleport the aircraft across the map in one frame.
        dt = std::min(dt, 0.1);

        handleEvents();
        update(dt);
        render();

        fps_accumulator += dt;
        ++fps_frames;
        if (fps_accumulator >= 0.5) {
            fps_ = fps_frames / fps_accumulator;
            fps_accumulator = 0.0;
            fps_frames = 0;
        }

        if (!options_.screenshot_path.empty() && frames_rendered_ >= options_.screenshot_frames) {
            writeScreenshot(options_.screenshot_path);
            running_ = false;
        }
    }
    return 0;
}

}  // namespace georay
