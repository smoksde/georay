#include "app.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

namespace georay {

App::App() : frame_(frame_origin_) {}

bool App::Options::loadScene(const std::string& path, Options& options, std::string& error) {
    std::ifstream file(path);
    if (!file) {
        error = "cannot open scene " + path;
        return false;
    }

    std::string line;
    int line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        // Strip comments and surrounding whitespace, then skip blanks. The
        // format is deliberately this dumb: a scene file is a handful of
        // numbers, and a JSON parser would be more code than the thing it
        // reads.
        const size_t hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);

        std::istringstream stream(line);
        std::string key;
        if (!(stream >> key)) continue;

        std::string value;
        std::getline(stream, value);
        const size_t first = value.find_first_not_of(" \t\r");
        const size_t last = value.find_last_not_of(" \t\r");
        value = (first == std::string::npos) ? "" : value.substr(first, last - first + 1);

        auto number = [&](double& target) {
            try {
                target = std::stod(value);
                return true;
            } catch (...) {
                error = path + ":" + std::to_string(line_number) + ": '" + value +
                        "' is not a number";
                return false;
            }
        };

        if (key == "name") {
            options.scene_name = value;
        } else if (key == "origin") {
            if (std::sscanf(value.c_str(), "%lf,%lf", &options.origin_latitude_deg,
                            &options.origin_longitude_deg) != 2) {
                error = path + ":" + std::to_string(line_number) + ": origin wants LAT,LON";
                return false;
            }
        } else if (key == "extent") {
            if (!number(options.half_extent_m)) return false;
        } else if (key == "dem") {
            options.dem_directory = value;
        } else if (key == "map" || key == "map-satellite") {
            options.satellite_source = value;
        } else if (key == "map-street") {
            options.street_source = value;
        } else if (key == "tiles") {
            options.tile_directory = value;
        } else if (key == "online") {
            options.online = (value == "1" || value == "true" || value == "yes");
        } else if (key == "mission") {
            options.mission_path = value;
        } else if (key == "map-mode") {
            options.map_mode = value;
        } else if (key == "advance") {
            if (!number(options.advance_seconds)) return false;
        } else if (key == "geoid") {
            options.geoid_grid_path = value;
        } else if (key == "geoid-separation") {
            if (!number(options.geoid_separation_m)) return false;
            options.geoid_separation_given = true;
        } else if (key == "pattern") {
            options.flight_pattern = value;
        } else if (key == "radius") {
            if (!number(options.orbit_radius_m)) return false;
        } else if (key == "height-agl") {
            if (!number(options.height_agl_m)) return false;
        } else if (key == "speed") {
            if (!number(options.speed_mps)) return false;
        } else {
            // An unknown key is a typo, and silently ignoring it would mean a
            // scene that quietly does not do what it says.
            error = path + ":" + std::to_string(line_number) + ": unknown setting '" + key + "'";
            return false;
        }
    }
    return true;
}

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

    {
        // Worth stating once at start-up: it is the hard ceiling on how much
        // ground a single map raster can cover, and it varies by machine.
        GLint max_texture = 0;
        glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_texture);
        std::cout << "georay: GL " << glGetString(GL_VERSION) << ", max texture "
                  << max_texture << " px\n";
    }

    scene_ = std::make_unique<Scene>();
    if (!scene_->initialise()) {
        std::cerr << "georay: shader setup failed: " << scene_->error() << "\n";
        return false;
    }

    if (!buildWorld()) return false;

    // A payload in the class of a small UAV mapping camera: 1" sensor,
    // 1920x1080 video out, a mild wide angle.
    camera_ = CameraModel::fromSensor(12.0, 13.2, 1920, 1080);
    // Off: the rendered view uses a linear projection matrix and cannot show
    // distortion, so a distorted camera and the one on screen would disagree
    // about which pixel a ray belongs to.
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

    tiles_ = std::make_unique<TileCache>(options_.tile_directory);
    if (options_.online) {
        if (!TileFetcher::curlAvailable()) {
            std::cerr << "georay: --online needs the curl binary on the PATH; "
                         "staying offline\n";
        } else {
            fetcher_ = std::make_unique<TileFetcher>(options_.tile_directory,
                                                     options_.online_tile_limit,
                                                     options_.online_delay_s);
            fetcher_->start();
            std::cout << "georay: online, fetching missing tiles (limit "
                      << options_.online_tile_limit << " this session)\n";
        }
    }
    const std::string requested[2] = {options_.satellite_source, options_.street_source};
    const char* layer_names[2] = {"satellite", "street map"};
    for (int layer = 0; layer < 2; ++layer) {
        if (requested[layer].empty() || requested[layer] == "none") continue;
        map_sources_[layer] = findTileSource(requested[layer]);
        if (!map_sources_[layer]) {
            std::cerr << "georay: unknown tile source '" << requested[layer] << "' for the "
                      << layer_names[layer] << " layer; known sources are "
                      << tileSourceNames() << "\n";
            continue;
        }
        std::cout << "georay: " << layer_names[layer] << " source "
                  << map_sources_[layer]->name << " (to zoom "
                  << map_sources_[layer]->max_zoom << ")\n";
    }

    scene_->ground().setSources(map_sources_[1], map_sources_[0]);
    if (options_.map_mode == "map" || options_.map_mode == "street") {
        ground_mode_ = GroundMode::Map;
    } else if (options_.map_mode == "height" || options_.map_mode == "heightmap") {
        ground_mode_ = GroundMode::HeightMap;
    } else {
        ground_mode_ = GroundMode::Satellite;
    }
    // A mode whose source never resolved would show bare chunks forever.
    if (ground_mode_ == GroundMode::Satellite && !map_sources_[0]) {
        ground_mode_ = map_sources_[1] ? GroundMode::Map : GroundMode::HeightMap;
    } else if (ground_mode_ == GroundMode::Map && !map_sources_[1]) {
        ground_mode_ = map_sources_[0] ? GroundMode::Satellite : GroundMode::HeightMap;
    }
    scene_->ground().setMode(ground_mode_);

    if (!options_.mission_path.empty()) loadMissionFile(options_.mission_path);

    if (geoid_.isIdentity() && using_dem_) {
        // Worth saying out loud: without it every elevation this tool reports
        // is wrong by the separation, which is tens of metres in most of the
        // populated world.
        std::cerr << "georay: no geoid separation given, so DEM heights stay orthometric.\n"
                  << "georay: pass --geoid-separation (about 47 for southern Germany) "
                     "or --geoid.\n";
    }

    return true;
}

double App::reanchorThreshold() const {
    // Far enough out that a pattern which stays near its own centre never
    // triggers at all. An orbit sits at a constant distance from the scene
    // origin, so a threshold below its radius re-anchors forever - rebuilding
    // the terrain window and re-uploading the map every twenty seconds to
    // move the origin somewhere no better than where it was.
    //
    // At 0.6 there is still a window-and-a-half of loaded ground ahead of the
    // aircraft when the move does happen, which is what the threshold is
    // actually for.
    return terrain_->halfExtent() * 0.6;
}

void App::reanchorFrame(const Geodetic& new_origin) {
    // Everything that lives in ENU metres is about to mean something
    // different. Capture it as geodetic first, restore it after.
    const Geodetic track = track_target_geodetic_;
    const Geodetic pattern_centre = pattern_centre_geodetic_;
    const glm::vec3 old_view_target = view_camera_.target();
    const Geodetic view_target =
        have_view_target_ ? view_target_geodetic_
                          : frame_.toGeodetic(glm::dvec3(old_view_target.x, old_view_target.y,
                                                         old_view_target.z));

    frame_origin_ = new_origin;
    // Anchor at ground level so ENU z stays near zero across the window,
    // which keeps the numbers readable and the float positions small.
    double ground_orthometric = 0.0;
    if (dem_ && dem_->sampleOrthometric(new_origin.latitude_deg, new_origin.longitude_deg,
                                        ground_orthometric)) {
        frame_origin_.altitude_m = geoid_.toEllipsoidal(
            new_origin.latitude_deg, new_origin.longitude_deg, ground_orthometric);
    }
    frame_ = EnuFrame(frame_origin_);

    // The terrain window is resampled around the new origin from the DEM,
    // which streams its own one-degree tiles from disk as they are needed -
    // so the world is as large as the tiles on disk, not as large as one
    // window.
    if (dem_) loadDemTerrain();
    // Every chunk's geometry is in ENU, and ENU has just come to mean
    // something else. They are rebuilt around the new origin on the next
    // update, a couple at a time.
    scene_->ground().invalidateGeometry();

    // Put the frame-dependent state back where it was on the ground.
    track_target_geodetic_ = track;
    const glm::dvec3 track_enu = frame_.fromGeodetic(track);
    flight_.setTrackTargetEnu(glm::dvec2(track_enu));

    // The pattern's own centre, likewise. Missing this was not subtle: the
    // orbit centre followed the frame, the frame followed the aircraft, and
    // the aircraft flew steadily away from its own scene until the terrain
    // window was 94 per cent interpolated.
    pattern_centre_geodetic_ = pattern_centre;
    flight_.setCentreEnu(glm::dvec2(frame_.fromGeodetic(pattern_centre)));

    view_target_geodetic_ = view_target;
    have_view_target_ = true;
    const glm::dvec3 view_enu = frame_.fromGeodetic(view_target);
    view_camera_.setTarget(glm::vec3(float(view_enu.x), float(view_enu.y), 0.0f));

    // The route's ENU coordinates moved too; rebuild it from the plan, whose
    // waypoints are geodetic and therefore unaffected.
    if (has_mission_) {
        const double separation =
            geoid_.separationAt(frame_origin_.latitude_deg, frame_origin_.longitude_deg);
        flight_.setMission(mission_, *terrain_, frame_, separation, /*keep_progress=*/true);
    }

    ++reanchor_count_;
}

void App::maybeReanchor() {
    // Only with real elevation data. The synthetic generators are functions
    // of ENU position, so moving the origin would slide the hills out from
    // under the aircraft - they are a fixed test scene, not a world.
    if (!using_dem_ || !dem_) return;
    seconds_since_reanchor_ += last_dt_;

    reanchor_distance_m_ = horizontalDistance(frame_origin_, uav_.position);
    if (reanchor_distance_m_ < reanchorThreshold()) return;

    // A dwell, so a track that skims the threshold cannot rebuild the world
    // every frame. A re-anchor costs a terrain resample and a map upload -
    // cheap once, ruinous sixty times a second.
    if (seconds_since_reanchor_ < 5.0) return;
    seconds_since_reanchor_ = 0.0;

    reanchorFrame(Geodetic(uav_.position.latitude_deg, uav_.position.longitude_deg, 0.0));
    reanchor_distance_m_ = horizontalDistance(frame_origin_, uav_.position);
}

bool App::loadMissionFile(const std::string& path) {
    Mission loaded;
    std::string error;
    if (!loadMission(path, loaded, error)) {
        std::cerr << "georay: " << error << "\n";
        return false;
    }

    mission_ = std::move(loaded);
    has_mission_ = true;

    const double separation = geoid_.separationAt(frame_origin_.latitude_deg,
                                                  frame_origin_.longitude_deg);
    flight_.setMission(mission_, *terrain_, frame_, separation);
    flight_.setPattern(FlightSimulator::Pattern::Mission);
    flight_.reset();

    std::cout << "georay: mission '" << mission_.name << "' - "
              << mission_.navigationItems().size() << " waypoints, "
              << flight_.routeLength() << " m of route";
    if (mission_.has_home) {
        std::cout << ", home " << mission_.home.latitude_deg << ", "
                  << mission_.home.longitude_deg;
    }
    std::cout << "\n";

    // Warn when the plan is not where the scene is. Flying a Berlin mission
    // over Munich terrain would otherwise just look like the aircraft left.
    double south, north, west, east;
    if (mission_.bounds(south, north, west, east)) {
        const double distance = horizontalDistance(
            frame_origin_, Geodetic((south + north) * 0.5, (west + east) * 0.5, 0.0));
        if (distance > terrain_->halfExtent() * 1.5) {
            std::cerr << "georay: this mission is " << (distance / 1000.0)
                      << " km from the scene origin; pass --origin near it\n";
        }
    }
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

double App::overlayScale() const {
    // Constant apparent size, near enough. A 28 m aircraft is right at the
    // default standoff and invisible from ten kilometres, and the geolocation
    // markers have the same problem - the thing you are meant to be looking
    // at should not be the hardest thing to see.
    return std::max(1.0, double(view_camera_.distance()) / 700.0);
}

void App::cycleGroundMode() {
    // Map -> satellite -> height map, skipping a tile-backed mode whose
    // source never resolved so the key always does something visible.
    for (int step = 0; step < 3; ++step) {
        ground_mode_ = GroundMode((int(ground_mode_) + 1) % 3);
        if (ground_mode_ == GroundMode::HeightMap) break;
        if (ground_mode_ == GroundMode::Map && map_sources_[1]) break;
        if (ground_mode_ == GroundMode::Satellite && map_sources_[0]) break;
    }
    scene_->ground().setMode(ground_mode_);
}

bool App::showStructures() const {
    // They exist to give synthetic terrain recognisable features with real
    // parallax. Real elevation data and real imagery already have both.
    return !using_dem_ && !map_sources_[0] && !map_sources_[1];
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
    FlightSimulator::Pattern pattern = FlightSimulator::Pattern::Orbit;
    if (options_.flight_pattern == "lawnmower") pattern = FlightSimulator::Pattern::Lawnmower;
    else if (options_.flight_pattern == "transit") pattern = FlightSimulator::Pattern::Transit;
    else if (options_.flight_pattern == "hold") pattern = FlightSimulator::Pattern::Hold;
    else if (options_.flight_pattern == "mission") pattern = FlightSimulator::Pattern::Mission;

    // A loaded plan wins over the scene's default pattern. Asking for a
    // mission and then flying a generic orbit around the scene centre would
    // ignore the one thing the file specifies.
    if (has_mission_ && flight_.hasMission()) pattern = FlightSimulator::Pattern::Mission;

    flight_.configure(pattern, glm::dvec2(0.0, 0.0));
    flight_.setGimbalMode(FlightSimulator::GimbalMode::TrackPoint);
    flight_.setTrackTargetEnu(glm::dvec2(0.0, 0.0));
    track_target_geodetic_ = frame_.toGeodetic(glm::dvec3(0.0, 0.0, 0.0));
    pattern_centre_geodetic_ = frame_.toGeodetic(glm::dvec3(flight_.centreEnu(), 0.0));
    flight_.setHeightAboveGround(options_.height_agl_m);
    flight_.setRadius(options_.orbit_radius_m);
    flight_.setSpeed(options_.speed_mps);

    uav_ = UavState();
    uav_.camera_offset_body = glm::dvec3(0.25, 0.0, 0.35);
    // One update with no time step, so the first frame already has a valid
    // pose rather than the default one at the origin.
    flight_.update(0.0, *terrain_, frame_, uav_);

    // Back off far enough to frame the terrain's relief as well as its
    // extent. Scaling by the horizontal size alone is fine on a plain and
    // puts the camera inside the mountain on real alpine terrain, where the
    // scene is 2 km tall and only 6 km wide.
    const float relief = float(terrain_->maxHeight() - terrain_->minHeight());
    float distance = std::max(float(terrain_->halfExtent()) * 0.5f, relief * 1.5f);
    glm::vec3 target(0.0f, 0.0f, float(terrain_->sampleHeight(0, 0)));

    // With a plan loaded, frame the plan instead of the terrain: the route is
    // the subject, and a view that cuts half of it off makes the waypoints
    // impossible to judge against the ground.
    if (has_mission_ && flight_.routeEnu().size() >= 2) {
        glm::dvec3 lo = flight_.routeEnu().front(), hi = lo;
        for (const glm::dvec3& p : flight_.routeEnu()) {
            lo = glm::min(lo, p);
            hi = glm::max(hi, p);
        }
        const glm::dvec3 centre = (lo + hi) * 0.5;
        target = glm::vec3(float(centre.x), float(centre.y), 0.0f);
        distance = std::max(distance, float(glm::length(glm::dvec2(hi - lo))) * 0.9f);
    }
    view_camera_.setTarget(target);
    view_camera_.setDistance(distance);

    have_pick_ = false;
    hover_valid_ = false;
}

// --------------------------------------------------------------------------
// Geometry helpers
// --------------------------------------------------------------------------

float App::answerPanelWidth() const {
    // Two columns of thirty characters, which is what the widest row needs.
    // Derived from the font rather than fixed, so the bottom row still fits
    // if the baked sizes change.
    return 60.0f * scene_->text().advance(TextRenderer::kNormal) + 30.0f;
}

glm::vec4 App::cameraRect() const {
    const float margin = 16.0f;
    const float aspect = float(camera_.aspectRatio());

    float h = float(height_) * (large_inset_ ? 0.50f : 0.28f);
    float w = h * aspect;

    // Never let the pane eat the window, and never let it run over the
    // geolocation readout it shares the bottom row with: on a narrow window
    // the width is the binding constraint, not the height.
    const float available = float(width_) - answerPanelWidth() - 2.0f * margin;
    if (w > available) {
        w = available;
        h = w / aspect;
    }

    const float y = float(height_) - h - margin;
    return glm::vec4(float(width_) - w - margin, y, w, h);
}

bool App::windowToImagePixel(int x, int y, glm::dvec2& out) const {
    const glm::vec4 rect = cameraRect();
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
}

// --------------------------------------------------------------------------
// Simulation
// --------------------------------------------------------------------------

void App::update(double dt) {
    last_dt_ = dt * sim_rate_;
    if (!paused_) flight_.update(last_dt_, *terrain_, frame_, uav_);

    // Before anything reads the frame this tick: if the aircraft has flown
    // far enough, move the origin to it and rebuild the window around it.
    maybeReanchor();

    if (follow_aircraft_) {
        // Track the point the camera is looking at rather than the aircraft
        // itself: that is the subject, and keeping it centred is what makes
        // the footprint and the rays readable while the aircraft moves.
        const glm::dvec2 target = flight_.trackTargetEnu();
        // At the ground plane, not at the terrain height: the ground renders
        // flat, so aiming at a relief that is not drawn would leave the
        // subject off-centre by the local elevation.
        view_camera_.setTarget(glm::vec3(float(target.x), float(target.y), 0.0f));
        view_target_geodetic_ = track_target_geodetic_;
        have_view_target_ = true;
    }

    // Stream the ground around wherever the camera is looking. This is the
    // whole of the loading decision: chunks near the view appear, chunks
    // behind it are dropped, and missing tiles are asked for as they are
    // found.
    scene_->ground().update(view_camera_, frame_, *terrain_, dem_.get(), geoid_, *tiles_,
                            fetcher_.get(), terrain_->halfExtent() * 4.0);

    if (fetcher_) {
        // Surface a fetcher problem once rather than per batch: a wrong URL
        // or a missing curl otherwise shows up only as a map that never
        // arrives, which looks like a slow network.
        const std::string error = fetcher_->lastError();
        if (!error.empty() && error != reported_fetch_error_) {
            reported_fetch_error_ = error;
            std::cerr << "georay: tile fetch: " << error << "\n";
        }
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
    renderSimulatedCamera();
    renderHud();

    SDL_GL_SwapWindow(window_);
    ++frames_rendered_;
}

int App::run() {
    Uint64 previous = SDL_GetPerformanceCounter();
    const double frequency = double(SDL_GetPerformanceFrequency());
    double fps_accumulator = 0.0;
    int fps_frames = 0;

    // Fast-forward before anything is drawn. Stepped at a fixed rate rather
    // than in one jump, because the flight patterns and the re-anchoring are
    // both incremental and a single enormous dt would skip straight past
    // them.
    if (options_.advance_seconds > 0.0) {
        const double step = 0.05;
        const int steps = int(options_.advance_seconds / step);
        for (int i = 0; i < steps; ++i) update(step);
        std::cout << "georay: advanced " << options_.advance_seconds << " s, "
                  << reanchor_count_ << " frame re-anchor(s)\n";
    }

    // In screenshot mode the cursor is parked over the middle of the camera
    // inset and a pixel is picked, so the captured frame exercises the
    // geolocation path rather than just the empty HUD.
    if (!options_.screenshot_path.empty()) {
        const glm::vec4 rect = cameraRect();
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

        // A screenshot has to wait for the ground to arrive. Chunks are built
        // a few per frame on purpose, so capturing on frame four would
        // photograph an empty plane - and capturing as soon as the first few
        // are ready photographs a patch, because the resident set is still
        // growing. Settled means both: nothing pending, and nothing new
        // appearing for a while.
        const size_t resident = scene_->ground().residentCount();
        if (resident != last_resident_chunks_) {
            last_resident_chunks_ = resident;
            frames_since_chunk_change_ = 0;
        } else {
            ++frames_since_chunk_change_;
        }
        const bool ground_settled = scene_->ground().pendingCount() == 0 && resident > 0 &&
                                    frames_since_chunk_change_ > 45;
        const bool waited_long_enough =
            frames_rendered_ >= options_.screenshot_frames &&
            (ground_settled || frames_rendered_ > 400);
        if (!options_.screenshot_path.empty() && waited_long_enough) {
            writeScreenshot(options_.screenshot_path);
            running_ = false;
        }
    }
    return 0;
}

}  // namespace georay
