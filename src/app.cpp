#include "app.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
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
        map_is_imagery_[layer] = map_sources_[layer]->imagery;
    }

    map_box_ = mapBox();
    updateMapTextures(/*force=*/true);
    for (int layer = 0; layer < 2; ++layer) {
        if (!map_sources_[layer]) continue;
        const MapTexture::Result& r = map_results_[layer];
        std::cout << "georay: " << layer_names[layer] << " " << map_sources_[layer]->name
                  << " z" << r.zoom << " " << r.width << "x" << r.height << " "
                  << r.metres_per_pixel << " m/px, " << r.tiles_present << "/"
                  << r.tiles_wanted << " tiles";
        if (r.tiles_present == 0) std::cout << " - nothing cached for this area";
        std::cout << "\n";
    }

    // Start on the requested layer, falling back to whichever has pixels -
    // asking for satellite and silently getting bare terrain because nothing
    // was cached would be confusing.
    map_layer_ = -1;
    if (options_.map_mode == "street" && map_results_[1].tiles_present > 0) {
        map_layer_ = 1;
    } else if (options_.map_mode != "off") {
        for (int layer = 0; layer < 2; ++layer) {
            if (map_results_[layer].tiles_present > 0) { map_layer_ = layer; break; }
        }
    }

    if (!options_.mission_path.empty()) loadMissionFile(options_.mission_path);

    if (geoid_.isIdentity() && using_dem_) {
        // Worth saying out loud: without it every elevation this tool reports
        // is wrong by the separation, which is tens of metres in most of the
        // populated world.
        std::cerr << "georay: no geoid separation given, so DEM heights stay orthometric.\n"
                  << "georay: pass --geoid-separation (about 47 for southern Germany) "
                     "or --geoid.\n";
    }

    rebuildTerrainMesh();
    return true;
}

MercatorBox App::mapBox() const {
    // The terrain window's own extent, so map and ground line up edge to
    // edge. Slightly larger, because the terrain mesh reaches exactly to the
    // half-extent and a texture that stopped there would show its clamp
    // colour along the rim.
    return MercatorBox::around(frame_origin_.latitude_deg, frame_origin_.longitude_deg,
                               terrain_->halfExtent() * 1.02);
}

int App::wantedMapZoom(const TileSource& source) const {
    // What one screen pixel covers on the ground, from whichever of the two
    // views is more demanding. The simulated camera usually wins - it is
    // close to the ground - but when it is pointed at the horizon the 3D view
    // is the one that needs the detail.
    const double view_mpp = 2.0 * double(view_camera_.distance()) *
                            std::tan(glm::radians(55.0) * 0.5) / std::max(height_, 1);
    double camera_mpp = view_mpp;
    if (hover_result_.valid && hover_result_.gsd_m > 0.0) {
        camera_mpp = hover_result_.gsd_m;
    } else if (pick_result_.valid && pick_result_.gsd_m > 0.0) {
        camera_mpp = pick_result_.gsd_m;
    }
    const double wanted = std::min(view_mpp, camera_mpp);

    // Two ceilings, and the lower wins.
    //
    // What the view wants, rounded up so the map is never coarser than the
    // screen - and what the stitched texture can actually hold, rounded
    // *down* for the opposite reason. A zoom finer than the pixel budget
    // downloads four times the tiles for a picture that gets averaged back
    // to the same thing.
    const int from_view = zoomForResolution(frame_origin_.latitude_deg, wanted,
                                            source.min_zoom, source.max_zoom);
    const double box_width_m = terrain_->halfExtent() * 2.04;
    const double budget_mpp = box_width_m / std::max(options_.map_max_pixels, 256);
    const int from_budget = zoomNotFinerThan(frame_origin_.latitude_deg, budget_mpp,
                                             source.min_zoom, source.max_zoom);
    return std::min(from_view, from_budget);
}

bool App::updateMapTextures(bool force) {
    if (!tiles_) return false;
    bool changed = false;

    for (int layer = 0; layer < 2; ++layer) {
        const TileSource* source = map_sources_[layer];
        if (!source) continue;

        // Only the visible layer is worth stitching. The other one costs the
        // same decode and upload for pixels nobody is looking at, and M
        // rebuilds it on the spot when it becomes visible.
        if (!force && map_layer_ != layer) continue;

        const int zoom = wantedMapZoom(*source);
        if (!force && zoom == map_zoom_[layer] && map_results_[layer].ok) continue;

        map_results_[layer] = map_textures_[layer].build(*tiles_, *source, map_box_, zoom,
                                                         options_.map_max_pixels);
        map_zoom_[layer] = map_results_[layer].zoom;
        changed = true;

        // Ask for whatever was missing. The fetcher deduplicates and caps
        // itself, so repeating a request for a tile the server does not have
        // costs nothing.
        if (fetcher_) {
            for (const auto& tile : map_results_[layer].missing) {
                fetcher_->request(*source, map_results_[layer].zoom, tile.first, tile.second);
            }
        }

        if (map_results_[layer].tiles_present == 0) {
            scene_->clearMapLayer(layer);
            continue;
        }
        if (!scene_->setMapTexture(layer, map_textures_[layer].pixels().data(),
                                   map_textures_[layer].width(),
                                   map_textures_[layer].height())) {
            std::cerr << "georay: " << scene_->error() << "\n";
            scene_->clearMapLayer(layer);
        }
    }
    return changed;
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
    // The box moves with the frame, so the textures and the UVs that index
    // them both have to be rebuilt - in that order, since the mesh bakes the
    // box the textures were stitched for.
    map_box_ = mapBox();
    updateMapTextures(/*force=*/true);
    rebuildTerrainMesh();

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
    view_camera_.setTarget(glm::vec3(float(view_enu.x), float(view_enu.y),
                                     float(terrain_->sampleHeight(view_enu.x, view_enu.y))));

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

void App::rebuildTerrainMesh() {
    scene_->buildTerrain(*terrain_, frame_, map_box_);
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

bool App::mapVisible() const { return map_layer_ >= 0 && scene_->hasMap(map_layer_); }

bool App::mapLayerHasPixels(int layer) const {
    return layer >= 0 && layer < 2 && map_sources_[layer] && map_results_[layer].tiles_present > 0;
}

bool App::mapIsImagery() const {
    return map_layer_ >= 0 && map_layer_ < 2 && map_is_imagery_[map_layer_];
}

const char* App::mapLayerName() const {
    if (map_layer_ < 0) return "OFF - BARE TERRAIN";
    return map_layer_ == 0 ? "SATELLITE" : "STREET MAP";
}

bool App::showStructures() const {
    // They exist to give synthetic terrain recognisable features with real
    // parallax. Real elevation data and real imagery already have both.
    return !using_dem_ && !mapLayerHasPixels(0) && !mapLayerHasPixels(1);
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
        target = glm::vec3(float(centre.x), float(centre.y),
                           float(terrain_->sampleHeight(centre.x, centre.y)));
        distance = std::max(distance, float(glm::length(glm::dvec2(hi - lo))) * 0.9f);
    }
    view_camera_.setTarget(target);
    view_camera_.setDistance(distance);

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
        case SDLK_m: {
            // Cycle satellite -> street -> off, skipping layers that are not
            // loaded, so M always does something visible.
            for (int step = 0; step < 3; ++step) {
                map_layer_ = (map_layer_ + 2) % 3 - 1;  // 0 -> 1 -> -1 -> 0
                if (map_layer_ < 0 || map_sources_[map_layer_]) break;
            }
            // The layer just switched to may never have been stitched, since
            // only the visible one is kept up to date.
            if (map_layer_ >= 0) updateMapTextures(/*force=*/true);
            break;
        }
        case SDLK_TAB:
            // Which pane V enlarges. Only meaningful with two of them.
            focused_camera_ = (focused_camera_ + 1) % kCameraPanes;
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
            rebuildTerrainMesh();
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
                rebuildTerrainMesh();
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

float App::answerPanelWidth() const {
    // Two columns of thirty characters, which is what the widest row needs.
    // Derived from the font rather than fixed, so the bottom row still fits
    // if the baked sizes change.
    return 60.0f * scene_->text().advance(TextRenderer::kNormal) + 30.0f;
}

bool App::cameraPaneVisible(int index) const {
    // Enlarging one pane hides the other: two panes at half the window each
    // would not fit, and overlapping them would hide the readouts.
    return !large_inset_ || index == focused_camera_;
}

glm::vec4 App::cameraRect(int index) const {
    const float margin = 16.0f;
    const float gap = 10.0f;
    const float aspect = float(camera_.aspectRatio());

    float h = float(height_) * (large_inset_ ? 0.50f : 0.28f);
    float w = h * aspect;

    // Never let the panes eat the window, and never let them run over the
    // geolocation readout they share the bottom row with: on a narrow window
    // the width is the binding constraint, not the height.
    const int panes = large_inset_ ? 1 : kCameraPanes;
    const float available = float(width_) - answerPanelWidth() - 2.0f * margin -
                            gap * float(panes);
    if (w * float(panes) > available) {
        w = available / float(panes);
        h = w / aspect;
    }

    const float y = float(height_) - h - margin;
    if (large_inset_) return glm::vec4(float(width_) - w - margin, y, w, h);

    // Right-aligned, simulated camera to the left of the live one, so the
    // pane you interact with sits next to the readouts it drives.
    const float right_x = float(width_) - w - margin;
    const float x = (index == kLiveCamera) ? right_x : right_x - w - gap;
    return glm::vec4(x, y, w, h);
}

bool App::windowToImagePixel(int x, int y, glm::dvec2& out) const {
    // Only the simulated pane maps to camera pixels. The live pane shows
    // footage from a real aircraft whose pose georay does not know, so a
    // click there has no ray to trace - see renderLiveCameraPlaceholder().
    if (!cameraPaneVisible(kSimulatedCamera)) return false;
    const glm::vec4 rect = cameraRect(kSimulatedCamera);
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
        view_camera_.setTarget(glm::vec3(float(target.x), float(target.y),
                                         float(terrain_->sampleHeight(target.x, target.y))));
        view_target_geodetic_ = track_target_geodetic_;
        have_view_target_ = true;
    }

    // Tiles arriving in the background change what a stitch would produce.
    // Throttled, because a stitch decodes and uploads a few hundred tiles and
    // doing it on every delivery would stutter the whole flight.
    seconds_since_restitch_ += dt;
    if (fetcher_ && seconds_since_restitch_ > 1.0) {
        // Surface a fetcher problem once rather than per batch: a wrong URL
        // or a missing curl otherwise shows up only as a map that never
        // arrives, which looks like a slow network.
        const std::string error = fetcher_->lastError();
        if (!error.empty() && error != reported_fetch_error_) {
            reported_fetch_error_ = error;
            std::cerr << "georay: tile fetch: " << error << "\n";
        }

        const int delivered = fetcher_->fetched();
        if (delivered != last_seen_fetched_) {
            last_seen_fetched_ = delivered;
            seconds_since_restitch_ = 0.0;
            updateMapTextures(/*force=*/true);
        }
    }
    // The view may have moved enough to want a different zoom.
    updateMapTextures(/*force=*/false);

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
    if (cameraPaneVisible(kSimulatedCamera)) renderSimulatedCamera();
    renderHud();

    SDL_GL_SwapWindow(window_);
    ++frames_rendered_;
}

void App::renderWorldView() {
    view_camera_.setAspect(float(width_) / float(height_));
    const glm::mat4 view_projection = view_camera_.viewProjection();
    const glm::vec3 eye = view_camera_.position();

    scene_->beginWorld(view_projection, eye, *terrain_, show_contours_ ? 25.0f : 0.0f,
                       map_layer_, mapIsImagery());
    scene_->drawTerrain();

    PrimitiveBatch& lines = scene_->world();
    // A grid drawn over a readable map is clutter, not scale.
    if (!mapVisible()) scene_->addGroundGrid(*terrain_, 200.0);
    scene_->addFrameAxes(300.0);

    // Structures, as wireframe boxes standing on the surface. Only on
    // synthetic ground: drawn over real imagery they are inventions sitting
    // among real buildings, and a tool about real coordinates should not put
    // made-up objects on a real map.
    for (const Structure& s : showStructures() ? structures_ : std::vector<Structure>{}) {
        const double base = terrain_->sampleHeight(s.position_enu.x, s.position_enu.y);
        const glm::vec3 centre(float(s.position_enu.x), float(s.position_enu.y),
                               float(base + s.height_m * 0.5));
        lines.addBox(centre,
                     glm::vec3(float(s.half_width_m), float(s.half_width_m),
                               float(s.height_m * 0.5)),
                     colours::landmark);
    }

    if (has_mission_ && flight_.hasMission()) {
        scene_->addMissionRoute(flight_.routeEnu(), flight_.currentLeg(), *terrain_);
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

void App::renderSimulatedCamera() {
    const glm::vec4 rect = cameraRect(kSimulatedCamera);
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

    scene_->beginWorld(view_projection, eye, *terrain_, show_contours_ ? 25.0f : 0.0f,
                       map_layer_, mapIsImagery());
    scene_->drawTerrain();

    PrimitiveBatch& lines = scene_->world();
    if (!mapVisible()) scene_->addGroundGrid(*terrain_, 200.0);
    for (const Structure& s : showStructures() ? structures_ : std::vector<Structure>{}) {
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

    // Named faces rather than a scale multiplier: the sizes are baked, and a
    // caller that scaled one of them would just be blurring it.
    //
    // Two sizes in play. The reference panels - platform, camera, simulation -
    // are dense and read at a glance, so they use the small face; the
    // geolocation answer is the thing you actually read, so it gets the
    // larger one. Every position below is derived from the real line heights
    // rather than from constants, because the baked faces are whatever
    // scripts/bake_font.py produced and hard-coded offsets would collide the
    // moment anyone changes a size.
    const int s = TextRenderer::kSmall;
    const int answer_face = TextRenderer::kNormal;
    const float line_h = text.lineHeight(s);
    const float answer_line_h = text.lineHeight(answer_face);
    const float pad = 10.0f;

    // A small layout helper, so panels line up without a pile of magic
    // offsets at every call site.
    struct Cursor {
        TextRenderer& text;
        float x, y;
        int face;
        float line_h;
        void row(const std::string& label, const std::string& value,
                 const glm::vec4& value_colour) {
            text.add(label, {x, y}, face, colours::text_label);
            // Values at a fixed column, so a changing number does not make
            // the whole row jitter.
            text.add(value, {x + 15.0f * text.advance(face), y}, face, value_colour);
            y += line_h;
        }
        void line(const std::string& s, const glm::vec4& colour) {
            text.add(s, {x, y}, face, colour);
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

    // Where the bottom-left readout begins. The left column has to stop above
    // it: the two share an x range, and a window short enough to make them
    // collide is not unusual - the column grew by three rows the moment
    // streaming diagnostics were added.
    const float answer_rows = 11.0f;
    const float answer_top = float(height_) - (answer_rows * answer_line_h + 2.0f * pad) - pad;

    // A running cursor down the left column, so panels stack from whatever
    // the one above actually needed.
    float column_y = pad;
    const float column_w = 20.0f * text.advance(s) + 15.0f * text.advance(s);

    // ---- platform and camera state ------------------------------------
    {
        const float w = column_w;
        const float h = 19.0f * line_h + 2.0f * pad;
        panel(pad, column_y, w, h);
        Cursor c{text, pad * 2.0f, column_y + pad, s, line_h};

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
        c.row("FX FY CX CY", num(camera_.fx(), 0) + " " + num(camera_.fy(), 0) + " " +
                                 num(camera_.cx(), 0) + " " + num(camera_.cy(), 0),
              colours::text_dim);
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
        column_y += h + pad;
    }

    // ---- simulation state ---------------------------------------------
    {
        const float w = column_w;
        // Budget first, content second. Everything below the base rows is a
        // diagnostic, and a diagnostic drawn over the coordinates it is meant
        // to qualify is worse than one that is absent.
        const float available_rows =
            std::max(9.0f, std::floor((answer_top - pad - column_y - 2.0f * pad) / line_h));

        float rows = 9.0f;
        auto want = [&](float n) {
            if (rows + n > available_rows) return false;
            rows += n;
            return true;
        };
        const bool show_gaps = using_dem_ && dem_report_.posts_filled > 0 && want(1.0f);
        const bool show_dem = using_dem_ && want(1.0f);
        const bool show_map_rows =
            (mapLayerHasPixels(0) || mapLayerHasPixels(1)) && want(mapVisible() ? 2.0f : 1.0f);
        const bool show_anchor = using_dem_ && want(1.0f);
        const bool show_window = map_sources_[0] != nullptr && want(1.0f);
        // Last, because the route it describes is already drawn in the 3D
        // view, while the frame anchor appears nowhere else at all.
        const bool show_mission = has_mission_ && want(3.0f);
        const float h = rows * line_h + 2.0f * pad;
        panel(pad, column_y, w, h);
        Cursor c{text, pad * 2.0f, column_y + pad, s, line_h};

        c.line(options_.scene_name.empty() ? std::string("SIMULATION") : options_.scene_name,
               colours::text_primary);
        c.row("PATTERN", FlightSimulator::patternName(flight_.pattern()), colours::text_good);
        c.row("STATE", paused_ ? "PAUSED" : "RUNNING",
              paused_ ? colours::text_warn : colours::text_good);
        c.row("RATE", num(sim_rate_, 3) + " X", colours::text_value);
        c.row("GROUND SPEED", num(flight_.speed(), 1) + " M/S", colours::text_value);
        c.row("ORBIT RADIUS", num(flight_.radius(), 0) + " M", colours::text_value);
        c.row("TERRAIN", terrain_->sourceName(),
              using_dem_ ? colours::text_good : colours::text_value);
        if (show_map_rows) {
            c.row("MAP MODE", mapLayerName(),
                  mapVisible() ? colours::text_good : colours::text_dim);
            if (mapVisible()) {
                // Ground resolution against the camera's own GSD is the
                // number that says whether a finer zoom could show you
                // anything. Below about one map pixel per camera pixel the
                // view is magnifying the map rather than resolving ground.
                const MapTexture::Result& r = map_results_[map_layer_];
                c.row("MAP RES", num(r.metres_per_pixel, 2) + " M/PX Z" +
                                     std::to_string(r.zoom),
                      colours::text_value);
            }
        }
        if (show_anchor) {
            // The floating origin, made visible. Without this the frame
            // moves silently, and a reader has no way to tell whether the
            // coordinates they are looking at came from one anchor or six.
            c.row("ANCHOR", num(reanchor_distance_m_, 0) + "/" +
                                num(reanchorThreshold(), 0) + " M  " +
                                std::to_string(reanchor_count_) + " MOVES",
                  colours::text_dim);
        }
        if (show_window) {
            // Tile coverage for the visible layer. Anything short of complete
            // means grey holes in the map, which is a missing-data condition
            // and not a rendering fault.
            const int layer = map_layer_ >= 0 ? map_layer_ : 0;
            const MapTexture::Result& r = map_results_[layer];
            const bool complete = r.tiles_wanted > 0 && r.tiles_present == r.tiles_wanted;
            c.row("MAP TILES", std::to_string(r.tiles_present) + "/" +
                                   std::to_string(r.tiles_wanted) +
                                   (complete ? "" : "  " + num(r.coverage() * 100.0, 0) + "%"),
                  complete ? colours::text_dim : colours::text_warn);
        }
        if (show_mission) {
            c.blank();
            c.line("MISSION", colours::text_primary);
            c.row("PLAN", mission_.name.substr(0, 22), colours::text_good);
            const size_t waypoints = mission_.navigationItems().size();
            c.row("WAYPOINTS", std::to_string(flight_.currentLeg() + 1) + " OF " +
                                   std::to_string(waypoints) + "  " +
                                   num(flight_.routeLength() / 1000.0, 2) + " KM",
                  colours::text_value);
        }
        c.row("FPS", num(fps_, 0), colours::text_dim);
        column_y += h + pad;
    }

    // ---- the answer ----------------------------------------------------
    //
    // Laid out in two columns: one deep column of fifteen rows would push the
    // panel past the camera panes, and this is the readout that has to stay
    // on screen.
    {
        const float col_w = 30.0f * text.advance(answer_face);
        const float w = answerPanelWidth();
        const float h = answer_rows * answer_line_h + 2.0f * pad;
        const float x = pad;
        const float y = answer_top;
        panel(x, y, w, h);

        Cursor left{text, x + pad, y + pad, answer_face, answer_line_h};
        const GeolocateResult& r = have_pick_ ? pick_result_ : hover_result_;
        const bool showing_pick = have_pick_;
        left.line(showing_pick ? "GEOLOCATION - PICKED PIXEL" : "GEOLOCATION - CURSOR",
                  colours::text_primary);

        if (!have_pick_ && !hover_valid_) {
            left.blank();
            left.line("MOVE THE CURSOR OVER THE SIMULATED CAMERA", colours::text_dim);
            left.line("AND CLICK A PIXEL TO GEOLOCATE IT.", colours::text_dim);
        } else if (!r.valid) {
            left.blank();
            if (r.pointed_at_sky) {
                left.line("NO GROUND: RAY IS ABOVE THE HORIZON", colours::text_warn);
                left.line("TILT THE GIMBAL DOWN (ARROW KEYS)", colours::text_dim);
            } else if (r.left_coverage) {
                left.line("NO GROUND: RAY LEFT THE TERRAIN TILE", colours::text_warn);
                left.line("THIS IS MISSING DATA, NOT A RESULT.", colours::text_dim);
            } else {
                left.line("NO GROUND FOUND ALONG THE RAY", colours::text_warn);
            }
        } else {
            const glm::dvec2 px = showing_pick ? pick_pixel_ : hover_pixel_;
            left.row("PIXEL", num(px.x, 1) + " , " + num(px.y, 1), colours::text_dim);
            left.row("LATITUDE", formatLatitude(r.ground_geodetic.latitude_deg),
                     colours::text_value);
            left.row("LONGITUDE", formatLongitude(r.ground_geodetic.longitude_deg),
                     colours::text_value);
            left.row("GROUND ELEV", num(r.ground_geodetic.altitude_m, 2) + " M",
                     colours::text_value);
            left.row("ENU E/N", num(r.ground_enu.x, 1) + " / " + num(r.ground_enu.y, 1),
                     colours::text_dim);
            left.row("SLANT RANGE", formatDistance(r.slant_range_m), colours::text_value);
            left.row("GROUND RANGE", formatDistance(r.ground_range_m), colours::text_value);
            left.row("DEPRESSION", num(r.depression_angle_deg, 1) + "^", colours::text_value);
            left.row("PIXEL GSD", num(r.gsd_m * 100.0, 1) + " CM", colours::text_value);

            if (showing_pick) {
                Cursor right{text, x + pad + col_w, y + pad + answer_line_h, answer_face,
                             answer_line_h};
                // A round trip that does not close means the forward and
                // inverse models disagree, and every number here is suspect.
                const bool clean = pick_reprojection_px_ < 0.05;
                right.row("REPROJECTION", num(pick_reprojection_px_, 4) + " PX",
                          clean ? colours::text_good : colours::text_warn);
                right.row("D/TILT", num(sensitivity_tilt_m_per_deg_, 1) + " M PER DEG",
                          colours::text_value);
                right.row("D/HEADING", num(sensitivity_yaw_m_per_deg_, 1) + " M PER DEG",
                          colours::text_value);
                right.row("D/ALTITUDE", num(sensitivity_alt_m_per_m_, 2) + " M PER M",
                          colours::text_value);

                // The flat-earth comparison belongs with the pixel it is
                // about, not in a panel of its own - there is nowhere left
                // for one once there are two camera panes.
                if (show_flat_comparison_ && pick_flat_result_.valid) {
                    right.blank();
                    const double error =
                        glm::length(glm::dvec2(pick_result_.ground_enu) -
                                    glm::dvec2(pick_flat_result_.ground_enu));
                    right.row("FLAT-EARTH", formatDistance(error) + " OFF",
                              error > 5.0 ? colours::text_warn : colours::text_good);
                    right.row("  ITS ELEV",
                              num(pick_flat_result_.ground_geodetic.altitude_m, 1) + " M VS " +
                                  num(pick_result_.ground_geodetic.altitude_m, 1),
                              colours::text_dim);
                }
            }
        }
    }

    // ---- the live-feed pane --------------------------------------------
    //
    // Deliberately empty. It is where footage from a real aircraft will go,
    // and an empty pane that says so is more honest than one showing the
    // simulation twice - which would make it impossible to tell, at a
    // glance, which picture is real.
    if (cameraPaneVisible(kLiveCamera)) {
        const glm::vec4 rect = cameraRect(kLiveCamera);
        fills.addQuad({rect.x, rect.y}, {rect.z, rect.w},
                      glm::vec4(0.055f, 0.065f, 0.08f, 1.0f));

        const glm::vec4 edge = focused_camera_ == kLiveCamera ? colours::text_label
                                                              : colours::panel_edge;
        lines.addSegment({rect.x, rect.y, 0}, {rect.x + rect.z, rect.y, 0}, edge);
        lines.addSegment({rect.x + rect.z, rect.y, 0}, {rect.x + rect.z, rect.y + rect.w, 0},
                         edge);
        lines.addSegment({rect.x + rect.z, rect.y + rect.w, 0}, {rect.x, rect.y + rect.w, 0},
                         edge);
        lines.addSegment({rect.x, rect.y + rect.w, 0}, {rect.x, rect.y, 0}, edge);

        // Corner brackets, so an empty pane still reads as a video frame
        // rather than as a hole in the interface.
        const float bracket = std::min(rect.z, rect.w) * 0.09f;
        const glm::vec4 faint(0.30f, 0.36f, 0.42f, 1.0f);
        const float x0 = rect.x + 8.0f, y0 = rect.y + 8.0f;
        const float x1 = rect.x + rect.z - 8.0f, y1 = rect.y + rect.w - 8.0f;
        lines.addSegment({x0, y0, 0}, {x0 + bracket, y0, 0}, faint);
        lines.addSegment({x0, y0, 0}, {x0, y0 + bracket, 0}, faint);
        lines.addSegment({x1, y0, 0}, {x1 - bracket, y0, 0}, faint);
        lines.addSegment({x1, y0, 0}, {x1, y0 + bracket, 0}, faint);
        lines.addSegment({x0, y1, 0}, {x0 + bracket, y1, 0}, faint);
        lines.addSegment({x0, y1, 0}, {x0, y1 - bracket, 0}, faint);
        lines.addSegment({x1, y1, 0}, {x1 - bracket, y1, 0}, faint);
        lines.addSegment({x1, y1, 0}, {x1, y1 - bracket, 0}, faint);

        const std::string title = "LIVE FEED";
        const std::string status = "NO SIGNAL";
        const std::string note = "AWAITING STREAM FROM AIRCRAFT";
        text.add(title, {rect.x + 8.0f, rect.y + 8.0f}, s, colours::text_dim);
        const int big = TextRenderer::kLarge;
        text.add(status,
                 {rect.x + (rect.z - text.textWidth(status, big)) * 0.5f,
                  rect.y + rect.w * 0.5f - text.lineHeight(big) * 0.5f},
                 big, colours::text_warn);
        text.add(note,
                 {rect.x + (rect.z - text.textWidth(note, TextRenderer::kSmall)) * 0.5f,
                  rect.y + rect.w * 0.5f + line_h * 1.4f},
                 TextRenderer::kSmall, colours::text_dim);
    }

    // ---- the simulated camera's frame and reticle ----------------------
    if (cameraPaneVisible(kSimulatedCamera)) {
        const glm::vec4 rect = cameraRect(kSimulatedCamera);
        const glm::vec4 edge = focused_camera_ == kSimulatedCamera ? colours::text_label
                                                                   : colours::panel_edge;
        lines.addSegment({rect.x, rect.y, 0}, {rect.x + rect.z, rect.y, 0}, edge);
        lines.addSegment({rect.x + rect.z, rect.y, 0}, {rect.x + rect.z, rect.y + rect.w, 0},
                         edge);
        lines.addSegment({rect.x + rect.z, rect.y + rect.w, 0}, {rect.x, rect.y + rect.w, 0},
                         edge);
        lines.addSegment({rect.x, rect.y + rect.w, 0}, {rect.x, rect.y, 0}, edge);

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

        // Text over the camera view needs its own backing. Without a map the
        // terrain is dark and light text reads fine; with one, the ground
        // underneath can be white paper, and the label disappears.
        auto labelled = [&](const std::string& content, float y, const glm::vec4& colour) {
            const float width = text.textWidth(content, s) + 10.0f;
            fills.addQuad({rect.x + 2.0f, y - 3.0f}, {width, line_h + 6.0f},
                          glm::vec4(0.04f, 0.06f, 0.09f, 0.82f));
            text.add(content, {rect.x + 7.0f, y}, s, colour);
        };

        labelled("SIMULATED CAMERA", rect.y + 6.0f, colours::text_primary);

        if (hover_valid_ && hover_result_.valid) {
            // A live readout pinned to the bottom of the inset, so a value is
            // available without clicking.
            const std::string live = formatLatitude(hover_result_.ground_geodetic.latitude_deg) +
                                     "  " +
                                     formatLongitude(hover_result_.ground_geodetic.longitude_deg);
            labelled(live, rect.y + rect.w - 8.0f - line_h, colours::text_value);
        }

        // Map credit. Not decoration: OpenStreetMap's tiles are CC-BY-SA and
        // the attribution travels inside the raster precisely so that whoever
        // fetched it gets credited wherever it is shown. Drawn whenever the
        // map is visible, at the bottom of the window where it is out of the
        // way but not hidden.
        if (mapVisible() && map_sources_[map_layer_] &&
            !map_sources_[map_layer_]->attribution.empty()) {
            const std::string credit = map_sources_[map_layer_]->attribution;
            const float credit_h = text.lineHeight(TextRenderer::kSmall);
            const float width = text.textWidth(credit, TextRenderer::kSmall) + 10.0f;
            // Just above the camera panes rather than along the bottom edge,
            // where it was being clipped by the window and overlapped by the
            // live pane. A credit line that cannot be read credits nobody.
            const float x = float(width_) - width - 16.0f;
            const float y = cameraRect(kLiveCamera).y - credit_h - 6.0f;
            fills.addQuad({x - 3.0f, y - 3.0f}, {width + 6.0f, credit_h + 6.0f},
                          glm::vec4(0.04f, 0.06f, 0.09f, 0.75f));
            text.add(credit, {x + 2.0f, y}, TextRenderer::kSmall, colours::text_dim);
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
            "M                    MAP / SATELLITE / OFF",
            "TAB                  FOCUS CAMERA PANE",
            "R                    RESET",
            "F12                  SCREENSHOT",
            "Q / ESC              QUIT",
        };
        const int rows = int(sizeof(keys) / sizeof(keys[0]));
        // Same glyph scale as the rest of the HUD: at scale 1 a 3x5 font is
        // three pixels wide and unreadable, which makes a help panel worse
        // than none.
        const int help_scale = s;
        const float help_line = text.lineHeight(help_scale);
        float w = 0.0f;
        for (const char* row : keys) w = std::max(w, text.textWidth(row, help_scale));
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
        const glm::vec4 rect = cameraRect(kSimulatedCamera);
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
