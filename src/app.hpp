#pragma once

// The application: window, input, simulation state, and the two views.
//
// The window shows a 3D scene you can orbit, with the simulated UAV
// camera's image inset into it. Clicking a pixel there runs the geolocation
// pipeline and reports a latitude/longitude - rendered with the same
// intrinsics and extrinsics the maths uses, so the ray drawn in the 3D view
// is a falsifiable claim: if it misses the feature you aimed at, you see it
// rather than having to trust a number.

#include <memory>
#include <string>
#include <vector>

#include <SDL2/SDL.h>

#include "flight.hpp"
#include "geo/camera_model.hpp"
#include "geo/dem.hpp"
#include "geo/geoid.hpp"
#include "geo/mercator.hpp"
#include "map/maptexture.hpp"
#include "map/tilecache.hpp"
#include "map/tilefetcher.hpp"
#include "map/tilesource.hpp"
#include "geo/mission.hpp"
#include "geo/geolocate.hpp"
#include "geo/pose.hpp"
#include "geo/terrain.hpp"
#include "render/scene.hpp"
#include "render/viewcamera.hpp"

namespace georay {

class App {
public:
    struct Options {
        int window_width = 1600;
        int window_height = 900;
        // When set, the app renders a few frames, writes a BMP here and
        // exits - makes the renderer testable from a script, not just by
        // looking at it.
        std::string screenshot_path;
        int screenshot_frames = 4;

        // Real elevation data. Empty means synthetic terrain - still the
        // default, since it needs no files and runs the moment it's built.
        std::string dem_directory;
        std::string geoid_grid_path;
        // A single separation when no grid is given - see geoid.hpp.
        double geoid_separation_m = 0.0;
        bool geoid_separation_given = false;

        // Named tile sources for the two map layers, satellite and drawn.
        // Both cover the same ground; M switches between them.
        std::string satellite_source = "esri";
        std::string street_source = "basemapde";
        std::string tile_directory = "data/tiles";

        // Fetch missing tiles while flying. On by default - a map that
        // fills itself in is the expected behaviour here, not an opt-in;
        // --offline is for a demo on a train.
        bool online = true;
        int online_tile_limit = 4000;
        double online_delay_s = 0.25;

        // A QGroundControl .plan or a QGC WPL .waypoints list to fly.
        std::string mission_path;

        // Which ground skin to start on: "map", "satellite" or "height".
        std::string map_mode = "satellite";

        // Advance the simulation this many seconds before the first frame -
        // makes a long flight, and the re-anchoring it triggers, reachable
        // without waiting.
        double advance_seconds = 0.0;

        double origin_latitude_deg = 48.137554;
        double origin_longitude_deg = 11.575497;
        double half_extent_m = 3000.0;

        // Initial flight, so a scene preset can describe a sortie rather than
        // just a patch of ground.
        std::string scene_name;
        std::string flight_pattern = "orbit";
        double orbit_radius_m = 520.0;
        double height_agl_m = 220.0;
        double speed_mps = 20.0;

        // Reads a scenes/*.scene file into these fields. Command-line flags
        // given after --scene still win, so a preset is a starting point
        // rather than a straitjacket.
        static bool loadScene(const std::string& path, Options& options, std::string& error);
    };

    App();
    ~App();

    App(const App&) = delete;
    App& operator=(const App&) = delete;

    bool initialise(const Options& options);
    int run();

private:
    // A vertical structure standing on the terrain: a mast, a tower block.
    // Gives the camera view recognisable parallax, and demonstrates the
    // DSM-versus-DTM problem - rays intersect the ground model, not the
    // structures, so clicking a tower's top geolocates the ground behind it.
    struct Structure {
        glm::dvec2 position_enu{0.0};
        double height_m = 25.0;
        double half_width_m = 8.0;
    };

    // --- setup -----------------------------------------------------------
    void buildStructures();
    // Whether the synthetic landmark towers belong in the scene at all.
    bool showStructures() const;
    bool mapIsImagery() const { return ground_mode_ == GroundMode::Satellite; }
    // Overlay geometry scales with viewing distance so the aircraft and its
    // markers keep a usable size on screen rather than shrinking to a pixel.
    double overlayScale() const;
    void resetSimulation();
    // Builds the ENU frame and the terrain, from real elevation data when a
    // DEM directory was given and from the synthetic generator otherwise.
    bool buildWorld();
    bool loadDemTerrain();
    bool loadMissionFile(const std::string& path);

    // --- floating origin --------------------------------------------------
    //
    // An ENU frame is a tangent plane; flat ground falls away from it as
    // d^2/2R (half a metre at 2 km, 4.4 km across Brandenburg), which caps
    // how far one frame can usefully reach.
    //
    // Fixed by moving the frame rather than enlarging it: the origin follows
    // the aircraft, so error is bounded by the visible radius instead of the
    // world's size, and the geolocation maths never learns anything changed
    // - it still sees one local frame. The alternative, a frame per terrain
    // tile, would make the ray marcher walk between frames mid-solve - not
    // worth disturbing the most carefully verified code here for the same
    // answer.
    void reanchorFrame(const Geodetic& new_origin);
    // Ground skin, cycled by M.
    void cycleGroundMode();
    // Re-anchors when the aircraft has drifted far enough from the origin
    // that the terrain window no longer comfortably surrounds it.
    void maybeReanchor();
    double reanchorThreshold() const;

    // --- frame -----------------------------------------------------------
    void handleEvents();
    void onKeyDown(const SDL_Keysym& key);
    void onMouseButtonDown(const SDL_MouseButtonEvent& event);
    void onMouseMotion(const SDL_MouseMotionEvent& event);
    void update(double dt);
    void render();

    void renderWorldView();
    void renderSimulatedCamera();
    void renderHud();

    // --- geometry helpers ------------------------------------------------
    // The simulated camera's pane, in window pixels, y down from the top.
    glm::vec4 cameraRect() const;
    // Width of the bottom-left readout, which the camera pane must clear.
    float answerPanelWidth() const;
    // Window pixel -> camera image pixel. Returns false when the point is
    // outside the inset.
    bool windowToImagePixel(int x, int y, glm::dvec2& out) const;
    // View matrix that renders the scene exactly as the simulated camera sees
    // it, given the current pose.
    glm::mat4 cameraViewMatrix() const;

    void pickPixel(const glm::dvec2& image_pixel);
    void writeScreenshot(const std::string& path);

    // --- state -----------------------------------------------------------
    SDL_Window* window_ = nullptr;
    SDL_GLContext gl_context_ = nullptr;
    int width_ = 1600, height_ = 900;
    bool running_ = false;
    Options options_;
    int frames_rendered_ = 0;
    size_t last_resident_chunks_ = 0;
    int frames_since_chunk_change_ = 0;

    std::unique_ptr<Scene> scene_;
    ViewCamera view_camera_;

    // The reference point of the local ENU frame - the "launch point" a
    // real flight log would carry; everything else is relative to this.
    Geodetic frame_origin_{48.137554, 11.575497, 515.0};
    EnuFrame frame_;
    int reanchor_count_ = 0;
    double seconds_since_reanchor_ = 1e9;  // so the first one is never delayed
    double last_dt_ = 0.0;
    double reanchor_distance_m_ = 0.0;  // how far the aircraft is from the origin now

    // Frame-independent copies, kept in geodetic because anything stored
    // only in ENU metres quietly moves when a re-anchor relocates the
    // origin.
    Geodetic track_target_geodetic_;
    Geodetic pattern_centre_geodetic_;
    Geodetic view_target_geodetic_;
    bool have_view_target_ = false;

    std::unique_ptr<Terrain> terrain_;
    Mission mission_;
    bool has_mission_ = false;
    // Only populated when real elevation data was requested; kept alive
    // since the terrain can switch between DEM and synthetic at runtime,
    // and re-reading the tiles each time would stall the frame.
    std::unique_ptr<DemSource> dem_;
    GeoidModel geoid_;
    // Tiles on disk, decoded on demand, shared by both layers (0 satellite,
    // 1 street map).
    std::unique_ptr<TileCache> tiles_;
    std::unique_ptr<TileFetcher> fetcher_;
    std::string reported_fetch_error_;
    const TileSource* map_sources_[2] = {nullptr, nullptr};
    Terrain::DemLoadReport dem_report_;
    bool using_dem_ = false;

    CameraModel camera_;
    UavState uav_;
    FlightSimulator flight_;
    std::vector<Structure> structures_;

    // --- results ---------------------------------------------------------
    Footprint footprint_;
    // The pixel under the cursor, geolocated every frame.
    bool hover_valid_ = false;
    glm::dvec2 hover_pixel_{0.0};
    GeolocateResult hover_result_;
    // The last clicked pixel, which stays put so it can be studied.
    bool have_pick_ = false;
    glm::dvec2 pick_pixel_{0.0};
    GeolocateResult pick_result_;
    // Round-trip residual: geolocate, project the answer back, measure the
    // distance from the original pixel. A pipeline that cannot reproduce
    // its own input has no business reporting coordinates.
    double pick_reprojection_px_ = 0.0;
    // Sensitivity of the answer to pose error, by finite difference.
    double sensitivity_tilt_m_per_deg_ = 0.0;
    double sensitivity_yaw_m_per_deg_ = 0.0;
    double sensitivity_alt_m_per_m_ = 0.0;

    // --- interaction -----------------------------------------------------
    bool paused_ = false;
    bool show_help_ = true;
    bool show_footprint_ = true;
    bool large_inset_ = false;
    // Darkens the ground so the bright overlays stay readable - on by
    // default, since against a paper-white map zoomed out they vanish.
    bool dark_ground_ = true;
    // Ground skin: map, satellite, or height map. M cycles through
    // whichever ones actually loaded.
    GroundMode ground_mode_ = GroundMode::Satellite;
    bool follow_aircraft_ = true;
    bool dragging_orbit_ = false;
    bool dragging_pan_ = false;
    double sim_rate_ = 1.0;

    double fps_ = 0.0;
};

}  // namespace georay
