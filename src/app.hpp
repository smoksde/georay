#pragma once

// The application: window, input, simulation state, and the two views.
//
// The window shows a 3D scene you can orbit, and inset into it the image the
// simulated UAV camera is producing. Clicking a pixel in that inset runs the
// geolocation pipeline on it and reports a latitude and longitude. That
// arrangement is the whole design idea: the inset is rendered with the same
// intrinsics and extrinsics the maths uses, so the ray drawn in the 3D view
// is a falsifiable claim about the pixel you clicked - if the ray misses the
// feature you aimed at, something is wrong, and you can see it rather than
// having to trust a number.

#include <memory>
#include <string>
#include <vector>

#include <SDL2/SDL.h>

#include "flight.hpp"
#include "geo/camera_model.hpp"
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
        // exits. That makes the renderer testable from a script instead of
        // only by looking at it, which matters for a tool whose output is a
        // picture.
        std::string screenshot_path;
        int screenshot_frames = 4;
    };

    App();
    ~App();

    App(const App&) = delete;
    App& operator=(const App&) = delete;

    bool initialise(const Options& options);
    int run();

private:
    // A vertical structure standing on the terrain: a mast, a tower block.
    //
    // They exist to give the camera view recognisable features with real
    // parallax, and to demonstrate something a prototype like this must not
    // paper over - rays intersect the ground model, not the structures, so
    // clicking the top of a tower geolocates the ground behind it. That is
    // the DSM-versus-DTM problem, and it is better shown than explained.
    struct Structure {
        glm::dvec2 position_enu{0.0};
        double height_m = 25.0;
        double half_width_m = 8.0;
    };

    // --- setup -----------------------------------------------------------
    void buildStructures();
    void resetSimulation();

    // --- frame -----------------------------------------------------------
    void handleEvents();
    void onKeyDown(const SDL_Keysym& key);
    void onMouseButtonDown(const SDL_MouseButtonEvent& event);
    void onMouseMotion(const SDL_MouseMotionEvent& event);
    void update(double dt);
    void render();

    void renderWorldView();
    void renderCameraInset();
    void renderHud();

    // --- geometry helpers ------------------------------------------------
    // The inset's rectangle in window pixels, y down from the top.
    glm::vec4 insetRect() const;
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

    std::unique_ptr<Scene> scene_;
    ViewCamera view_camera_;

    // The reference point of the local ENU frame: everything in the
    // simulation is expressed relative to this, and it is the "launch point"
    // a real flight log would carry.
    Geodetic frame_origin_{48.137554, 11.575497, 515.0};
    EnuFrame frame_;

    std::unique_ptr<Terrain> terrain_;
    // A flat ground model at the frame origin's elevation, used only to
    // compute what a flat-earth geolocation would have said. Keeping it as a
    // real Terrain rather than a special case means the comparison goes
    // through exactly the same pipeline.
    std::unique_ptr<Terrain> flat_reference_;

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
    GeolocateResult pick_flat_result_;  // the same pixel against flat ground
    // Round-trip residual for the picked pixel: geolocate, project the answer
    // back, and measure how far from the original pixel it lands. Shown in
    // the HUD because a pipeline that cannot reproduce its own input has no
    // business reporting coordinates.
    double pick_reprojection_px_ = 0.0;
    // Sensitivity of the answer to pose error, by finite difference.
    double sensitivity_tilt_m_per_deg_ = 0.0;
    double sensitivity_yaw_m_per_deg_ = 0.0;
    double sensitivity_alt_m_per_m_ = 0.0;

    // --- interaction -----------------------------------------------------
    bool paused_ = false;
    bool show_help_ = true;
    bool show_footprint_ = true;
    bool show_contours_ = true;
    bool show_flat_comparison_ = true;
    bool large_inset_ = false;
    bool follow_aircraft_ = true;
    bool dragging_orbit_ = false;
    bool dragging_pan_ = false;
    double sim_rate_ = 1.0;

    double fps_ = 0.0;
};

}  // namespace georay
