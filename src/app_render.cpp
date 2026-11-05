// Everything App draws: the 3D world view, the simulated camera inset, the
// HUD overlay, and the screenshot path that captures all three. Split out of
// app.cpp because this is the one part of the App class that is purely about
// putting pixels on screen - it reads simulation state but never changes it.

#include "app.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

namespace georay {
namespace {

glm::vec3 toFloat(const glm::dvec3& v) { return glm::vec3(float(v.x), float(v.y), float(v.z)); }

// Fixed-decimal formatting in one call, where std::to_string always gives
// six decimals and std::ostringstream needs three lines per number.
std::string num(double value, int decimals) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
    return buffer;
}

// A hemisphere letter rather than a sign. Six decimal places is ~11 cm at
// the equator - one more digit than this tool can justify, one fewer and it
// would round away differences it's meant to show.
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

void App::renderWorldView() {
    view_camera_.setAspect(float(width_) / float(height_));
    const glm::mat4 view_projection = view_camera_.viewProjection();
    const glm::vec3 eye = view_camera_.position();

    // One screen pixel is this many world units per metre of distance, which
    // is what keeps the overlay ribbons a constant thickness on screen.
    const double per_pixel = 2.0 * std::tan(glm::radians(55.0) * 0.5) / std::max(height_, 1);
    scene_->beginWorld(view_projection, eye, ground_mode_, dark_ground_, per_pixel);
    scene_->drawGround();

    const double glyph = overlayScale();
    PrimitiveBatch& lines = scene_->world();
    scene_->addFrameAxes(300.0 * glyph);

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
    if (!planned_items_.empty()) {
        // Flight-altitude ENU for each point, resampled from the terrain
        // fresh every frame - robust to a re-anchor moving the frame under
        // the plan, the same way resolveAltitude's terrain-relative case
        // works. -1 as the current leg: nothing is flying this route yet, so
        // no leg should be highlighted.
        std::vector<glm::dvec3> planned_route_enu;
        planned_route_enu.reserve(planned_items_.size());
        for (const Waypoint& item : planned_items_) {
            const glm::dvec3 xy = frame_.fromGeodetic(
                Geodetic(item.latitude_deg, item.longitude_deg, frame_.origin().altitude_m));
            const double ground_z = terrain_->sampleHeight(xy.x, xy.y);
            planned_route_enu.push_back(glm::dvec3(xy.x, xy.y, ground_z + item.altitude_m));
        }
        scene_->addMissionRoute(planned_route_enu, -1, *terrain_);
    }
    scene_->addAircraft(uav_, frame_, *terrain_, 28.0 * glyph);
    // The frustum is edges only. Filling its volume was tried and dropped:
    // two of the four side faces are always near edge-on, so they read as
    // stray triangles rather than as a beam, and the thick edges plus the
    // filled footprint already say the same thing more clearly.
    scene_->addCameraFrustum(uav_, camera_, frame_, footprint_, 2500.0);
    if (show_footprint_) {
        // The imaged area is the thing this tool is about, so it carries
        // the strongest fill of anything on screen.
        scene_->addFootprintFill(footprint_, glm::vec4(0.30f, 0.92f, 0.95f, 0.20f));
        scene_->addFootprint(footprint_, *terrain_);
    }

    if (hover_valid_) scene_->addPickRay(hover_result_, *terrain_, 2500.0, 25.0 * glyph);
    if (have_pick_) scene_->addPickRay(pick_result_, *terrain_, 2500.0, 25.0 * glyph);

    // The gimbal's tracking target, so it is clear what the camera is locked
    // onto even when the boresight is off the bottom of the frustum.
    if (flight_.gimbalMode() == FlightSimulator::GimbalMode::TrackPoint) {
        const glm::dvec2 t = flight_.trackTargetEnu();
        scene_->addGroundMarker(glm::dvec3(t.x, t.y, terrain_->sampleHeight(t.x, t.y)),
                                colours::boresight, 14.0 * glyph);
    }

    scene_->endWorld();
}

void App::renderSimulatedCamera() {
    const glm::vec4 rect = cameraRect();
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

    // The inset is the camera's own image, so its pixel scale is the
    // camera's, not the 3D view's.
    const double per_pixel =
        2.0 * std::tan(glm::radians(camera_.verticalFovDeg()) * 0.5) / rect.w;
    scene_->beginWorld(view_projection, eye, ground_mode_, dark_ground_, per_pixel);
    scene_->drawGround();

    PrimitiveBatch& lines = scene_->world();
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

    auto edgeRect = [&](float x, float y, float w, float h, const glm::vec4& colour) {
        lines.addSegment({x, y, 0}, {x + w, y, 0}, colour);
        lines.addSegment({x + w, y, 0}, {x + w, y + h, 0}, colour);
        lines.addSegment({x + w, y + h, 0}, {x, y + h, 0}, colour);
        lines.addSegment({x, y + h, 0}, {x, y, 0}, colour);
    };

    auto panel = [&](float x, float y, float w, float h) {
        fills.addQuad({x, y}, {w, h}, colours::panel);
        // A one-pixel frame: enough to separate the panel from the terrain
        // behind it without drawing attention to itself.
        edgeRect(x, y, w, h, colours::panel_edge);
    };

    // A running cursor down the left column, so panels stack from whatever
    // the one above actually needed.
    float column_y = pad;
    const float column_w = 20.0f * text.advance(s) + 15.0f * text.advance(s);

    // ---- platform state -------------------------------------------------
    {
        const float w = column_w;
        const float h = 11.0f * line_h + 2.0f * pad;
        panel(pad, column_y, w, h);
        Cursor c{text, pad * 2.0f, column_y + pad, s, line_h};

        c.line("PLATFORM", colours::text_primary);
        c.row("LAT", formatLatitude(uav_.position.latitude_deg), colours::text_value);
        c.row("LON", formatLongitude(uav_.position.longitude_deg), colours::text_value);
        // Height above ground has to be worked out entirely in ENU: the
        // terrain's heights are ENU, the geodetic altitude is above the
        // ellipsoid, and subtracting one from the other silently adds the
        // frame origin's elevation to the answer.
        const glm::dvec3 camera_enu = cameraPositionEnu(uav_, frame_);
        const double agl = camera_enu.z - terrain_->sampleHeight(camera_enu.x, camera_enu.y);
        c.row("HEIGHT AGL", num(agl, 1) + " M", colours::text_value);
        c.row("HEADING", num(uav_.attitude.yaw_deg, 1) + "^ " + compass(uav_.attitude.yaw_deg),
              colours::text_value);
        c.blank();
        c.line("GIMBAL", colours::text_primary);
        c.row("MODE", FlightSimulator::gimbalModeName(flight_.gimbalMode()),
              colours::text_good);
        c.row("TILT DOWN", num(uav_.gimbal.tilt_deg, 2) + "^", colours::text_value);
        column_y += h + pad;
    }

    // ---- simulation state ---------------------------------------------
    {
        const float w = column_w;
        float rows = 6.0f;
        const bool show_mission = has_mission_ && (rows += 2.0f, true);
        const bool show_planning =
            (planning_mode_ || !planned_items_.empty()) && (rows += 1.0f, true);
        const float h = rows * line_h + 2.0f * pad;
        panel(pad, column_y, w, h);
        Cursor c{text, pad * 2.0f, column_y + pad, s, line_h};

        c.line(options_.scene_name.empty() ? std::string("SIMULATION") : options_.scene_name,
               colours::text_primary);
        c.row("PATTERN", FlightSimulator::patternName(flight_.pattern()), colours::text_good);
        c.row("STATE", paused_ ? "PAUSED" : "RUNNING",
              paused_ ? colours::text_warn : colours::text_good);
        c.row("GROUND SPEED", num(flight_.speed(), 1) + " M/S", colours::text_value);
        c.row("TERRAIN", terrain_->sourceName(),
              using_dem_ ? colours::text_good : colours::text_value);
        c.row("GROUND", groundModeName(ground_mode_), colours::text_good);
        if (show_mission) {
            c.blank();
            c.line("MISSION", colours::text_primary);
            const size_t waypoints = mission_.navigationItems().size();
            c.row("WAYPOINTS", std::to_string(flight_.currentLeg() + 1) + " OF " +
                                   std::to_string(waypoints) + "  " +
                                   num(flight_.routeLength() / 1000.0, 2) + " KM",
                  colours::text_value);
        }
        if (show_planning) {
            c.row("PLANNING", (planning_mode_ ? std::string("ON - ") : std::string("OFF - ")) +
                                  std::to_string(planned_items_.size()) + " WPT",
                  planning_mode_ ? colours::text_good : colours::text_dim);
        }
        column_y += h + pad;
    }

    // ---- the simulated camera's frame and reticle ----------------------
    {
        const glm::vec4 rect = cameraRect();
        edgeRect(rect.x, rect.y, rect.z, rect.w, colours::panel_edge);

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
        const TileSource* credit_source = scene_->ground().activeSource();
        if (credit_source && !credit_source->attribution.empty()) {
            const std::string credit = credit_source->attribution;
            const float credit_h = text.lineHeight(TextRenderer::kSmall);
            const float width = text.textWidth(credit, TextRenderer::kSmall) + 10.0f;
            // Just above the camera pane rather than along the bottom edge,
            // where it was being clipped by the window. A credit line that
            // cannot be read credits nobody.
            const float x = float(width_) - width - 16.0f;
            const float y = rect.y - credit_h - 6.0f;
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
            "F                    FOOTPRINT",
            "V                    CAMERA VIEW SIZE",
            "M                    MAP / SATELLITE / HEIGHT",
            "N                    DARK GROUND",
            "R                    RESET",
            "F12                  SCREENSHOT",
            "",
            "L                    PLANNING MODE",
            "LMB IN 3D VIEW        (PLANNING) ADD WAYPOINT",
            "Z                    UNDO LAST WAYPOINT",
            "C                    CLEAR PLAN",
            "X                    ADD LANDING AT HOME",
            "S                    SAVE PLAN AND FLY IT",
            "",
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

}  // namespace georay
