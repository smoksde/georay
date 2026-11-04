#pragma once

// Owns the GL resources and wraps the two drawing passes the app needs:
//
//   world pass   - the terrain surface plus 3D overlay lines, in ENU metres
//   overlay pass - HUD text and panels, in window pixels
//
// The app decides *what* to draw; this decides *how*. Keeping the split means
// the geolocation code and the drawing code never appear in the same
// function, which is what stops "it looks right" from substituting for "it
// is right".

#include <memory>
#include <string>
#include <vector>

#include "geo/camera_model.hpp"
#include "geo/geolocate.hpp"
#include "geo/mercator.hpp"
#include "render/groundchunks.hpp"
#include "geo/pose.hpp"
#include "geo/terrain.hpp"
#include "gl/mesh.hpp"
#include "gl/shader.hpp"
#include "gl/text.hpp"

namespace georay {

// Palette, in one place. Colours carry meaning in this tool - a ray is
// orange, a footprint is cyan, an answer is yellow - so they are named rather
// than written inline at each call site.
namespace colours {
inline constexpr glm::vec4 grid{0.35f, 0.40f, 0.46f, 0.55f};
inline constexpr glm::vec4 grid_major{0.52f, 0.58f, 0.64f, 0.75f};
inline constexpr glm::vec4 axis_east{0.90f, 0.35f, 0.35f, 1.0f};
inline constexpr glm::vec4 axis_north{0.40f, 0.85f, 0.45f, 1.0f};
inline constexpr glm::vec4 axis_up{0.45f, 0.60f, 1.00f, 1.0f};
inline constexpr glm::vec4 aircraft{1.00f, 0.95f, 0.70f, 1.0f};
inline constexpr glm::vec4 frustum{0.45f, 0.80f, 0.95f, 0.85f};
inline constexpr glm::vec4 footprint{0.30f, 0.90f, 0.95f, 1.0f};
inline constexpr glm::vec4 boresight{0.55f, 0.95f, 0.70f, 0.9f};
inline constexpr glm::vec4 pick_ray{1.00f, 0.55f, 0.15f, 1.0f};
inline constexpr glm::vec4 pick_hit{1.00f, 0.90f, 0.20f, 1.0f};
inline constexpr glm::vec4 landmark{0.95f, 0.45f, 0.85f, 1.0f};
inline constexpr glm::vec4 route{0.55f, 0.75f, 1.00f, 0.95f};
inline constexpr glm::vec4 route_active{1.00f, 0.80f, 0.35f, 1.0f};
inline constexpr glm::vec4 waypoint{0.70f, 0.85f, 1.00f, 1.0f};
inline constexpr glm::vec4 text_primary{0.94f, 0.96f, 0.98f, 1.0f};
inline constexpr glm::vec4 text_dim{0.62f, 0.68f, 0.74f, 1.0f};
inline constexpr glm::vec4 text_label{0.50f, 0.78f, 0.95f, 1.0f};
inline constexpr glm::vec4 text_value{1.00f, 0.90f, 0.45f, 1.0f};
inline constexpr glm::vec4 text_warn{1.00f, 0.50f, 0.40f, 1.0f};
inline constexpr glm::vec4 text_good{0.55f, 0.95f, 0.60f, 1.0f};
inline constexpr glm::vec4 panel{0.04f, 0.06f, 0.09f, 0.90f};
inline constexpr glm::vec4 panel_edge{0.35f, 0.45f, 0.55f, 0.9f};
}  // namespace colours

class Scene {
public:
    Scene();
    ~Scene();

    // Compiles shaders and builds the font atlas. Returns false and leaves
    // error() set if anything failed, so main can exit with a message rather
    // than present a black window.
    bool initialise();
    const std::string& error() const { return error_; }

    // The ground, as flat chunks streamed around the camera.
    GroundChunks& ground() { return ground_; }
    const GroundChunks& ground() const { return ground_; }

    // --- world pass -------------------------------------------------------
    // `map_layer` selects which raster to drape, or -1 for none.
    // `imagery` tells the shader whether that layer is a photograph or a
    // drawn map, which changes how much shading and contouring it gets.
    // `world_per_pixel_per_metre` converts a screen width in pixels into a
    // world width at a given distance, which is what lets the overlay lines
    // keep a constant thickness on screen. Pass 2*tan(fov/2)/viewport_height.
    void beginWorld(const glm::mat4& view_projection, const glm::vec3& eye_position,
                    GroundMode mode, bool dark, double world_per_pixel_per_metre);
    // Draws every resident ground chunk.
    void drawGround();
    PrimitiveBatch& world() { return world_lines_; }
    // Translucent world-space triangles - the imaged area and the frustum
    // volume. Thin lines alone vanish at a distance; a filled surface still
    // reads as a shape when it is twenty pixels across.
    PrimitiveBatch& worldFills() { return world_fills_; }

    // A segment drawn as a camera-facing ribbon of constant screen width.
    //
    // GL_LINES cannot be widened in a core profile - glLineWidth above 1 is
    // not required to do anything, and on most drivers does not - so a line
    // that has to stay visible from ten kilometres has to be geometry. These
    // go in the fill batch, under the thin lines.
    void addThickSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec4& colour,
                         float width_px);
    void addThickPolyline(const std::vector<glm::vec3>& points, const glm::vec4& colour,
                          float width_px, bool closed);
    void endWorld();

    // --- overlay pass -----------------------------------------------------
    void beginOverlay(int width, int height);
    PrimitiveBatch& panels() { return overlay_fills_; }
    PrimitiveBatch& overlay() { return overlay_lines_; }
    TextRenderer& text() { return text_; }
    void endOverlay();

    // --- composed world overlays -----------------------------------------
    // Each of these adds to the world line batch; they are grouped here
    // because they are about the simulation's geometry rather than about GL.

    void addFrameAxes(double length_m);
    // `terrain` is only used to drop a plumb line to the ground beneath the
    // aircraft, which is what makes its horizontal position readable in a
    // perspective view - without it, altitude and distance cannot be
    // separated by eye.
    void addAircraft(const UavState& state, const EnuFrame& frame, const Terrain& terrain,
                     double size_m);
    // The camera frustum, drawn out to where its corner rays meet the ground
    // (or to a fixed length for the corners that miss).
    void addCameraFrustum(const UavState& state, const CameraModel& camera,
                          const EnuFrame& frame, const Footprint& footprint,
                          double fallback_length_m);
    void addFootprint(const Footprint& footprint, const Terrain& terrain);
    // The imaged area as a filled translucent surface, under its outline.
    void addFootprintFill(const Footprint& footprint, const glm::vec4& colour);
    void addPickRay(const GeolocateResult& result, const Terrain& terrain,
                    double miss_length_m, double marker_size_m = 25.0);
    void addGroundMarker(const glm::dvec3& enu, const glm::vec4& colour, double size_m);

    // A loaded flight plan: the legs, the waypoints, and a plumb line from
    // each down to the ground. The plumb lines are the point - a waypoint's
    // altitude is the thing most often wrong in a plan, and a route drawn
    // without them tells you nothing about height.
    void addMissionRoute(const std::vector<glm::dvec3>& route_enu, int current_leg,
                         const Terrain& terrain);

private:
    std::unique_ptr<Shader> terrain_shader_;
    std::unique_ptr<Shader> line_shader_;
    std::unique_ptr<Shader> text_shader_;

    GroundChunks ground_;
    PrimitiveBatch world_lines_{PrimitiveBatch::Mode::Lines};
    PrimitiveBatch world_fills_{PrimitiveBatch::Mode::Triangles};
    PrimitiveBatch overlay_lines_{PrimitiveBatch::Mode::Lines};
    PrimitiveBatch overlay_fills_{PrimitiveBatch::Mode::Triangles};
    TextRenderer text_;

    glm::mat4 overlay_projection_{1.0f};
    // Remembered between beginWorld() and endWorld() so the line shader can
    // be given the same transform the terrain was drawn with, without the
    // caller having to pass it twice and risk passing two different ones.
    glm::mat4 pending_view_projection_{1.0f};
    glm::vec3 eye_{0.0f};
    double world_per_pixel_per_metre_ = 0.0;
    std::string error_;
};

}  // namespace georay
