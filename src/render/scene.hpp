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

    // Up to two map layers can be draped: in practice satellite imagery and
    // a street map of the same ground, switchable at runtime.
    static constexpr int kMapLayers = 2;

    // Rebuilds the terrain surface mesh. Called once at start-up and again
    // whenever the terrain changes.
    //
    // `frame` and `map` are needed together: the map's texture coordinates
    // are computed per post from that post's own latitude and longitude,
    // which only the frame can supply. Pass a null map for untextured terrain.
    // `box` fixes the geographic extent the map layers cover. Texture
    // coordinates come from it alone, so a layer can be re-stitched at any
    // zoom afterwards without the mesh being rebuilt.
    void buildTerrain(const Terrain& terrain, const EnuFrame& frame, const MercatorBox& box);

    // Up to two map layers - in practice satellite imagery and a street map
    // of the same ground, switchable at runtime. Call before buildTerrain,
    // which bakes the texture coordinates that index them.
    //
    // Returns false and sets error() when the raster cannot be uploaded: too
    // large for the driver, or out of memory. Both otherwise present as
    // untextured terrain, indistinguishable from a missing file.
    // Uploads RGB pixels as a layer's texture. The caller owns the stitching;
    // this only moves bytes to the GPU.
    bool setMapTexture(int layer, const unsigned char* rgb, int width, int height);
    void clearMapTextures();
    void clearMapLayer(int layer);
    bool hasMap(int layer) const {
        return layer >= 0 && layer < kMapLayers && map_textures_[layer] != 0;
    }

    // --- world pass -------------------------------------------------------
    // `map_layer` selects which raster to drape, or -1 for none.
    // `imagery` tells the shader whether that layer is a photograph or a
    // drawn map, which changes how much shading and contouring it gets.
    void beginWorld(const glm::mat4& view_projection, const glm::vec3& eye_position,
                    const Terrain& terrain, float contour_interval_m, int map_layer,
                    bool imagery);
    void drawTerrain();
    PrimitiveBatch& world() { return world_lines_; }
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

    void addGroundGrid(const Terrain& terrain, double spacing_m);
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
    void addPickRay(const GeolocateResult& result, const Terrain& terrain,
                    double miss_length_m);
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

    SurfaceMesh terrain_mesh_;
    GLuint map_textures_[kMapLayers] = {0, 0};
    PrimitiveBatch world_lines_{PrimitiveBatch::Mode::Lines};
    PrimitiveBatch overlay_lines_{PrimitiveBatch::Mode::Lines};
    PrimitiveBatch overlay_fills_{PrimitiveBatch::Mode::Triangles};
    TextRenderer text_;

    glm::mat4 overlay_projection_{1.0f};
    // Remembered between beginWorld() and endWorld() so the line shader can
    // be given the same transform the terrain was drawn with, without the
    // caller having to pass it twice and risk passing two different ones.
    glm::mat4 pending_view_projection_{1.0f};
    std::string error_;
};

}  // namespace georay
