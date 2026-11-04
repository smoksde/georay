#include "render/scene.hpp"

#include <algorithm>
#include <cmath>

namespace georay {
namespace {

glm::vec3 toFloat(const glm::dvec3& v) { return glm::vec3(float(v.x), float(v.y), float(v.z)); }

}  // namespace

Scene::Scene() = default;

Scene::~Scene() { clearMapTextures(); }

bool Scene::initialise() {
    terrain_shader_ = std::make_unique<Shader>(shaderPath("terrain.vert"),
                                               shaderPath("terrain.frag"));
    line_shader_ = std::make_unique<Shader>(shaderPath("line.vert"), shaderPath("line.frag"));
    text_shader_ = std::make_unique<Shader>(shaderPath("text.vert"), shaderPath("text.frag"));

    for (const Shader* s : {terrain_shader_.get(), line_shader_.get(), text_shader_.get()}) {
        if (!s->ok()) {
            error_ = s->error();
            return false;
        }
    }

    text_.initialise(assetPath("fonts/georay-font.bin"));
    return true;
}

bool Scene::setMapTexture(int layer, const unsigned char* rgb, int width, int height) {
    if (layer < 0 || layer >= kMapLayers || !rgb || width < 1 || height < 1) return false;

    // A texture larger than the driver's limit does not fail loudly - the
    // upload is simply dropped and the terrain renders untextured, which
    // looks like the map was never found. Check first and say so.
    GLint max_size = 0;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_size);
    if (width > max_size || height > max_size) {
        error_ = "map texture is " + std::to_string(width) + "x" + std::to_string(height) +
                 " but this GPU allows at most " + std::to_string(max_size);
        return false;
    }

    if (map_textures_[layer] == 0) glGenTextures(1, &map_textures_[layer]);
    glBindTexture(GL_TEXTURE_2D, map_textures_[layer]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb);

    if (glGetError() == GL_OUT_OF_MEMORY) {
        glDeleteTextures(1, &map_textures_[layer]);
        map_textures_[layer] = 0;
        error_ = "the GPU could not hold a " + std::to_string(width) + "x" +
                 std::to_string(height) + " map texture";
        return false;
    }

    // Mipmaps matter here, not as polish: the texture is thousands of pixels
    // across and the 3D view shows all of it at once, so without them the
    // minified map is a shimmering mess of aliased lines.
    glGenerateMipmap(GL_TEXTURE_2D);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // Clamp rather than repeat: a tiled map would put a second city next to
    // the first, which is worse than an honest edge.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Anisotropic filtering if the driver has it. The terrain is usually seen
    // at a shallow angle, which is exactly the case trilinear filtering blurs
    // into uselessness - and a blurred map cannot be read.
    if (GLEW_EXT_texture_filter_anisotropic) {
        GLfloat max_anisotropy = 1.0f;
        glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_anisotropy);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT,
                        std::min(max_anisotropy, 8.0f));
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

void Scene::clearMapLayer(int layer) {
    if (layer < 0 || layer >= kMapLayers) return;
    if (map_textures_[layer] != 0) {
        glDeleteTextures(1, &map_textures_[layer]);
        map_textures_[layer] = 0;
    }
}

void Scene::clearMapTextures() {
    for (int layer = 0; layer < kMapLayers; ++layer) {
        if (map_textures_[layer] != 0) {
            glDeleteTextures(1, &map_textures_[layer]);
            map_textures_[layer] = 0;
        }
    }
}

void Scene::buildTerrain(const Terrain& terrain, const EnuFrame& frame,
                         const MercatorBox& box) {
    const int n = terrain.resolution();

    std::vector<SurfaceVertex> vertices;
    vertices.reserve(size_t(n) * size_t(n));
    for (int iy = 0; iy < n; ++iy) {
        for (int ix = 0; ix < n; ++ix) {
            const glm::dvec2 p = terrain.gridPosition(ix, iy);
            const double h = terrain.heightAt(ix, iy);
            // The analytic normal from sampleNormal() rather than a
            // face-averaged one, so the shaded surface and the surface the
            // raycaster intersects are the same surface. A smoothed normal
            // would make the render disagree with the maths exactly where
            // the relief is steepest - which is where it matters.
            const glm::dvec3 nrm = terrain.sampleNormal(p.x, p.y);

            SurfaceVertex vertex{glm::vec3(float(p.x), float(p.y), float(h)), toFloat(nrm), {}};
            if (box.valid()) {
                // The post's own position on the ellipsoid, through the full
                // Web Mercator transform. Doing it here, once per post, is
                // what keeps the map registered to the ground to within a
                // pixel across the whole scene.
                const Geodetic here = frame.toGeodetic(glm::dvec3(p.x, p.y, h));
                const glm::dvec2 uv = box.uvFor(here.latitude_deg, here.longitude_deg);
                vertex.map_uv = glm::vec2(float(uv.x), float(uv.y));
            }
            vertices.push_back(vertex);
        }
    }

    std::vector<unsigned int> indices;
    indices.reserve(size_t(n - 1) * size_t(n - 1) * 6);
    for (int iy = 0; iy + 1 < n; ++iy) {
        for (int ix = 0; ix + 1 < n; ++ix) {
            const unsigned int a = unsigned(iy * n + ix);
            const unsigned int b = a + 1;
            const unsigned int c = a + unsigned(n);
            const unsigned int d = c + 1;
            // Counter-clockwise seen from above (+z), matching the +z normals
            // above, so back-face culling keeps the surface rather than the
            // underside.
            indices.insert(indices.end(), {a, b, d, a, d, c});
        }
    }

    terrain_mesh_.upload(vertices, indices);
}

void Scene::beginWorld(const glm::mat4& view_projection, const glm::vec3& eye,
                       const Terrain& terrain, float contour_interval_m, int map_layer,
                       bool imagery) {
    terrain_shader_->bind();
    terrain_shader_->set("u_view_projection", view_projection);
    terrain_shader_->set("u_camera_position", eye);
    // A sun from the north-west and fairly high: low enough for relief to
    // cast obvious shading, high enough that nothing important is in shadow.
    terrain_shader_->set("u_sun_direction",
                         glm::normalize(glm::vec3(-0.45f, 0.35f, 0.82f)));
    terrain_shader_->set("u_height_min", float(terrain.minHeight()));
    terrain_shader_->set("u_height_max", float(terrain.maxHeight() + 1.0));
    terrain_shader_->set("u_contour_interval", contour_interval_m);

    const bool mapped = hasMap(map_layer);
    terrain_shader_->set("u_has_map", mapped ? 1 : 0);
    terrain_shader_->set("u_map_is_imagery", imagery ? 1 : 0);
    if (mapped) {
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, map_textures_[map_layer]);
        terrain_shader_->set("u_map", 0);
    }

    pending_view_projection_ = view_projection;
}

void Scene::drawTerrain() { terrain_mesh_.draw(); }

void Scene::endWorld() {
    line_shader_->bind();
    line_shader_->set("u_view_projection", pending_view_projection_);
    world_lines_.flush();
}

void Scene::beginOverlay(int width, int height) {
    // Pixel coordinates with the origin at the top-left and y increasing
    // downwards, which is how both SDL reports the mouse and how text reads.
    overlay_projection_ = glm::ortho(0.0f, float(width), float(height), 0.0f, -1.0f, 1.0f);

    glDisable(GL_DEPTH_TEST);
    // Back-face culling has to go too, not just the depth test. The overlay
    // projection flips y (pixel coordinates run downwards, clip space runs
    // up), which reverses the winding of every quad - so with culling on,
    // panels and text are drawn and then thrown away, and the HUD is simply
    // absent with no error anywhere.
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Scene::endOverlay() {
    // Panels first, then lines, then text: each layer must sit on top of the
    // previous one, and with the depth test off that is purely draw order.
    line_shader_->bind();
    line_shader_->set("u_view_projection", overlay_projection_);
    overlay_fills_.flush();
    overlay_lines_.flush();

    text_shader_->bind();
    text_shader_->set("u_projection", overlay_projection_);
    text_shader_->set("u_atlas", 0);
    text_.flush();

    glDisable(GL_BLEND);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void Scene::addGroundGrid(const Terrain& terrain, double spacing_m) {
    if (spacing_m <= 0.0) return;
    const double extent = terrain.halfExtent();
    const int steps = int(extent / spacing_m);

    // Draped over the relief, not drawn on a flat plane at some reference
    // height: a flat grid spends most of its length buried inside hills,
    // which is both useless and misleading about where the ground is. Each
    // line is therefore subdivided and each sample lifted just clear of the
    // surface.
    const double sub = 50.0;  // metres between samples along a grid line
    const int sub_steps = int((2.0 * extent) / sub);
    const float lift = 1.5f;

    auto drape = [&](bool along_north) {
        for (int i = -steps; i <= steps; ++i) {
            const double fixed = i * spacing_m;
            // Every fifth line brighter, so distances are countable at a glance.
            const glm::vec4 c = (i % 5 == 0) ? colours::grid_major : colours::grid;
            glm::vec3 previous;
            for (int k = 0; k <= sub_steps; ++k) {
                const double moving = -extent + k * sub;
                const double e = along_north ? fixed : moving;
                const double n = along_north ? moving : fixed;
                const glm::vec3 p(float(e), float(n), float(terrain.sampleHeight(e, n)) + lift);
                if (k > 0) world_lines_.addSegment(previous, p, c);
                previous = p;
            }
        }
    };
    drape(true);
    drape(false);
}

void Scene::addFrameAxes(double length_m) {
    const float l = float(length_m);
    const glm::vec3 o(0.0f, 0.0f, 0.0f);
    world_lines_.addSegment(o, {l, 0, 0}, colours::axis_east);
    world_lines_.addSegment(o, {0, l, 0}, colours::axis_north);
    world_lines_.addSegment(o, {0, 0, l}, colours::axis_up);
}

void Scene::addAircraft(const UavState& state, const EnuFrame& frame, const Terrain& terrain,
                        double size_m) {
    // A recognisable plan-view airframe rather than a dot, because its
    // orientation is half the input to the geolocation: if the model is
    // yawed 180 degrees from what you intended, a cross tells you nothing
    // and a fuselage tells you immediately.
    const glm::dmat3 enu_from_body = rotationEnuFromNed() * rotationNedFromBody(state.attitude);
    const glm::dvec3 centre = frame.fromGeodetic(state.position);

    auto body = [&](double x, double y, double z) {
        return toFloat(centre + enu_from_body * (glm::dvec3(x, y, z) * size_m));
    };

    const glm::vec4 c = colours::aircraft;
    world_lines_.addSegment(body(1.2, 0, 0), body(-1.0, 0, 0), c);   // fuselage
    world_lines_.addSegment(body(0.1, -1.0, 0), body(0.1, 1.0, 0), c);  // main wing
    world_lines_.addSegment(body(-0.9, -0.4, 0), body(-0.9, 0.4, 0), c);  // tailplane
    world_lines_.addSegment(body(-0.9, 0, 0), body(-0.9, 0, -0.45), c);   // fin (up is -z)
    // Wingtips dropped a little, so roll is visible from directly above.
    world_lines_.addSegment(body(0.1, -1.0, 0), body(0.0, -1.0, 0.1), c);
    world_lines_.addSegment(body(0.1, 1.0, 0), body(0.0, 1.0, 0.1), c);

    // A plumb line down to the ground directly below, and a ring where it
    // lands: together they pin the aircraft's horizontal position against the
    // grid, which perspective alone leaves ambiguous.
    const glm::dvec3 camera = cameraPositionEnu(state, frame);
    const double ground_below = terrain.sampleHeight(camera.x, camera.y);
    const glm::vec3 above = toFloat(camera);
    const glm::vec3 below(above.x, above.y, float(ground_below));
    world_lines_.addSegment(above, below, glm::vec4(glm::vec3(c), 0.5f),
                            glm::vec4(glm::vec3(c), 0.12f));
    world_lines_.addCircleXY(below + glm::vec3(0, 0, 0.5f), float(size_m) * 0.8f,
                             glm::vec4(glm::vec3(c), 0.4f), 24);
}

void Scene::addCameraFrustum(const UavState& state, const CameraModel& camera,
                             const EnuFrame& frame, const Footprint& footprint,
                             double fallback_length_m) {
    const glm::dvec3 apex = cameraPositionEnu(state, frame);
    const glm::dmat3 enu_from_optical = rotationEnuFromOptical(state);
    const std::array<glm::dvec2, 4> corners = camera.imageCorners();

    glm::vec3 far_corner[4];
    bool grounded[4];
    for (int i = 0; i < 4; ++i) {
        grounded[i] = footprint.corners[i].valid;
        if (grounded[i]) {
            far_corner[i] = toFloat(footprint.corners[i].ground_enu);
        } else {
            // A corner ray that found no ground still has to be drawn, or the
            // frustum silently loses an edge exactly when the view is partly
            // off the map - which is when you most need to see it. The end
            // point is an arbitrary distance along the ray, though, not a
            // place, so it must not be drawn as one.
            const glm::dvec3 dir =
                glm::normalize(enu_from_optical * camera.pixelToRay(corners[i]));
            far_corner[i] = toFloat(apex + dir * fallback_length_m);
        }
    }

    const glm::vec3 apex_f = toFloat(apex);
    const glm::vec4 faded(glm::vec3(colours::frustum), 0.18f);
    for (int i = 0; i < 4; ++i) {
        // Fading the side edges outward keeps the apex readable where all
        // four converge on the aircraft.
        world_lines_.addSegment(apex_f, far_corner[i], colours::frustum,
                                grounded[i] ? glm::vec4(glm::vec3(colours::frustum), 0.25f)
                                            : glm::vec4(glm::vec3(colours::frustum), 0.0f));

        // The far edge is only a real footprint edge when both of its ends
        // are on the ground. Joining a ground hit to a cut-off ray draws a
        // long line across the scene that looks exactly like an edge of the
        // imaged area and is nothing of the kind - so it is drawn faint, as
        // the construction line it is.
        const int j = (i + 1) % 4;
        const bool real_edge = grounded[i] && grounded[j];
        world_lines_.addSegment(far_corner[i], far_corner[j],
                                real_edge ? colours::frustum : faded);
    }

    // Boresight: the centre pixel's ray, which is the one a user instinctively
    // reads as "where the camera is pointing".
    const glm::dvec3 boresight = boresightEnu(state);
    world_lines_.addSegment(apex_f, toFloat(apex + boresight * fallback_length_m),
                            colours::boresight,
                            glm::vec4(glm::vec3(colours::boresight), 0.0f));
}

void Scene::addFootprint(const Footprint& footprint, const Terrain& terrain) {
    // Walk the samples in order and draw a segment only between neighbours
    // that both found ground. Chaining the valid points into one polyline
    // instead would connect samples that are not adjacent - drawing long
    // straight lines across the gap, which look exactly like a footprint
    // edge and are not one.
    std::vector<glm::vec3> run;
    auto emit = [&]() {
        if (run.size() >= 2) world_lines_.addPolyline(run, colours::footprint, false);
        run.clear();
    };

    for (const FootprintSample& sample : footprint.outline) {
        if (!sample.valid) {
            emit();
            continue;
        }
        // Lifted a hair off the surface so it is not swallowed by z-fighting
        // with the terrain it is lying on.
        run.push_back(glm::vec3(float(sample.enu.x), float(sample.enu.y),
                                float(sample.enu.z) + 0.6f));
    }

    // Only close the ring when every sample found ground; otherwise the
    // closing segment would be another line that is not a footprint edge.
    if (footprint.complete && run.size() >= 3) {
        world_lines_.addPolyline(run, colours::footprint, true);
        run.clear();
    }
    emit();
    (void)terrain;
}

void Scene::addPickRay(const GeolocateResult& result, const Terrain& terrain,
                       double miss_length_m) {
    const glm::vec3 origin = toFloat(result.ray_origin_enu);

    if (!result.valid) {
        // Draw the ray anyway, fading out: a ray that found nothing is still
        // the most useful thing on screen for working out why.
        const glm::vec3 end = toFloat(result.ray_origin_enu +
                                      result.ray_direction_enu * miss_length_m);
        world_lines_.addSegment(origin, end, colours::pick_ray,
                                glm::vec4(glm::vec3(colours::pick_ray), 0.0f));
        return;
    }

    world_lines_.addSegment(origin, toFloat(result.ground_enu), colours::pick_ray);
    addGroundMarker(result.ground_enu, colours::pick_hit, 25.0);

    // A vertical drop line from the hit up to the aircraft's altitude, so the
    // hit's position on the ground can be read off against the grid.
    const glm::vec3 hit = toFloat(result.ground_enu);
    world_lines_.addSegment(hit, glm::vec3(hit.x, hit.y, float(result.ray_origin_enu.z)),
                            glm::vec4(glm::vec3(colours::pick_hit), 0.55f),
                            glm::vec4(glm::vec3(colours::pick_hit), 0.0f));
    (void)terrain;
}

void Scene::addMissionRoute(const std::vector<glm::dvec3>& route_enu, int current_leg,
                            const Terrain& terrain) {
    if (route_enu.empty()) return;

    for (size_t i = 0; i + 1 < route_enu.size(); ++i) {
        // The leg being flown is highlighted, so the HUD's "waypoint 3 of 7"
        // has something to point at.
        const glm::vec4 colour = (int(i) == current_leg) ? colours::route_active : colours::route;
        world_lines_.addSegment(toFloat(route_enu[i]), toFloat(route_enu[i + 1]), colour);
    }

    for (size_t i = 0; i < route_enu.size(); ++i) {
        const glm::vec3 p = toFloat(route_enu[i]);
        const double ground = terrain.sampleHeight(route_enu[i].x, route_enu[i].y);
        const glm::vec3 below(p.x, p.y, float(ground));

        // A plumb line down to the ground, and a ring where it lands: a
        // waypoint drawn as a dot in mid-air gives no sense of its height,
        // which is the number most often wrong in a plan.
        world_lines_.addSegment(p, below, glm::vec4(glm::vec3(colours::waypoint), 0.55f),
                                glm::vec4(glm::vec3(colours::waypoint), 0.1f));
        world_lines_.addCircleXY(below + glm::vec3(0, 0, 0.5f), 12.0f,
                                 glm::vec4(glm::vec3(colours::waypoint), 0.45f), 20);

        // A box at the waypoint itself, sized so it stays visible from the
        // default viewing distance.
        world_lines_.addBox(p, glm::vec3(10.0f), colours::waypoint);
    }
}

void Scene::addGroundMarker(const glm::dvec3& enu, const glm::vec4& colour, double size_m) {
    const glm::vec3 p = toFloat(enu);
    const float s = float(size_m);
    // A ring on the ground plus a vertical stalk: the ring says where on the
    // surface, the stalk makes it visible from a shallow angle where a flat
    // ring collapses to a line.
    world_lines_.addCircleXY(p + glm::vec3(0, 0, 0.5f), s, colour, 32);
    world_lines_.addSegment(p, p + glm::vec3(0, 0, s * 1.6f), colour,
                            glm::vec4(glm::vec3(colour), 0.0f));
    world_lines_.addSegment(p - glm::vec3(s * 1.4f, 0, 0), p + glm::vec3(s * 1.4f, 0, 0), colour);
    world_lines_.addSegment(p - glm::vec3(0, s * 1.4f, 0), p + glm::vec3(0, s * 1.4f, 0), colour);
}

}  // namespace georay
