#include "render/scene.hpp"

#include <algorithm>
#include <cmath>

namespace georay {
namespace {

glm::vec3 toFloat(const glm::dvec3& v) { return glm::vec3(float(v.x), float(v.y), float(v.z)); }

// Ground overlays are drawn on the plane, not at the elevation they were
// computed at: the ground renders flat, so anything at its true terrain
// height disappears wherever that height is below the plane - most of a
// city, since the plane sits at the frame origin's elevation. (The
// footprint fill came out as four stray wedges before this fix.) The
// elevation is not lost - it's reported as a number - and a shape half
// there would be worse.
constexpr float kGroundLift = 0.8f;
glm::vec3 onPlane(const glm::dvec3& v) {
    return glm::vec3(float(v.x), float(v.y), kGroundLift);
}

}  // namespace

Scene::Scene() = default;

Scene::~Scene() = default;

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

void Scene::beginWorld(const glm::mat4& view_projection, const glm::vec3& eye,
                       GroundMode mode, bool dark, double world_per_pixel_per_metre) {
    eye_ = eye;
    world_per_pixel_per_metre_ = world_per_pixel_per_metre;
    terrain_shader_->bind();
    terrain_shader_->set("u_view_projection", view_projection);
    terrain_shader_->set("u_camera_position", eye);
    terrain_shader_->set("u_ground_mode", int(mode));
    terrain_shader_->set("u_dark", dark ? 1 : 0);
    pending_view_projection_ = view_projection;
}

void Scene::drawGround() { ground_.draw(*terrain_shader_); }

void Scene::addThickSegment(const glm::vec3& a, const glm::vec3& b, const glm::vec4& colour,
                            float width_px) {
    const glm::vec3 along = b - a;
    const float length = glm::length(along);
    if (length < 1e-4f) return;
    const glm::vec3 direction = along / length;

    // Width and facing per endpoint, not once at the midpoint: a width taken
    // from the middle of a segment spanning real depth (aircraft 400 m up,
    // ground a km out) comes out wedge-shaped - wide near, thin far.
    auto ribbon = [&](const glm::vec3& point) {
        const glm::vec3 to_eye = eye_ - point;
        const float distance = glm::length(to_eye);
        if (distance < 1e-4f) return glm::vec3(0.0f);
        glm::vec3 side = glm::cross(direction, to_eye / distance);
        const float side_length = glm::length(side);
        if (side_length < 1e-5f) return glm::vec3(0.0f);  // seen end-on
        const float half =
            std::max(float(distance * world_per_pixel_per_metre_) * width_px * 0.5f, 1e-3f);
        return side / side_length * half;
    };

    const glm::vec3 side_a = ribbon(a);
    const glm::vec3 side_b = ribbon(b);
    if (glm::length(side_a) < 1e-6f && glm::length(side_b) < 1e-6f) return;

    world_fills_.addTriangle(a - side_a, a + side_a, b + side_b, colour);
    world_fills_.addTriangle(a - side_a, b + side_b, b - side_b, colour);
}

void Scene::addThickPolyline(const std::vector<glm::vec3>& points, const glm::vec4& colour,
                             float width_px, bool closed) {
    if (points.size() < 2) return;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        addThickSegment(points[i], points[i + 1], colour, width_px);
    }
    if (closed) addThickSegment(points.back(), points.front(), colour, width_px);
}

void Scene::endWorld() {
    line_shader_->bind();
    line_shader_->set("u_view_projection", pending_view_projection_);

    // Fills first, blended and without writing depth: they are translucent
    // surfaces that overlap one another, and depth-writing them would make
    // whichever drew first hide the rest.
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    world_fills_.flush();
    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);

    // Then the lines on top, so edges stay crisp over their own fill.
    world_lines_.flush();
    glDisable(GL_BLEND);
}

void Scene::beginOverlay(int width, int height) {
    // Pixel coordinates with the origin at the top-left and y increasing
    // downwards, which is how both SDL reports the mouse and how text reads.
    overlay_projection_ = glm::ortho(0.0f, float(width), float(height), 0.0f, -1.0f, 1.0f);

    glDisable(GL_DEPTH_TEST);
    // Culling has to go too: the overlay projection flips y (pixels run
    // down, clip space runs up), reversing every quad's winding - with
    // culling on, the HUD is silently drawn and discarded.
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void Scene::endOverlay() {
    // Panels, then lines, then text: with the depth test off, layering is
    // purely draw order.
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

void Scene::addFrameAxes(double length_m) {
    const float l = float(length_m);
    const glm::vec3 o(0.0f, 0.0f, 0.0f);
    world_lines_.addSegment(o, {l, 0, 0}, colours::axis_east);
    world_lines_.addSegment(o, {0, l, 0}, colours::axis_north);
    world_lines_.addSegment(o, {0, 0, l}, colours::axis_up);
}

void Scene::addAircraft(const UavState& state, const EnuFrame& frame, const Terrain& terrain,
                        double size_m) {
    // A recognisable plan-view airframe rather than a dot: orientation is
    // half the input to geolocation, and a 180-degree yaw error shows
    // immediately on a fuselage but not on a cross.
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

    // A plumb line to the ground and a ring where it lands, pinning the
    // aircraft's horizontal position against the grid - ambiguous in
    // perspective alone.
    const glm::dvec3 camera = cameraPositionEnu(state, frame);
    const glm::vec3 above = toFloat(camera);
    const glm::vec3 below(above.x, above.y, kGroundLift);
    (void)terrain;
    world_lines_.addSegment(above, below, glm::vec4(glm::vec3(c), 0.5f),
                            glm::vec4(glm::vec3(c), 0.12f));
    world_lines_.addCircleXY(below, float(size_m) * 0.8f, glm::vec4(glm::vec3(c), 0.4f), 24);
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
            // A corner ray with no ground still has to be drawn, or the
            // frustum loses an edge exactly when the view is partly off the
            // map. The endpoint is an arbitrary distance along the ray, not
            // a place, so it must not be drawn as one.
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
        addThickSegment(apex_f, far_corner[i], colours::frustum, 2.0f);

        // Only a real footprint edge when both ends are on the ground -
        // joining a hit to a cut-off ray would look like an edge and isn't
        // one, so it's drawn faint as the construction line it is.
        const int j = (i + 1) % 4;
        const bool real_edge = grounded[i] && grounded[j];
        if (real_edge) {
            addThickSegment(far_corner[i], far_corner[j], colours::frustum, 2.5f);
        } else {
            world_lines_.addSegment(far_corner[i], far_corner[j], faded);
        }
    }

    // Boresight: the centre pixel's ray, read instinctively as "where the
    // camera is pointing".
    const glm::dvec3 boresight = boresightEnu(state);
    addThickSegment(apex_f, toFloat(apex + boresight * fallback_length_m * 0.35),
                    colours::boresight, 1.8f);
}

void Scene::addFootprint(const Footprint& footprint, const Terrain& terrain) {
    // Draw a segment only between neighbours that both found ground.
    // Chaining all valid points into one polyline instead would connect
    // non-adjacent samples with a line across the gap that looks like an
    // edge and isn't one.
    std::vector<glm::vec3> run;
    auto emit = [&]() {
        if (run.size() >= 2) addThickPolyline(run, colours::footprint, 3.0f, false);
        run.clear();
    };

    for (const FootprintSample& sample : footprint.outline) {
        if (!sample.valid) {
            emit();
            continue;
        }
        run.push_back(onPlane(sample.enu));
    }

    // Close the ring only when every sample found ground.
    if (footprint.complete && run.size() >= 3) {
        addThickPolyline(run, colours::footprint, 3.0f, true);
        run.clear();
    }
    emit();
    (void)terrain;
}

void Scene::addPickRay(const GeolocateResult& result, const Terrain& terrain,
                       double miss_length_m, double marker_size_m) {
    const glm::vec3 origin = toFloat(result.ray_origin_enu);

    if (!result.valid) {
        // Draw the ray anyway, fading out - still the most useful thing on
        // screen for working out why it found nothing.
        const glm::vec3 end = toFloat(result.ray_origin_enu +
                                      result.ray_direction_enu * miss_length_m);
        world_lines_.addSegment(origin, end, colours::pick_ray,
                                glm::vec4(glm::vec3(colours::pick_ray), 0.0f));
        return;
    }

    addThickSegment(origin, onPlane(result.ground_enu), colours::pick_ray, 2.5f);
    addGroundMarker(result.ground_enu, colours::pick_hit, marker_size_m);

    // A vertical drop line to the aircraft's altitude, so the hit reads off
    // against the grid.
    const glm::vec3 hit = onPlane(result.ground_enu);
    world_lines_.addSegment(hit, glm::vec3(hit.x, hit.y, float(result.ray_origin_enu.z)),
                            glm::vec4(glm::vec3(colours::pick_hit), 0.55f),
                            glm::vec4(glm::vec3(colours::pick_hit), 0.0f));
    (void)terrain;
}

void Scene::addMissionRoute(const std::vector<glm::dvec3>& route_enu, int current_leg,
                            const Terrain& terrain) {
    if (route_enu.empty()) return;

    for (size_t i = 0; i + 1 < route_enu.size(); ++i) {
        // The leg being flown is highlighted, giving "waypoint 3 of 7"
        // something to point at.
        const glm::vec4 colour = (int(i) == current_leg) ? colours::route_active : colours::route;
        world_lines_.addSegment(toFloat(route_enu[i]), toFloat(route_enu[i + 1]), colour);
    }

    for (size_t i = 0; i < route_enu.size(); ++i) {
        const glm::vec3 p = toFloat(route_enu[i]);
        const double ground = terrain.sampleHeight(route_enu[i].x, route_enu[i].y);
        const glm::vec3 below(p.x, p.y, float(ground));

        // A plumb line and a ring where it lands - a dot in mid-air gives no
        // sense of height, the number most often wrong in a plan.
        world_lines_.addSegment(p, below, glm::vec4(glm::vec3(colours::waypoint), 0.55f),
                                glm::vec4(glm::vec3(colours::waypoint), 0.1f));
        world_lines_.addCircleXY(below + glm::vec3(0, 0, 0.5f), 12.0f,
                                 glm::vec4(glm::vec3(colours::waypoint), 0.45f), 20);

        // A box at the waypoint, sized to stay visible from the default
        // viewing distance.
        world_lines_.addBox(p, glm::vec3(10.0f), colours::waypoint);
    }
}

void Scene::addFootprintFill(const Footprint& footprint, const glm::vec4& colour) {
    // Only a complete outline encloses an area; filling a partial one would
    // shade a polygon that isn't the footprint.
    if (!footprint.complete || footprint.outline.size() < 3) return;

    glm::dvec3 centroid(0.0);
    int count = 0;
    for (const FootprintSample& sample : footprint.outline) {
        if (!sample.valid) return;
        centroid += sample.enu;
        ++count;
    }
    if (count < 3) return;
    centroid /= double(count);

    // A fan from the centroid - the footprint is convex enough for this at
    // any realistic angle, and a general tessellator is too much machinery
    // for a quadrilateral with rounded edges.
    for (size_t i = 0; i < footprint.outline.size(); ++i) {
        const glm::dvec3& a = footprint.outline[i].enu;
        const glm::dvec3& b = footprint.outline[(i + 1) % footprint.outline.size()].enu;
        world_fills_.addTriangle(onPlane(centroid), onPlane(a), onPlane(b), colour);
    }
}

void Scene::addGroundMarker(const glm::dvec3& enu, const glm::vec4& colour, double size_m) {
    const glm::vec3 p = onPlane(enu);
    const float s = float(size_m);
    // A ring plus a vertical stalk: the ring says where on the surface, the
    // stalk keeps it visible at a shallow angle where a flat ring collapses
    // to a line.
    world_lines_.addCircleXY(p, s, colour, 32);
    world_lines_.addSegment(p, p + glm::vec3(0, 0, s * 1.6f), colour,
                            glm::vec4(glm::vec3(colour), 0.0f));
    world_lines_.addSegment(p - glm::vec3(s * 1.4f, 0, 0), p + glm::vec3(s * 1.4f, 0, 0), colour);
    world_lines_.addSegment(p - glm::vec3(0, s * 1.4f, 0), p + glm::vec3(0, s * 1.4f, 0), colour);
}

}  // namespace georay
