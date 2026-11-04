#include "geo/geolocate.hpp"

#include <cmath>

namespace georay {

GeolocateResult geolocatePixel(const UavState& state, const CameraModel& camera,
                               const Terrain& terrain, const EnuFrame& frame,
                               const glm::dvec2& pixel) {
    GeolocateResult out;

    // 1-3: build the world-frame ray.
    const glm::dvec3 ray_optical = camera.pixelToRay(pixel);
    out.ray_direction_enu = glm::normalize(rotationEnuFromOptical(state) * ray_optical);
    out.ray_origin_enu = cameraPositionEnu(state, frame);

    // 4: intersect it with the ground.
    const RayHit hit = terrain.raycast(out.ray_origin_enu, out.ray_direction_enu);
    out.march_steps = hit.steps;

    if (!hit.hit) {
        // Distinguish "above the horizon" (not fixed by more terrain) from
        // "ran out of map" (is). An upward ray also technically leaves the
        // modelled square, but reporting that would imply more terrain
        // might help - it would not.
        out.pointed_at_sky = out.ray_direction_enu.z >= 0.0;
        out.left_coverage = hit.left_coverage && !out.pointed_at_sky;
        return out;
    }
    out.left_coverage = hit.left_coverage;

    // 5: back to latitude and longitude.
    out.valid = true;
    out.ground_enu = hit.point;
    out.ground_geodetic = frame.toGeodetic(hit.point);
    out.ground_elevation_m = hit.point.z;
    out.slant_range_m = hit.distance;

    const glm::dvec3 offset = hit.point - out.ray_origin_enu;
    out.ground_range_m = std::hypot(offset.x, offset.y);
    out.depression_angle_deg =
        rad2deg(std::atan2(-out.ray_direction_enu.z,
                           std::hypot(out.ray_direction_enu.x, out.ray_direction_enu.y)));

    // GSD: trace a neighbouring pixel's ray against the tangent plane at the
    // hit rather than dividing range by focal length - costs nothing extra
    // and, unlike that textbook formula, stays right for an oblique view.
    {
        const glm::dvec3 neighbour_optical = camera.pixelToRay(pixel + glm::dvec2(1.0, 0.0));
        const glm::dvec3 neighbour_enu =
            glm::normalize(rotationEnuFromOptical(state) * neighbour_optical);
        const double denom = glm::dot(neighbour_enu, hit.normal);
        if (std::fabs(denom) > 1e-9) {
            const double t = glm::dot(hit.point - out.ray_origin_enu, hit.normal) / denom;
            if (t > 0.0) {
                const glm::dvec3 neighbour_point = out.ray_origin_enu + neighbour_enu * t;
                out.gsd_m = glm::length(neighbour_point - hit.point);
            }
        }
        // denom ~ 0: a grazing view where GSD is genuinely unbounded, and
        // left at zero reads as "not meaningful", which it is.
    }

    return out;
}

ProjectResult projectEnu(const UavState& state, const CameraModel& camera,
                         const EnuFrame& frame, const glm::dvec3& point_enu) {
    ProjectResult out;

    const glm::dvec3 origin = cameraPositionEnu(state, frame);
    const glm::dvec3 offset = point_enu - origin;
    out.range_m = glm::length(offset);

    // World -> optical is the inverse of the chain geolocatePixel uses; it is
    // orthonormal, so transpose.
    const glm::dvec3 point_optical = glm::transpose(rotationEnuFromOptical(state)) * offset;

    out.pixel = camera.projectToPixel(point_optical, &out.in_front);
    out.in_image = out.in_front && camera.pixelInImage(out.pixel);
    return out;
}

ProjectResult projectGeodetic(const UavState& state, const CameraModel& camera,
                              const EnuFrame& frame, const Geodetic& target) {
    return projectEnu(state, camera, frame, frame.fromGeodetic(target));
}

glm::mat4 glViewMatrix(const UavState& state, const EnuFrame& frame) {
    // World (ENU) -> optical, then optical -> GL eye space. The second step
    // is the diag(1, -1, -1) flip: the optical frame has +y down and +z
    // forward, GL eye space has +y up and -z forward.
    const glm::dmat3 optical_from_enu = glm::transpose(rotationEnuFromOptical(state));
    glm::dmat3 flip(1.0);
    flip[1][1] = -1.0;
    flip[2][2] = -1.0;
    const glm::dmat3 eye_from_enu = flip * optical_from_enu;

    const glm::dvec3 origin = cameraPositionEnu(state, frame);
    const glm::dvec3 translation = -(eye_from_enu * origin);

    glm::mat4 view(1.0f);
    for (int col = 0; col < 3; ++col) {
        for (int row = 0; row < 3; ++row) {
            view[col][row] = float(eye_from_enu[col][row]);
        }
    }
    view[3] = glm::vec4(float(translation.x), float(translation.y), float(translation.z), 1.0f);
    return view;
}

Footprint computeFootprint(const UavState& state, const CameraModel& camera,
                           const Terrain& terrain, const EnuFrame& frame,
                           int samples_per_edge) {
    Footprint out;
    const int n = samples_per_edge > 1 ? samples_per_edge : 1;

    const std::array<glm::dvec2, 4> corners = camera.imageCorners();
    for (int i = 0; i < 4; ++i) {
        out.corners[i] = geolocatePixel(state, camera, terrain, frame, corners[i]);
    }

    out.complete = true;
    for (int edge = 0; edge < 4; ++edge) {
        const glm::dvec2 a = corners[edge];
        const glm::dvec2 b = corners[(edge + 1) % 4];
        // Each edge contributes its start plus the interior samples; the next
        // edge's start supplies the shared corner, so no point is duplicated.
        for (int s = 0; s < n; ++s) {
            const double t = double(s) / double(n);
            const GeolocateResult r =
                geolocatePixel(state, camera, terrain, frame, a + (b - a) * t);
            out.outline.push_back({r.valid, r.ground_enu});
            if (!r.valid) out.complete = false;
        }
    }

    // Shoelace area on the horizontal projection. Only reported for a closed
    // footprint: with points missing, the polygon is not the footprint and
    // any number computed from it would be a plausible-looking fabrication.
    if (out.complete && out.outline.size() >= 3) {
        double twice_area = 0.0;
        for (size_t i = 0; i < out.outline.size(); ++i) {
            const glm::dvec3& p = out.outline[i].enu;
            const glm::dvec3& q = out.outline[(i + 1) % out.outline.size()].enu;
            twice_area += p.x * q.y - q.x * p.y;
        }
        out.area_m2 = std::fabs(twice_area) * 0.5;
    }

    return out;
}

}  // namespace georay
