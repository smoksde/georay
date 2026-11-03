#version 330 core

in vec3 v_world;
in vec3 v_normal;
in vec2 v_map_uv;

uniform vec3 u_sun_direction;     // unit vector from the surface towards the sun
uniform float u_height_min;
uniform float u_height_max;
uniform float u_contour_interval;  // metres between contour lines; <= 0 disables them
uniform vec3 u_camera_position;
uniform int u_has_map;             // 1 when a map raster is draped over the terrain
uniform sampler2D u_map;

out vec4 frag_colour;

// Elevation ramp, used when there is no map. Deliberately a map-like
// hypsometric tint rather than a photographic one: the question this tool
// answers is "which point on the ground is this", and distinguishable
// elevation bands make a hit's height readable straight off the surface.
vec3 elevationTint(float t) {
    vec3 low  = vec3(0.21, 0.33, 0.20);
    vec3 mid  = vec3(0.45, 0.47, 0.26);
    vec3 high = vec3(0.55, 0.42, 0.28);
    vec3 peak = vec3(0.82, 0.82, 0.84);
    if (t < 0.4)       return mix(low,  mid,  t / 0.4);
    else if (t < 0.75) return mix(mid,  high, (t - 0.4) / 0.35);
    else               return mix(high, peak, (t - 0.75) / 0.25);
}

void main() {
    vec3 normal = normalize(v_normal);
    float span = max(u_height_max - u_height_min, 1.0);
    float t = clamp((v_world.z - u_height_min) / span, 0.0, 1.0);

    vec3 base;
    bool mapped = false;
    if (u_has_map == 1) {
        // Outside the raster the map has nothing to say, so fall back to the
        // elevation tint rather than smearing the edge pixels outwards -
        // which would look like map data where there is none.
        if (v_map_uv.x >= 0.0 && v_map_uv.x <= 1.0 && v_map_uv.y >= 0.0 && v_map_uv.y <= 1.0) {
            base = texture(u_map, v_map_uv).rgb;
            mapped = true;
        }
    }
    if (!mapped) {
        base = elevationTint(t);
        // Slope shading, which makes relief legible from a near-overhead view
        // where the diffuse term alone is nearly flat. Not applied to the map,
        // whose own colours carry the information.
        float slope = 1.0 - normal.z;
        base *= 1.0 - 0.45 * clamp(slope * 2.0, 0.0, 1.0);
    }

    float diffuse = max(dot(normal, u_sun_direction), 0.0);
    vec3 colour;
    if (mapped) {
        // A light touch on the map: enough shading for relief to read, not so
        // much that a street name goes dark. A map that cannot be read is
        // worse than no map, since the whole point is recognising the place a
        // geolocated pixel landed on.
        colour = base * (0.82 + 0.25 * diffuse);
    } else {
        colour = base * (0.35 + 0.75 * diffuse);
    }

    // Contour lines, derived from the fragment's own height gradient so they
    // stay roughly a pixel wide at any zoom instead of aliasing into moire
    // when the terrain is far away.
    if (u_contour_interval > 0.0) {
        float h = v_world.z / u_contour_interval;
        float width = max(fwidth(h), 1e-5);
        float line = abs(fract(h) - 0.5) / width;
        float strength = mapped ? 0.85 : 0.65;
        colour = mix(colour * strength, colour, clamp(line - 0.5, 0.0, 1.0));
    }

    // Distance haze, so the horizon does not end in a hard edge and the eye
    // gets a depth cue in a scene that is otherwise untextured.
    float distance_m = length(v_world - u_camera_position);
    float haze = clamp(distance_m / 12000.0, 0.0, mapped ? 0.35 : 0.65);
    colour = mix(colour, vec3(0.52, 0.60, 0.70), haze);

    frag_colour = vec4(colour, 1.0);
}
