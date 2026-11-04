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
uniform int u_map_is_imagery;      // 1 for satellite imagery, 0 for a drawn map
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
    if (mapped && u_map_is_imagery == 0) {
        // A drawn map is already a flat, deliberate picture. Shading it like
        // a surface fights its own design and makes labels hard to read, so
        // it gets only a hint of relief - just enough that a hillside is not
        // completely flat.
        colour = base * (0.94 + 0.10 * diffuse);
    } else if (mapped) {
        // Imagery carries its own baked-in lighting, so this is a light touch
        // too, but enough that terrain relief still reads.
        colour = base * (0.82 + 0.25 * diffuse);
    } else {
        colour = base * (0.35 + 0.75 * diffuse);
    }

    // Contour lines, derived from the fragment's own height gradient so they
    // stay roughly a pixel wide at any zoom instead of aliasing into moire
    // when the terrain is far away.
    // Contours over a drawn map double up with its own contour and boundary
    // lines, which is exactly the noise that mode exists to avoid.
    bool want_contours = u_contour_interval > 0.0 && !(mapped && u_map_is_imagery == 0);
    if (want_contours) {
        float h = v_world.z / u_contour_interval;
        float width = max(fwidth(h), 1e-5);
        float line = abs(fract(h) - 0.5) / width;
        float strength = mapped ? 0.85 : 0.65;
        colour = mix(colour * strength, colour, clamp(line - 0.5, 0.0, 1.0));
    }

    // Distance haze, so the horizon does not end in a hard edge and the eye
    // gets a depth cue in a scene that is otherwise untextured.
    //
    // Barely any of it on a drawn map. Atmospheric haze over flat cartography
    // is a category error - there is no air between the viewer and a map -
    // and at regional scale, where everything is tens of kilometres away, it
    // saturates and bleaches the whole sheet to pale grey.
    float distance_m = length(v_world - u_camera_position);
    float haze_limit = mapped ? (u_map_is_imagery == 1 ? 0.35 : 0.10) : 0.65;
    float haze = clamp(distance_m / 12000.0, 0.0, haze_limit);
    colour = mix(colour, vec3(0.52, 0.60, 0.70), haze);

    frag_colour = vec4(colour, 1.0);
}
