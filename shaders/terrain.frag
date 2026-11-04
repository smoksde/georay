#version 330 core

// The ground: a flat chunk with one texture on it, whatever the mode.
//
// All three skins - street map, imagery, height map - are produced as
// textures before they reach the GPU, so the only work here is tone.

in vec3 v_world;
in vec3 v_normal;
in vec2 v_map_uv;

uniform sampler2D u_map;
uniform vec3 u_camera_position;
uniform int u_ground_mode;   // 0 map, 1 satellite, 2 height map
uniform int u_dark;          // 1 to darken the ground for overlay contrast

out vec4 frag_colour;

void main() {
    vec3 colour = texture(u_map, v_map_uv).rgb;

    // Dark mode, and the reason for it is legibility rather than taste: the
    // aircraft, its frustum and its footprint are thin bright overlays, and
    // against a paper-white map zoomed out they disappear. Darkening the
    // ground is what lets them stay readable without making them garish.
    //
    // No keyless dark basemap exists to fetch instead - the obvious ones now
    // want an API key - so the tone is applied here. That turns out better
    // than a dark tile set would be: it works on whatever is already cached,
    // for any source, and costs nothing to switch.
    if (u_dark == 1) {
        float luma = dot(colour, vec3(0.299, 0.587, 0.114));
        if (u_ground_mode == 0) {
            // A drawn map inverts rather than dims. Paper white becomes near
            // black and black ink becomes light grey, which keeps every
            // feature's contrast instead of crushing the whole sheet. The
            // original chroma is added back at reduced strength so coloured
            // roads stay distinguishable.
            vec3 chroma = colour - vec3(luma);
            colour = mix(vec3(0.055, 0.065, 0.085), vec3(0.60, 0.65, 0.72), 1.0 - luma) +
                     chroma * 0.6;
        } else if (u_ground_mode == 1) {
            // Imagery cannot be inverted and stay recognisable, so it is
            // dimmed and slightly desaturated instead.
            colour = mix(vec3(luma), colour, 0.85) * 0.5;
        } else {
            // The height ramp's colours carry meaning, so only the level
            // changes.
            colour *= 0.55;
        }
    }

    // Distance haze, and almost none of it on a drawn map or a height map:
    // there is no air between a viewer and a map, and at regional scale a
    // saturating haze bleaches the whole sheet exactly where legibility
    // matters most. Imagery keeps more, because a photograph of ground
    // genuinely does recede into atmosphere.
    float distance_m = length(v_world - u_camera_position);
    float limit = (u_ground_mode == 1) ? 0.30 : 0.07;
    float haze = clamp(distance_m / 20000.0, 0.0, limit);
    colour = mix(colour, vec3(0.52, 0.60, 0.70) * (u_dark == 1 ? 0.35 : 1.0), haze);

    frag_colour = vec4(colour, 1.0);
}
