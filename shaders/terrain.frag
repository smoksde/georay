#version 330 core

// The ground: a flat chunk with one texture on it, whatever the mode.
//
// No elevation tinting, slope shading or contour generation here any more.
// All three skins - street map, imagery, height map - are produced as
// textures before they reach the GPU, which is what lets them share one
// path and one shader.

in vec3 v_world;
in vec3 v_normal;
in vec2 v_map_uv;

uniform sampler2D u_map;
uniform vec3 u_camera_position;
uniform int u_map_is_imagery;   // 1 for a photograph, 0 for a drawn map

out vec4 frag_colour;

void main() {
    vec3 colour = texture(u_map, v_map_uv).rgb;

    // Distance haze, and almost none of it on a drawn map or a height map:
    // there is no air between a viewer and a map, and at regional scale a
    // saturating haze bleaches the whole sheet to pale grey exactly where
    // legibility matters most. Imagery keeps more of it, because a
    // photograph of ground genuinely does recede into atmosphere.
    float distance_m = length(v_world - u_camera_position);
    float limit = (u_map_is_imagery == 1) ? 0.30 : 0.07;
    float haze = clamp(distance_m / 20000.0, 0.0, limit);
    colour = mix(colour, vec3(0.52, 0.60, 0.70), haze);

    frag_colour = vec4(colour, 1.0);
}
