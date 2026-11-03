#version 330 core

in vec2 v_uv;
in vec4 v_colour;

uniform sampler2D u_atlas;

out vec4 frag_colour;

void main() {
    float coverage = texture(u_atlas, v_uv).r;
    // Drop fully transparent fragments rather than blending them: the HUD is
    // drawn with the depth test off and alpha blending on, and a zero-alpha
    // fragment still costs a blend.
    if (coverage < 0.5) discard;
    frag_colour = vec4(v_colour.rgb, v_colour.a * coverage);
}
