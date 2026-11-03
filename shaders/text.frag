#version 330 core

in vec2 v_uv;
in vec4 v_colour;

uniform sampler2D u_atlas;

out vec4 frag_colour;

void main() {
    float coverage = texture(u_atlas, v_uv).r;
    // Drop only what is genuinely empty. The old hard 0.5 cutoff suited a
    // 1-bit 3x5 font; a rasterised face carries antialiasing in this channel,
    // and thresholding it would throw away the very thing that makes real
    // text legible at 11 px.
    if (coverage < 0.004) discard;
    frag_colour = vec4(v_colour.rgb, v_colour.a * coverage);
}
