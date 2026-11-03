#version 330 core

layout(location = 0) in vec2 a_position;  // window pixels, y down from the top
layout(location = 1) in vec2 a_uv;
layout(location = 2) in vec4 a_colour;

uniform mat4 u_projection;

out vec2 v_uv;
out vec4 v_colour;

void main() {
    v_uv = a_uv;
    v_colour = a_colour;
    gl_Position = u_projection * vec4(a_position, 0.0, 1.0);
}
