#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec4 a_colour;

uniform mat4 u_view_projection;

out vec4 v_colour;

void main() {
    v_colour = a_colour;
    gl_Position = u_view_projection * vec4(a_position, 1.0);
}
