#version 330 core

layout(location = 0) in vec3 a_position;  // ENU metres, relative to the view origin
layout(location = 1) in vec3 a_normal;

uniform mat4 u_view_projection;

out vec3 v_world;
out vec3 v_normal;

void main() {
    v_world = a_position;
    v_normal = a_normal;
    gl_Position = u_view_projection * vec4(a_position, 1.0);
}
