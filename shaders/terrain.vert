#version 330 core

layout(location = 0) in vec3 a_position;  // ENU metres
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_map_uv;    // map raster coordinates, v downwards

uniform mat4 u_view_projection;

out vec3 v_world;
out vec3 v_normal;
out vec2 v_map_uv;

void main() {
    v_world = a_position;
    v_normal = a_normal;
    v_map_uv = a_map_uv;
    gl_Position = u_view_projection * vec4(a_position, 1.0);
}
