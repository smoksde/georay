#version 330 core

layout(location = 0) in vec3 a_position;   // ENU metres
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec2 a_map_uv0;    // layer 0 raster coordinates, v downwards
layout(location = 3) in vec2 a_map_uv1;    // layer 1

uniform mat4 u_view_projection;
uniform int u_map_layer;                   // which layer's coordinates to use

out vec3 v_world;
out vec3 v_normal;
out vec2 v_map_uv;

void main() {
    v_world = a_position;
    v_normal = a_normal;
    v_map_uv = (u_map_layer == 1) ? a_map_uv1 : a_map_uv0;
    gl_Position = u_view_projection * vec4(a_position, 1.0);
}
