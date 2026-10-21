#version 330 core

// The bands between geodesic level sets (engine::LevelSetSurface layout): each vertex
// with where it grew from and its band. Band k is drawn grown by clamp(u_grow - k, 0, 1)
// — 0 collapses it onto the level set it starts from, 1 is the band as solved.

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aFrom;
layout(location = 2) in float aBand;

uniform mat4 view;
uniform mat4 projection;
uniform float u_grow;   // bands grown (fractional: the last one partly)

flat out float vBand;

void main()
{
    float t = clamp(u_grow - aBand, 0.0, 1.0);
    gl_Position = projection * view * vec4(mix(aFrom, aPos, t), 1.0);
    vBand = aBand;
}
