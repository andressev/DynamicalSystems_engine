#version 330 core

// "Transform flow" (stage C): trails and particles of the linear flow carried from the
// linear ball U to V, every vertex at once. A trail point theta (relative to v*) goes
// to   v* + u_shift + mix(theta, h~^{-1}(theta), u_morph),   u_shift sliding from U's
// offset to 0 — the same h~^{-1} that deforms V (engine::ConjugacyMap). The line below
// is replaced by ConjugacyMap::glsl() when the sim loads this file. Shaded by
// fragment.glsl (flat color, round points for the heads).
// @CONJUGACY_MAP

layout(location = 0) in vec3 aPos;   // trail point (world: v* + theta)
layout(location = 1) in float aMag;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 u_center;     // v*
uniform vec3 u_shift;      // U's offset * (1 - morph)
uniform float u_morph;     // 0 = in U, 1 = in V
uniform float u_pointSize;

out float vMag;
out float vFade;
out float vShade;
out vec3 vWorld;

void main()
{
    vec3 theta = aPos - u_center;
    vec3 x = u_center + u_shift + mix(theta, hInv(theta), u_morph);
    vWorld = x;
    vMag = 0.0;
    vFade = 1.0;
    vShade = 1.0;
    gl_PointSize = u_pointSize;
    gl_Position = projection * view * vec4(x, 1.0);
}
