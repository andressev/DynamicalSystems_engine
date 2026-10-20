#version 330 core

// A 2-D local manifold x* + K(theta): the unit disk of E scaled to the grown radius
// and pushed through engine::ConjugacyMap on the GPU (on E it is exactly K). Shaded by
// fragment.glsl. The line below is replaced by ConjugacyMap::glsl() when the sim
// loads this file.
// @CONJUGACY_MAP

layout(location = 0) in vec3 aPos;   // theta / radius, on the unit disk of E

uniform mat4 view;
uniform mat4 projection;
uniform vec3 u_center;    // x*
uniform float u_radius;   // trusted radius * growth

out float vMag;
out float vFade;
out float vShade;

void main()
{
    gl_Position = projection * view * vec4(u_center + hInv(u_radius * aPos), 1.0);
    vMag = 0.0;
    vFade = 1.0;
    vShade = 1.0;
}
