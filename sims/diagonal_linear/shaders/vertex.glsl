#version 330 core

// Shared by arrows, arrowheads, trajectories, axes and the invariant subspaces.
// aMag is |f| at a glyph (speed along a trail); aFade runs 0 at an arrow's tail to 1 at
// its tip; aShade is the CPU-baked lambert term of an arrowhead facet. Meshes that do
// not supply an attribute get the GL default (0), and the uniforms below ignore it.
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aMag;
layout(location = 2) in float aFade;
layout(location = 3) in float aShade;

uniform mat4 view;
uniform mat4 projection;
uniform float u_magScale;   // 1 / (magnitude mapped to the top of the ramp)
uniform float u_pointSize;
uniform int u_useFade;      // 1 = opacity gradient from aFade (arrow shafts)
uniform int u_shadeMode;    // 1 = modulate color by aShade (arrowhead facets)

out float vMag;
out float vFade;
out float vShade;

void main()
{
    gl_Position = projection * view * vec4(aPos, 1.0);
    gl_PointSize = u_pointSize;
    vMag = clamp(aMag * u_magScale, 0.0, 1.0);
    vFade = (u_useFade == 1) ? aFade : 1.0;
    vShade = (u_shadeMode == 1) ? aShade : 1.0;
}
