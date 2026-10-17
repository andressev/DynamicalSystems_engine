#version 330 core

// Shared by arrows, arrowheads, trajectories and axes (same pair as vector_field and
// diagonal_linear).
// aMag is |f| at a glyph (speed along a trail); aFade runs 0 at an arrow's tail to 1 at
// its tip; aShade is the CPU-baked lambert term of an arrowhead facet. Meshes that do
// not supply an attribute get the GL default (0), and the uniforms below ignore it.
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aMag;
layout(location = 2) in float aFade;
layout(location = 3) in float aShade;
// instanced arrow glyphs (engine::ArrowGlyphs): aPos is then the base shape's local
// position in arrow lengths (x along the arrow), aNormal its local facet normal
// (w = 1 -> lit here, else aShade is used), and each instance carries its tail,
// unit direction and (length, |f|)
layout(location = 4) in vec4 aNormal;
layout(location = 5) in vec3 iTail;
layout(location = 6) in vec3 iDir;
layout(location = 7) in vec2 iLenMag;

uniform mat4 view;
uniform mat4 projection;
uniform float u_magScale;   // 1 / (magnitude mapped to the top of the ramp)
uniform float u_pointSize;
uniform int u_useFade;      // 1 = opacity gradient from aFade (arrow shafts)
uniform int u_shadeMode;    // 1 = modulate color by aShade (arrowhead facets)
uniform int u_instanced;    // 1 while ArrowGlyphs draws a batch
uniform int u_flat;         // 1 = R2 batch: the arrow frame lies in the z = 0 plane

out float vMag;
out float vFade;
out float vShade;

const vec3 kLightDir = vec3(0.35, 0.75, 0.55) / length(vec3(0.35, 0.75, 0.55));
const float kAmbient = 0.55;

void main()
{
    vec3 pos = aPos;
    float mag = aMag;
    float shade = aShade;
    if (u_instanced == 1)
    {
        // the arrow's frame: d along it, a and b across (same basis the CPU used)
        vec3 d = iDir;
        vec3 a, b;
        if (u_flat == 1)
        {
            a = vec3(-d.y, d.x, 0.0);
            b = vec3(0.0);
        }
        else
        {
            vec3 helper = abs(d.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
            a = normalize(cross(d, helper));
            b = cross(d, a);
        }
        pos = iTail + iLenMag.x * (d * aPos.x + a * aPos.y + b * aPos.z);
        mag = iLenMag.y;
        if (aNormal.w > 0.5)
        {
            vec3 n = normalize(d * aNormal.x + a * aNormal.y + b * aNormal.z);
            shade = kAmbient + (1.0 - kAmbient) * max(0.0, dot(n, kLightDir));
        }
    }

    gl_Position = projection * view * vec4(pos, 1.0);
    gl_PointSize = u_pointSize;
    vMag = clamp(mag * u_magScale, 0.0, 1.0);
    vFade = (u_useFade == 1) ? aFade : 1.0;
    vShade = (u_shadeMode == 1) ? shade : 1.0;
}
