#version 330 core

// Shared by every primitive this sim draws: arrow shafts, arrowhead cones,
// trajectories, axes. aMag is whatever scalar the geometry carries — |f| at a glyph,
// speed along a trajectory — normalized here so the fragment ramp only ever sees
// [0, 1]. aFade is the glyph-local gradient coordinate (0 at an arrow's tail, 1 at its
// tip); trails have no such attribute and derive it from vertex age instead. aShade is
// the CPU-baked lambert term of an arrowhead facet (only the head mesh supplies it).
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aMag;
layout(location = 2) in float aFade;
layout(location = 3) in float aShade;

uniform mat4 view;
uniform mat4 projection;
uniform float u_magScale;   // 1 / (magnitude mapped to the top of the ramp)
uniform float u_pointSize;

// 0 = flat alpha, 1 = aFade attribute (arrows), 2 = age along the trail (line strips)
uniform int u_alphaMode;
uniform float u_ageFirst;   // gl_VertexID of the oldest live trail point
uniform float u_ageSpan;    // (live point count - 1), never 0

uniform int u_shadeMode;    // 1 = modulate color by aShade (arrowhead facets)

// Distance falloff: brightness rides continuously from full at a particle down to
// u_fadeMin at u_fadeFar, measured to the NEAREST particle, so a dense glyph grid
// stops burying the curves. u_focusCount = 0 disables it.
// u_fadeNear is normally 0 (the ramp starts at the particle); raising it holds a
// solid bubble around each one first. u_fadeCurve shapes the ramp: 1 = linear,
// below 1 = most of the brightness hugs the particle, above 1 = stays bright further.
const int kMaxFocus = 8;
uniform int u_focusCount;
uniform vec3 u_focus[kMaxFocus];
uniform float u_fadeNear;
uniform float u_fadeFar;
uniform float u_fadeMin;
uniform float u_fadeCurve;

out float vMag;
out float vFade;      // 0 = faded end, 1 = solid end
out float vShade;     // 1 = unshaded
out float vFocus;     // distance-falloff multiplier, already in [u_fadeMin, 1]

void main()
{
    gl_Position = projection * view * vec4(aPos, 1.0);
    gl_PointSize = u_pointSize;
    vMag = clamp(aMag * u_magScale, 0.0, 1.0);

    if (u_alphaMode == 1)
        vFade = aFade;
    else if (u_alphaMode == 2)
        vFade = clamp((float(gl_VertexID) - u_ageFirst) / u_ageSpan, 0.0, 1.0);
    else
        vFade = 1.0;

    vShade = (u_shadeMode == 1) ? aShade : 1.0;

    vFocus = 1.0;
    if (u_focusCount > 0)
    {
        float d = 1e30;
        for (int i = 0; i < u_focusCount; ++i)
            d = min(d, distance(aPos, u_focus[i]));
        float t = clamp((d - u_fadeNear) / max(u_fadeFar - u_fadeNear, 1e-4), 0.0, 1.0);
        vFocus = mix(1.0, u_fadeMin, pow(t, u_fadeCurve));
    }
}
