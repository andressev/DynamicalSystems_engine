#version 330 core

// Template for the parser-generated raymarching shader. The sim replaces the
// two placeholder lines at runtime (engine::Expression::glslComplexLib and the
// generated vec2 fc(vec3 p)) and compiles via Shader::fromSources.
//
// The typed f(x,y,z) is an arbitrary implicit function, NOT a signed distance
// field, so sphere tracing is unsafe — this marches the ray in fixed steps
// through a bounding sphere looking for a sign change of F = f - c, then
// refines the root by bisection (cf. Mitchell 1990).

out vec4 FragColor;
in vec2 ndc;

uniform mat4 u_invViewProj; // clip -> world, builds the per-pixel ray
uniform mat4 u_viewProj;    // world -> clip, writes gl_FragDepth at the hit
uniform float u_c;          // the level:  f(x,y,z) = c
uniform float u_range;      // bounding-sphere radius of the marched domain
uniform float u_time;       // seconds since the sim started (variable t)

//__COMPLEX_LIB__

//__F__

// the scalar field whose zero set we render (real part of the parsed value)
float F(vec3 p)
{
    return fc(p).x - u_c;
}

void main()
{
    // unproject the near/far corners of this pixel to get a world-space ray
    vec4 pNear = u_invViewProj * vec4(ndc, -1.0, 1.0);
    vec4 pFar  = u_invViewProj * vec4(ndc,  1.0, 1.0);
    vec3 ro = pNear.xyz / pNear.w;
    vec3 rd = normalize(pFar.xyz / pFar.w - ro);

    // clip the ray to the bounding sphere |p| = u_range
    float b = dot(ro, rd);
    float cc = dot(ro, ro) - u_range * u_range;
    float disc = b * b - cc;
    if (disc < 0.0)
        discard;
    float sq = sqrt(disc);
    float t0 = max(-b - sq, 0.0);
    float t1 = -b + sq;
    if (t1 <= t0)
        discard;

    // fixed-step march: find the first sign change of F along the ray
    const int STEPS = 220;
    float dt = (t1 - t0) / float(STEPS);
    float tPrev = t0;
    float fPrev = F(ro + rd * t0);
    bool hit = false;
    float tHit = t0;
    for (int i = 1; i <= STEPS; ++i)
    {
        float t = t0 + dt * float(i);
        float fCur = F(ro + rd * t);
        if (fPrev * fCur <= 0.0)
        {
            // bisection refine on [tPrev, t]
            float ta = tPrev, tb = t, fa = fPrev;
            for (int j = 0; j < 14; ++j)
            {
                float tm = 0.5 * (ta + tb);
                float fm = F(ro + rd * tm);
                if (fa * fm <= 0.0) { tb = tm; }
                else                { ta = tm; fa = fm; }
            }
            tHit = 0.5 * (ta + tb);
            hit = true;
            break;
        }
        tPrev = t;
        fPrev = fCur;
    }
    if (!hit)
        discard;

    vec3 p = ro + rd * tHit;

    // normal of a level set is the (normalized) gradient; central differences
    float e = 0.0015 * u_range;
    vec3 n = normalize(vec3(
        F(p + vec3(e, 0.0, 0.0)) - F(p - vec3(e, 0.0, 0.0)),
        F(p + vec3(0.0, e, 0.0)) - F(p - vec3(0.0, e, 0.0)),
        F(p + vec3(0.0, 0.0, e)) - F(p - vec3(0.0, 0.0, e))));

    // two-sided shading: teal where we see the f > c side, amber the f < c side
    bool backside = dot(n, rd) > 0.0;
    vec3 nn = backside ? -n : n;
    vec3 base = backside ? vec3(0.82, 0.52, 0.24) : vec3(0.22, 0.62, 0.56);

    vec3 l = normalize(vec3(0.5, 0.8, 0.3));
    float diff = max(dot(nn, l), 0.0);
    float head = max(dot(nn, -rd), 0.0);   // headlight so no side is pitch black
    vec3 h = normalize(l - rd);
    float spec = pow(max(dot(nn, h), 0.0), 48.0);
    vec3 col = base * (0.12 + 0.55 * diff + 0.33 * head) + vec3(0.35) * spec;

    // real depth for the hit so the axis lines intersect the surface correctly
    vec4 clip = u_viewProj * vec4(p, 1.0);
    gl_FragDepth = clamp(clip.z / clip.w * 0.5 + 0.5, 0.0, 1.0);

    FragColor = vec4(col, 1.0);
}
