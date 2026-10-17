#version 330 core

// The glass balls: V = B_r(x*) and, in stage C, U and the deformed V (drawn last,
// blended, no depth writes). Fresnel-weighted: nearly clear face-on, bright at the
// silhouette, plus a sharp highlight; back faces at half strength so the far wall reads
// as the inside of the ball.
//
// Crash shockwaves (stage C): where a pair hits the rim, a ring runs over the sphere
// from the impact point, bending and color-splitting the scene behind it (u_scene, a
// engine::FrameCopy of the frame so far) with a flash inside it — after the "shockwave"
// shader by Benjamin Weschler (MIT, 2023), moved from screen space onto the sphere:
// the distance is the arc from the impact over u_waveReach on the sphere the ball is
// made from (vSphere — on V that is the linear ball's, so the ring hugs V's real border),
// the push is radial on screen from the impact.
in vec3 vWorld;
in vec3 vNormal;
in vec3 vSphere;
out vec4 FragColor;

uniform vec3 u_eye;
uniform vec3 u_color;    // tint
uniform float u_alpha;   // overall strength (fades in/out around a handoff)

uniform vec3 u_center;   // this ball's centre and radius (shared with the vertex stage)
uniform float u_radius;

const int kMaxWaves = 8;
const float PI = 3.1415926;
uniform int u_waveCount;                 // 0 = no shockwave on this ball
uniform vec3 u_waveDir[kMaxWaves];       // impact, as a point of the unit sphere the ball is made from
uniform vec3 u_wavePos[kMaxWaves];       // impact, in world space
uniform float u_waveT[kMaxWaves];        // 0 -> 1 over the wave's life
uniform vec3 u_waveColor[kMaxWaves];     // the exploding particle's color
uniform float u_waveReach;               // arc (radians) the ring travels
uniform float u_waveForce;               // push, in ball radii on screen
uniform float u_waveThickness;
uniform float u_waveFeather;
uniform float u_waveAberration;          // color split, in ball radii on screen
uniform float u_waveFlash;
uniform sampler2D u_scene;
uniform vec4 u_viewport;                 // x, y, width, height of the captured frame
uniform mat4 u_viewProj;
uniform float u_proj11;                  // projection[1][1]

const vec3 kLightDir = vec3(0.35, 0.75, 0.55) / length(vec3(0.35, 0.75, 0.55));

vec2 toPixels(vec3 p)
{
    vec4 c = u_viewProj * vec4(p, 1.0);
    return (c.xy / c.w * 0.5 + 0.5) * u_viewport.zw + u_viewport.xy;
}

void main()
{
    vec3 v = normalize(u_eye - vWorld);
    vec3 n = normalize(vNormal);
    if (!gl_FrontFacing)
        n = -n;
    float ndv = clamp(dot(n, v), 0.0, 1.0);
    float fresnel = pow(1.0 - ndv, 3.0);
    float spec = pow(max(dot(reflect(-kLightDir, n), v), 0.0), 90.0);
    float side = gl_FrontFacing ? 1.0 : 0.5;

    float a = u_alpha * side * (0.04 + 0.55 * fresnel + 0.7 * spec);
    vec3 rgb = mix(u_color, vec3(1.0), 0.4 * fresnel + spec);

    if (u_waveCount > 0)
    {
        // arcs are measured on the undeformed sphere, so on V the ring follows the
        // real (deformed) border
        vec3 dirHere = normalize(vSphere);
        vec4 cc = u_viewProj * vec4(u_center, 1.0);
        float radiusPx = u_radius * u_proj11 / max(cc.w, 1e-4) * 0.5 * u_viewport.w;
        vec2 frag = gl_FragCoord.xy;

        vec2 push = vec2(0.0), split = vec2(0.0);
        float ring = 0.0, flash = 0.0;
        vec3 flashColor = vec3(0.0);
        for (int i = 0; i < u_waveCount; ++i)
        {
            float d = acos(clamp(dot(dirHere, u_waveDir[i]), -1.0, 1.0)) / u_waveReach;
            if (d > 1.2)
                continue;
            float pos = sin(u_waveT[i] * PI * 0.5);   // easeOutSine
            float inner = smoothstep(pos - u_waveThickness - u_waveFeather, pos - u_waveThickness, d);
            float outer = smoothstep(pos - u_waveFeather, pos, d);
            float shape = inner - outer;
            vec2 r = frag - toPixels(u_wavePos[i]);
            vec2 dir = dot(r, r) > 1e-6 ? normalize(r) : vec2(0.0);
            push += dir * u_waveForce * radiusPx * shape;
            split += dir * u_waveAberration * radiusPx * shape;
            ring = max(ring, shape);
            float ip = 1.0 - pos;
            float f = (1.0 - inner) * ip * ip * ip * ip;
            if (f > flash)
            {
                flash = f;
                flashColor = u_waveColor[i];
            }
        }
        float m = clamp(max(ring, flash), 0.0, 1.0);
        if (m > 0.001)
        {
            vec2 uv = (frag - u_viewport.xy) / u_viewport.zw;
            vec2 px = 1.0 / u_viewport.zw;
            float rC = texture(u_scene, uv - (push - split) * px).r;
            float gC = texture(u_scene, uv - push * px).g;
            float bC = texture(u_scene, uv - (push + split) * px).b;
            vec3 scene = vec3(rC, gC, bC);
            // the flash brightens what is behind, and glows in the particle's color so
            // it shows on the dark background too
            scene += u_waveFlash * scene * flash + 0.35 * u_waveFlash * flashColor * flash;
            // a thin bright crest in the particle's color on the moving ring
            scene += 0.25 * u_waveFlash * flashColor * ring;
            rgb = mix(rgb, scene, m);
            a = mix(a, u_alpha, m);
        }
    }
    FragColor = vec4(rgb, clamp(a, 0.0, 1.0));
}
