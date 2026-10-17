#version 330 core

in float vMag;
in float vFade;
in float vShade;
in vec3 vWorld;
out vec4 FragColor;

uniform int u_useRamp;     // 1 = color by magnitude, 0 = flat u_color
uniform vec3 u_color;
uniform vec3 u_color2;     // second color of the u_useRamp == 2 gradient
uniform float u_alpha;     // overall opacity of this draw
uniform float u_alphaTail; // opacity multiplier at the faded end (vFade = 0)
uniform int u_round;       // 1 = points are shaded discs instead of squares
uniform vec3 u_clipCenter;  // the ball V = B_r(x*): nothing is drawn outside it
uniform float u_clipRadius; // <= 0: no clipping

// Same cold-to-hot ramp as vector_field: slow reads blue, fast reads red.
vec3 ramp(float s)
{
    vec3 c0 = vec3(0.16, 0.26, 0.90);
    vec3 c1 = vec3(0.10, 0.80, 0.85);
    vec3 c2 = vec3(0.55, 0.90, 0.25);
    vec3 c3 = vec3(0.98, 0.80, 0.15);
    vec3 c4 = vec3(0.95, 0.20, 0.18);

    float x = clamp(s, 0.0, 1.0) * 4.0;
    if (x < 1.0) return mix(c0, c1, x);
    if (x < 2.0) return mix(c1, c2, x - 1.0);
    if (x < 3.0) return mix(c2, c3, x - 2.0);
    return mix(c3, c4, x - 3.0);
}

void main()
{
    float edge = 1.0;
    if (u_clipRadius > 0.0)
    {
        float d = length(vWorld - u_clipCenter);
        if (d > u_clipRadius)
            discard;
        edge = 1.0 - smoothstep(0.96 * u_clipRadius, u_clipRadius, d); // soft rim
    }

    // u_useRamp 2: a two-color gradient along vMag, u_color -> u_color2 (the h^{-1} links,
    // from psi^t's orange to phi^t's green)
    vec3 base = u_useRamp == 1 ? ramp(vMag) : u_useRamp == 2 ? mix(u_color, u_color2, vMag) : u_color;
    vec3 rgb = base * vShade;
    float alpha = u_alpha * edge * mix(u_alphaTail, 1.0, vFade);

    if (u_round == 1)
    {
        vec2 d = gl_PointCoord - vec2(0.5);
        float r = length(d) * 2.0;
        if (r > 1.0)
            discard;
        alpha *= smoothstep(1.0, 0.8, r);
        // fake a lit sphere so a particle reads as a bead
        float lambert = clamp(sqrt(max(0.0, 1.0 - r * r)) * 0.55
                              + dot(normalize(vec3(d.x, -d.y, 0.75)),
                                    normalize(vec3(-0.4, 0.5, 0.75))) * 0.45,
                              0.0, 1.0);
        rgb *= 0.55 + 0.65 * lambert;
    }

    FragColor = vec4(rgb, alpha);
}
