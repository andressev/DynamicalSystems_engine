#version 330 core

in vec3 vWorld;
in vec3 vNormal;
out vec4 FragColor;

uniform vec3 u_eye;
uniform int u_mode;       // 0 = glass ball, 1 = flat-colored lines (the E^s disk)
uniform vec3 u_color;     // glass tint / line color
uniform float u_alpha;

void main()
{
    if (u_mode == 1)
    {
        FragColor = vec4(u_color, u_alpha);
        return;
    }

    // glass: faint tint face-on, bright Fresnel rim, one sharp highlight; two-sided
    vec3 v = normalize(u_eye - vWorld);
    vec3 n = normalize(vNormal);
    if (dot(n, v) < 0.0)
        n = -n;
    float ndv = clamp(dot(n, v), 0.0, 1.0);
    float fresnel = pow(1.0 - ndv, 3.0);
    vec3 L = normalize(vec3(0.35, 0.75, 0.55));
    float spec = pow(max(dot(reflect(-L, n), v), 0.0), 80.0);

    vec3 rgb = u_color * (0.25 + 0.35 * ndv) + vec3(0.85) * fresnel + vec3(spec);
    float alpha = clamp(u_alpha + 0.65 * fresnel + spec, 0.0, 1.0);
    FragColor = vec4(rgb, alpha);
}
