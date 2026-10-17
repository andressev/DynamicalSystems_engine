#version 330 core

in vec3 vWorld;
in vec3 vNormal;
out vec4 FragColor;

uniform vec3 u_eye;
uniform vec3 u_color;
uniform float u_glow;   // 0 = idle, 1 = hovered (eased on the CPU, pulses)
uniform int u_mode;     // 0 = solid bead, 1 = halo shell (additive)
uniform float u_alpha;  // fades the bead (and its halo) in and out around a handoff

const vec3 kLightDir = vec3(0.35, 0.75, 0.55) / length(vec3(0.35, 0.75, 0.55));

void main()
{
    vec3 v = normalize(u_eye - vWorld);
    vec3 n = normalize(vNormal);
    float ndv = clamp(dot(n, v), 0.0, 1.0);

    if (u_mode == 1)
    {
        // soft ball of light: brightest through the middle, nothing at the silhouette
        float a = u_glow * pow(ndv, 2.0) * 0.55 * u_alpha;
        FragColor = vec4(u_color * a, a);
        return;
    }

    float lambert = max(dot(n, kLightDir), 0.0);
    float spec = pow(max(dot(reflect(-kLightDir, n), v), 0.0), 40.0);
    float rim = pow(1.0 - ndv, 2.5);
    vec3 rgb = u_color * (0.45 + 0.55 * lambert) + vec3(0.6 * spec)
             + mix(vec3(0.0), vec3(1.0, 0.75, 0.6), u_glow) * rim * (0.4 + 0.8 * u_glow)
             + u_color * 0.5 * u_glow;
    FragColor = vec4(rgb, u_alpha);
}
