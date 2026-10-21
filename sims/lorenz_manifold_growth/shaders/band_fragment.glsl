#version 330 core

// Bands alternate light / dark, so every geodesic level set shows as a color change.

flat in float vBand;
out vec4 FragColor;

uniform vec3 u_colorEven;   // bands 0, 2, 4, ...
uniform vec3 u_colorOdd;
uniform float u_alpha;

void main()
{
    FragColor = vec4(mod(vBand, 2.0) < 0.5 ? u_colorEven : u_colorOdd, u_alpha);
}
