#version 330 core

// Template for the parser-generated domain-coloring shader. The sim replaces
// the two placeholder lines below at runtime (engine::Expression::glslComplexLib
// and the generated vec2 f(vec2 z)) and compiles via Shader::fromSources.

out vec4 FragColor;
in vec3 ourColor;
in vec3 localPos;

uniform float u_time;  // seconds since the sim started (variable t)
uniform vec2 u_mouse;  // cursor as a complex number in [-1,1]^2 (variable m)

//__COMPLEX_LIB__

//__F__

void main()
{
    // stereographic projection from north pole (0,0,1) onto the complex plane
    vec3 p = normalize(localPos);
    vec2 z = vec2(p.x, p.y) / (1.0 - p.z + 1e-6);

    vec2 fz = f(z);

    // domain coloring: hue encodes arg(f(z)), brightness encodes |f(z)|
    float arg = cArg(fz);
    float hue = arg / (2.0 * 3.14159265) + 0.5;
    float r = cMod(fz);
    float brightness = r / (1.0 + r); // compress [0,∞) → [0,1)

    // HSV→RGB (saturation=1)
    vec3 c = clamp(abs(fract(hue + vec3(0.0, 2.0/3.0, 1.0/3.0)) * 6.0 - 3.0) - 1.0, 0.0, 1.0);

    FragColor = vec4(brightness * c, 1.0);
}
