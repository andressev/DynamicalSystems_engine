#version 330 core

out vec4 FragColor;
in vec3 ourColor;
in vec3 localPos;

uniform vec2 resolution;
uniform vec2 mouse; // normalized [0,1] x [0,1], origin bottom-left
vec2 cMul(vec2 a, vec2 b);
vec2 cDiv(vec2 a, vec2 b);
vec2 cConj(vec2 a);
float cMod(vec2 a);
float cArg(vec2 a);
vec2 cExp(vec2 a);
vec2 cLog(vec2 a);
vec2 cPow(vec2 a, vec2 b);
vec2 cSqrt(vec2 a);
vec2 cSin(vec2 a);
vec2 cCos(vec2 a);


void main()
{
    // stereographic projection from north pole (0,0,1) onto the complex plane
    vec3 p = normalize(localPos);
    vec2 z = vec2(p.x, p.y) / (1.0 - p.z + 1e-6);

    float mx = mouse.x;
    float my = mouse.y;

    vec2 z2 = cMul(z, z);
    vec2 z3 = cMul(z2, z);
    vec2 z4 = cMul(z3, z);
    vec2 z5 = cMul(z4, z);
    vec2 z6 = cMul(z5, z);
    vec2 z7 = cMul(z6, z);

    // each coefficient is a real scalar built from mx/my combinations
    float a0 =  mx * my;
    float a1 =  mx - my;
    float a2 =  mx * mx - my;
    float a3 =  my * my - mx;
    float a4 =  mx * mx * my;
    float a5 =  my * my * mx;
    float a6 =  mx - my * my;
    float a7 =  mx * my * (mx - my);

    vec2 poly = a0 * vec2(1.0, 0.0)
              + a1 * z
              + a2 * z2
              + a3 * z3
              + a4 * z4
              + a5 * z5
              + a6 * z6
              + a7 * z7;

    vec2 fz =poly;

    // domain coloring: hue encodes arg(f(z)), brightness encodes |f(z)|
    float arg = cArg(fz);
    float hue = arg / (2.0 * 3.14159265) + 0.5;
    float r = cMod(fz);
    float brightness = r / (1.0 + r); // compress [0,∞) → [0,1)

    // HSV→RGB (saturation=1)
    vec3 c = clamp(abs(fract(hue + vec3(0.0, 2.0/3.0, 1.0/3.0)) * 6.0 - 3.0) - 1.0, 0.0, 1.0);
    
    FragColor = vec4(brightness * c, 1.0);
}

// a = (ax, ay), b = (bx, by)

vec2 cMul(vec2 a, vec2 b) {
    return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
}

vec2 cDiv(vec2 a, vec2 b) {
    float denom = dot(b, b); // |b|^2
    return vec2(dot(a, b), a.y * b.x - a.x * b.y) / denom;
}

vec2 cConj(vec2 a) {
    return vec2(a.x, -a.y);
}

float cMod(vec2 a) {
    return length(a); // sqrt(x^2 + y^2)
}

float cArg(vec2 a) {
    return atan(a.y, a.x);
}

vec2 cExp(vec2 a) {
    // e^(x+iy) = e^x * (cos y + i sin y)
    return exp(a.x) * vec2(cos(a.y), sin(a.y));
}

vec2 cLog(vec2 a) {
    // ln|a| + i*arg(a)
    return vec2(log(length(a)), atan(a.y, a.x));
}

vec2 cPow(vec2 a, vec2 b) {
    // a^b = e^(b * ln(a))
    return cExp(cMul(b, cLog(a)));
}

vec2 cSqrt(vec2 a) {
    float r = length(a);
    return vec2(sqrt((r + a.x) / 2.0), sign(a.y) * sqrt((r - a.x) / 2.0));
}

vec2 cSin(vec2 a) {
    // sin(x+iy) = sin(x)cosh(y) + i cos(x)sinh(y)
    return vec2(sin(a.x) * cosh(a.y), cos(a.x) * sinh(a.y));
}

vec2 cCos(vec2 a) {
    // cos(x+iy) = cos(x)cosh(y) - i sin(x)sinh(y)
    return vec2(cos(a.x) * cosh(a.y), -sin(a.x) * sinh(a.y));
}