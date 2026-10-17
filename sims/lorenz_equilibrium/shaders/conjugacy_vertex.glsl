#version 330 core

// The deformed ball V = x* + h~^{-1}(B_rho(0)) of stage C: the unit sphere pushed
// through engine::ConjugacyMap on the GPU (h~^{-1} = K_s + K_u, exact on E^s and E^u,
// an interpolation between). Shaded by glass_fragment.glsl. The line below is replaced
// by ConjugacyMap::glsl() when the sim loads this file.
// @CONJUGACY_MAP

layout(location = 0) in vec3 aPos;   // unit sphere

uniform mat4 view;
uniform mat4 projection;
uniform vec3 u_center;     // x*
uniform float u_radius;    // rho of the linear ball
uniform int u_morphOn;     // 1: the "Transform flow" ghost, between U's sphere and V
uniform float u_morph;     // 0 = U's round sphere, 1 = V
uniform vec3 u_shift;      // U's offset * (1 - morph)

out vec3 vWorld;
out vec3 vNormal;
out vec3 vSphere;   // theta / rho: where on the linear ball's sphere this point of V comes from

vec3 mapPoint(vec3 q)
{
    vec3 t = u_radius * normalize(q);
    return u_morphOn == 1 ? mix(t, hInv(t), u_morph) + u_shift : hInv(t);
}

void main()
{
    vec3 p = normalize(aPos);
    vSphere = p;
    vec3 x = mapPoint(p);

    // normal of the deformed surface by forward differences along the sphere
    vec3 h = abs(p.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 t1 = normalize(cross(p, h));
    vec3 t2 = cross(p, t1);
    const float e = 1e-3;
    vec3 n = cross(mapPoint(p + e * t1) - x, mapPoint(p + e * t2) - x);
    if (dot(n, x - (u_morphOn == 1 ? u_shift : vec3(0.0))) < 0.0)
        n = -n;   // outward

    vWorld = u_center + x;
    vNormal = n;
    gl_Position = projection * view * vec4(vWorld, 1.0);
}
