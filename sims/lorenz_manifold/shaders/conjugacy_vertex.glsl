#version 330 core

// Meshes pushed through h~^{-1} on the GPU (engine::ConjugacyMap): the unit sphere
// becomes V = h~^{-1}(B_r(0)), a unit disk in E^s becomes the W^s patch. The line
// below is replaced by ConjugacyMap::glsl() when the sim loads this file.
// @CONJUGACY_MAP

layout(location = 0) in vec3 aPos;   // unit sphere point, or (x, y, 0) in the unit disk

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 u_center;     // x*
uniform float u_radius;    // r of B_r(0)
uniform int u_mode;        // 0 = sphere, 1 = disk in E^s
uniform vec3 u_e0;         // orthonormal basis of E^s (disk mode)
uniform vec3 u_e1;

out vec3 vWorld;
out vec3 vNormal;

vec3 thetaOf(vec3 p)
{
    return u_mode == 0 ? u_radius * normalize(p) : u_radius * (p.x * u_e0 + p.y * u_e1);
}

void main()
{
    vec3 x = hInv(thetaOf(aPos));

    // normal of the deformed surface by forward differences in the mesh's own parameters
    vec3 t1, t2;
    if (u_mode == 0)
    {
        vec3 n = normalize(aPos);
        vec3 h = abs(n.z) < 0.9 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
        t1 = normalize(cross(n, h));
        t2 = cross(n, t1);
    }
    else
    {
        t1 = vec3(1.0, 0.0, 0.0);
        t2 = vec3(0.0, 1.0, 0.0);
    }
    const float e = 1e-3;
    vec3 n = cross(hInv(thetaOf(aPos + e * t1)) - x, hInv(thetaOf(aPos + e * t2)) - x);
    if (u_mode == 0 && dot(n, x) < 0.0)
        n = -n;   // outward on the ball

    vec4 world = model * vec4(u_center + x, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(model) * n;
    gl_Position = projection * view * world;
}
