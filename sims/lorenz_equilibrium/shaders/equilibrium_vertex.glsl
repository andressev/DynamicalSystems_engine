#version 330 core

// The equilibrium markers: a unit sphere (engine::Shapes, layout {3, 3, 2}) placed at
// v* and scaled to u_radius. Drawn twice per marker: the solid bead, then (when it
// glows) a larger halo shell.
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 view;
uniform mat4 projection;
uniform vec3 u_center;
uniform float u_radius;

out vec3 vWorld;
out vec3 vNormal;
out vec3 vSphere;   // the unit-sphere point this is (the glass shockwave measures arcs on it)

void main()
{
    vec3 world = u_center + u_radius * aPos;
    vWorld = world;
    vSphere = aPos;
    vNormal = aNormal;
    gl_Position = projection * view * vec4(world, 1.0);
}
