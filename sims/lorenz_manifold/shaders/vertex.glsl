#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColorLevel;
layout(location = 2) in vec3 aColorHeight;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform int u_colorMode; // 0 = by ring level, 1 = by height (z)

out vec3 vColor;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    gl_PointSize = 4.0;
    vColor = (u_colorMode == 1) ? aColorHeight : aColorLevel;
}
