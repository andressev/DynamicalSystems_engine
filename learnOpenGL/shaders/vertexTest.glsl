#version 330 core

layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 ourColor;
out vec3 localPos;


void main()
{
    vec4 pos=projection*view*model*vec4(aPos+2.f,1.0f);
    
    gl_Position = pos;
    localPos = vec3(30.f);
    
    ourColor = pos.xyz;
}