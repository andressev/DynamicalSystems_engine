#version 330 core

// Fullscreen pass: Shapes::planeVertices() is an XZ quad spanning [-1,1], so
// (x, z) maps straight onto NDC. No matrices involved; the camera enters only
// through the fragment shader's unprojection uniforms.

layout(location = 0) in vec3 aPos;

out vec2 ndc;

void main()
{
    ndc = vec2(aPos.x, aPos.z);
    gl_Position = vec4(aPos.x, aPos.z, 0.0, 1.0);
}
