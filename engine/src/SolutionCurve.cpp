#include <engine/SolutionCurve.h>
#include <engine/Shapes.h>

#include <gtc/matrix_transform.hpp>

namespace engine {

namespace {

const char* kVertexSrc = R"(#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 mvp;
out vec3 normal;
void main()
{
    gl_Position = mvp * vec4(aPos, 1.0);
    normal = aNormal;
}
)";

const char* kBodyFragSrc = R"(#version 330 core
in vec3 normal;
out vec4 FragColor;
uniform vec3 color;
void main()
{
    vec3 l = normalize(vec3(0.5, 0.8, 0.3));
    float diff = max(dot(normalize(normal), l), 0.0);
    FragColor = vec4(color * (0.35 + 0.65 * diff), 1.0);
}
)";

const char* kTrailFragSrc = R"(#version 330 core
out vec4 FragColor;
uniform vec3 color;
void main()
{
    FragColor = vec4(color, 1.0);
}
)";

} // namespace

void SolutionCurve::reset(const glm::vec3& position)
{
    pos = position;
    carry = 0.0f;
    trail.clear();
    trail.push_back(pos);
    trailDirty = true;
}

void SolutionCurve::update(float dt, float t)
{
    if (!field)
        return;

    carry += dt;
    // cap the work per frame so a tiny stepSize degrades speed, not framerate
    int steps = 0;
    const int maxStepsPerFrame = 2000;
    float localT = t - carry;
    while (carry >= stepSize && steps < maxStepsPerFrame)
    {
        pos = rk4Step(pos, localT, stepSize);
        localT += stepSize;
        carry -= stepSize;
        ++steps;
        if (trailEnabled)
            pushTrailPoint(pos);
    }
}

glm::vec3 SolutionCurve::rk4Step(const glm::vec3& x, float t, float h) const
{
    glm::vec3 k1 = field(x, t);
    glm::vec3 k2 = field(x + 0.5f * h * k1, t + 0.5f * h);
    glm::vec3 k3 = field(x + 0.5f * h * k2, t + 0.5f * h);
    glm::vec3 k4 = field(x + h * k3, t + h);
    return x + (h / 6.0f) * (k1 + 2.0f * k2 + 2.0f * k3 + k4);
}

void SolutionCurve::pushTrailPoint(const glm::vec3& p)
{
    trail.push_back(p);
    if ((int)trail.size() > trailMax)
        trail.erase(trail.begin(), trail.begin() + (trail.size() - trailMax));
    trailDirty = true;
}

void SolutionCurve::ensureGL()
{
    if (bodyMesh)
        return;

    bodyMesh = std::make_unique<Mesh>(
        Shapes::sphereVertices(16, 16, 1.0f),
        Shapes::sphereIndices(16, 16),
        std::vector<int>{3, 3, 2});
    // placeholder line; updateData() re-uploads the real trail each frame
    trailMesh = std::make_unique<Mesh>(
        std::vector<float>{0.f, 0.f, 0.f, 0.f, 0.f, 0.f},
        std::vector<unsigned int>{0, 1},
        std::vector<int>{3});

    bodyShader = Shader::fromSources(kVertexSrc, kBodyFragSrc);
    trailShader = Shader::fromSources(kVertexSrc, kTrailFragSrc);
}

void SolutionCurve::draw(const glm::mat4& view, const glm::mat4& projection)
{
    ensureGL();

    glm::mat4 vp = projection * view;

    if (trailEnabled && trail.size() >= 2)
    {
        if (trailDirty)
        {
            std::vector<float> verts;
            verts.reserve(trail.size() * 3);
            std::vector<unsigned int> idx;
            idx.reserve(trail.size());
            for (size_t i = 0; i < trail.size(); ++i)
            {
                verts.push_back(trail[i].x);
                verts.push_back(trail[i].y);
                verts.push_back(trail[i].z);
                idx.push_back((unsigned int)i);
            }
            trailMesh->updateData(verts, idx);
            trailDirty = false;
        }
        glLineWidth(2.0f);
        trailShader.use();
        trailShader.setMat4("mvp", vp);
        trailShader.setVec3("color", trailColor);
        trailMesh->draw(GL_LINE_STRIP);
    }

    glm::mat4 model = glm::scale(glm::translate(glm::mat4(1.0f), pos), glm::vec3(bodyRadius));
    bodyShader.use();
    bodyShader.setMat4("mvp", vp * model);
    bodyShader.setVec3("color", bodyColor);
    bodyMesh->draw(GL_TRIANGLES);
}

} // namespace engine
