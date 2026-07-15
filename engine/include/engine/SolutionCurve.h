#ifndef ENGINE_SOLUTION_CURVE_H
#define ENGINE_SOLUTION_CURVE_H

#include <engine/Mesh.h>
#include <engine/Shader.h>

#include <glm.hpp>
#include <functional>
#include <memory>
#include <vector>

namespace engine {

// A CPU-integrated particle tracing a solution curve of x' = field(x, t).
// Sims attach one by declaring a member, hand it a vector field (any
// std::function — ambient, projected-onto-a-surface, 2D by keeping z = 0),
// call update(dt, t) each frame and draw(view, projection) each render.
// Integrates with fixed-step RK4; renders as a small lit sphere plus an
// optional line-strip trail of the curve so far. GL resources are created
// lazily on first draw, so plain member declaration is safe.
class SolutionCurve {
public:
    using Field = std::function<glm::vec3(const glm::vec3& p, float t)>;

    Field field;                          // x' = field(x, t); no-op while unset
    float stepSize = 0.001f;               // RK4 step h, in simulation-time units
    bool trailEnabled = true;
    int trailMax = 2000;                  // oldest trail points dropped past this
    float bodyRadius = 0.08f;
    glm::vec3 bodyColor{0.95f, 0.80f, 0.30f};
    glm::vec3 trailColor{0.95f, 0.80f, 0.30f};

    // Place the particle and clear its trail.
    void reset(const glm::vec3& position);

    const glm::vec3& position() const { return pos; }

    // Advance simulation time by dt, taking RK4 steps of stepSize
    // (leftover time is carried to the next frame). t is the sim clock,
    // passed through to time-dependent fields.
    void update(float dt, float t);

    // Draw the body (and trail, if enabled) with the sim's camera matrices.
    void draw(const glm::mat4& view, const glm::mat4& projection);

private:
    glm::vec3 pos{0.0f};
    float carry = 0.0f;                   // accumulated time < one step
    std::vector<glm::vec3> trail;
    bool trailDirty = false;

    // lazy GL state (created on first draw, while the context is live)
    std::unique_ptr<Mesh> bodyMesh;
    std::unique_ptr<Mesh> trailMesh;
    Shader bodyShader;
    Shader trailShader;

    void ensureGL();
    glm::vec3 rk4Step(const glm::vec3& x, float t, float h) const;
    void pushTrailPoint(const glm::vec3& p);
};

} // namespace engine

#endif
