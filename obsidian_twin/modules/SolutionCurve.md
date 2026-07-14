---
type: module
layer: gfx
status: wip
code:
  - engine/include/engine/SolutionCurve.h
  - engine/src/SolutionCurve.cpp
---
# SolutionCurve

`engine/include/engine/SolutionCurve.h`, `engine/src/SolutionCurve.cpp`

## What it is
A CPU-integrated particle tracing a solution curve of $\dot{x} = V(x, t)$: give it a
vector field as a `std::function`, it advances with fixed-step RK4 and draws itself as
a small lit sphere plus an optional line-strip trail of the curve so far. Works in 3D
or 2D (keep $z = 0$ in the field). The field is arbitrary — ambient, or a
tangent-projected surface field for flows on manifolds.

## Dependencies
- Engine: [[Mesh]] (body sphere + dynamic trail via `updateData`), [[Shader]]
  (two small embedded programs), [[Shapes]] (sphere geometry)
- External: glm

## How to use it
Attach = declare a member (GL state is created lazily on first draw, so no `init` work
is required):
1. `engine::SolutionCurve particle;` as a sim member.
2. Configure: `particle.field = [](const glm::vec3& p, float t){ ... };`,
   `particle.reset(startPos)`; knobs are public members — `stepSize` (RK4 $h$),
   `trailEnabled`, `trailMax`, `bodyRadius`, `bodyColor`, `trailColor`.
3. In `update(dt)`: `particle.update(dt, simTime)` — takes as many $h$-steps as fit,
   carries the remainder (capped per frame so tiny steps can't stall the app).
4. In `render`: `particle.draw(view, projection)`.
5. Reference sim: [[surface-renderer-ray-marched]] (flow-demo particle riding the
   raymarched level set via tangent projection + feedback retraction).

## The math
Classic RK4: $x_{n+1} = x_n + \tfrac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)$ with the four
stage evaluations of the field at $t$, $t+\tfrac{h}{2}$, $t+h$. For surface-constrained
flows, the *field* encodes the constraint (tangent projection plus a $-\lambda F\,
\nabla f / |\nabla f|^2$ feedback term) — the module stays a plain integrator.

## Notes
- The trail re-uploads its vertex buffer only on frames where points were added
  ([[Mesh]] `updateData`, dynamic draw).
- Not copyable (owns GL via [[Mesh]]/[[Shader]]); movable through its members.
- Future: custom body model (spaceship etc.) — needs a model loader (prefer
  glTF/OBJ over FBX; assimp if vendored) and a `setBodyMesh` hook; sphere is the
  default regardless. Many-particle version belongs to CUDA (M4 in [[roadmap]]).
