---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/Camera.h
  - engine/src/Camera.cpp
---
# Camera

`engine/include/engine/Camera.h`, `engine/src/Camera.cpp`

## What it is
Orbit/free camera plus the projection parameters — view **and** projection come from
here so no other code hand-builds matrices (see [[render-pipeline]]). The one shared
camera is owned by [[App]]; sims configure it rather than construct it.

## Dependencies
- Engine: none
- External: glm, glfw (input polling)

## How to use it
Attach = configure the shared instance in `init` via `app.camera()`, then read matrices
in `render`:
1. In `init`: set `mode`, `orbitRadius`, and constraints to frame your scene.
2. In `render(const Camera& camera)`: `camera.getViewMatrix()`,
   `camera.getProjectionMatrix()` into your shader uniforms.
3. Reference sim: [[riemann-sphere]] (starts at radius 7, constrained to the sphere).

Knobs:
- `mode` — `Orbit` (spherical coords around `orbitCenter`, WASD rotate, Q/E zoom) or
  `Free` (WASD translate; rotation TODO).
- `processInput(GLFWwindow*, float dt)` — called by [[App]], dt from its single clock.
- `fovDeg` (55), `aspect` (set by App each frame), `nearPlane`, `farPlane`.
- `orbitRadius` (3) — current orbit distance; sims may set it in `init()`.
- `orbitRadiusMin` (0.1), `orbitSlowZone` (0 = off) — orbit constraints: the radius
  never goes below `orbitRadiusMin`, and within `orbitSlowZone` of it all orbit movement
  scales down proportionally (10% floor). [[riemann-sphere]] uses them to keep the
  camera outside the sphere and slow it near the surface.

## Notes
- Orbit phi is clamped to (0.05, π−0.05) to avoid the `lookAt` up-vector singularity at
  the poles.
- Orbit radius is clamped to `orbitRadiusMin` every frame (not just on Q presses).
