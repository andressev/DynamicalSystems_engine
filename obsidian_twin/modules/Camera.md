# Camera

`engine/include/engine/Camera.h`, `engine/src/Camera.cpp`

Orbit/free camera plus the projection parameters — view **and** projection come from here
so no other code hand-builds matrices (see [[render-pipeline]]).

## Interface
- `mode` — `Orbit` (spherical coords around `orbitCenter`, WASD rotate, Q/E zoom) or
  `Free` (WASD translate; rotation TODO).
- `processInput(GLFWwindow*, float dt)` — dt supplied by [[App]] (single clock).
- `getViewMatrix()`, `getProjectionMatrix()`.
- `fovDeg` (55), `aspect` (set by App each frame), `nearPlane`, `farPlane` — public knobs.
- `orbitRadius` (3) — current orbit distance; sims may set it in `init()` to frame their
  scene ([[riemann-sphere]] starts at 7).
- `orbitRadiusMin` (0.1), `orbitSlowZone` (0 = off) — orbit constraints, set by sims in
  `init()`: the radius never goes below `orbitRadiusMin`, and within `orbitSlowZone` of it
  all orbit movement scales down proportionally (10% floor). [[riemann-sphere]] uses them
  to keep the camera outside the sphere and slow it near the surface.

## Invariants
- Orbit phi is clamped to (0.05, π−0.05) to avoid the `lookAt` up-vector singularity at the poles.
- Orbit radius is clamped to `orbitRadiusMin` every frame (not just on Q presses).
