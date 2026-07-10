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

## Invariants
- Orbit phi is clamped to (0.05, π−0.05) to avoid the `lookAt` up-vector singularity at the poles.
