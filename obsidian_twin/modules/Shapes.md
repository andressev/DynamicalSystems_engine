---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/Shapes.h
---
# Shapes

`engine/include/engine/Shapes.h` (header-only)

## What it is
Procedural geometry generators returning `std::vector`s that feed straight into
[[Mesh]]. (The old `custom/shapes.h` returned raw `new[]` arrays that leaked — fixed
here.)

## Dependencies
- Engine: none (output shaped for [[Mesh]])
- External: glm

## How to use it
Attach = call a generator pair in `init` and hand the vectors to a [[Mesh]]:
- `sphereVertices(sectors, stacks, radius)` + `sphereIndices(...)` — pos/normal/uv,
  layout `{3,3,2}`. Poles on the Z axis; [[riemann-sphere]] rotates them onto Y with its
  model matrix.
- `axisVertices(extent = 500)` + `axisIndices()` — 3 axis lines, layout `{3}`, draw with
  `GL_LINES`.
- `planeVertices()` + `planeIndices()` — unit XY quad, layout `{3}`.

## Notes
- Header-only; new generators go in the same header until it earns a split.
