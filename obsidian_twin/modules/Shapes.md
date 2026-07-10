# Shapes

`engine/include/engine/Shapes.h` (header-only)

Procedural geometry generators returning `std::vector`s that feed straight into [[Mesh]].
(The old `custom/shapes.h` returned raw `new[]` arrays that leaked — fixed here.)

## Generators
- `sphereVertices(sectors, stacks, radius)` + `sphereIndices(...)` — pos/normal/uv,
  layout `{3,3,2}`. Poles on the Z axis; [[riemann-sphere]] rotates them onto Y with its
  model matrix.
- `axisVertices(extent = 500)` + `axisIndices()` — 3 axis lines, layout `{3}`, `GL_LINES`.
- `planeVertices()` + `planeIndices()` — unit XY quad, layout `{3}`.
