---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/Mesh.h
  - engine/src/Mesh.cpp
---
# Mesh

`engine/include/engine/Mesh.h`, `engine/src/Mesh.cpp`

## What it is
Resource wrapper around one VAO + VBO (+ EBO). Non-copyable, movable, deletes GL objects
in destructor. **The single place vertex buffers live on the GPU** — the planned CUDA
interop (M4 in [[roadmap]]) registers the buffer returned by `vbo()` with
`cudaGraphicsGLRegisterBuffer`.

Two flavors: the **static indexed** mesh (vertices + indices, `GL_STATIC_DRAW`, drawn
with `glDrawElements`) and the **dynamic index-less** one (a reserved capacity written
piecemeal, `GL_DYNAMIC_DRAW`, drawn with `glDrawArrays`) for geometry that changes at
runtime — growing trajectories, glyph grids re-sampled every frame
([[0006-cpu-field-sampling-dynamic-glyph-mesh]]).

## Dependencies
- Engine: none (typically fed by [[Shapes]])
- External: glad

## How to use it
Attach = hold a `std::unique_ptr<engine::Mesh>` member, create in `init`:
1. `Mesh(vertices, indices, layout)` — interleaved float data; `layout` is floats per
   attribute in location order, e.g. `{3, 3, 2}` for pos/normal/uv ([[Shapes]] sphere)
   or `{3}` for plain positions.
2. In `render`: bind your [[Shader]], then `mesh->draw(mode = GL_TRIANGLES)` — binds the
   VAO and `glDrawElements`.
3. Reference sim: [[riemann-sphere]] (sphere + axis lines).

**Dynamic geometry** instead:
1. `Mesh(vertexCapacity, layout)` — reserves the buffer, uploads nothing, no index buffer.
2. `setVertices(data, floatCount, floatOffset = 0)` — `glBufferSubData` into it. A write
   that would cross the capacity is dropped whole, so callers must size for the worst
   case (or reallocate by move-assigning a bigger `Mesh`, which is what the glyph grid
   in [[vector-field]] does when the grid grows).
3. `drawArrays(mode, vertexCount, first = 0)` — draw only the part currently filled.
4. Reference users: [[Trajectory]] (one vertex appended per integration step), and
   `rebuildArrows()` in [[vector-field]] (whole buffer rewritten per rebuild).

**Instanced drawing** (either flavor):
1. `setInstanceLayout(instanceCapacity, layout)` — adds a dynamic per-instance buffer
   whose attributes take the locations *after* the vertex layout's, with divisor 1.
   Calling it again replaces (and can grow) the buffer.
2. `setInstances(data, floatCount)` — write the records (over-capacity writes dropped).
3. `drawInstanced(mode, instanceCount, vertexCount = -1)` — indexed when the mesh has
   indices, else over `vertexCount` vertices.
4. Reference user: [[ArrowGlyphs]] (one static cone/shaft shape, one record per arrow).

Also: `indexCount()`, `vertexCapacity()`, `instanceCapacity()`, `vbo()` (the CUDA
interop point).

## Notes
- No raw `glGen*`/`glDelete*` in sims — GL buffer ownership lives here and in [[Shader]]
  only.
- The two constructors are exclusive: a dynamic mesh has no EBO, so `draw()` (which is
  `glDrawElements`) must not be used on it, and `setVertices`/`drawArrays` are
  meaningless on a static one (`capacity` is 0, so `setVertices` silently does nothing).
- Attribute layout is fixed at construction for both flavors; changing it means a new
  `Mesh`. The instance layout is the exception — `setInstanceLayout` may be re-called.
- One instance buffer per mesh: two base meshes drawing the same instances (shafts and
  heads in [[ArrowGlyphs]]) each upload their own copy — a few bytes per instance.
- No base-instance draws: the context is GL 3.3, so drawing a sub-range of instances
  means a separate batch (what [[diagonal-linear]] does per subspace).
