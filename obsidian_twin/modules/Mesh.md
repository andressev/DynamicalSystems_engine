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
Resource wrapper around one VAO + VBO + EBO. Non-copyable, movable, deletes GL objects
in destructor. **The single place vertex buffers live on the GPU** — the planned CUDA
interop (M4 in [[roadmap]]) registers the buffer returned by `vbo()` with
`cudaGraphicsGLRegisterBuffer`.

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

Also: `indexCount()`, `vbo()` (the CUDA interop point).

## Notes
- No raw `glGen*`/`glDelete*` in sims — GL buffer ownership lives here and in [[Shader]]
  only.
