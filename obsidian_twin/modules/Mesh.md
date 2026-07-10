# Mesh

`engine/include/engine/Mesh.h`, `engine/src/Mesh.cpp`

Resource wrapper around one VAO + VBO + EBO. Non-copyable, movable, deletes GL objects in
destructor. **The single place vertex buffers live on the GPU** — the planned CUDA
interop (M4 in [[roadmap]]) registers the buffer returned by `vbo()` with
`cudaGraphicsGLRegisterBuffer`.

## Interface
- `Mesh(vertices, indices, layout)` — interleaved float data; `layout` is floats per
  attribute in location order, e.g. `{3, 3, 2}` for pos/normal/uv ([[Shapes]] sphere)
  or `{3}` for plain positions.
- `draw(mode = GL_TRIANGLES)` — binds the VAO and `glDrawElements`.
- `indexCount()`, `vbo()`.
