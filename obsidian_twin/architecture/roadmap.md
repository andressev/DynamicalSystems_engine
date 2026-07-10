# Roadmap

## M1 — foundation (done, 2026-07)
Engine static lib ([[App]], [[Window]], [[Sim]], [[SimRegistry]], [[Camera]], [[Shader]],
[[Mesh]], [[Shapes]]), ImGui, hybrid explorer + standalone executables,
[[riemann-sphere]] ported as the first sim.

## M2 — function parser
Tokenizer + recursive-descent parser → AST → GLSL codegen: emit
`vec2 f(vec2 z) { ... }` from a typed expression like `z^2 + sin(z)`, splice into a
fragment-shader template, hot-recompile via `Shader::fromSources` (already implemented
for this). ImGui text box in the Riemann sim. Decision record:
[[0003-hand-rolled-parser-glsl-codegen]].

## M3 — ODE / vector fields
- RK4 integrator over the same AST evaluated on CPU.
- `VectorFieldSim`: arrow glyphs / streamlines for `dx/dt = f(x, y), dy/dt = g(x, y)`.
- Flows on parametric manifolds (sphere first — reuse [[Shapes]] sphere + [[riemann-sphere]] math).

## M4 — CUDA interop
`cudaGraphicsGLRegisterBuffer` on [[Mesh]]'s VBO (exposed via `Mesh::vbo()`) for
GPU-side particle advection of flows. This is why all vertex buffers live in one class.
