# Roadmap

## M1 — foundation (done, 2026-07)
Engine static lib ([[App]], [[Window]], [[Sim]], [[SimRegistry]], [[Camera]], [[Shader]],
[[Mesh]], [[Shapes]]), ImGui, hybrid explorer + standalone executables,
[[riemann-sphere]] ported as the first sim.

## M2 — function parser (done, 2026-07)
[[Expression]]: tokenizer + recursive-descent parser → AST → CPU eval
(`std::complex<double>`) + GLSL codegen, spliced into a fragment template and
hot-recompiled via `Shader::fromSources`. Live function box in [[riemann-sphere]] with
`z`, `t` (time) and `m` (mouse) variables. Decision record:
[[0003-hand-rolled-parser-glsl-codegen]].

## M3 — ODE / vector fields
- RK4 integrator over the same AST evaluated on CPU; add a fast pure-real
  `evalReal` to [[Expression]] (real functions of up to ~3 variables).
- `VectorFieldSim`: arrow glyphs / streamlines for `dx/dt = f(x, y), dy/dt = g(x, y)`.
- Flows on parametric manifolds (sphere first — reuse [[Shapes]] sphere + [[riemann-sphere]] math).

## M4 — CUDA interop
`cudaGraphicsGLRegisterBuffer` on [[Mesh]]'s VBO (exposed via `Mesh::vbo()`) for
GPU-side particle advection of flows. This is why all vertex buffers live in one class.
