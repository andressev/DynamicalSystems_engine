# Roadmap

## Now
<!-- Max ~5 lines. Current milestone, what's mid-flight, next step. Update as you go —
     this is the first thing read at session start ([[Home]] routing). -->
- Between M2 and M3: module/vault system just restructured ([[0004-modules-vs-simulations]],
  [[0005-vault-as-ai-navigation]]); nothing mid-flight in code.
- Next: start M3 — `Integrator` (math layer, via `/new-module`), then `VectorFieldSim`.

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
