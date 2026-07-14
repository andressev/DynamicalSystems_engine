# Roadmap

## Now
<!-- Max ~5 lines. Current milestone, what's mid-flight, next step. Update as you go —
     this is the first thing read at session start ([[Home]] routing). -->
- Level-set track opened (2026-07-13): method survey written
  (`docs/level-set-meshing-proposal.html`), [[surface-renderer-ray-marched]] added as
  the raymarched reference view. [[riemann-sphere]] gained a flat plane mode.
- Next on this track: marching-cubes mesh sim (the proposal's M1), then CUDA (M4);
  symbolic ∇f in [[Expression]] is the highest-leverage enabler.
- M3 effectively started: [[SolutionCurve]] module (RK4 particle + trail, gfx layer)
  added, demoed as a surface flow in [[surface-renderer-ray-marched]] (tangent
  projection + retraction feedback over CPU `Expression::eval`). `evalReal` +
  symbolic ∇f in [[Expression]] remain the enablers for many-particle flows;
  `VectorFieldSim` still pending.

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
