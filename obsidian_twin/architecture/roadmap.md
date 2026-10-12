# Roadmap

## Now
<!-- Max ~5 lines. Current milestone, what's mid-flight, next step. Update as you go —
     this is the first thing read at session start ([[Home]] routing). -->
- **M3 landed (2026-09-24)**: [[VectorField]] + [[Integrator]] + [[Trajectory]] modules
  and the [[vector-field]] sim — typed fields on R2/R3, arrow glyphs colored by
  magnitude, trajectories integrated in real time, camera-follow. [[Expression]] gained
  `evalReal`/`usesVariable`; [[Mesh]] gained dynamic buffers
  ([[0006-cpu-field-sampling-dynamic-glyph-mesh]]).
- Level-set track: [[surface-renderer-ray-marched]] is the raymarched reference view;
  [[lorenz-manifold]] Phase 1 done, Phase 2 (flow growth past the Taylor disk) can now
  reuse [[Integrator]]'s RK4 instead of its private one.
- [[diagonal-linear]] (wip): minimal R3 sibling of [[vector-field]] for diagonal linear
  systems — $E^s$/$E^u$ drawn in, balls of trajectories; next step is general $A$.
- LaTeX labels: [[LatexImage]] (MicroTeX layout, CPU raster → texture) is live —
  [[diagonal-linear]] shows $\dot{x} = Ax$ with the live $A$ top-right
  ([[0007-latex-labels-microtex-imgui]]). Next: $\varphi^t(x_0)$ riding a trajectory
  head (needs a cheap per-frame path), $\dot{x} = f(x)$ from [[Expression]].
- Next: marching-cubes mesh sim (M1 of the level-set proposal), then CUDA (M4) — where
  glyph sampling and many-seed advection are the first kernels; symbolic ∇f in
  [[Expression]] stays the highest-leverage enabler.

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

## M3 — ODE / vector fields (done, 2026-09)
- [[Expression]] `evalReal` (real walk of the same AST) + `usesVariable`; [[VectorField]]
  turns typed component strings into a real field on R2/R3.
- [[Integrator]]: fixed-step Euler/RK2/RK4 over any right-hand side, reverse-time capable
  ([[runge-kutta]]). [[Trajectory]]: ODE state + growing polyline in a dynamic [[Mesh]].
- [[vector-field]] sim: arrow glyphs colored by |F| ([[integral-curves]]), click-to-seed
  trajectories integrated live, camera follow, R2 phase-plane and R3 attractor modes.
- Still open on this track: streamlines/LIC, equilibrium + nullcline detection, and flows
  on parametric manifolds (sphere first — reuse [[Shapes]] sphere + [[riemann-sphere]]
  math; [[surface-renderer-ray-marched]] level sets once they are meshed).

## M4 — CUDA interop
`cudaGraphicsGLRegisterBuffer` on [[Mesh]]'s VBO (exposed via `Mesh::vbo()`) for
GPU-side particle advection of flows. This is why all vertex buffers live in one class.
