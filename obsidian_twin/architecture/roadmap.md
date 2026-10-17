# Roadmap

## Now
<!-- Max ~5 lines. Current milestone, what's mid-flight, next step. Update as you go —
     this is the first thing read at session start ([[Home]] routing). -->
- **Hartman–Grobman track (started 2026-10-16)** — see the section below. M0 done
  ([[ArrowGlyphs]]), M1 done ([[Eigen3]] + [[InvariantManifold]], checked by the new
  `tests/math_tests` exe), plus a GPU pass ([[0008-gpu-work-split-vertex-stage-and-cuda]]):
  instanced [[ArrowGlyphs]], parallel glyph sampling, [[ConjugacyMap]] (deformation in
  GLSL, previewed in [[lorenz-manifold]]), parallel [[lorenz-manifold]] rebuild (Taylor
  rings, Phase 2 shooting per point, mesh assembly). M2 done: [[lorenz-flow]] + the shared
  [[LorenzSystem]] module. M3 done: click on a bead → zoom → handoff to the new
  [[lorenz-equilibrium]] (glass ball $V$, everything clipped to it) and back, via
  `App::requestSwitch` ([[0009-sim-handoff-requestswitch]]). M4 done: stage A in
  [[lorenz-equilibrium]] ($E^s$/$E^u$ through $x^*$, $f$ vs $A(x-x^*)$ arrows, HUD,
  hyperbolicity warning); singles can run on [[TaylorIntegrator]]. M5 done: stage B
  (ring in $E^s$: $\psi^t$, $\varphi^t$ from $E^s$, $\varphi^t$ from $W^s$ via $K_s$,
  links + formula check). M6 done: stage C (section Hartman–Grobman: $U$ and
  $V = x^* + \tilde h^{-1}(U)$ via [[ConjugacyMap]], paired orbits that dissolve together,
  and a radius check — invariance defect + folding — with the safe radius; crashes with
  shockwaves via [[FrameCopy]]; "Transform flow" carries the linear picture onto $V$ on
  the GPU). Tried and dropped: a full conjugacy $H$ by a cross-section — not continuous
  at $C_\pm$ (gap on $W^u$) nor at $O$ (blow-up near $E^s$ along the fast stable
  direction); a real one needs the stable/unstable foliations. Next: M7 — refraction
  through the glass, polish.
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
- [[vector-field]] rebuilt (2026-10-11) in [[diagonal-linear]]'s style: typed field
  typeset via the new `Expression::latex`, $\phi^t(x_0)$ labels, ball/disk + singles,
  R2/R3 each with its own saved settings; R3 follow-a-single camera and red
  $\dot{x}$ tangent arrows on singles. Dropped: chase-cam ride, glyph distance fade.
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

## Hartman–Grobman on Lorenz (in progress, 2026-10)
Two sims, kept separate so neither carries the other's UI: `lorenz_flow` (global field,
red equilibria, hover glow, click → eased zoom) hands off to `lorenz_equilibrium`
(ball $V = B_r(x^*)$, stages A/B/C) through a new deferred `App::requestSwitch` + a
key→double handoff map; the shared [[Camera]] makes the switch seamless (needs an ADR).
Conjugacy only on the invariant manifolds, via the parametrization method
([[lorenz-stable-manifold]]) generalized to any equilibrium and complex pairs; $V$ is the
sphere pushed through $x^* + K_s(\theta_s) + K_u(\theta_u)$ — exact on $E^s$/$E^u$,
an interpolation elsewhere. Manifold surfaces themselves are *not* drawn yet (later).
Orbits leaving $U$/$V$ dissolve on the rim, both at the same instant.
- M0 `ArrowGlyphs` module (done) · M1 `Eigen3` + `InvariantManifold` (done; std-only, checked
  against [[lorenz-manifold]]'s origin coefficients + an invariance-error test) ·
  M2 [[lorenz-flow]] + [[LorenzSystem]] (done) · M3 handoff + zoom + glass ball/clip (done) · M4 (done) stage A ($E^s$/$E^u$,
  $f$ vs $A(x-x^*)$, HUD, non-hyperbolic warning) · M5 (done) stage B ($\varphi^t$ vs
  $\psi^t$ from $E^s$) · M6 (done) stage C (U and deformed V, $h^{-1}$ links, dissolve) ·
  M7 refraction (`RenderTarget`), polish, notes.
- Risk: $C_\pm$ at $\rho = 28$ is near the Hopf point ($\rho_H \approx 24.74$,
  $\lambda \approx 0.094 \pm 10.19i$) → near-resonant small divisors, small valid
  radius for $K_u$ (measured: rel. defect $6\cdot10^{-5}$ at $|\theta|=2$, $0.44$ at 4);
  the $r$ slider shows the invariance error.

## M4 — CUDA interop
`cudaGraphicsGLRegisterBuffer` on [[Mesh]]'s VBO (exposed via `Mesh::vbo()`) for
GPU-side particle advection of flows. This is why all vertex buffers live in one class.

## Later — big balls (parked 2026-10-16)
200 particles looks right for now; the ball slider caps at 1000. To go to ~10k:
- batch trails: one shared buffer + `glMultiDrawArrays`, per-trail color/alpha as an
  attribute, heads instanced (2000 draw calls → ~2; measured ~5 ms CPU submit at 1000);
- integrate on the GPU (compute shader or CUDA, M4), writing straight into that buffer —
  no upload; [[Expression]]'s GLSL codegen covers typed fields. Float, not double, so
  paths stop being bit-identical to the CPU ones;
- a trail-point budget (mirrored ring ≈ 70 KB per particle at 3000 points).
Numbers: [[TrajectoryRunner]].
