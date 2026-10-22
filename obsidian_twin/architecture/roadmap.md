# Roadmap

## Now
<!-- Max ~5 lines. Current milestone, what's mid-flight, next step. Update as you go —
     this is the first thing read at session start ([[Home]] routing). -->
- **Manifold growth (started 2026-10-18)** — [[lorenz-manifold-growth]]: hover a bead →
  popup → grow $W_{loc}$, then grow $W$: **done for curves** ($W^u(O)$, $W^s(C_\pm)$);
  **surfaces from stored level sets** ([[0011-precomputed-level-set-surfaces]]): $W^s(O)$
  at the classic parameters grows band by band to $\eta = 97$; $W^u(C_\pm)$ not stored yet.
- **Geodesic level sets (started 2026-10-18)** — built step by step, *as the user
  specifies the method*, in the walkthrough sim [[learn-geodesic-level-sets]] (entered
  from the growth popup). Done: the BVP as in the paper (continuation in $\tau$,
  [[geodesic-bvp]]), Solve all r, Accept ring (ring → new edge): three rings grown by
  hand. Since then: Grow rings with the §3.2 $\Delta_i$ adaptation, bands light / dark
  blue, **Save surface** → $W^s(O)$ stored (98 rings, $\eta = 97$, stops at a fold).
  **Next: as the user specifies** — section below.
- **Hartman–Grobman track (started 2026-10-16)** — see the section below. M0 done
  ([[ArrowGlyphs]]), M1 done ([[Eigen3]] + [[InvariantManifold]], checked by the new
  `tests/math_tests` exe), plus a GPU pass ([[0008-gpu-work-split-vertex-stage-and-cuda]]):
  instanced [[ArrowGlyphs]], parallel glyph sampling, [[ConjugacyMap]] (deformation in
  GLSL, previewed in [[lorenz-manifold]]), parallel [[lorenz-manifold]] rebuild (Taylor
  rings, Phase 2 shooting per point, mesh assembly). M2 done: [[lorenz-flow]] + the shared
  [[LorenzSystem]] module. M3 done: click on a bead → zoom → handoff to the new
  [[lorenz-equilibrium]] (glass ball $V$, everything clipped to it) and back, via
  `App::requestSwitch` ([[0009-sim-handoff-requestswitch]]). M4 done: stage A in
  [[lorenz-equilibrium]] ($E^s$/$E^u$ through $v^*$, $f$ vs $A(v-v^*)$ arrows, HUD,
  hyperbolicity warning); singles can run on [[TaylorIntegrator]]. M5 done: stage B
  (ring in $E^s$: $\psi^t$, $\varphi^t$ from $E^s$, $\varphi^t$ from $W^s$ via $K_s$,
  links + formula check). M6 done: stage C (section Hartman–Grobman: $U$ and
  $V = v^* + \tilde h^{-1}(U)$ via [[ConjugacyMap]], paired orbits that dissolve together,
  and a radius check — invariance defect + folding — with the safe radius; crashes with
  shockwaves via [[FrameCopy]]; "Transform flow" carries the linear picture onto $V$ on
  the GPU). Tried and dropped: a full conjugacy $H$ by a cross-section — not continuous
  at $C_\pm$ (gap on $W^u$) nor at $O$ (blow-up near $E^s$ along the fast stable
  direction); a real one needs the stable/unstable foliations. **Finished 2026-10-18**:
  Transform flow toggles U ↔ V (always 400 seeds), **T** sends a ghost of the pairs'
  $\psi^t$ across (orange → green), section **Linear ball U** ($E^s$/$E^u$ in $U$,
  disks of pairs on them, Transform subspaces onto $K(E^s)$/$K(E^u)$ with glowing
  tangent miniatures $T_{v^*}W^s(v^*)$), $W^s$/$W^u$ in $V$, Camera on U / V,
  $\bar 0$ at $U$'s centre; App: panel auto-pull at the top-left corner, F11 borderless
  full screen ([[Window]]). Refraction through the glass stays open.
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
  [[diagonal-linear]] shows $\dot{v} = Av$ with the live $A$ top-right
  ([[0007-latex-labels-microtex-imgui]]). Next: $\varphi^t(v_0)$ riding a trajectory
  head (needs a cheap per-frame path), $\dot{v} = f(v)$ from [[Expression]].
- [[vector-field]] rebuilt (2026-10-11) in [[diagonal-linear]]'s style: typed field
  typeset via the new `Expression::latex`, $\phi^t(v_0)$ labels, ball/disk + singles,
  R2/R3 each with its own saved settings; R3 follow-a-single camera and red
  $\dot{v}$ tangent arrows on singles. Dropped: chase-cam ride, glyph distance fade.
- Next: marching-cubes mesh sim (M1 of the level-set proposal), then CUDA (M4) — where
  glyph sampling and many-seed advection are the first kernels; symbolic ∇f in
  [[Expression]] stays the highest-leverage enabler.

## Geodesic level sets track (started 2026-10-18)
Handoff for the next session — read [[learn-geodesic-level-sets]] (the sim note, every
mechanism with its function names) and the "One step" paragraph of
[[global-manifold-growth]] (the method as the user specified it) first.
- **Ground rule:** the user re-reads the algorithm (Krauskopf & Osinga) and specifies
  each step; implement what they describe, don't invent the method. Ask when a step is
  ambiguous.
- **Where things live:** `sims/learn_geodesic_level_sets/LearnGeodesicLevelSetsSim.cpp`
  (one file, ~2300 lines). Patch + mesh: `rebuild`, `buildMesh`, `spreadEdge`
  (maxSpread / minSpread rule on the edge only; inner rings stay uniform in $\theta$).
  Edge point: `chooseEdge`, `edgeFrame` ($r$, $r^\pm$, $n$, `out`, `across`),
  `renderEdgePoint`. Particles: `launchAt`, `shootFromR` (V), `shootRing` (B), `findHit`
  ($b_r(\tau)$, stops there), `renderHits`. BVP: `edgePointD`, `makeAnchor`,
  `flowFrom` (hand-stepped Taylor for exactly $\tau$), `correct`, `continueTau`,
  `bisectTau`, `pointAt`, `solveAnchor`, `solveAt` (double-click), Solve all r
  (`startSolveAll`, a slice per frame), `recutRing` ($\Delta_i$ = min, mids),
  `renderBand`; rings: `acceptRing`, `ringSpan` / `outwardAt` (the edge as a polyline
  past layer 0), `renderLayers`. View: `viewFrame` (own orbit frame, screen-up in $\mathcal{F}_r$).
- **State:** rings grow by hand — Solve all r, then Accept ring, repeated; at the
  classic parameters three rings at $\Delta_i = 1$ (0.5 s, then ~0.1 s per ring in
  debug), the z-axis vertices exact. Not yet exercised: broken branches,
  $\Delta_i < \Delta_{\min}$.
- **Likely next steps** (confirm with the user): the $\Delta_i$
  adaptation between $\Delta_{\min}$ and $\Delta_{\max}$ (the method's accuracy
  criteria); then hand the grown surface back to [[lorenz-manifold-growth]] so its
  surface `Grow W` comes alive.
- **Conventions here:** particles run in the manifold's own direction of time
  (backward on $W^s$); $\tau$ is signed ($< 0$ backward). The explorer locks its exe
  while open (LNK1168): standalone builds still link.
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
(ball $V = B_r(v^*)$, stages A/B/C) through a new deferred `App::requestSwitch` + a
key→double handoff map; the shared [[Camera]] makes the switch seamless (needs an ADR).
Conjugacy only on the invariant manifolds, via the parametrization method
([[lorenz-stable-manifold]]) generalized to any equilibrium and complex pairs; $V$ is the
sphere pushed through $v^* + K_s(\theta_s) + K_u(\theta_u)$ — exact on $E^s$/$E^u$,
an interpolation elsewhere. Manifold surfaces themselves are *not* drawn yet (later).
Orbits leaving $U$/$V$ dissolve on the rim, both at the same instant.
- M0 `ArrowGlyphs` module (done) · M1 `Eigen3` + `InvariantManifold` (done; std-only, checked
  against [[lorenz-manifold]]'s origin coefficients + an invariance-error test) ·
  M2 [[lorenz-flow]] + [[LorenzSystem]] (done) · M3 handoff + zoom + glass ball/clip (done) · M4 (done) stage A ($E^s$/$E^u$,
  $f$ vs $A(v-v^*)$, HUD, non-hyperbolic warning) · M5 (done) stage B ($\varphi^t$ vs
  $\psi^t$ from $E^s$) · M6 (done) stage C (U and deformed V, $h^{-1}$ links, dissolve) ·
  M7 refraction (`RenderTarget`), polish, notes. · Next track: [[lorenz-manifold-growth]]
  (grow $W_{loc}$ → $W$ per equilibrium; surfaces by geodesic level sets later).
- Risk: $C_\pm$ at $\rho = 28$ is near the Hopf point ($\rho_H \approx 24.74$,
  $\lambda \approx 0.094 \pm 10.19i$) → near-resonant small divisors, small valid
  radius for $K_u$ (measured: rel. defect $6\cdot10^{-5}$ at $|\theta|=2$, $0.44$ at 4);
  the Hartman–Grobman radius slider stops at the measured safe radius (O 12.0,
  C± 2.49).

## M4 — CUDA interop
`cudaGraphicsGLRegisterBuffer` on [[Mesh]]'s VBO (exposed via `Mesh::vbo()`) for
GPU-side particle advection of flows. This is why all vertex buffers live in one class.

## Later — manifold morph (parked 2026-10-20)
Blend stored surfaces ([[LevelSetSurface]]) of different parameters in
[[lorenz-manifold-growth]]. Correspondence from the method itself: label every point by
$(\eta, \theta_0)$ — $\theta_0$ its angle on the $W_{loc}$ circle, inherited by $r_{new}$
from its BVP start, averaged for points inserted at accept — store a resampled
$(\eta, \theta_0)$ grid, save snapshots (e.g. $\rho = 24, 26, 28$), and a $\rho$ slider
mixes the two nearest per vertex on the GPU. A picture, not the true manifold between
snapshots: fine when they are close, misleading across bifurcations (homoclinic
explosion at $\rho \approx 13.93$); $\theta_0$ holds only while rings don't fold.

## Later — $W^u(C_\pm)$ by level sets (parked 2026-10-20)
Tried in [[learn-geodesic-level-sets]]: at $\rho = 28$ four rings ($\eta = 4$), then "r_new
grows backward" ($\alpha_r = \pi$); at $\rho = 60$ it fails on ring 1. Cause: the first
edge is a round circle in $E^u$, but the complex pair ($0.094 \pm 10.2i$) rotates along
tilted ellipses, so the linear flow points inward on ~49% of it (41% at $\rho = 60$).
Proposed fix: for a complex pair start from $|s| = c$ in $K$'s parameter coordinates
(flow $s \mapsto e^{\lambda t}s$, uniform expansion) — an ellipse in $E^u$; $W^s(O)$
unchanged. Check against Krauskopf & Osinga first. $W^u(C_-)$ is then the mirror of
$W^u(C_+)$ under $(x,y,z) \mapsto (-x,-y,z)$.

## Later — big balls (parked 2026-10-16)
200 particles looks right for now; the ball slider caps at 1000. To go to ~10k:
- batch trails: one shared buffer + `glMultiDrawArrays`, per-trail color/alpha as an
  attribute, heads instanced (2000 draw calls → ~2; measured ~5 ms CPU submit at 1000);
- integrate on the GPU (compute shader or CUDA, M4), writing straight into that buffer —
  no upload; [[Expression]]'s GLSL codegen covers typed fields. Float, not double, so
  paths stop being bit-identical to the CPU ones;
- a trail-point budget (mirrored ring ≈ 70 KB per particle at 3000 points).
Numbers: [[TrajectoryRunner]].
