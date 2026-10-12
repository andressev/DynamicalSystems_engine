---
type: sim
status: wip
code:
  - sims/lorenz_manifold/LorenzManifoldSim.cpp
---
# lorenz-manifold

`sims/lorenz_manifold/` — lib `sim_lorenz_manifold`, standalone exe
`lorenz_manifold_standalone`.

Renders the 2-D stable manifold $W^s(0)$ of the Lorenz system as concentric geodesic
rings, built as a Taylor-series patch (no ODE integration) — the "Lorenz manifold."

## What is being simulated
The set of points that flow into the Lorenz system's origin equilibrium under its
forward dynamics: a curved 2-D surface through the origin, tangent to the plane
spanned by the two stable eigendirections of the Jacobian there. The rings $C_i$ are
approximate geodesic circles at increasing distance from the origin, all evaluated
straight from one Taylor map $K$ — no simulation runs, this is a static (but
parameter-tunable) surface, unlike the flow sims [[roadmap]] M3 will add.

## How it works
`init` sets up the orbit camera, enables `GL_PROGRAM_POINT_SIZE`, loads the point/line
shader pair, and calls `rebuild()`. `rebuild()` runs `buildManifold` (see
[[lorenz-stable-manifold]] for the math) entirely in `double` on the CPU: computes the
Jacobian eigendata, solves the coefficient recursion for `p[3][order+1][order+1]`,
then evaluates `K` on `N` concentric rings, each starting from `M` points (uniform in
psi, or equal-arc resampled) and then optionally refined by `refineRingSpacing`
(`adaptive spacing`): any edge whose chord length exceeds `maxGap` is bisected at its
exact midpoint psi through `K` (not lerped), repeating until every edge is under the
cap or the ring hits the 4000-point safety cap. Ring point counts are independent —
outer rings that need more points to stay under `maxGap` get them without inflating
the (small) inner rings. The rings are cast to `float` and uploaded as one flat `Mesh`
(all points, `GL_POINTS`), one small `Mesh` per ring (`GL_LINE_LOOP`, for visible ring
structure), and one solid-surface `Mesh` (`GL_TRIANGLES`, built by `buildSurface`) that
fills the gaps between rings. Any parameter change in `drawUI` calls `rebuild()` again,
replacing the `Mesh` objects outright (no in-place VBO update — `Mesh` has none).

`buildSurface` stitches each consecutive ring pair into a triangle strip by walking
both rings' `psi` values together (advance whichever ring's next vertex has the
smaller psi) — since both rings share the same angular parameter, this stays correct
even when `adaptive spacing` gives them different point counts, unlike a naive
equal-index quad grid. A fan from an explicit origin vertex (`K(0,0) = 0` exactly) to
the innermost ring closes the tip.

## Modules attached
- [[Mesh]] — one points mesh (all rings flattened), one line-loop mesh per ring, and
  one solid-surface triangle mesh; all rebuilt (not updated) whenever a build
  parameter changes.
- [[Shader]] — one small point/line shader pair, loaded once from files.
- [[Camera]] — orbit camera; `init` sets `orbitRadius`/`orbitRadiusMin` to frame the
  manifold near the origin.

## The math
See [[lorenz-stable-manifold]] for the parametrization-method derivation, the
coefficient recursion, and the isotropic-coordinate rescaling. This sim is Phase 1
(Taylor-only) of that note's plan; Phase 2 (flow-based growth past the convergence
disk) is scoped there but not implemented.

## Controls / uniforms
- `sigma`, `rho`, `beta` — Lorenz parameters (default 10, 28, 8/3, the classic chaotic
  regime).
- `order` — Taylor truncation order (default 10; higher = larger convergence disk, more
  compute per rebuild).
- `rings (N)`, `points/ring (M)`, `dRho` — mesh resolution and radial step. Default
  `dRho=0.4`, `N=15` reaches parameter-space radius 6, checked numerically to stay
  within ~3 significant digits of an order-20 truncation out past radius 10 — an
  earlier `dRho=0.05` default left the whole patch under a unit across.
- `equal-arc redistribution` — toggles step-5 resampling (chord-length arc-length
  remeshing) vs. raw uniform-$\psi$ points, as the base ring `adaptive spacing`
  refines from.
- `adaptive spacing` / `max gap` — when on, inserts extra points (via bisection
  through $K$) on any ring edge longer than `max gap`, so rings at large $\rho_i$
  automatically get denser than $M$ without raising $M$ globally.
- `model scale` — uniform scale on the `model` matrix, since Lorenz coordinates run
  much larger than the rest of the scene.
- `show surface` / `show points` / `show rings`, `color by height (z)` — render
  toggles; color defaults to a red → blue gradient by ring level (red nearest the
  origin, blue the furthest-out ring), or by $z$ when the height toggle is on. The
  solid surface draws first so the point/ring wireframe sits visibly on top of it.
- Uniforms: `model`, `view`, `projection`, `u_colorMode` (0 = level, 1 = height).

## Future
Phase 2 flow-based growth past the Taylor disk (RK4 on the reversed field, Newton
shooting for the next geodesic ring) — see [[lorenz-stable-manifold]]. Once
[[Expression]] grows a real-vector-field path this could share plumbing with the M3
`VectorFieldSim`/`Integrator` work in [[roadmap]].
