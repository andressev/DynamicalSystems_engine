---
type: sim
status: wip
code:
  - sims/diagonal_linear/DiagonalLinearSim.cpp
  - sims/diagonal_linear/shaders/vertex.glsl
  - sims/diagonal_linear/shaders/fragment.glsl
---
# diagonal-linear

`sims/diagonal_linear/` — lib `sim_diagonal_linear`, standalone exe `diagonal_linear_standalone`.

A minimal R3 phase portrait of a diagonal linear system, with its stable (blue) and
unstable (red) eigenspaces drawn in and balls of initial conditions dropped into the flow.

## What is being simulated
$\dot{\mathbf{x}} = \Lambda \mathbf{x}$ with $\Lambda = \mathrm{diag}(\lambda_1, \lambda_2, \lambda_3)$,
so the eigenvectors are the coordinate axes and the flow is
$x_i(t) = x_i(0)\, e^{\lambda_i t}$. The invariant subspaces split $\mathbb{R}^3$:
$E^s = \mathrm{span}\{e_i : \lambda_i < 0\}$, $E^u = \mathrm{span}\{e_i : \lambda_i > 0\}$,
$E^c = \mathrm{span}\{e_i : \lambda_i = 0\}$. A ball of seeds shows the flow acting on
volumes: it stretches along $E^u$, collapses onto $E^u$ as the $E^s$ components decay,
and its volume scales by $e^{(\lambda_1+\lambda_2+\lambda_3)t}$ (Liouville). A stripped-down
sibling of [[vector-field]], which was later rebuilt in this sim's style (same labels,
overlay and panel) for any typed field.

## How it works
`init` builds the axes, a small dynamic subspace mesh, and the right-hand side
$\Lambda\mathbf{x}$ as a lambda (no [[VectorField]] parse, the field is fixed in form).
`update` drops a ball on click, advances every [[Trajectory]] with RK4 ([[Integrator]]),
and rebuilds the arrows / subspaces when a $\lambda$ or glyph setting changed.

`rebuildSubspaces()` classifies each axis by the sign of $\lambda_i$ ($|\lambda_i| < 10^{-4}$
counts as center) and emits per subspace: its axis lines, plus a translucent square when it
is 2-dimensional, or a cube's edges when it is all of $\mathbb{R}^3$. Lines are drawn with
`GL_LEQUAL` so they win the depth tie with the grey axes; the planes go last without depth
writes so trails show through them. Along every stable/unstable half-axis it also bakes
shaded cones in the subspace's color pointing the way the flow runs — toward the origin
on $E^s$, away on $E^u$; $E^c$ is at rest and gets none. Each subspace also gets an $E^s$/$E^u$/$E^c$ label in its color
(a [[LatexImage]] typeset once), anchored `label position` (default 0.35) of the extent out
along its $+$ directions — on a line, its $+$ half; on a plane, the $(+,+)$ diagonal —
drawn up-right of the anchor so it never covers the line, projected every frame (`labels`).

Arrow glyphs are the [[vector-field]] construction, via [[ArrowGlyphs]] (solid cone or barbs, LOD by count,
baked facet shading), sampled exactly on a cube of `domain radius`.

Curves are one list of `Curve {traj, seed, color, kind}` with `Kind` one of `Ball`,
`Single`, `Manifold`. There is **one ball at a time**: `dropBall()` erases the `Ball`
curves and seeds a new white ball — a Fibonacci
lattice on the sphere (default, the clearest view of it becoming an ellipsoid) or uniform
points through the volume. A click drops a **single** trajectory, always green — clear of
red/blue (subspaces) and white (ball) — drawn `singles thickness` times fatter and
labelled $\phi^t(\text{seed})$ with the seed's actual coordinates. `Spawn circle in E^s`
seeds `Manifold` curves (light blue) on a circle of radius $r$ about the origin in the
plane of the first two stable axes (in a 1D $E^s$: the two points $\pm r$). Since
$A$ is diagonal, zero components stay zero, so these curves never leave $E^s$ and
visibly contract to the origin.

Every single also carries a red **tangent vector** $\dot{x} = Ax$ out of its head
(`rebuildTangents()`, every frame, constant on-screen size: `length` × camera distance),
labelled $\dot{x}$ just beyond its tip; when it points up on screen the $\phi^t$ label
moves below the particle. `camera follows a single` makes the orbit camera track a
single (newest, or `follow #`): `followCamera()` moves [[Camera]]'s `orbitCenter` and
`position` together after the integration step, so it never lags; unticking returns to
the saved centre. Same mechanism as [[vector-field]].

**Settings persistence.** `settingsTable()` lists every tunable as key → pointer; `Save
settings` writes it as `key value` lines to `sims/diagonal_linear/settings.txt`
(absolute path via `SIMS_DIR`, so explorer and standalone share it) and `init` loads it
if present, clamping the sizes a hand edit could blow up. The **camera view** is saved
too: `captureCamera()` copies the orbit angles, radius and center ([[Camera]]'s
`orbitTheta`/`orbitPhi`/`orbitRadius`/`orbitCenter`) on save, and `applySavedCamera()`
restores them at the end of `init` (NaN = never saved, keep the default framing).
Curves are not saved. Explorer thumbnail: `docs/screenshots/diagonal_linear.png`.

## Modules attached
- [[TrajectoryRunner]] — advances every curve (integrate on all cores, then upload) and
  times integrate / upload / draw; readout under Other settings → Performance.
- [[FlowKeys]] — arrow-key speed and direction.
- [[Integrator]] — RK4 stepping of $\Lambda\mathbf{x}$, reverse time.
- [[Trajectory]] — one per seed: state, trail polyline, `drawHead()` particle.
- [[Shader]] — one program from files, shared by every draw.
- [[ArrowGlyphs]] — the glyph grid, the tangent arrows, and one head-only cone batch
  per subspace kind (instanced; colored per batch).
- [[Mesh]] — dynamic subspace geometry; static axes.
- [[Shapes]] — axis lines.
- [[LatexImage]] — top-right $\dot{x} = Ax$ overlay, written out as the live
  diagonal matrix times the column $(x_1, x_2, x_3)^T$, re-typeset only when a $\lambda_i$ changes (`show x' = Ax`); and
  $\phi^t(x_0)$ above every single particle — $t$ is the ODE time integrated since its
  seed (0 at $x_0$, negative in reverse), composed per frame from pieces typeset once
  (`label singles with phi^t(x0)`).
- [[Camera]] — plain orbit camera, framed in `init`; zoom speed scaled by `domain radius`.

## The math
- Flows, equilibria, glyph conventions: [[integral-curves]]. Stepping: [[runge-kutta]].
- Click-to-drop is the same ray/plane unprojection as [[vector-field]] (camera-facing
  plane `click depth` of the way to `orbitCenter`).

## Controls / uniforms
**Other settings → LaTeX**: `equation` ($\dot x = Ax$), `phi^t labels` (also $\dot x$), `E^s, E^u labels` sizes in px, saved.
The panel is split into sections, each under a red `sectionHeader()` title: System,
LaTeX overlays, Invariant subspaces, Flow, Ball of trajectories, Circle in
$E^s$, Single trajectories, then a collapsed **Other settings** header (closed by default)
holding Camera (`Reset camera`), Integrator (`method`, `max step h`, saved), Arrow field
and Trails & particles, then Settings.
**System**: $\lambda_1,\lambda_2,\lambda_3$ sliders, presets (sink, source, both saddles, a
degenerate one with a center line), readout of $\dim E^s/E^u/E^c$, `show subspaces`,
`axes`, `subspace extent`, `plane alpha`, `line width`, `flow arrows on subspaces`
(+ `arrows per half-axis`, `subspace arrow size`).
**Arrows**: `grid N`, `arrow scale`, head solid/barbs + `head length`/`head width`,
`length by magnitude`, `hide below`, `domain radius`, `arrow alpha`, `alpha gradient`,
auto/manual color scale.
**Flow**: Play/Pause (also **Space**, ignored while typing; arrow keys via [[FlowKeys]]), `reverse`, `Restart` (all seeds back to start), `Clear all`,
`speed`; ball `center`, `radius`, `particles`, `fill volume`, `single at center` (also drops
a green single at the ball's centre — labelled, followable, removed with the ball),
`Drop ball` (replaces the current ball), `Remove ball`; **click the view** for a single trajectory (seeded under the cursor on a camera-facing
plane `click depth` of the way to the orbit center), `tangent vector x'` + `length`,
`Clear singles`, `camera follows a single` + `newest` / `follow #`;
`circle radius`, `circle particles`, `Spawn circle in E^s` (replaces the previous one),
`Remove circle`; `show trails`, `trail points`, `trail width`, `trail alpha`,
`particle size`, `singles thickness`.
**Save settings** at the bottom.
**Uniforms**: `view`, `projection`, `u_magScale`, `u_pointSize`, `u_useFade`,
`u_shadeMode`, `u_useRamp`, `u_color`, `u_alpha`, `u_alphaTail`, `u_round`.

## Future
- Full (non-diagonal) $A$: eigenvectors from a solver, complex pairs as rotating planes.
- Ball volume readout vs. $e^{\mathrm{tr}\,\Lambda\, t}$; drawing the ball as a mesh hull.
