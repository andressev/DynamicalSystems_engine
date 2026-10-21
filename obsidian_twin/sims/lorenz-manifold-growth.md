---
type: sim
status: wip
code:
  - sims/lorenz_manifold_growth/LorenzManifoldGrowthSim.cpp
  - sims/lorenz_manifold_growth/shaders/vertex.glsl
  - sims/lorenz_manifold_growth/shaders/fragment.glsl
  - sims/lorenz_manifold_growth/shaders/surface_vertex.glsl
  - sims/lorenz_manifold_growth/shaders/band_vertex.glsl
  - sims/lorenz_manifold_growth/shaders/band_fragment.glsl
  - sims/lorenz_manifold_growth/shaders/equilibrium_vertex.glsl
  - sims/lorenz_manifold_growth/shaders/equilibrium_fragment.glsl
---
# lorenz-manifold-growth

`sims/lorenz_manifold_growth/` — lib `sim_lorenz_manifold_growth`, standalone exe
`lorenz_manifold_growth_standalone`.

The global Lorenz flow with its equilibria as beads, where the stable and unstable
manifold of each equilibrium is *grown*: first the local Taylor patch out of $x^*$, then
the global manifold carried on by the flow.

## What is being simulated
The same picture as [[lorenz-flow]] — arrow field, balls and singles, the beads $O$,
$C_\pm$ — minus the zoom/handoff (a click on a bead no longer flies to it). Hovering a
bead opens a **popup** beside it with two sections, $W^u(x^*)$ and $W^s(x^*)$ (the
actual name in the notation: $W^u(O)$, $W^s(C_+)$...). Each section first offers
**Grow $W^u_{\mathrm{loc}}(x^*)$**: the patch of the parametrization method
([[lorenz-stable-manifold]], [[InvariantManifold]]) grows out of $x^*$ over
`growth time`, out to the radius where the map is still trusted. Once grown, the
section offers **Grow $W^u(x^*)$**: for a *curve* (at the classic parameters $W^u(O)$,
$W^s(C_\pm)$) the two ends of $W_{loc}$ flow on — forward in time on $W^u$, backward on
$W^s$ — each as a trajectory that keeps its whole curve, so the manifold is the Taylor
arc plus the orbits of its ends ([[global-manifold-growth]]). For a *surface* the global
manifold is grown ahead of time by geodesic level sets and stored
([[0011-precomputed-level-set-surfaces]], [[LevelSetSurface]]): `loadStoredSurfaces` reads
every `data/surface_<O|Cp|Cm>_<s|u>.lss` at init, and `storedFor` enables `Grow W` only
when the file's tags match $\sigma, \rho, \beta$, K order, equilibrium and side (else the
button is disabled — "classic parameters only" if a file exists, "not yet" if not).
At the classic parameters $W^s(O)$ is stored out to geodesic distance $\eta = 97$ (98
bands). Pressing it sets `global`; `shownEta` then grows at a steady geodesic speed, the
whole surface in `surface growth time` (Pause/Resume, then Regrow). `renderSurfaces`
draws every band up to the one holding `shownEta` with `band_vertex.glsl`: band $k$ at
$\mathrm{clamp}(u_{grow} - k, 0, 1)$ slides its new points out from where they grew, and
`band_fragment.glsl` colors bands light / dark blue by turns, so each geodesic circle shows.
Next to it, **Learn geodesic level sets** zooms onto the patch ($3.2\,r$ away) while
everything else fades (`enterLevelSets`, `sceneFade`), then switches to
[[learn-geodesic-level-sets]] (`handOffLevelSets`, [[0009-sim-handoff-requestswitch]]):
parameters, which patch, K order, bead size, fill opacity, the view to return to, and a
bitmask `loc` of every grown $W_{loc}$. Coming back (`fromLevelSets` in `init`) restores
those, keeps the camera on the patch and zooms out to the saved view while the scene
fades back in. The global branches are not handed over; they are gone after a visit.
The first `update` after `init` clamps its `dt` (it carries init's time, most of a second
in debug), so the zoom out starts where it should.

## How it works
`init` loads `settings.txt`, builds the axis and sphere meshes and sets the ODE
right-hand side to [[LorenzSystem]]`::eval`. `applyParams()` (any $\sigma/\rho/\beta$
change) re-reads the equilibria, runs [[Eigen3]] on each Jacobian and classifies it,
then `rebuildManifolds()`, which only marks every equilibrium stale (`ready = false`,
`manifoldGen` bumped) and drops the global branches — they belonged to the old field —
and any watch on them. Dragging $\sigma/\rho/\beta$ therefore moves only the field and
the equilibria. `pollManifolds` (every `update`) starts one worker (`std::async`) per
stale equilibrium once no slider is held (`paramsDragging`), and applies a finished
result only if its `jobGen` is still the current `manifoldGen` (else it is dropped and
redone). The worker, `computeManifolds`, is pure: `K.build` on both sides (order
`K order`, default 12, at most [[ConjugacyMap]]`::kMaxOrder` = 16) and `trustedRadius(K)`
— the largest $|\theta|$ (bisection up to 25) where the relative invariance defect stays
under $10^{-3}$ at eight radii out to it (`relDefectBelow`, the eight checked in parallel,
outermost first; same result as the original `maxRelDefect` loop, bit for bit, ~6x
faster). Until its equilibrium is `ready` a side shows "computing..." in the popup and
cannot be grown; what was grown locally reappears at the new radius. `init` (and the
handoff back) computes synchronously (`computeManifoldsNow`). `applyManifolds` then
sets the equilibrium's [[ConjugacyMap]] from $K_s, K_u$ (needed by every surface) and
runs `buildLocal`: a curve as two strips of `kCurveSteps` points $x^* + K(\pm r\,e)$
along the orthonormal basis of $E$ (on the CPU); a surface as the *unit disk of $E$* in
`kRings` $\times$ `kSegs` triangles followed by every ring as a loop. Each side keeps its
own dynamic [[Mesh]].

`update`: [[FlowKeys]] + Space, advance balls/singles as in lorenz_flow, then the
**branches**: forward ones by $+|h|$, backward ones by $-|h|$ (the singles' method —
Taylor by default — so the manifold's own direction is fixed and only the speed is the
flow's; the `reverse` flag does not retract a branch). Each side's branches stop at
`branch time` of field time (default 21: 21 s at 1x) or while paused (`S.t0`, `S.limit`,
`S.paused`).

**Backward curves** (`curveStep`, `advanceCurves` — every curve: balls, singles and
branches). The Lorenz flow backward blows up in finite time (a $W^s(C_\pm)$ branch used
to leave the screen in $\approx 0.09$ field time). Running backward, a curve's step is
scaled so its arc speed is at most `backward speed cap` (default 15 world units per
field time) — a time change only, the traced curve is the same — and a curve farther
than `pause beyond` $\times$ `domain radius` (default 12.5, about 375 units) from the cube's centre stands
still until the flow runs forward again. Curves at full step integrate together in
parallel, slowed ones one by one. Forward curves are untouched. Hover
picking (`pickEquilibrium`, the lorenz_flow ray–sphere test), a click on a bead
*pins* its popup (`togglePin`; a click elsewhere drops a single as before), glow eased
toward the hovered / popup bead, `grow` of every local manifold eased $0 \to 1$,
`updateCamera`, overlays, then `drawManifoldPopup`.

**The popup** (`drawManifoldPopup`, `drawSideSection`): an ImGui window placed beside
the bead's screen point every frame (clamped into the window; a pinned popup whose
bead is off-screen parks at the right edge). It opens on hover and stays while the
mouse is on the bead or on the popup (`kPopupGrace` of slack only for the trip from
the bead; once the mouse has been on it, `popupEntered`, leaving closes it at once), or
for good while pinned (small `x`, or click the bead again, to unpin). Header: the name
and type; a `Ball` button + radius slider (`eqBallRadius`): a ball of trajectories
around $x^*$ via `dropBall` (the panel's particle count / fill volume, stacks up to
`kMaxBalls`); then $W^u$ then $W^s$, each header with a `ball` button
(`dropManifoldBall`: `ballCount` seeds $x^* + K(\theta)$ on $W_{loc}$ within
$\min(\texttt{eqBallRadius}, r)$ — evenly on the segment, a sunflower on the disk — in
the side's color). Button faces are typeset
with [[LatexImage]] (`latexButton`: an `ImageButton` of the cached texture, a plain
text button if the typesetting fails), `tex()` caches every label by string, size
and color. Per side: `Grow W_loc` → (grown) `r = ` trusted radius, then for a curve
`Grow W` → `watch` + `+`/`-` (which end), then `Pause`/`Resume` with the branch's
flow time `t`, turning into `Extend` (another `branch time`) once the limit is
reached; for a surface the disabled pair; `reset`.

**Watch** (`watchEq`, `watchSide`, `Side::branchChoice`): the camera rides the tip of
the chosen branch through the same follow machinery as lorenz_flow's "camera follows
the newest" (`followTarget()` — the watched branch wins over the newest single; slide
on over `follow ease`, ease back over `kReturnSeconds` when it ends). A tap of **1**
detaches in place (watch off if watching, else the single follow toggles); holding
**1** recenters (`recenterCamera`: everything off, home view). A watched backward
branch ($W^s(C_\pm)$) slows to the speed cap and stops at the freeze distance, so the
camera never chases it off to infinity.

`render` — everything that writes depth first, then what only blends, so a curve
behind a sheet shows through it tinted and one in front stays in front: beads, axes,
`renderLocalCurves` (1-D $W_{loc}$ as lines `width` px drawn up to `grow`), trails and
heads, `renderBranches` (the branch curve and a bead at its tip, `width` px), glyphs
(no depth writes), `renderSurfaces` (2-D $W_{loc}$ through `surface_vertex.glsl`, which
scales the unit disk by $r \cdot$ `grow` and applies $K$ per vertex — `hInv` of the
equilibrium's [[ConjugacyMap]], exactly $K$ on $E$ — so the surface grows continuously:
a `surface opacity` fill that tests but never writes depth — at opacity 1 it is opaque and does write depth (`fillDepth`), so its folds hide properly — plus the edge ring; then a
stored surface's bands, the same way), halos.

**Clip planes** (`fitClipPlanes`, every frame): the far plane reaches past the whole
scene — cube plus the freeze distance — from wherever the camera is, so zooming far out
shrinks the picture instead of clipping it away (it used to sit at a fixed 600). The
near plane is left at the shared [[Camera]]'s default.

## Modules attached
- [[InvariantManifold]] — $K$ of each side at each equilibrium; `relDefectBelow` for the
  trusted radius; `orthoBasis`/`toParam`/`eval` for the patch and the branch seeds.
- [[ConjugacyMap]] — $K_s + K_u$ on the GPU: the vertex stage of the 2-D local manifolds.
- [[LevelSetSurface]] — the stored global surfaces (`data/*.lss`), grown band by band.
- [[LorenzSystem]] — the field (ODE rhs, jet, glyph sampling), equilibria, Jacobians,
  the quadratic part for $K$.
- [[Eigen3]] — type of each linearization (popup header).
- [[TaylorIntegrator]] / [[Integrator]] — singles *and branches* on the singles'
  method (Taylor default), balls on RK4.
- [[Trajectory]] — balls, singles, and the branches (`kBranchPoints` = 120000, so a
  branch keeps its whole curve).
- [[TrajectoryRunner]] — parallel integrate + upload, Performance readout.
- [[FlowKeys]] — arrow-key speed and direction, tap/hold of key 1.
- [[ArrowGlyphs]] — the instanced field on a cube of `domain radius`.
- [[Shader]] — the glyph pair (also the manifolds) and the bead pair, copied from
  lorenz_flow.
- [[Mesh]] / [[Shapes]] — axes, unit sphere, one dynamic mesh per manifold side.
- [[LatexImage]] — the system top-right, $O$/$C_\pm$ labels, the popup's section
  titles and button faces, the $W^u_{\mathrm{loc}}(O)$ / $W^u(O)$ label by each patch.
- [[Camera]] — orbit; the sim drives `orbitCenter` while watching / easing back.
- [[App]] — screenshots, `kRecenterHoldSeconds`; `requestSwitch` / `handoff` to and
  from [[learn-geodesic-level-sets]].

## The math
- The Taylor patch and why it is exact on the manifold: [[lorenz-stable-manifold]];
  the flow carrying it on, and why surfaces need more: [[global-manifold-growth]].
- Measured trusted radii at the classic parameters (order 12, defect $< 10^{-3}$):
  $W^u(O)$ and $W^s(C_\pm)$ reach the search cap 25; $W^s(O)$ 12.16; $W^u(C_\pm)$ 2.50
  (the near-Hopf complex pair, as in [[lorenz-equilibrium]]'s check). Both sides of one
  $x^*$ are drawn at the **smaller** of the two (`rebuildManifolds`), so the curve never
  reaches past the surface: $O$ at 12.16, $C_\pm$ at 2.50 — and a curve's branches
  start from those ends.

## Controls / uniforms
Hotkeys as [[lorenz-flow]]: Space play/pause, arrows speed/direction, **1** tap
detach (watch or single follow) / hold recenter, **2** arrows fade, **R** clear
trajectories, **B** / **V** balls, **0** screenshot, **H** panel ([[App]]). No
Backspace (nothing to zoom back from).
Panel: **Lorenz system**, **Equilibria** (one row per bead: hover lights it, click
pins its popup, `[pinned]` tag; `Reset manifolds` once anything is grown), **Flow**,
**Ball of trajectories**, **Single trajectories**, collapsed **Other settings**
(camera; integrator — `ball`, `singles, manifolds`; **Manifolds**: `K order`,
`growth time`, `width`, `surface opacity`, `branch time`, `surface growth time`, `backward speed cap`,
`pause beyond`; LaTeX sizes; bead size + `labels`, which
also governs the $W$ labels; arrows; trails; performance), **Settings**, **Screenshot**.
Explorer thumbnail: `docs/screenshots/lorenz_manifold_growth.png`.
Uniforms as [[lorenz-flow]] (glyph pair + bead pair).

## Future
- Surfaces: only $W^s(O)$ at the classic parameters is stored; $W^u(C_\pm)$ (or other
  parameters) need their own Grow rings + Save surface in [[learn-geodesic-level-sets]].
- A third sim with beads would justify extracting the bead/picking/halo/popup code
  into a module ([[0010-manifold-popup-and-copied-baseline]]).
- Branches ignore `reverse`; a retracting branch would need a trail that can shrink.
- The arrow cube itself still covers only `domain radius`; zoomed far out it is a
  small block in the middle.
