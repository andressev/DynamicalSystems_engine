---
type: sim
status: wip
code:
  - sims/lorenz_equilibrium/LorenzEquilibriumSim.cpp
  - sims/lorenz_equilibrium/shaders/vertex.glsl
  - sims/lorenz_equilibrium/shaders/fragment.glsl
  - sims/lorenz_equilibrium/shaders/equilibrium_vertex.glsl
  - sims/lorenz_equilibrium/shaders/equilibrium_fragment.glsl
  - sims/lorenz_equilibrium/shaders/glass_fragment.glsl
  - sims/lorenz_equilibrium/shaders/conjugacy_vertex.glsl
  - sims/lorenz_equilibrium/shaders/morph_vertex.glsl
---
# lorenz-equilibrium

`sims/lorenz_equilibrium/` — lib `sim_lorenz_equilibrium`, standalone exe `lorenz_equilibrium_standalone`.

The local picture at one Lorenz equilibrium $x^*$: field and trajectories inside a glass
ball $V = B_r(x^*)$, everything clipped to it. Mode 2 of the Hartman–Grobman track.

## What is being simulated
The Lorenz flow ([[lorenz-flow]] has the equations) seen only inside $V$. This is where
Hartman–Grobman lives: near a hyperbolic $x^*$ the flow is conjugate to its linearization.
It shows the flow in $V$ — at $C_\pm$ a dropped ball of seeds flattens onto the
unstable plane within a fraction of a second ($\lambda_s \approx -13.85$) and then
spirals slowly outward ($0.094 \pm 10.19i$) — next to its linearization (**stage A**,
M4): $E^s$ and $E^u$ through $x^*$ and the field $f$ against $A(x - x^*)$ with
$A = Df(x^*)$. **Stage B** (M5) compares the flows on the stable side:
$\psi^t(x) = x^* + e^{At}(x - x^*)$ keeps $E^s$ flat, the true $\varphi^t$ does not —
only the curved $W^s = x^* + K_s(\theta)$ is invariant, with
$\varphi^t(x^* + K_s(s)) = x^* + K_s(e^{\Lambda t}s)$. **Stage C** (M6, section
Hartman–Grobman) shows the whole conjugacy: the linear ball $U = B_\rho(x^*)$ next to
its image $V = x^* + \tilde h^{-1}(U)$, $\tilde h^{-1} = K_s + K_u$ — exact on $E^s$/$E^u$,
an interpolation between — with paired orbits and a check of whether $\rho$ is small
enough for the homeomorphism.

## How it works
**Entering.** Normally from [[lorenz-flow]]: clicking a bead zooms onto it and, when the
zoom lands, calls `App::requestSwitch("lorenz_equilibrium", handoff)`
([[0009-sim-handoff-requestswitch]]). Handoff keys: `sigma`, `rho`, `beta`, `eq` (0 = O,
1 = C+, 2 = C−, the order of [[LorenzSystem]]`::equilibria`), `beadRadius`, and
`globalCx/Cy/Cz/R` — lorenz_flow's view to return to, kept untouched and handed back.
`init` takes them, leaves the camera where the zoom put it, and fades the glass, arrows
and trails in over 0.6 s; the bead arrives at lorenz_flow's focused size (`beadRadius`)
and eases down to this sim's much smaller `bead size` (0.06 — here $x^*$ is a point the
flow is seen around, not a marker). Leaving hands the current size back, and lorenz_flow
grows it from there. From the menu (no handoff) it frames $x^*$ at $3r$ from its saved settings.

**Leaving.** `Back to global view` / **Backspace** fades the content out (0.35 s), then
hands back `sigma/rho/beta/eq` + the return view; lorenz_flow comes up on $x^*$ and eases
out. Only offered when `lorenz_flow` is registered (not in the standalone).

**Frame.** `update`: advance every [[Trajectory]] via [[TrajectoryRunner]], click drops a
single on the camera-facing plane through $x^*$ (only inside $V$), keep the orbit centred
on $x^*$ (it moves with $\rho$). `render`: bead, then arrows and trails with the glyph
pair **clipped to $V$** (`u_clipCenter`/`u_clipRadius`: fragments outside discarded, a
soft 4% rim), the halo, then the glass last — blended, no depth writes, Fresnel-weighted
(clear face-on, bright at the silhouette, sharp highlight, back faces at half).
Arrows: a lattice through the cube around $V$, points outside the ball dropped, sampled
in parallel.

**Stage A — the linearization.** `applyParams` takes $A = Df(x^*)$ ([[LorenzSystem]])
and its [[Eigen3]] decomposition.
- `rebuildSubspaces()`: orthonormal bases of $E^s$/$E^u$ (Gram–Schmidt over
  `realBasis`, $\{\mathrm{Re}\,v, \mathrm{Im}\,v\}$ for the complex pair) cut to $V$ —
  a line is a diameter with two arrows of the linear flow (in on $E^s$, out on $E^u$);
  a plane is a translucent disk + rim with two rings of $Au$ arrows (they lie in the
  disk, $A$ maps $E$ to itself, so the rotation of a complex pair shows). Blue $E^s$,
  red $E^u$ (as [[diagonal-linear]]), labelled. Sink/source ($E = \mathbb{R}^3$): none.
- `arrows` mode: $f$, $A(x - x^*)$, both ($Au$ flat white over the colored $f$, one
  shared scale — equal near $x^*$, parting toward the glass), or the remainder
  $f - A(x-x^*) = Q(u,u)$ on its own scale.
- HUD (under the system, top-right): $A$ with numbers, the eigenvalues (a pair as
  $a \pm bi$), $\dim E^s$/$\dim E^u$. Hyperbolicity: red when $\min|\mathrm{Re}\,\lambda| < 10^{-3}$
  (Hartman–Grobman does not apply), orange below 0.2 — $C_\pm$ at $\rho = 28$ sit at
  0.094, near the Hopf point.

**Stage B — $\varphi^t$ vs $\psi^t$ from $E^s$.** `buildManifold()` (in `applyParams`)
builds $K_s$ with [[InvariantManifold]] (stable side, `K_s order`, default 12).
`seedRing()`: seeds $\theta$ on a circle of `ring radius`$\cdot r$ in $E^s$ (the two
points $\pm\theta$ when $E^s$ is a line, as at $C_\pm$), each starting three curves at
the same field time — $\psi^t$ from $x^* + \theta$ on the linear field (orange),
$\varphi^t$ from the same point (green: leaves $E^s$), $\varphi^t$ from
$x^* + K_s(s)$, $s$ = `toParam`$(\theta)$ (blue: stays on $W^s$). All three run on the
singles' method; for Taylor, $\psi^t$ uses an exact linear jet
($c_1 = A(x - x^*)$, $c_{k+1} = A c_k/(k+1)$). `rebuildStageMesh()` each frame: link
lines $\psi^t \leftrightarrow \varphi^t$ from the same seed (orange to green, same $t$ —
how far the true flow has left the linear one), and the formula points
$x^* + K_s(e^{\Lambda t}s)$ (`check`, yellow, drawn over the blue heads with no depth
test). Measured at $r = 3$, ring $0.6r$, $t = 0.3$:
blue head vs formula $7\cdot10^{-14}$ at $O$, $4\cdot10^{-14}$ at $C_+$; green vs blue
0.57 at $O$, 0.056 at $C_+$. `labels`: $\phi^t(x_0)$ over the green head and $\psi^t(\theta_0)$ under the orange one — the true and the linear flow from the same seed, the pair the link joins — on every seed; size in Other settings → LaTeX. The ring is cleared by a new $x^*$/parameters, `Clear all`;
`Restart` resets it.

**Stage C — Hartman–Grobman.** `buildConjugacy()` (from `buildManifold`) builds $K_s$,
$K_u$ (order capped at [[ConjugacyMap]]`::kMaxOrder` = 16) and `hm.set` uploads them;
not hyperbolic / no saddle → the section says why and offers nothing.
- `linear ball radius` $\rho$. **Show conjugacy** runs a timeline (`advanceHgTimeline`;
  **Hide** plays it backwards and then drops the pairs): first everything but $V$ fades
  to black (`contentFade` $\times (1 - $`hgFocus`$)$) and the camera frames $V$ — the
  sphere through `hInv` in `shaders/conjugacy_vertex.glsl`, green glass (red when the
  check fails), holding the true field sampled at $x^* + \tilde h^{-1}(\theta)$ on a
  lattice $\theta \in B_\rho$ and $\varphi^t$ (green) from $x^* + \tilde h^{-1}(\theta)$
  for pair seeds on the sphere $0.6\rho$; then the camera eases out (log-interpolated
  distance, centre to the midpoint) and $U$ fades in beside it — round orange glass at
  `hgOffset()` = screen-right at the press $\times\,\rho(2 + $`distance`$)$, holding
  $A\theta$ and $\psi^t$ (orange). $U$'s content is integrated around $x^*$ and drawn
  shifted by the vertex shader's `u_offset`, clipped to $U$'s sphere. `links` join each
  $\psi^t$ head (in $U$) to its $\varphi^t$ partner (in $V$) across the gap;
  `h^-1(psi^t)` marks $x^* + \tilde h^{-1}(\psi^t - x^*)$ in $V$ (yellow) — on the green
  head on the manifolds only. Above each ball its name and, between the name and the glass, the system it holds: $U$ over $\dot\theta = Df(x^*)\,\theta$, $V = \tilde h^{-1}(U)$ over $\dot x = f(x)$. Drawn in front of the panels
  (`LatexImage::drawAt(..., foreground)`); the camera frames the pair a little low to
  leave room for them, and the HUD's linearization block hides while $U$ is shown (its
  label already says $\dot\theta = Df(x^*)\,\theta$).
- **Dissolve — the crash**: each particle crashes on its own ball's real border —
  $\psi^t$ when $|\psi^t - x^*| = \rho$, $\varphi^t$ when $|\tilde h(\varphi^t - x^*)| = \rho$,
  i.e. on the deformed surface $x^* + \tilde h^{-1}(\rho S^2)$, with $\tilde h$ by Newton
  on `hInv` warm-started from the last frame (`hgForward`; residual $\sim10^{-15}$, crash
  within one frame step of the surface). For the true conjugacy the two crashes would be
  simultaneous; with the interpolated $\tilde h^{-1}$ they drift apart off the manifolds —
  the gap is the interpolation error. When either crashes, **both explode** in that
  instant — the one that hit exactly on its border (`crossing`: bisection on the segment
  between last frame's state and this one, through $\tilde h$ for $V$), with the
  shockwave on its glass centred there; its partner where it is. Both stop dead (no
  longer integrated; the head is drawn frozen at that point, unclipped), flash
  white, shrink away and break into a spray of fragments in their color
  (`spawnShards`: 14 each, random directions, slowing) — head, shards, trail fade and
  shockwave all on one clock, the shockwave `duration`; and each ball gets a **shockwave** from the impact (`spawnBurst`):
  in `glass_fragment.glsl` a ring runs over the ball (arc distance from the impact /
  `propagation`, measured on the sphere the ball is made from — `vSphere`, so on $V$ the
  ring follows the deformed border — easeOutSine), pushing the scene behind it radially on screen and
  splitting its colors, with a flash inside in the particle's color (orange on $U$, green
  on $V$) — Benjamin Weschler's screen-space shockwave (MIT) moved onto the sphere. The
  scene behind is a [[FrameCopy]] captured just before the balls are drawn; up to 8
  waves per ball (`setWaves`). Other settings → **Crash shockwave**: `intensity`,
  `propagation` (degrees of arc, 0.5–120 on a logarithmic slider, default 20), `distortion`, `ring thickness`, `feathering`,
  `color split`, `duration`, saved.
- **Transform flow** (button): seeds on the sphere $0.5\rho$ in $U$ flow under $\psi^t$
  (orange) until the first one hits $U$'s border (`max flow time`, ODE, is only a
  fallback for a flow too slow to get there), pause
  0.7 s, then the whole picture is carried to $V$ over `transform time`: every trail point
  and head goes through `shaders/morph_vertex.glsl` — $\theta \mapsto x^* +
  \text{shift}\,(1-s) + \mathrm{mix}(\theta, \tilde h^{-1}(\theta), s)$, $s$ eased $0\to1$,
  color orange $\to$ green — so all vertices move in parallel on the GPU (the same
  [[ConjugacyMap]] GLSL as $V$); a ghost of $U$'s glass travels along, deforming from the
  round sphere into $V$ (`conjugacy_vertex.glsl` with `u_morphOn`). They stay in $V$
  until the next transform, Reseed, a radius / parameter change, or Hide
  (`startMorph`, `updateMorph`, `renderMorph`, `renderMorphGhost`).
- **Labels** (`labels`): $\phi^t(x_0)$ over every green head in $V$, $\psi^t(\theta_0)$
  over every orange head in $U$, and a small $\tilde h^{-1}$ (as in $V$'s label) at the middle of every link (the link itself a gradient from $\psi^t$'s orange to $\varphi^t$'s green — `fragment.glsl`'s `u_useRamp` 2) —
  the link is the map from the linear orbit's point to its partner (`drawHgPairLabels`).
- **The check** (`runHgCheck`, on release of the slider and on any parameter change):
  `hgDefect(ρ)` — the largest relative invariance defect of $K_s$ and $K_u$ over the
  sphere $|\theta| = \rho$, each $\theta$ split into its $E^s$/$E^u$ parts; `hgMinDet(ρ)`
  — the smallest $\det D\tilde h^{-1}$ on a lattice in $U$ (central differences; 1 at
  $x^*$, $\le 0.05$ = folding over, no longer one-to-one). Green "ok" below $10^{-3}$,
  orange "approximate" below $5\cdot10^{-2}$, red "too big" above or when it folds; plus
  the **safe radius** (bisection up to 25), which is also the `linear ball radius` slider's maximum — computed first, so the radius is pulled down to it when $x^*$ or the parameters change. Measured: at $O$ defect $3\cdot10^{-11}$ at
  $\rho = 3$ and safe beyond 6; at $C_+$ $10^{-8}$ at 1, $6\cdot10^{-5}$ at 2, $10^{-2}$ at 3,
  safe up to $\approx 2.49$ (the near-Hopf $K_u$). $\det$ stays $> 0.7$ there — the
  defect, not folding, is what limits $\rho$ for Lorenz.

## Modules attached
- [[FrameCopy]] — the frame so far, bent by the crash shockwaves on the glass.
- [[ConjugacyMap]] — $\tilde h^{-1} = K_s + K_u$: $V$ deformed per vertex on the GPU,
  pair seeds and the check on the CPU.
- [[InvariantManifold]] — $K_s$ for stage B (seeds on $W^s$, the formula check).
- [[TaylorIntegrator]] — singles on the Taylor method (default: order 20, tolerance
  $10^{-14}$), the cloud on RK4; `Integrator` section: one method combo for each, order
  + tolerance shown when Taylor is picked, `max step h` = trail spacing for Taylor.
- [[LorenzSystem]] — the field, the equilibria, which $x^*$.
- [[Integrator]] / [[Trajectory]] — RK4 seeds and singles.
- [[TrajectoryRunner]] — parallel integrate + upload; Performance readout.
- [[FlowKeys]] — arrow-key speed and direction.
- [[ArrowGlyphs]] — the instanced field inside $V$.
- [[Shader]] — glyph pair (+ clip), bead pair, glass (bead vertex + `glass_fragment`).
- [[Mesh]] / [[Shapes]] — unit sphere for bead, halo and glass.
- [[LatexImage]] — the system top-right, the $x^*$ label.
- [[Camera]] — orbit around $x^*$; kept as handed over.

## The math
- Linearization, conjugacy, why hyperbolicity matters: [[integral-curves]];
  the parametrization method behind stages B/C: [[lorenz-stable-manifold]].

## Controls / uniforms
**Other settings → LaTeX**: `equation`, `HUD` (the typeset matrix/eigenvalues; its plain-text lines scale along), `equilibrium label`, `E^s, E^u labels` sizes in px, saved.
Red `sectionHeader()` titles: **Equilibrium** (O / C+ / C− radio — switching clears the
trajectories; `Back to global view`), **Lorenz system** ($\sigma,\rho,\beta$, Classic,
`show x' = f(x)`), **Linearization** (`E^s, E^u` + plane opacity, `HUD`, `arrows`:
f / A(x - x*) / both / f - A(x - x*)), **Stable side: phi^t vs psi^t** (`ring radius`,
`ring seeds` when $E^s$ is a plane, Seed / Clear ring, colored toggles for the three
curves, `labels`, `links`, `check x* + K(e^{Lt} s)`), **Hartman-Grobman** (`linear ball radius`,
the check line + safe radius, Show / Hide conjugacy, `distance` between the balls,
Reseed, Transform flow + `transform time` / `max flow time`, `pairs`, `labels`,
`links (h^-1)`, `h^-1(psi^t)`), **Flow** (Play/Pause,
Space, arrow keys — a logarithmic ramp down to $10^{-4}$, a decade per 1.2 s held —,
reverse, Restart, Clear all, `speed` on a logarithmic slider from $10^{-4}$), **Trajectories in V**
(`particles`, `seed radius` of $r$, `fill volume`, Drop/Remove seeds; click inside $V$
for a single, Clear singles), collapsed **Other settings** (camera; integrator — `seeds` and `singles` methods,
Taylor `order` + `tolerance`; ball V — `radius r`, `glass` + strength, `K_s order`; look — bead
size, label, singles thickness; arrows; trails; performance), **Settings**
(`settings.txt`, camera angles + distance), **Screenshot** (key **0**).
Explorer thumbnail: `docs/screenshots/lorenz_equilibrium.png`.

## Future
- M5–M6: $\varphi^t$ vs $\psi^t$; the deformed ball via [[ConjugacyMap]]; orbits leaving
  $V$ dissolve on the rim (now they are only clipped).
- M7: refraction through the glass (`RenderTarget`).
