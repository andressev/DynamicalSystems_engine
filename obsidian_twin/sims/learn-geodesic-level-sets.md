---
type: sim
status: wip
code:
  - sims/learn_geodesic_level_sets/LearnGeodesicLevelSetsSim.cpp
  - sims/learn_geodesic_level_sets/shaders/vertex.glsl
  - sims/learn_geodesic_level_sets/shaders/fragment.glsl
  - sims/learn_geodesic_level_sets/shaders/band_vertex.glsl
  - sims/learn_geodesic_level_sets/shaders/band_fragment.glsl
  - sims/learn_geodesic_level_sets/shaders/equilibrium_vertex.glsl
  - sims/learn_geodesic_level_sets/shaders/equilibrium_fragment.glsl
---
# learn-geodesic-level-sets

`sims/learn_geodesic_level_sets/` — lib `sim_learn_geodesic_level_sets`, standalone exe
`learn_geodesic_level_sets_standalone`.

The walkthrough of geodesic level sets, one step at a time. So far: the 2-D local
manifold of one Lorenz equilibrium alone — $W^s_{\mathrm{loc}}(O)$ at the classic
parameters — the triangle mesh it is drawn with (vertices in yellow), and a chosen edge
point $r$ with its half-plane $\mathcal{F}_r$.

## What is being simulated
Nothing moves yet. The patch $v^* + K(\theta)$ of the parametrization method
([[lorenz-stable-manifold]], [[InvariantManifold]]) out to the trusted radius, exactly
as [[lorenz-manifold-growth]] draws it, with its bead. **Show mesh** (or **M**) fades in
the mesh the surface really is: $v^*$, `rings` rings of `vertices per ring` points
$v^* + K(\theta)$ with $\theta$ on circles of radius $r\,i/\text{rings}$ in $E$, the
triangles between them as edges, every vertex as a yellow dot. The rings are evenly
spaced in $\theta$, not by distance on the surface, so on screen they bunch up near
$v^*$ and spread out toward the edge.

**The edge follows the method's spacing rule** (`spreadEdge`): the `rings` - 1 inner
rings keep `segs` even angles, but the edge starts from `segs` even angles and then,
until nothing changes, gets a point midway (in angle, on the true edge) wherever two
neighbours are farther apart than **maxSpread**, and loses a point closer than
**minSpread** to the one before it (unless the gap left would pass maxSpread). Angle 0
always stays. Defaults maxSpread $= 1$, minSpread $= 0.25$ (world units, the
parameters of geodesic level sets, [[global-manifold-growth]]); at the classic
parameters the edge of $W^s_{\mathrm{loc}}(O)$ gets 156 points, neighbours 0.39–0.99
apart. The band between the last inner ring and the edge is triangulated by walking
both rings by angle (`buildMesh`). The two values are typeset compactly top-right
(`drawParamsOverlay`) and set in the panel (**Geodesic level sets**); any change
re-spreads the edge (and, as any new mesh, clears $r$'s hits and the particles). **Edge point.** Click a vertex of the edge (the last ring; the one under the cursor
lights up) or pick it with the `r` slider. That vertex is $r$, its neighbours on the
edge are $r^+$ (next) and $r^-$ (previous), and the camera eases in onto it. Drawn and
typeset beside each: $r$, $r^\pm$, the chord $r^- r^+$, and the grey half-plane
$\mathcal{F}_r$ through $r$ with normal $n = \tfrac12 (r^+ - r^-)$ (not drawn). Only the half outside the edge is drawn: $\mathcal{F}_r$ is spanned by
`out`, the radial direction of $K$ at $r$ with its $n$ part removed, and
`across` $= \hat n \times$ `out`, and covers $r + a\,\text{out} + b\,\text{across}$ for
$0 \le a \le L$, $|b| \le L/2$, with $L$ = `plane size` $\times\,|r^+ - r^-|$. Its edge
side, the line through $r$ along `across`, is drawn brighter.

**Upright view of $\mathcal{F}_r$.** The zoom onto $r$ also turns the view so that
screen-up is `across`, which lies in $\mathcal{F}_r$ and is normal to the surface: the
patch reads as a floor and $\mathcal{F}_r$ as a wall standing on its edge, seen
diagonally from the patch side (so neither $n$ nor $\mathcal{F}_r$ is end-on) and a
little from above, on whichever side of $n$ and of the surface the camera already was. [[Camera]] orbits about world $+Y$ only, so the sim keeps its own orbit
frame (`viewFrame`, a quaternion whose local $y$ is screen-up): Camera places the orbit
in world axes, then `placeCamera` (first thing in `update`) re-places it in the frame and
sets `up`. The frame eases (`easeFrameTo`, slerp, smootherstep) alongside
`Camera::easeOrbitTo` with the same duration. Framing the patch eases it back to world
axes; `handBack` first turns upright where the camera stands (`kUprightSeconds`), then
switches, so lorenz_manifold_growth gets a world-up camera. Saved camera angles are in
world axes (`worldAngles`); the destructor restores world-up on the shared camera.

**The edge ball and its particles.** A green ball sits on the true edge
$v^* + K(r(\cos a\, e_0 + \sin a\, e_1))$ (`edgePoint`, not the mesh's polygon). Press
and drag slides it along the edge (`slideBall`): the cursor is projected onto each
segment between edge samples (`kEdgeSamples`) within `kBallStep` of the ball's current
angle, so the ball moves continuously and never leaps to another stretch of the edge
that passes near the cursor on screen. A click — released before the mouse moves `kBallDragPixels` (4 px) — or **Launch** starts a particle there,
integrated with the [[TaylorIntegrator]] in the manifold's own direction of time —
**backward on $W^s$** (into the past, away from $v^*$), forward on $W^u$. Arrows set the
speed (log ramp, $10^{-6}$ to 1 field time per second, default $5 \cdot 10^{-4}$: the
step past the edge is small) and flip the direction against that; **Space** pauses,
**R** clears. From a press on the ball until its release the mouse is the ball's (`ballPressed`), so it never picks or solves at an edge vertex — not even when the dragged ball slides out from under the cursor.
**Shoot r → ball** (once $r$ is chosen) launches `shootCount` particles (default 32)
evenly along the edge from $r$ to the ball, the shorter way round, as one batch;
`renderFronts` joins their heads in order (a stopped one at its $b_r$), so the edge arc
is seen as the flow carries it. The shot's first particle starts at $r$, already in
$\mathcal{F}_r$: it is a hit at once ($b_r = r$, $\tau = 0$, distance 0) and never moves.
**V** shoots it too, wherever the cursor is (with no $r$ chosen it only says so in the status line). **B** (or **Whole edge**) sends a
particle from every edge point at once (`shootRing`, at most `kMaxRing`, trails of
`kRingPoints` points spaced `kRingSpacing` radii apart, ~10 radii long), in the manifold's direction of time, joined head to head as a closed
loop — the ring tears into streaks almost at once (bunching at the cusp, stretching
off along the strong direction), which is why the surface is grown by geodesic level
sets and not by flowing its edge. These never stop at $\mathcal{F}_r$. When $r$ moves (`chooseEdge`), the ball
follows it along the edge, keeping its angular offset from $r$, so the next shot spans
the same stretch from the new $r$.

**Where a particle meets $\mathcal{F}_r$.** Each step, `findHit` checks whether the
particle crossed $\mathcal{F}_r$: a strict sign change of $(v - r)\cdot n$ between the
state before and after the step, on the outer half ($(b - r)\cdot\text{out} \ge 0$; the
plane is taken as the whole half-plane, not just its drawn part). The crossing is
interpolated inside the step in double precision (the step is tiny, so the chord is the
curve): the point $b_r(\tau)$, the signed field time $\tau$ since the edge ($< 0$
backward on $W^s$), and $\|b_r(\tau) - r\|$. A particle **stops** at its crossing (it is no longer integrated; its head is at most
one tiny step past $b_r$, under the dot). A single particle started exactly in the plane (the
ball at $r$) has to come back to it; a crossing right at $r$ gets a small tolerance on
the inner-half test (rounding near $r$ must not reject it). `syncHits` clears the hits
when $r$ or the mesh changed, also right before a shot. `renderHits` draws a golden dot there (additive glow around a solid core,
pulsing) and the segment to $r$; `drawHitLabels` typesets
$b_r(\tau = \dots)$ beyond the dot (away from $r$ on screen) and
$\|b_r(\tau) - r\| = \dots$ beside the segment, for the **latest** hit only (older hits
keep dot and segment; their labels would pile up). Choosing another $r$, or a new mesh,
clears the hits (`hitEdge`, `meshVersion`). Measured at the classic parameters, $r$ =
edge vertex 0, ball $0.12$ rad from it: $\tau \approx -0.0113$,
$\|b_r(\tau) - r\| \approx 2.06$; particles from the other side of $r$ bent away and
never met it. A shot of 32 from $r$ to $0.35$ rad: 31 met $\mathcal{F}_r$, their $b_r$
a curve in the plane ($\tau$ from $-0.001$ on), the one from $r$ itself went on.

**The geodesic BVP** ([[global-manifold-growth]] for the method). **Solve geodesic
BVP** (under Edge point) finds, for the current $r$, the edge point whose orbit meets
$\mathcal{F}_r$ at distance $\Delta_{\max}$ from $r$, and marks it $r_{new}$ (cyan,
labelled; the current $r$'s orbit from its start on the edge drawn to it).
It is the paper's formulation ([[geodesic-bvp]]): the branch $s(\tau)$ of zeros of
$G(s,\tau) = \hat n^\top(\varphi^\tau(\gamma(s)) - r)$ continued in $\tau$ from
$(s_r, 0)$ — one branch, no sides, no half-plane. `flowFrom` integrates in double
precision straight from the jet for exactly time $\tau$ (the last step cut; steps
capped at $0.02\,\Delta$ only when drawing the orbit); `correct` solves $G = 0$ for the
start by secant at fixed $\tau$ (rejects a slide over two neighbour gaps);
`continueTau` steps $\tau$ (first step $0.02\,\Delta / |f(r)|$, predictor along the last
$ds/d\tau$; a step must converge and move $b$ by at most $0.15\,\Delta$, else it halves;
below $10^{-9}$ of the first step the branch is broken; the step adapts so $\|b - r\|$
grows $\approx 0.05\,\Delta$ per step) until $\|b - r\| \ge \Delta$, the hit bisected in
$\tau$ (`bisectTau`); `pointAt` does the same bisection for a smaller $\Delta_i$. On the
z-axis of $W^s(O)$ the start stays at $r$ ($10^{-8}$) and $b = r \pm \Delta\,\hat z$
exactly, $\tau$ the analytic $\ln(z_b/z_r)/\beta$. `out` / the half-plane are only for
drawing $\mathcal{F}_r$ and for the particles' hits.
`recutRing`: $\Delta_i$ = the minimum over all solved points (and mids) of what they
reached, every point re-cut to it on its stored branch; neighbours whose $r_{new}$ are
more than maxSpread apart get the edge point midway solved too (`mids`). `renderBand`
draws the triangles $r \to r^+ \to r_{new}$, $r^+ \to r^+_{new} \to r_{new}$ (three
through a mid), their lines and the points. $\Delta_i$ joins the top-right readout (whose second line, `angleLatex`, gives the §3.2
angle bounds, all four shown in degrees — the panel sliders too — while settings and the
method keep radians); the
panel line gives $\tau$, $\|b - r\|$ and whether the branch broke or $\Delta_i$ fell
below $\Delta_{\min}$. Measured at the classic parameters (debug): a whole first ring (156 points, 8
mids) in 0.5 s at $\Delta_i = 1$, the next rings (164, 178 points) in about 0.1 s each.
A new mesh, a new $\Delta_{\min}/\Delta_{\max}$, or **Clear solutions** drops them.

**Accepted bands** are kept as a [[LevelSetSurface]] (`accepted`: per band its triangles,
where each vertex grew from, and $\eta$ at its outer level set), appended to `bandsMesh`
(`appendBand`) and drawn by `band_vertex.glsl` / `band_fragment.glsl` light blue and dark
blue by turns (`kStableBand`; orange-reds on $W^u$), so each geodesic circle shows.
**Save surface** (`saveSurface`, once rings are accepted and no run is going) writes them
with the parameters, K order, equilibrium, side and radius to
`lorenz_manifold_growth/data/surface_<O|Cp|Cm>_<s|u>_rho<rho>.lss` (one file per $\rho$), which [[lorenz-manifold-growth]]
grows ([[0011-precomputed-level-set-surfaces]]). Stored: $W^s(O)$ at the classic
parameters, Grow rings 400 from the default settings — it ran 98 rings to $\eta = 97$
(~1 min, debug, 2162 edge points at the end) and stopped at "r_new grows backward".

**The rest of the run** (growing ring after ring) comes as the user specifies it, and the next steps
(one iteration) come from them.

## How it works
Entered from the popup of [[lorenz-manifold-growth]] ("Learn geodesic level sets" on a
grown surface). That sim zooms onto the patch ($3.2\,r$ away) while the rest of its
scene fades, then calls `App::requestSwitch` ([[0009-sim-handoff-requestswitch]]) with
$\sigma, \rho, \beta$, `eq`, `side`, `order`, the bead size and fill opacity, its global
view and a bitmask of what it had grown (`loc`). `init` takes those, and the camera is
left where the zoom put it, so the switch shows no jump. Opened from the menu, it
frames $O$ itself (`frameCamera`). `handBack` (**Backspace** / "Back to the flow")
returns everything it was handed, plus the parameters as they are now and
`fromLevelSets`. The growth sim then restores what was grown and zooms back out while
its scene fades in.

`rebuild` sets the field, finds the equilibrium and builds $K$ for both sides. It shows
the 2-D side: the one handed over, else whichever is a surface. The radius is the
smaller trusted radius of the two sides (the same bisection on `maxRelDefect`), so it
is the growth sim's radius. `buildMesh` fills one dynamic [[Mesh]] with the triangles
(a fan around $v^*$, two per quad), their edges as line pairs (ring segments, spokes,
the quads' diagonals) and every vertex as a point; the last ring of points doubles as
the boundary loop. `render`: the bead, then the fill, the edges (`meshFade`), the
boundary and the vertices as round points, with no depth writes on the patch.
`update`: Backspace, **M**, **0** (screenshot), the mesh and $r$ fades, edge picking
(`pickEdge`: nearest edge vertex within `kPickPixels` on screen → `chooseEdge` →
`zoomToEdge`), camera speeds, the labels. `edgeFrame` builds $r$, $r^\pm$, $n$ and
$\mathcal{F}_r$'s frame from the mesh vertices (`verts`) and $K$; `renderEdgePoint`
draws them from a small second [[Mesh]]; `drawEdgeLabels` places each label off its
point along the on-screen direction of $\pm n$ or `out`, so they stay apart from any
view. While $r$ is shown, the $W^s_{\mathrm{loc}}$ label fades out.

## Modules attached
- [[InvariantManifold]] — $K$ of both sides; `maxRelDefect` for the trusted radius;
  `orthoBasis`/`toParam`/`eval` for the mesh vertices.
- [[LorenzSystem]] — equilibria, Jacobian, the quadratic part for $K$.
- [[LevelSetSurface]] — the accepted bands (`accepted`), written by **Save surface**.
- [[Mesh]] / [[Shapes]] — the patch buffer (fill + edges + points), the edge-point
  overlay ($\mathcal{F}_r$, chord, $n$, points), axes, unit sphere.
- [[Shader]] — the glyph pair (patch, edges, round vertices) and the bead pair, copied
  from lorenz_manifold_growth.
- [[Trajectory]] / [[TaylorIntegrator]] — the particles from the edge ball (trail +
  head), on [[LorenzSystem]]`::jet`; their state before/after each step for the
  crossing of $\mathcal{F}_r$.
- [[FlowKeys]] — arrow-key speed and direction of the particles, the corner readout.
- [[TaylorIntegrator]] — also stepped by hand in the BVP (`stepSize`, `evaluate`) to
  bisect crossings of $\mathcal{F}_r$ on each step's polynomial.
- [[LatexImage]] — maxSpread, minSpread, $\Delta_{\min}$, $\Delta_{\max}$, $\Delta_i$
  top-right; $r_{new}$; the $O$, $W^s_{\mathrm{loc}}(O)$, $r$, $r^\pm$,
  $\mathcal{F}_r$, $b_r(\tau = \dots)$ and
  $\|b_r(\tau) - r\|$ labels.
- [[Camera]] — orbit around $v^*$; `easeOrbitTo` for the recenter / reframe / zoom
  onto $r$; position, front and `up` re-placed in the sim's own orbit frame.
- [[App]] — handoff in and out (`handoff`, `requestSwitch`, `hasSim`), screenshots,
  `kRecenterEaseSeconds`.

## The math
- The patch: [[lorenz-stable-manifold]]; why a surface needs geodesic level sets to
  grow on: [[global-manifold-growth]].

## Controls / uniforms
**Backspace** back to the flow, **M** mesh, **Space** / arrows particles (pause, speed,
direction), **R** clear particles, **B** the whole edge flows, drag / click the green ball, **V** shoots $r$ → ball, hold **1** reframe the patch, **0**
screenshot, **H** panel ([[App]]). Panel: **Local manifold** (equilibrium combo, trusted
$r$, Back to the flow), **Mesh** (show mesh, rings, vertices per ring, counts),
**Particles** (Play/Pause, speed, reverse, `ball` angle,
Launch, Shoot r → ball + particle count, Clear particles, Whole edge), **Geodesic level sets** (maxSpread, minSpread, Delta_min, Delta_max), **Edge point**
(`r` slider — none or a vertex index, or click an edge vertex; Zoom to r, Clear;
Solve geodesic BVP — or **double-click** an edge vertex —, Solve all r — every vertex
not solved yet, from the one after the last solved (`lastSolved`) round the edge,
`kSolveAllBudgetMs` of each frame, with a count and Stop, one `recutRing` at the end —,
Clear solutions, a status line). Collapsed **Other settings** (camera; edge point — plane size,
plane opacity, zoom distance; particles — ball size, particle size, trail width;
**LaTeX** — labels on/off, parameters on/off, a size for each: equilibrium, $W_{loc}$,
$r$/$n$/$\mathcal{F}_r$, $b_r(\tau)$ and distance, parameters; look — fill / edge opacity, edge width, vertex
size, boundary width, mesh fade, bead size, labels, axes; Lorenz system — $\sigma$,
$\rho$, $\beta$, K order), **Settings**, **Screenshot**.
Explorer thumbnail: `docs/screenshots/learn_geodesic_level_sets.png`.
Uniforms as [[lorenz-flow]] (glyph pair + bead pair).

## Future
- The walkthrough proper, as the user specifies it after re-reading the algorithm:
  one iteration from $r$ and $\mathcal{F}_r$.
- Then geodesic level set growth itself, for `Grow W` on surfaces in
  [[lorenz-manifold-growth]].
