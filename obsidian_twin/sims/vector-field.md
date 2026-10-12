---
type: sim
status: stable
code:
  - sims/vector_field/VectorFieldSim.cpp
  - sims/vector_field/shaders/vertex.glsl
  - sims/vector_field/shaders/fragment.glsl
---
# vector-field

`sims/vector_field/` — lib `sim_vector_field`, standalone exe `vector_field_standalone`.

Type a vector field on $\mathbb{R}^2$ or $\mathbb{R}^3$, see it as a grid of arrows
colored by magnitude, and drop trajectories into it that integrate in real time — with
the camera able to ride along on one.

## What is being simulated
The flow of $\dot{\mathbf{x}} = F(\mathbf{x}, t)$ for a field typed at runtime
([[integral-curves]]). Two pictures of the same object are on screen at once: the
*instantaneous* field (arrow glyphs on a lattice, direction by orientation, $|F|$ by
color) and the *integrated* field (trajectories, each a solution curve seeded by a
click). The R2 mode is the classic phase-plane portrait — the presets walk through
saddle, center, spiral sink, van der Pol limit cycle, damped pendulum. The R3 mode is
the same machinery in space, where the presets are the standard strange attractors
(Lorenz, Rössler, Thomas); seeding a curve in the Lorenz field and following it with the
camera is the sim's party trick.

This is M3 of [[roadmap]] — the ODE milestone the engine was being built toward, and
the first sim whose interesting work happens on the **CPU** rather than in a shader.

## How it works
`init` compiles one small shader pair, builds the axis lines ([[Shapes]]), and wraps the
field once as an ODE right-hand side:
`rhs = [this](const double* x, double t, double* dxdt) { field.eval(x, t, dxdt); }`.

`update(dt)` is the whole simulation:
1. R2 pan/zoom keys, and the click-to-seed test.
2. If playing, advance the field clock by `dt * speed` (negated when reversed) and call
   `traj->advance(integrator, rhs, h)` on every [[Trajectory]] — each integrates with
   [[Integrator]] and appends the points it walked through.
3. Camera follow, in one of two styles (R3): **orbit the head** (`camera.orbitCenter =
   traj->head()`, orbit keys still work around the moving point) or **ride the
   particle** — a chase rig that sits behind it along its own velocity and looks at it.
   In R2 the ortho window's center tracks the head instead.
4. Rebuild the glyph grid if anything it depends on changed — and every frame while the
   field mentions $t$ (`VectorField::usesTime()`), which is what makes a time-dependent
   field animate.

`rebuildArrows()` samples $F$ on the lattice in one pass (tracking $|F|_{\max}$ for the
color scale), then emits the glyph geometry in a second pass into **two** dynamic
[[Mesh]] buffers: shafts (`GL_LINES`, layout `{3,1,1}`) and solid heads
(`GL_TRIANGLES`, layout `{3,1,1,1}`). A head is a cone in R3 — 7 segments, sides plus a
base cap, 42 vertices — or a filled triangle in R2, and the shaft stops at the head's
base so it never pokes through the point. The extra float per head vertex is a lambert
term baked per facet on the CPU: an unshaded cone is just a silhouette, and the whole
reason for solid heads is that thin barbs turn to scribble when the camera is close.
The cheap `barbs` style is still there for dense grids.

In R2 the lattice covers the visible window and is snapped to multiples of the spacing,
so glyphs stay anchored to the plane while panning instead of crawling with the view; in
R3 it is a cube of `domain radius` around the origin, defaulting to a coarser `grid N`
(12 vs 26) because a solid head is an order of magnitude more geometry than a shaft.

**Head level of detail.** `grid N` reaches 50 in R3, which is $51^3 =$ 132651 arrows —
at a full 7-segment capped cone each, 134 MB of vertices. So `headLod()` picks the cone
by arrow count: 7 segments + base cap up to 20k arrows, 5 + cap to 50k, 4 uncapped to
90k, 3 uncapped beyond. Nothing is lost visually, because at that density a head is a
couple of pixels across. Measured at the top end (debug build): 132651 arrows, 1.19M
head vertices, 27.3 MB, and a 930 ms rebuild — which is why glyph-geometry sliders stop
rebuilding per frame and wait for the mouse release once the grid passes
`kLiveRebuildArrows` (20k).

`render` draws axes, arrows and trails with the same program, alpha-blended: every
vertex is position + one scalar (+ a gradient coordinate on glyphs, + a facet shade on
arrowheads), `u_useRamp` picks between the cold→hot ramp (magnitude, speed) and a flat
color (axes, particles, per-curve hues), and `u_alphaMode` picks where opacity comes
from — flat, the glyph's own tail→tip gradient, or point age along a trail (from
`gl_VertexID` against `Trajectory::drawFirst()`/`drawCount()`), so the oldest end of a
trail thins out instead of ending in a stub. Particles set `u_round`, which cuts the
point sprite down to a disc with a soft rim via `gl_PointCoord` and shades it like a
small sphere.

Two things keep a dense R3 grid from burying the curves. **Glyphs never write depth**
(`glDepthMask(GL_FALSE)` for the arrow pass): they blend and depth-test, but a faded
arrow in front of a curve cannot punch a hole in it, and a particle is never hidden
behind a glyph. And the **distance falloff**: the vertex shader takes the distance
$d$ from each glyph vertex to the *nearest* particle (up to 8 heads in `u_focus[]`, the
followed one first) and ramps the alpha continuously from full brightness at the
particle down to `minimum alpha` at `fully faded at`,
$$\alpha = \mathrm{lerp}\left(1,\ \alpha_{\min},\ t^{\,\gamma}\right), \qquad t = \mathrm{clamp}\!\left(\frac{d - r_0}{r_1 - r_0}, 0, 1\right),$$
with $r_0$ the optional `full-bright radius` (0 by default), $r_1$ = `fully faded at`
and $\gamma$ = `falloff curve` ($1$ linear, $<1$ concentrates the brightness on the
particle, $>1$ reaches further out). An earlier version used `smoothstep`, which holds
a flat plateau at both ends — no gradient where you are actually looking; the plain
power ramp is gradual the whole way. It is on by default in R3, off in R2 (one layer of
glyphs is not clutter, and the radii are fixed world units, so zooming out would dim
everything) — the checkbox offers it in both.

## Modules attached
- [[VectorField]] — parses the typed components into the field; `usesTime()` drives the
  per-frame glyph rebuild.
- [[Integrator]] — RK4/RK2/Euler stepping, `maxStep` substepping, reverse time.
- [[Trajectory]] — one per curve: ODE state, trail buffer, `draw()`/`drawHead()`,
  `head()` for the camera to follow.
- [[Expression]] — indirectly, under [[VectorField]]: same parser, `evalReal` path.
- [[Shader]] — one program from files (no codegen here: the field is evaluated on the
  CPU, see [[0006-cpu-field-sampling-dynamic-glyph-mesh]]).
- [[Mesh]] — two dynamic buffers for the glyph grid, shafts and solid heads (both
  rebuilt in place), plus a static one for the axes.
- [[Shapes]] — axis lines.
- [[Camera]] — orbit camera in R3 (`orbitCenter` retargeted for follow; `position`,
  `front` and `up` driven directly while riding, with `inputEnabled` cleared so orbit
  mode doesn't fight it); in R2 `inputEnabled = false` and the sim drives its own
  orthographic window. The sim sets `orbitAngularSpeed` and `speed` separately — the
  zoom rate scales with `domain radius`, the turn rate never does.

## The math
- Fields, integral curves, uniqueness, equilibria, and why glyph length is uniform while
  color carries magnitude: [[integral-curves]].
- The integration schemes, their orders, stability limits and failure modes:
  [[runge-kutta]].
- The R2 view is an orthographic window on the plane: $\texttt{ortho}(c_x \pm s a,\
  c_y \pm s)$ with $s$ the half-height and $a$ the aspect, so domain units stay square
  and pan speed scales with zoom.
- Click-to-seed in R3 is a ray/plane intersection: the cursor ray is unprojected through
  the inverse view-projection, and stopped on the plane through `orbitCenter` with the
  camera's forward as normal — the one depth a 2D click can unambiguously mean.
- The chase camera smooths its direction with a frame-rate independent exponential
  filter. With $\hat{v} = v/|v|$ and $\alpha = 1 - e^{-k \Delta t}$,
  $$\hat{d} \leftarrow \frac{(1-\alpha)\hat{d} + \alpha\hat{v}}{\lVert (1-\alpha)\hat{d} + \alpha\hat{v} \rVert}$$
  so the response rate $k$ (the `chase smoothing` slider) is in $1/\text{s}$ and does not
  depend on frame rate. Raw velocity would whip the view around at every
  Lorenz lobe transition. It falls back to world $+Z$ as the up vector when the flight
  direction is within ~14° of $+Y$, where `lookAt` degenerates.

## Controls / uniforms
**Field**: R2/R3 radio (switching reloads the default preset and clears the curves),
`dx/dt`, `dy/dt`, `dz/dt` boxes (Enter or Apply), preset combo. Variables $x, y, z, t$;
parse errors show in red and the previous field keeps drawing.

**Glyphs**: show arrows / axes, `grid N` (up to 80 in R2, 50 in R3), `arrow scale`,
`head:` solid / barbs, `head length` (fraction of the arrow taken by the cone),
`head width` (cone radius as a fraction of that), `length by magnitude`,
`hide below` (drop arrows under a fraction of $|F|_{\max}$), `domain radius` (R3),
`arrow alpha`, `auto color scale` / manual `max |f|`, `alpha gradient` (opacity at the
arrow's tail as a fraction of its tip — 1.0 turns the gradient off). The readout shows
the sampled $|F|_{\max}$, the arrow count, and — in R3 with solid heads — the cone
detail actually in use and the megabytes it costs. `brighten near particles` adds
`full-bright radius`, `fully faded at`, `minimum alpha` (the floor the ramp never goes
below, so distant structure stays faintly visible rather than disappearing) and
`falloff curve`.

**Flow**: Play/Pause, `reverse`, `Restart curves` (re-seeds every curve at its original
point, at the current field time), `speed`, integration `method`, `max step h`,
`trail points`, `min point spacing`, `trail width`, `particle size`, `color trails by
speed`, `fade tail` + `oldest point alpha`.

**Trajectories**: **click anywhere in the view to drop one**, or type a seed and press
Add; `Random seeds` scatters N in the visible window (R2) or the domain cube (R3);
`Clear all`. Per-curve rows show position, $|v|$, point count, a `follow` button and a
delete button; a diverged curve says so and gets a red head. `camera follows` + `target`
make the camera follow the chosen curve. `ride the particle` (R3) switches from
orbiting the head to flying behind it, with `chase distance`, `chase height` and
`chase smoothing`; with riding off, `orbit speed` (rad/s) and `zoom speed` (units/s)
tune the orbit camera.

**Navigation**: R3 — WASD orbit, Q/E zoom; while riding, Q/E pull the chase camera in
and out and W/S raise and lower it. R2 — WASD pan, Q/E zoom (the camera is off; the sim
owns the view).

**Uniforms** (all set in `render`): `view`, `projection`, `u_magScale`
($1/|F|_{\max}$ for the ramp), `u_pointSize`, `u_useRamp`, `u_color`, `u_alpha`,
`u_alphaTail`, `u_alphaMode` (+ `u_ageFirst`/`u_ageSpan` for trail fading), `u_round`,
`u_shadePoint`, `u_shadeMode` (arrowhead facet shading), and the falloff set
`u_focusCount`/`u_focus[]`/`u_fadeNear`/`u_fadeFar`/`u_fadeMin`.

## Future
- CUDA advection of many seeds at once ([[roadmap]] M4): the per-curve VBO inside
  [[Trajectory]] is already the interop point, and [[Integrator]]'s RK4 is written to
  port to a kernel.
- Streamlines / LIC as a denser alternative to glyphs, and nullcline or equilibrium
  detection (needs symbolic $\nabla$ in [[Expression]]).
- Flows on manifolds — the same [[VectorField]] + [[Integrator]] pair restricted to a
  surface — is the bridge to [[surface-renderer-ray-marched]] and to
  [[lorenz-manifold]] Phase 2, which wants exactly this RK4 for backward growth.
