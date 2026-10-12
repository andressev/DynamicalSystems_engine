---
type: math
status: stable
code:
  - engine/src/VectorField.cpp
  - engine/src/Trajectory.cpp
  - sims/vector_field/VectorFieldSim.cpp
---
# integral-curves

## The idea
A vector field assigns a vector to every point of space: $F : \mathbb{R}^n \to
\mathbb{R}^n$, read as "if you are here, this is the velocity you are given". An
**integral curve** (trajectory, orbit) is a path that obeys that instruction everywhere
— the answer to "let go of a particle here, where does it end up?". The picture of a
field is therefore two pictures that must agree: the *instantaneous* one (arrows at
sample points) and the *integrated* one (curves that are everywhere tangent to those
arrows). [[vector-field]] draws both at once, which is the whole point of the sim.

## The equations
An autonomous field gives the ODE
$$\dot{\mathbf{x}} = F(\mathbf{x}), \qquad \mathbf{x}(0) = \mathbf{x}_0,$$
and a time-dependent one $\dot{\mathbf{x}} = F(\mathbf{x}, t)$. In coordinates, the
sim's $\mathbb{R}^3$ case is
$$\dot{x} = f(x,y,z,t), \qquad \dot{y} = g(x,y,z,t), \qquad \dot{z} = h(x,y,z,t).$$

Picard–Lindelöf: if $F$ is locally Lipschitz, through every point there is exactly one
integral curve. Two consequences the visuals rely on: **curves never cross** (a crossing
in the picture is either a projection artifact or a numerical error), and the flow map
$\varphi_t$ is invertible — running $t \to -t$ retraces the same curve, which is what
the "reverse" toggle does.

**Equilibria** are the zeros $F(\mathbf{x}^*) = 0$; near one, $F(\mathbf{x}^* +
\mathbf{u}) \approx J\mathbf{u}$ with $J = DF(\mathbf{x}^*)$, and the eigenvalues of
$J$ classify the local picture — saddle, node, spiral, center (real/complex,
sign of the real part). The glyph field goes visually dark at an equilibrium because
$|F| \to 0$ there, which is exactly what the color ramp is for.

## Glyph conventions
A glyph grid samples $F$ on a lattice and draws one arrow per sample. Two scalars
compete for the arrow's length — magnitude and legibility — and the sim resolves it the
standard way:

- **Direction** is carried by the arrow: $\hat{F} = F/|F|$.
- **Magnitude** is carried by color, $s = \mathrm{clamp}(|F| / |F|_{\max}, 0, 1)$ through
  a cold→hot ramp, with $|F|_{\max}$ the max over the sampled grid (auto) or a typed
  value (manual).
- **Length is uniform by default** — a fixed fraction of the grid spacing — because
  fields of interest ([[lorenz-stable-manifold]], any field with an equilibrium) span
  orders of magnitude, and length-scaled arrows then collapse to invisible dots in
  exactly the region worth looking at. `length by magnitude` turns the classical
  quiver scaling back on, clamped to 5% of the cell so nothing vanishes.
- Arrows are anchored **tail-on-sample-point**, so a glyph reads "the flow leaves here
  going that way".
- The head is a **solid cone** (a filled triangle in the plane), shaded per facet. A
  line-drawn barb collapses into ambiguous scribble exactly when you fly close enough
  to care about one arrow; a shaded solid tells you which way it points from any angle.
- In R3 a glyph lattice is a *volume* of clutter, most of it far from anything the
  viewer is tracking. Fading glyph alpha with distance from the particles — down to a
  floor, never to nothing — keeps local structure readable without throwing the global
  picture away. The same is not needed in the plane, where a lattice is one layer deep.

Colouring the trajectories by $|F|$ with the same ramp makes the two pictures directly
comparable: a curve turns red exactly where it crosses the red arrows.

## Where it's implemented
- `engine/src/VectorField.cpp` — the field itself: one [[Expression]] per component,
  evaluated with `evalReal`.
- `engine/src/Trajectory.cpp` — integral curves: state, real-time stepping through
  [[Integrator]], polyline geometry.
- `sims/vector_field/VectorFieldSim.cpp` — `rebuildArrows()` builds the lattice and the
  glyph geometry (the two passes: sample + find $|F|_{\max}$, then emit).
- `sims/vector_field/shaders/fragment.glsl` — the cold→hot ramp both pictures share.

## Pitfalls
- **Sampling lies.** The glyph grid is a finite lattice; structure smaller than the
  spacing (a tight spiral, a thin separatrix) is simply not sampled. Curves do not have
  that problem — when the two disagree, believe the curve and refine the grid.
- **Equilibria are invisible in a normalized glyph field**: a normalized arrow at
  $|F| = 10^{-9}$ looks exactly like one at $|F| = 10$. The color ramp is the only cue,
  which is why "hide below" (drop arrows under a fraction of $|F|_{\max}$) exists.
- **A single outlier sets the scale.** Auto color scale takes the max over the grid, so
  one near-singular sample (e.g. a $1/r$ field sampled near the origin) washes the whole
  field blue. Switch to a manual max, or move the grid off the singularity.
- **Curves that cross in $\mathbb{R}^3$** are almost always the projection: uniqueness
  forbids a real crossing. Orbit the camera before believing one.
- **Time-dependent fields**: the arrows shown are $F(\cdot, t)$ at the current instant,
  and are *not* tangent to the drawn trajectory history — a curve's shape records where
  the field was pointing when it passed, not where it points now.
