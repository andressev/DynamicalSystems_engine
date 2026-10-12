---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/Trajectory.h
  - engine/src/Trajectory.cpp
---
# Trajectory

`engine/include/engine/Trajectory.h`, `engine/src/Trajectory.cpp`

## What it is
One integral curve of a vector field ([[integral-curves]]), integrated in real time and
drawn as a growing polyline. It owns three things that always travel together: the ODE
state ($x$ and $t$), the CPU point list, and the dynamic [[Mesh]] those points stream
into. A sim keeps a handful of these and calls `advance` each frame; nothing else in the
sim needs to know how a trail is buffered.

Curves in $\mathbb{R}^2$ are stored with $z = 0$, so one buffer layout and one shader
serve both dimensions.

## Dependencies
- Engine: [[Integrator]] (lower layer, `math`), [[Mesh]] (same layer, `gfx`)
- External: glad (through [[Mesh]]), glm

## How to use it
Attach = hold `std::vector<std::unique_ptr<engine::Trajectory>>` (or a single
`std::unique_ptr`). **Constructing one allocates a VBO**, so build them in `init` or
from a UI callback — never in the sim's constructor.

1. Seed: `auto t = std::make_unique<engine::Trajectory>(dim, seed, t0, maxPoints);`
   where `seed` is `dim` doubles and `t0` is the field time at the seeding moment.
2. Per frame: `traj->advance(integrator, rhs, dt)` — integrates and appends the points
   it walked through, with the point's speed $|f|$ as a per-vertex attribute.
3. In `render`, with your shader bound: `traj->draw()` (the trail, `GL_LINE_STRIP`) and
   `traj->drawHead()` (the newest point, `GL_POINTS` — needs `GL_PROGRAM_POINT_SIZE`).
   The vertex layout is `{3, 1}`: position + scalar, the same pair the glyph grid uses,
   which is why one shader colors both.
4. Camera follow is the caller's job. Orbiting the moving curve is one line —
   `camera.orbitCenter = traj->head()` — and riding it is a rig built from `head()` and
   `velocity()` (see [[vector-field]]; it must also clear [[Camera]]'s `inputEnabled`).
5. Reference sim: [[vector-field]].

Interface:
- `advance(integrator, f, dt)` — `dt < 0` runs the flow backwards ([[Integrator]]).
- `reset(seed, t0)` — back to a single point; keeps `dim` and the capacity.
- `draw()`, `drawHead()`.
- `head()`, `velocity()`, `speed()`, `time()`, `state()`, `dim()`, `size()`, `empty()`.
- `drawFirst()` / `drawCount()` — the vertex range `draw()` issues, so a shader can turn
  `gl_VertexID` into point age and fade the old end of the trail ([[vector-field]] does).
- `diverged()` — the curve left `kEscape` ($10^7$) or went non-finite.
- `minSpacing` — points nearer than this to the previous one are not recorded.
- `maxPoints()` / `setMaxPoints(n)` — trail length; resizing keeps the newest points.

## Notes
- **Divergence is terminal, not fatal.** On a non-finite or escaped state the last good
  point is restored, `diverged()` flips, and `advance` becomes a no-op — the blown-up
  curve stays on screen (drawn in red by [[vector-field]]) instead of poisoning the
  buffer with NaNs. Re-seed with `reset` to revive it.
- **Every substep is recorded.** `advance` splits $dt$ into substeps of at most
  `integrator.maxStep` and appends each one, so a higher time scale makes the trail
  denser per frame instead of one long chord per frame (which read as lag). New points
  are uploaded once per `advance`, in at most two ring runs (plus mirrors).
- **`minSpacing` protects the trail, not the accuracy**: integration substeps are
  unaffected, only committing is skipped. The newest point is a *live tip* moved in
  place every substep until it is `minSpacing` from the point before it, so the head
  and trail tip track the true position (drawing the last committed point instead made
  slow particles hop). Without it a curve creeping toward an
  equilibrium spends its whole buffer standing still.
- **Buffer strategy — a mirrored ring**: the [[Mesh]] holds `2 * maxPoints` vertices and
  every appended point is written twice, at slot `i` and at `i + maxPoints`. Any
  chronological window of `maxPoints` points is then *contiguous*, so a full trail is
  still one `glDrawArrays(GL_LINE_STRIP, drawFirst(), drawCount())` with no seam.
  The cost is 2x buffer memory and a few `glBufferSubData` calls per `advance`; the
  payoff is that **once full, each appended point retires exactly one point from the
  tail**. The earlier design (drop the oldest half, re-upload) was cheaper still but
  made the trail visibly vanish in blocks.
- `setMaxPoints` rebuilds the GL buffer, so it is a UI-time operation, not a per-frame
  one. It keeps the newest points in order, whether it grows or shrinks the trail.
- The trail buffer is exactly the kind of data CUDA will want to write directly
  ([[roadmap]] M4): the VBO is reachable through the [[Mesh]] it owns.
