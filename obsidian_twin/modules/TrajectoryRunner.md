---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/TrajectoryRunner.h
  - engine/src/TrajectoryRunner.cpp
---
# TrajectoryRunner

`engine/include/engine/TrajectoryRunner.h`, `engine/src/TrajectoryRunner.cpp`

## What it is
Advances a sim's [[Trajectory]] set each frame and profiles the three costs of drawing a
flow: **integrate** (CPU; optionally across all cores, one task per curve), **upload**
(new trail points to the GPU, GL thread), and **draw** (CPU submit time + GPU time via
[[GpuTimer]]). The readout is how we decide what to optimize next.

## Dependencies
- Engine: [[Trajectory]] (`integrate()` / `flush()`), [[Integrator]], [[TaylorIntegrator]], [[GpuTimer]]
- External: imgui (the readout), std `<execution>`

## How to use it
1. Member: `engine::TrajectoryRunner runner;`
2. `update`: collect `Trajectory*` of every curve and call
   `runner.advance(trajs, integrator, rhs, h)` instead of `advance()` per curve.
   With `parallel` on, `rhs` must be thread-safe ([[VectorField]] and [[LorenzSystem]]
   are; so is a lambda over `const` data).
   A Taylor overload, `advance(trajs, taylor, jet, h, sampleStep)` ([[TaylorIntegrator]]),
   serves curves on another method — call both in one frame (cloud on RK4, singles on
   Taylor); the timings are summed per frame and smoothed at `beginDraw()`.
   Both have a per-curve-step form, `advance(trajs, ..., dts)` (`dts[i]` for
   `trajs[i]`), still one parallel pass — for a time change per curve
   ([[lorenz-manifold-growth]]'s backward speed cap).
3. `render`: `runner.beginDraw()` / `runner.endDraw()` around the trail + head draws.
4. UI: a "Performance" section calling `runner.drawUI()` (toggle + smoothed timings).
5. Reference sims: [[vector-field]], [[diagonal-linear]], [[lorenz-flow]].

## Notes
- **Parallel is bit-identical to serial**: each curve's arithmetic is unchanged, only
  which core runs it — the trails look exactly the same.
- First measurement (2026-10-16, Debug, [[lorenz-flow]], 1000 curves × 3000-point
  trails): integrate 0.3 ms parallel vs 1.6 ms serial; upload ~1.2 ms; **draw ~5 ms CPU
  submit** (2000 draw calls + uniform changes) and **GPU 1 → 7+ ms as the trails fill**
  (~3M line vertices). Drawing, not integration, is the cost — next step is batching
  every trail into one buffer / one multi-draw, then trail-point budget.
- 2026-10-21, Debug, [[lorenz-manifold-growth]], a 1000-curve ball × 3000 points, vsync
  off: draw ~6–8 ms CPU (2 draws per curve) and ~5 ms GPU, integrate ~0.3 ms. The
  per-curve-step pass took the backward update from ~3 ms (capped curves one by one) to
  ~1 ms. Caching uniform locations in [[Shader]] (a hash map) measured no faster in
  Debug than `glGetUniformLocation` — tried and reverted.
