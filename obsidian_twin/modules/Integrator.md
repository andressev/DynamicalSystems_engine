---
type: module
layer: math
status: stable
code:
  - engine/include/engine/Integrator.h
  - engine/src/Integrator.cpp
---
# Integrator

`engine/include/engine/Integrator.h`, `engine/src/Integrator.cpp`

## What it is
Fixed-step ODE integrator for $\dot{v} = f(v, t)$ — Euler, midpoint RK2 or classic RK4
([[runge-kutta]]) — over any right-hand side a caller can express as a callable.
Dimension-agnostic up to `kMaxDim = 4`, stack-only inside (no allocation per step), and
deliberately ignorant of where $f$ comes from: a parsed [[VectorField]], a hard-coded
Lorenz system, or a future CUDA kernel's host-side twin. The M3 integrator in
[[roadmap]].

## Dependencies
- Engine: none (it is handed a `std::function`, not a field)
- External: std-only — `<functional>`, `<cmath>`

## How to use it
Attach = hold an `engine::Integrator` member (all defaults valid: RK4, `maxStep` 0.01).
1. Build the right-hand side once, in `init`:
   `Integrator::Field rhs = [this](const double* x, double t, double* dxdt) { field.eval(x, t, dxdt); };`
2. Per frame, advance the state by the frame's time budget:
   `integrator.advance(rhs, dim, x, t, dt)` — `x` and `t` are updated in place.
3. Expose `method` and `maxStep` in the UI if the sim wants them (`methodName()` gives
   the display string).
4. Reference sim: [[vector-field]], through [[Trajectory]] (which owns the state and
   the geometry and calls `advance` for you).

Interface:
- `step(f, dim, x, t, h)` — exactly one step of size `h`.
- `advance(f, dim, x, t, dt, maxSubsteps = 4096)` — splits `dt` into equal substeps of
  at most `maxStep` and returns how many it took.
- `method` (`Euler` / `RK2` / `RK4`), `maxStep`, `methodName(m)`.

## Notes
- **Negative `dt` runs the flow backwards** — every method is written so `advance(…, -h)`
  after `advance(…, +h)` returns to the start (to integration accuracy). That is how the
  "reverse" toggle in [[vector-field]] works, and it is what [[lorenz-manifold]] Phase 2
  would need for backward growth.
- `maxSubsteps` is a frame-hitch guard: a 2-second stall or a tiny `maxStep` caps the
  work per call instead of freezing the window. The state then lags real time — for
  visualization that is the right trade.
- Fixed step, no adaptive control and no error estimate. Stiff fields (or a step past
  the stability limit, [[runge-kutta]]) will drift or blow up; the caller is expected to
  notice — [[Trajectory]] does, with its escape check.
- Verified against $\dot{x} = x$ (RK4 hits $e$ to $2\times 10^{-10}$ at $h = 0.01$),
  against the unit circle closing after one turn, and for exact forward/backward
  round-tripping. No test target in the build yet — the harness compiled
  `Integrator.cpp` + `VectorField.cpp` + `Expression.cpp` alone, which is possible
  precisely because they are `math`-layer.
