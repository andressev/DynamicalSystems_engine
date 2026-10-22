---
type: module
layer: math
status: stable
code:
  - engine/include/engine/TaylorIntegrator.h
  - engine/src/TaylorIntegrator.cpp
  - tests/math_tests.cpp
---
# TaylorIntegrator

`engine/include/engine/TaylorIntegrator.h`, `engine/src/TaylorIntegrator.cpp`

## What it is
The Taylor series method for $\dot v = f(v,t)$: at each step the solution's polynomial
$v(t+s) = \sum_{k=0}^{N} c_k s^k$ is built to order $N$ from the field's **jet** (its
normalized derivatives along the solution), and the step is chosen from the decay of
the last two coefficients (Jorba & Zou 2005):
$h = 0.9 \min_{k \in \{N-1, N\}} (\varepsilon / \lVert c_k \rVert_\infty)^{1/k}$,
$\varepsilon$ = `tolerance` $\cdot \max(1, \lVert v \rVert_\infty)$. For a polynomial field
the jet is an exact recurrence, so this is the accurate integrator for the few curves
that must be right — the singles in [[lorenz-flow]] / [[lorenz-equilibrium]] — while the
clouds stay on [[Integrator]]'s RK4. See [[runge-kutta]] for the RK side.

## Dependencies
- Engine: none
- External: std-only

## How to use it
1. Member: `engine::TaylorIntegrator taylor;` (+ a `Jet`, e.g.
   `[this](const double* x, double, int n, double* c) { system.jet(x, n, c); }` —
   [[LorenzSystem]]`::jet`).
2. Knobs: `order` (default 20, up to `kMaxOrder` = 60), `tolerance` (default
   $10^{-14}$), `maxSteps` per call.
3. Usually through [[Trajectory]]'s Taylor `integrate(taylor, jet, dt, sampleStep)` /
   [[TrajectoryRunner]]'s matching `advance`, which put the dense-output samples in the
   trail. Directly: `advance(jet, dim, x, t, dt, sampleStep, sample)` — adaptive steps,
   the last landing exactly on $t + dt$, `sample(x, x', t)` at every step end and at
   most `sampleStep` apart inside long steps (the polynomial evaluated, no field calls).
4. `stepSize(c, dim)`, `evaluate(c, dim, order, s, x, dx)` — the two pieces, exposed.

## Notes
- Order and tolerance are separate: the tolerance sets the accuracy, the order how few
  (and how expensive) the steps are. Low order at a tight tolerance means tiny steps —
  `maxSteps` caps a frame. $N \approx -\tfrac12 \ln \varepsilon$ ($\approx 16$ for
  $10^{-14}$) is the classic sweet spot; 20 is a safe default.
- Tests (`tests/math_tests.cpp`): harmonic oscillator to $t = 20$ in 16 steps, error
  $4\cdot10^{-16}$, and back to the seed; Lorenz from $(1,1,1)$ to $t = 2$ in 46 steps,
  matching RK4 at $h = 10^{-4}$ (20000 steps) to $5\cdot10^{-13}$.
- Chaos still wins eventually: on the Lorenz attractor even $10^{-14}$ per step
  separates from the true orbit after $t \approx 30$–$40$.
