---
type: math
status: stable
code:
  - engine/src/Integrator.cpp
---
# runge-kutta

## The idea
Integrating $\dot{\mathbf{v}} = F(\mathbf{v}, t)$ numerically means guessing where the
curve goes over one step $h$ from samples of $F$. Euler takes one sample at the start
and walks straight — the error piles up fast. Runge–Kutta methods spend a few extra
evaluations *inside* the step, at states Euler-predicted from the earlier samples, and
combine them so that several terms of the Taylor error cancel. RK4 costs four
evaluations and buys two more orders of accuracy than midpoint RK2; for visualization
that is an enormous return, since a smooth-looking curve at a big step is worth more
than a cheap wrong one.

## The equations
One step from $(\mathbf{v}_n, t_n)$ of size $h$, as implemented:

**Euler** (order 1)
$$\mathbf{v}_{n+1} = \mathbf{v}_n + h F(\mathbf{v}_n, t_n)$$

**Midpoint RK2** (order 2)
$$k_1 = F(\mathbf{v}_n, t_n), \qquad k_2 = F\!\left(\mathbf{v}_n + \tfrac{h}{2}k_1,\ t_n + \tfrac{h}{2}\right), \qquad \mathbf{v}_{n+1} = \mathbf{v}_n + h\,k_2$$

**Classic RK4** (order 4)
$$\begin{aligned}
k_1 &= F(\mathbf{v}_n,\ t_n) \\
k_2 &= F\!\left(\mathbf{v}_n + \tfrac{h}{2}k_1,\ t_n + \tfrac{h}{2}\right) \\
k_3 &= F\!\left(\mathbf{v}_n + \tfrac{h}{2}k_2,\ t_n + \tfrac{h}{2}\right) \\
k_4 &= F\!\left(\mathbf{v}_n + h k_3,\ t_n + h\right) \\
\mathbf{v}_{n+1} &= \mathbf{v}_n + \tfrac{h}{6}\left(k_1 + 2k_2 + 2k_3 + k_4\right)
\end{aligned}$$

Local truncation error is $O(h^{p+1})$ and the accumulated (global) error over a fixed
time interval is $O(h^p)$, with $p = 1, 2, 4$. Measured on $\dot{x} = x$ over
$t \in [0,1]$ at $h = 0.01$: Euler $1.3\times10^{-2}$, RK2 $4.5\times10^{-5}$,
RK4 $2.2\times10^{-10}$ — the three orders, visible.

**Stability**, not accuracy, is what makes a step "too big": for the test equation
$\dot{x} = \lambda x$ the step is stable only while $h\lambda$ stays inside the method's
stability region ($|1 + h\lambda| < 1$ for Euler; RK4's region reaches to about
$h|\lambda| \approx 2.79$ on the negative real axis). A field with a fast contracting
direction (large $|\lambda|$) therefore needs a small $h$ no matter how smooth the
answer looks.

## Where it's implemented
- `engine/src/Integrator.cpp` — `Integrator::step` (the three tableaus, sharing the
  stack arrays) and `Integrator::advance` (substep splitting).
- [[Integrator]] is the module note; [[Trajectory]] is the caller that turns steps into
  geometry.

## Pitfalls
- **A frame is not a step.** `advance(dt)` splits the frame's elapsed time into substeps
  of at most `maxStep`, so the curve does not change shape when the frame rate does. The
  substep count is capped, so a long stall makes the simulation *lag* rather than
  spending seconds catching up.
- **RK4 is not symplectic.** Over long runs on a conservative field (a center, an
  undamped pendulum) the energy drifts slowly and closed orbits spiral. It is accurate,
  not structure-preserving — for long-time Hamiltonian work a symplectic method is the
  right tool.
- **Backward integration is exact in the method, not in floating point.** $-h$ retraces
  the curve to integration accuracy, but round-off (and any divergence in between) is
  not undone.
- **Stiff fields** (Rossler near its $z$ spike, anything with widely separated
  eigenvalues) blow up at a step that looked fine elsewhere. [[Trajectory]] catches the
  blow-up instead of letting NaNs propagate; the fix is a smaller `maxStep` or an
  adaptive method (not implemented).
- **Non-smooth right-hand sides** (`abs`, `sqrt` of something that changes sign) break
  the order assumptions — RK4 silently degrades to first order across the kink.
