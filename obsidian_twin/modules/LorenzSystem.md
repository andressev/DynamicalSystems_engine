---
type: module
layer: math
status: stable
code:
  - engine/include/engine/LorenzSystem.h
  - engine/src/LorenzSystem.cpp
  - tests/math_tests.cpp
---
# LorenzSystem

`engine/include/engine/LorenzSystem.h`, `engine/src/LorenzSystem.cpp`

## What it is
The Lorenz system $\dot x = \sigma(y-x)$, $\dot y = x(\rho - z) - y$, $\dot z = xy - \beta z$
as a plain value type ($\sigma, \rho, \beta$), plus everything the Hartman–Grobman sims
need about it: the field, its equilibria, the Jacobian there, and the quadratic part
$Q$ for [[InvariantManifold]]. Shared by [[lorenz-flow]] and the coming
`lorenz_equilibrium` (M2 of the Hartman–Grobman track, [[roadmap]]), so the two sims
cannot disagree on where $C_\pm$ are or what $Df$ is.

## Dependencies
- Engine: [[InvariantManifold]] (only for the `Quadratic` type)
- External: std-only

## How to use it
1. Member: `engine::LorenzSystem system;` — set `sigma`/`rho`/`beta` directly.
2. As an ODE right-hand side for [[Integrator]] / [[Trajectory]]:
   `rhs = [this](const double* x, double, double* dxdt) { system.eval(x, dxdt); };`
   `eval` is stateless, so it is safe from parallel loops (glyph sampling).
3. `equilibria()` — the origin always; $C_\pm = (\pm\sqrt{\beta(\rho-1)}, \pm\sqrt{\beta(\rho-1)}, \rho-1)$
   when $\rho > 1$, $\beta > 0$; each with a name (`"O"`, `"C+"`, `"C-"`).
4. `jacobian(x, J)` — $Df(v)$; feed it to [[Eigen3]] for the linearization or to
   `InvariantManifold::build` together with `LorenzSystem::quadratic()`.
5. `jet(x, order, c)` — the solution's Taylor coefficients through $v$ for
   [[TaylorIntegrator]]: $(k+1)\,c_{k+1}$ is the field with each product ($xz$, $xy$)
   replaced by its Cauchy product $\sum_j a_j b_{k-j}$ — exact, $O(N^2)$.
6. Reference sims: [[lorenz-flow]], [[lorenz-equilibrium]].

## Notes
- The field is quadratic, so $f(v^* + u) = Df(v^*)\,u + Q(u,u)$ holds *exactly* with the
  same $Q$ at every equilibrium ($\dot y$ gets $-u_x u_z$, $\dot z$ gets $u_x u_y$);
  `tests/math_tests.cpp` checks this and $f(v^*) = 0$.
- [[lorenz-manifold]] still carries its own private copy of the field and Jacobian
  (left alone for now; it can switch to this module when that sim is revisited).
