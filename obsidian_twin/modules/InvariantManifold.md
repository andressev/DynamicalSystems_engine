---
type: module
layer: math
status: stable
code:
  - engine/include/engine/InvariantManifold.h
  - engine/src/InvariantManifold.cpp
  - tests/math_tests.cpp
---
# InvariantManifold

`engine/include/engine/InvariantManifold.h`, `engine/src/InvariantManifold.cpp`

## What it is
The local stable or unstable manifold of an equilibrium $x^*$ of a **quadratic** field
$f(x^* + u) = Au + Q(u,u)$, as a Taylor map $K:\mathbb{R}^k\to\mathbb{R}^3$
($k = 1$ or $2$) built by the parametrization method ([[lorenz-stable-manifold]]). $K$
conjugates the linear flow on the parameter to the true flow:

$$\varphi^t\big(x^* + K(s)\big) = x^* + K\big(e^{\Lambda t}s\big).$$

It is the [[lorenz-manifold]] recursion generalized to any equilibrium, either side,
and a complex-conjugate eigenpair. In the Hartman–Grobman sims it *is* $h^{-1}$
restricted to $E^s$ and $E^u$ (see [[roadmap]]).

## Dependencies
- Engine: [[Eigen3]] (same layer)
- External: std-only

## How to use it
1. Member: `engine::InvariantManifold Ws, Wu;`
2. On parameter change: `Ws.build(A, Q, Side::Stable, order)` with $A = Df(x^*)$ and
   $Q$ as a tensor `Q[c][i][j]` ($Q(u,v)_c = \sum_{ij} Q_{cij} u_i v_j$). For Lorenz
   $Q$ is the same at every equilibrium: `Q[1][0][2] = -1` ($-u_x u_z$),
   `Q[2][0][1] = 1` ($u_x u_y$). `build` returns false (`error()`) when the side is
   0- or 3-dimensional or $A$ is degenerate.
3. Map a point $\theta \in E$ (relative to $x^*$) to the manifold:
   `x = x* + eval(toParam(theta))`. Flow it linearly: `flowParam(s, t)` $= e^{\Lambda t}s$.
   `linear(s)` $= Bs$ is the inverse of `toParam` on $E$.
4. Validity: `maxRelDefect(r)` — worst relative invariance defect on the sphere of
   radius $r$ in $E$; `relDefectBelow(r, tol)` — the same verdict as `maxRelDefect(r) < tol`, stopping at the first sample over (for radius searches); `minDivisor()` — the smallest $|\mu - \lambda_j|$ met, the
   resonance warning.
5. Reference: `tests/math_tests.cpp`; first sim user will be the Lorenz equilibrium sim.

Interface: `build`, `valid`, `error`, `dim`, `order`, `complexPair`, `eigenvalue(i)`,
`eigen()`, `eval`, `linear`, `toParam`, `flowParam`, `basis(i)`, `orthoBasis(i)`,
`defect`, `maxRelDefect`, `relDefectBelow`, `minDivisor`, `coeff(m, j)`.

## Notes
- **Coordinates.** $s$ is chosen so $DK(0)$ is the identity on $E$: $K(s) = Bs + O(|s|^2)$
  with $B$ = real eigenvectors, or $\{\mathrm{Re}\,v, \mathrm{Im}\,v\}$ for a pair. In
  the complex case the Taylor variables are $(z, \bar z)$ with $z = (s_0 - i s_1)/2$,
  coefficients satisfy $p_{jm} = \overline{p_{mj}}$, and $K$ is the real part.
- **Homological equation** per multi-index: $(A - \mu I)\,p_{mj} = -[Q(K,K)]_{mj}$ with
  $\mu = m\lambda_0 + j\lambda_1$; solved by complex Gaussian elimination.
- **The defect is cheap and exact-in-form**: $\frac{d}{dt}K(e^{\Lambda t}s)|_0 =
  \sum (m\lambda_0 + j\lambda_1)\,p_{mj}\,w^{(m,j)}$, so no finite differences.
- **Pitfall: near-resonance at $C_\pm$.** At $\sigma=10,\rho=28,\beta=8/3$ the unstable
  pair at $C_\pm$ is $0.094 \pm 10.19i$, close to Hopf; $\mu - \lambda \approx (d-1)\cdot0.094$
  gives small divisors (min 0.19 at order 12). Measured relative defect: $10^{-8}$
  at $|\theta|=1$, $6\cdot10^{-5}$ at 2, $0.44$ at 4 — keep $r \lesssim 2$ there. The
  origin is benign ($10^{-13}$ at $|\theta| = 2$).
- Matches [[lorenz-manifold]]'s coefficients at the origin to $4\cdot10^{-15}$ after
  rescaling eigenvectors, and RK4 conjugacy checks pass to $\sim10^{-12}$.
