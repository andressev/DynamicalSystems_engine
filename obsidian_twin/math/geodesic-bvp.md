---
type: math
status: wip
code:
  - sims/learn_geodesic_level_sets/LearnGeodesicLevelSetsSim.cpp
---
# geodesic-bvp

## The idea
One step of geodesic level sets (Krauskopf & Osinga 2003, §3.1): from a mesh point $r$
of the current ring $C_i$, find the point $b_r$ of the manifold in the hyperplane
$\mathcal{F}_r$ (through $r$, normal $\hat n$ from the neighbours) at distance
$\Delta_i$ from $r$. The ring of all $b_r$ is the next geodesic level set. Used by
[[global-manifold-growth]], built in [[learn-geodesic-level-sets]].

Both formulations below use the same $\tau$ — the integration time from the start on
$C_i$ to the point in $\mathcal{F}_r$. They differ in **which variable is independent**
when the solution curve is followed, and that is what decides whether the z-axis vertex
of $W^s(O)$ can be solved.

## The equations
Let $\gamma(s)$ parametrize $C_i$ (piecewise linear), $\gamma(s_r) = r$, and $\varphi^\tau$
the flow in the manifold's direction of time (backward on $W^s$). Every BVP solution is a
zero of

$$G(s,\tau) = \hat n^{\top}\big(\varphi^\tau(\gamma(s)) - r\big),$$

with $b_r = \varphi^\tau(\gamma(s))$. The trivial solution is $(s,\tau) = (s_r, 0)$. Near it
the zero set $\{G = 0\}$ is a curve through $(s_r, 0)$ in the $(s,\tau)$ plane. The two
partial derivatives there:

$$\frac{\partial G}{\partial s}(s_r,0) = \hat n^{\top}\gamma'(s_r) \neq 0
\quad(\hat n \text{ is nearly the tangent of } C_i),$$

$$\frac{\partial G}{\partial \tau}(s_r,0) = \hat n^{\top} f(r)
\quad(\text{the flow's component across } \mathcal{F}_r).$$

**The paper — graph over $\tau$.** Since $\partial G/\partial s \neq 0$, the implicit
function theorem gives a unique smooth $s(\tau)$ with $G(s(\tau),\tau) = 0$ for
$\tau \in (-\varepsilon, \varepsilon)$, always. Continue it in $\tau \ge 0$ and stop at the
first zero of

$$\Delta_i - \big\|b_r(\tau) - r\big\| .$$

**Ours so far — graph over $s$.** We slide the start $s$ and let $\tau(s)$ be the first
time the orbit from $\gamma(s)$ strictly crosses $\mathcal{F}_r$. Writing $\tau = \tau(s)$
needs $\partial G/\partial \tau \neq 0$, i.e. $\hat n^{\top} f(r) \neq 0$.

**Why only one side of $s_r$.** The branch leaves $(s_r, 0)$ with slope

$$s'(0) = -\frac{\partial G/\partial \tau}{\partial G/\partial s}
= -\frac{\hat n^{\top} f(r)}{\hat n^{\top}\gamma'(s_r)} .$$

The neighbourhood from the theorem is two-sided in $\tau$, but only $\tau \ge 0$ is usable
(the manifold's direction of time). If $\hat n^{\top} f(r) \neq 0$, the $\tau > 0$ half of
the curve moves $s$ to **one** side of $s_r$; the other side of $s_r$ carries the $\tau < 0$
half, which a forward-only integration never finds. That is why our code tries both sides
and keeps the one that works.

**The z-axis vertex.** On the axis $x = y = 0$ (invariant, inside $W^s(O)$), $f(r)$ points
along the axis, inside $\mathcal{F}_r$, so $\hat n^{\top} f(r) = 0$ and $s'(0) = 0$. In fact
$\varphi^\tau(r)$ stays on the axis for every $\tau$, so the branch is the vertical line

$$s(\tau) \equiv s_r, \qquad b_r(\tau) = \varphi^\tau(r) \ \text{climbing the axis}.$$

Not a graph over $s$: sliding $s$ off $s_r$ to either side leaves the solution curve at
once (the transverse dynamics near the axis is a saddle, $\det = \sigma(1-(\rho - z)) < 0$,
so neighbouring orbits peel away without returning), and the orbit of $r$ itself never
*strictly* crosses $\mathcal{F}_r$. In the paper's formulation it is just the trivial branch
continued — nothing special, which is why the paper never mentions it.

**First crossing vs endpoint.** The paper's $b_r(\tau)$ is the endpoint at time $\tau$ of
the continued branch, not the first crossing. Along a long branch the orbit from $s(\tau)$
may cross $\mathcal{F}_r$ earlier and come back; a first-crossing rule then jumps to another
piece of $\{G = 0\}$ (our "b moved at most $0.15\,\Delta$" check guards against that).

**Robust version — pseudo-arclength in $(s,\tau)$.** Following the curve in both variables
handles vertical tangents (the axis), horizontal ones and folds without special cases:
predictor along the tangent $t \perp \nabla G$, corrector solving

$$G(s,\tau) = 0, \qquad t^{\top}\big((s,\tau) - (s,\tau)_{\text{pred}}\big) = 0 .$$

## Where it's implemented
`LearnGeodesicLevelSetsSim.cpp`, the paper's graph over $\tau$ (since 2026-10-20; it
replaced the graph-over-$s$ version): `flowFrom` ($\varphi^\tau(\gamma(s))$ for exactly
$\tau$), `correct` (secant in $s$ at fixed $\tau$), `continueTau` (steps $\tau$, predictor
along $ds/d\tau$), `bisectTau` / `pointAt` (the $\tau$ with $\|b - r\| = \Delta$). Not
pseudo-arclength: a fold in $s$ shows up as a broken branch.

## Pitfalls
- Graph over $s$ fails where $\hat n^{\top} f(r) = 0$: the z-axis vertex of $W^s(O)$ (no
  solution found at any $\Delta$ — a smaller $\Delta$ does not help).
- With $\tau$ as the independent variable the half-plane / `out` test is unnecessary:
  continuity from $b_r(0) = r$ selects the branch; $r$ and $\hat n$ define $\mathcal{F}_r$.
- Graph over $\tau$ fails where the branch folds in $s$ ($\partial G/\partial s = 0$ later
  on); pseudo-arclength covers both.
