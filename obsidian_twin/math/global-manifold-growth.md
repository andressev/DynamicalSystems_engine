---
type: math
status: wip
code:
  - sims/lorenz_manifold_growth/LorenzManifoldGrowthSim.cpp
  - engine/src/InvariantManifold.cpp
---
# global-manifold-growth

## The idea
The parametrization method gives only a *local* piece $W_{loc} = v^* + K(B_r)$ of a
stable or unstable manifold ([[lorenz-stable-manifold]]), trusted out to the radius
where the invariance defect $|f(v^* + K(s)) - DK(s)\,\Lambda s|$ is still small. The
global manifold is what the flow makes of that piece:

$$W^u(v^*) = \bigcup_{t \ge 0} \varphi^t\big(W^u_{loc}\big), \qquad
W^s(v^*) = \bigcup_{t \le 0} \varphi^t\big(W^s_{loc}\big).$$

So growing $W$ means *flowing* $W_{loc}$ — forward in time for the unstable side,
backward for the stable one.

## Curves (dim 1)
A one-dimensional $W_{loc}$ is an arc through $v^*$ with two ends $v^* + K(\pm r\,e)$.
Its orbit is the arc itself plus the two orbits of the ends: every other point of the
arc flows along it (the arc is invariant), so nothing new appears between. The
global curve is therefore

$$W = W_{loc} \;\cup\; \{\varphi^t(v^* + K(r e)) : t \gtrless 0\} \;\cup\;
\{\varphi^t(v^* + K(-r e)) : t \gtrless 0\},$$

two trajectories — the **branches** — integrated with a good method (the Taylor
method here) and drawn as the whole curve they have traced. At the classic Lorenz
parameters the curves are $W^u(O)$ (the two branches that wind onto the attractor,
first around $C_\mp$) and $W^s(C_\pm)$ (both branches run off to infinity backward).

## Surfaces (dim 2) — why flowing is not enough
Flowing a circle of points on a two-dimensional $W_{loc}$ does **not** give a usable
mesh: the flow stretches along the strong direction and compresses across it, so the
points bunch into a few streaks and the surface between them is lost (on $W^s(O)$ the
backward flow is dominated by the fast eigenvalue $\approx -22.8$). Growing a surface
needs a construction that keeps the mesh regular — **geodesic level sets**
(Krauskopf & Osinga): the surface is built ring by ring, each new ring at a fixed
geodesic distance from the last, every new point found by flowing from the edge
into a plane perpendicular to the edge there.

**One step, as the user specified it** (built visually in [[learn-geodesic-level-sets]];
the growth sim's surface `Grow W` is still disabled). The edge's points keep neighbour
distances in $[\mathrm{minSpread}, \mathrm{maxSpread}]$. At an edge point $r$ with
neighbours $r^\pm$, the half-plane $\mathcal{F}_r$ through $r$ with normal
$n = \tfrac12(r^+ - r^-)$ (the half outside the edge). The **geodesic BVP**: find the
point of the edge whose orbit, in the manifold's own direction of time (backward on
$W^s$), meets $\mathcal{F}_r$ at $b_r(\tau)$ with
$$\|b_r(\tau) - r\| = \Delta_i .$$
(Graph over $s$ vs the paper's graph over $\tau$, and the z-axis vertex: [[geodesic-bvp]].)
It is solved as in the paper: continuation **in $\tau$** from the trivial solution
(start $= r$, $\tau = 0$, $b = r$), the start on the edge corrected at each $\tau$ so the
orbit's endpoint lies in $\mathcal{F}_r$ — one branch, to whichever side it goes; if it
cannot be continued, it broke, and the farthest distance reached is taken.
**Accept ring** makes the solved ring the edge: every $r_{new}$ and mid in order,
re-spread by maxSpread / minSpread, a polyline parametrized by arc length, each point's
outward direction the one it grew in; the band joins the surface and the next ring is
solved from it.
The solution is marked $r_{new}$. Neighbouring solutions are joined by the triangles
$r \to r^+ \to r_{new}$ and $r^+ \to r^+_{new} \to r_{new}$; where
$\|r_{new} - r^+_{new}\| > \mathrm{maxSpread}$ a point in between (the BVP at the edge
point midway) goes in. All points of a ring take the **minimum** of the distances
reached, so the ring is one geodesic level set; the step $\Delta_i$ stays between
$\Delta_{\min}$ and $\Delta_{\max}$.

## Where implemented
- `trustedRadius`, `buildLocal`, `startGlobal` and the branch integration in
  [[lorenz-manifold-growth]] (`LorenzManifoldGrowthSim.cpp`); $K$ itself in
  [[InvariantManifold]].

## Pitfalls
- The trusted radius is judged by the *relative* defect at sampled radii; a map that
  passes at every sample but misbehaves between them would go unnoticed (eight
  radii, 48 angles — fine for Lorenz).
- Backward branches of $W^s(C_\pm)$ reach $|v| \sim 10^7$ in finite flow time; the
  trajectory then stops (`Trajectory::diverged`) and the branch tip stays put.
- A branch's direction is the manifold's own; the flow's `reverse` does not retract
  it (a trail cannot shrink).
