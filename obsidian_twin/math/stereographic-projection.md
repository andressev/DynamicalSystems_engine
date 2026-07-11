---
type: math
status: stable
code:
  - sims/riemann_sphere/shaders/domainColorTemplate.glsl
---
# stereographic-projection

## The idea
Maps the unit sphere (minus the north pole) one-to-one onto the complex plane, so every
point of the sphere corresponds to a complex number and the north pole plays the role of
$\infty$. This is what makes the sphere in [[riemann-sphere]] the *Riemann* sphere: a
function $f(z)$ drawn on it is shown over the whole extended complex plane at once.

## The equations
Projecting from the north pole $N = (0, 0, 1)$ of a unit sphere, a surface point
$p = (p_x, p_y, p_z)$ lands on the plane at

$$z = \frac{p_x + i\,p_y}{1 - p_z}$$

Inverse intuition: points near the south pole map near $0$, the equator maps to the
unit circle, and $z \to \infty$ as $p \to N$.

## Where it's implemented
- `sims/riemann_sphere/shaders/domainColorTemplate.glsl` — per fragment, on the sphere's
  local-space position (the sphere from [[Shapes]] has poles on the local Z axis; the
  −90° model-matrix rotation that puts them on world Y happens *after*, so the formula
  stays in the clean form above).
- The projected $z$ is then fed to the generated `f(z)` (GLSL) and colored by
  [[domain-coloring]].

## Pitfalls
- The formula divides by $1 - p_z$: exactly at the north pole it blows up. That is
  mathematically correct ($z = \infty$), and [[domain-coloring]]'s brightness formula
  $|f|/(1+|f|)$ saturates instead of NaN-ing for large $|z|$, so no special-casing is
  needed — but any *new* use of the projection (CPU-side sampling, picking) must expect
  huge values near the pole.
- Apply the projection in the space where the poles are axis-aligned (local space here);
  projecting rotated world-space positions gives a rotated plane.
