---
type: math
status: stable
code:
  - sims/riemann_sphere/shaders/domainColorTemplate.glsl
  - engine/src/Expression.cpp
---
# domain-coloring

## The idea
A complex function $f : \mathbb{C} \to \mathbb{C}$ has a 2D input and a 2D output —
four dimensions, too many to graph directly. Domain coloring paints the *input* domain,
encoding the output value at each point as a color: the argument (angle) of $f(z)$
becomes hue, the modulus becomes brightness. Zeros show as dark points with the full
hue wheel around them; poles show as bright points; the number of times hue cycles
around a zero/pole reads off its order.

## The equations
For each point $z$ of the domain (here: the sphere via [[stereographic-projection]]):

$$\text{hue} = \arg f(z) \qquad\qquad \text{brightness} = \frac{|f(z)|}{1 + |f(z)|}$$

where $\arg$ lands in $(-\pi, \pi]$ and wraps around the color wheel, and the
brightness map is a Möbius-style compression — $0$ at zeros, $\to 1$ at poles, smooth
and NaN-free for every finite value.

## Where it's implemented
- `sims/riemann_sphere/shaders/domainColorTemplate.glsl` — the coloring `main()`, with
  placeholder lines where [[Expression]] splices the complex-arithmetic library and the
  generated `vec2 f(vec2 z)`.
- `engine/src/Expression.cpp` — the GLSL codegen that produces `f`.

## Pitfalls
- **`pow` NaNs at zero**: GLSL's exp/log-based complex power is undefined at $z = 0$,
  which would punch NaN holes exactly at the most interesting points (zeros). This is
  why [[Expression]] has the integer-power rule — $z^n$ for small integer $n$ compiles
  to repeated multiplication (`cPowi`) instead. Any new coloring/shading that computes
  powers must go through the same helpers.
- $\arg$ is discontinuous along the negative real axis; as a *hue* that's invisible
  (the color wheel closes), but using it for anything scalar (contours, height) will
  show a seam.
