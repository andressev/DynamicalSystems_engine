---
type: math
status: stable
code:
  - sims/surface_renderer_ray_marched/shaders/raymarchTemplate.glsl
---
# raymarching-level-sets

## The idea
Render the level set $f(x,y,z)=c$ with no mesh: every pixel independently casts a ray
into the world and finds its first crossing of the surface. Since a typed $f$ is **not**
a signed distance field (no eikonal property $|\nabla f|=1$, its value says nothing
about distance), sphere tracing is unsafe — we use fixed-step sampling + bisection on
the sign of $F = f - c$ instead.

## The equations
**Ray from a pixel** (unprojection). For NDC(normalized device coordinates) coords $q\in[-1,1]^2$, with
$M^{-1}=(PV)^{-1}$ and homogeneous divide:
$$r_o = M^{-1}(q,-1,1)\big/w, \qquad r_{far} = M^{-1}(q,+1,1)\big/w, \qquad r_d = \frac{r_{far}-r_o}{|r_{far}-r_o|}$$

**Clip to the domain sphere** $|p| = R$ (uniform `u_range`). Substituting
$p = r_o + t\,r_d$ gives $t^2 + 2bt + cc = 0$ with $b = r_o\cdot r_d$,
$cc = r_o\cdot r_o - R^2$, so
$$t_{0,1} = -b \mp \sqrt{b^2 - cc}$$
No real roots → the pixel can never see the domain → discard ($f$ evaluated 0 times).
$t_0$ is clamped to $0$ when the camera is inside the sphere.

**March + bisection.** Sample $F$ at $N=220$ equal steps on $[t_0, t_1]$. A sign change
$F(t_{i-1})\,F(t_i) \le 0$ brackets a root (intermediate value theorem); 14 bisection
iterations shrink the bracket by $2^{14}$, sub-pixel accuracy. First crossing = nearest
surface, so self-occlusion is automatic.

**Shading normal** — the level-set normal is the normalized gradient, by central
differences with $\varepsilon = 0.0015R$:
$$n = \frac{\nabla F}{|\nabla F|}, \qquad \partial_x F \approx \frac{F(p+\varepsilon e_x)-F(p-\varepsilon e_x)}{2\varepsilon}$$

**Depth handshake.** The hit $p$ is re-projected with the same $PV$ the axis geometry
used and written to `gl_FragDepth` $= \tfrac{1}{2}(clip_z/clip_w)+\tfrac{1}{2}$, so
rasterized lines and raymarched pixels occlude each other correctly.

## Where it's implemented
`sims/surface_renderer_ray_marched/shaders/raymarchTemplate.glsl` — `main()` does ray,
clip, march, bisection, normal, depth; the parser-generated `fc` (from [[Expression]])
supplies $f$. Uniforms fed per frame in `SurfaceRendererRayMarchedSim::render`.
Used by [[surface-renderer-ray-marched]].

## Pitfalls
- **Thin features**: two crossings inside one step leave the sign unchanged — the pixel
  misses the surface (shimmer). Tighter `u_range` = finer steps; certified fix is
  interval arithmetic (proposal §M5).
- **Cost profile**: rays missing the sphere are ~free; rays through the domain that hit
  nothing are the worst case (all $N$ samples, no early break).
- Forgetting the homogeneous divide by $w$ after unprojection breaks the ray silently.
- $c$ at a critical value of $f$: the level set is singular there ($\nabla f = 0$
  somewhere on it) — normals degenerate at those points for exactly one slider value;
  harmless in practice.
