---
type: sim
status: wip
code:
  - sims/surface_renderer_ray_marched/SurfaceRendererRayMarchedSim.cpp
  - sims/surface_renderer_ray_marched/shaders/raymarchTemplate.glsl
---
# surface-renderer-ray-marched

`sims/surface_renderer_ray_marched/` — lib `sim_surface_renderer_ray_marched`,
standalone exe `surface_renderer_ray_marched_standalone`.

Raymarched graphing calculator for implicit surfaces: type $f(x,y,z)$ and a level $c$,
see the level set $f^{-1}(c)$ — a 2-manifold in $\mathbb{R}^3$ (when $c$ is a regular
value, by the preimage theorem).

## What is being simulated
The level set $f(x,y,z) = c$ of a typed real function of three variables. Dragging $c$
sweeps the family of level sets, and the surface changes topology exactly at the
critical values of $f$ (Morse theory). No mesh exists — every pixel independently finds
its own intersection with the surface, so this renders the *exact* surface at whatever
the screen resolution is. This is the "reference renderer" mode from the method
comparison in `docs/level-set-meshing-proposal.html`; a mesh-extracting sibling sim
(marching cubes, CUDA) is the planned follow-up that arrows/flows will attach to.

## How it works
`init` builds a fullscreen quad ([[Shapes]] plane) and the axis lines, loads the two
shader sources, and compiles the default function. `applyFunction` parses the text with
[[Expression]] over variables $x, y, z, t$, splices the generated code and the complex
lib into `shaders/raymarchTemplate.glsl`, and hot-swaps the program via
`Shader::fromSources` — same pattern as [[riemann-sphere]]. Real coordinates enter the
complex evaluator as `vec2(v, 0.0)` and the real part of the result is the field value.

Per frame, `render` draws the axes normally, then the quad: the fragment shader
unprojects the pixel through `u_invViewProj` into a world ray, clips it to a bounding
sphere (`u_range`), fixed-step marches $F = f - c$ looking for a sign change, bisects
the bracketing interval, shades from the central-difference gradient, and writes real
`gl_FragDepth` at the hit so the axis lines intersect the surface correctly; misses
`discard`.

## Modules attached
- [[Expression]] — parses $f(x,y,z)$, generates the GLSL spliced into the raymarcher.
- [[Shader]] — axis shader from files; raymarch shader hot-recompiled on every Apply.
- [[Mesh]] — fullscreen quad + axis-line vertex buffers.
- [[Shapes]] — plane (as the fullscreen quad) and axis geometry.
- [[Camera]] — orbit camera; only its matrices are used (the quad ignores MVP; rays are
  built by unprojection). `init` re-enables `inputEnabled` defensively.
- [[SolutionCurve]] — the flow-demo particle: its field evaluates a CPU copy of the
  parsed $f$ (an ambient swirl tangent-projected via $V - (V\cdot n)n$, plus a
  $-F\,\nabla f/|\nabla f|^2$ feedback term that keeps RK4 glued to the level set).
  Drawn after the quad — `gl_FragDepth` makes it composite correctly.

## The math
- [[raymarching-level-sets]] — the full method: pixel → ray (unprojection), bounding
  sphere clip, fixed-step + bisection root finding, gradient normals, depth write.
- Level sets: $c$ regular $\Rightarrow$ $f^{-1}(c)$ is a smooth 2-manifold; almost every
  $c$ is regular (Sard). Topology changes when $c$ crosses a critical value.
- Method survey and citations: `docs/level-set-meshing-proposal.html`.

## Controls / uniforms
- Preset combo (tangle cube / torus / rippled sphere / gyroid / klein bottle) — sets
  $f$, $c$, $a$ and the domain radius; hand-editing $f$ flips it back to "custom".
- `f(x,y,z) =` text box (Enter or Apply); parse errors show in red, previous shader
  keeps rendering.
- `c` drag — the level; `a (distort)` drag — free parameter available as variable `a`
  in $f$ and $V$. Both are uniforms, so sweeping them never recompiles.
- `domain radius` drag — bounding-sphere radius `u_range` the rays search inside.
- Variables: `x, y, z`, `t` (seconds → `u_time`), `a` (→ `u_a`). Default preset: the
  tangle cube $x^4+y^4+z^4-a(x^2+y^2+z^2)$ at $a=5$, $c=-11.8$ (topology changes at
  $0, -6.25, -12.5, -18.75$).
- Uniforms set per frame in `render`: `u_viewProj`, `u_invViewProj`, `u_c`, `u_range`,
  `u_time`, `u_a`.
- Flow demo: checkbox spawns a [[SolutionCurve]] particle flowing on the surface;
  the ambient field $V(x,y,z,t)$ is typed as three component expressions (default
  swirl $(-y,\,x,\,0.4)$), parsed by [[Expression]] and tangent-projected; trail
  toggle, RK4 step-size drag, reset button.
- Follow cam: rides the particle — position $p + h\,n - 0.6h\,d$ (normal offset $h$
  adjustable, $d$ = motion direction), looking ahead along $d$, up $= n$, all
  exponentially smoothed. Takes the camera via `Camera::inputEnabled = false`;
  orbit control returns when toggled off (or when the demo is).
- WASD rotate, Q/E zoom (orbit camera).

## Future
- Symbolic gradient from [[Expression]] → exact normals (and later Hermite data /
  tangent projection for flows).
- Mesh-extraction sibling sim (marching cubes → [[Mesh]], then CUDA per [[roadmap]] M4);
  this sim stays as the ground-truth comparison view.
