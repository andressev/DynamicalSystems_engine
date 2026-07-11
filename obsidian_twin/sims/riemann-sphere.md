---
type: sim
status: stable
code:
  - sims/riemann_sphere/RiemannSphereSim.cpp
  - sims/riemann_sphere/shaders/domainColorTemplate.glsl
---
# riemann-sphere

`sims/riemann_sphere/` — lib `sim_riemann_sphere`, standalone exe `riemann_sphere_standalone`.

Domain coloring of a typed complex function `f(z)` on the Riemann sphere.

## What is being simulated
A complex function `f(z)` visualized over the whole extended complex plane at once: the
sphere is the Riemann sphere, every point of it corresponds to a complex number (∞ at
the north pole), and the surface color encodes the value of `f` there.

## How it works
The UI string is parsed by [[Expression]], compiled to GLSL and hot-swapped into the
sphere shader at runtime (`Shader::fromSources`, see [[Shader]]). `init` builds the
sphere and axis meshes and constrains the camera; `update` advances `t`; `render` feeds
`u_time`/`u_mouse` uniforms and draws; `drawUI` owns the function text box and Apply.
The interesting code is `applyFunction()` in `RiemannSphereSim.cpp` and
`shaders/domainColorTemplate.glsl`.

## Modules attached
- [[Expression]] — parses the typed `f(z)`, generates the GLSL spliced into the shader.
- [[Shader]] — file-loaded at first, then hot-recompiled from sources on every Apply.
- [[Mesh]] — sphere + axis-line vertex buffers.
- [[Shapes]] — generates the sphere (100×100, r = 2.5) and axis geometry.
- [[Camera]] — shared orbit camera; constrained in `init` to stay outside the sphere
  (`orbitRadiusMin` = r + 0.2) and slow near the surface (`orbitSlowZone` = 2).

## The math
- [[stereographic-projection]] — sphere fragment positions → complex plane:
  $z = (p_x + i\,p_y)/(1 - p_z)$ (in `shaders/domainColorTemplate.glsl`).
- [[domain-coloring]] — $\text{hue} = \arg f(z)$,
  $\text{brightness} = |f(z)|/(1 + |f(z)|)$.
- The sphere is rotated −90° about X so its poles sit on the world Y axis; axis lines
  share the same model matrix.

## Controls / uniforms
- ImGui text box (`f(z) =`, Enter or Apply). Parse errors show in red and the previous
  shader keeps rendering — the sphere never goes blank while typing.
- Variables: `z`, `t` (seconds since sim start → `u_time`), `m` (cursor in [-1,1]² as a
  complex number → `u_mouse`, set every frame). Default `z^2 + m` follows the mouse.
- WASD rotate, Q/E zoom (orbit camera).

## Future
- (This sim replaced the old fixed degree-7 polynomial with coefficient sliders /
  follow-mouse mode, and `complexFragment.glsl` with it.)
