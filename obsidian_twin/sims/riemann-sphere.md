# riemann-sphere

`sims/riemann_sphere/` — lib `sim_riemann_sphere`, standalone exe `riemann_sphere_standalone`.

Domain coloring of a typed complex function `f(z)` on the Riemann sphere: the UI string
is parsed by [[Expression]], compiled to GLSL and hot-swapped into the sphere shader at
runtime (`Shader::fromSources`, see [[Shader]]).

## The math
- Sphere fragment positions are stereographically projected from the north pole onto the
  complex plane: `z = (p.x + i p.y) / (1 − p.z)` (in `shaders/domainColorTemplate.glsl`).
- Domain coloring: hue = arg f(z), brightness = |f(z)| / (1 + |f(z)|).
- The sphere ([[Shapes]], 100×100, r = 2.5) is rotated −90° about X so its poles sit on
  the world Y axis; axis lines share the same model matrix.

## Function input
- ImGui text box (`f(z) =`, Enter or Apply). Parse errors show in red and the previous
  shader keeps rendering — the sphere never goes blank while typing.
- Variables: `z`, `t` (seconds since sim start → `u_time`), `m` (cursor in [-1,1]² as a
  complex number → `u_mouse`, set every frame). Default `z^2 + m` follows the mouse.
- `shaders/domainColorTemplate.glsl` holds the projection + coloring with
  `//__COMPLEX_LIB__` and `//__F__` placeholder lines; `applyFunction` in
  `RiemannSphereSim.cpp` splices `Expression::glslComplexLib()` and the generated
  `vec2 f(vec2 z)` and recompiles. (This replaced the old fixed degree-7 polynomial
  with coefficient sliders / follow-mouse mode, and `complexFragment.glsl` with it.)
- Camera: shared orbit camera ([[Camera]]) — WASD rotate, Q/E zoom; constrained in `init`
  to stay outside the sphere (`orbitRadiusMin` = r + 0.2) and slow near the surface
  (`orbitSlowZone` = 2).
