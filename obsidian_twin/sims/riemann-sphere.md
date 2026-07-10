# riemann-sphere

`sims/riemann_sphere/` — lib `sim_riemann_sphere`, standalone exe `riemann_sphere_standalone`.

Domain coloring of a degree-7 complex polynomial
`f(z) = a0 + a1 z + … + a7 z^7` on the Riemann sphere.

## The math
- Sphere fragment positions are stereographically projected from the north pole onto the
  complex plane: `z = (p.x + i p.y) / (1 − p.z)` (in `shaders/complexFragment.glsl`).
- Domain coloring: hue = arg f(z), brightness = |f(z)| / (1 + |f(z)|).
- The sphere ([[Shapes]], 100×100, r = 2.5) is rotated −90° about X so its poles sit on
  the world Y axis; axis lines share the same model matrix.

## Controls / uniforms
- Coefficients `a[8]` are a uniform float array, set every frame from C++.
- **Follow mouse** (default on): a0..a7 are polynomial combinations of the normalized
  cursor position — the original learnOpenGL behavior.
- Sliders (a0..a7, −2..2) take over when follow-mouse is off; default f(z) = z.
- Camera: shared orbit camera ([[Camera]]) — WASD rotate, Q/E zoom.

## Future
M2 in [[roadmap]] replaces the fixed polynomial with parsed expressions compiled through
`Shader::fromSources` ([[Shader]]).
