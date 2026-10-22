---
type: sim
status: wip
code:
  - sims/lorenz_flow/LorenzFlowSim.cpp
  - sims/lorenz_flow/shaders/vertex.glsl
  - sims/lorenz_flow/shaders/fragment.glsl
  - sims/lorenz_flow/shaders/equilibrium_vertex.glsl
  - sims/lorenz_flow/shaders/equilibrium_fragment.glsl
---
# lorenz-flow

`sims/lorenz_flow/` — lib `sim_lorenz_flow`, standalone exe `lorenz_flow_standalone`.

The global picture of the Lorenz flow — arrow field, trajectories, and the equilibria as
red beads you can hover and click to fly onto. Mode 1 of the Hartman–Grobman track.

## What is being simulated
$\dot x = \sigma(y-x)$, $\dot y = x(\rho-z) - y$, $\dot z = xy - \beta z$ with live
$\sigma, \rho, \beta$. Its equilibria are the origin $O$ and, for $\rho > 1$,
$C_\pm = (\pm\sqrt{\beta(\rho-1)}, \pm\sqrt{\beta(\rho-1)}, \rho - 1)$. Each is a red bead;
hovering shows its linearization $Df(v^*)$ — eigenvalues and type (at the classic
parameters $O$ is a saddle with $\dim E^s = 2$, $C_\pm$ are saddle-foci with a complex
unstable pair $\approx 0.094 \pm 10.19i$). Clicking eases the camera onto it and, when
the zoom lands, hands off to [[lorenz-equilibrium]]. The local
picture inside a ball $V$ around $v^*$ — what Hartman–Grobman says there — is the
separate [[lorenz-equilibrium]] sim; this sim stays global so neither carries the
other's UI.

**Handoff** ([[0009-sim-handoff-requestswitch]]). `focusOn` sets `handingOff` when
`lorenz_equilibrium` is registered; over the second half of the zoom everything but the
focused bead fades out (`currentSceneFade()`: arrows, trails, heads, the other beads and
labels), and when the zoom lands `handOff()` requests the switch with `sigma/rho/beta`,
`eq`, `beadRadius` and the global view `globalCx/Cy/Cz/R`. Coming back, `init` finds
`eq` in the handoff: same parameters, camera left on $v^*$, the bead already shrunk and
lit, then `backToGlobal()` eases out to the handed-back view while the scene fades in
(`returning`). Trajectories don't survive the round trip. The standalone has no
`lorenz_equilibrium`, so there the zoom lands and stays (Back / Ball here as before).

## How it works
`init` loads `settings.txt`, builds the axis and a unit sphere mesh, and sets the ODE
right-hand side to [[LorenzSystem]]`::eval`. `applyParams()` (any $\sigma/\rho/\beta$
change) re-reads the equilibria, runs [[Eigen3]] on each Jacobian, classifies it
(`classify()`: sink / source / saddle, "-focus" with a complex pair, or non-hyperbolic)
and marks the glyphs and the LaTeX overlay dirty.

`update`: advance every [[Trajectory]]; pick the bead under the cursor
(`pickEquilibrium()`: ray from the cursor, ray–sphere test with a radius of
$\max(1.6\,r_{bead}, 0.025 \cdot \text{distance})$ so far beads stay easy to hit, nearest
wins); ease each bead's `glow` toward 1 while hovered; on click either `focusOn(i)` (bead)
or drop a single (empty space). `updateCamera()` runs the **eased zoom**: centre lerped
and radius interpolated in log space under a quintic smoothstep over `zoom duration`;
the global view is remembered on the way in and `Back to global view` (or **Backspace**
— Escape belongs to [[App]]) flies back to it. The focused bead shrinks (to 0.4 of `bead size`, eased), keeps glowing and drops out of picking — it no longer reacts to the mouse. While focused the orbit centre stays on
$v^*$ even as $\rho$ is dragged. Since [[Camera]]'s orbit position is computed in
`processInput` (before `update`), `updateCamera` recomputes position/front itself so
the frame shows this frame's camera.

`render`: beads first (opaque, `equilibrium_*.glsl`: lambert + spec + a warm Fresnel rim
scaled by `u_glow`), then axes, glyphs and trails with the [[vector-field]] glyph shader
pair, then the **halos** — a larger shell per glowing bead, additive, no depth writes,
alpha $\propto (n\cdot v)^2$ so it is a soft ball of light, pulsing.

## Modules attached
- [[TaylorIntegrator]] — singles on the Taylor method (default: order 20, tolerance
  $10^{-14}$), the cloud on RK4; `Integrator` section: one method combo for each, order
  + tolerance shown when Taylor is picked, `max step h` = trail spacing for Taylor.
- [[TrajectoryRunner]] — advances every curve (integrate on all cores, then upload) and
  times integrate / upload / draw; readout under Other settings → Performance.
- [[FlowKeys]] — arrow-key speed and direction.
- [[LorenzSystem]] — the field (ODE rhs, glyph sampling), equilibria, Jacobians.
- [[Eigen3]] — eigenvalues + type of each linearization, for the tooltip and panel.
- [[ArrowGlyphs]] — the instanced field grid, sampled in parallel on a cube of
  `domain radius` around `domain center z`.
- [[Integrator]] / [[Trajectory]] — RK4 balls and click-dropped singles.
- [[Shader]] — the glyph pair (copied from [[vector-field]]) and the bead pair.
- [[Mesh]] / [[Shapes]] — axis lines, unit sphere for beads and halos.
- [[LatexImage]] — the system with live numbers top-right; $O$, $C_+$, $C_-$ labels.
- [[Camera]] — orbit camera; the sim drives `orbitCenter`/`orbitRadius` for the zoom.

## The math
- Flows, equilibria, linearization: [[integral-curves]]. Stepping: [[runge-kutta]].
- The bead's type is read off the eigenvalues of $Df(v^*)$; whether that linear picture
  is *trustworthy* near $v^*$ is exactly the Hartman–Grobman question taken up in
  `lorenz_equilibrium` (needs hyperbolicity — the tooltip says when it fails).

## Controls / uniforms
**Other settings → LaTeX**: `equation`, `equilibrium labels`, `phi^t labels` sizes in px (the show toggles stay in their sections), saved.
Panel sections (red `sectionHeader()` titles): **Lorenz system** ($\sigma, \rho, \beta$,
`Classic (10, 28, 8/3)`, `show v' = f(v)`), **Equilibria** (one row per bead: just its
name, `[focused]` on the zoomed one — point and type live in the hover tooltip; hovering the row lights the bead; zooming is by mouse only; `Back to global view`;
`Ball here` drops the ball on the focused bead, remembering the global ball centre/radius,
which come back on `Back to global view` or `Reset camera`), **Flow** (`Clear all` is also key **R**; arrow keys ramp the speed logarithmically down to $10^{-4}$, a decade per 1.2 s held, and `speed` is a logarithmic slider from there), **Ball of
trajectories**, **Ball of trajectories** (`Drop ball` is also key **B**; **V** drops one with a single at its centre under the cursor, where a click would seed, sized to the view (a fixed fraction of the visible half-height at that depth); V's balls stack up to 3 (`kMaxBalls`), the oldest — with its centre single — going when a fourth is dropped, while B and the button still replace) also has `single at center` (a green single at the ball's
centre, removed with the ball); **Single trajectories** has `camera follows the newest` (also a tap of key **1** — switching off this way leaves the camera where it is, no ease back; holding 1 recenters; `show arrows` is key **2**, fading in/out)
— the orbit centre slides onto the newest single's head (attached at once, the offset
at that moment decaying with a cubic ease-out over `follow ease`, as in [[vector-field]];
another drop re-slides from where the camera is) and rides it (a focused bead
takes over while zoomed; unticking eases back to where the camera was). Also in
**Single trajectories**: (`label with phi^t(v0)` + size: every single
carries $\phi^t(v_0)$, composed per frame from pieces typeset once, as in
[[vector-field]] — always the symbol $v_0$, never the seed's numbers), collapsed
**Other settings** (camera; integrator — `ball` and `singles` methods, Taylor `order` +
`tolerance`, `max step h`; equilibria look — `zoom distance`, `zoom duration`,
`bead size`, `labels`; arrows; trails; performance), **Save settings** (parameters,
controls and the *global* camera — while zoomed, the remembered one), **Screenshot**
(scene only at `resolution scale` × window into `screenshots/`, also key **0** — same as
[[vector-field]]). Explorer thumbnail: `docs/screenshots/lorenz_flow.png`.
Bead uniforms: `view`, `projection`, `u_center`, `u_radius`, `u_eye`, `u_color`,
`u_glow`, `u_alpha` (handoff fade), `u_mode` (0 bead, 1 halo). Glyph uniforms as in [[vector-field]].

## Future
- Carry the trajectories through the handoff (today only numbers cross).
- Near-camera glyph clutter when zoomed in (the cube's arrows surround the camera);
  moot once the zoom ends in the handoff.
