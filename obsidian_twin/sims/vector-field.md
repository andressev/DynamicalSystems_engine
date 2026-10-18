---
type: sim
status: stable
code:
  - sims/vector_field/VectorFieldSim.cpp
  - sims/vector_field/shaders/vertex.glsl
  - sims/vector_field/shaders/fragment.glsl
---
# vector-field

`sims/vector_field/` — lib `sim_vector_field`, standalone exe `vector_field_standalone`.

Type any vector field on $\mathbb{R}^2$ or $\mathbb{R}^3$: see it as arrow glyphs, read
$\dot{\mathbf{x}} = f(\mathbf{x})$ typeset from exactly what you typed, and drop a ball
of trajectories or single labelled ones into the flow.

## What is being simulated
The flow of $\dot{\mathbf{x}} = f(\mathbf{x}, t)$ for a field typed at runtime
([[integral-curves]]). Two pictures of the same object at once: the *instantaneous*
field (glyphs, direction by orientation, $|f|$ by color) and the *integrated* field —
a ball (in R2 a disk) of seeds deforming under the flow, and single trajectories, each
labelled $\phi^t(x_0)$ with $x_0$ written out. R2 is the phase plane (saddle, center,
spiral sink, van der Pol, damped pendulum, a time-dependent field); R3 the strange
attractors (Lorenz, Rössler, Thomas — the latter's field times 10, so it runs at the
others' pace) — a ball dropped into Lorenz traces the attractor.

Rebuilt (2026-10-11) in the style of [[diagonal-linear]]: same labels, overlay, seeding
and panel layout, minus that sim's linear-algebra extras (no $E^s$/$E^u$ — a general
field has no global eigenspaces).

## How it works
**Two modes, two settings.** All per-mode state — the typed components, every control,
the R2 window or the R3 camera — lives in one `Settings` struct. The sim keeps two
(`cfg` active, `stashed` the other mode) and swaps them on an R2/R3 switch, so each
mode keeps its own field and view; curves are cleared (a curve in R2 is not one in R3).
`Save R2/R3 settings` writes the active one to `settings_2d.txt` / `settings_3d.txt`
(text values are the rest of the line, so typed fields with spaces survive); `init`
loads both. In R3 the orbit view is copied from [[Camera]]'s `orbitTheta`/`orbitPhi`/
`orbitRadius`/`orbitCenter` on save and applied in `enterMode()`; `camRadius <= 0`
means "frame the domain" ($3\times$ `domain radius`).

**Views.** R3 orbits the shared [[Camera]]. R2 clears `inputEnabled` and draws through
its own orthographic window `ortho(c_x ± s·a, c_y ± s)` (WASD pan, Q/E zoom in view
units); `toScreen()` and the click handler use the same matrices. R2 renders with depth
testing off — one plane, so draw order (arrows, trail, head) decides.

**Frame.** `update`: view keys and click-to-seed; if playing, advance the field clock
and every [[Trajectory]] by `dt · speed` (negative when reversed) with [[Integrator]]
(method / max step from the settings); rebuild glyphs when the R2 window moved, a glyph
control changed, or every frame for a field that mentions $t$; then the two overlays.
`render`: axes, glyph shafts + heads (blended, no depth writes), then per curve its
trail (flat color, or the speed ramp) and round head — singles `singles thickness`
times fatter, a diverged head red.

**Glyphs** are the old vector-field construction ([[0006-cpu-field-sampling-dynamic-glyph-mesh]]):
sampled on the CPU — in parallel, `std::execution::par` over the lattice — and drawn
instanced by [[ArrowGlyphs]] (one record per arrow; "last rebuild … ms" in the panel); solid heads are cones in R3
(LOD by arrow count) and flat triangles in R2, or line barbs. The R2 lattice is anchored
in the plane (origin, power-of-two spacing — map tiles), so zooming magnifies it like the
curves: the level is the first spacing $2^k$ at or above the target $s = 2\,\text{viewScale}/N$,
and the next finer level — the points between — is a second [[ArrowGlyphs]] batch
(`glyphsFine`) whose alpha ramps with $t = 2^k / s \in [1, 2)$ via a smoothstep, so arrows
fade in between their neighbours and the switch at $t = 2$ is seamless. Arrow length
follows $s$ (screen-constant). R3 is a cube of `domain radius`.

**Overlays** ([[LatexImage]], decision [[0007-latex-labels-microtex-imgui]]):
- top-right $\begin{pmatrix}\dot x\\ \dot y\end{pmatrix} = \begin{pmatrix}f_1\\ f_2\end{pmatrix}$
  typeset from `VectorField::latex()` — [[Expression]]'s LaTeX printer over the parsed
  AST, so it is exactly the integrated field, re-typeset only on Apply;
- $\phi^{t}(x_0)$ above each single, $t$ = time since it was dropped (`time() - t0`),
  composed every frame from pieces typeset once (the `TimeLabelGlyphs` pattern from
  [[diagonal-linear]]) with $x_0$ typeset once per curve — or, with `show x0 values`
  off, the symbol $(x_0)$ (one shared piece) instead of the coordinates;
- a red **tangent vector** out of each single's head — $\dot{x} = f(x)$ evaluated there,
  rebuilt every frame (`rebuildTangents()`) into two small dynamic [[Mesh]] buffers with
  the glyph `pushArrow`, at constant on-screen size (`length` × camera distance in R3,
  × view half-height in R2, capped in world units — 0.3 `domain radius` in R3, 2 in R2 —
  so zooming out never grows it past the scene) — labelled $\dot{x}$, placed just beyond the tip along the
  arrow's screen direction. When the arrow points up on screen the $\phi^t$ label
  moves below the particle, so the two never cross.

**Follow camera.** `camera follows a single` (R3) / `view follows a single` (R2) tracks
a single (the newest, or `follow #`). The ball's `single at center` toggle drops one at
the ball's centre with every drop (removed with the ball) — the newest single, so the
camera rides the middle of the ball. In R2 the pan/zoom window is centred on the head
every frame (WASD panning is off while following, Q/E zoom stays yours; the glyph grid
resamples as the window moves). The centre does not snap: when the followed single
changes (new drop, follow switched on, `follow #`) the camera attaches to the new head
at once plus the offset it had at that moment, and that offset decays to zero over
`follow ease` seconds with a cubic ease-out — full speed away, gentle arrival, never
lagging the particle's own motion; a drop mid-slide restarts from the current centre.
Identity is the single's [[Trajectory]] pointer (`followedTraj`). 0 s snaps. (A
smootherstep from the old centre was tried first and felt like a stall at the start.) In R3 `followCamera()` runs in `update` after the
integration step and moves [[Camera]]'s `orbitCenter` *and* `position` together,
keeping the orbit offset — so the view neither lags a frame nor stops following while
ImGui owns the keyboard (when `processInput` is skipped); A/D/W/S/Q/E still orbit and
zoom around the particle. Unticking returns the centre to the saved one, same angles
and distance.

## Modules attached
- [[TrajectoryRunner]] — advances every curve (integrate on all cores, then upload) and
  times integrate / upload / draw; readout under Other settings → Performance.
- [[FlowKeys]] — arrow-key speed and direction.
- [[VectorField]] — parses the typed components; `eval` is the ODE right-hand side,
  `usesTime()` drives per-frame glyph rebuilds, `latex()` the overlay.
- [[Expression]] — under [[VectorField]]: `evalReal` for the flow, `latex` for display.
- [[Integrator]] — Euler/RK2/RK4 with `maxStep` substepping, reverse time.
- [[Trajectory]] — one per curve: state, trail, `drawHead()`, `time()` for $\phi^t$.
- [[LatexImage]] — the $\dot{x} = f(x)$ overlay and the $\phi^t(x_0)$ label pieces.
- [[Shader]] — one program (same pair as [[diagonal-linear]]).
- [[ArrowGlyphs]] — the glyph grid (R2: level + fading finer level), the red tangent arrows.
- [[Mesh]] — static axes.
- [[Shapes]] — axis lines.
- [[Camera]] — R3 orbit (rates and saved view set by the sim); R2 `inputEnabled = false`.

## The math
- Fields, integral curves, the flow map $\phi^t$: [[integral-curves]].
- Integration schemes and their failure modes: [[runge-kutta]].
- R3 click-to-seed: the cursor ray, unprojected through the inverse view-projection,
  stopped on a camera-facing plane `click depth` of the way to `orbitCenter` — so the
  seed lands just in front of the camera, under the cursor.

## Controls / uniforms
**Other settings → LaTeX**: `equation` (the $\dot x = f(x)$ overlay) and `labels` ($\phi^t(x_0)$ and $\dot x$) sizes in px, saved per mode.
Red `sectionHeader()` titles: **Vector field** (R2/R3, `dx/dt`… boxes + Apply/Enter,
presets, red parse error — the previous field keeps drawing, syntax help, field time) ·
**LaTeX overlays** (`show x' = f(x)`, `label singles with phi^t(x0)`) · **Flow**
(Play/Pause — also **Space**, ignored while typing in a field; arrow keys via
[[FlowKeys]]: Up/Down speed on a logarithmic ramp down to $10^{-4}$ — a decade per 1.2 s held —, Left reverse, Right forward; `speed` is a logarithmic slider from $10^{-4}$ — `reverse`, `Restart` — every curve back to its seed at the current field
time, `Clear all` (also key **R**), `speed`, count + how many blew up) · **Disk / Ball of trajectories**
(`center`, `radius`, `particles`, `fill disk/volume`, Drop — also key **B**; **V** drops one with a single at its centre under the cursor, where a click would seed, sized to the view (a fixed fraction of the visible half-height at that depth); V's balls stack up to 3 (`kMaxBalls`), the oldest — with its centre single — going when a fourth is dropped, while B and the button still replace; replaces the previous —
/ Remove) · **Single trajectories** (click the view; `click depth` in R3,
`singles thickness`, `label size`, `show x0 values`, `tangent vector x'` + `length`, `Clear singles`;
`camera`/`view follows a single` (also a tap of key **1** — switching off this way leaves the camera where it is; holding 1 glides to the home view — R2 through `updateViewEase`, the window's own smootherstep, cancelled by any pan/zoom key) + `newest` / `follow #`) · collapsed **Other settings** (camera and integrator first):
Arrow field (show arrows — also key **2**, fades in/out — /axes, `grid N`, `arrow scale`, head solid/barbs + length/width,
`length by magnitude`, `hide below`, `domain radius` (R3), alpha + `alpha gradient`,
color scale), Trails & particles (`show trails`, `color by speed`, `trail points`,
width, alpha, `particle size`), Integrator (`method`, `max step h`), View/Camera (R2
centre + half-height + Reset; R3 `orbit speed`, `zoom speed` (0 = auto) + Reset) ·
**Settings (R2/R3)**: the save button for the active mode · **Screenshot**: key **0**
(or the button) saves the scene alone — no panel, no LaTeX labels — at
`resolution scale` × the window via `App::captureScreenshot` to `<repo>/screenshots/`;
`render` multiplies line widths/point sizes by `app->renderScale()` so it keeps the look.

Uniforms (set in `render`): `view`, `projection`, `u_magScale` ($1/|f|_{\max}$),
`u_pointSize`, `u_useRamp`, `u_color`, `u_alpha`, `u_alphaTail`, `u_useFade`,
`u_shadeMode`, `u_round`.

## Future
- Dropped in the rebuild, could come back: ride-the-particle (chase cam behind the
  velocity), distance fade of glyphs near particles, trail fading by point age,
  typed/random seeds.
- CUDA advection of many seeds ([[roadmap]] M4) — a ball of 1000 is the natural test.
- Nullclines / equilibria (needs symbolic $\nabla$ in [[Expression]]).
