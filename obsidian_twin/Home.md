# Home

Map of content for the Math Explorer engine — an OpenGL engine for exploring ODEs,
flows on manifolds and vector fields, with the complex grapher as its first sim.

The project has two kinds of parts ([[0004-modules-vs-simulations]]):
**modules** — self-contained engine resources sims attach by declaring a member — and
**sims** — the simulations themselves. Every module has a note here (what it is,
dependencies, how to use it); every sim's note says what is simulated, how it works,
and which modules it attaches. `tools/check_twin.py` verifies code ↔ vault stay in sync.

## Routing — if you're doing X, read Y first

| Task                            | Read first                                                                  |
| ------------------------------- | --------------------------------------------------------------------------- |
| Anything (session start)        | the `## Now` section of [[roadmap]]                                         |
| New sim                         | [[conventions]], [[Sim]], then "How to use it" of each module you'll attach |
| New module                      | [[0004-modules-vs-simulations]], `modules/_template`                        |
| Touching the render path        | [[render-pipeline]], [[Shader]], [[Mesh]]                                   |
| Touching parsing / GLSL codegen | [[Expression]], [[0003-hand-rolled-parser-glsl-codegen]]                    |
| Sim switching / lifecycle       | [[sim-lifecycle]], [[App]], [[SimRegistry]]                                 |
| ODEs / flows / trajectories     | [[integral-curves]], [[VectorField]], [[Integrator]], [[Trajectory]]        |
| Anything mathy                  | the relevant note under `math/` (below)                                     |

Notes carry the *why*, *where*, and *now* that code can't
([[0005-vault-as-ai-navigation]]); read the code itself for the *what*. Every
module/sim/math note lists its source files under `code:` in frontmatter — verified by
the drift checker, so a green check means the pointers are current.

## Functionality
On the explore mode, from main an [[App]] is started where simulations can be alternated, on the app a loop is run wich calls the current sim update. The explorer opens on a thumbnail picker menu (tiles from `docs/screenshots/<sim>.png`); the Explorer panel's "menu" button returns to it.

Each sim holds their own open gl data, and when swithcing is released. 

## Architecture
- [[render-pipeline]] — frame flow: App loop → Sim → Mesh/Shader → GL
- [[sim-lifecycle]] — registry, switching, init/destroy order
- [[roadmap]] — milestones (function parser, ODE/vector fields, CUDA interop)
- [[conventions]] — code style, ownership rules, how to add a sim/module

## Modules
Layer rule: a module may only depend on same-or-lower layers (math is std-only;
core is the skeleton sims don't choose). New modules via the `/new-module` skill.

### core — the runtime skeleton
- [[App]] — owns the main loop, window, camera, ImGui, active sim
- [[Window]] — GLFW window + GL context owner
- [[Sim]] — the interface every simulation implements
- [[SimRegistry]] — name → factory map, explicit registration

### gfx — GL-facing resources
- [[Camera]] — orbit/free camera, view + projection matrices
- [[Shader]] — GLSL program wrapper (file or in-memory sources)
- [[Mesh]] — RAII VAO/VBO/EBO, static or dynamic; the future CUDA interop point
- [[Shapes]] — procedural geometry generators
- [[ArrowGlyphs]] — instanced arrow glyph batches (quiver, tangents, cones)
- [[FlowKeys]] — shared arrow-key playback: Up/Down speed (to 0), Left/Right direction
- [[ConjugacyMap]] — $\tilde h^{-1} = K_s + K_u$ per vertex in GLSL (deformed ball, $W^s$ patch)
- [[Trajectory]] — one integral curve: ODE state + growing polyline on the GPU
- [[TrajectoryRunner]] — advances a sim's trajectories (parallel integrate) and times integrate/upload/draw
- [[FrameCopy]] — a texture copy of the frame so far, for effects that bend what is behind them
- [[GpuTimer]] — GPU time of a stretch of draw calls (timer queries, never stalls)
- [[LatexImage]] — LaTeX math typeset (MicroTeX) into a texture, drawn via ImGui

### math — pure computation, std-only
- [[Expression]] — function parser: UI string → AST → CPU eval (complex + real) + GLSL codegen
- [[VectorField]] — typed strings → a real field on R2/R3; the ODE right-hand side
- [[Integrator]] — fixed-step Euler/RK2/RK4 over any right-hand side
- [[TaylorIntegrator]] — high-order Taylor series method, adaptive steps, dense output (singles)
- [[Eigen3]] — 3×3 eigenvalues/vectors (complex pairs), hyperbolicity, $E^s$/$E^u$ bases
- [[InvariantManifold]] — parametrization method: Taylor map $K$ of $W^s$/$W^u$ at any equilibrium
- [[LevelSetSurface]] — a surface grown by geodesic level sets, band by band, saved / loaded (`.lss`)
- [[LorenzSystem]] — the Lorenz field, its equilibria, Jacobian and quadratic part

### compute — CUDA interop (future, M4 in [[roadmap]])
- (none yet)

## Sims
- [[riemann-sphere]] — domain coloring of complex polynomials on the Riemann sphere
- [[surface-renderer-ray-marched]] — raymarched implicit surfaces f(x,y,z) = c, live c sweep
- [[lorenz-manifold]] — geodesic rings of the Lorenz stable manifold, built from a Taylor-series patch
- [[vector-field]] — any typed field on R2/R3: glyphs, typeset $\dot{x} = f(x)$, balls of trajectories and $\phi^t(x_0)$-labelled singles; per-mode settings
- [[diagonal-linear]] — $\dot x = \mathrm{diag}(\lambda)\,x$ in R3: stable/unstable subspaces + balls of trajectories
- [[lorenz-flow]] — the global Lorenz flow: field, trajectories, equilibria to hover (linearization) and click (eased zoom → handoff)
- [[lorenz-equilibrium]] — the flow inside a glass ball $V$ around one Lorenz equilibrium; where Hartman–Grobman stages A/B/C land
- [[lorenz-manifold-growth]] — grow $W^s$/$W^u$ of each Lorenz equilibrium from a hover popup: the Taylor patch $W_{loc}$ first, then the flow carries a curve on, a stored surface grows band by band ($W^s(O)$, classic parameters); watch a branch
- [[learn-geodesic-level-sets]] — the walkthrough of geodesic level sets, entered from the growth popup: the mesh, the BVP, rings grown and accepted (bands light / dark blue), Save surface

## Math
Domain concepts as first-class notes (idea / equations / where implemented / pitfalls);
sims link to them instead of re-deriving. New concepts from `math/_template.md`.
- [[stereographic-projection]] — sphere ↔ complex plane, the Riemann-sphere map
- [[domain-coloring]] — complex function → hue/brightness
- [[lorenz-stable-manifold]] — parametrization method: Taylor patch for a stable manifold
- [[integral-curves]] — vector fields, flows, equilibria, and glyph conventions
- [[runge-kutta]] — Euler/RK2/RK4: order, stability, and how they fail
- [[global-manifold-growth]] — from $W_{loc}$ to $W$: flow the ends of a curve; a surface needs geodesic level sets

## Decisions
- [[0001-hybrid-scene-interface-plus-standalone-executables]]
- [[0002-imgui-ui-layer]]
- [[0003-hand-rolled-parser-glsl-codegen]]
- [[0004-modules-vs-simulations]]
- [[0005-vault-as-ai-navigation]]
- [[0006-cpu-field-sampling-dynamic-glyph-mesh]]
- [[0007-latex-labels-microtex-imgui]]
- [[0008-gpu-work-split-vertex-stage-and-cuda]]
- [[0009-sim-handoff-requestswitch]]
- [[0010-manifold-popup-and-copied-baseline]]
- [[0011-precomputed-level-set-surfaces]]
