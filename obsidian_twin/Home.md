# Home

Map of content for the Math Explorer engine — an OpenGL engine for exploring ODEs,
flows on manifolds and vector fields, with the complex grapher as its first sim.

The project has two kinds of parts ([[0004-modules-vs-simulations]]):
**modules** — self-contained engine resources sims attach by declaring a member — and
**sims** — the simulations themselves. Every module has a note here (what it is,
dependencies, how to use it); every sim's note says what is simulated, how it works,
and which modules it attaches. `tools/check_twin.py` verifies code ↔ vault stay in sync.

## Routing — if you're doing X, read Y first

| Task | Read first |
|---|---|
| Anything (session start) | the `## Now` section of [[roadmap]] |
| New sim | [[conventions]], [[Sim]], then "How to use it" of each module you'll attach |
| New module | [[0004-modules-vs-simulations]], `modules/_template` |
| Touching the render path | [[render-pipeline]], [[Shader]], [[Mesh]] |
| Touching parsing / GLSL codegen | [[Expression]], [[0003-hand-rolled-parser-glsl-codegen]] |
| Sim switching / lifecycle | [[sim-lifecycle]], [[App]], [[SimRegistry]] |
| Anything mathy | the relevant note under `math/` (below) |

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
- [[Mesh]] — RAII VAO/VBO/EBO; the future CUDA interop point
- [[Shapes]] — procedural geometry generators
- [[SolutionCurve]] — CPU particle integrating x' = V(x,t) (RK4), sphere body + trail

### math — pure computation, std-only
- [[Expression]] — function parser: UI string → AST → CPU eval + GLSL codegen

### compute — CUDA interop (future, M4 in [[roadmap]])
- (none yet)

## Sims
- [[riemann-sphere]] — domain coloring of complex polynomials on the Riemann sphere
- [[surface-renderer-ray-marched]] — raymarched implicit surfaces f(x,y,z) = c, live c sweep

## Math
Domain concepts as first-class notes (idea / equations / where implemented / pitfalls);
sims link to them instead of re-deriving. New concepts from `math/_template.md`.
- [[stereographic-projection]] — sphere ↔ complex plane, the Riemann-sphere map
- [[domain-coloring]] — complex function → hue/brightness
- [[raymarching-level-sets]] — pixel → ray, sign-change march + bisection on f − c

## Decisions
- [[0001-hybrid-scene-interface-plus-standalone-executables]]
- [[0002-imgui-ui-layer]]
- [[0003-hand-rolled-parser-glsl-codegen]]
- [[0004-modules-vs-simulations]]
- [[0005-vault-as-ai-navigation]]
