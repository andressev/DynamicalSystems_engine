# Home

Map of content for the Math Explorer engine — an OpenGL engine for exploring ODEs,
flows on manifolds and vector fields, with the complex grapher as its first sim.


## Functionality
On the explore mode, from main an [[App]] is started where simulations can be alternated, on the app a loop is run wich calls the current sim update.  

Each sim holds their own open gl data, and when swithcing is released. 


## Architecture
- [[render-pipeline]] — frame flow: App loop → Sim → Mesh/Shader → GL
- [[sim-lifecycle]] — registry, switching, init/destroy order
- [[roadmap]] — milestones (function parser, ODE/vector fields, CUDA interop)
- [[conventions]] — code style, ownership rules, how to add a sim

## Engine modules
- [[App]] — owns the main loop, window, camera, ImGui, active sim
- [[Window]] — GLFW window + GL context owner
- [[Sim]] — the interface every simulation implements
- [[SimRegistry]] — name → factory map, explicit registration
- [[Camera]] — orbit/free camera, view + projection matrices
- [[Shader]] — GLSL program wrapper (file or in-memory sources)
- [[Expression]] — function parser: UI string → AST → CPU eval + GLSL codegen
- [[Mesh]] — RAII VAO/VBO/EBO; the future CUDA interop point
- [[Shapes]] — procedural geometry generators

## Sims
- [[riemann-sphere]] — domain coloring of complex polynomials on the Riemann sphere

## Decisions
- [[0001-hybrid-scene-interface-plus-standalone-executables]]
- [[0002-imgui-ui-layer]]
- [[0003-hand-rolled-parser-glsl-codegen]]
