# Conventions

## Ownership rules
- GL resources (buffers, programs) are owned **only** by engine wrappers: [[Mesh]] and [[Shader]].
  Sims never call `glGen*`/`glDelete*` directly. Global GL state calls (`glLineWidth`,
  `glPolygonMode`) are allowed in sim render code.
- One [[Window]] alive at a time (it owns glfwInit/Terminate and the GL context).
- Sims create their GL resources in `init()` (context is live there), not in constructors.
  [[App]] destroys the active sim before ImGui/context teardown.

## Code style
- Engine code in `namespace engine`, headers in `engine/include/engine/`, include as
  `#include <engine/Name.h>`.
- glm is on the include path at its root: `#include <glm.hpp>`, `<gtc/matrix_transform.hpp>`.
- Classes holding GL handles: non-copyable, movable, release in destructor.
- Sim classes live in their .cpp inside an anonymous namespace; only the factory function
  is exported through the header.

## How to add a sim
Use the `/new-sim` skill. Manually, the steps are:
1. `sims/<name>/` with `<Name>Sim.h` (factory declaration only), `<Name>Sim.cpp`
   (class + factory), `shaders/`, and a `CMakeLists.txt` calling
   `add_sim(<name> <Name>Sim.h make<Name>Sim <Name>Sim.cpp)`.
2. `add_subdirectory("<name>")` in `sims/CMakeLists.txt`.
3. Register in `apps/explorer/main.cpp`: `app.sims().add("<name>", &make<Name>Sim);`
   and link `sim_<name>` in `apps/explorer/CMakeLists.txt`.
4. Create `obsidian_twin/sims/<name>.md` from [[_template]] and link it in [[Home]].

The standalone executable `<name>_standalone` comes for free from `add_sim()`
(see [[sim-lifecycle]]).

## Asset paths
Shaders are loaded with the `SIMS_DIR` compile definition (absolute path to `sims/`),
e.g. `SIMS_DIR "riemann_sphere/shaders/vertex.glsl"`. No copying to the build tree.

## Math tests
`math`-layer modules get checks in `tests/math_tests.cpp` (target `math_tests`, also
`ctest`). The test target compiles the module sources directly without the engine lib,
so a stray glm/glad include in a math module fails the build — the std-only rule is
enforced there. Run `out/build/x64-debug/tests/math_tests.exe`; exit 1 on any failure.
