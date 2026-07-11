---
name: new-module
description: Scaffold a new engine module (header + cpp + CMake entry + obsidian_twin note) in the Math Explorer engine. Use when the user asks to create/add a new module, engine resource, or reusable component.
---

# Scaffold a new engine module

A module is a compile-time component (see decision
`obsidian_twin/decisions/0004-modules-vs-simulations.md`): one self-contained class a
sim attaches by declaring a member and detaches by deleting it. One module = one header
+ one cpp + one vault note — all three or nothing.

Given a module name in `PascalCase` (e.g. `Integrator`):

1. **Pick the layer with the user** if not obvious, and enforce the dependency rule:
   - `core` — runtime skeleton (rare; App/Window/Sim/SimRegistry territory).
   - `gfx` — GL-facing resource; may include glad/glfw/glm and other gfx modules.
   - `math` — pure computation; **std-only**: no glad, no glfw, no glm, no engine
     includes outside `math`. (Keeps it unit-testable and CUDA-portable.)
   - `compute` — CUDA interop; may depend on gfx (e.g. [[Mesh]] buffers).
   A module may only depend on same-or-lower layers (math < gfx < compute). If the
   requested module needs an upward dependency, stop and flag it — the design is wrong.

2. **Create the class** (model shape on `Mesh.h`/`Mesh.cpp`):
   - `engine/include/engine/<Name>.h` — header guard `ENGINE_<NAME>_H`,
     `namespace engine`, minimal includes per the layer rule.
   - `engine/src/<Name>.cpp` — implementation. Header-only is allowed for pure
     generators (like `Shapes.h`); then skip the cpp and CMake step.
   - Rules: RAII ownership, no globals, no reaching into `App`. If it owns GL objects:
     non-copyable, movable, delete in destructor, default-constructed = empty/invalid
     (see [[Shader]]/[[Mesh]] for the pattern).

3. **Register in the build**: add `"src/<Name>.cpp"` to the `add_library(engine STATIC ...)`
   list in `engine/CMakeLists.txt`.

4. **Document — required, not optional**: copy `obsidian_twin/modules/_template.md` to
   `obsidian_twin/modules/<Name>.md`, set `layer:`/`status:` frontmatter, and fill in
   every section. "How to use it" must contain the attach recipe: the member
   declaration, what to call in `init`/`update`/`render`, and a reference sim once one
   uses it. Add the module under the matching layer heading in
   `obsidian_twin/Home.md` → "## Modules".

5. **Verify**: build (`cmake --build out/build/x64-debug` — needs a VS dev shell), then
   run `python tools/check_twin.py` and fix anything it reports.

A module with no sim using it yet is fine (`status: wip`); a module with no note is not.
