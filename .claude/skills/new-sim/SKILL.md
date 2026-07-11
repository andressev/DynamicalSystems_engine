---
name: new-sim
description: Scaffold a new simulation (Sim subclass, factory, CMake target, shaders folder, obsidian_twin note) in the Math Explorer engine. Use when the user asks to create/add a new sim or simulation.
---

# Scaffold a new sim

Given a sim name in `snake_case` (e.g. `vector_field` → class `VectorFieldSim`,
factory `makeVectorFieldSim`):

1. **Create `sims/<name>/`** with:
   - `<Name>Sim.h` — header-guard, includes `<engine/Sim.h>` + `<memory>`, declares ONLY
     `std::unique_ptr<engine::Sim> make<Name>Sim();` (the class stays out of the header).
   - `<Name>Sim.cpp` — class in an anonymous namespace implementing `engine::Sim`
     (`name()`, `init(App&)`, `update(dt)`, `render(const Camera&)`, optional `drawUI()`),
     factory definition at the bottom. Model it on
     `sims/riemann_sphere/RiemannSphereSim.cpp`. GL resources (`engine::Mesh`,
     `engine::Shader` via `std::unique_ptr`) are created in `init`, never in the
     constructor. No raw `glGen*` calls.
   - `shaders/` — start by copying `vertex.glsl` from `sims/riemann_sphere/shaders/` if a
     standard MVP vertex stage fits. Load paths as `SIMS_DIR "<name>/shaders/<file>"`.
   - `CMakeLists.txt` — exactly:
     `add_sim(<name> <Name>Sim.h make<Name>Sim "<Name>Sim.cpp")`
2. **Register the sim:**
   - `sims/CMakeLists.txt`: add `add_subdirectory("<name>")`.
   - `apps/explorer/CMakeLists.txt`: add `sim_<name>` to `target_link_libraries`.
   - `apps/explorer/main.cpp`: include the header, add
     `app.sims().add("<name>", &make<Name>Sim);`.
3. **Document:** copy `obsidian_twin/sims/_template.md` to `obsidian_twin/sims/<name>.md`
   (note filename uses hyphens: `vector_field` → `vector-field.md`), fill in every
   section — especially **"Modules attached"**: one `[[Module]]` line per engine module
   the sim uses, with what it's used for. Add the sim under "## Sims" in
   `obsidian_twin/Home.md`.
4. **Verify:** configure + build (`cmake --preset x64-debug`,
   `cmake --build out/build/x64-debug` — needs a VS dev shell), then run both
   `explorer.exe` (picker should list the new sim) and `<name>_standalone.exe`, and run
   `python tools/check_twin.py`.

The standalone executable comes automatically from `add_sim()`; do not write a `main.cpp`.
