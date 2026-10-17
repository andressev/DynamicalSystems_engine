# Sim lifecycle

## Registration (explicit, no static-init magic)
Each sim's header exposes only a factory: `std::unique_ptr<engine::Sim> make<Name>Sim();`.
Executables register what they want on a [[SimRegistry]]:

- `apps/explorer/main.cpp` — one `app.sims().add("name", &factory)` line per sim.
- `apps/standalone/main.cpp` — generic; `add_sim()` compiles it once per sim with
  `STANDALONE_SIM_HEADER` / `STANDALONE_SIM_FACTORY` / `STANDALONE_SIM_NAME` definitions.

Static registration was rejected because each sim is its own static library and the
linker strips unreferenced static initializers (see
[[0001-hybrid-scene-interface-plus-standalone-executables]]).

## Build shape
`add_sim(<name> <header> <factory> <sources...>)` (defined in `sims/CMakeLists.txt`) creates:
- `sim_<name>` static lib, linking [[App|engine]] PUBLIC, its own folder as PUBLIC include dir,
  `SIMS_DIR` compile definition for shader paths.
- `<name>_standalone` executable (gated by `BUILD_STANDALONE_SIMS`, default ON).

## Runtime switching
Multi-sim builds start with **no active sim**: [[App]] shows the thumbnail picker menu
until a tile is clicked, and the Explorer panel's "menu" button destroys the active sim
(end of frame) and returns to it. Then [[App]]::switchTo(index):
1. `activeSim.reset()` — old sim's destructor frees its Meshes/Shaders while the GL
   context is still current.
2. `registry.createByIndex(i)` — factory builds the new sim (no GL work in constructors).
3. `sim.init(app)` — sim creates meshes/shaders, may keep the `App&` for window queries.

**Sim-to-sim handoff** ([[0009-sim-handoff-requestswitch]]): a sim may call
`App::requestSwitch(name, handoff)`; at the end of the frame App stores the handoff map
and runs the same `switchTo`. The new sim reads `app.handoff()` in `init` (empty when it
came from the menu or launch) and, if handed over, keeps the shared camera instead of
framing its own view — that is what makes [[lorenz-flow]] → [[lorenz-equilibrium]] one
continuous zoom.

On shutdown, ~App resets the active sim **before** ImGui/context teardown, and [[Window]]
is destroyed last (member declaration order).
