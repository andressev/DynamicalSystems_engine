# Sim

`engine/include/engine/Sim.h`

The interface every simulation implements. One sim = one folder under `sims/`,
built as its own static library (see [[sim-lifecycle]]).

## Contract
- `name()` — stable id shown in the picker.
- `init(App&)` — GL context is live; create [[Mesh]]es/[[Shader]]s here, keep the `App&`
  if window info is needed later. Never do GL work in constructors.
- `update(float dt)` — advance math/state.
- `render(const Camera&)` — issue draw calls using the shared [[Camera]].
- `drawUI()` — optional ImGui widgets, drawn inside the explorer panel.

Convention: the concrete class lives in the sim's .cpp (anonymous namespace); the header
exposes only `std::unique_ptr<engine::Sim> make<Name>Sim();`.
