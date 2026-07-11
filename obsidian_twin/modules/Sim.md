---
type: module
layer: core
status: stable
code:
  - engine/include/engine/Sim.h
---
# Sim

`engine/include/engine/Sim.h`

## What it is
The interface every simulation implements — the contract that keeps sims detachable.
One sim = one folder under `sims/`, built as its own static library (see
[[sim-lifecycle]]). Deliberately minimal: new capabilities become modules a sim pulls
in, never new virtuals every sim must implement.

## Dependencies
- Engine: none (forward-declares [[App]] and [[Camera]])
- External: std-only

## How to use it
Implement the contract in your sim's .cpp:
- `name()` — stable id shown in the picker.
- `init(App&)` — GL context is live; create [[Mesh]]es/[[Shader]]s here, keep the `App&`
  if window info is needed later. Never do GL work in constructors.
- `update(float dt)` — advance math/state.
- `render(const Camera&)` — issue draw calls using the shared [[Camera]].
- `drawUI()` — optional ImGui widgets, drawn inside the explorer panel.

Scaffold with the `/new-sim` skill rather than by hand.

## Notes
- Convention: the concrete class lives in the sim's .cpp (anonymous namespace); the
  header exposes only `std::unique_ptr<engine::Sim> make<Name>Sim();`.
