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
- `recenterCamera()` — optional: the sim's home view (what its Reset camera/view button
  does). [[App]] calls it when key **1** is held past `App::kRecenterHoldSeconds` (1 s);
  a tap of 1 stays the sim's own (follow toggle in the flow sims, via
  [[FlowKeys]] `tapHold`). Every sim implements it so the key works everywhere, and
  glides there with [[Camera]] `easeOrbitTo` over `App::kRecenterEaseSeconds`
  ([[vector-field]] R2 has its own view ease) rather than snapping like the button.

Scaffold with the `/new-sim` skill rather than by hand.

## Notes
- Convention: the concrete class lives in the sim's .cpp (anonymous namespace); the
  header exposes only `std::unique_ptr<engine::Sim> make<Name>Sim();`.
