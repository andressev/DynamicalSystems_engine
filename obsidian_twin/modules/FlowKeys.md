---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/FlowKeys.h
  - engine/src/FlowKeys.cpp
---
# FlowKeys

`engine/include/engine/FlowKeys.h`, `engine/src/FlowKeys.cpp`

## What it is
The keyboard playback controls every flow sim shares, in one place so they behave
identically everywhere: holding **Up** speeds the flow up, holding **Down** slows it
down to $0$ (frozen), **Left** runs time backwards, **Right** forwards again. The speed
ramps while the key is held — `kRate` $= 0.5$ per second at or below $1\times$ (so
$0 \to 1$ takes about 2 s), proportionally faster above so $10\times$ stays reachable.

## Dependencies
- Engine: none
- External: imgui (key state, `WantCaptureKeyboard`) — gfx layer, like [[LatexImage]].

## How to use it
1. No member — a static call. In `update`, next to the Space play/pause toggle:
   `engine::FlowKeys::handle(cfg.timeScale, cfg.reverse, dt, maxSpeed);`
2. The sim's `speed` slider should start at $0$ so the keys and the slider agree.
   Or pass a `minScale > 0` (5th argument): the ramp becomes logarithmic — every factor
   of 10 takes `kDecadeSeconds` (1.2 s) of holding, either way, clamped to
   [`minScale`, `maxScale`] — for a sim that needs very slow flows
   ([[lorenz-equilibrium]]: down to $10^{-4}$, with a logarithmic slider to match).
3. `FlowKeys::kHint` — `"(space, arrows)"`, shown next to the Play button.
4. `FlowKeys::drawStatus(timeScale, reverse, playing)` right after `handle` — the
   bottom-right readout: speed (`1.00x`, more digits below 0.1 so a slow flow never
   reads 0.00x, a size up) over `normal` / `reversed` (red),
   dimmed while paused. An ImGui overlay, so screenshots leave it out.
5. Reference sims: [[vector-field]], [[diagonal-linear]], [[lorenz-flow]],
   [[lorenz-equilibrium]].

## Notes
- Ignored while ImGui has the keyboard (typing in a field, keyboard nav in a panel).
- The arrow keys are free in every sim: the [[Camera]] uses WASD/QE, R2 panning in
  [[vector-field]] uses WASD/QE too.
- Sims without a flow (riemann sphere, ray-marched surfaces, Lorenz manifold) don't call it.
