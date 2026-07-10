

`engine/include/engine/App.h`, `engine/src/App.cpp`

Owns [[Window]], [[Camera]], a [[SimRegistry]], ImGui, and the active [[Sim]];
runs the main loop (see [[render-pipeline]]).

## Interface
- `App(title, width, height)` — creates window + GL context, enables depth test, inits ImGui.
- `sims()` → registry (register factories before `run`).
- `window()`, `camera()` — sims may use these from `init`/`update`.
- `run(startSim = "")` → exit code; first registered sim if empty.

## Invariants
- The sim picker combo only appears when more than one sim is registered — the same App
  serves the explorer and standalone builds.
- Camera keyboard input and ESC are suppressed while ImGui wants the keyboard.
- `~App` destroys the active sim before shutting down ImGui; Window (context) dies last.

## Sim
The active sim is updated through  activeSim->update(dt "time" )