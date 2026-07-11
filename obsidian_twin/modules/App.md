---
type: module
layer: core
status: stable
code:
  - engine/include/engine/App.h
  - engine/src/App.cpp
---
# App

`engine/include/engine/App.h`, `engine/src/App.cpp`

## What it is
The runtime skeleton: owns [[Window]], [[Camera]], a [[SimRegistry]], ImGui, and the
active [[Sim]]; runs the main loop (see [[render-pipeline]]). Core layer — sims don't
attach it, they receive it in `init(App&)`.

## Dependencies
- Engine: [[Window]], [[Camera]], [[SimRegistry]], [[Sim]]
- External: glad, glfw, imgui

## How to use it
Executables (not sims) construct it:
1. `App(title, width, height)` — creates window + GL context, enables depth test, inits ImGui.
2. `sims()` → registry; register factories before `run`.
3. `run(startSim = "")` → exit code; runs the first registered sim if empty.

Sims use the `App&` handed to `init`:
- `window()`, `camera()` — window info and camera constraints, from `init`/`update`.
- The active sim is driven as `activeSim->update(dt)` each frame.

## Notes
- The sim picker combo only appears when more than one sim is registered — the same App
  serves the explorer and standalone builds.
- Camera keyboard input and ESC are suppressed while ImGui wants the keyboard.
- `~App` destroys the active sim before shutting down ImGui; Window (context) dies last.
