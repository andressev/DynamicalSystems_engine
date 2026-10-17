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
2. `sims()` → registry; register factories (and optional thumbnail paths) before `run`.
3. `run(startSim = "")` → exit code. One registered sim (or explicit `startSim`) starts
   immediately; several sims with no `startSim` open on the thumbnail picker menu
   (no active sim until the user clicks a tile).

Sims use the `App&` handed to `init`:
- `window()`, `camera()` — window info and camera constraints, from `init`/`update`.
- The active sim is driven as `activeSim->update(dt)` each frame.
- `captureScreenshot(path, scale)` — from `update`, once state is current: re-renders the
  active sim alone (no ImGui) into an offscreen 8x-MSAA FBO at `scale` × the framebuffer,
  resolves and writes an RGB PNG. During it `renderScale()` returns `scale` (else 1);
  multiply pixel-sized things (`glLineWidth`, point sizes) by it.
- `requestSwitch(name, handoff)` — switch to another registered sim at the end of the
  frame, handing it a `Handoff` (`std::map<std::string, double>`); the new sim reads
  `handoff()` in `init` (empty from the menu). The [[Camera]] is left as it is. False
  when `name` isn't registered; `hasSim(name)` asks first
  ([[0009-sim-handoff-requestswitch]]; used by [[lorenz-flow]] ↔ [[lorenz-equilibrium]]).

## Notes
- The sim picker is a centered menu of thumbnail tiles (`drawMenuUI`): images come from
  [[SimRegistry]]'s per-sim `thumbnailPath`, loaded via stb_image into GL textures owned
  by App (`loadThumbnails`); a missing image falls back to a plain named tile. The
  Explorer panel's "menu" button returns to it (deferred to end of frame via `wantMenu`
  so the sim's UI isn't destroyed mid-frame). Menu only exists when more than one sim is
  registered — the same App serves the explorer and standalone builds.
- Camera keyboard input and ESC are suppressed while ImGui wants the keyboard.
- Screenshot PNGs are written by a tiny encoder in `App.cpp` (stored deflate blocks, no
  zlib/stb_image_write), so files are raw-sized (~11 MB at 2560×1440).
- `~App` destroys the active sim before shutting down ImGui; Window (context) dies last.
