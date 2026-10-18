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
- **1 held** for `kRecenterHoldSeconds` (1 s) calls the active [[Sim]]'s
  `recenterCamera()` once, in every sim ([[FlowKeys]] `tapHold` tells a hold from a
  tap, so a sim reading the tap for its follow toggle never sees the hold as a tap).
  The recenter glides over `kRecenterEaseSeconds` (1 s): sims call [[Camera]]
  `easeOrbitTo`, which `run` advances each frame (`cam.updateEase(dt)`) before input.
- **F11** toggles borderless full screen ([[Window]] `toggleFullscreen`; the window's
  maximize button enters it as well).
- **H** toggles the Explorer panel (App's FPS/menu row + the sim's `drawUI`) in every
  sim — `showControls`, checked in `run` before `drawAppUI`; a sim reads / sets it with
  `controlsVisible()` / `setControlsVisible(bool)` (e.g. [[lorenz-equilibrium]] hides it
  when Show conjugacy starts the timeline). A hidden panel also pulls in by itself
  when the mouse reaches the window's top-left corner (`kPanelHotCorner`, 48 px,
  opened uncollapsed) and goes again once the mouse has been off it for
  `kPanelLeaveSeconds` (0.6 s) or still for `kPanelIdleSeconds` (3 s) —
  `updateAutoPanel`, before the sim's update; a panel shown with H or by the sim stays
  (`controlsAuto`). Only the panel: overlays
  a sim draws itself ([[FlowKeys]] status, LaTeX HUD) stay. A hidden panel skips
  `drawUI` entirely, so sims must not depend on it running every frame.
- Screenshot PNGs are written by a tiny encoder in `App.cpp` (stored deflate blocks, no
  zlib/stb_image_write), so files are raw-sized (~11 MB at 2560×1440).
- `~App` destroys the active sim before shutting down ImGui; Window (context) dies last.
