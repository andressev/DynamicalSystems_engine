---
type: module
layer: core
status: stable
code:
  - engine/include/engine/SimRegistry.h
  - engine/src/SimRegistry.cpp
---
# SimRegistry

`engine/include/engine/SimRegistry.h`, `engine/src/SimRegistry.cpp`

## What it is
Ordered name → factory list mapping sim names to their factory functions
(`SimFactory = std::unique_ptr<Sim> (*)()`). Core layer — owned by [[App]].

## Dependencies
- Engine: [[Sim]]
- External: std-only

## How to use it
Executables register sims before `App::run`:
- `add(name, factory, thumbnailPath = "")` — registration order = picker order;
  `thumbnailPath` is the image for [[App]]'s menu tile (explorer passes
  `THUMBS_DIR "<name>.png"` → `docs/screenshots/`; empty/missing = plain tile).
- `create(name)` / `createByIndex(i)` / `indexOf(name)` / `names()` /
  `thumbnailPaths()` / `empty()`

## Notes
- Registration is explicit from each executable's `main()` — see [[sim-lifecycle]] for
  why static-init registration was rejected.
