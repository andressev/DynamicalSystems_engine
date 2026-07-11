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
- `add(name, factory)` — registration order = picker order.
- `create(name)` / `createByIndex(i)` / `indexOf(name)` / `names()` / `empty()`

## Notes
- Registration is explicit from each executable's `main()` — see [[sim-lifecycle]] for
  why static-init registration was rejected.
