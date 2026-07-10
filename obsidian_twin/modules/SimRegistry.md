# SimRegistry

`engine/include/engine/SimRegistry.h`, `engine/src/SimRegistry.cpp`

Ordered name → factory list. `SimFactory = std::unique_ptr<Sim> (*)()`.

## Interface
- `add(name, factory)` — registration order = picker order.
- `create(name)` / `createByIndex(i)` / `indexOf(name)` / `names()` / `empty()`

Registration is explicit from each executable's `main()` — see [[sim-lifecycle]] for why
static-init registration was rejected.
