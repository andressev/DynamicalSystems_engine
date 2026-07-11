# 0004 — Modules vs simulations (compile-time components, not a plugin system)

**Date:** 2026-07-11 · **Status:** accepted

## Context
The engine will grow a large number of reusable resources (integrators, vector fields,
colormaps, CUDA buffers...). Sims need to attach/detach them easily, and the
[[Home|obsidian twin]] must not rot as the count grows.

## Decision
1. **A module is a compile-time component**: one self-contained class, one header + one
   cpp (+ one vault note), RAII ownership, no globals. Attaching = declaring a member
   and calling it in `init`/`update`/`render`; detaching = deleting the member. No
   runtime attach/detach machinery (ECS/plugin registry) — that only pays off when
   modules must swap while running, which no current use case needs.
2. **Layer rule**: `core` (App, Window, Sim, SimRegistry — the skeleton) · `gfx`
   (GL-facing: Shader, Mesh, Camera, Shapes) · `math` (pure computation, **std-only** —
   unit-testable, CUDA-portable) · `compute` (future CUDA interop; may touch gfx).
   A module depends only on same-or-lower layers; math < gfx < compute.
3. **One module = one note** in `obsidian_twin/modules/` from `_template.md`
   (what it is / dependencies / how to use it / notes). The "How to use it" section is
   the attach recipe. Sim notes list "Modules attached"; the reverse direction
   ("used by") comes from Obsidian backlinks, never from hand-maintained lists.
4. **Enforcement is mechanical**: the `/new-module` and `/new-sim` skills scaffold the
   structure; `tools/check_twin.py` cross-checks headers ↔ module notes, sim folders ↔
   sim notes, and wikilink targets.
5. Headers stay flat in `engine/include/engine/` for now; physical `core/gfx/math/
   compute/` subfolders happen when the module count crosses ~15 (layer lives in note
   frontmatter until then).

## Consequences
- `Sim` stays minimal forever — new capabilities become modules sims pull in, never new
  virtuals on the interface.
- Every module is born documented; drift is detected, not remembered.
