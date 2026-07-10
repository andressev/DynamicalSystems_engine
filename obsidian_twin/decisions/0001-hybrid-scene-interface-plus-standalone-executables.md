# 0001 — Hybrid: Scene interface + per-sim standalone executables

**Date:** 2026-07-10 · **Status:** accepted

## Decision
Every simulation implements the [[Sim]] interface and is built as its own static library
`sim_<name>`. The explorer executable links all sims and offers a runtime picker; the
`add_sim()` CMake helper additionally generates a `<name>_standalone` executable linking
engine + that one sim.

## Why
- Andres wants both: an engine feel with runtime switching, **and** the ability to work
  on one sim in total isolation.
- Per-sim static libs make the standalone build trivial (link exactly one lib) and keep
  compile times proportional to what you're working on.

## Consequence
Registration must be explicit (factory function per sim, one `add()` line per
executable): static-init self-registration inside a static library gets stripped by the
linker unless forced with whole-archive hacks. See [[sim-lifecycle]].
