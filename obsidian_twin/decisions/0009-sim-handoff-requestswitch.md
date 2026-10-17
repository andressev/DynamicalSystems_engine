# 0009 — Sim-to-sim handoff: deferred `requestSwitch` + a key→number map

**Date:** 2026-10-16 · **Status:** accepted — implemented in [[App]]

## Context
The Hartman–Grobman track is two sims kept apart so neither carries the other's UI
([[roadmap]]): [[lorenz-flow]] (global) and [[lorenz-equilibrium]] (local, inside a
ball $V$ around $x^*$). Clicking a bead must carry you from one to the other as one
continuous motion, and back. Until now a sim could only be entered from the menu, and
sims never know each other (separate static libs, [[0004-modules-vs-simulations]]).

## Decision
1. **`App::requestSwitch(name, Handoff)`**, applied at the end of the frame (like the
   menu button), so the requesting sim finishes its `update`/UI. Returns false when the
   target isn't registered — standalone builds just don't offer the switch (`hasSim`).
2. **`Handoff` is `std::map<std::string, double>`**, read by the new sim in `init` via
   `App::handoff()` (empty when started from the menu). Plain numbers keep sims free of
   each other's headers; the keys are a contract between the two sims, written down in
   their notes.
3. **The shared [[Camera]] is the continuity.** App doesn't touch it on a switch; the
   receiving sim simply doesn't reframe when it was handed over. Each side fades its own
   content (out before leaving, in after arriving), so nothing pops.

## Consequences
- A sim's in-memory state (trajectories) does not survive the round trip — only what's in
  the map. Enough here: parameters, which $x^*$, the global view to return to.
- Keys are stringly typed; a typo silently falls back to the default. Acceptable for two
  sims; a typed struct would need a shared header (a module) if this spreads.
- The standalone `lorenz_flow` keeps the old behaviour: the zoom lands and stays.
