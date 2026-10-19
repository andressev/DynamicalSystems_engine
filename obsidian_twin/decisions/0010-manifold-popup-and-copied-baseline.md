# 0010 — Manifold growth as its own sim: hover popup per bead, lorenz_flow's baseline copied

**Date:** 2026-10-18 · **Status:** accepted — implemented in [[lorenz-manifold-growth]]

## Context
Growing $W^s$/$W^u$ of each Lorenz equilibrium needs the global picture of
[[lorenz-flow]] (field, beads, trajectories) but a different interaction on the beads:
no zoom/handoff, instead controls *per equilibrium and per side* (grow $W_{loc}$, grow
$W$, watch a branch). Bolting that onto lorenz_flow would mix two UIs (the same reason
[[lorenz-equilibrium]] is separate, [[0009-sim-handoff-requestswitch]]).

## Decision
1. **A new sim**, `lorenz_manifold_growth`, that **copies** lorenz_flow's baseline
   (settings table, beads + picking + halos, balls/singles, arrows, overlays) and drops
   the zoom/handoff, rather than extracting that baseline into engine modules first.
   Two sims sharing by copy is the precedent (lorenz_equilibrium); a third would make
   an `EquilibriumBeads`-style module worth it.
2. **The per-equilibrium UI is a popup beside the bead**, not panel sections: it opens
   on hover, stays while the mouse is on the bead or on it (with a grace), and a click
   on the bead pins it. The panel keeps one row per bead (hover lights, click pins) so
   a pinned popup is reachable when the bead is off-screen (e.g. while watching).
3. **Button faces are typeset** ([[LatexImage]] textures in `ImGui::ImageButton`) so
   the notation is the real one — $W^u_{\mathrm{loc}}(O)$, $W^s(C_+)$ — with a plain
   text fallback if the typesetting fails.
4. **Surfaces stay unavailable** past $W_{loc}$ until geodesic level sets exist
   ([[global-manifold-growth]]); the popup shows the disabled button rather than
   hiding the plan. "Learn geodesic level sets" became its own sim,
   [[learn-geodesic-level-sets]], reached by a zoom + handoff like lorenz_flow →
   lorenz_equilibrium: the walkthrough is a different UI on a different scene.

## Consequences
- ~1700 lines with a large overlap with lorenz_flow; fixes to the shared baseline must
  be applied twice until a module exists.
- The popup is a third place UI lives (panel, overlays, popup); it follows the panel's
  "no explanations" rule — name, type, the buttons, the trusted radius, nothing else.
