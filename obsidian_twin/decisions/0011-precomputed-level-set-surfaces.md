# 0011 — Precomputed level-set surfaces

**Date:** 2026-10-20 · **Status:** accepted — implemented in [[lorenz-manifold-growth]]

## Context
[[lorenz-manifold-growth]] should grow the 2-D manifolds ($W^s(O)$ first) smoothly from
a button, but growing them by geodesic level sets takes about a minute of BVPs (debug)
and lives in [[learn-geodesic-level-sets]].

## Decision
Grow once in [[learn-geodesic-level-sets]] (Grow rings, then **Save surface**) and store
the bands with [[LevelSetSurface]] in `sims/lorenz_manifold_growth/data/surface_<O|Cp|Cm>_<s|u>_rho<rho>.lss`
(one file per $\rho$ since 2026-10-21),
tagged with $\sigma, \rho, \beta$, K order, equilibrium, side and the $W_{loc}$ radius.
The growth sim loads every file at init and offers its `Grow W` only when the tags match
the live field and K order — otherwise the first band would not meet its $W_{loc}$.
Each vertex stores where it grew from, so the growth is animated on the GPU
(`band_vertex.glsl`) with no solving at run time.

## Consequences
- Only the stored parameters grow a surface (the classic ones for $W^s(O)$); other
  parameters still say so in the popup.
- The data file is large (~13 MB for $W^s(O)$ out to $\eta = 97$) and must be re-saved
  when the method or the patch changes.
