# 0008 — GPU work split: vertex stage for drawn data, CUDA for evolving state

**Date:** 2026-10-16 · **Status:** accepted

## Context
Two costs showed up before the Hartman–Grobman sims: rebuilding dense glyph grids
(every cone built and uploaded on the CPU — the reason `kLiveRebuildArrows` exists) and
the coming per-vertex deformations by $\tilde h^{-1}$ (a sphere, a disk). CUDA is the
long-term compute path ([[roadmap]] M4; geodesic manifold growth will need it), and GL
compute shaders were the alternative. Machine: RTX 4060 Laptop, CUDA 12.4/13.2, GL 3.3
core context.

## Decision
1. **Data that is only drawn is computed in the vertex stage.** Glyphs are instanced
   ([[ArrowGlyphs]], [[Mesh]] instancing): the CPU writes 8 floats per arrow. Mesh
   deformations by a known map run in GLSL ([[ConjugacyMap]]); parameters change by
   uniform, not by re-upload.
2. **State that evolves or is read back goes to CUDA**, not GL compute shaders — one
   GPU compute path, double precision and real tooling when needed, interop through
   [[Mesh]]'s VBO. First kernel: many-seed advection (M4), after the HG sims.
3. **Until then, CPU parallelism with `std::execution::par`** (MSVC STL, no TBB) for
   embarrassingly parallel CPU loops: field sampling for glyphs ([[VectorField]] made
   thread-safe), Taylor rings in [[lorenz-manifold]].

## Consequences
- Supersedes point 2 of [[0006-cpu-field-sampling-dynamic-glyph-mesh]] (glyph geometry
  is no longer CPU-built); its point 1 stands — the field is still evaluated once, on
  the CPU, so arrows and trajectories agree.
- Sims drawing glyphs must carry the instanced path in their vertex shader (contract in
  `ArrowGlyphs.h`).
- No compute shaders, so the GL 3.3 context stays; no base-instance draws (batches
  instead).
- CUDA on a laptop needs the NVIDIA GPU to own the GL context (`NvOptimusEnablement`)
  for interop — to handle when M4 starts.
