# 0006 — Vector fields are sampled on the CPU into a dynamic mesh

**Date:** 2026-09-24 · **Status:** accepted

## Context
Every sim so far evaluates the user's typed function **in a shader**: [[Expression]]
generates GLSL, it is spliced into a template and the program is hot-recompiled on Apply
([[0003-hand-rolled-parser-glsl-codegen]]). The M3 vector-field work needed the same
typed function in a place a shader cannot serve it: an [[Integrator]] stepping a
trajectory needs $F(\mathbf{x}, t)$ at a point that depends on the previous answer, on
the CPU, thousands of times a second.

That leaves a choice for the *glyph grid*: evaluate the field twice (GLSL for arrows,
C++ for curves), or once on the CPU for both.

## Decision
1. **One evaluation path, on the CPU.** The field is evaluated only through
   `Expression::evalReal` — a real-valued twin of `eval` added for this — behind
   [[VectorField]]. Arrows and trajectories therefore always agree by construction; there
   is no second implementation of the field to drift.
2. **Glyph geometry is rebuilt into a dynamic [[Mesh]]**, not generated in a shader.
   [[Mesh]] gained an index-less `GL_DYNAMIC_DRAW` constructor plus `setVertices` /
   `drawArrays` for this, which [[Trajectory]] reuses for trails.
3. **No GLSL codegen in this sim at all** — it ships a plain shader pair with a color
   ramp. Apply changes data, never the program.
4. `evalReal` is added to [[Expression]] rather than a new parser: same AST, same
   language, a second walker. `usesVariable` came with it, so a sim can ask whether the
   field really depends on $t$ before paying for a per-frame rebuild.

## Consequences
- Cost is bounded by the grid, not the screen: a 27×27×27 lattice is ~20k arrows and
  ~40k AST evaluations per rebuild, which measures in the low milliseconds and only runs
  when something changed (or every frame for a time-dependent field — still 400+ FPS).
  A fragment-shader field would instead pay per pixel, every frame, forever.
- The glyph grid is CPU-side geometry, so it can be **queried** — the sim knows
  $|F|_{\max}$ over the lattice and uses it for the auto color scale. A shader field
  would need a reduction pass for the same number.
- This is deliberately the CUDA on-ramp ([[roadmap]] M4): the expensive part is now a
  data-parallel loop over lattice points and seeds writing into a VBO — exactly the
  shape `cudaGraphicsGLRegisterBuffer` wants, with [[Mesh]] already the single owner of
  that buffer.
- The cost is a real one: field structure below the lattice spacing is invisible
  (noted in [[integral-curves]]), where a per-pixel shader field would resolve it. If a
  dense continuous view (LIC, streamline textures) is ever wanted, that is a *second*
  view built on the same [[VectorField]], not a replacement for this one.
