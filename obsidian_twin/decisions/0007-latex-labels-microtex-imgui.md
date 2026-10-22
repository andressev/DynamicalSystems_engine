# 0007 — LaTeX labels via MicroTeX, drawn through ImGui

**Date:** 2026-10-11 · **Status:** accepted — implemented as [[LatexImage]]

## Context
Sims want typeset math on screen: a label riding a trajectory's head, e.g.
$\varphi^{t}(v_0)$ with the live $t$, and a corner readout of the system,
$\dot{v} = f(v)$, showing the actual $f$ the user typed. ImGui alone only draws plain
text, and KaTeX/MathJax would mean a browser engine.

## Decision
1. **MicroTeX** (C++, MIT) does the LaTeX parsing and box layout. It is vendored in
   `External/microtex` (core sources + `res/` fonts) with its one dependency
   **tinyxml2** in `External/tinyxml2`; `cmake/microtex.cmake` builds them as the static
   library `microtex`. Nothing links it yet.
2. **Drawing will go through ImGui**, by implementing MicroTeX's abstract
   `tex::Graphics2D` (glyphs, lines, rects, paths) on an `ImDrawList`. That serves both
   cases with no new GL code: the foreground draw list at a screen point projected from
   the head (via [[Camera]]) for the trajectory label, and a fixed-corner window for
   $\dot{v} = f(v)$.
3. MicroTeX's bundled platform backends (Qt, Skia, Cairo) were dropped when vendoring;
   `src/platform/gdi_win` is kept only as a reference implementation of
   `Graphics2D`.

**As built (same day):** rather than drawing glyph-by-glyph into an `ImDrawList`, the
backend rasterizes the whole formula on the CPU into one texture shown by
`ImGui::Image` ([[LatexImage]]). Simpler and pixel-exact; the cost is a re-raster per
change, which suits formulas that change on user input.

## Consequences
- The backend is the gfx-layer module [[LatexImage]]; `engine` links `microtex`
  privately, so only `LatexImage.cpp` sees MicroTeX headers. Glyphs come from the
  vendored `.ttf` fonts via stb_truetype.
- A label whose text changes every frame ($\varphi^t(v_0)$ with live $t$) would pay a
  full re-raster per frame — it wants a glyph cache or a static/dynamic split first.
- $\dot{v} = f(v)$ needs the typed $f$ as LaTeX, i.e. a LaTeX printer in [[Expression]]
  next to its GLSL codegen.
- Upstream MicroTeX is quiet since 2024; it is vendored at a fixed commit (see
  `External/microtex/VENDORED.txt`), so that is acceptable.
