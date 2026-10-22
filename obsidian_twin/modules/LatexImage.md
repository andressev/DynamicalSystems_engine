---
type: module
layer: gfx
status: wip
code:
  - engine/include/engine/LatexImage.h
  - engine/src/LatexImage.cpp
---
# LatexImage

`engine/include/engine/LatexImage.h`, `engine/src/LatexImage.cpp`

## What it is
A LaTeX math string typeset into a GL texture, for on-screen formulas such as
$\dot{v} = Av$ with the live matrix. MicroTeX does the TeX parsing and box layout; this
module is its platform backend: a `tex::Graphics2D` that rasterizes onto a CPU canvas
(stb_truetype glyphs, exact-coverage rules, distance-AA lines), uploaded once as an RGBA
texture and drawn with `ImGui::Image`. Motivated by
[[0007-latex-labels-microtex-imgui]].

## Dependencies
- Engine: none.
- External: glad, imgui (`imstb_truetype.h`, compiled privately with `STBTT_STATIC`),
  glm, MicroTeX + tinyxml2 (the `microtex` target, linked **privately** into `engine` —
  only `LatexImage.cpp` sees MicroTeX headers).

## How to use it
1. Member: `engine::LatexImage label;` — default-constructed is empty, no GL yet.
2. When the formula changes (not every frame): `label.set(R"(\dot{v} = Av)", 24.0f);`
   On a parse error it returns `false`, keeps the previous image and fills `error()`.
3. Inside any ImGui window (e.g. from `update()` or `drawUI()`): `label.draw();`.
4. Reference sim: [[diagonal-linear]] — top-right $\dot{v} = Av$ overlay, re-typeset
   only when a $\lambda_i$ moves; and a per-frame $\phi^t(v_0)$ label riding each
   single particle, composed from cached pieces (see Notes).

Interface:
- `set(latex, textSize = 22, color = white)` — typeset; `textSize` is pixels per em.
- `draw()` — `ImGui::Image` at natural pixel size.
- `drawAt(x, y, alpha, foreground = false)` — top-left at a screen point (ImGui display
  pixels) on the background draw list: over the 3D scene, under every ImGui window; with
  `foreground` on the foreground list, over the windows too (titles a panel must never
  hide — [[lorenz-equilibrium]]'s ball names). For labels that follow something in the
  world, project it with the [[Camera]] matrices first.
- `baseline()`, `advance()`, `kPad` — for composing pieces on one line: the image is the
  TeX box plus `kPad` px all round; `baseline()` is px from the image top,
  `advance()` the box width.
- `empty()`, `texture()`, `width()`, `height()`, `source()`, `error()`.

## Notes
- MicroTeX is initialized lazily on the first `set()` with fonts from
  `External/microtex/res` (absolute path baked in via `MICROTEX_RES_DIR`), and never
  released — it holds process-wide static state.
- The three MicroTeX platform factories (`tex::Font::create`, `Font::_create`,
  `TextLayout::create`) are defined in `LatexImage.cpp`: exactly one backend per
  program. `\text{}` maps "Serif"/"SansSerif" to the bundled Computer Modern text fonts.
- Glyphs honor translation and scale, **not rotation** (`\rotatebox` would draw upright).
- Cost: a layout plus a full CPU raster per `set()` — measured ~3 ms for
  $\phi^{1.23}(v_0)$ in Debug. Fine for a formula that changes on user input, not
  for one that changes every frame. **Per-frame text pattern**: typeset the static parts
  and each possible character once, then lay them out every frame with
  `baseline()`/`advance()` and `drawAt()` — zero typesetting per frame
  ([[diagonal-linear]]'s `TimeLabelGlyphs`: superscripts at $0.7\times$ size raised
  $0.42$ em, TeX's script proportions).
- The texture is at the rendered pixel size; no HiDPI scaling yet.
