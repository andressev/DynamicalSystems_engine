# 0012 — LaTeX board per sim, owned by App

**Date:** 2026-10-21 · **Status:** accepted — implemented in [[App]] + [[LatexBoard]]

## Context
Presenting meant Alt-Tab between the explorer and PowerPoint, and every switch hitched
(first exclusive full screen, then still too slow). The equations a talk needs are few
and belong next to the sim anyway.

## Decision
- [[LatexBoard]]: pages of LaTeX from a text file on a centered white card.
- [[App]] owns one and points it at `sims/<name>/board.tex` (the registry name is the
  sim folder) on every switch; **L** toggles it, **PageDown** / **PageUp** or its ◀ ▶ arrows page it — in
  every sim, with no per-sim code. The engine gets its own `SIMS_DIR` definition for
  the path.
- The content is a text file, not C++: edited without a rebuild, re-read on each open.

## Consequences
- A sim gets a board by adding `board.tex`; no file = the card says so.
- L, PageDown and PageUp are reserved app-wide, like H and F11.
- Paging deliberately skips the arrow keys (clickers also send them), since arrows are
  the flow speed / direction ([[FlowKeys]]).
