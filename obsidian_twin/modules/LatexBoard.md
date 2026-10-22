---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/LatexBoard.h
  - engine/src/LatexBoard.cpp
---
# LatexBoard

`engine/include/engine/LatexBoard.h`, `engine/src/LatexBoard.cpp`

## What it is
Pages of LaTeX read from a text file and shown as a centered white card over the
scene: the equations of a talk inside the running sim, so presenting needs no switch to
slides (Alt-Tab to PowerPoint hitched). [[App]] owns one for every sim
([[0012-latex-board-per-sim]]).

## Dependencies
- Engine: [[LatexImage]] (one per formula row).
- External: imgui, std.

## How to use it
Sims don't attach it: [[App]] points it at `sims/<name>/board.tex` on every switch, so
a sim gets a board by adding that file. Keys (in [[App]]): **L** shows / hides,
**PageDown** / **PageUp** turn pages (what a presentation clicker sends); the ◀ ▶
arrows at the bottom of the card turn them by click.

`board.tex` format:
- one formula per line, math mode, centered on its own row; prose as `\text{...}`;
- a blank line is vertical space; a line `---` starts the next page;
- lines starting with `%` are comments.

Interface: `setFile(path)` (closes), `open()` (re-reads the file, keeps the page),
`close()`, `toggle()`, `visible()`, `nextPage()` / `prevPage()`, `draw()` (between
`ImGui::NewFrame` and `ImGui::Render`). Reference: [[diagonal-linear]]'s `board.tex`.

## Notes
- The file is read on every open, so edit it while the app runs and press L twice.
- Text size is $0.045 \times$ the display height, so the card reads the same on a
  laptop and a projector; a resize re-typesets the page (only the current one is typeset).
- A row that fails to parse shows in red with MicroTeX's error; a missing file shows
  its path on the card.
- The card takes the mouse only where it is (sims skip clicks while ImGui wants the
  mouse); it can't be moved and stays under the Explorer panel.
