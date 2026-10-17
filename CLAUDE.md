# Math Explorer

An engine for exploring ODEs — flows on manifolds, vector-field visualizers with live
function input — built on OpenGL, eventually with CUDA interop for GPU compute.
Simulations ("sims") are self-contained modules; the complex graphing calculator on the
Riemann sphere is the first sim.

## Build & run

Requires a Visual Studio developer environment (the CMake presets read `VSINSTALLDIR`;
from a plain shell, run via `VsDevCmd.bat -arch=x64` first).

```
cmake --preset x64-debug
cmake --build out/build/x64-debug
```

- Explorer (all sims, runtime picker): `out/build/x64-debug/apps/explorer/explorer.exe`
- Standalone (one sim only): `out/build/x64-debug/sims/riemann_sphere/riemann_sphere_standalone.exe`

## Layout

| Path | What it is |
|---|---|
| `engine/` | Static library: Window, Shader, Camera, Mesh, Shapes, Expression (function parser → CPU eval + GLSL codegen), Sim interface, SimRegistry, App loop, glad + ImGui |
| `sims/<name>/` | One sim per folder: `Sim` subclass, factory, own `shaders/`, own CMakeLists calling `add_sim()` |
| `apps/explorer/` | Multi-sim executable; registers every sim |
| `apps/standalone/` | Generic single-sim `main.cpp`, compiled once per sim by `add_sim()` |
| `tests/` | `math_tests` exe: std-only checks of the math-layer modules (`out/build/x64-debug/tests/math_tests.exe`) |
| `External/` | Vendored deps (glfw, glad, glm, stb_image, imgui, microtex + tinyxml2 — built by `cmake/microtex.cmake`, linked privately into engine for `LatexImage`) — never modify |
| `obsidian_twin/` | Obsidian-style vault mirroring the architecture (see rule below) |
| `learnOpenGL/`, `custom/`, `explanations/` | Frozen reference playground — **never modify** |

## Modules vs simulations

The project has two kinds of parts (decision `obsidian_twin/decisions/0004-...`):
**modules** — self-contained engine classes sims attach by declaring a member (one
header + one cpp + one note in `obsidian_twin/modules/`, all three or nothing) — and
**sims**. No runtime attach/detach machinery; composition is compile-time.

Modules are layered; a module depends only on same-or-lower layers:
`core` (App/Window/Sim/SimRegistry skeleton) · `gfx` (GL-facing) · `math` (pure
computation, **std-only** — no glad/glfw/glm) · `compute` (future CUDA, may use gfx).
Layer is declared in the note's frontmatter; headers stay flat in
`engine/include/engine/` until ~15 modules. New module: use the `/new-module` skill.

## Conventions

- Sims implement `engine::Sim` (`engine/include/engine/Sim.h`) and expose a factory
  function `std::unique_ptr<engine::Sim> make<Name>Sim();` in their header. The sim class
  itself stays in the .cpp. Registration is explicit in `apps/explorer/main.cpp`
  (one `app.sims().add(...)` line per sim) — no static-init registration.
- GL resource ownership lives only in engine wrappers (`Mesh`, `Shader`); no raw
  `glGen*`/`glDelete*` in sims. `Mesh` is the future CUDA-interop point.
- Shaders live in the sim's own `shaders/` folder, loaded via the `SIMS_DIR` compile
  definition: `SIMS_DIR "<sim_name>/shaders/file.glsl"`.
- New sim: use the `/new-sim` skill (`.claude/skills/new-sim/SKILL.md`).
- Engine code is `namespace engine`, headers under `engine/include/engine/`.

## Obsidian twin rule

`obsidian_twin/` is the architecture's markdown mirror (for AI + human navigation,
Obsidian `[[wikilinks]]`). **Before non-trivial work, open `obsidian_twin/Home.md` and
follow its routing table** ("if you're doing X, read Y first"), plus the `## Now`
section of `architecture/roadmap.md` for current state — and update `## Now` when the
state of in-flight work changes. Shared math concepts live as notes in
`obsidian_twin/math/` (idea / equations / where implemented / pitfalls) and are linked
from sim notes, never re-derived in them. Write math in vault notes as Obsidian LaTeX —
`$...$` inline, `$$...$$` for display equations; backticks only for actual code/GLSL
identifiers, never for formulas. **Any change to a module's public interface or any new
architectural mechanism requires updating the matching note** in `obsidian_twin/modules/`
or `obsidian_twin/architecture/`. New sims get a note in `obsidian_twin/sims/` from
`_template.md`, linked from `Home.md`. Significant design choices get a short record in
`obsidian_twin/decisions/`. Notes are short pointers into code — never code copies.
Module notes follow `modules/_template.md` (what it is / dependencies / how to use it /
notes); sim notes follow `sims/_template.md` and must list "Modules attached". Never
hand-maintain "used by" lists — Obsidian backlinks cover the reverse direction. After
touching engine headers, sims, or the vault, run `python tools/check_twin.py` — it
cross-checks headers ↔ module notes, sim folders ↔ sim notes, and wikilink targets.
