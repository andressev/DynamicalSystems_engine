---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/ArrowGlyphs.h
  - engine/src/ArrowGlyphs.cpp
---
# ArrowGlyphs

`engine/include/engine/ArrowGlyphs.h`, `engine/src/ArrowGlyphs.cpp`

## What it is
A batch of arrow glyphs — a field's quiver, the red $\dot{x}$ tangent on a particle, the
flow-direction cones on an eigenspace — drawn with **GPU instancing**. The CPU writes one
8-float record per arrow (tail, unit direction, length, $|f|$); the shaft/cone/barb
shapes are tiny static base meshes in the arrow's local frame and the vertex shader
places, orients and lights them ([[0008-gpu-work-split-vertex-stage-and-cuda]]).
Extracted (2026-10-16) from identical private copies in [[vector-field]] and
[[diagonal-linear]] (M0 of the Hartman–Grobman track, [[roadmap]]); went instanced the
same day — previously every cone was ~250 floats built and uploaded per rebuild.

## Dependencies
- Engine: [[Mesh]] (instanced draws), [[Shader]] (sets two uniforms)
- External: glm, glad (through [[Mesh]])

## How to use it
1. Member: `engine::ArrowGlyphs glyphs;` — no GL work until `upload()`. One instance per
   independently colored/rebuilt group (grid, tangents, one per subspace).
2. Rebuild: `clear()`; style knobs `barbs` / `headFraction` / `headWidth` / `lod`
   (shared by the whole batch — it is one mesh); `add(p, dir, len, mag, dim)` per arrow
   or `addField(points, vectors, baseLen, colorMax, hideBelow, lengthByMagnitude, dim)`;
   then `upload()` — **also on early-out paths**, it is what empties the batch.
3. `render`, glyph shader bound: `drawShafts(shader)` (fade on), `drawHeads(shader)`
   (shade mode on). Both set `u_instanced`/`u_flat` and reset `u_instanced` to 0.
4. The sim's vertex shader must implement the instanced path — see the contract in the
   header (locations 4–7, `u_instanced`, `u_flat`) and the copy in
   `sims/vector_field/shaders/vertex.glsl`.
5. Reference sims: [[vector-field]] (R2 + R3 grid, tangents), [[diagonal-linear]]
   (grid, tangents, head-only cones per subspace with `headFraction = 1`).

Interface: `add`, `addField`, `clear`, `upload`, `drawShafts`, `drawHeads`, `arrows()`,
`empty()`, static `lodFor(arrows)`.

## Notes
- **Lighting moved to the GPU**: cone facets carry a local normal; the shader rotates it
  into the arrow frame and applies the fixed light (same constants the CPU used). R2
  triangles keep a fixed per-vertex shade (1 at the tip, 0.88 at the base).
- The arrow frame is built in the shader exactly as the CPU did (`helper` axis switch at
  $|d_z| = 0.9$, or $(-d_y, d_x, 0)$ in R2), so glyphs look identical to before.
- `dir` must be unit length; `mag` only feeds the color ramp.
- Base shapes rebuild only when the style or dimension changes; instance buffers only
  grow (n + n/4 + 64 slack).
- No sub-range draws (GL 3.3 has no base instance): split groups into batches.
- Planned: an optional clip ball for the Lorenz equilibrium sim — shader-side `discard`
  outside $B_r(x^*)$.
