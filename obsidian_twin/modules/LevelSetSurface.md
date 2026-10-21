---
type: module
layer: math
status: stable
code:
  - engine/include/engine/LevelSetSurface.h
  - engine/src/LevelSetSurface.cpp
---
# LevelSetSurface

`engine/include/engine/LevelSetSurface.h`, `engine/src/LevelSetSurface.cpp`

## What it is
A 2-D manifold grown by geodesic level sets, kept band by band so it can be saved once
and grown again later without solving any BVP ([[0011-precomputed-level-set-surfaces]]).
Band $k$ is the strip of triangles between level sets $k$ and $k+1$, with $\eta_k$ the
geodesic distance of its outer level set from the first edge. Every vertex also keeps
*where it grew from* — its start on the previous level set (a vertex on that level set
grew from itself) — so a band can be drawn partly grown, sliding from → pos.

## Dependencies
- Engine: none
- External: std-only (`<map>`, `<string>`, `<vector>`, `<fstream>`)

## How to use it
1. Member: `engine::LevelSetSurface surf;`
2. Writing: push `Band{eta, pos, from}` per accepted ring (triangles, xyz per vertex,
   `from` in the same layout), set `tags` (what it was grown for: parameters, K order,
   radius), `save(path)`. Reference: [[learn-geodesic-level-sets]] (`saveSurface`).
3. Reading: `load(path)` (false → empty), `tag(key)` to check it fits, `bands` /
   `vertexCount()` to build a mesh. Reference: [[lorenz-manifold-growth]]
   (`loadStoredSurfaces`, drawn by its `band_vertex.glsl`).

Interface: `bands`, `tags`, `save`, `load`, `tag`, `vertexCount`, `empty`.

## Notes
- Binary, little-endian, magic `LSS1`: tags (name, double), then per band $\eta$, vertex
  count, positions, froms. Floats — the data is for drawing.
- Files use `.lss` (`*.bin` is git-ignored).
