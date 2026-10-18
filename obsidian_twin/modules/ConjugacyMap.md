---
type: module
layer: gfx
status: wip
code:
  - engine/include/engine/ConjugacyMap.h
  - engine/src/ConjugacyMap.cpp
  - sims/lorenz_manifold/shaders/conjugacy_vertex.glsl
---
# ConjugacyMap

`engine/include/engine/ConjugacyMap.h`, `engine/src/ConjugacyMap.cpp`

## What it is
The manifold-exact Hartman–Grobman map at a hyperbolic equilibrium, evaluated **per
vertex on the GPU**:

$$\tilde h^{-1}(\theta) = K_s(s) + K_u(u), \qquad \theta = B_s s + B_u u,$$

from the two [[InvariantManifold]] parametrizations. On $E^s$ and $E^u$ it is the
conjugacy restricted to the invariant manifolds; between them it is an interpolation
(the true $h$ is not computable this way). Both sides' Taylor coefficients live in one
uniform buffer; `glsl()` is the matching `vec3 hInv(vec3 theta)` a sim splices into its
vertex shader, so a sphere (the deformed ball $V$) or a disk in $E^s$ (the $W^s$ patch)
deforms in the vertex stage and changing $r$ is only a uniform
([[0008-gpu-work-split-vertex-stage-and-cuda]]). `hInv()` is the same map on the CPU in
double, for the few points that need exactness (trajectories).

## Dependencies
- Engine: [[InvariantManifold]], [[Eigen3]] (math, lower layer); [[Shader]] on the
  user's side for `bindUniformBlock`
- External: glad (owns one `GL_UNIFORM_BUFFER`)

## How to use it
1. Members: `engine::ConjugacyMap hm;` and a shader whose vertex source has
   `ConjugacyMap::glsl()` spliced in (after `#version`). Once after creating it:
   `shader.bindUniformBlock(ConjugacyMap::kBlockName, ConjugacyMap::kBinding)`.
2. On parameter change: build `Ws`/`Wu` ([[InvariantManifold]], order
   $\le$ `kMaxOrder` = 16) and `hm.set(Ws, Wu)` — false (`error()`) unless the dims add to 3.
3. `render`: `hm.bind()`, then in GLSL `x = xStar + hInv(theta)`.
4. CPU: `hm.hInv(theta)`, `hm.split(theta, s, u)`, `hm.stable()` / `hm.unstable()`.
5. Reference: [[lorenz-equilibrium]] stage C (the deformed ball $V$, pair seeds, the
   radius check); also the "Conjugacy preview" section of [[lorenz-manifold]] (GPU-deformed
   $E^s$ disk that must lie on the CPU surface).

## Notes
- std140 block: `mat4` split matrix, `ivec4[2]` (k, order, complex pair), then
  `vec4[2 · 153 · 2]` coefficients laid out triangularly for order 16 (~9.9 KB, under
  the 16 KB guaranteed UBO size). The C++ `Block` struct mirrors it (static_assert).
- **Float on the GPU**: coefficients and evaluation are `float` — fine for drawing;
  anything that must be exact (orbit points, rim crossings) uses the CPU `hInv`.
- Normals of a deformed mesh are forward differences in the mesh's own parameters
  (step $10^{-3}$), three `hInv` calls per vertex.
- Trust radius is the [[InvariantManifold]] one: at $C_\pm$ ($\rho=28$) keep
  $r \lesssim 2$; the preview shows `maxRelDefect(r)` for both sides.
- Naming: the sims' labels write this map as $\tilde h : U \to V$ since 2026-10-18 (the
  user's preference — the map sends the linear ball to the true one); the code keeps
  `hInv`/`hInv()` from when it was written as $\tilde h^{-1}$.
