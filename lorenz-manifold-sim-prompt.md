# Build: Lorenz stable-manifold geodesic level sets (CPU compute + OpenGL render)

## Context
Extend my existing C++/OpenGL app (WSL2 + CMake, LearnOpenGL-style, VAO/VBO/EBO,
GLM, shader hot-reload, a `Shapes` utility, an existing camera with
view/projection uniforms). Add a self-contained module that computes the **2-D
stable manifold Wˢ(0) of the Lorenz system** on the CPU in double precision and
renders it as geodesic rings of points. Reuse my camera and shader-loading
utility; add one dedicated point/line shader pair.

Everything numeric is `double`; cast to `float` only when filling GL buffers.

## The math, briefly
Lorenz: `ẋ = σ(y−x)`, `ẏ = ρx − y − xz`, `ż = xy − βz`. Equilibrium at the origin.
Jacobian there:

```
A = [ -σ   σ   0 ]
    [  ρ  -1   0 ]
    [  0   0  -β ]
```

Spectrum (σ=10, ρ=28, β=8/3): one **unstable** value ≈ +11.83 and two **stable**
values (−β = −2.667 and ≈ −22.83). We want the 2-D **stable** manifold spanned by
the two stable eigendirections — the classic "Lorenz manifold."

The parametrization method gives Taylor coefficients `p[c][m][n]` (c ∈ {x,y,z}) of
a map `K : R² → R³`,

```
K_c(θ1,θ2) = Σ_{m+n ≤ order} p[c][m][n] · θ1^m · θ2^n
```

`K` **is** the local manifold, accurate inside a disk (its radius of convergence).
Geodesic level sets are rings `C_i` at (approximately) geodesic distance `η_i`
from the origin; `M_i` is the mesh of points on `C_i`.

## Phase 1 — MVP: render the manifold straight from K (no ODE)

### 1. Eigendata
```
root = sqrt((σ+1)² + 4σ(ρ−1))
l1   = -β
l2   = (-(σ+1) - root) / 2          // the two STABLE eigenvalues used by K
ev1  = (0, 0, 1)                     // eigenvector for l1
ev2  = (σ/(σ+l2), 1, 0)             // eigenvector for l2
```
(For orientation only, the unstable value is `(-(σ+1)+root)/2`; not needed here.)

### 2. Coefficient recursion — port this faithfully, do NOT re-derive the signs
Channels: `x=0, y=1, z=2`. Store `p[3][order+1][order+1]`, zero-initialized.

Cauchy product (2-D convolution) of two channels:
```
S(p, c1, c2, m, n) = Σ_{i=0..m} Σ_{j=0..n}  p[c1][i][j] * p[c2][m-i][n-j]
```

Seed order-1 terms, then solve order by order:
```
p[:, 1, 0] = ev1
p[:, 0, 1] = ev2

for d = 2 .. order:
    for m = 0 .. d:
        n  = d - m
        mu = m*l1 + n*l2
        Mmat = A - mu * I3                       // 3x3
        b = ( 0,
              +S(p, 0, 2, m, n),                 // conv(x,z)  → the -xz term row
              -S(p, 0, 1, m, n) )                // conv(x,y)  → the  xy term row
        p[:, m, n] = solve(Mmat, b)              // 3x3 linear solve
```
Because `p[:,0,0] = 0`, the convolutions never reference the current order, so each
solve depends only on already-computed lower orders. `Mmat` is nonsingular as long
as `mu` is not an eigenvalue of `A` (holds for these Lorenz values / real θ).

Default `order = 10`.

### 3. Orthonormalize the tangent basis (important)
The induced metric at the origin is `g(0) = [ev1 ev2]ᵀ[ev1 ev2]`. Here `ev1 ⟂ ev2`
but `|ev2| ≠ 1`, so equal steps in (θ1,θ2) are NOT equal length. Work in isotropic
coordinates (a,b):
```
s2 = |ev2| = sqrt( (σ/(σ+l2))² + 1 )
θ1 = a
θ2 = b / s2
```
so a unit (a,b)-circle is a metric circle to O(radius²).

### 4. Build the rings
Params: `N` rings, `M` points/ring, `dRho` radial step.
```
for i = 1 .. N:
    rho = i * dRho
    for k = 0 .. M-1:
        psi = 2π k / M
        a = rho*cos(psi);  b = rho*sin(psi)
        θ1 = a;  θ2 = b / s2
        P_ik = K(θ1, θ2)          // evaluate the Taylor polynomial → R³ point
```
Evaluate `K` by precomputing powers of θ1, θ2 and summing over `m+n ≤ order`.

### 5. (Optional, improves mesh) equal-arc redistribution
Per ring, oversample `psi` finely (say 8×M), accumulate **Euclidean** chord length in
R³ along the ring, then resample to `M` points at equal arc length. This is the
cheap stand-in for induced-metric arc length and needs no tensors. Skip for the
first run; add once points look correct.

These `C_i` are approximate geodesic circles (exact to O(rho²)); that's expected and
fine — do not try to make them exact.

## Rendering
- One `std::vector` of rings, e.g. `std::vector<std::vector<glm::dvec3>>`, plus a
  flat `std::vector<glm::vec3>` (float) + per-vertex level index for upload.
- Upload once to a VBO (regenerate on parameter change). Draw:
  - `GL_POINTS` for the mesh (`gl_PointSize` in the vertex shader),
  - `GL_LINE_LOOP` per ring for structure (one draw call per ring, or a shared
    EBO with per-ring loops),
  - optionally radial `GL_LINE_STRIP` connecting `M_i[k]` across i for the "spokes."
- Color by level `i` (a gradient, e.g. deep blue → cyan → white) passed as a vertex
  attribute; optionally a toggle to color by height `z`.
- Transform with my existing view/projection uniforms; the manifold lives in Lorenz
  coordinates (roughly |x|,|y| ≲ 20, z ≲ 40 once it grows) — add a uniform model
  scale so it frames nicely.

## Controls
Expose `σ, ρ, β, order, N, M, dRho` (constants at first is fine; wire to my param/
hot-reload path if quick). Recompute the manifold and rebuild the VBO on change.

## Numerical requirements
- `double` for all manifold math; `float` only at GL upload.
- Standard values σ=10, ρ=28, β=8/3, order=10, N≈15, M≈128. Pick `dRho` small and
  tune by eye: when rings start crossing or shooting off, you've hit the edge of K's
  convergence disk — that's the cue for Phase 2, not a bug.

## Acceptance criteria (Phase 1)
- Builds under my CMake/WSL2 setup, no new heavy deps (hand-rolled 3×3 solve is fine;
  GLM is fine).
- Renders ~15 concentric rings forming a smooth surface patch through the origin,
  tangent to the plane spanned by ev1=(0,0,1) and ev2≈(−0.78,1,0).
- Rings are well-distributed (after step 5) and the surface is smooth out to the
  convergence limit.
- Parameters adjustable with live recompute.

---

## Phase 2 — extend past the Taylor disk (scope now, implement later; leave stubbed)
To grow beyond K's convergence radius, switch to flow-based geodesic growth.
**Because Wˢ is a stable manifold, integrate the REVERSED field `ẋ = -f(x)`** (RK4,
small fixed step, double precision) so the manifold expands outward.

Per new level, for each `r ∈ M_i` with ring-neighbors `r⁻, r⁺`:
1. tangent `t̂ = normalize(r⁺ − r⁻)`; transversal plane `𝓕_r` = plane through `r`
   with normal `t̂`.
2. Shoot: find `(s, τ)` such that `b = φ⁻ᵗ(γ(s))` (backward flow from a point
   `γ(s)` interpolated along `C_i` near `r`) satisfies
   `t̂·(b − r) = 0`  and  `‖b − r‖ = Δ = η_{i+1} − η_i`.
   2×2 Newton with finite-difference Jacobian, warm-started in τ; small Δ.
3. `b` becomes the new point in `M_{i+1}`.

Remesh between levels: insert a point on any ring edge whose arc-gap exceeds a
tolerance (keeps spacing bounded as the front grows). Levels are sequential; points
within a level are independent (parallelize later if needed).

Keep Phase 2 behind a flag so the Phase 1 build stands alone.
