---
type: module
layer: math
status: stable
code:
  - engine/include/engine/Eigen3.h
  - engine/src/Eigen3.cpp
  - tests/math_tests.cpp
---
# Eigen3

`engine/include/engine/Eigen3.h`, `engine/src/Eigen3.cpp`

## What it is
Eigenvalues and eigenvectors of a real $3\times 3$ matrix in double precision — the
linearization $A = Df(x^*)$ at an equilibrium, from which $E^s$, $E^u$ and
hyperbolicity are read. Hand-rolled instead of vendoring the Eigen library, so the
math layer stays std-only and CUDA-portable; $3\times3$ is small enough to do exactly.
M1 of the Hartman–Grobman track in [[roadmap]].

## Dependencies
- Engine: none
- External: std-only (`<complex>`, `<array>`, `<vector>`)

## How to use it
1. `engine::Eigen3 e(A);` (or `e.compute(A)` again when parameters move — it is cheap).
2. `e.values()` — sorted by real part ascending; a complex pair is adjacent, $+\mathrm{Im}$ first.
   `e.vectors()[i]` — the matching eigenvector, unit Hermitian norm, largest component
   real positive (so real eigenvalues give real vectors).
3. Classification: `hyperbolic(tol)`, `dimStable(tol)`, `dimUnstable(tol)`,
   `isComplex(i)`; `realBasis(sign)` is a real basis of $E^s$ (`sign < 0`) or $E^u$ —
   $\{\mathrm{Re}\,v, \mathrm{Im}\,v\}$ for a complex pair.
4. `independent()` — false for defective / degenerate $A$; don't trust subspaces then.
5. Used by [[InvariantManifold]]; checked in `tests/math_tests.cpp`.

## Notes
- Method: characteristic cubic → one real root (Cardano / trigonometric form) → deflate
  to a quadratic → Newton-polish every root on the cubic. Eigenvectors are a
  *bilinear* cross product of two rows of $A - \lambda I$ (no conjugation — a
  complex null vector needs $r \cdot v = 0$, not $\bar r \cdot v = 0$).
- A conjugate pair is stored as exact conjugates (value and vector), so
  `vals[i] == conj(vals[i-1])` is a safe exact test and real bases built from it are
  exactly real.
- Defective matrices (repeated eigenvalue, 1-D eigenspace) are not resolved — they
  come back with duplicated vectors and `independent() == false`.
- Residual $|Av - \lambda v|$ on 2000 random matrices: ~$10^{-15}$.
