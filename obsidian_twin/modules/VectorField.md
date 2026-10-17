---
type: module
layer: math
status: stable
code:
  - engine/include/engine/VectorField.h
  - engine/src/VectorField.cpp
---
# VectorField

`engine/include/engine/VectorField.h`, `engine/src/VectorField.cpp`

## What it is
A real vector field on $\mathbb{R}^2$ or $\mathbb{R}^3$, built from typed strings: one
[[Expression]] per component over the variables $x, y[, z]$ and $t$. It is the adapter
that turns the function parser into an ODE right-hand side — the thing
[[Integrator]] integrates, [[Trajectory]] follows and the glyph grid samples
([[integral-curves]]). M3 in [[roadmap]].

Two components parse to $\dot{x} = f(x,y,t)$, $\dot{y} = g(x,y,t)$; three add
$\dot{z} = h(x,y,z,t)$. Nothing else about the field is stored — no domain, no grid,
no geometry; those belong to whoever is drawing it.

## Dependencies
- Engine: [[Expression]] (same layer — `math`)
- External: std-only (no glad/glfw/glm — the math-layer purity rule)

## How to use it
Attach = hold a `engine::VectorField` member (default-constructed, `valid() == false`).
1. On Apply, parse the UI strings:
   `field.set(dim, {"y", "-sin(x) - 0.2y"}, error)` — returns false with a
   component-labelled message in `error` ("dy/dt: unknown name 'sni' at character 1").
   **A failed `set` leaves the previous field intact**, so a typo never blanks the view;
   show `error` in red and keep drawing, as [[riemann-sphere]] does.
2. Sample it wherever you need a vector: `field.eval(x, t, out)` with `x` and `out`
   holding `dim()` doubles, or `field.magnitude(x, t)` for $|f|$ alone.
3. Hand it to an [[Integrator]] as a right-hand side by wrapping it once in `init`:
   `rhs = [this](const double* x, double t, double* dxdt) { field.eval(x, t, dxdt); };`
4. Reference sim: [[vector-field]].

Interface:
- `set(dim, components, error)` — `dim` is 2 or 3 and `components.size()` must match.
  Parses over `{"x","y","t"}` or `{"x","y","z","t"}`; all-or-nothing.
- `eval(x, t, out)`, `magnitude(x, t)` — the field itself.
- `latex()` — each component typeset via `Expression::latex` over its own variable
  names, for an on-screen $\dot{\mathbf{x}} = f(\mathbf{x})$ ([[vector-field]]).
- `dim()`, `valid()`, `variables()`, `sources()` — what it was built from.
- `usesTime()` — true iff some component actually mentions $t$ (via
  `Expression::usesVariable`). Sims use it to decide whether a glyph grid has to be
  re-sampled every frame or only when something changes.

## Notes
- **Evaluation is real, not complex.** It goes through `Expression::evalReal`, so
  constants keep only their real part (`i` reads as $0$) and the complex-only functions
  collapse to their real meaning (`conj`/`re` identity, `im` $=0$, `arg` $\in \{0,\pi\}$).
  A field typed with `i` in it is a user error the parser cannot catch.
- **`eval` is thread-safe**: its value array is `thread_local` (one per thread,
  reused, no per-call allocation), so one field can be sampled from many threads at once
  — [[vector-field]] evaluates its glyph grid with `std::execution::par`. Only `parse`
  must not run concurrently with evaluation.
- Values are `double` all the way through; only the glyph/trail geometry narrows to
  `float` at the GPU boundary.
