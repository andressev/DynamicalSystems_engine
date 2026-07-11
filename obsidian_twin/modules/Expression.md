---
type: module
layer: math
status: stable
code:
  - engine/include/engine/Expression.h
  - engine/src/Expression.cpp
---
# Expression

`engine/include/engine/Expression.h`, `engine/src/Expression.cpp`

## What it is
A function parser that takes in a string at runtime and converts it into a function and
also GLSL code. The M2 function parser ([[roadmap]],
[[0003-hand-rolled-parser-glsl-codegen]]).
Tokenizer → recursive-descent parser → AST; one parse serves both CPU evaluation and
GLSL codegen, so typed UI input drives shaders and C++ alike.

## Dependencies
- Engine: none
- External: std-only (pure C++, zero deps — the math-layer purity rule)

## How to use it
Reference implementation: `applyFunction()` in `sims/riemann_sphere/RiemannSphereSim.cpp`.

1. **Write a fragment template** in your sim's `shaders/` folder containing everything
   *except* the function: your uniforms, the placeholder lines `//__COMPLEX_LIB__` and
   `//__F__`, and a `main()` that calls `f(...)`. Load it (and your vertex shader) into
   strings in `init`.
2. **Pick your variable names** — whatever your sim's math speaks:
   `parse(src, {"z","t","m"}, err)` or `{"x","y","t"}`. On failure, show `err` in ImGui
   and **keep the previous shader**; never swap in a broken one.
3. **Map each variable to a GLSL expression** when generating code:
   `expr.glsl({"z", "vec2(u_time, 0.0)", "u_mouse"})` — every variable must become a
   `vec2` (complex) expression available inside your shader. Real uniforms are wrapped
   as `vec2(u_x, 0.0)`.
4. **Splice and compile**: wrap the result in `vec2 f(vec2 z) { return ...; }`, replace
   the two placeholders (lib first, then f), and hot-swap with
   `shader = Shader::fromSources(vertexSrc, frag)` (plain `Shader` member,
   move-assign — see [[Shader]]).
5. **Feed the uniforms each frame** in `render()` (`setFloat("u_time", ...)` etc.).
   Structure changes at Apply-time (recompile); values change per frame (uniforms).
6. For CPU-side use of the same input, call `eval(values)` with the values in the same
   order as the parse-time variable list.

Interface:
- `Expression::parse(source, variables, error)` — variables are caller-defined names
  (e.g. `{"z","t","m"}`), so each sim picks its own. Invalid result + position-annotated
  `error` on failure; never throws.
- `eval(values)` — `std::complex<double>`, values matching the parse-time variable list.
  Complex-only for now; M3 adds a fast pure-real eval for real ODE functions
  (up to ~3 variables).
- `glsl(glslVars)` — emits a `vec2` expression; `glslVars[i]` is the GLSL expression
  standing in for `variables[i]` (e.g. `{"z", "vec2(u_time, 0.0)", "u_mouse"}`). This
  mapping is how one parser serves many sims/uniform layouts.
- `glslComplexLib()` — canonical `cMul/cDiv/cExp/.../cPowi` helper source; sims splice
  it into their fragment template, then compile via `Shader::fromSources` ([[Shader]]).

## Language
`+ - * / ^` (right-assoc `^`), unary minus, parens, implicit multiplication
(`2z`, `(z-1)(z+1)` — but identifiers are maximal-munch, so `iz` is one unknown name);
functions `sin cos tan sinh cosh tanh exp log ln sqrt conj abs arg re im`; constants
`i pi e tau`; scientific notation (`2e3`).

## Integer-power rule
`^` with a small integral real constant exponent (|n| ≤ 32) compiles to repeated
multiplication (`cPowi` in GLSL, a loop on CPU) instead of exp/log-based `pow`, so
`z^2` is exact and finite at `z = 0` — pow would NaN there and ruin domain coloring.

## Notes
- No test target in the build; `parse/eval/glsl` were verified with a standalone
  harness compiling `Expression.cpp` alone (it has zero deps) — worth turning into a
  real test target when a test setup exists.
