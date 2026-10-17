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
6. For CPU-side use of the same input, call `eval(values)` (complex) or `evalReal(values)`
   (real) with the values in the same order as the parse-time variable list.

**Steps 1, 3-5 are the shader path only.** For pure CPU math — an [[Integrator]] right-hand
side, a sampler — there is no codegen step at all: parse, then call `evalReal` per sample.
[[VectorField]] is that path packaged; [[0006-cpu-field-sampling-dynamic-glyph-mesh]] says
why the vector-field track evaluates on the CPU instead of generating a shader.

Interface:
- `Expression::parse(source, variables, error)` — variables are caller-defined names
  (e.g. `{"z","t","m"}`), so each sim picks its own. Invalid result + position-annotated
  `error` on failure; never throws.
- `eval(values)` — `std::complex<double>`, values matching the parse-time variable list.
- `evalReal(values)` — the same AST walked with `double`s, for real right-hand sides an
  [[Integrator]] hits thousands of times a frame ([[VectorField]] is the user). Constants
  keep only their real part, so `i` reads as $0$, and the complex-only functions collapse
  to their real meaning (`conj`/`re` identity, `im` $= 0$, `arg` $\in \{0, \pi\}$) —
  a real field typed with `i` in it is a user error the parser cannot catch.
- `usesVariable(index)` — does `variables[index]` actually appear? Lets a sim skip work
  that only a typed variable would justify (the per-frame glyph rebuild in
  [[vector-field]] happens only when the field really mentions $t$).
- `glsl(glslVars)` — emits a `vec2` expression; `glslVars[i]` is the GLSL expression
  standing in for `variables[i]` (e.g. `{"z", "vec2(u_time, 0.0)", "u_mouse"}`). This
  mapping is how one parser serves many sims/uniform layouts.
- `glslComplexLib()` — canonical `cMul/cDiv/cExp/.../cPowi` helper source; sims splice
  it into their fragment template, then compile via `Shader::fromSources` ([[Shader]]).
- `latex(texVars)` — the AST typeset as LaTeX math for [[LatexImage]] (`texVars[i]`
  stands for `variables[i]`): `/` → `\frac`, `^` → superscript, products juxtapose
  (`2x`, `x\,y`, `\cdot` before a number), calls → `\sin(…)`, `e^{…}`, `\sqrt{…}`,
  `\left|…\right|`; constants equal to $\pi$, $\tau$, $e$ print as symbols; signs are
  `{-}` so a leading minus hugs its operand; parentheses only where precedence needs
  them (`\left(…\right)` only around fractions). [[vector-field]]'s overlay uses it.

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
  real test target when a test setup exists. `evalReal`/`usesVariable` were verified the
  same way (agreement with `eval` on the real line, `x^2` finite at 0, `t` detection),
  and `latex` against a table of ~30 inputs covering every operator and precedence case.
- The real and complex walkers share the AST, the integer-power rule and the function
  table (one row per function carries both evaluators plus the GLSL pattern) — adding a
  function means filling all three, or the two paths drift apart.
