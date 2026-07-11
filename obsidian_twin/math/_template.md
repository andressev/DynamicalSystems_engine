---
type: math
status: stable | wip
code:
  - path/to/where/it/is/implemented
---
# <concept-name>

## The idea
The concept in plain words — what it does and why this project needs it.

## The equations
The key formulas, with symbols named. As deep as needed; this note is where
derivations live so sim notes don't have to re-explain.
Write math in Obsidian LaTeX — `$x$` inline, `$$x$$` for display equations — never in
code blocks (backticks are for actual code/GLSL identifiers only).

## Where it's implemented
File + function/shader for each place the math touches code. Multiple sims may link
here — that's the point: one concept note serves every sim that uses the concept.

## Pitfalls
Numerical traps, singularities, edge cases — and how the code dodges them.
