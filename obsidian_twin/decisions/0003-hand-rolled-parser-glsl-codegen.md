# 0003 — Hand-rolled function parser with GLSL codegen

**Date:** 2026-07-10 · **Status:** accepted, implementation is M2 (see [[roadmap]])

## Decision
Write our own tokenizer + recursive-descent parser producing an AST, then generate GLSL
(`vec2 f(vec2 z) { ... }` over the complex helpers already in
`sims/riemann_sphere/shaders/complexFragment.glsl`), splice into a fragment template and
hot-recompile via `Shader::fromSources` ([[Shader]] — already implemented).

## Why over a library (exprtk/muparser)
- Libraries evaluate on CPU only; we need GLSL (and later CUDA) codegen anyway, which is
  the hard part they don't cover.
- Fits the learning goals of the project; zero dependencies; exprtk's single header is a
  compile-time hog.
- The same AST can be evaluated on CPU for the M3 RK4 integrator and codegen'd to CUDA
  in M4.
