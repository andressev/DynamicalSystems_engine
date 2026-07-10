# 0002 — Dear ImGui as the UI layer

**Date:** 2026-07-10 · **Status:** accepted

## Decision
Vendor Dear ImGui (core + glfw/opengl3 backends, `External/imgui/`, master @ da137cb)
and build it into the engine. [[App]] owns init/frame/render/shutdown; sims add widgets
via `Sim::drawUI()`.

## Why
De-facto standard for graphics research tools: text input (future function box), sliders,
combos, FPS overlay — with two backend files and no build system of its own. The
alternative (console/file input) would have been retrofitted with ImGui anyway.
