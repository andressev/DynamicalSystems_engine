---
type: module
layer: core
status: stable
code:
  - engine/include/engine/Window.h
  - engine/src/Window.cpp
---
# Window

`engine/include/engine/Window.h`, `engine/src/Window.cpp`

## What it is
Owns the GLFW window and the OpenGL 3.3 core context. Exactly one alive at a time
(calls `glfwInit`/`glfwTerminate` in ctor/dtor). Core layer — [[App]] owns it; sims
reach it through `app.window()`.

## Dependencies
- Engine: none
- External: glad, glfw, glm

## How to use it
Sims never construct one; query it via `app.window()`:
- `shouldClose` / `requestClose` / `swapBuffers` / `pollEvents` (App loop's job)
- `width()` / `height()` / `aspect()` — framebuffer size, kept fresh by the resize
  callback (which also updates the GL viewport).
- `cursorPosNormalized()` — [0,1]², origin bottom-left (used by [[riemann-sphere]]'s
  follow-mouse coefficients).
- `keyDown(int key)` — true while held; takes uppercase ASCII (GLFW letter/digit key
  codes match ASCII). Lets sims poll keys without touching GLFW directly.
- `mouseDown(int button = 0)` — true while the mouse button is held; GLFW numbering
  (0 left, 1 right, 2 middle). Sims edge-detect clicks themselves (used by
  [[surface-renderer-ray-marched]]'s click-to-place particle).
- `handle()` — raw `GLFWwindow*` for [[Camera]] input and the ImGui backend.

## Notes
- Throws `std::runtime_error` if GLFW/glad initialization fails.
