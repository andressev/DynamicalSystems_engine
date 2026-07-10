# Window

`engine/include/engine/Window.h`, `engine/src/Window.cpp`

Owns the GLFW window and the OpenGL 3.3 core context. Exactly one alive at a time
(calls `glfwInit`/`glfwTerminate` in ctor/dtor). Throws `std::runtime_error` on failure.

## Interface
- `shouldClose` / `requestClose` / `swapBuffers` / `pollEvents`
- `width()` / `height()` / `aspect()` — framebuffer size, kept fresh by the resize callback
  (which also updates the GL viewport).
- `cursorPosNormalized()` — [0,1]², origin bottom-left (used by [[riemann-sphere]]'s
  follow-mouse coefficients).
- `handle()` — raw `GLFWwindow*` for [[Camera]] input and the ImGui backend.
