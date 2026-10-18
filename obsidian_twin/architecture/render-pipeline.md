# Render pipeline

One frame, driven by [[App]]::run (`engine/src/App.cpp`):

```
pollEvents
dt = clock delta
ImGui new frame
camera.updateEase(dt)                          ([[Camera]]: a recenter glide, if any)
if ImGui doesn't want the keyboard:
    ESC -> close, Camera.processInput(dt)     ([[Camera]])
    H -> toggle the controls panel
    1 held 1 s -> sim.recenterCamera()           ([[Sim]], [[FlowKeys]] tapHold)
camera.aspect <- window aspect                 ([[Window]])
sim.update(dt)                                 ([[Sim]])
clear color + depth
sim.render(camera)      -> Shader.use + uniforms, Mesh.draw   ([[Shader]], [[Mesh]])
App UI (FPS, menu button) + sim.drawUI()      (ImGui; skipped while hidden with H)
ImGui render
swapBuffers
```

Key properties:
- Projection comes from one place: `Camera::getProjectionMatrix()` (fov 55°, near 0.1,
  far 100); aspect is refreshed from the framebuffer every frame, so resizing never
  desyncs fov.
- The viewport is updated by [[Window]]'s framebuffer-size callback.
- Sims own *what* is drawn (meshes, shaders, model matrices); App owns *when*.
- Depth testing is enabled once in App's constructor.
