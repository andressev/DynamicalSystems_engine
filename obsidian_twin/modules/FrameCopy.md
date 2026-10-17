---
type: module
layer: gfx
status: wip
code:
  - engine/include/engine/FrameCopy.h
  - engine/src/FrameCopy.cpp
---
# FrameCopy

`engine/include/engine/FrameCopy.h`, `engine/src/FrameCopy.cpp`

## What it is
A texture holding a copy of what has been drawn so far this frame, for effects that
bend the scene behind them — the crash shockwave on the glass balls of
[[lorenz-equilibrium]] now, refraction through the glass later (M7 in [[roadmap]]).
`capture()` blits the current viewport of the framebuffer being drawn into (the window,
or [[App]]'s offscreen screenshot target) into its own texture; the blit also resolves
the window's 4x MSAA, which `glCopyTexImage2D` cannot.

## Dependencies
- Engine: none
- External: glad (owns one texture + one framebuffer)

## How to use it
1. Member: `engine::FrameCopy sceneCopy;` (construct anywhere; GL objects are made on
   the first capture).
2. `render`: draw what should be bent, then `sceneCopy.capture()`, then
   `sceneCopy.bind(unit)` and draw the effect with a `sampler2D` on that unit, sampling
   at `(gl_FragCoord.xy - viewport.xy) / viewport.zw` — pass `viewportX/Y`, `width`,
   `height` as a `vec4` uniform.
3. Capture only on frames that use it (a full-screen blit is cheap, not free).
4. Reference: [[lorenz-equilibrium]] (`renderHgBalls`, `glass_fragment.glsl`).

## Notes
- Resized to the viewport on demand, so it follows window resizes and
  `App::captureScreenshot`'s larger offscreen target.
- Restores the read/draw framebuffer bindings it found.
