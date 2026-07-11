---
type: module
layer: gfx
status: stable
code:
  - engine/include/engine/Shader.h
  - engine/src/Shader.cpp
---
# Shader

`engine/include/engine/Shader.h`, `engine/src/Shader.cpp`

## What it is
Compiled + linked GLSL program wrapper. Non-copyable, movable, `glDeleteProgram` in
destructor — one of the two GL resource owners (with [[Mesh]]); sims never call raw GL
resource functions.

## Dependencies
- Engine: none
- External: glad, glm

## How to use it
Attach = hold a plain `Shader` member, move-assign in `init`:
1. Member: `engine::Shader shader;` (default-constructed = invalid, `ID == 0`).
2. In `init`: `shader = Shader(vertexPath, fragmentPath);` — paths via
   `SIMS_DIR "<name>/shaders/file.glsl"`.
3. In `render`: `shader.use();` then `setBool/Int/Float/Vec2/Vec3/Vec4/Mat4(name, ...)`.
4. Reference sim: [[riemann-sphere]].

For runtime-generated GLSL: `Shader::fromSources(vertexSrc, fragmentSrc)` compiles from
in-memory strings — this is the [[Expression]] codegen path: generate GLSL, splice into
a template, hot-recompile, move-assign over the old shader.

## Notes
- Compile/link errors are printed to stdout (`ERROR::SHADER...`), not thrown — on a
  failed hot-recompile keep the previous shader.
