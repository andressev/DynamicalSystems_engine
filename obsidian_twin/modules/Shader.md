# Shader

`engine/include/engine/Shader.h`, `engine/src/Shader.cpp`

Compiled + linked GLSL program. Non-copyable, movable, `glDeleteProgram` in destructor.
Compile/link errors are printed to stdout (`ERROR::SHADER...`), not thrown.

## Interface
- `Shader(vertexPath, fragmentPath)` — load from files.
- `Shader::fromSources(vertexSrc, fragmentSrc)` — compile from in-memory strings.
  **This is the hook for the M2 function-parser codegen path** (see [[roadmap]]):
  generate GLSL, splice into a template, hot-recompile.
- `use()`, `setBool/Int/Float/Vec2/Vec3/Vec4/Mat4(name, ...)`.
- Default-constructed Shader has `ID == 0` (invalid); sims typically hold
  `std::unique_ptr<Shader>` created in `init`.
