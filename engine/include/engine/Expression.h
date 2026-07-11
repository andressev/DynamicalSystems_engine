#ifndef ENGINE_EXPRESSION_H
#define ENGINE_EXPRESSION_H

#include <complex>
#include <memory>
#include <string>
#include <vector>

namespace engine {

// A parsed math expression over caller-defined variables, e.g. "z^2 + sin(t*z)".
// One parse produces an AST that can be evaluated on the CPU (std::complex) and
// compiled to a GLSL vec2 expression, so the same typed input drives shaders and
// C++ integrators alike. Pure std C++ — no GL/glm dependencies.
class Expression {
public:
    Expression();  // invalid expression; valid() == false
    ~Expression();
    Expression(Expression&&) noexcept;
    Expression& operator=(Expression&&) noexcept;
    Expression(const Expression&) = delete;
    Expression& operator=(const Expression&) = delete;

    // Parse `source` over the given variable names (e.g. {"z","t","m"}).
    // On failure returns an invalid expression and fills `error` with a
    // position-annotated message for display in the UI.
    static Expression parse(const std::string& source,
                            const std::vector<std::string>& variables,
                            std::string& error);

    bool valid() const { return root != nullptr; }

    // values[i] is the value of variables[i] passed to parse().
    std::complex<double> eval(const std::vector<std::complex<double>>& values) const;

    // Emit a GLSL vec2 expression; glslVars[i] is the GLSL expression standing in
    // for variables[i] (e.g. {"z", "u_time", "u_mouse"}). The generated code calls
    // the helpers from glslComplexLib(), which must be present in the shader.
    std::string glsl(const std::vector<std::string>& glslVars) const;

    // Source of the complex-arithmetic helper functions (cMul, cExp, cPowi, ...)
    // that glsl() output depends on; sims splice this into their shader templates.
    static const char* glslComplexLib();

    struct Node;  // AST node; defined only in the .cpp (opaque to users)

private:
    std::unique_ptr<Node> root;
};

} // namespace engine

#endif
