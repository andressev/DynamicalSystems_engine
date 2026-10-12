#ifndef ENGINE_VECTORFIELD_H
#define ENGINE_VECTORFIELD_H

#include <engine/Expression.h>

#include <string>
#include <vector>

namespace engine {

// A real vector field on R^2 or R^3, built from typed strings: one Expression per
// component, over the variables x, y[, z] and t. This is the bridge between the
// function parser and everything that flows — glyph grids, Integrator right-hand
// sides, trajectories — and the reason Expression grew a real-valued eval.
// Pure std C++ (math layer): no GL, no glm.
class VectorField {
public:
    static constexpr int kMaxDim = 3;

    // Parse `components` (one per dimension, dim = 2 or 3) over {"x","y","t"} or
    // {"x","y","z","t"}. On the first bad component this returns false and fills
    // `error`, leaving the previously parsed field untouched — a typo in the UI
    // never blanks the screen.
    bool set(int dim, const std::vector<std::string>& components, std::string& error);

    bool valid() const { return d > 0; }
    int dim() const { return d; }

    // True if any component actually mentions t; a sim uses this to decide whether
    // its glyph grid has to be re-sampled every frame.
    bool usesTime() const { return timeDependent; }

    // Variable names this field was parsed over, in order, t last.
    const std::vector<std::string>& variables() const { return vars; }
    // The strings the components were parsed from (for UI echo / saving).
    const std::vector<std::string>& sources() const { return srcs; }

    // out[0..dim-1] = f(x, t); `x` must hold dim values. Silent no-op while invalid.
    void eval(const double* x, double t, double* out) const;

    // |f(x, t)|
    double magnitude(const double* x, double t) const;

private:
    int d = 0;
    bool timeDependent = false;
    std::vector<Expression> comps;
    std::vector<std::string> vars;
    std::vector<std::string> srcs;
    // eval() is const but needs a scratch value array; the engine is single
    // threaded, so one buffer per field is enough.
    mutable std::vector<double> scratch;
};

} // namespace engine

#endif
