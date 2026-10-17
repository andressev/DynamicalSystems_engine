#include <engine/VectorField.h>

#include <cmath>

namespace engine {

bool VectorField::set(int dim, const std::vector<std::string>& components,
                      std::string& error)
{
    if (dim < 2 || dim > kMaxDim || (int)components.size() != dim)
    {
        error = "vector field needs exactly " + std::to_string(dim) + " components";
        return false;
    }

    std::vector<std::string> names;
    names.push_back("x");
    names.push_back("y");
    if (dim == 3)
        names.push_back("z");
    names.push_back("t");

    // parse everything into locals first: a failure must leave the live field alone
    std::vector<Expression> parsed;
    for (int i = 0; i < dim; ++i)
    {
        Expression e = Expression::parse(components[i], names, error);
        if (!e.valid())
        {
            const char* label[] = {"x", "y", "z"};
            error = std::string("d") + label[i] + "/dt: " + error;
            return false;
        }
        parsed.push_back(std::move(e));
    }

    d = dim;
    comps = std::move(parsed);
    vars = std::move(names);
    srcs = components;

    timeDependent = false;
    for (const Expression& e : comps)
        timeDependent = timeDependent || e.usesVariable(d); // t is the last variable

    return true;
}

void VectorField::eval(const double* x, double t, double* out) const
{
    if (d <= 0)
        return;

    // one value array per thread, so eval() may run on many threads at once (the
    // glyph grid samples in parallel); reused across calls, so no allocation per call
    thread_local std::vector<double> scratch;
    scratch.resize(d + 1);
    for (int i = 0; i < d; ++i)
        scratch[i] = x[i];
    scratch[d] = t;

    for (int i = 0; i < d; ++i)
        out[i] = comps[i].evalReal(scratch);
}

double VectorField::magnitude(const double* x, double t) const
{
    double v[kMaxDim] = {0.0, 0.0, 0.0};
    eval(x, t, v);
    double sum = 0.0;
    for (int i = 0; i < d; ++i)
        sum += v[i] * v[i];
    return std::sqrt(sum);
}

std::vector<std::string> VectorField::latex() const
{
    std::vector<std::string> out;
    for (const Expression& e : comps)
        out.push_back(e.latex(vars));
    return out;
}

} // namespace engine
