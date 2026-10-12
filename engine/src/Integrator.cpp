#include <engine/Integrator.h>

#include <cmath>

namespace engine {

void Integrator::step(const Field& f, int dim, double* x, double& t, double h) const
{
    if (dim <= 0 || dim > kMaxDim)
        return;

    double k1[kMaxDim], k2[kMaxDim], k3[kMaxDim], k4[kMaxDim], tmp[kMaxDim];

    switch (method)
    {
    case Method::Euler:
        f(x, t, k1);
        for (int i = 0; i < dim; ++i)
            x[i] += h * k1[i];
        break;

    case Method::RK2: // midpoint
        f(x, t, k1);
        for (int i = 0; i < dim; ++i)
            tmp[i] = x[i] + 0.5 * h * k1[i];
        f(tmp, t + 0.5 * h, k2);
        for (int i = 0; i < dim; ++i)
            x[i] += h * k2[i];
        break;

    case Method::RK4: // classic four-stage
        f(x, t, k1);
        for (int i = 0; i < dim; ++i)
            tmp[i] = x[i] + 0.5 * h * k1[i];
        f(tmp, t + 0.5 * h, k2);
        for (int i = 0; i < dim; ++i)
            tmp[i] = x[i] + 0.5 * h * k2[i];
        f(tmp, t + 0.5 * h, k3);
        for (int i = 0; i < dim; ++i)
            tmp[i] = x[i] + h * k3[i];
        f(tmp, t + h, k4);
        for (int i = 0; i < dim; ++i)
            x[i] += (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
        break;
    }

    t += h;
}

int Integrator::advance(const Field& f, int dim, double* x, double& t, double dt,
                        int maxSubsteps) const
{
    if (dt == 0.0 || dim <= 0 || dim > kMaxDim)
        return 0;

    double cap = maxStep > 1e-9 ? maxStep : 1e-9;
    int n = (int)std::ceil(std::abs(dt) / cap);
    if (n < 1)
        n = 1;
    if (n > maxSubsteps)
        n = maxSubsteps;

    double h = dt / n;
    for (int k = 0; k < n; ++k)
        step(f, dim, x, t, h);
    return n;
}

const char* Integrator::methodName(Method m)
{
    switch (m)
    {
    case Method::Euler: return "Euler";
    case Method::RK2:   return "RK2 (midpoint)";
    case Method::RK4:   return "RK4";
    }
    return "?";
}

} // namespace engine
