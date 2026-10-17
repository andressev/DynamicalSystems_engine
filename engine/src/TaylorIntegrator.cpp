#include <engine/TaylorIntegrator.h>

#include <algorithm>
#include <cmath>

namespace engine {

double TaylorIntegrator::stepSize(const double* c, int dim) const
{
    const int n = std::clamp(order, 2, kMaxOrder);
    double scale = 1.0;
    for (int i = 0; i < dim; ++i)
        scale = std::max(scale, std::abs(c[i]));
    const double eps = std::max(tolerance, 1e-18) * scale;

    // Jorba-Zou: the radius the last two terms allow, so each would be ~eps at h
    double h = 1e30;
    for (int k : {n - 1, n})
    {
        double norm = 0.0;
        for (int i = 0; i < dim; ++i)
            norm = std::max(norm, std::abs(c[k * dim + i]));
        if (norm > 0.0)
            h = std::min(h, std::pow(eps / norm, 1.0 / k));
    }
    if (h >= 1e30)
        h = 1.0;   // the polynomial is exact (all high terms 0): any step will do
    return 0.9 * h;
}

void TaylorIntegrator::evaluate(const double* c, int dim, int order, double s, double* x,
                                double* dxds)
{
    for (int i = 0; i < dim; ++i)
    {
        // Horner for x and its derivative together
        double v = c[order * dim + i];
        double dv = 0.0;
        for (int k = order - 1; k >= 0; --k)
        {
            dv = dv * s + v;
            v = v * s + c[k * dim + i];
        }
        x[i] = v;
        if (dxds)
            dxds[i] = dv;
    }
}

int TaylorIntegrator::advance(const Jet& jet, int dim, double* x, double& t, double dt,
                              double sampleStep, const Sample& sample) const
{
    if (dt == 0.0 || dim <= 0 || dim > kMaxDim)
        return 0;
    const int n = std::clamp(order, 2, kMaxOrder);
    double c[(kMaxOrder + 1) * kMaxDim];
    double p[kMaxDim], dp[kMaxDim];

    const double dir = dt > 0.0 ? 1.0 : -1.0;
    const double tEnd = t + dt;
    int steps = 0;
    while (dir * (tEnd - t) > 0.0 && steps < maxSteps)
    {
        jet(x, t, n, c);
        double h = std::min(stepSize(c, dim), dir * (tEnd - t));
        const bool last = h >= dir * (tEnd - t);
        h *= dir;

        // dense output inside the step, then the step end itself
        if (sample && sampleStep > 0.0)
        {
            int m = (int)std::ceil(std::abs(h) / sampleStep);
            for (int j = 1; j < m; ++j)
            {
                double s = h * (double)j / (double)m;
                evaluate(c, dim, n, s, p, dp);
                sample(p, dp, t + s);
            }
        }
        evaluate(c, dim, n, h, p, dp);
        for (int i = 0; i < dim; ++i)
            x[i] = p[i];
        t = last ? tEnd : t + h;
        ++steps;
        if (sample)
            sample(x, dp, t);
    }
    return steps;
}

} // namespace engine
