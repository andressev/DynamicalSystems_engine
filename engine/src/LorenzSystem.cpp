#include <engine/LorenzSystem.h>

#include <cmath>

namespace engine {

void LorenzSystem::eval(const double* x, double* dxdt) const
{
    dxdt[0] = sigma * (x[1] - x[0]);
    dxdt[1] = x[0] * (rho - x[2]) - x[1];
    dxdt[2] = x[0] * x[1] - beta * x[2];
}

void LorenzSystem::jet(const double* x, int order, double* c) const
{
    c[0] = x[0];
    c[1] = x[1];
    c[2] = x[2];
    for (int k = 0; k < order; ++k)
    {
        double xz = 0.0, xy = 0.0;   // k-th coefficients of x z and x y
        for (int j = 0; j <= k; ++j)
        {
            xz += c[3 * j] * c[3 * (k - j) + 2];
            xy += c[3 * j] * c[3 * (k - j) + 1];
        }
        const double* a = &c[3 * k];
        double* next = &c[3 * (k + 1)];
        const double inv = 1.0 / (double)(k + 1);
        next[0] = sigma * (a[1] - a[0]) * inv;
        next[1] = (rho * a[0] - xz - a[1]) * inv;
        next[2] = (xy - beta * a[2]) * inv;
    }
}

std::vector<LorenzSystem::Equilibrium> LorenzSystem::equilibria() const
{
    std::vector<Equilibrium> out;
    out.push_back({{0.0, 0.0, 0.0}, "O"});
    if (rho > 1.0 && beta > 0.0)
    {
        double c = std::sqrt(beta * (rho - 1.0));
        out.push_back({{c, c, rho - 1.0}, "C+"});
        out.push_back({{-c, -c, rho - 1.0}, "C-"});
    }
    return out;
}

void LorenzSystem::jacobian(const Vec3& x, double J[3][3]) const
{
    J[0][0] = -sigma;     J[0][1] = sigma; J[0][2] = 0.0;
    J[1][0] = rho - x[2]; J[1][1] = -1.0;  J[1][2] = -x[0];
    J[2][0] = x[1];       J[2][1] = x[0];  J[2][2] = -beta;
}

InvariantManifold::Quadratic LorenzSystem::quadratic()
{
    InvariantManifold::Quadratic Q{};
    Q[1][0][2] = -1.0;   // y' gets -u_x u_z
    Q[2][0][1] = 1.0;    // z' gets  u_x u_y
    return Q;
}

} // namespace engine
