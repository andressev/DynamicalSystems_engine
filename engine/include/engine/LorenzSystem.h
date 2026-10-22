#ifndef ENGINE_LORENZ_SYSTEM_H
#define ENGINE_LORENZ_SYSTEM_H

#include <engine/InvariantManifold.h>

#include <array>
#include <string>
#include <vector>

namespace engine {

// The Lorenz system x' = sigma(y - x), y' = x(rho - z) - y, z' = xy - beta z, with
// everything the Hartman-Grobman sims need about it in one place: the field, its
// equilibria, the Jacobian there and the (point-independent) quadratic part. Pure std
// (math layer); a plain value type, so copying it is copying three doubles.
struct LorenzSystem {
    using Vec3 = std::array<double, 3>;

    double sigma = 10.0;
    double rho = 28.0;
    double beta = 8.0 / 3.0;

    struct Equilibrium {
        Vec3 x{};
        std::string name;   // "O", "C+", "C-"
    };

    // dxdt = f(x); thread-safe (no state).
    void eval(const double* x, double* dxdt) const;

    // Taylor coefficients of the solution through x, c[3k + i] for k = 0..order (c_0 =
    // x), for TaylorIntegrator: the field is quadratic, so (k+1) c_{k+1} = f's terms with
    // each product xz, xy replaced by its Cauchy product sum_j a_j b_{k-j} — exact, O(order^2).
    void jet(const double* x, int order, double* c) const;

    // The origin always; C+- = (+-sqrt(beta(rho-1)), +-sqrt(beta(rho-1)), rho-1) when
    // rho > 1 and beta > 0 (they are born from the origin in a pitchfork at rho = 1).
    std::vector<Equilibrium> equilibria() const;

    // Df(x) = [[-sigma, sigma, 0], [rho - z, -1, -x], [y, x, -beta]].
    void jacobian(const Vec3& x, double J[3][3]) const;

    // f(v* + u) = Df(v*) u + Q(u, u), the same Q at every equilibrium (the field is
    // quadratic): y' gets -u_x u_z, z' gets u_x u_y. In InvariantManifold's Q[c][i][j].
    static InvariantManifold::Quadratic quadratic();
};

} // namespace engine

#endif
