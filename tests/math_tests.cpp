// Checks for the std-only math modules (no GL, no window): Eigen3 and InvariantManifold
// on the Lorenz system. Run: out/build/x64-debug/tests/math_tests.exe  (exit 1 on failure)

#include <engine/Eigen3.h>
#include <engine/InvariantManifold.h>
#include <engine/LorenzSystem.h>
#include <engine/TaylorIntegrator.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

namespace {

using engine::Eigen3;
using engine::InvariantManifold;
using Vec3 = std::array<double, 3>;
using cd = std::complex<double>;

int failures = 0;

void check(bool ok, const char* what, double value, double limit)
{
    std::printf("  [%s] %-58s %.3e (limit %.1e)\n", ok ? " ok " : "FAIL", what, value, limit);
    if (!ok)
        ++failures;
}

void expectBelow(const char* what, double value, double limit)
{
    check(std::isfinite(value) && value < limit, what, value, limit);
}

// ---- Lorenz -----------------------------------------------------------------------

struct Lorenz {
    double sigma = 10.0, rho = 28.0, beta = 8.0 / 3.0;

    Vec3 f(const Vec3& x) const
    {
        return {sigma * (x[1] - x[0]), x[0] * (rho - x[2]) - x[1], x[0] * x[1] - beta * x[2]};
    }
    void jacobian(const Vec3& x, double A[3][3]) const
    {
        double J[3][3] = {{-sigma, sigma, 0.0}, {rho - x[2], -1.0, -x[0]}, {x[1], x[0], -beta}};
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                A[i][j] = J[i][j];
    }
    // f(x* + u) = A u + Q(u, u): the quadratic part is the same at every equilibrium
    static InvariantManifold::Quadratic quadratic()
    {
        InvariantManifold::Quadratic Q{};
        Q[1][0][2] = -1.0;   // y' gets -u_x u_z
        Q[2][0][1] = 1.0;    // z' gets  u_x u_y
        return Q;
    }
    Vec3 cPlus() const
    {
        double c = std::sqrt(beta * (rho - 1.0));
        return {c, c, rho - 1.0};
    }
};

Vec3 rk4(const Lorenz& L, Vec3 x, double T, double h)
{
    int steps = (int)std::ceil(std::abs(T) / h);
    double dt = T / steps;
    auto add = [](const Vec3& a, const Vec3& b, double s) {
        return Vec3{a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]};
    };
    for (int i = 0; i < steps; ++i)
    {
        Vec3 k1 = L.f(x), k2 = L.f(add(x, k1, dt / 2)), k3 = L.f(add(x, k2, dt / 2)),
             k4 = L.f(add(x, k3, dt));
        for (int c = 0; c < 3; ++c)
            x[c] += dt / 6.0 * (k1[c] + 2 * k2[c] + 2 * k3[c] + k4[c]);
    }
    return x;
}

double dist(const Vec3& a, const Vec3& b)
{
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1])
                     + (a[2] - b[2]) * (a[2] - b[2]));
}

double eigResidual(const double A[3][3], const Eigen3& e)
{
    double worst = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        double r = 0.0;
        for (int row = 0; row < 3; ++row)
        {
            cd s = -e.values()[i] * e.vectors()[i][row];
            for (int c = 0; c < 3; ++c)
                s += A[row][c] * e.vectors()[i][c];
            r += std::norm(s);
        }
        worst = std::max(worst, std::sqrt(r) / std::max(1.0, std::abs(e.values()[i])));
    }
    return worst;
}

// ---- tests ------------------------------------------------------------------------

void testEigen()
{
    std::printf("Eigen3\n");
    Lorenz L;
    double A[3][3];

    L.jacobian({0, 0, 0}, A);
    Eigen3 e(A);
    expectBelow("origin: |A v - l v|", eigResidual(A, e), 1e-12);
    double root = std::sqrt((L.sigma + 1) * (L.sigma + 1) + 4 * L.sigma * (L.rho - 1));
    double l2 = (-(L.sigma + 1) - root) / 2, l3 = (-(L.sigma + 1) + root) / 2;
    expectBelow("origin: eigenvalues vs closed form",
                std::abs(e.values()[0] - l2) + std::abs(e.values()[1] + L.beta)
                    + std::abs(e.values()[2] - l3),
                1e-12);
    check(e.dimStable() == 2 && e.dimUnstable() == 1, "origin: dim E^s = 2, dim E^u = 1",
          e.dimStable() * 10 + e.dimUnstable(), 21);

    L.jacobian(L.cPlus(), A);
    e.compute(A);
    expectBelow("C+: |A v - l v|", eigResidual(A, e), 1e-12);
    check(e.isComplex(1) && e.isComplex(2) && e.values()[1] == std::conj(e.values()[2]),
          "C+: exact conjugate pair", e.values()[1].imag(), 0);
    std::printf("      C+ eigenvalues: %.6f, %.6f +- %.6fi\n", e.values()[0].real(),
                e.values()[1].real(), e.values()[1].imag());
    auto basis = e.realBasis(+1);
    check(basis.size() == 2, "C+: E^u real basis has 2 vectors", (double)basis.size(), 2);

    // non-hyperbolic: rho = 1 at the origin has a zero eigenvalue
    Lorenz L1;
    L1.rho = 1.0;
    L1.jacobian({0, 0, 0}, A);
    e.compute(A);
    check(!e.hyperbolic(), "rho = 1: origin flagged non-hyperbolic",
          std::abs(e.values()[2].real()), 1e-6);

    // random matrices
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-5, 5);
    double worst = 0.0;
    for (int t = 0; t < 2000; ++t)
    {
        double R[3][3];
        for (auto& row : R)
            for (double& v : row)
                v = U(rng);
        Eigen3 er(R);
        worst = std::max(worst, eigResidual(R, er));
    }
    expectBelow("2000 random 3x3: worst |A v - l v|", worst, 1e-9);
}

// The original recursion from sims/lorenz_manifold (origin, stable side), with its
// eigenvector scaling: theta1 <-> -beta along (0,0,1), theta2 <-> lambda_2 along
// (sigma / (sigma + lambda_2), 1, 0).
std::vector<std::vector<std::vector<double>>> referenceCoefficients(const Lorenz& L, int order)
{
    std::vector<std::vector<std::vector<double>>> p(
        3, std::vector<std::vector<double>>(order + 1, std::vector<double>(order + 1, 0.0)));
    double root = std::sqrt((L.sigma + 1) * (L.sigma + 1) + 4 * L.sigma * (L.rho - 1));
    double l1 = -L.beta, l2 = (-(L.sigma + 1) - root) / 2;
    p[2][1][0] = 1.0;
    p[0][0][1] = L.sigma / (L.sigma + l2);
    p[1][0][1] = 1.0;
    double A[3][3] = {{-L.sigma, L.sigma, 0}, {L.rho, -1, 0}, {0, 0, -L.beta}};
    auto S = [&](int c1, int c2, int m, int n) {
        double s = 0;
        for (int i = 0; i <= m; ++i)
            for (int j = 0; j <= n; ++j)
                s += p[c1][i][j] * p[c2][m - i][n - j];
        return s;
    };
    for (int d = 2; d <= order; ++d)
        for (int m = 0; m <= d; ++m)
        {
            int n = d - m;
            cd mu = m * l1 + n * l2;
            Eigen3::CVec3 b = {0.0, S(0, 2, m, n), -S(0, 1, m, n)};
            // reuse the module's solver path indirectly: plain Cramer like the sim
            double M[3][3];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    M[i][j] = A[i][j] - (i == j ? mu.real() : 0.0);
            auto det3 = [](double m3[3][3]) {
                return m3[0][0] * (m3[1][1] * m3[2][2] - m3[1][2] * m3[2][1])
                     - m3[0][1] * (m3[1][0] * m3[2][2] - m3[1][2] * m3[2][0])
                     + m3[0][2] * (m3[1][0] * m3[2][1] - m3[1][1] * m3[2][0]);
            };
            double D = det3(M);
            for (int col = 0; col < 3; ++col)
            {
                double Mc[3][3];
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        Mc[r][c] = c == col ? b[r].real() : M[r][c];
                p[col][m][n] = det3(Mc) / D;
            }
        }
    return p;
}

void testAgainstLorenzManifold()
{
    std::printf("InvariantManifold vs sims/lorenz_manifold (origin, stable, order 10)\n");
    Lorenz L;
    double A[3][3];
    L.jacobian({0, 0, 0}, A);
    InvariantManifold W;
    bool ok = W.build(A, Lorenz::quadratic(), InvariantManifold::Side::Stable, 10);
    check(ok && W.dim() == 2 && !W.complexPair(), "built, dim 2, real", W.dim(), 2);
    if (!ok)
        return;

    auto ref = referenceCoefficients(L, 10);
    // mine: s_a along v_a (lambda_2 ~ -22.8, first in ascending order), s_b along
    // v_b (-beta). v_a = alpha * (sigma/(sigma+l2), 1, 0), v_b = gamma * (0, 0, 1).
    double alpha = W.coeff(1, 0)[1].real();           // y component of ref is 1
    double gamma = W.coeff(0, 1)[2].real();           // z component of ref is 1
    double worst = 0.0;
    for (int m = 0; m <= 10; ++m)
        for (int j = 0; j + m <= 10; ++j)
            for (int c = 0; c < 3; ++c)
            {
                double mine = W.coeff(m, j)[c].real();
                double theirs = ref[c][j][m] * std::pow(alpha, m) * std::pow(gamma, j);
                double scale = std::max(1e-12, std::abs(theirs));
                worst = std::max(worst, std::abs(mine - theirs) / scale
                                            * (std::abs(theirs) > 1e-14 ? 1.0 : 0.0));
                worst = std::max(worst, std::abs(W.coeff(m, j)[c].imag()));
            }
    expectBelow("all coefficients match after rescaling", worst, 1e-9);
}

// Conjugacy phi^tau(x* + K(s)) == x* + K(e^{Lambda tau} s), integrated with RK4.
void testConjugacy(const char* name, const Vec3& xs, InvariantManifold::Side side, int order,
                   double r, double tau, double limit)
{
    Lorenz L;
    double A[3][3];
    L.jacobian(xs, A);
    InvariantManifold W;
    if (!W.build(A, Lorenz::quadratic(), side, order))
    {
        std::printf("  [FAIL] %s: %s\n", name, W.error().c_str());
        ++failures;
        return;
    }
    std::printf("%s: dim %d%s, order %d, min small divisor %.4f\n", name, W.dim(),
                W.complexPair() ? " (complex pair)" : "", order, W.minDivisor());
    for (double rr : {0.25, 0.5, 1.0, 2.0, 4.0, 8.0})
        std::printf("      max rel. invariance defect at |theta| = %-4g: %.3e\n", rr,
                    W.maxRelDefect(rr));

    double worst = 0.0;
    for (int i = 0; i < 12; ++i)
    {
        double ph = 6.283185307179586 * i / 12;
        Vec3 theta;
        for (int c = 0; c < 3; ++c)
            theta[c] = r * (W.dim() == 2 ? std::cos(ph) * W.orthoBasis(0)[c]
                                               + std::sin(ph) * W.orthoBasis(1)[c]
                                         : (i % 2 ? -1.0 : 1.0) * W.orthoBasis(0)[c]);
        auto s = W.toParam(theta);
        // linear(toParam(theta)) must give theta back (theta lies in E)
        auto back = W.linear(s);
        worst = std::max(worst, dist(back, theta) / r);

        Vec3 x0 = W.eval(s), x1 = W.eval(W.flowParam(s, tau));
        for (int c = 0; c < 3; ++c)
        {
            x0[c] += xs[c];
            x1[c] += xs[c];
        }
        Vec3 flowed = rk4(L, x0, tau, 1e-4);
        worst = std::max(worst, dist(flowed, x1) / std::max(dist(x1, xs), 1e-12));
    }
    char what[128];
    std::snprintf(what, sizeof(what), "phi^%.2g(K(s)) vs K(e^{L t}s), |theta| = %g", tau, r);
    expectBelow(what, worst, limit);
}

// LorenzSystem: f = 0 at every equilibrium, and f(x* + u) = Df(x*) u + Q(u, u) exactly
// (the field is quadratic, so the split has no remainder).
void testLorenzSystem()
{
    std::printf("LorenzSystem\n");
    engine::LorenzSystem sys;
    auto eqs = sys.equilibria();
    check(eqs.size() == 3, "rho = 28: three equilibria", (double)eqs.size(), 3);
    const auto Q = engine::LorenzSystem::quadratic();
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> unit(-3.0, 3.0);
    double worstZero = 0.0, worstSplit = 0.0;
    for (const auto& e : eqs)
    {
        double f[3];
        sys.eval(e.x.data(), f);
        worstZero = std::max(worstZero, std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]));
        double J[3][3];
        sys.jacobian(e.x, J);
        for (int k = 0; k < 20; ++k)
        {
            double u[3] = {unit(rng), unit(rng), unit(rng)}, x[3], fx[3];
            for (int c = 0; c < 3; ++c)
                x[c] = e.x[c] + u[c];
            sys.eval(x, fx);
            for (int c = 0; c < 3; ++c)
            {
                double s = 0.0;
                for (int i = 0; i < 3; ++i)
                {
                    s += J[c][i] * u[i];
                    for (int j = 0; j < 3; ++j)
                        s += Q[c][i][j] * u[i] * u[j];
                }
                worstSplit = std::max(worstSplit, std::abs(s - fx[c]));
            }
        }
    }
    expectBelow("|f(x*)| at O, C+, C-", worstZero, 1e-12);
    expectBelow("f(x* + u) - (Df u + Q(u,u))", worstSplit, 1e-11);
    sys.rho = 0.5;
    check(sys.equilibria().size() == 1, "rho = 0.5: origin only", (double)sys.equilibria().size(), 1);
}

} // namespace

// Taylor: exact on the harmonic oscillator; on Lorenz, against RK4 at a tiny step.
void testTaylor()
{
    std::printf("TaylorIntegrator\n");
    engine::TaylorIntegrator ti;
    ti.order = 20;

    // x' = -y, y' = x: c_{k+1} = (-y_k, x_k) / (k+1); x(t) = (cos t, sin t)
    engine::TaylorIntegrator::Jet osc = [](const double* x, double, int n, double* c) {
        c[0] = x[0];
        c[1] = x[1];
        for (int k = 0; k < n; ++k)
        {
            c[2 * (k + 1)] = -c[2 * k + 1] / (k + 1);
            c[2 * (k + 1) + 1] = c[2 * k] / (k + 1);
        }
    };
    double x[2] = {1.0, 0.0}, t = 0.0;
    int samples = 0;
    int steps = ti.advance(osc, 2, x, t, 20.0, 0.01, [&](const double*, const double*, double) { ++samples; });
    expectBelow("oscillator to t = 20: |x - (cos, sin)|",
                std::hypot(x[0] - std::cos(20.0), x[1] - std::sin(20.0)), 1e-12);
    check(t == 20.0, "lands exactly on t + dt", t, 20.0);
    check(samples >= 2000, "dense output every <= 0.01", samples, 2000);
    std::printf("    (%d steps)\n", steps);
    double back[2] = {x[0], x[1]};
    ti.advance(osc, 2, back, t, -20.0);
    expectBelow("and back to the seed", std::hypot(back[0] - 1.0, back[1]), 1e-12);

    // Lorenz from (1, 1, 1) to t = 2 vs RK4 at h = 1e-4 (error ~1e-11 there)
    engine::LorenzSystem sys;
    engine::TaylorIntegrator::Jet lj = [&](const double* p, double, int n, double* c) { sys.jet(p, n, c); };
    double a[3] = {1.0, 1.0, 1.0};
    t = 0.0;
    steps = ti.advance(lj, 3, a, t, 2.0);
    double b[3] = {1.0, 1.0, 1.0};
    auto f = [&](const double* p, double* d) { sys.eval(p, d); };
    const double h = 1e-4;
    for (int k = 0; k < 20000; ++k)
    {
        double k1[3], k2[3], k3[3], k4[3], tmp[3];
        f(b, k1);
        for (int i = 0; i < 3; ++i) tmp[i] = b[i] + 0.5 * h * k1[i];
        f(tmp, k2);
        for (int i = 0; i < 3; ++i) tmp[i] = b[i] + 0.5 * h * k2[i];
        f(tmp, k3);
        for (int i = 0; i < 3; ++i) tmp[i] = b[i] + h * k3[i];
        f(tmp, k4);
        for (int i = 0; i < 3; ++i) b[i] += h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    expectBelow("Lorenz to t = 2: Taylor vs RK4(1e-4)",
                std::max({std::abs(a[0] - b[0]), std::abs(a[1] - b[1]), std::abs(a[2] - b[2])}), 1e-8);
    std::printf("    (%d Taylor steps vs 20000 RK4)\n", steps);
}

int main()
{
    testLorenzSystem();
    testTaylor();
    testEigen();
    testAgainstLorenzManifold();

    Lorenz L;
    Vec3 origin = {0, 0, 0}, cp = L.cPlus();
    using S = InvariantManifold::Side;
    testConjugacy("origin W^s", origin, S::Stable, 12, 1.0, 0.1, 1e-6);
    testConjugacy("origin W^u", origin, S::Unstable, 12, 1.0, 0.1, 1e-6);
    testConjugacy("C+ W^s", cp, S::Stable, 12, 1.0, 0.05, 1e-6);
    testConjugacy("C+ W^u", cp, S::Unstable, 12, 0.5, 0.2, 1e-5);

    std::printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
    return failures ? 1 : 0;
}
