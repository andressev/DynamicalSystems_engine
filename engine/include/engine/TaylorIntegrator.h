#ifndef ENGINE_TAYLOR_INTEGRATOR_H
#define ENGINE_TAYLOR_INTEGRATOR_H

#include <functional>

namespace engine {

// High-order Taylor series method for dx/dt = f(x, t): at each step the solution's
// Taylor polynomial x(t + s) = sum_k c_k s^k is built to `order` from a "jet" the field
// provides (its normalized derivatives — for a polynomial field like Lorenz an exact,
// cheap recurrence), and the step is chosen from how fast the last two coefficients
// decay (Jorba & Zou 2005), so the truncation error stays near `tolerance`. Far more
// accurate per unit work than RK4 at tight tolerances — meant for the few trajectories
// that must be right (singles), not for clouds. Math layer: std-only.
class TaylorIntegrator {
public:
    static constexpr int kMaxDim = 4;
    static constexpr int kMaxOrder = 60;

    // Fill c[k * dim + i], k = 0..order: the k-th Taylor coefficient of x_i(t + s) at
    // s = 0 (c_0 = x). See LorenzSystem::jet.
    using Jet = std::function<void(const double* x, double t, int order, double* c)>;

    int order = 20;
    double tolerance = 1e-14;   // per step, relative to max(1, |x|_inf)
    int maxSteps = 20000;       // per advance(), so a hitch can't freeze the frame

    // The step size the coefficients c (order + 1 rows of dim) allow, > 0.
    double stepSize(const double* c, int dim) const;

    // x(s) and x'(s) from the polynomial, 0 <= |s| <= the step it was built for.
    static void evaluate(const double* c, int dim, int order, double s, double* x,
                         double* dxds = nullptr);

    // Advance by dt (either sign) in adaptive steps; the last one lands exactly on
    // t + dt. `sample(x, dxdt, t)` (optional) is called at every step end and, inside
    // steps longer than sampleStep, at points spaced at most sampleStep apart (dense
    // output: the polynomial evaluated, no extra field calls) — so a drawn trail stays
    // smooth however long the steps get. Returns the number of steps taken.
    using Sample = std::function<void(const double* x, const double* dxdt, double t)>;
    int advance(const Jet& jet, int dim, double* x, double& t, double dt,
                double sampleStep = 0.0, const Sample& sample = nullptr) const;
};

} // namespace engine

#endif
