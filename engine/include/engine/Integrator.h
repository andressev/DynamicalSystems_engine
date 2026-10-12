#ifndef ENGINE_INTEGRATOR_H
#define ENGINE_INTEGRATOR_H

#include <functional>

namespace engine {

// Fixed-step ODE integrator for dx/dt = f(x, t), dimension-agnostic up to kMaxDim.
// It knows nothing about where f comes from: a parsed VectorField, a hard-coded
// Lorenz system or anything else that fills a derivative array. M3 in the roadmap;
// pure std C++ (math layer), so the same code can move to a CUDA kernel later.
class Integrator {
public:
    static constexpr int kMaxDim = 4;

    enum class Method { Euler, RK2, RK4 };

    // dxdt[0..dim-1] = f(x, t)
    using Field = std::function<void(const double* x, double t, double* dxdt)>;

    Method method = Method::RK4;
    double maxStep = 0.01;  // largest step an advance() substep is allowed to take

    // One step of size h, in place. h may be negative — that integrates the flow
    // backwards in time.
    void step(const Field& f, int dim, double* x, double& t, double h) const;

    // Advance by dt, split into equal substeps of at most maxStep, never more than
    // maxSubsteps of them (so a frame hitch or a tiny maxStep cannot freeze the
    // frame). Returns the number of substeps actually taken.
    int advance(const Field& f, int dim, double* x, double& t, double dt,
                int maxSubsteps = 4096) const;

    static const char* methodName(Method m);
};

} // namespace engine

#endif
