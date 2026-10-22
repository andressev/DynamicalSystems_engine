#ifndef ENGINE_INVARIANT_MANIFOLD_H
#define ENGINE_INVARIANT_MANIFOLD_H

#include <engine/Eigen3.h>

#include <array>
#include <complex>
#include <string>
#include <vector>

namespace engine {

// The local stable or unstable manifold of an equilibrium v* of a QUADRATIC field,
//     f(v* + u) = A u + Q(u, u),
// as a Taylor map K: R^k -> R^3 (k = 1 or 2) built by the parametrization method:
// K conjugates the linear flow on the parameter to the true flow on the manifold,
//     phi^t(v* + K(s)) = v* + K(e^{Lambda t} s).
// It is the Lorenz-manifold recursion (math/lorenz-stable-manifold) generalised to any
// equilibrium, either side, and a complex-conjugate eigenpair. Pure std (math layer).
//
// Parameter coordinates s are chosen so that K's linear part is the identity on the
// eigenspace E: linear(s) = B s with B the real basis from Eigen3::realBasis (real
// eigenvectors, or {Re v, Im v} for a pair). So a point theta of E has
// s = toParam(theta), and v* + K(toParam(theta)) is h^{-1}(theta) restricted to E.
// All results are relative to v* (add it yourself).
class InvariantManifold {
public:
    using cd = std::complex<double>;
    using Vec3 = std::array<double, 3>;
    using Param = std::array<double, 2>;   // k = 1 uses [0] only
    // Q(u, v)_c = sum_ij Q[c][i][j] u_i v_j  (need not be symmetric)
    using Quadratic = std::array<std::array<std::array<double, 3>, 3>, 3>;

    enum class Side { Stable, Unstable };

    // Solve the coefficients up to total degree `order`. Returns false (and sets
    // error()) when the chosen side is not 1- or 2-dimensional, or A is degenerate.
    bool build(const double A[3][3], const Quadratic& Q, Side side, int order,
               double hyperbolicTol = 1e-6);

    bool valid() const { return k > 0; }
    const std::string& error() const { return err; }
    int dim() const { return k; }
    int order() const { return n; }
    bool complexPair() const { return pair; }
    cd eigenvalue(int i) const { return lambda[i]; }   // i < dim()
    const Eigen3& eigen() const { return eig; }

    Vec3 eval(const Param& s) const;                  // K(s)
    Vec3 linear(const Param& s) const;                // B s = DK(0) s, a point of E
    Param toParam(const Vec3& theta) const;           // least-squares inverse of linear
    Param flowParam(const Param& s, double t) const;  // e^{Lambda t} s
    Vec3 basis(int i) const { return B[i]; }          // columns of B (real, not unit)
    Vec3 orthoBasis(int i) const { return O[i]; }     // orthonormal basis of E

    // |f(v* + K(s)) - d/dt K(e^{Lambda t} s)|_{t=0}: zero for the exact manifold, so it
    // measures truncation + convergence. maxDefect samples theta on the sphere of
    // radius r in E (a circle for k = 2, the two points +-r for k = 1) and returns the
    // largest defect relative to |f| there.
    double defect(const Param& s) const;
    double maxRelDefect(double r, int samples = 64) const;
    // maxRelDefect(r, samples) < tol, exactly, but stops at the first sample at or over
    // tol — for searches that only need the verdict (a trusted-radius bisection).
    bool relDefectBelow(double r, double tol, int samples = 64) const;

    // Smallest |mu - lambda_j| over every solved order (mu = alpha . lambda) and every
    // eigenvalue of A: the small divisor. Near 0 = near-resonance, coefficients blow up.
    double minDivisor() const { return minDiv; }

    // Coefficient p_{m,j} (complex; m + j <= order, j = 0 when k = 1). In the complex
    // case the variables are (z, conj z) with z = (s0 - i s1) / 2.
    const std::array<cd, 3>& coeff(int m, int j) const { return p[idx(m, j)]; }

private:
    int idx(int m, int j) const { return m * (n + 1) + j; }
    void vars(const Param& s, cd& w1, cd& w2) const;
    Vec3 field(const Vec3& u) const;   // A u + Q(u, u)
    double defectAndField(const Param& s, double& fieldNorm) const;   // defect, |f(K(s))|
    Param sphereParam(double r, int i, int count) const;   // maxRelDefect's sample i

    int k = 0, n = 0;
    bool pair = false;
    std::string err;
    double A_[3][3] = {};
    Quadratic Q_{};
    Eigen3 eig;
    cd lambda[2] = {};
    Vec3 B[2] = {}, O[2] = {};
    double gram[2][2] = {};          // inverse of B^T B, for toParam
    std::vector<std::array<cd, 3>> p;
    double minDiv = 0.0;
};

} // namespace engine

#endif
