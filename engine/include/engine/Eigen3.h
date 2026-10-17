#ifndef ENGINE_EIGEN3_H
#define ENGINE_EIGEN3_H

#include <array>
#include <complex>
#include <vector>

namespace engine {

// Eigen-decomposition of a real 3x3 matrix, in double precision — the linearization
// A = Df(x*) at an equilibrium. Eigenvalues come from the characteristic cubic (one
// real root, deflated to a quadratic, all three Newton-polished); eigenvectors from the
// null space of A - lambda I as a cross product of two of its rows. A complex pair is
// stored as exact conjugates (values and vectors), so real bases built from it are
// exact. Pure std (math layer).
//
// Not handled: defective matrices (a repeated eigenvalue with a 1-D eigenspace) — the
// vector for the repeated value is then a duplicate; `independent()` reports it.
class Eigen3 {
public:
    using cd = std::complex<double>;
    using CVec3 = std::array<cd, 3>;
    using Vec3 = std::array<double, 3>;

    Eigen3() = default;
    explicit Eigen3(const double A[3][3]) { compute(A); }

    void compute(const double A[3][3]);

    // Sorted by real part, ascending; a conjugate pair is adjacent, +Im first.
    const std::array<cd, 3>& values() const { return vals; }
    // vectors()[i] is the eigenvector for values()[i]: unit Hermitian norm, phase fixed
    // so its largest component is real and positive (real for real eigenvalues).
    const std::array<CVec3, 3>& vectors() const { return vecs; }

    // An eigenvalue with |Re| <= tol is central; hyperbolic = none is.
    bool hyperbolic(double tol = 1e-6) const;
    int dimStable(double tol = 1e-6) const;    // count with Re < -tol
    int dimUnstable(double tol = 1e-6) const;  // count with Re > tol
    bool isComplex(int i, double tol = 1e-12) const;

    // A real basis of E^s (sign < 0) or E^u (sign > 0): a real eigenvector per real
    // eigenvalue, {Re v, Im v} per complex pair. Not orthonormal.
    std::vector<Vec3> realBasis(int sign, double tol = 1e-6) const;

    // False when two eigenvectors came out parallel (defective / degenerate A).
    bool independent() const { return indep; }

private:
    std::array<cd, 3> vals{};
    std::array<CVec3, 3> vecs{};
    bool indep = false;
};

} // namespace engine

#endif
