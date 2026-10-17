#include <engine/Eigen3.h>

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

using cd = std::complex<double>;
using CVec3 = Eigen3::CVec3;

// Bilinear (not Hermitian) cross product: a . (a x b) = 0 holds without conjugation,
// which is what a null vector of a complex matrix needs.
CVec3 cross(const CVec3& a, const CVec3& b)
{
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double norm2(const CVec3& v)
{
    return std::norm(v[0]) + std::norm(v[1]) + std::norm(v[2]);
}

// Unit Hermitian norm, largest component rotated onto the positive real axis — a
// deterministic phase, so a real eigenvector comes out real.
CVec3 normalized(CVec3 v)
{
    int big = 0;
    for (int i = 1; i < 3; ++i)
        if (std::abs(v[i]) > std::abs(v[big]))
            big = i;
    cd phase = std::abs(v[big]) > 0.0 ? std::conj(v[big]) / std::abs(v[big]) : cd(1.0);
    double n = std::sqrt(norm2(v));
    for (cd& c : v)
        c = n > 0.0 ? c * phase / n : c;
    return v;
}

// Null vector of A - lambda I. Rank 2 (simple eigenvalue): the largest cross product of
// two rows. Rank 1 (repeated eigenvalue, 2-D eigenspace): a vector orthogonal (bilinearly)
// to the one nonzero row; `which` picks the first or second such vector.
CVec3 nullVector(const double A[3][3], cd lambda, int which)
{
    CVec3 r[3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r[i][j] = A[i][j] - (i == j ? lambda : cd(0.0));

    double scale = 0.0;
    for (const CVec3& row : r)
        scale = std::max(scale, norm2(row));

    CVec3 best{};
    double bestN = 0.0;
    for (auto [i, j] : {std::pair{0, 1}, std::pair{0, 2}, std::pair{1, 2}})
    {
        CVec3 c = cross(r[i], r[j]);
        double n = norm2(c);
        if (n > bestN)
        {
            bestN = n;
            best = c;
        }
    }
    if (bestN > 1e-20 * scale * scale)
        return normalized(best);

    // rank <= 1
    int row = 0;
    for (int i = 1; i < 3; ++i)
        if (norm2(r[i]) > norm2(r[row]))
            row = i;
    if (norm2(r[row]) <= 1e-24 * std::max(1.0, scale))
    {
        CVec3 e{};               // A = lambda I: any basis
        e[which % 3] = 1.0;
        return e;
    }
    // first complement vector: r x e_k for the axis least aligned with r
    int k = 0;
    for (int i = 1; i < 3; ++i)
        if (std::abs(r[row][i]) < std::abs(r[row][k]))
            k = i;
    CVec3 ek{};
    ek[k] = 1.0;
    CVec3 v1 = cross(r[row], ek);
    return normalized(which == 0 ? v1 : cross(r[row], v1));
}

// p(l) = l^3 + a l^2 + b l + c
cd cubic(cd l, double a, double b, double c) { return ((l + a) * l + b) * l + c; }
cd cubicPrime(cd l, double a, double b) { return (3.0 * l + 2.0 * a) * l + b; }

cd polish(cd l, double a, double b, double c)
{
    for (int it = 0; it < 6; ++it)
    {
        cd d = cubicPrime(l, a, b);
        if (std::abs(d) == 0.0)
            break;
        cd step = cubic(l, a, b, c) / d;
        l -= step;
        if (std::abs(step) <= 1e-15 * std::max(1.0, std::abs(l)))
            break;
    }
    return l;
}

// One real root of the monic cubic: Cardano on the depressed form, then Newton.
double realRoot(double a, double b, double c)
{
    double p = b - a * a / 3.0;
    double q = 2.0 * a * a * a / 27.0 - a * b / 3.0 + c;
    double disc = q * q / 4.0 + p * p * p / 27.0;
    double t;
    if (disc >= 0.0)
    {
        double s = std::sqrt(disc);
        t = std::cbrt(-q / 2.0 + s) + std::cbrt(-q / 2.0 - s);
    }
    else
    {
        // three real roots: trigonometric form, take the largest
        double m = 2.0 * std::sqrt(-p / 3.0);
        double arg = std::clamp(3.0 * q / (p * m), -1.0, 1.0);
        t = m * std::cos(std::acos(arg) / 3.0);
    }
    return polish(cd(t - a / 3.0), a, b, c).real();
}

} // namespace

void Eigen3::compute(const double A[3][3])
{
    // characteristic polynomial det(lI - A) = l^3 + a l^2 + b l + c
    double tr = A[0][0] + A[1][1] + A[2][2];
    double minors = A[0][0] * A[1][1] - A[0][1] * A[1][0]
                  + A[0][0] * A[2][2] - A[0][2] * A[2][0]
                  + A[1][1] * A[2][2] - A[1][2] * A[2][1];
    double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
               - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
               + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    double a = -tr, b = minors, c = -det;

    double r = realRoot(a, b, c);
    // deflate: l^2 + (a + r) l + (b + r (a + r))
    double qb = a + r, qc = b + r * qb;
    double disc = qb * qb - 4.0 * qc;
    double scale = std::max({1.0, std::abs(qb), std::sqrt(std::abs(qc))});
    if (disc < 0.0 && -disc > 1e-24 * scale * scale)
    {
        cd z = polish(cd(-qb / 2.0, std::sqrt(-disc) / 2.0), a, b, c);
        if (z.imag() < 0.0)
            z = std::conj(z);
        vals = {cd(r), z, std::conj(z)};
    }
    else
    {
        double s = std::sqrt(std::max(disc, 0.0));
        // stable quadratic formula: no cancellation in the root of larger magnitude
        double q = -0.5 * (qb + (qb >= 0.0 ? s : -s));
        double r1 = q, r2 = q != 0.0 ? qc / q : 0.0;
        vals = {cd(r), polish(cd(r1), a, b, c).real(), polish(cd(r2), a, b, c).real()};
    }

    // ascending real part; keep a conjugate pair adjacent with +Im first
    std::sort(vals.begin(), vals.end(), [](const cd& x, const cd& y) {
        if (x.real() != y.real())
            return x.real() < y.real();
        return x.imag() > y.imag();
    });

    for (int i = 0; i < 3; ++i)
    {
        if (i > 0 && isComplex(i) && vals[i] == std::conj(vals[i - 1]))
        {
            for (int k = 0; k < 3; ++k)
                vecs[i][k] = std::conj(vecs[i - 1][k]);
            continue;
        }
        // repeated real value: take the next vector of a 2-D eigenspace
        int which = 0;
        if (i > 0 && !isComplex(i)
            && std::abs(vals[i] - vals[i - 1]) <= 1e-9 * std::max(1.0, std::abs(vals[i])))
            which = 1;
        vecs[i] = nullVector(A, vals[i], which);
        if (!isComplex(i))
            for (cd& comp : vecs[i])
                comp = cd(comp.real(), 0.0);
    }

    // independence: |det [v0 v1 v2]| well away from 0 (vectors are unit)
    CVec3 c01 = cross(vecs[0], vecs[1]);
    cd d = c01[0] * vecs[2][0] + c01[1] * vecs[2][1] + c01[2] * vecs[2][2];
    indep = std::abs(d) > 1e-8;
}

bool Eigen3::isComplex(int i, double tol) const
{
    return std::abs(vals[i].imag()) > tol * std::max(1.0, std::abs(vals[i]));
}

bool Eigen3::hyperbolic(double tol) const
{
    for (const cd& l : vals)
        if (std::abs(l.real()) <= tol)
            return false;
    return true;
}

int Eigen3::dimStable(double tol) const
{
    return (int)std::count_if(vals.begin(), vals.end(),
                              [tol](const cd& l) { return l.real() < -tol; });
}

int Eigen3::dimUnstable(double tol) const
{
    return (int)std::count_if(vals.begin(), vals.end(),
                              [tol](const cd& l) { return l.real() > tol; });
}

std::vector<Eigen3::Vec3> Eigen3::realBasis(int sign, double tol) const
{
    std::vector<Vec3> out;
    for (int i = 0; i < 3; ++i)
    {
        double re = vals[i].real();
        if (!(sign < 0 ? re < -tol : re > tol))
            continue;
        if (isComplex(i))
        {
            if (vals[i].imag() < 0.0)
                continue; // its conjugate already contributed both vectors
            out.push_back({vecs[i][0].real(), vecs[i][1].real(), vecs[i][2].real()});
            out.push_back({vecs[i][0].imag(), vecs[i][1].imag(), vecs[i][2].imag()});
        }
        else
        {
            out.push_back({vecs[i][0].real(), vecs[i][1].real(), vecs[i][2].real()});
        }
    }
    return out;
}

} // namespace engine
