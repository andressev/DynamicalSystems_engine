#include <engine/InvariantManifold.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {

namespace {

using cd = std::complex<double>;
using CVec3 = std::array<cd, 3>;

// (A - mu I) x = b by Gaussian elimination with partial pivoting.
CVec3 solveShifted(const double A[3][3], cd mu, CVec3 b)
{
    cd M[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            M[i][j] = A[i][j] - (i == j ? mu : cd(0.0));

    for (int col = 0; col < 3; ++col)
    {
        int piv = col;
        for (int r = col + 1; r < 3; ++r)
            if (std::abs(M[r][col]) > std::abs(M[piv][col]))
                piv = r;
        if (piv != col)
        {
            for (int j = 0; j < 3; ++j)
                std::swap(M[col][j], M[piv][j]);
            std::swap(b[col], b[piv]);
        }
        for (int r = col + 1; r < 3; ++r)
        {
            cd f = M[r][col] / M[col][col];
            for (int j = col; j < 3; ++j)
                M[r][j] -= f * M[col][j];
            b[r] -= f * b[col];
        }
    }
    CVec3 x{};
    for (int r = 2; r >= 0; --r)
    {
        cd s = b[r];
        for (int j = r + 1; j < 3; ++j)
            s -= M[r][j] * x[j];
        x[r] = s / M[r][r];
    }
    return x;
}

double length(const InvariantManifold::Vec3& v)
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double dot(const InvariantManifold::Vec3& a, const InvariantManifold::Vec3& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

} // namespace

bool InvariantManifold::build(const double A[3][3], const Quadratic& Q, Side side, int order,
                              double hyperbolicTol)
{
    k = 0;
    pair = false;
    err.clear();
    p.clear();
    n = std::max(order, 1);
    Q_ = Q;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            A_[i][j] = A[i][j];

    eig.compute(A);
    if (!eig.independent())
    {
        err = "degenerate linearization (eigenvectors not independent)";
        return false;
    }

    // the eigen-indices on the requested side, in Eigen3's order
    int pick[3], count = 0;
    for (int i = 0; i < 3; ++i)
    {
        double re = eig.values()[i].real();
        if (side == Side::Stable ? re < -hyperbolicTol : re > hyperbolicTol)
            pick[count++] = i;
    }
    if (count < 1 || count > 2)
    {
        err = count == 0 ? "this side is empty (no eigenvalues)"
                         : "this side is all of R^3 (no 1- or 2-D manifold)";
        return false;
    }

    // first-order coefficients and the real basis B with linear(s) = B s
    const auto& vals = eig.values();
    const auto& vecs = eig.vectors();
    p.assign((size_t)(n + 1) * (n + 1), CVec3{});
    auto re = [](const CVec3& v) { return Vec3{v[0].real(), v[1].real(), v[2].real()}; };
    auto im = [](const CVec3& v) { return Vec3{v[0].imag(), v[1].imag(), v[2].imag()}; };
    if (count == 2 && eig.isComplex(pick[0]))
    {
        // variables (z, conj z), z = (s0 - i s1) / 2: K_lin = v z + conj(v z)
        //                                                  = Re v s0 + Im v s1
        pair = true;
        int i = vals[pick[0]].imag() > 0.0 ? pick[0] : pick[1];
        lambda[0] = vals[i];
        lambda[1] = std::conj(vals[i]);
        p[idx(1, 0)] = vecs[i];
        for (int c = 0; c < 3; ++c)
            p[idx(0, 1)][c] = std::conj(vecs[i][c]);
        B[0] = re(vecs[i]);
        B[1] = im(vecs[i]);
    }
    else
    {
        for (int a = 0; a < count; ++a)
        {
            lambda[a] = vals[pick[a]].real();
            B[a] = re(vecs[pick[a]]);
            p[a == 0 ? idx(1, 0) : idx(0, 1)] = vecs[pick[a]];
        }
    }
    k = count;

    // orthonormal basis of E (Gram-Schmidt) and (B^T B)^{-1} for toParam
    O[0] = B[0];
    double l0 = length(O[0]);
    for (double& c : O[0])
        c /= l0;
    if (k == 2)
    {
        double d = dot(B[1], O[0]);
        for (int c = 0; c < 3; ++c)
            O[1][c] = B[1][c] - d * O[0][c];
        double l1 = length(O[1]);
        for (double& c : O[1])
            c /= l1;
        double g00 = dot(B[0], B[0]), g01 = dot(B[0], B[1]), g11 = dot(B[1], B[1]);
        double det = g00 * g11 - g01 * g01;
        gram[0][0] = g11 / det;
        gram[1][1] = g00 / det;
        gram[0][1] = gram[1][0] = -g01 / det;
    }
    else
    {
        O[1] = {0.0, 0.0, 0.0};
        gram[0][0] = 1.0 / dot(B[0], B[0]);
        gram[0][1] = gram[1][0] = gram[1][1] = 0.0;
    }

    // Q's nonzero entries, so the Cauchy products skip the zero ones
    struct Term {
        int c, i, j;
        double q;
    };
    std::vector<Term> terms;
    for (int c = 0; c < 3; ++c)
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                if (Q[c][i][j] != 0.0)
                    terms.push_back({c, i, j, Q[c][i][j]});

    // homological equation, order by order:
    //   (A - (m lambda_0 + j lambda_1) I) p_mj = -[Q(K, K)]_mj
    // where the right side only involves coefficients of lower total degree
    minDiv = std::numeric_limits<double>::infinity();
    for (int d = 2; d <= n; ++d)
    {
        for (int m = d; m >= 0; --m)
        {
            int j = d - m;
            if (k == 1 && j > 0)
                continue;
            cd mu = (double)m * lambda[0] + (double)j * lambda[1];
            for (const cd& l : vals)
                minDiv = std::min(minDiv, std::abs(mu - l));

            CVec3 b{};
            for (int a1 = 0; a1 <= m; ++a1)
                for (int a2 = 0; a2 <= j; ++a2)
                {
                    if (a1 + a2 == 0 || a1 + a2 == d)
                        continue; // p_00 = 0
                    const CVec3& u = p[idx(a1, a2)];
                    const CVec3& v = p[idx(m - a1, j - a2)];
                    for (const Term& t : terms)
                        b[t.c] -= t.q * u[t.i] * v[t.j];
                }
            p[idx(m, j)] = solveShifted(A_, mu, b);
        }
    }
    if (!std::isfinite(minDiv))
        minDiv = 0.0;
    return true;
}

void InvariantManifold::vars(const Param& s, cd& w1, cd& w2) const
{
    if (pair)
    {
        w1 = cd(s[0], -s[1]) * 0.5;
        w2 = std::conj(w1);
    }
    else
    {
        w1 = s[0];
        w2 = k == 2 ? s[1] : 0.0;
    }
}

InvariantManifold::Vec3 InvariantManifold::eval(const Param& s) const
{
    if (!valid())
        return {0.0, 0.0, 0.0};
    cd w1, w2;
    vars(s, w1, w2);
    std::vector<cd> pw2(n + 1);
    pw2[0] = 1.0;
    for (int j = 1; j <= n; ++j)
        pw2[j] = pw2[j - 1] * w2;

    CVec3 out{};
    cd pw1 = 1.0;
    for (int m = 0; m <= n; ++m, pw1 *= w1)
        for (int j = 0; j + m <= n && (k == 2 || j == 0); ++j)
        {
            cd basis = pw1 * pw2[j];
            const CVec3& c = p[idx(m, j)];
            for (int i = 0; i < 3; ++i)
                out[i] += c[i] * basis;
        }
    return {out[0].real(), out[1].real(), out[2].real()};
}

InvariantManifold::Vec3 InvariantManifold::linear(const Param& s) const
{
    Vec3 out{};
    for (int a = 0; a < k; ++a)
        for (int c = 0; c < 3; ++c)
            out[c] += B[a][c] * s[a];
    return out;
}

InvariantManifold::Param InvariantManifold::toParam(const Vec3& theta) const
{
    if (!valid())
        return {0.0, 0.0};
    double bt[2] = {dot(B[0], theta), k == 2 ? dot(B[1], theta) : 0.0};
    return {gram[0][0] * bt[0] + gram[0][1] * bt[1], gram[1][0] * bt[0] + gram[1][1] * bt[1]};
}

InvariantManifold::Param InvariantManifold::flowParam(const Param& s, double t) const
{
    if (pair)
    {
        cd z = cd(s[0], -s[1]) * 0.5 * std::exp(lambda[0] * t);
        return {2.0 * z.real(), -2.0 * z.imag()};
    }
    return {s[0] * std::exp(lambda[0].real() * t),
            k == 2 ? s[1] * std::exp(lambda[1].real() * t) : 0.0};
}

InvariantManifold::Vec3 InvariantManifold::field(const Vec3& u) const
{
    Vec3 f{};
    for (int c = 0; c < 3; ++c)
    {
        for (int i = 0; i < 3; ++i)
        {
            f[c] += A_[c][i] * u[i];
            for (int j = 0; j < 3; ++j)
                f[c] += Q_[c][i][j] * u[i] * u[j];
        }
    }
    return f;
}

double InvariantManifold::defect(const Param& s) const
{
    if (!valid())
        return 0.0;
    cd w1, w2;
    vars(s, w1, w2);
    std::vector<cd> pw2(n + 1);
    pw2[0] = 1.0;
    for (int j = 1; j <= n; ++j)
        pw2[j] = pw2[j - 1] * w2;

    // d/dt K(e^{Lambda t} s) at t = 0 = sum (m lambda_0 + j lambda_1) p_mj w^(m,j)
    CVec3 dK{};
    cd pw1 = 1.0;
    for (int m = 0; m <= n; ++m, pw1 *= w1)
        for (int j = 0; j + m <= n && (k == 2 || j == 0); ++j)
        {
            cd w = ((double)m * lambda[0] + (double)j * lambda[1]) * pw1 * pw2[j];
            const CVec3& c = p[idx(m, j)];
            for (int i = 0; i < 3; ++i)
                dK[i] += c[i] * w;
        }
    Vec3 f = field(eval(s));
    Vec3 diff = {f[0] - dK[0].real(), f[1] - dK[1].real(), f[2] - dK[2].real()};
    return length(diff);
}

double InvariantManifold::maxRelDefect(double r, int samples) const
{
    if (!valid())
        return 0.0;
    double worst = 0.0;
    int count = k == 1 ? 2 : std::max(samples, 4);
    for (int i = 0; i < count; ++i)
    {
        Vec3 theta;
        if (k == 1)
        {
            double sgn = i == 0 ? r : -r;
            theta = {O[0][0] * sgn, O[0][1] * sgn, O[0][2] * sgn};
        }
        else
        {
            double ph = 6.283185307179586 * i / count;
            double c = r * std::cos(ph), sn = r * std::sin(ph);
            for (int a = 0; a < 3; ++a)
                theta[a] = c * O[0][a] + sn * O[1][a];
        }
        Param s = toParam(theta);
        double fl = length(field(eval(s)));
        worst = std::max(worst, defect(s) / std::max(fl, 1e-300));
    }
    return worst;
}

} // namespace engine
