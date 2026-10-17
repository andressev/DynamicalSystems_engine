#include "LorenzManifoldSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/ConjugacyMap.h>
#include <engine/InvariantManifold.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>

#include <imgui.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <execution>
#include <fstream>
#include <glm.hpp>
#include <gtc/constants.hpp>
#include <gtc/matrix_transform.hpp>
#include <limits>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace {

using glm::dvec3;

// Tunable parameters for the manifold build; changing any of these
// (except colorMode) triggers a full recompute + VBO rebuild.
struct Params {
    double sigma = 10.0;
    double rho = 28.0;
    double beta = 8.0 / 3.0;
    int order = 10;
    int N = 15;     // rings
    int M = 128;    // points per ring
    // At order 10 the Taylor patch stays within ~3 significant digits of an
    // order-20 truncation out past rho=10 (checked numerically), so N*dRho
    // can safely reach ~6 without visibly diverging; 0.05 left the whole
    // patch a fraction of a unit across.
    double dRho = 0.4;
    bool equalArc = true;
    // Adaptive spacing: after the base M-point ring is built, bisect any edge
    // whose chord length exceeds maxGap, inserting an exact point from K at
    // the midpoint psi, until every edge is under the cap (or the safety cap
    // maxPointsPerRing is hit). Lets rings at large rho — which need many more
    // points per turn than small ones — grow past M without wasting points on
    // the inner rings.
    bool adaptiveSpacing = true;
    double maxGap = 0.3;
    static constexpr int maxPointsPerRing = 4000;
};

// Phase 2: flow-based growth of the manifold past K's convergence disk. See
// obsidian_twin/math/lorenz-stable-manifold.md for the shooting + curvature
// derivation this ports. Disabled by default so the Taylor-only Phase 1 path
// stands alone (also keeps the default rebuild cheap: RK4 shooting is far
// more expensive than evaluating K).
struct GrowthParams {
    bool enabled = true; // TEMP: diagnostic run
    int levels = 40; // TEMP: diagnostic run
    double deltaInit = 0.4;   // initial radial step Delta_0 for the first grown level
    double deltaMin = 1e-4;   // give up growing further once Delta drops below this
    double deltaMax = 5.0;
    double tauInit = 0.05;    // initial guess for the backward-flow time tau
    // Curvature acceptance/step-control thresholds (see growLevel). alpha_r is
    // reported in DEGREES (curvatureAngle converts), so (Delta*alpha) is also
    // in Delta-times-degrees units -- the dAlpha thresholds are scaled by the
    // same 180/pi factor as alphaMin/Max to stay consistent.
    double alphaMin = 17.19;
    double alphaMax = 22.92;
    double dAlphaMin = 5.73;   // (Delta_i * alpha_r)_min
    double dAlphaMax = 57.30;  // (Delta_i * alpha_r)_max
    static constexpr int maxHalvings = 12;
};

// A ring sample keeps the psi that produced it so adaptive refinement (and the
// solid-surface stitcher) can bisect/match exactly through K rather than lerp
// between two nearby points. Grown (Phase 2) rings reuse the parent ring's psi
// index-for-index, since they keep the same point count.
struct RingSample {
    double psi;
    dvec3 pt;
};

// Coefficients p[c][m][n] of K_c(theta1,theta2) = sum p[c][m][n] theta1^m theta2^n,
// c in {x=0,y=1,z=2}. See obsidian_twin/math/lorenz-stable-manifold.md for the
// parametrization-method derivation this ports.
struct Manifold {
    int order = 0;
    std::vector<std::vector<std::vector<double>>> p; // [3][order+1][order+1]
    double s2 = 1.0; // |ev2|, isotropic-coordinate scale for theta2
    std::vector<std::vector<RingSample>> rings; // [N][M-ish, adaptive spacing varies it]
    double zMin = 0.0, zMax = 0.0;
    int taylorRingCount = 0; // rings[0..taylorRingCount) came from K; the rest from growth
    double lastDelta = 0.0;  // Delta_i accepted for the final grown level (diagnostics)
};

// Cauchy product (2-D convolution) of channels c1,c2 up to (m,n) inclusive.
double S(const std::vector<std::vector<std::vector<double>>>& p, int c1, int c2, int m, int n)
{
    double sum = 0.0;
    for (int i = 0; i <= m; ++i)
        for (int j = 0; j <= n; ++j)
            sum += p[c1][i][j] * p[c2][m - i][n - j];
    return sum;
}

// Solve a 3x3 linear system via Cramer's rule.
std::array<double, 3> solve3x3(const double A[3][3], const std::array<double, 3>& b)
{
    auto det3 = [](const double m[3][3]) {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    };
    double D = det3(A);

    std::array<double, 3> x{};
    for (int col = 0; col < 3; ++col) {
        double Ac[3][3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Ac[r][c] = (c == col) ? b[r] : A[r][c];
        x[col] = det3(Ac) / D;
    }
    return x;
}

// Evaluate K_c(theta1,theta2) for all three channels at once.
dvec3 evalK(const std::vector<std::vector<std::vector<double>>>& p, int order,
            double theta1, double theta2)
{
    std::vector<double> t1pow(order + 1), t2pow(order + 1);
    t1pow[0] = t2pow[0] = 1.0;
    for (int i = 1; i <= order; ++i) {
        t1pow[i] = t1pow[i - 1] * theta1;
        t2pow[i] = t2pow[i - 1] * theta2;
    }

    dvec3 out(0.0);
    for (int m = 0; m <= order; ++m)
        for (int n = 0; n <= order - m; ++n) {
            double basis = t1pow[m] * t2pow[n];
            out.x += p[0][m][n] * basis;
            out.y += p[1][m][n] * basis;
            out.z += p[2][m][n] * basis;
        }
    return out;
}

dvec3 pointAt(const std::vector<std::vector<std::vector<double>>>& p, int order,
              double rho, double s2, double psi)
{
    double a = rho * std::cos(psi);
    double b = rho * std::sin(psi);
    return evalK(p, order, a, b / s2);
}

std::vector<RingSample> uniformPsiRing(const std::vector<std::vector<std::vector<double>>>& p,
                                        int order, double rho, double s2, int M)
{
    std::vector<RingSample> ring(M);
    for (int k = 0; k < M; ++k) {
        double psi = 2.0 * glm::pi<double>() * k / M;
        ring[k] = { psi, pointAt(p, order, rho, s2, psi) };
    }
    return ring;
}

// Step 5 (optional): resample a closed ring to equal Euclidean arc length.
// Oversamples psi at 8x the target point count, walks the chord lengths, then
// linearly interpolates back down to `M` equally-spaced-by-arc-length points.
std::vector<RingSample> equalArcResample(const std::vector<std::vector<std::vector<double>>>& p,
                                          int order, double rho, double s2, int M)
{
    const int oversample = M * 8;
    std::vector<RingSample> fine(oversample);
    for (int k = 0; k < oversample; ++k) {
        double psi = 2.0 * glm::pi<double>() * k / oversample;
        fine[k] = { psi, pointAt(p, order, rho, s2, psi) };
    }

    std::vector<double> cum(oversample + 1, 0.0);
    for (int k = 0; k < oversample; ++k) {
        const dvec3& next = fine[(k + 1) % oversample].pt;
        cum[k + 1] = cum[k] + glm::length(next - fine[k].pt);
    }
    double total = cum[oversample];

    std::vector<RingSample> out(M);
    int src = 0;
    for (int k = 0; k < M; ++k) {
        double target = total * k / M;
        while (src < oversample && cum[src + 1] < target)
            ++src;
        double segLen = cum[src + 1] - cum[src];
        double t = segLen > 1e-12 ? (target - cum[src]) / segLen : 0.0;
        const RingSample& a = fine[src];
        const RingSample& b = fine[(src + 1) % oversample];
        out[k] = { a.psi + t * (b.psi - a.psi), a.pt + t * (b.pt - a.pt) };
    }
    return out;
}

// Adaptive spacing: bisect any ring edge whose chord length exceeds maxGap,
// inserting an exact K(psi) point at the midpoint psi, repeating until every
// edge is under the cap or the ring hits maxPoints.
std::vector<RingSample> refineRingSpacing(const std::vector<std::vector<std::vector<double>>>& p,
                                           int order, double rho, double s2,
                                           std::vector<RingSample> ring, double maxGap,
                                           int maxPoints)
{
    bool changed = true;
    while (changed && (int)ring.size() < maxPoints) {
        changed = false;
        std::vector<RingSample> next;
        next.reserve(ring.size() * 2);
        for (size_t k = 0; k < ring.size(); ++k) {
            const RingSample& a = ring[k];
            const RingSample& b = ring[(k + 1) % ring.size()];
            next.push_back(a);
            if (glm::length(b.pt - a.pt) > maxGap && (int)next.size() < maxPoints) {
                double psiB = (k + 1 == ring.size()) ? b.psi + 2.0 * glm::pi<double>() : b.psi;
                double midPsi = a.psi + (psiB - a.psi) * 0.5;
                next.push_back({ midPsi, pointAt(p, order, rho, s2, midPsi) });
                changed = true;
            }
        }
        ring = std::move(next);
    }
    return ring;
}

// ---- Phase 2: flow-based growth past the Taylor disk -----------------------
//
// Once K's series stops being trustworthy (rings crossing/diverging), further
// rings are grown by integrating the REVERSED Lorenz field backward from the
// current outermost ring (backward time pushes points along the stable
// manifold away from the origin, since forward time contracts to it).

dvec3 lorenzField(const dvec3& x, double sigma, double rho, double beta)
{
    return dvec3(sigma * (x.y - x.x), rho * x.x - x.y - x.x * x.z, x.x * x.y - beta * x.z);
}

dvec3 rk4Step(const dvec3& x, double h, double sigma, double rho, double beta)
{
    // Reversed field: integrating -f forward in time walks backward along the
    // Lorenz flow, which is what expands the stable manifold outward.
    auto f = [&](const dvec3& v) { return -lorenzField(v, sigma, rho, beta); };
    dvec3 k1 = f(x);
    dvec3 k2 = f(x + 0.5 * h * k1);
    dvec3 k3 = f(x + 0.5 * h * k2);
    dvec3 k4 = f(x + h * k3);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// Fixed-step RK4 over [0, tau], step capped at ~0.01 so the integration stays
// accurate even when Newton (below) asks for a comparatively large tau.
dvec3 flowReversed(const dvec3& x0, double tau, double sigma, double rho, double beta)
{
    if (tau <= 0.0)
        return x0;
    int steps = std::max(4, (int)std::ceil(tau / 0.01));
    double h = tau / steps;
    dvec3 x = x0;
    for (int i = 0; i < steps; ++i)
        x = rk4Step(x, h, sigma, rho, beta);
    return x;
}

struct ShotResult {
    dvec3 q;       // shooting start point gamma(s) on the previous ring
    dvec3 b;       // phi^{-tau}(q), the candidate point in the new ring
    double tau = 0.0;
    bool ok = false;
};

// For point r on ring i (with ring-neighbors rMinus, rPlus), find (s, tau)
// such that b = phi^{-tau}(gamma(s)) lands on the plane through r normal to
// the ring tangent, at exactly distance Delta from r. gamma(s) = r + s*that
// locally interpolates along the ring near r, letting Newton slide the
// shooting start point tangentially to compensate for drift.
// 2x2 Newton, finite-difference Jacobian, warm-started in tau (per the
// original Phase-2 sketch in obsidian_twin/math/lorenz-stable-manifold.md).
ShotResult shootPoint(const dvec3& r, const dvec3& rMinus, const dvec3& rPlus, double Delta,
                      double sigma, double rho, double beta, double tauInit)
{
    dvec3 that = glm::normalize(rPlus - rMinus);

    auto gamma = [&](double s) { return r + s * that; };
    auto g = [&](double s, double tau, dvec3& bOut) {
        dvec3 q = gamma(s);
        dvec3 b = flowReversed(q, tau, sigma, rho, beta);
        dvec3 d = b - r;
        bOut = b;
        return glm::dvec2(glm::dot(that, d), glm::length(d) - Delta);
    };

    double s = 0.0, tau = std::max(tauInit, 1e-6);
    ShotResult res;

    // TEMP: dump the first few shots' Newton path (atomic: shots run in parallel)
    static std::atomic<int> debugBudget{ 0 };
    bool debug = debugBudget.load(std::memory_order_relaxed) > 0 && debugBudget.fetch_sub(1) > 0;

    for (int iter = 0; iter < 25; ++iter) {
        dvec3 b;
        glm::dvec2 v = g(s, tau, b);
        if (debug)
            fprintf(stderr, "  [shoot] iter=%d s=%.6f tau=%.6f g1=%.6f g2=%.6f |b-r|=%.6f\n",
                    iter, s, tau, v.x, v.y, glm::length(b - r));
        if (!std::isfinite(v.x) || !std::isfinite(v.y)) {
            if (debug) fprintf(stderr, "  [shoot] non-finite, aborting\n");
            return res;
        }
        if (std::abs(v.x) < 1e-9 && std::abs(v.y) < std::max(1e-9, 1e-6 * Delta)) {
            res.ok = true;
            res.q = gamma(s);
            res.b = b;
            res.tau = tau;
            return res;
        }

        double epsS = 1e-6;
        double epsTau = std::max(1e-6, tau * 1e-6);
        dvec3 bS;
        glm::dvec2 vS = g(s + epsS, tau, bS);
        dvec3 bT;
        glm::dvec2 vT = g(s, tau + epsTau, bT);

        double J[2][2] = {
            { (vS.x - v.x) / epsS, (vT.x - v.x) / epsTau },
            { (vS.y - v.y) / epsS, (vT.y - v.y) / epsTau },
        };
        double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
        if (std::abs(det) < 1e-14) {
            if (debug) fprintf(stderr, "  [shoot] singular Jacobian, aborting\n");
            return res;
        }

        double ds = (J[1][1] * (-v.x) - J[0][1] * (-v.y)) / det;
        double dtau = (J[0][0] * (-v.y) - J[1][0] * (-v.x)) / det;
        s += ds;
        tau += dtau;
        if (!std::isfinite(s) || !std::isfinite(tau) || tau <= 0.0) {
            if (debug) fprintf(stderr, "  [shoot] s/tau blew up (ds=%.6f dtau=%.6f), aborting\n", ds, dtau);
            return res;
        }
    }
    if (debug) fprintf(stderr, "  [shoot] ran out of iterations\n");
    return res; // did not converge in time
}

// alpha_r: the turning angle of the actual growth path p_r -> r -> b_r, where
// p_r in M_{i-1} is r's own predecessor (the point it was shot from one level
// back). Compares the previous step's direction (r - p_r) to this step's
// direction (b_r - r): both point "forward" along growth, so a straight
// front gives alpha_r ~ 0 and any bend increases it. Using real growth
// history here (rather than a point mechanically tied to the same Newton
// system that produced b_r) is what makes this a real curvature signal.
double curvatureAngle(const dvec3& p, const dvec3& r, const dvec3& b)
{
    dvec3 v1 = r - p, v2 = b - r;
    double n1 = glm::length(v1), n2 = glm::length(v2);
    if (n1 < 1e-12 || n2 < 1e-12)
        return 0.0;
    double c = glm::clamp(glm::dot(v1, v2) / (n1 * n2), -1.0, 1.0);
    return glm::degrees(std::acos(c));
}

// Grow one new ring from `prev` (M_i), given `pPrev` (M_{i-1}, index-aligned
// with `prev` so pPrev[k] is prev[k]'s own predecessor), adapting Delta by
// curvature:
//  - shoot every point at the current Delta;
//  - if any shot fails, or any alpha_r fails the (alpha_max, Delta*alpha_max)
//    pair, halve Delta and redo the whole ring;
//  - once accepted, always grow Delta for the next level (see below).
// Every shot only reads ring i, so the whole ring is shot in parallel, one task per
// point. tauPerDelta[k] (in/out, index-aligned with prev) is the tau/Delta point k
// converged to one level back -- its own history, so no ordering between points is
// needed. Stored as a ratio because tau ~ Delta/|f| scales with Delta: the warm start
// stays good across halvings and doublings.
bool growLevel(const std::vector<RingSample>& prev, const std::vector<dvec3>& pPrev,
               double sigma, double rho, double beta,
               double& Delta, const GrowthParams& gp, std::vector<double>& tauPerDelta,
               std::vector<RingSample>& outRing)
{
    const double alphaMax = gp.alphaMax;
    size_t n = prev.size();
    if (n < 3)
        return false;

    std::vector<size_t> pointIndex(n);
    std::iota(pointIndex.begin(), pointIndex.end(), size_t(0));

    for (int halving = 0; halving <= GrowthParams::maxHalvings; ++halving) {
        std::vector<RingSample> candidate(n);
        std::vector<double> alphas(n);
        std::vector<double> candTau(n);
        bool atFloor = Delta <= gp.deltaMin; // Delta already clamped to the floor below

        // n = no failure; otherwise some failing k (the whole ring is redone anyway,
        // so the other tasks bail out as soon as one shot fails)
        std::atomic<size_t> failK{ n };
        std::for_each(std::execution::par, pointIndex.begin(), pointIndex.end(), [&](size_t k) {
            if (failK.load(std::memory_order_relaxed) < n)
                return;
            const dvec3& r = prev[k].pt;
            const dvec3& rMinus = prev[(k + n - 1) % n].pt;
            const dvec3& rPlus = prev[(k + 1) % n].pt;
            ShotResult sr = shootPoint(r, rMinus, rPlus, Delta, sigma, rho, beta,
                                       tauPerDelta[k] * Delta);
            if (!sr.ok) {
                failK.store(k, std::memory_order_relaxed);
                return;
            }
            candTau[k] = sr.tau / Delta;
            alphas[k] = curvatureAngle(pPrev[k], r, sr.b);
            candidate[k] = { prev[k].psi, sr.b };
        });

        if (failK.load() < n) {
            fprintf(stderr, "[growLevel] halving=%d Delta=%.6f: shootPoint FAILED at k=%zu\n",
                    halving, Delta, failK.load());
            if (atFloor)
                return false; // can't even shoot at the floor Delta -- nothing to fall back to
            Delta = std::max(Delta * 0.5, gp.deltaMin);
            continue;
        }

        double maxAlpha = *std::max_element(alphas.begin(), alphas.end());
        double maxDA = Delta * maxAlpha;
        bool withinMax = true;
        for (double a : alphas)
            if (!(a < alphaMax && Delta * a < gp.dAlphaMax)) { withinMax = false; break; }

        // Delta has hit its floor: accept whatever curvature we got rather than
        // rejecting forever -- deltaMin is the resolution we're willing to live
        // with, so refusing to ever place a point there would just stall growth.
        bool accept = withinMax || atFloor;

        fprintf(stderr, "[growLevel] halving=%d Delta=%.6f: maxAlpha=%.4f deg, Delta*maxAlpha=%.4f -> %s\n",
                halving, Delta, maxAlpha, maxDA,
                accept ? (withinMax ? "OK" : "forced (at Delta floor)") : "reject");

        if (!accept) {
            Delta = std::max(Delta * 0.5, gp.deltaMin);
            continue;
        }

        // Accepted: this Delta and these b_r become ring i+1. Always grow the
        // step for the next level (clamped at deltaMax) rather than only
        // doubling when alphaMin/dAlphaMin also clear -- let the halving loop
        // above pull Delta back down again next level if it was too greedy.
        outRing = std::move(candidate);
        tauPerDelta = std::move(candTau);

        Delta = std::min(Delta * 2.0, gp.deltaMax);
        return true;
    }
    return false; // curvature never settled even at the smallest Delta tried
}

void growManifold(Manifold& man, const Params& params, const GrowthParams& gp)
{
    man.taylorRingCount = (int)man.rings.size();
    man.lastDelta = gp.deltaInit;
    if (!gp.enabled || man.rings.empty())
        return;

    double Delta = glm::clamp(gp.deltaInit, gp.deltaMin, gp.deltaMax);
    std::vector<RingSample> current = man.rings.back();
    // per-point warm start as tau/Delta; tauInit is the guess at deltaInit
    std::vector<double> tauPerDelta(current.size(), gp.tauInit / std::max(gp.deltaInit, 1e-12));

    // Predecessor positions for level 0 (p_r for each r in the last Taylor
    // ring): evaluated exactly through K at the same psi, one ring further
    // in. Taylor rings can have different point counts under adaptive
    // spacing, so matching by index into the stored ring N-1 could be wrong
    // -- re-evaluating K at r's own psi sidesteps that entirely.
    double rhoPrevTaylor = std::max(0.0, (params.N - 1) * params.dRho);
    std::vector<dvec3> pPrev(current.size());
    for (size_t k = 0; k < current.size(); ++k)
        pPrev[k] = pointAt(man.p, params.order, rhoPrevTaylor, man.s2, current[k].psi);

    for (int lvl = 0; lvl < gp.levels; ++lvl) {
        std::vector<RingSample> next;
        bool ok = growLevel(current, pPrev, params.sigma, params.rho, params.beta, Delta, gp,
                             tauPerDelta, next);
        fprintf(stderr, "[growth] level %d: %s Delta=%.6f max tau/Delta=%.6f\n",
                lvl, ok ? "accepted" : "FAILED", Delta,
                *std::max_element(tauPerDelta.begin(), tauPerDelta.end()));
        fflush(stderr);
        if (!ok)
            break;

        for (const RingSample& s : next) {
            man.zMin = std::min(man.zMin, s.pt.z);
            man.zMax = std::max(man.zMax, s.pt.z);
        }
        man.rings.push_back(next);
        // Next level's predecessor is this level's ring: same point count
        // and psi (growLevel always preserves both), so the index-aligned
        // correspondence is exact -- no re-evaluation needed past level 0.
        for (size_t k = 0; k < current.size(); ++k)
            pPrev[k] = current[k].pt;
        current = std::move(next);
        man.lastDelta = Delta;

        if (Delta < gp.deltaMin)
            break;
    }
}

Manifold buildManifold(const Params& params, const GrowthParams& gp)
{
    Manifold man;
    man.order = params.order;
    man.p.assign(3, std::vector<std::vector<double>>(
                         params.order + 1, std::vector<double>(params.order + 1, 0.0)));

    // Eigendata of the Jacobian at the origin (parametrization method setup —
    // see obsidian_twin/math/lorenz-stable-manifold.md).
    double root = std::sqrt((params.sigma + 1.0) * (params.sigma + 1.0)
                             + 4.0 * params.sigma * (params.rho - 1.0));
    double l1 = -params.beta;
    double l2 = (-(params.sigma + 1.0) - root) / 2.0;
    dvec3 ev1(0.0, 0.0, 1.0);
    dvec3 ev2(params.sigma / (params.sigma + l2), 1.0, 0.0);
    man.s2 = glm::length(ev2);

    double A[3][3] = {
        { -params.sigma, params.sigma, 0.0 },
        { params.rho, -1.0, 0.0 },
        { 0.0, 0.0, -params.beta },
    };

    man.p[0][1][0] = ev1.x; man.p[1][1][0] = ev1.y; man.p[2][1][0] = ev1.z;
    man.p[0][0][1] = ev2.x; man.p[1][0][1] = ev2.y; man.p[2][0][1] = ev2.z;

    for (int d = 2; d <= params.order; ++d) {
        for (int m = 0; m <= d; ++m) {
            int n = d - m;
            double mu = m * l1 + n * l2;
            double Mmat[3][3] = {
                { A[0][0] - mu, A[0][1], A[0][2] },
                { A[1][0], A[1][1] - mu, A[1][2] },
                { A[2][0], A[2][1], A[2][2] - mu },
            };
            std::array<double, 3> b = {
                0.0,
                S(man.p, 0, 2, m, n),
                -S(man.p, 0, 1, m, n),
            };
            std::array<double, 3> x = solve3x3(Mmat, b);
            man.p[0][m][n] = x[0];
            man.p[1][m][n] = x[1];
            man.p[2][m][n] = x[2];
        }
    }

    // Each Taylor ring only reads the (finished) coefficients, so the rings are built
    // in parallel, one task per ring; z-range is a serial pass afterwards.
    man.rings.resize(params.N);
    std::vector<int> ringIndex(params.N);
    std::iota(ringIndex.begin(), ringIndex.end(), 1);
    std::for_each(std::execution::par, ringIndex.begin(), ringIndex.end(), [&](int i) {
        double rho_i = i * params.dRho;

        std::vector<RingSample> samples = params.equalArc
            ? equalArcResample(man.p, params.order, rho_i, man.s2, params.M)
            : uniformPsiRing(man.p, params.order, rho_i, man.s2, params.M);

        if (params.adaptiveSpacing)
            samples = refineRingSpacing(man.p, params.order, rho_i, man.s2, std::move(samples),
                                        params.maxGap, Params::maxPointsPerRing);

        man.rings[i - 1] = std::move(samples);
    });

    man.zMin = std::numeric_limits<double>::max();
    man.zMax = std::numeric_limits<double>::lowest();
    for (const auto& ring : man.rings)
        for (const RingSample& s : ring) {
            man.zMin = std::min(man.zMin, s.pt.z);
            man.zMax = std::max(man.zMax, s.pt.z);
        }
    if (man.zMax <= man.zMin)
        man.zMax = man.zMin + 1.0;

    growManifold(man, params, gp);

    return man;
}

// A shader file with the line containing `marker` replaced by `insert` (how the
// ConjugacyMap GLSL gets spliced into this sim's vertex shader).
std::string loadWithInsert(const char* path, const char* marker, const std::string& insert)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    std::string src = ss.str();
    size_t at = *marker ? src.find(marker) : std::string::npos;
    if (at != std::string::npos) {
        size_t lineStart = src.rfind('\n', at);
        size_t lineEnd = src.find('\n', at);
        lineStart = lineStart == std::string::npos ? 0 : lineStart + 1;
        src.replace(lineStart, (lineEnd == std::string::npos ? src.size() : lineEnd) - lineStart,
                    insert);
    }
    return src;
}

// Red (nearest the origin, t=0) -> blue (the furthest-out geodesic ring, t=1).
// `scale` in (0,1] moves where full blue is reached: t is rescaled by 1/scale
// before clamping, so a smaller scale saturates to blue sooner (leaving more
// of the ring range visually "maxed out" blue instead of stretching the red-
// to-blue transition across the whole manifold).
glm::vec3 gradient(double t, double scale)
{
    scale = glm::clamp(scale, 0.01, 1.0);
    t = glm::clamp(t / scale, 0.0, 1.0);
    glm::dvec3 red(0.85, 0.08, 0.08);
    glm::dvec3 blue(0.05, 0.15, 0.95);
    return glm::vec3(glm::mix(red, blue, t));
}

// Alternate each ring between a darker and lighter version of its gradient
// color so consecutive bands stay visually distinct even where the gradient
// itself changes slowly.
glm::vec3 applyBanding(glm::vec3 color, size_t ringIndex, bool enabled, float contrast)
{
    if (!enabled)
        return color;
    float factor = (ringIndex % 2 == 0) ? (1.0f - contrast) : (1.0f + contrast);
    return glm::clamp(color * factor, glm::vec3(0.0f), glm::vec3(1.0f));
}

// Lorenz stable-manifold viewer: renders geodesic level-set rings of the 2-D
// stable manifold W^s(0). Phase 1 computes rings straight from the Taylor map
// K (parametrization method); Phase 2, when enabled, keeps growing past K's
// convergence disk by integrating the reversed Lorenz field with a curvature-
// adaptive step (see obsidian_twin/math/lorenz-stable-manifold.md).
class LorenzManifoldSim : public engine::Sim {
public:
    const char* name() const override { return "lorenz_manifold"; }

    void init(engine::App& app) override
    {
        this->app = &app;

        app.camera().inputEnabled = true;
        app.camera().orbitRadius = 14.0f;
        app.camera().orbitRadiusMin = 0.5f;
        app.camera().orbitSlowZone = 0.0f;

        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(
            SIMS_DIR "lorenz_manifold/shaders/vertex.glsl",
            SIMS_DIR "lorenz_manifold/shaders/fragment.glsl");

        hmShader = engine::Shader::fromSources(
            loadWithInsert(SIMS_DIR "lorenz_manifold/shaders/conjugacy_vertex.glsl",
                           "@CONJUGACY_MAP", engine::ConjugacyMap::glsl()),
            loadWithInsert(SIMS_DIR "lorenz_manifold/shaders/conjugacy_fragment.glsl", "", ""));
        hmShader.bindUniformBlock(engine::ConjugacyMap::kBlockName,
                                  engine::ConjugacyMap::kBinding);
        buildPreviewMeshes();

        rebuild();
    }

    void update(float dt) override { (void)dt; }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix();
        glm::mat4 model = glm::scale(glm::mat4(1.0f), glm::vec3(modelScale));

        shader.use();
        shader.setMat4("model", model);
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setInt("u_colorMode", colorByHeight ? 1 : 0);

        if (showSurface && surfaceMesh)
            surfaceMesh->draw(GL_TRIANGLES);

        if (showPoints && pointsMesh)
            pointsMesh->draw(GL_POINTS);

        if (showRings) {
            glLineWidth(1.5f);
            for (auto& ring : ringMeshes)
                ring->draw(GL_LINE_LOOP);
        }

        if ((showHmDisk || showHmBall) && hm.valid())
            renderConjugacyPreview(camera, model, view, projection);
    }

    void drawUI() override
    {
        bool dirty = false;
        ImGui::TextUnformatted("Lorenz stable manifold W^s(0)");

        dirty |= ImGui::DragScalar("sigma", ImGuiDataType_Double, &params.sigma, 0.1f);
        dirty |= ImGui::DragScalar("rho", ImGuiDataType_Double, &params.rho, 0.1f);
        dirty |= ImGui::DragScalar("beta", ImGuiDataType_Double, &params.beta, 0.01f);
        dirty |= ImGui::SliderInt("order", &params.order, 2, 16);
        dirty |= ImGui::SliderInt("rings (N)", &params.N, 1, 200);
        dirty |= ImGui::SliderInt("points/ring (M)", &params.M, 8, 512);
        dirty |= ImGui::DragScalar("dRho", ImGuiDataType_Double, &params.dRho, 0.001f,
                                   nullptr, nullptr, "%.4f");
        dirty |= ImGui::Checkbox("equal-arc redistribution", &params.equalArc);
        dirty |= ImGui::Checkbox("adaptive spacing", &params.adaptiveSpacing);
        if (params.adaptiveSpacing) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            dirty |= ImGui::DragScalar("max gap##maxGap", ImGuiDataType_Double, &params.maxGap,
                                       0.005f, nullptr, nullptr, "%.3f");
        }

        params.maxGap = std::max(params.maxGap, 1e-3);

        ImGui::Separator();
        ImGui::TextUnformatted("Phase 2: flow growth past the Taylor disk");
        dirty |= ImGui::Checkbox("enabled##growth", &growth.enabled);
        if (growth.enabled) {
            dirty |= ImGui::SliderInt("levels##growth", &growth.levels, 1, 60);
            dirty |= ImGui::DragScalar("Delta init", ImGuiDataType_Double, &growth.deltaInit,
                                       0.005f, nullptr, nullptr, "%.4f");
            dirty |= ImGui::DragScalar("tau init", ImGuiDataType_Double, &growth.tauInit,
                                       0.001f, nullptr, nullptr, "%.4f");
            dirty |= ImGui::DragScalar("alpha max (deg)", ImGuiDataType_Double, &growth.alphaMax,
                                       0.05f, nullptr, nullptr, "%.2f");
            dirty |= ImGui::DragScalar("alpha min (deg)", ImGuiDataType_Double, &growth.alphaMin,
                                       0.05f, nullptr, nullptr, "%.2f");
            dirty |= ImGui::DragScalar("(Delta*alpha) max", ImGuiDataType_Double, &growth.dAlphaMax,
                                       0.01f, nullptr, nullptr, "%.4f");
            dirty |= ImGui::DragScalar("(Delta*alpha) min", ImGuiDataType_Double, &growth.dAlphaMin,
                                       0.005f, nullptr, nullptr, "%.4f");

            growth.deltaInit = std::max(growth.deltaInit, growth.deltaMin);
            growth.alphaMin = std::min(growth.alphaMin, growth.alphaMax);
            growth.dAlphaMin = std::min(growth.dAlphaMin, growth.dAlphaMax);

            ImGui::TextDisabled("grown %d ring(s), final Delta = %.5f",
                                 std::max(0, (int)ringMeshes.size() - lastTaylorRingCount),
                                 lastDelta);
        }

        if (dirty)
            rebuild();

        ImGui::Separator();
        ImGui::DragFloat("model scale", &modelScale, 0.01f, 0.01f, 5.0f);
        ImGui::Checkbox("show surface", &showSurface);
        ImGui::SameLine();
        ImGui::Checkbox("show points", &showPoints);
        ImGui::SameLine();
        ImGui::Checkbox("show rings", &showRings);
        ImGui::Checkbox("color by height (z)", &colorByHeight);

        bool colorDirty = false;
        colorDirty |= ImGui::SliderFloat("gradient scale (blue max)", &gradientScale, 0.05f, 1.0f);
        colorDirty |= ImGui::Checkbox("band colors (dark/light per ring)", &bandColors);
        if (bandColors) {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            colorDirty |= ImGui::SliderFloat("band contrast", &bandContrast, 0.0f, 0.9f);
        }
        if (colorDirty && !dirty)
            rebuildMeshes();

        ImGui::TextDisabled("last rebuild %.1f ms", rebuildMs);
        if (!buildError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", buildError.c_str());

        drawConjugacyUI();

        ImGui::TextWrapped(
            "Rings C_i are approximate geodesic circles of W^s(0) at radius "
            "rho_i = i*dRho in isotropic (theta1,theta2) coordinates, evaluated "
            "straight from the order-%d Taylor map K. With flow growth enabled, "
            "further rings past the Taylor disk are grown by backward-integrating "
            "the reversed field with a curvature-adaptive step.", params.order);
    }

private:
    // ---- conjugacy preview: h~^{-1} = K_s + K_u evaluated per vertex on the GPU ---------
    // A try-out of engine::ConjugacyMap at the origin before the Lorenz equilibrium sim
    // exists (Hartman-Grobman track, roadmap). The E^s disk pushed through h~^{-1} on the
    // GPU must lie exactly on this sim's CPU-built surface — a visual check of the GLSL.

    void rebuildConjugacy()
    {
        hmError.clear();
        double A[3][3] = {
            { -params.sigma, params.sigma, 0.0 },
            { params.rho, -1.0, 0.0 },
            { 0.0, 0.0, -params.beta },
        };
        engine::InvariantManifold::Quadratic Q{};
        Q[1][0][2] = -1.0;   // y' gets -u_x u_z
        Q[2][0][1] = 1.0;    // z' gets  u_x u_y
        int order = std::min(params.order, engine::ConjugacyMap::kMaxOrder);
        engine::InvariantManifold ws, wu;
        bool ok = ws.build(A, Q, engine::InvariantManifold::Side::Stable, order)
               && wu.build(A, Q, engine::InvariantManifold::Side::Unstable, order)
               && hm.set(ws, wu);
        if (!ok)
            hmError = !ws.valid() ? ws.error() : !wu.valid() ? wu.error() : hm.error();
    }

    void buildPreviewMeshes()
    {
        // unit sphere (positions only are read) and a unit disk as rings + spokes
        sphereMesh = std::make_unique<engine::Mesh>(engine::Shapes::sphereVertices(96, 64, 1.0f),
                                                    engine::Shapes::sphereIndices(96, 64),
                                                    std::vector<int>{ 3, 3, 2 });
        std::vector<float> disk;
        const int rings = 10, spokes = 24, segs = 160;
        for (int i = 1; i <= rings; ++i) {
            float r = (float)i / rings;
            for (int k = 0; k < segs; ++k)
                for (int e = 0; e < 2; ++e) {
                    float a = 2.0f * glm::pi<float>() * (float)(k + e) / segs;
                    disk.insert(disk.end(), { r * std::cos(a), r * std::sin(a), 0.0f });
                }
        }
        for (int k = 0; k < spokes; ++k) {
            float a = 2.0f * glm::pi<float>() * (float)k / spokes;
            for (int j = 0; j < 20; ++j)
                for (int e = 0; e < 2; ++e) {
                    float r = (float)(j + e) / 20.0f;
                    disk.insert(disk.end(), { r * std::cos(a), r * std::sin(a), 0.0f });
                }
        }
        std::vector<unsigned int> idx(disk.size() / 3);
        std::iota(idx.begin(), idx.end(), 0u);
        diskMesh = std::make_unique<engine::Mesh>(disk, idx, std::vector<int>{ 3 });
    }

    void renderConjugacyPreview(const engine::Camera& camera, const glm::mat4& model,
                                const glm::mat4& view, const glm::mat4& projection)
    {
        hm.bind();
        hmShader.use();
        hmShader.setMat4("model", model);
        hmShader.setMat4("view", view);
        hmShader.setMat4("projection", projection);
        hmShader.setVec3("u_center", glm::vec3(0.0f));   // x* = origin
        hmShader.setFloat("u_radius", hmRadius);
        hmShader.setVec3("u_eye", camera.position);
        auto e0 = hm.stable().orthoBasis(0), e1 = hm.stable().orthoBasis(1);
        hmShader.setVec3("u_e0", glm::vec3((float)e0[0], (float)e0[1], (float)e0[2]));
        hmShader.setVec3("u_e1", glm::vec3((float)e1[0], (float)e1[1], (float)e1[2]));

        if (showHmDisk && hm.stable().dim() == 2) {
            hmShader.setInt("u_mode", 1);
            hmShader.setVec3("u_color", glm::vec3(1.0f));
            hmShader.setFloat("u_alpha", 0.9f);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glLineWidth(1.5f);
            diskMesh->draw(GL_LINES);
            glLineWidth(1.0f);
        }
        if (showHmBall) {
            hmShader.setInt("u_mode", 0);
            hmShader.setVec3("u_color", glm::vec3(0.55f, 0.75f, 1.0f));
            hmShader.setFloat("u_alpha", hmAlpha);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glDepthMask(GL_FALSE);
            sphereMesh->draw(GL_TRIANGLES);
            glDepthMask(GL_TRUE);
        }
        glDisable(GL_BLEND);
        shader.use();
    }

    void drawConjugacyUI()
    {
        ImGui::Separator();
        if (!ImGui::CollapsingHeader("Conjugacy preview (h^-1 on the GPU)"))
            return;
        ImGui::Checkbox("deformed ball V = h^-1(B_r)", &showHmBall);
        ImGui::Checkbox("E^s disk through h^-1 (must lie on the surface)", &showHmDisk);
        ImGui::SliderFloat("r##hm", &hmRadius, 0.1f, 12.0f, "%.2f");
        ImGui::SliderFloat("glass alpha", &hmAlpha, 0.0f, 0.6f);
        if (!hmError.empty()) {
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", hmError.c_str());
            return;
        }
        // how far the Taylor maps can be trusted at this r (relative invariance defect)
        ImGui::TextDisabled("defect at r: W^s %.1e, W^u %.1e   (min small divisor %.3f)",
                            hm.stable().maxRelDefect(hmRadius), hm.unstable().maxRelDefect(hmRadius),
                            std::min(hm.stable().minDivisor(), hm.unstable().minDivisor()));
        ImGui::TextWrapped("Exact on E^s and E^u (K_s, K_u); an interpolation in between. "
                           "Moving r costs nothing: only a uniform changes.");
    }

    void rebuild()
    {
        buildError.clear();
        if (params.order < 1 || params.N < 1 || params.M < 3) {
            buildError = "invalid parameters";
            return;
        }

        auto t0 = std::chrono::steady_clock::now();
        cachedManifold = buildManifold(params, growth);
        rebuildConjugacy();
        lastTaylorRingCount = cachedManifold.taylorRingCount;
        lastDelta = cachedManifold.lastDelta;

        rebuildMeshes();
        rebuildMs = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();
    }

    // Re-bakes vertex colors (gradient scale / banding / height) and rebuilds
    // the GL meshes from the cached manifold geometry, without recomputing K
    // or re-running the (much more expensive) Phase-2 shooting.
    void rebuildMeshes()
    {
        const Manifold& man = cachedManifold;
        size_t ringCount = man.rings.size();

        // Flat buffer of every ring vertex (pos + level color + height color),
        // shared by the points mesh, the per-ring line loops, and (with one
        // extra origin vertex appended below) the solid surface.
        // Offsets first (prefix sum over ring sizes), then every ring fills its own
        // slice of the buffer in parallel; the GL uploads below stay on this thread.
        std::vector<unsigned int> ringVertStart(ringCount);
        unsigned int vcount = 0;
        for (size_t i = 0; i < ringCount; ++i) {
            ringVertStart[i] = vcount;
            vcount += (unsigned int)man.rings[i].size();
        }
        std::vector<float> verts((size_t)vcount * 9);

        std::vector<size_t> ringIndex(ringCount);
        std::iota(ringIndex.begin(), ringIndex.end(), size_t(0));
        std::for_each(std::execution::par, ringIndex.begin(), ringIndex.end(), [&](size_t i) {
            glm::vec3 levelColor = gradient(double(i) / std::max<size_t>(1, ringCount - 1),
                                            gradientScale);
            levelColor = applyBanding(levelColor, i, bandColors, bandContrast);

            float* out = verts.data() + (size_t)ringVertStart[i] * 9;
            for (const RingSample& s : man.rings[i]) {
                glm::vec3 pos(s.pt);
                glm::vec3 heightColor = gradient((s.pt.z - man.zMin) / (man.zMax - man.zMin),
                                                 gradientScale);
                heightColor = applyBanding(heightColor, i, bandColors, bandContrast);

                for (float f : { pos.x, pos.y, pos.z,
                                 levelColor.r, levelColor.g, levelColor.b,
                                 heightColor.r, heightColor.g, heightColor.b })
                    *out++ = f;
            }
        });

        ringMeshes.clear();
        ringMeshes.reserve(ringCount);
        for (size_t i = 0; i < ringCount; ++i) {
            auto first = verts.begin() + (size_t)ringVertStart[i] * 9;
            std::vector<float> ringVerts(first, first + man.rings[i].size() * 9);
            std::vector<unsigned int> ringIdx(man.rings[i].size());
            std::iota(ringIdx.begin(), ringIdx.end(), 0u);
            ringMeshes.push_back(std::make_unique<engine::Mesh>(
                ringVerts, ringIdx, std::vector<int>{ 3, 3, 3 }));
        }

        std::vector<unsigned int> pointIdx(verts.size() / 9);
        for (unsigned int k = 0; k < pointIdx.size(); ++k)
            pointIdx[k] = k;
        pointsMesh = std::make_unique<engine::Mesh>(verts, pointIdx, std::vector<int>{ 3, 3, 3 });

        buildSurface(man, verts, ringVertStart, vcount);
    }

    // Solid fill between rings: a triangle strip stitched between each pair
    // of consecutive rings, matched by shared psi (so it stays correct when
    // adaptive spacing gives them different point counts), plus a fan closing
    // the tip at the origin (K(0,0) = 0 exactly).
    void buildSurface(const Manifold& man, std::vector<float> verts,
                      const std::vector<unsigned int>& ringVertStart, unsigned int vcount)
    {
        surfaceMesh.reset();
        size_t ringCount = man.rings.size();
        if (ringCount == 0)
            return;

        std::vector<unsigned int> surfIdx;

        unsigned int originIdx = vcount;
        {
            glm::vec3 levelColor = gradient(0.0, gradientScale);
            glm::vec3 heightColor = gradient((0.0 - man.zMin) / (man.zMax - man.zMin), gradientScale);
            for (float f : { 0.0f, 0.0f, 0.0f, levelColor.r, levelColor.g, levelColor.b,
                             heightColor.r, heightColor.g, heightColor.b })
                verts.push_back(f);

            const auto& ring0 = man.rings[0];
            size_t n0 = ring0.size();
            for (size_t k = 0; k < n0; ++k) {
                surfIdx.push_back(originIdx);
                surfIdx.push_back(ringVertStart[0] + (unsigned)k);
                surfIdx.push_back(ringVertStart[0] + (unsigned)((k + 1) % n0));
            }
        }

        // The strips between ring pairs are independent: each pair is stitched in
        // parallel into its own index list, then they are joined in ring order.
        std::vector<std::vector<unsigned int>> strips(ringCount - 1);
        std::vector<size_t> pairIndex(ringCount - 1);
        std::iota(pairIndex.begin(), pairIndex.end(), size_t(0));
        std::for_each(std::execution::par, pairIndex.begin(), pairIndex.end(), [&](size_t i) {
            const auto& A = man.rings[i];
            const auto& B = man.rings[i + 1];
            unsigned int baseA = ringVertStart[i], baseB = ringVertStart[i + 1];
            size_t nA = A.size(), nB = B.size();
            std::vector<unsigned int>& strip = strips[i];
            strip.reserve((nA + nB) * 3);
            size_t ia = 0, ib = 0;
            while (ia < nA || ib < nB) {
                double psiANext = (ia + 1 < nA) ? A[ia + 1].psi : A[0].psi + 2.0 * glm::pi<double>();
                double psiBNext = (ib + 1 < nB) ? B[ib + 1].psi : B[0].psi + 2.0 * glm::pi<double>();
                bool advanceA = (ib >= nB) || (ia < nA && psiANext <= psiBNext);

                unsigned int a0 = baseA + (unsigned)(ia % nA);
                unsigned int b0 = baseB + (unsigned)(ib % nB);
                if (advanceA) {
                    unsigned int a1 = baseA + (unsigned)((ia + 1) % nA);
                    strip.insert(strip.end(), { a0, a1, b0 });
                    ++ia;
                } else {
                    unsigned int b1 = baseB + (unsigned)((ib + 1) % nB);
                    strip.insert(strip.end(), { a0, b1, b0 });
                    ++ib;
                }
            }
        });
        for (const auto& strip : strips)
            surfIdx.insert(surfIdx.end(), strip.begin(), strip.end());

        surfaceMesh = std::make_unique<engine::Mesh>(verts, surfIdx, std::vector<int>{ 3, 3, 3 });
    }

    engine::App* app = nullptr;
    engine::Shader shader;
    std::unique_ptr<engine::Mesh> pointsMesh;
    std::vector<std::unique_ptr<engine::Mesh>> ringMeshes;
    std::unique_ptr<engine::Mesh> surfaceMesh;

    Params params;
    GrowthParams growth;
    Manifold cachedManifold;
    std::string buildError;
    int lastTaylorRingCount = 0;
    double lastDelta = 0.0;
    double rebuildMs = 0.0;   // wall time of the last full rebuild (K, rings, growth, meshes)

    float modelScale = 1.0f;
    bool showSurface = true;
    bool showPoints = true;
    bool showRings = true;
    bool colorByHeight = false;

    engine::Shader hmShader;           // conjugacy preview (ConjugacyMap GLSL spliced in)
    engine::ConjugacyMap hm;
    std::unique_ptr<engine::Mesh> sphereMesh;
    std::unique_ptr<engine::Mesh> diskMesh;
    std::string hmError;
    bool showHmBall = false;
    bool showHmDisk = false;
    float hmRadius = 3.0f;
    float hmAlpha = 0.08f;

    float gradientScale = 1.0f;  // 1.0 = full red->blue across the whole range
    bool bandColors = true;
    float bandContrast = 0.35f;
};

} // namespace

std::unique_ptr<engine::Sim> makeLorenzManifoldSim()
{
    return std::make_unique<LorenzManifoldSim>();
}
