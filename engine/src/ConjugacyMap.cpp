#include <engine/ConjugacyMap.h>

#include <glad/glad.h>

#include <cmath>
#include <utility>
#include <vector>

namespace engine {

namespace {

constexpr int kMaxTri = (ConjugacyMap::kMaxOrder + 1) * (ConjugacyMap::kMaxOrder + 2) / 2;

// std140 image of the GLSL block below: mat4, ivec4[2], vec4[2 * kMaxTri * 2]. Every
// member is a multiple of 16 bytes, so the C++ layout matches with no padding.
struct Block {
    float split[16];             // mat4, column-major (only the 3x3 is used)
    int info[2][4];              // per side: k, order, complex pair, unused
    float coef[2][kMaxTri][2][4];// per side, per (m, j): (re x, im x, re y, im y), (re z, im z, -, -)
};
static_assert(sizeof(Block) == 64 + 32 + 2 * kMaxTri * 32, "std140 layout");

} // namespace

ConjugacyMap::~ConjugacyMap()
{
    release();
}

ConjugacyMap::ConjugacyMap(ConjugacyMap&& o) noexcept
    : ws(std::move(o.ws)), wu(std::move(o.wu)), ok(o.ok), err(std::move(o.err)), ubo(o.ubo)
{
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            splitM[i][j] = o.splitM[i][j];
    o.ubo = 0;
    o.ok = false;
}

ConjugacyMap& ConjugacyMap::operator=(ConjugacyMap&& o) noexcept
{
    if (this != &o)
    {
        release();
        ws = std::move(o.ws);
        wu = std::move(o.wu);
        ok = o.ok;
        err = std::move(o.err);
        ubo = o.ubo;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                splitM[i][j] = o.splitM[i][j];
        o.ubo = 0;
        o.ok = false;
    }
    return *this;
}

void ConjugacyMap::release()
{
    if (ubo != 0)
    {
        glDeleteBuffers(1, &ubo);
        ubo = 0;
    }
}

bool ConjugacyMap::set(const InvariantManifold& Ws, const InvariantManifold& Wu)
{
    ok = false;
    err.clear();
    if (!Ws.valid() || !Wu.valid() || Ws.dim() + Wu.dim() != 3)
    {
        err = "needs a hyperbolic saddle: valid stable and unstable manifolds, dims adding to 3";
        return false;
    }
    if (Ws.order() > kMaxOrder || Wu.order() > kMaxOrder)
    {
        err = "order above ConjugacyMap::kMaxOrder";
        return false;
    }
    ws = Ws;
    wu = Wu;

    // columns of M = [B_s | B_u]; theta = M (s, u) so (s, u) = M^{-1} theta
    double M[3][3];
    int col = 0;
    for (const InvariantManifold* W : {&ws, &wu})
        for (int a = 0; a < W->dim(); ++a, ++col)
            for (int r = 0; r < 3; ++r)
                M[r][col] = W->basis(a)[r];
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
               - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
               + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (!(std::abs(det) > 1e-12))
    {
        err = "E^s and E^u do not span R^3";
        return false;
    }
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            // inverse = adjugate / det: entry (i, j) is the cofactor of (j, i)
            int r0 = (j + 1) % 3, r1 = (j + 2) % 3, c0 = (i + 1) % 3, c1 = (i + 2) % 3;
            splitM[i][j] = (M[r0][c0] * M[r1][c1] - M[r0][c1] * M[r1][c0]) / det;
        }

    std::vector<Block> block(1);
    Block& b = block[0];
    std::fill(std::begin(b.split), std::end(b.split), 0.0f);
    for (int c = 0; c < 3; ++c)
        for (int r = 0; r < 3; ++r)
            b.split[c * 4 + r] = (float)splitM[r][c];
    b.split[15] = 1.0f;

    const InvariantManifold* sides[2] = {&ws, &wu};
    for (int side = 0; side < 2; ++side)
    {
        const InvariantManifold& W = *sides[side];
        b.info[side][0] = W.dim();
        b.info[side][1] = W.order();
        b.info[side][2] = W.complexPair() ? 1 : 0;
        b.info[side][3] = 0;
        int idx = 0;
        for (int m = 0; m <= kMaxOrder; ++m)
            for (int j = 0; j + m <= kMaxOrder; ++j, ++idx)
            {
                float* v0 = b.coef[side][idx][0];
                float* v1 = b.coef[side][idx][1];
                bool inUse = m + j <= W.order() && (W.dim() == 2 || j == 0);
                const auto c = inUse ? W.coeff(m, j) : std::array<std::complex<double>, 3>{};
                v0[0] = (float)c[0].real(); v0[1] = (float)c[0].imag();
                v0[2] = (float)c[1].real(); v0[3] = (float)c[1].imag();
                v1[0] = (float)c[2].real(); v1[1] = (float)c[2].imag();
                v1[2] = 0.0f;               v1[3] = 0.0f;
            }
    }

    if (ubo == 0)
        glGenBuffers(1, &ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, ubo);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(Block), &b, GL_DYNAMIC_DRAW);
    glBindBuffer(GL_UNIFORM_BUFFER, 0);
    ok = true;
    return true;
}

void ConjugacyMap::split(const Vec3& theta, Param& s, Param& u) const
{
    double c[3];
    for (int i = 0; i < 3; ++i)
        c[i] = splitM[i][0] * theta[0] + splitM[i][1] * theta[1] + splitM[i][2] * theta[2];
    if (ws.dim() == 2)
    {
        s = {c[0], c[1]};
        u = {c[2], 0.0};
    }
    else
    {
        s = {c[0], 0.0};
        u = {c[1], c[2]};
    }
}

ConjugacyMap::Vec3 ConjugacyMap::hInv(const Vec3& theta) const
{
    if (!ok)
        return theta;
    Param s, u;
    split(theta, s, u);
    Vec3 a = ws.eval(s), b = wu.eval(u);
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

void ConjugacyMap::bind() const
{
    if (ubo != 0)
        glBindBufferBase(GL_UNIFORM_BUFFER, kBinding, ubo);
}

const std::string& ConjugacyMap::glsl()
{
    static const std::string src = "#define HM_MAX_TRI " + std::to_string(kMaxTri) + "\n" + R"(
// ---- engine::ConjugacyMap: h~^{-1}(theta) = K_s(s) + K_u(u) --------------------------
layout(std140) uniform ConjugacyMapBlock {
    mat4 hm_split;                       // theta -> (s, u), 3x3 part
    ivec4 hm_info[2];                    // per side: k, order, complex pair
    vec4 hm_coef[2 * HM_MAX_TRI * 2];
};

vec2 hm_cmul(vec2 a, vec2 b) { return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x); }

// K(s) for one side: sum over m + j <= order of Re(p_mj w1^m w2^j), with w = s for real
// eigenvalues and w = (z, conj z), z = (s0 - i s1) / 2, for a complex pair
vec3 hm_K(int side, vec2 s)
{
    ivec4 info = hm_info[side];
    vec2 w1, w2;
    if (info.z == 1)
    {
        w1 = 0.5 * vec2(s.x, -s.y);
        w2 = vec2(w1.x, -w1.y);
    }
    else
    {
        w1 = vec2(s.x, 0.0);
        w2 = vec2(info.x == 2 ? s.y : 0.0, 0.0);
    }
    vec3 outv = vec3(0.0);
    vec2 p1 = vec2(1.0, 0.0);
    int base = side * HM_MAX_TRI * 2;
    int idx = 0;
    for (int m = 0; m <= HM_MAX_TRI; ++m)          // bounded by the order inside
    {
        if (m > info.y)
            break;
        vec2 bas = p1;
        for (int j = 0; j <= info.y - m; ++j)
        {
            vec4 c0 = hm_coef[base + 2 * idx];
            vec4 c1 = hm_coef[base + 2 * idx + 1];
            outv += vec3(c0.x * bas.x - c0.y * bas.y,
                         c0.z * bas.x - c0.w * bas.y,
                         c1.x * bas.x - c1.y * bas.y);
            bas = hm_cmul(bas, w2);
            ++idx;
        }
        // skip the slots of this m beyond the order (the buffer is laid out for the max)
        idx += )" + std::to_string(kMaxOrder) + R"( - m - (info.y - m);
        p1 = hm_cmul(p1, w1);
    }
    return outv;
}

vec3 hInv(vec3 theta)
{
    vec3 c = mat3(hm_split) * theta;
    vec2 s, u;
    if (hm_info[0].x == 2) { s = c.xy;             u = vec2(c.z, 0.0); }
    else                   { s = vec2(c.x, 0.0);   u = c.yz; }
    return hm_K(0, s) + hm_K(1, u);
}
// ---- end ConjugacyMap -------------------------------------------------------------------
)";
    return src;
}

} // namespace engine
