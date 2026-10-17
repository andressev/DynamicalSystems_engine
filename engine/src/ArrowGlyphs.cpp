#include <engine/ArrowGlyphs.h>
#include <engine/Shader.h>

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

// Base-shape vertex: local pos(3) + magnitude(1, unused) + fade(1) + fixed shade(1)
// + local normal(4, w = 1 -> lit). Positions are in arrow lengths, in the frame
// (x = along the arrow, y = a, z = b) the vertex shader builds per instance.
const std::vector<int> kBaseLayout = {3, 1, 1, 1, 4};
const std::vector<int> kInstanceLayout = {3, 3, 2};

void push(std::vector<float>& out, float x, float y, float z, float fade, float shade,
          float nx = 0.0f, float ny = 0.0f, float nz = 0.0f, float lit = 0.0f)
{
    out.insert(out.end(), {x, y, z, 0.0f, fade, shade, nx, ny, nz, lit});
}

std::unique_ptr<Mesh> makeBase(const std::vector<float>& verts)
{
    int n = (int)verts.size() / 10;
    if (n == 0)
        return nullptr;
    std::vector<unsigned int> idx(n);
    for (int i = 0; i < n; ++i)
        idx[i] = (unsigned)i;
    return std::make_unique<Mesh>(verts, idx, kBaseLayout);
}

int withSlack(int n) { return n + n / 4 + 64; }

} // namespace

ArrowGlyphs::Lod ArrowGlyphs::lodFor(int arrows)
{
    // instancing makes dense grids cheap to build; the LOD now only trims fill and
    // vertex work for very dense ones
    if (arrows <= 50000) return {7, true};
    if (arrows <= 150000) return {5, true};
    return {4, false};
}

void ArrowGlyphs::clear()
{
    inst.clear();
}

void ArrowGlyphs::add(const glm::vec3& p, const glm::vec3& dir, float len, float mag, int d)
{
    dim = d;
    inst.insert(inst.end(), {p.x, p.y, p.z, dir.x, dir.y, dir.z, len, mag});
}

void ArrowGlyphs::addField(const std::vector<glm::vec3>& points,
                           const std::vector<glm::vec3>& vectors, float baseLen,
                           float colorMax, float hideBelow, bool lengthByMagnitude, int d)
{
    lod = lodFor((int)points.size());
    dim = d;
    float scaleMax = std::max(colorMax, 1e-6f);
    float cutoff = hideBelow * colorMax;
    size_t n = std::min(points.size(), vectors.size());
    inst.reserve(inst.size() + n * kFloatsPerInstance);
    for (size_t i = 0; i < n; ++i)
    {
        float mag = glm::length(vectors[i]);
        if (!(mag > 1e-9f) || mag < cutoff)
            continue;
        float len = baseLen;
        if (lengthByMagnitude)
            len *= std::clamp(mag / scaleMax, 0.05f, 1.0f);
        add(points[i], vectors[i] / mag, len, mag, d);
    }
}

void ArrowGlyphs::rebuildBase(const Style& s)
{
    built = s;
    std::vector<float> lines, tris;
    float hf = s.headFraction;

    if (s.barbs)
    {
        // full-length shaft, then two (R2) or four (R3) barbs back from the tip
        push(lines, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
        push(lines, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f);
        const float back = 0.68f, wide = 0.16f;
        float sides[4][2] = {{wide, 0.0f}, {-wide, 0.0f}, {0.0f, wide}, {0.0f, -wide}};
        for (int i = 0; i < (s.dim == 2 ? 2 : 4); ++i)
        {
            push(lines, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f);
            push(lines, back, sides[i][0], sides[i][1], 0.9f, 1.0f);
        }
    }
    else
    {
        // the shaft stops at the head's base so it never pokes through; a head-only
        // batch (headFraction >= 1) has no shaft at all
        if (hf < 1.0f)
        {
            push(lines, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            push(lines, 1.0f - hf, 0.0f, 0.0f, 1.0f, 1.0f);
        }
        float base = 1.0f - hf, rad = hf * s.headWidth;
        if (s.dim == 2)
        {
            push(tris, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f);
            push(tris, base, rad, 0.0f, 1.0f, 0.88f);
            push(tris, base, -rad, 0.0f, 1.0f, 0.88f);
        }
        else
        {
            int seg = std::clamp(s.segments, 3, kMaxConeSegments);
            float hyp = std::sqrt(rad * rad + hf * hf);
            float slant = rad / hyp, axial = hf / hyp;
            for (int i = 0; i < seg; ++i)
            {
                float t0 = 6.28318530718f * (float)i / (float)seg;
                float t1 = 6.28318530718f * (float)(i + 1) / (float)seg;
                float tm = 0.5f * (t0 + t1);
                // facet normal: outward at the facet's middle, tilted toward the tip
                float nx = slant, ny = axial * std::cos(tm), nz = axial * std::sin(tm);
                push(tris, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f, nx, ny, nz, 1.0f);
                push(tris, base, rad * std::cos(t0), rad * std::sin(t0), 1.0f, 1.0f, nx, ny, nz, 1.0f);
                push(tris, base, rad * std::cos(t1), rad * std::sin(t1), 1.0f, 1.0f, nx, ny, nz, 1.0f);
                if (s.cap)
                {
                    push(tris, base, 0.0f, 0.0f, 1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 1.0f);
                    push(tris, base, rad * std::cos(t1), rad * std::sin(t1), 1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 1.0f);
                    push(tris, base, rad * std::cos(t0), rad * std::sin(t0), 1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 1.0f);
                }
            }
        }
    }
    shaftBase = makeBase(lines);
    headBase = makeBase(tris);
}

void ArrowGlyphs::upload()
{
    uploaded = arrows();
    if (uploaded == 0)
        return;

    Style want{dim, barbs, headFraction, headWidth, lod.segments, lod.cap};
    bool rebuilt = !(want == built) || (!shaftBase && !headBase);
    if (rebuilt)
        rebuildBase(want);

    for (Mesh* m : {shaftBase.get(), headBase.get()})
    {
        if (!m)
            continue;
        if (rebuilt || m->instanceCapacity() < uploaded)
            m->setInstanceLayout(withSlack(uploaded), kInstanceLayout);
        m->setInstances(inst.data(), inst.size());
    }
}

void ArrowGlyphs::drawShafts(const Shader& shader) const
{
    if (uploaded == 0 || !shaftBase)
        return;
    shader.setInt("u_instanced", 1);
    shader.setInt("u_flat", built.dim == 2 ? 1 : 0);
    shaftBase->drawInstanced(GL_LINES, uploaded);
    shader.setInt("u_instanced", 0);
}

void ArrowGlyphs::drawHeads(const Shader& shader) const
{
    if (uploaded == 0 || !headBase)
        return;
    shader.setInt("u_instanced", 1);
    shader.setInt("u_flat", built.dim == 2 ? 1 : 0);
    headBase->drawInstanced(GL_TRIANGLES, uploaded);
    shader.setInt("u_instanced", 0);
}

} // namespace engine
