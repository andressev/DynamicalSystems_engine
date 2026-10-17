#ifndef ENGINE_ARROW_GLYPHS_H
#define ENGINE_ARROW_GLYPHS_H

#include <engine/Mesh.h>

#include <glm.hpp>
#include <memory>
#include <vector>

namespace engine {

class Shader;

// A batch of arrow glyphs — the quiver of a vector field, tangent vectors on particles,
// flow-direction cones on a subspace — drawn with GPU instancing. The CPU only writes
// one small record per arrow (tail, unit direction, length, |f|); the cone/shaft/barb
// shapes live in static base meshes in the arrow's local frame and the vertex shader
// places them. Solid heads are cones in R3 and flat triangles in R2 (z = 0 plane);
// "barbs" are cheap line heads drawn with the shafts.
//
// Shader contract (the sims' glyph vertex shader), locations:
//   0 vec3 local position (x along the arrow, y/z across), in arrow lengths
//   1 float magnitude (unused here)  2 float fade (0 tail -> 1 tip)
//   3 float fixed shade              4 vec4 local normal, w = 1 -> lit by the normal
//   5 vec3 instance tail   6 vec3 instance unit direction   7 vec2 (length, |f|)
// and the uniforms u_instanced (1 while drawing a batch) and u_flat (1 = R2 frame).
// drawShafts/drawHeads set both and reset u_instanced to 0.
//
// Usage: clear(), add() any number of arrows, upload(), then drawShafts()/drawHeads()
// with the glyph shader bound.
class ArrowGlyphs {
public:
    static constexpr int kFloatsPerInstance = 8;
    static constexpr int kMaxConeSegments = 7;

    // Cone resolution; lodFor coarsens it for very dense grids.
    struct Lod {
        int segments = kMaxConeSegments;  // clamped to [3, kMaxConeSegments]
        bool cap = true;                  // close the cone's base
    };
    static Lod lodFor(int arrows);

    // How the whole batch looks (shared by every arrow in it — they are one mesh).
    bool barbs = false;          // line heads instead of solid ones
    float headFraction = 0.28f;  // solid head length, fraction of the arrow
    float headWidth = 0.32f;     // solid head radius, fraction of the head length
    Lod lod;

    void clear();

    // Tail on p, tip at p + dir * len; dir must be unit length. `mag` feeds the
    // shader's color ramp. dim 2 lays the batch flat in the z = 0 plane (the last
    // add's dim wins — one batch is one dimension).
    void add(const glm::vec3& p, const glm::vec3& dir, float len, float mag, int dim = 3);

    // A sampled field as a quiver: one arrow per (points[i], vectors[i]), skipping
    // zero vectors and those under `hideBelow * colorMax`. Length is `baseLen`, or with
    // lengthByMagnitude baseLen * clamp(|v| / colorMax, 0.05, 1). Sets lod from the
    // sample count.
    void addField(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& vectors,
                  float baseLen, float colorMax, float hideBelow, bool lengthByMagnitude,
                  int dim = 3);

    // Push the instance records to the GPU; rebuilds the base shapes if the style changed.
    void upload();

    void drawShafts(const Shader& shader) const;
    void drawHeads(const Shader& shader) const;

    int arrows() const { return (int)inst.size() / kFloatsPerInstance; }
    bool empty() const { return uploaded == 0; }

private:
    struct Style {
        int dim = 0;
        bool barbs = false;
        float headFraction = 0.0f, headWidth = 0.0f;
        int segments = 0;
        bool cap = false;
        bool operator==(const Style&) const = default;
    };
    void rebuildBase(const Style& s);

    std::vector<float> inst;
    int dim = 3;
    int uploaded = 0;
    Style built;
    std::unique_ptr<Mesh> shaftBase;   // GL_LINES
    std::unique_ptr<Mesh> headBase;    // GL_TRIANGLES; null for barbs
};

} // namespace engine

#endif
