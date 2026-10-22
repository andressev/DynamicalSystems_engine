#ifndef ENGINE_CONJUGACY_MAP_H
#define ENGINE_CONJUGACY_MAP_H

#include <engine/InvariantManifold.h>

#include <string>

namespace engine {

// The (manifold-exact) Hartman-Grobman map at a hyperbolic equilibrium v*, evaluated
// per vertex on the GPU:
//     h~^{-1}(theta) = K_s(s) + K_u(u),   theta = B_s s + B_u u  (E^s (+) E^u = R^3)
// built from the two InvariantManifold parametrizations. On E^s and E^u it IS the
// conjugacy restricted to the invariant manifolds; off them it is an interpolation
// (the true h is not computable this way). Result is relative to v*.
//
// The Taylor coefficients of both sides go into one uniform buffer; glsl() is the
// matching GLSL (block + `vec3 hInv(vec3 theta)`) a sim splices into its vertex shader,
// so a whole mesh — a sphere, a disk in E^s — deforms in the vertex stage. hInv() is
// the same map on the CPU in double, for the few points that need exactness
// (trajectories). Owns one GL buffer: non-copyable, movable; create after the context.
class ConjugacyMap {
public:
    using Vec3 = InvariantManifold::Vec3;
    using Param = InvariantManifold::Param;

    static constexpr int kMaxOrder = 16;          // coefficient slots in the GLSL block
    static constexpr unsigned int kBinding = 1;   // uniform-buffer binding point
    static constexpr const char* kBlockName = "ConjugacyMapBlock";

    ConjugacyMap() = default;
    ~ConjugacyMap();
    ConjugacyMap(const ConjugacyMap&) = delete;
    ConjugacyMap& operator=(const ConjugacyMap&) = delete;
    ConjugacyMap(ConjugacyMap&& other) noexcept;
    ConjugacyMap& operator=(ConjugacyMap&& other) noexcept;

    // Both manifolds of the same equilibrium; their dimensions must add to 3 and their
    // orders be <= kMaxOrder. Uploads the buffer. False (see error()) otherwise.
    bool set(const InvariantManifold& Ws, const InvariantManifold& Wu);
    bool valid() const { return ok; }
    const std::string& error() const { return err; }

    Vec3 hInv(const Vec3& theta) const;                       // CPU twin, double
    void split(const Vec3& theta, Param& s, Param& u) const;  // theta -> (s, u)
    const InvariantManifold& stable() const { return ws; }
    const InvariantManifold& unstable() const { return wu; }

    // Bind the buffer to kBinding before drawing; the shader must have been linked
    // to it once with shader.bindUniformBlock(kBlockName, kBinding).
    void bind() const;

    // GLSL for the vertex shader (GLSL 330): the uniform block and vec3 hInv(vec3).
    static const std::string& glsl();

private:
    void release();

    InvariantManifold ws, wu;
    double splitM[3][3] = {};   // [B_s | B_u]^{-1}
    bool ok = false;
    std::string err;
    unsigned int ubo = 0;
};

} // namespace engine

#endif
