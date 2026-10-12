#ifndef ENGINE_TRAJECTORY_H
#define ENGINE_TRAJECTORY_H

#include <engine/Integrator.h>
#include <engine/Mesh.h>

#include <glm.hpp>
#include <vector>

namespace engine {

// One integral curve of a vector field, integrated in real time and drawn as a
// growing polyline. It owns the ODE state (position + time), the CPU point list and
// the dynamic Mesh those points stream into, so a sim can keep a handful of these
// around and just call advance(dt) every frame.
//
// Once maxPoints is reached the trail is a strict FIFO: every appended point retires
// exactly one point from the tail, so the curve erodes continuously instead of losing
// a block at a time. See the note on the mirrored ring buffer below.
//
// 2D curves are stored with z = 0, so the same buffer and shader serve R^2 and R^3.
// Construction allocates a VBO: build these in a sim's init()/UI callbacks, never
// before the GL context exists.
class Trajectory {
public:
    Trajectory(int dim, const double* seed, double t0 = 0.0, int maxPoints = 20000);

    Trajectory(const Trajectory&) = delete;
    Trajectory& operator=(const Trajectory&) = delete;
    Trajectory(Trajectory&&) noexcept = default;
    Trajectory& operator=(Trajectory&&) noexcept = default;

    // Back to a single point at `seed`; keeps dim and the trail capacity.
    void reset(const double* seed, double t0 = 0.0);

    // Integrate `f` by dt of ODE time (negative = run the flow backwards), in substeps
    // of at most integrator.maxStep, and append every substep point — so raising the
    // time scale makes the trail denser per frame, never coarser. Stops for good once the state leaves kEscape or goes
    // non-finite — a blown-up curve stays on screen instead of poisoning the buffer.
    void advance(const Integrator& integrator, const Integrator::Field& f, double dt);

    void draw() const;      // GL_LINE_STRIP over the live trail, oldest vertex first
    void drawHead() const;  // GL_POINTS, the newest point only

    // The vertex range draw() issues. A shader that fades the trail by age can turn
    // gl_VertexID into "how old is this point" with these: age 0 at drawFirst(),
    // 1 at drawFirst() + drawCount() - 1.
    int drawFirst() const;
    int drawCount() const { return count; }

    // Points nearer than this to the previous one are not committed, so a slow region
    // of the field doesn't spend the whole trail standing still. The newest point still
    // follows the true position every step (it is moved in place until it is far enough
    // to commit), so the head and the trail tip never lag. World units.
    float minSpacing = 0.0f;

    bool empty() const { return count == 0; }
    int size() const { return count; }
    int maxPoints() const { return capacity; }
    void setMaxPoints(int n);        // rebuilds the buffer, keeping the newest points

    int dim() const { return d; }
    glm::vec3 head() const;          // current position, world space (z = 0 in 2D)
    glm::vec3 velocity() const;      // field value last seen at the head
    double speed() const;            // |velocity|
    double time() const { return t; }
    const double* state() const { return x; }
    bool diverged() const { return blownUp; }

    // |x_i| beyond this counts as escape to infinity.
    static constexpr double kEscape = 1e7;

private:
    // Returns false when minSpacing moved the live tip instead of adding a slot. upload = false leaves the GPU
    // copy stale; follow with uploadSlots().
    bool appendPoint(const glm::vec3& p, float speed, bool upload = true);
    void uploadAll();
    void uploadSlots(int firstSlot, int n);  // n ring slots from firstSlot, + mirrors
    int newestSlot() const;          // ring index of the last recorded point

    static constexpr int kFloatsPerPoint = 4; // pos(3) + speed(1)
    static constexpr int kMaxSubsteps = 4096; // per advance(), so a hitch can't freeze

    int d = 3;
    double x[Integrator::kMaxDim] = {0.0, 0.0, 0.0, 0.0};
    double t = 0.0;
    double vel[Integrator::kMaxDim] = {0.0, 0.0, 0.0, 0.0};
    bool blownUp = false;

    // Ring of `capacity` points, mirrored: slot i is stored at i and at i + capacity.
    // Any chronological window of `capacity` points is then contiguous, so the whole
    // trail is still one glDrawArrays with no seam and no per-frame shifting.
    int capacity = 0;
    int count = 0;   // live points, <= capacity
    int write = 0;   // next slot to write (and, once full, the oldest point)
    std::vector<float> cpu;          // 2 * capacity * kFloatsPerPoint
    Mesh mesh;
};

} // namespace engine

#endif
