#include <engine/Trajectory.h>

#include <cmath>
#include <cstring>

namespace engine {

namespace {

bool finiteState(const double* x, int dim)
{
    for (int i = 0; i < dim; ++i)
        if (!std::isfinite(x[i]) || std::abs(x[i]) > Trajectory::kEscape)
            return false;
    return true;
}

int clampCapacity(int n) { return n < 2 ? 2 : n; }

} // namespace

Trajectory::Trajectory(int dim, const double* seed, double t0, int maxPoints)
    : d(dim < 2 ? 2 : (dim > 3 ? 3 : dim)),
      capacity(clampCapacity(maxPoints)),
      cpu((size_t)clampCapacity(maxPoints) * 2 * kFloatsPerPoint, 0.0f),
      mesh(clampCapacity(maxPoints) * 2, std::vector<int>{3, 1})
{
    reset(seed, t0);
}

void Trajectory::reset(const double* seed, double t0)
{
    for (int i = 0; i < Integrator::kMaxDim; ++i)
        x[i] = vel[i] = 0.0;
    for (int i = 0; i < d; ++i)
        x[i] = seed[i];

    t = t0;
    blownUp = false;
    count = 0;
    write = 0;
    appendPoint(head(), 0.0f);
}

void Trajectory::advance(const Integrator& integrator, const Integrator::Field& f,
                         double dt)
{
    if (blownUp || dt == 0.0)
        return;

    // Same substep split as Integrator::advance, but every substep is recorded, so a
    // fast flow draws a smooth curve instead of one long chord per frame.
    double cap = integrator.maxStep > 1e-9 ? integrator.maxStep : 1e-9;
    int n = (int)std::ceil(std::abs(dt) / cap);
    if (n < 1)
        n = 1;
    if (n > kMaxSubsteps)
        n = kMaxSubsteps;
    double h = dt / n;

    // the newest slot is the live tip and may be moved in place, so it is dirty too
    int firstSlot = newestSlot();
    int appended = 0;
    for (int k = 0; k < n; ++k)
    {
        double before[Integrator::kMaxDim];
        std::memcpy(before, x, sizeof(before));
        double tBefore = t;

        integrator.step(f, d, x, t, h);

        if (!finiteState(x, d))
        {
            std::memcpy(x, before, sizeof(before)); // keep the last good point drawable
            t = tBefore;
            blownUp = true;
            break;
        }

        f(x, t, vel);
        if (appendPoint(head(), (float)speed(), false))
            ++appended;
    }

    uploadSlots(firstSlot, appended + 1);
}

int Trajectory::newestSlot() const
{
    return (write - 1 + capacity) % capacity;
}

bool Trajectory::appendPoint(const glm::vec3& p, float speedAtPoint, bool upload)
{
    float v[kFloatsPerPoint] = {p.x, p.y, p.z, speedAtPoint};

    // minSpacing: the newest point is a live tip that tracks the true position. While
    // it is still within minSpacing of the point before it, it is moved in place
    // instead of committed — the trail stays sparse but the head never lags behind.
    if (count >= 2 && minSpacing > 0.0f)
    {
        int prevSlot = (write - 2 + capacity) % capacity;
        const float* last = &cpu[(size_t)prevSlot * kFloatsPerPoint];
        glm::vec3 prev(last[0], last[1], last[2]);
        if (glm::length(p - prev) < minSpacing)
        {
            int tip = newestSlot();
            size_t slot = (size_t)tip * kFloatsPerPoint;
            size_t mirror = (size_t)(tip + capacity) * kFloatsPerPoint;
            std::memcpy(&cpu[slot], v, sizeof(v));
            std::memcpy(&cpu[mirror], v, sizeof(v));
            if (upload)
            {
                mesh.setVertices(v, kFloatsPerPoint, slot);
                mesh.setVertices(v, kFloatsPerPoint, mirror);
            }
            return false;
        }
    }

    // write the slot and its mirror, so the live window stays contiguous
    size_t slot = (size_t)write * kFloatsPerPoint;
    size_t mirror = (size_t)(write + capacity) * kFloatsPerPoint;
    std::memcpy(&cpu[slot], v, sizeof(v));
    std::memcpy(&cpu[mirror], v, sizeof(v));
    if (upload)
    {
        mesh.setVertices(v, kFloatsPerPoint, slot);
        mesh.setVertices(v, kFloatsPerPoint, mirror);
    }

    write = (write + 1) % capacity;
    if (count < capacity)
        ++count;
    // once count == capacity, each append retires exactly one point from the tail
    return true;
}

void Trajectory::uploadSlots(int firstSlot, int n)
{
    if (n <= 0)
        return;
    if (n >= capacity)
    {
        uploadAll();
        return;
    }

    // [firstSlot, firstSlot + n) may wrap the ring once: at most two runs, each
    // written to the slot and to its mirror
    int run1 = n < capacity - firstSlot ? n : capacity - firstSlot;
    int runs[2][2] = {{firstSlot, run1}, {0, n - run1}};
    for (auto& r : runs)
    {
        if (r[1] <= 0)
            continue;
        size_t len = (size_t)r[1] * kFloatsPerPoint;
        for (int base : {r[0], r[0] + capacity})
        {
            size_t off = (size_t)base * kFloatsPerPoint;
            mesh.setVertices(&cpu[off], len, off);
        }
    }
}

void Trajectory::uploadAll()
{
    if (capacity > 0)
        mesh.setVertices(cpu.data(), cpu.size(), 0);
}

int Trajectory::drawFirst() const
{
    // not yet wrapped: the points sit in order from 0. Wrapped: the oldest live point
    // is the one `write` is about to overwrite, and the mirror makes [write, write +
    // capacity) a single contiguous chronological run.
    return count < capacity ? 0 : write;
}

void Trajectory::setMaxPoints(int n)
{
    int wanted = clampCapacity(n);
    if (wanted == capacity)
        return;

    int keep = count < wanted ? count : wanted;
    int first = drawFirst();
    int skip = count - keep;   // drop the oldest ones that no longer fit

    std::vector<float> kept((size_t)wanted * 2 * kFloatsPerPoint, 0.0f);
    for (int i = 0; i < keep; ++i)
    {
        int src = (first + skip + i) % capacity;
        const float* from = &cpu[(size_t)src * kFloatsPerPoint];
        std::memcpy(&kept[(size_t)i * kFloatsPerPoint], from,
                    kFloatsPerPoint * sizeof(float));
        std::memcpy(&kept[(size_t)(i + wanted) * kFloatsPerPoint], from,
                    kFloatsPerPoint * sizeof(float));
    }

    cpu = std::move(kept);
    capacity = wanted;
    count = keep;
    write = keep % wanted;
    mesh = Mesh(capacity * 2, std::vector<int>{3, 1});
    uploadAll();
}

void Trajectory::draw() const
{
    if (count >= 2)
        mesh.drawArrays(GL_LINE_STRIP, count, drawFirst());
}

void Trajectory::drawHead() const
{
    if (count >= 1)
        mesh.drawArrays(GL_POINTS, 1, newestSlot());
}

glm::vec3 Trajectory::head() const
{
    return glm::vec3((float)x[0], (float)x[1], d == 3 ? (float)x[2] : 0.0f);
}

glm::vec3 Trajectory::velocity() const
{
    return glm::vec3((float)vel[0], (float)vel[1], d == 3 ? (float)vel[2] : 0.0f);
}

double Trajectory::speed() const
{
    double sum = 0.0;
    for (int i = 0; i < d; ++i)
        sum += vel[i] * vel[i];
    return std::sqrt(sum);
}

} // namespace engine
