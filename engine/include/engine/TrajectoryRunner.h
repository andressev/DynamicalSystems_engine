#ifndef ENGINE_TRAJECTORY_RUNNER_H
#define ENGINE_TRAJECTORY_RUNNER_H

#include <engine/GpuTimer.h>
#include <engine/Integrator.h>
#include <engine/TaylorIntegrator.h>

#include <chrono>
#include <functional>
#include <vector>

namespace engine {

class Trajectory;

// Advances a sim's trajectories each frame and profiles the three costs of drawing a
// flow: integrating (CPU, optionally across all cores — one task per curve, results
// bit-identical to serial), uploading the new trail points (GL thread), and drawing the
// trails (CPU submit time + GPU time). The readout is what tells which one to optimize.
class TrajectoryRunner {
public:
    bool parallel = true;   // integrate curves on all cores (f must be thread-safe)

    // integrate() every curve by dt, then flush() them all on this (GL) thread.
    void advance(const std::vector<Trajectory*>& curves, const Integrator& integrator,
                 const Integrator::Field& f, double dt);
    // The same with the Taylor method (Trajectory's Taylor integrate). A sim may call
    // both in one frame (e.g. RK4 for a ball, Taylor for singles): the timings add up.
    void advance(const std::vector<Trajectory*>& curves, const TaylorIntegrator& taylor,
                 const TaylorIntegrator::Jet& jet, double dt, double sampleStep);
    // Both again with a step per curve (dts[i] for curves[i]), still one parallel pass —
    // for sims that slow some curves down (a time change per curve).
    void advance(const std::vector<Trajectory*>& curves, const Integrator& integrator,
                 const Integrator::Field& f, const std::vector<double>& dts);
    void advance(const std::vector<Trajectory*>& curves, const TaylorIntegrator& taylor,
                 const TaylorIntegrator::Jet& jet, const std::vector<double>& dts,
                 double sampleStep);

    // Bracket the trail/head draw calls in render().
    void beginDraw();
    void endDraw();

    // "parallel integration" toggle + the timings (smoothed), for a Performance section.
    void drawUI();

    double integrateMs() const { return integrate; }
    double uploadMs() const { return upload; }
    double drawCpuMs() const { return drawCpu; }
    double drawGpuMs() const { return gpu.lastMs(); }

private:
    using Clock = std::chrono::steady_clock;
    static double smooth(double avg, double sample) { return avg * 0.9 + sample * 0.1; }

    // integrate each curve with `step` (given the curve's slot in `curves`, so its index
    // is &slot - curves.data()), then flush them all; adds to this frame's sums
    void run(const std::vector<Trajectory*>& curves,
             const std::function<void(Trajectory* const&)>& step);

    GpuTimer gpu;
    Clock::time_point drawStart;
    double integrate = 0.0, upload = 0.0, drawCpu = 0.0;
    int curveCount = 0;
    // this frame's advance() calls, smoothed into the readout at beginDraw()
    double frameIntegrate = 0.0, frameUpload = 0.0;
    int frameCurves = 0;
};

} // namespace engine

#endif
