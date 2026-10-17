#include <engine/TrajectoryRunner.h>

#include <engine/Trajectory.h>

#include <imgui.h>

#include <algorithm>
#include <execution>

namespace engine {

namespace {

double msSince(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
        .count();
}

} // namespace

void TrajectoryRunner::advance(const std::vector<Trajectory*>& curves,
                               const Integrator& integrator, const Integrator::Field& f,
                               double dt)
{
    run(curves, [&](Trajectory* c) { c->integrate(integrator, f, dt); });
}

void TrajectoryRunner::advance(const std::vector<Trajectory*>& curves,
                               const TaylorIntegrator& taylor, const TaylorIntegrator::Jet& jet,
                               double dt, double sampleStep)
{
    run(curves, [&](Trajectory* c) { c->integrate(taylor, jet, dt, sampleStep); });
}

void TrajectoryRunner::run(const std::vector<Trajectory*>& curves,
                           const std::function<void(Trajectory*)>& step)
{
    frameCurves += (int)curves.size();
    auto t0 = Clock::now();
    if (parallel)
        std::for_each(std::execution::par, curves.begin(), curves.end(), step);
    else
        std::for_each(curves.begin(), curves.end(), step);
    frameIntegrate += msSince(t0);

    auto t1 = Clock::now();
    for (Trajectory* c : curves)
        c->flush();
    frameUpload += msSince(t1);
}

void TrajectoryRunner::beginDraw()
{
    // close the frame's advance() sums (zero while paused)
    integrate = smooth(integrate, frameIntegrate);
    upload = smooth(upload, frameUpload);
    curveCount = frameCurves;
    frameIntegrate = frameUpload = 0.0;
    frameCurves = 0;

    gpu.begin();
    drawStart = Clock::now();
}

void TrajectoryRunner::endDraw()
{
    drawCpu = smooth(drawCpu, msSince(drawStart));
    gpu.end();
}

void TrajectoryRunner::drawUI()
{
    ImGui::Checkbox("parallel integration", &parallel);
    ImGui::SameLine();
    ImGui::TextDisabled("(%d curves)", curveCount);
    ImGui::TextDisabled("integrate %.2f ms   upload %.2f ms", integrate, upload);
    ImGui::TextDisabled("draw trails %.2f ms CPU, %.2f ms GPU", drawCpu, gpu.lastMs());
    ImGui::TextDisabled("frame %.2f ms (%.0f fps)", 1000.0f / std::max(ImGui::GetIO().Framerate, 1e-3f),
                        ImGui::GetIO().Framerate);
}

} // namespace engine
