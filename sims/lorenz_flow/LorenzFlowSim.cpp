#include "LorenzFlowSim.h"

#include <engine/App.h>
#include <engine/ArrowGlyphs.h>
#include <engine/Camera.h>
#include <engine/FlowKeys.h>
#include <engine/TrajectoryRunner.h>
#include <engine/Eigen3.h>
#include <engine/Integrator.h>
#include <engine/LatexImage.h>
#include <engine/LorenzSystem.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>
#include <engine/TaylorIntegrator.h>
#include <engine/Trajectory.h>

#include <gtc/matrix_transform.hpp>
#include <imgui.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <execution>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

// ---- colors, small helpers ------------------------------------------------------

const glm::vec3 kBallColor(1.0f);
const glm::vec3 kSingleColor = {0.35f, 0.95f, 0.45f};   // click-dropped: always green
const glm::vec3 kDivergedColor = {0.95f, 0.25f, 0.25f}; // a head that blew up
const glm::vec3 kEquilibriumColor = {0.92f, 0.12f, 0.10f};
const glm::vec3 kHaloColor = {1.0f, 0.38f, 0.22f};

const char* kSettingsPath = SIMS_DIR "lorenz_flow/settings.txt";
// Clicking a bead zooms onto it and hands off to this sim (when it is registered — not
// in the standalone build, where the zoom just stays).
const char* kEquilibriumSim = "lorenz_equilibrium";

// UI section title: a red labelled separator with breathing room above (same look as
// vector_field / diagonal_linear).
void sectionHeader(const char* title)
{
    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.36f, 0.30f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Separator, ImVec4(1.0f, 0.36f, 0.30f, 0.45f));
    ImGui::SeparatorText(title);
    ImGui::PopStyleColor(2);
}

// Number for typeset labels: 2 decimals, trailing zeros dropped (2.50 -> 2.5, 2.00 -> 2).
std::string texNumber(float v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%.2f", v);
    std::string t = b;
    while (t.back() == '0')
        t.pop_back();
    if (t.back() == '.')
        t.pop_back();
    return t == "-0" ? std::string("0") : t;
}

// "saddle-focus (dim E^s = 1, dim E^u = 2)" etc. — what the linearization says.
std::string classify(const engine::Eigen3& e)
{
    if (!e.hyperbolic())
        return "non-hyperbolic (an eigenvalue on the imaginary axis)";
    int s = e.dimStable(), u = e.dimUnstable();
    bool pair = e.isComplex(0) || e.isComplex(1) || e.isComplex(2);
    std::string kind = u == 0 ? "sink" : s == 0 ? "source" : "saddle";
    if (pair)
        kind = (u == 0 || s == 0) ? "spiral " + kind : "saddle-focus";
    char dims[64];
    std::snprintf(dims, sizeof(dims), " (dim E^s = %d, dim E^u = %d)", s, u);
    return kind + dims;
}

float smoothstep(float a, float b, float x)
{
    float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// Quintic smoothstep: zero velocity and acceleration at both ends of the zoom.
float smootherstep(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (6.0f * t - 15.0f) + 10.0f);
}

constexpr int kTaylor = 3;   // singleMethod: Euler, RK2, RK4 (Integrator), then Taylor

// ---- settings -------------------------------------------------------------------

struct Settings {
    float sigma = 10.0f;
    float rho = 28.0f;
    float beta = 8.0f / 3.0f;
    bool showSystemLatex = true;
    float systemLatexSize = 30.0f;  // the system overlay, px

    float eqRadius = 0.7f;        // marker size, world units
    bool showEqLabels = true;
    float eqLabelSize = 30.0f;
    float zoomRadius = 9.0f;      // orbit distance once zoomed onto an equilibrium
    float zoomSeconds = 1.4f;

    bool showArrows = true;
    bool showAxes = false;
    bool barbedHeads = false;
    bool lengthByMagnitude = false;
    int gridN = 12;
    float arrowScale = 0.8f;
    float headFraction = 0.36f;
    float headWidth = 0.14f;
    float hideBelow = 0.0f;
    float domainR = 30.0f;        // half-size of the sampled cube
    float domainZ = 25.0f;        // its centre height (the attractor sits around z = 25)
    float arrowAlpha = 0.35f;
    float arrowTailAlpha = 0.25f;

    float timeScale = 1.0f;
    bool reverse = false;
    int method = 2;               // Integrator::Method, RK4
    float maxStep = 0.005f;
    int singleMethod = 3;         // singles: Euler, RK2, RK4 or Taylor (kTaylor)
    int taylorOrder = 20;
    int taylorTolExp = 14;        // Taylor tolerance 1e-taylorTolExp

    float ballCenter[3] = {1.0f, 1.0f, 25.0f};
    float ballRadius = 1.0f;
    int ballCount = 200;
    bool ballFilled = false;
    bool ballWithSingle = false;  // drop a followable single at the ball's centre too
    bool followSingle = false;    // the orbit camera tracks the newest single

    float spawnDepth = 0.3f;      // click seeds: fraction of camera -> orbit center
    float singleScale = 2.0f;
    bool showTimeLabels = true;   // phi^t(x_0) above every single
    float timeLabelSize = 36.0f;

    bool showTrails = true;
    bool trailBySpeed = false;
    int trailPoints = 3000;
    float trailWidth = 1.5f;
    float trailAlpha = 0.7f;
    float particleSize = 7.0f;

    // global orbit view (camRadius <= 0: frame the domain automatically)
    float camTheta = 0.6f, camPhi = 1.1f, camRadius = 0.0f;
    float camCenter[3] = {0.0f, 0.0f, 25.0f};
    float orbitSpeed = 1.2f;      // rad/s
    float zoomSpeed = 0.0f;       // world units/s, <= 0: scale with the view
};

// One persisted value, by key: exactly one of the pointers is set.
struct Setting {
    const char* key;
    float* f = nullptr;
    int* i = nullptr;
    bool* b = nullptr;
};

std::vector<Setting> settingsTable(Settings& s)
{
    return {
        {"sigma", &s.sigma}, {"rho", &s.rho}, {"beta", &s.beta},
        {"showSystemLatex", nullptr, nullptr, &s.showSystemLatex},
        {"systemLatexSize", &s.systemLatexSize},
        {"eqRadius", &s.eqRadius},
        {"showEqLabels", nullptr, nullptr, &s.showEqLabels},
        {"eqLabelSize", &s.eqLabelSize},
        {"zoomRadius", &s.zoomRadius}, {"zoomSeconds", &s.zoomSeconds},
        {"showArrows", nullptr, nullptr, &s.showArrows},
        {"showAxes", nullptr, nullptr, &s.showAxes},
        {"barbedHeads", nullptr, nullptr, &s.barbedHeads},
        {"lengthByMagnitude", nullptr, nullptr, &s.lengthByMagnitude},
        {"gridN", nullptr, &s.gridN},
        {"arrowScale", &s.arrowScale}, {"headFraction", &s.headFraction},
        {"headWidth", &s.headWidth}, {"hideBelow", &s.hideBelow},
        {"domainR", &s.domainR}, {"domainZ", &s.domainZ},
        {"arrowAlpha", &s.arrowAlpha}, {"arrowTailAlpha", &s.arrowTailAlpha},
        {"timeScale", &s.timeScale},
        {"reverse", nullptr, nullptr, &s.reverse},
        {"method", nullptr, &s.method}, {"maxStep", &s.maxStep},
        {"singleMethod", nullptr, &s.singleMethod}, {"taylorOrder", nullptr, &s.taylorOrder},
        {"taylorTolExp", nullptr, &s.taylorTolExp},
        {"ballCenterX", &s.ballCenter[0]}, {"ballCenterY", &s.ballCenter[1]},
        {"ballCenterZ", &s.ballCenter[2]}, {"ballRadius", &s.ballRadius},
        {"ballCount", nullptr, &s.ballCount},
        {"ballFilled", nullptr, nullptr, &s.ballFilled},
        {"ballWithSingle", nullptr, nullptr, &s.ballWithSingle},
        {"followSingle", nullptr, nullptr, &s.followSingle},
        {"spawnDepth", &s.spawnDepth}, {"singleScale", &s.singleScale},
        {"showTimeLabels", nullptr, nullptr, &s.showTimeLabels},
        {"timeLabelSize", &s.timeLabelSize},
        {"showTrails", nullptr, nullptr, &s.showTrails},
        {"trailBySpeed", nullptr, nullptr, &s.trailBySpeed},
        {"trailPoints", nullptr, &s.trailPoints},
        {"trailWidth", &s.trailWidth}, {"trailAlpha", &s.trailAlpha},
        {"particleSize", &s.particleSize},
        {"cameraTheta", &s.camTheta}, {"cameraPhi", &s.camPhi},
        {"cameraRadius", &s.camRadius},
        {"cameraCenterX", &s.camCenter[0]}, {"cameraCenterY", &s.camCenter[1]},
        {"cameraCenterZ", &s.camCenter[2]},
        {"orbitSpeed", &s.orbitSpeed}, {"zoomSpeed", &s.zoomSpeed},
    };
}

bool saveSettingsFile(Settings& s, const char* path)
{
    std::ofstream out(path);
    if (!out)
        return false;
    for (const Setting& e : settingsTable(s))
    {
        out << e.key << ' ';
        if (e.f) out << *e.f;
        else if (e.i) out << *e.i;
        else out << (*e.b ? 1 : 0);
        out << '\n';
    }
    return (bool)out;
}

// "key value" lines. Unknown keys are ignored, and sizes a hand edit could blow up
// are clamped.
void loadSettingsFile(Settings& s, const char* path)
{
    std::ifstream in(path);
    if (!in)
        return;
    std::vector<Setting> table = settingsTable(s);
    std::string line;
    while (std::getline(in, line))
    {
        size_t sp = line.find(' ');
        if (sp == std::string::npos)
            continue;
        std::string key = line.substr(0, sp);
        double v = std::strtod(line.c_str() + sp + 1, nullptr);
        for (const Setting& e : table)
        {
            if (key != e.key)
                continue;
            if (e.f) *e.f = (float)v;
            else if (e.i) *e.i = (int)v;
            else *e.b = v != 0.0;
        }
    }
    s.gridN = std::clamp(s.gridN, 2, 50);
    s.ballCount = std::clamp(s.ballCount, 1, 1000);
    s.trailPoints = std::clamp(s.trailPoints, 32, 20000);
    s.domainR = std::clamp(s.domainR, 0.5f, 200.0f);
    s.method = std::clamp(s.method, 0, 2);
    s.systemLatexSize = std::clamp(s.systemLatexSize, 10.0f, 120.0f);
    s.singleMethod = std::clamp(s.singleMethod, 0, kTaylor);
    s.taylorOrder = std::clamp(s.taylorOrder, 2, engine::TaylorIntegrator::kMaxOrder);
    s.taylorTolExp = std::clamp(s.taylorTolExp, 4, 16);
    s.maxStep = std::clamp(s.maxStep, 1e-5f, 0.5f);
    s.zoomRadius = std::clamp(s.zoomRadius, 0.5f, 200.0f);
    s.zoomSeconds = std::clamp(s.zoomSeconds, 0.05f, 10.0f);
}

// ---- the sim --------------------------------------------------------------------

// The global picture of the Lorenz flow: arrow field, balls and singles of
// trajectories, and the equilibria as red beads — hover one for its linearization,
// click it to ease the camera onto it. Mode 1 of the Hartman-Grobman track; the
// local picture inside a ball around x* is the separate lorenz_equilibrium sim.
// See obsidian_twin/sims/lorenz-flow.md.
class LorenzFlowSim : public engine::Sim {
public:
    const char* name() const override { return "lorenz_flow"; }

    void init(engine::App& app) override
    {
        this->app = &app;
        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(SIMS_DIR "lorenz_flow/shaders/vertex.glsl",
                                SIMS_DIR "lorenz_flow/shaders/fragment.glsl");
        eqShader = engine::Shader(SIMS_DIR "lorenz_flow/shaders/equilibrium_vertex.glsl",
                                  SIMS_DIR "lorenz_flow/shaders/equilibrium_fragment.glsl");
        axisMesh = std::make_unique<engine::Mesh>(engine::Shapes::axisVertices(1000.0f),
                                                  engine::Shapes::axisIndices(),
                                                  std::vector<int>{3});
        sphereMesh = std::make_unique<engine::Mesh>(engine::Shapes::sphereVertices(48, 32, 1.0f),
                                                    engine::Shapes::sphereIndices(48, 32),
                                                    std::vector<int>{3, 3, 2});

        rhs = [this](const double* x, double, double* dxdt) { system.eval(x, dxdt); };
        jet = [this](const double* x, double, int order, double* c) { system.jet(x, order, c); };

        loadSettingsFile(cfg, kSettingsPath);

        engine::Camera& cam = app.camera();
        cam.mode = engine::Camera::Mode::Orbit;
        cam.inputEnabled = true;
        cam.up = glm::vec3(0.0f, 1.0f, 0.0f);
        cam.orbitRadiusMin = 0.1f;
        cam.orbitSlowZone = 0.0f;
        cam.farPlane = std::max(500.0f, 20.0f * cfg.domainR);

        // back from lorenz_equilibrium: same parameters, the camera still on x*, then
        // the eased zoom out to the global view it was entered from
        const engine::App::Handoff& in = app.handoff();
        auto value = [&](const char* key, double fallback) {
            auto it = in.find(key);
            return it == in.end() ? fallback : it->second;
        };
        const int returnEq = (int)value("eq", -1.0);
        if (returnEq >= 0)
        {
            cfg.sigma = (float)value("sigma", cfg.sigma);
            cfg.rho = (float)value("rho", cfg.rho);
            cfg.beta = (float)value("beta", cfg.beta);
        }
        else
            applyCamera();
        applyParams();
        if (returnEq >= 0 && returnEq < eqCount)
        {
            if (in.count("globalR"))
            {
                globalCenter = glm::vec3((float)value("globalCx", 0.0), (float)value("globalCy", 0.0),
                                         (float)value("globalCz", 0.0));
                globalRadius = (float)value("globalR", 90.0);
            }
            else
            {
                // lorenz_equilibrium was opened from the menu: return to the saved view
                globalCenter = glm::vec3(cfg.camCenter[0], cfg.camCenter[1], cfg.camCenter[2]);
                globalRadius = cfg.camRadius > 0.0f ? cfg.camRadius : 3.0f * cfg.domainR;
            }
            focused = returnEq;
            // the bead comes back at the size lorenz_equilibrium drew it, and grows
            eqs[returnEq].scale = (float)value("beadRadius", cfg.eqRadius * kFocusedScale) /
                                  std::max(cfg.eqRadius, 1e-3f);
            eqs[returnEq].glow = 1.0f;
            backToGlobal();
            returning = true;
        }
        else if (returnEq >= 0)
            applyCamera();   // that equilibrium no longer exists at these parameters
    }

    void update(float dt) override
    {
        // Space toggles play/pause, unless ImGui has the keyboard
        const ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false))
            playing = !playing;
        // arrows: Up/Down speed (down to 0), Left reverse, Right forward
        engine::FlowKeys::handle(cfg.timeScale, cfg.reverse, dt, 10.0f);
        engine::FlowKeys::drawStatus(cfg.timeScale, cfg.reverse, playing);
        // Backspace, not Escape: App closes the window on Escape
        if (!io.WantCaptureKeyboard && focused >= 0 &&
            ImGui::IsKeyPressed(ImGuiKey_Backspace, false))
            backToGlobal();

        integrator.method = (engine::Integrator::Method)cfg.method;
        integrator.maxStep = cfg.maxStep;
        singleIntegrator.method = (engine::Integrator::Method)std::min(cfg.singleMethod, 2);
        singleIntegrator.maxStep = cfg.maxStep;
        taylor.order = cfg.taylorOrder;
        taylor.tolerance = std::pow(10.0, -(double)cfg.taylorTolExp);
        if (playing)
        {
            double h = (double)dt * cfg.timeScale * (cfg.reverse ? -1.0 : 1.0);
            fieldTime += h;
            // the many curves with `method`, singles with their own (Taylor by default)
            std::vector<engine::Trajectory*> many, singles;
            for (Curve& c : curves)
                (c.kind == Kind::Single ? singles : many).push_back(c.traj.get());
            runner.advance(many, integrator, rhs, h);   // integrate (parallel) + upload
            if (cfg.singleMethod == kTaylor)
                runner.advance(singles, taylor, jet, h, cfg.maxStep);
            else
                runner.advance(singles, singleIntegrator, rhs, h);
        }

        // hover + click: a bead under the cursor is picked before anything else
        // (the focused bead is out of it: it no longer reacts to the mouse)
        hovered = handingOff ? -1 : io.WantCaptureMouse ? uiHovered : pickEquilibrium();
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse && !handingOff)
        {
            if (hovered >= 0)
                focusOn(hovered);
            else
                dropSingleUnderCursor();
        }
        uiHovered = -1;   // the UI re-reports it while a row is hovered
        // glow toward 1 while hovered, and steadily while focused; the focused bead also
        // shrinks so it doesn't fill the zoomed view
        float ease = 1.0f - std::exp(-12.0f * dt);
        float shrinkEase = 1.0f - std::exp(-4.0f * dt);
        for (int i = 0; i < eqCount; ++i)
        {
            EqView& e = eqs[i];
            bool lit = i == hovered || i == focused;
            e.glow += ((lit ? 1.0f : 0.0f) - e.glow) * ease;
            e.scale += ((i == focused ? kFocusedScale : 1.0f) - e.scale) * shrinkEase;
        }
        clock += dt;

        updateCamera(dt);
        sceneFade = currentSceneFade();

        if (arrowsDirty)
            rebuildArrows();

        drawSystemOverlay();
        drawEquilibriumLabels();
        drawTimeLabels();
        drawEquilibriumTooltip();

        // 0: supersampled screenshot of the scene alone (no panel, no LaTeX labels)
        if (!io.WantCaptureKeyboard &&
            (ImGui::IsKeyPressed(ImGuiKey_0, false) || ImGui::IsKeyPressed(ImGuiKey_Keypad0, false)))
            takeScreenshot();
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix();
        const float px = app->renderScale();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // equilibria first: opaque, so the blended glyphs and trails layer over them
        renderEquilibria(camera, view, projection, false);

        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setFloat("u_magScale", 1.0f / std::max(sampledMaxMag, 1e-6f));
        shader.setFloat("u_pointSize", cfg.particleSize * px);
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_alphaTail", 1.0f);
        shader.setInt("u_useFade", 0);
        shader.setInt("u_shadeMode", 0);
        shader.setInt("u_round", 0);
        shader.setInt("u_useRamp", 0);

        if (cfg.showAxes)
        {
            shader.setVec3("u_color", glm::vec3(0.35f, 0.35f, 0.42f));
            glLineWidth(1.0f * px);
            axisMesh->draw(GL_LINES);
        }

        if (cfg.showArrows && !glyphs.empty())
        {
            // glyphs blend but never write depth, so they cannot hide a trajectory
            glDepthMask(GL_FALSE);
            shader.setInt("u_useRamp", 1);
            shader.setInt("u_useFade", 1);
            shader.setFloat("u_alpha", cfg.arrowAlpha * sceneFade);
            shader.setFloat("u_alphaTail", cfg.arrowTailAlpha);
            glLineWidth(1.5f * px);
            glyphs.drawShafts(shader);
            shader.setInt("u_useFade", 0);
            shader.setInt("u_shadeMode", 1);
            glyphs.drawHeads(shader);
            shader.setInt("u_shadeMode", 0);
            shader.setInt("u_useRamp", 0);
            shader.setFloat("u_alpha", 1.0f);
            shader.setFloat("u_alphaTail", 1.0f);
            glDepthMask(GL_TRUE);
        }

        runner.beginDraw();
        for (const Curve& c : curves)
        {
            float k = c.kind == Kind::Single ? cfg.singleScale : 1.0f;
            if (cfg.showTrails)
            {
                glLineWidth(cfg.trailWidth * k * px);
                shader.setVec3("u_color", c.color);
                shader.setInt("u_useRamp", cfg.trailBySpeed ? 1 : 0);
                shader.setFloat("u_alpha", cfg.trailAlpha * sceneFade);
                c.traj->draw();
                shader.setInt("u_useRamp", 0);
            }
            shader.setFloat("u_alpha", sceneFade);
            shader.setVec3("u_color", c.traj->diverged() ? kDivergedColor : c.color);
            shader.setFloat("u_pointSize", cfg.particleSize * k * px);
            shader.setInt("u_round", 1);
            c.traj->drawHead();
            shader.setInt("u_round", 0);
        }
        shader.setFloat("u_alpha", 1.0f);
        runner.endDraw();
        glLineWidth(1.0f);

        // halos last: additive light that brightens whatever is around the bead
        renderEquilibria(camera, view, projection, true);
        glDisable(GL_BLEND);
    }

    void drawUI() override
    {
        drawSystemUI();
        drawEquilibriaUI();
        drawFlowUI();
        drawBallUI();
        drawSinglesUI();

        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.36f, 0.30f, 1.0f));
        bool showOther = ImGui::CollapsingHeader("Other settings (camera, integrator, LaTeX, equilibria, arrows, trails, performance)");
        ImGui::PopStyleColor();
        if (showOther)
        {
            ImGui::Indent();
            drawViewUI();
            drawIntegratorUI();
            drawLatexUI();
            drawEquilibriumLookUI();
            drawArrowUI();
            drawTrailUI();
            sectionHeader("Performance");
            runner.drawUI();
            ImGui::Unindent();
        }

        sectionHeader("Settings");
        if (ImGui::Button("Save settings"))
            settingsStatus = saveCurrentSettings() ? "saved" : "could not write settings file";
        ImGui::SameLine();
        ImGui::TextDisabled("(parameters, controls and global camera)");
        if (!settingsStatus.empty())
            ImGui::TextDisabled("%s", settingsStatus.c_str());

        sectionHeader("Screenshot");
        ImGui::SliderInt("resolution scale", &shotScale, 1, 4, "%dx window");
        if (ImGui::Button("Take screenshot"))
            takeScreenshot();
        ImGui::SameLine();
        ImGui::TextDisabled("(or press 0)");
        if (!shotStatus.empty())
            ImGui::TextDisabled("%s", shotStatus.c_str());
    }

private:
    // Scene only, at shotScale x the window, into <repo>/screenshots/ with a timestamp.
    void takeScreenshot()
    {
        std::time_t now = std::time(nullptr);
        std::tm local{};
        localtime_s(&local, &now);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
        std::string path = std::string(SIMS_DIR "../screenshots/lorenz_flow_") + stamp + ".png";
        shotStatus = app->captureScreenshot(path, shotScale)
            ? "saved screenshots/" + std::filesystem::path(path).filename().string()
            : "screenshot failed (see console)";
    }

    enum class Kind { Ball, Single };

    struct Curve {
        std::unique_ptr<engine::Trajectory> traj;
        glm::vec3 seed;
        double t0;          // field time it was seeded at: phi's t = time - t0
        glm::vec3 color;
        Kind kind;
        bool withBall = false;   // the single at the ball's centre: goes with it
    };

    // Pieces of the \phi^t(x_0) label, each typeset once and re-composed every frame, so
    // a t that changes every frame costs no typesetting (as in vector_field).
    struct TimeLabelGlyphs {
        float size = 0.0f;
        engine::LatexImage phi;
        engine::LatexImage seedSymbol;   // (x_0)
        engine::LatexImage chars[12];    // superscript pieces, in kTimeLabelChars order
    };
    static constexpr const char* kTimeLabelChars = "0123456789.-";

    // One equilibrium as drawn: where it is, its linearization, its hover state.
    struct EqView {
        engine::LorenzSystem::Equilibrium eq;
        glm::vec3 pos{0.0f};
        engine::Eigen3 eig;
        std::string kind;
        float glow = 0.0f;              // eased toward 1 while hovered or focused
        float scale = 1.0f;             // eased toward kFocusedScale while focused
        engine::LatexImage label;       // O, C_+, C_-
        std::string labelTex;
        float labelSize = 0.0f;
    };

    // The eased camera move: centre and radius from -> to (radius interpolated in log,
    // so a 10x zoom feels uniform), orbit angles left to the user.
    struct CameraTween {
        bool active = false;
        float t = 0.0f, duration = 1.0f;
        glm::vec3 c0{0.0f}, c1{0.0f};
        float r0 = 1.0f, r1 = 1.0f;
    };

    // ---- UI sections ------------------------------------------------------------

    void drawSystemUI()
    {
        sectionHeader("Lorenz system");
        bool changed = false;
        changed |= ImGui::DragFloat("sigma", &cfg.sigma, 0.05f, 0.01f, 100.0f, "%.3f");
        changed |= ImGui::DragFloat("rho", &cfg.rho, 0.05f, 0.0f, 300.0f, "%.3f");
        changed |= ImGui::DragFloat("beta", &cfg.beta, 0.01f, 0.01f, 50.0f, "%.4f");
        if (ImGui::Button("Classic (10, 28, 8/3)"))
        {
            cfg.sigma = 10.0f;
            cfg.rho = 28.0f;
            cfg.beta = 8.0f / 3.0f;
            changed = true;
        }
        ImGui::SameLine();
        ImGui::Checkbox("show x' = f(x)", &cfg.showSystemLatex);
        if (changed)
            applyParams();
        ImGui::TextDisabled("field time t = %.3f", fieldTime);
    }

    void drawEquilibriaUI()
    {
        sectionHeader("Equilibria");
        for (int i = 0; i < eqCount; ++i)
        {
            const EqView& e = eqs[i];
            // hovering a row lights its bead
            ImGui::Text("%s%s", e.eq.name.c_str(), focused == i ? "   [focused]" : "");
            if (ImGui::IsItemHovered())
                uiHovered = i;
        }
        if (focused >= 0)
        {
            if (ImGui::Button("Back to global view"))
                backToGlobal();
            ImGui::SameLine();
            ImGui::TextDisabled("(Backspace)");
            ImGui::SameLine();
            if (ImGui::Button("Ball here"))
                dropBallOnFocused();
        }
    }

    // Look-and-feel of the beads and the zoom, under Other settings.
    void drawEquilibriumLookUI()
    {
        sectionHeader("Equilibria");
        ImGui::SliderFloat("zoom distance", &cfg.zoomRadius, 1.0f, 60.0f, "%.1f");
        ImGui::SliderFloat("zoom duration", &cfg.zoomSeconds, 0.2f, 4.0f, "%.1f s");
        ImGui::SliderFloat("bead size", &cfg.eqRadius, 0.1f, 3.0f, "%.2f");
        ImGui::Checkbox("labels", &cfg.showEqLabels);
    }

    // Sizes of everything typeset: the system top-right, the O / C+- labels, phi^t(x0).
    void drawLatexUI()
    {
        sectionHeader("LaTeX");
        if (ImGui::SliderFloat("equation", &cfg.systemLatexSize, 14.0f, 72.0f, "%.0f px"))
            latexDirty = true;
        ImGui::SliderFloat("equilibrium labels", &cfg.eqLabelSize, 14.0f, 72.0f, "%.0f px");
        ImGui::SliderFloat("phi^t labels", &cfg.timeLabelSize, 14.0f, 72.0f, "%.0f px");
    }

    void drawFlowUI()
    {
        sectionHeader("Flow");
        if (ImGui::Button(playing ? "Pause" : "Play"))
            playing = !playing;
        ImGui::SameLine();
        ImGui::TextDisabled("%s", engine::FlowKeys::kHint);
        ImGui::SameLine();
        ImGui::Checkbox("reverse", &cfg.reverse);
        ImGui::SameLine();
        if (ImGui::Button("Restart"))
            restartTrajectories();
        ImGui::SameLine();
        if (ImGui::Button("Clear all"))
            curves.clear();
        ImGui::SliderFloat("speed", &cfg.timeScale, 0.0f, 10.0f, "%.2f x");
        int diverged = 0;
        for (const Curve& c : curves)
            diverged += c.traj->diverged() ? 1 : 0;
        ImGui::TextDisabled("%d trajectories total", (int)curves.size());
        if (diverged)
        {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(kDivergedColor.r, kDivergedColor.g, kDivergedColor.b,
                                      1.0f), "(%d blew up)", diverged);
        }
    }

    void drawBallUI()
    {
        sectionHeader("Ball of trajectories");
        ImGui::SetNextItemWidth(220.0f);
        ImGui::InputFloat3("center", cfg.ballCenter);
        ImGui::SliderFloat("radius", &cfg.ballRadius, 0.01f, 20.0f, "%.2f");
        ImGui::SliderInt("particles", &cfg.ballCount, 1, 1000);
        ImGui::Checkbox("fill volume", &cfg.ballFilled);
        ImGui::SameLine();
        ImGui::Checkbox("single at center", &cfg.ballWithSingle);
        if (ImGui::Button("Drop ball"))
            dropBall();
        ImGui::SameLine();
        if (ImGui::Button("Remove ball"))
            removeBall();
    }

    void drawSinglesUI()
    {
        sectionHeader("Single trajectories");
        ImGui::TextDisabled("click empty space to drop one under the cursor");
        ImGui::SliderFloat("click depth", &cfg.spawnDepth, 0.05f, 1.0f,
                           "%.2f of the way to the orbit center");
        ImGui::SliderFloat("singles thickness", &cfg.singleScale, 1.0f, 4.0f, "%.1f x");
        ImGui::Checkbox("label with phi^t(x0)", &cfg.showTimeLabels);
        if (ImGui::Button("Clear singles"))
            removeKind(Kind::Single);
        ImGui::SameLine();
        ImGui::Checkbox("camera follows the newest", &cfg.followSingle);
    }

    void drawArrowUI()
    {
        sectionHeader("Arrow field");
        ImGui::Checkbox("show arrows", &cfg.showArrows);
        ImGui::SameLine();
        ImGui::Checkbox("axes", &cfg.showAxes);

        // a big lattice rebuilds on slider release, a small one live
        bool editedLive = false, editedDone = false;
        auto track = [&](bool changed) {
            editedLive = editedLive || changed;
            editedDone = editedDone || ImGui::IsItemDeactivatedAfterEdit();
        };
        track(ImGui::SliderInt("grid N", &cfg.gridN, 2, 50));
        track(ImGui::SliderFloat("arrow scale", &cfg.arrowScale, 0.1f, 1.5f));
        ImGui::TextUnformatted("head:");
        ImGui::SameLine();
        int style = cfg.barbedHeads ? 1 : 0;
        ImGui::RadioButton("solid", &style, 0);
        ImGui::SameLine();
        ImGui::RadioButton("barbs", &style, 1);
        if ((style == 1) != cfg.barbedHeads)
        {
            cfg.barbedHeads = style == 1;
            arrowsDirty = true;
        }
        if (!cfg.barbedHeads)
        {
            track(ImGui::SliderFloat("head length", &cfg.headFraction, 0.1f, 0.7f,
                                     "%.2f of arrow"));
            track(ImGui::SliderFloat("head width", &cfg.headWidth, 0.04f, 0.5f,
                                     "%.2f of head length"));
        }
        arrowsDirty |= ImGui::Checkbox("length by magnitude", &cfg.lengthByMagnitude);
        track(ImGui::SliderFloat("hide below", &cfg.hideBelow, 0.0f, 0.5f, "%.3f of max"));
        track(ImGui::DragFloat("domain radius", &cfg.domainR, 0.2f, 0.5f, 200.0f));
        track(ImGui::DragFloat("domain center z", &cfg.domainZ, 0.2f, -200.0f, 200.0f));
        ImGui::SliderFloat("arrow alpha", &cfg.arrowAlpha, 0.05f, 1.0f);
        ImGui::SliderFloat("alpha gradient", &cfg.arrowTailAlpha, 0.0f, 1.0f, "tail %.2f of tip");
        if (editedDone || (editedLive && glyphs.arrows() <= kLiveRebuildArrows))
            arrowsDirty = true;
        ImGui::TextDisabled("sampled max |f| = %.4g   arrows = %d", sampledMaxMag, glyphs.arrows());
        ImGui::TextDisabled("last rebuild %.2f ms", rebuildMs);
    }

    void drawTrailUI()
    {
        sectionHeader("Trails & particles");
        ImGui::Checkbox("show trails", &cfg.showTrails);
        ImGui::SameLine();
        ImGui::Checkbox("color by speed", &cfg.trailBySpeed);
        if (ImGui::SliderInt("trail points", &cfg.trailPoints, 32, 20000))
            for (Curve& c : curves)
                c.traj->setMaxPoints(cfg.trailPoints);
        ImGui::SliderFloat("trail width", &cfg.trailWidth, 1.0f, 6.0f);
        ImGui::SliderFloat("trail alpha", &cfg.trailAlpha, 0.05f, 1.0f);
        ImGui::SliderFloat("particle size", &cfg.particleSize, 2.0f, 20.0f);
    }

    // The many curves keep a cheap method; singles (few, and the ones you follow) may
    // use the Taylor method, whose order and tolerance then appear. For Taylor, max step
    // h only spaces the trail points sampled from its polynomial.
    void drawIntegratorUI()
    {
        sectionHeader("Integrator");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::Combo("ball", &cfg.method, "Euler\0RK2 (midpoint)\0RK4\0");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::Combo("singles", &cfg.singleMethod, "Euler\0RK2 (midpoint)\0RK4\0Taylor\0");
        if (cfg.singleMethod == kTaylor)
        {
            ImGui::SetNextItemWidth(160.0f);
            if (ImGui::InputInt("order", &cfg.taylorOrder))
                cfg.taylorOrder = std::clamp(cfg.taylorOrder, 2, engine::TaylorIntegrator::kMaxOrder);
            ImGui::SetNextItemWidth(160.0f);
            ImGui::SliderInt("tolerance", &cfg.taylorTolExp, 4, 16, "1e-%d");
        }
        ImGui::SetNextItemWidth(160.0f);
        ImGui::DragFloat("max step h", &cfg.maxStep, 0.0005f, 1e-5f, 0.5f, "%.4f");
        cfg.maxStep = std::clamp(cfg.maxStep, 1e-5f, 0.5f);
    }

    void drawViewUI()
    {
        sectionHeader("Camera");
        ImGui::TextDisabled("A/D orbit, W/S tilt, Q/E zoom");
        ImGui::SliderFloat("orbit speed", &cfg.orbitSpeed, 0.1f, 4.0f, "%.2f rad/s");
        ImGui::SliderFloat("zoom speed", &cfg.zoomSpeed, 0.0f, std::max(20.0f, cfg.domainR * 2.0f),
                           cfg.zoomSpeed > 0.0f ? "%.1f /s" : "auto");
        if (ImGui::Button("Reset camera"))
        {
            cfg.camRadius = 0.0f;
            cfg.camTheta = 0.6f;
            cfg.camPhi = 1.1f;
            cfg.camCenter[0] = cfg.camCenter[1] = 0.0f;
            cfg.camCenter[2] = cfg.domainZ;
            focused = -1;
            tween.active = false;
            handingOff = returning = false;
            restoreBallSettings();
            applyCamera();
        }
    }

    // ---- equilibria: picking, glow, labels, zoom ---------------------------------

    // New sigma/rho/beta: the field, the equilibria and their linearizations.
    void applyParams()
    {
        system.sigma = cfg.sigma;
        system.rho = cfg.rho;
        system.beta = cfg.beta;
        arrowsDirty = true;
        latexDirty = true;

        std::vector<engine::LorenzSystem::Equilibrium> found = system.equilibria();
        eqCount = (int)std::min(found.size(), eqs.size());
        for (int i = 0; i < eqCount; ++i)
        {
            EqView& e = eqs[i];
            e.eq = found[i];
            e.pos = glm::vec3((float)e.eq.x[0], (float)e.eq.x[1], (float)e.eq.x[2]);
            double J[3][3];
            system.jacobian(e.eq.x, J);
            e.eig.compute(J);
            e.kind = classify(e.eig);
            std::string tex = e.eq.name == "O" ? "O" : e.eq.name == "C+" ? "C_+" : "C_-";
            if (tex != e.labelTex)
            {
                e.labelTex = tex;
                e.labelSize = 0.0f;   // re-typeset
            }
        }
        for (int i = eqCount; i < (int)eqs.size(); ++i)
            eqs[i].glow = 0.0f;
        if (focused >= eqCount)
            backToGlobal();   // C+- merged into the origin (rho <= 1)
    }

    // The bead under the cursor (nearest along the ray), or -1. The pick radius never
    // drops below ~2.5% of the camera distance, so a far bead stays easy to hit.
    int pickEquilibrium() const
    {
        glm::vec3 origin, dir;
        if (!cursorRay(origin, dir))
            return -1;
        const glm::vec3 eye = app->camera().position;
        int best = -1;
        float bestT = 1e30f;
        for (int i = 0; i < eqCount; ++i)
        {
            if (i == focused)
                continue;   // zoomed onto it: it ignores the mouse
            float R = std::max(1.6f * cfg.eqRadius, 0.025f * glm::length(eqs[i].pos - eye));
            glm::vec3 oc = origin - eqs[i].pos;
            float b = glm::dot(oc, dir);
            float disc = b * b - (glm::dot(oc, oc) - R * R);
            if (disc < 0.0f)
                continue;
            float t = -b - std::sqrt(disc);
            if (t < 0.0f)
                t = -b + std::sqrt(disc);
            if (t > 0.0f && t < bestT)
            {
                bestT = t;
                best = i;
            }
        }
        return best;
    }

    bool cursorRay(glm::vec3& origin, glm::vec3& dir) const
    {
        glm::vec2 ndc = app->window().cursorPosNormalized() * 2.0f - 1.0f;
        const engine::Camera& cam = app->camera();
        glm::mat4 invVP = glm::inverse(cam.getProjectionMatrix() * cam.getViewMatrix());
        glm::vec4 nearH = invVP * glm::vec4(ndc, -1.0f, 1.0f);
        glm::vec4 farH = invVP * glm::vec4(ndc, 1.0f, 1.0f);
        if (nearH.w == 0.0f || farH.w == 0.0f)
            return false;
        origin = glm::vec3(nearH) / nearH.w;
        dir = glm::normalize(glm::vec3(farH) / farH.w - origin);
        return true;
    }

    // Ease the orbit centre onto x* and the distance down to zoomRadius. The global
    // view is remembered on the way in, so "back" returns exactly where you were.
    void focusOn(int i)
    {
        if (i < 0 || i >= eqCount)
            return;
        const engine::Camera& cam = app->camera();
        if (focused < 0)
        {
            globalCenter = cam.orbitCenter;
            globalRadius = cam.orbitRadius;
        }
        focused = i;
        handingOff = app->hasSim(kEquilibriumSim);
        startTween(eqs[i].pos, cfg.zoomRadius);
    }

    // The zoom onto a bead has landed: switch to lorenz_equilibrium at x*, handing it
    // the parameters, the bead's look and the global view to come back to. The camera
    // is shared and left where it is, so the switch shows no jump.
    void handOff()
    {
        engine::App::Handoff out{
            {"sigma", cfg.sigma}, {"rho", cfg.rho}, {"beta", cfg.beta},
            {"eq", focused},
            {"beadRadius", cfg.eqRadius * kFocusedScale},
            {"globalCx", globalCenter.x}, {"globalCy", globalCenter.y},
            {"globalCz", globalCenter.z}, {"globalR", globalRadius},
        };
        if (!app->requestSwitch(kEquilibriumSim, std::move(out)))
            handingOff = false;
    }

    // Opacity of everything but the focused bead: fading out over the end of the zoom
    // into a handoff, back in over the zoom out after returning.
    float currentSceneFade() const
    {
        float s = tween.active ? tween.t / tween.duration : 1.0f;
        if (handingOff)
            return 1.0f - smoothstep(0.45f, 0.95f, s);
        if (returning && tween.active)
            return smoothstep(0.05f, 0.6f, s);
        return 1.0f;
    }

    void backToGlobal()
    {
        if (focused < 0)
            return;
        focused = -1;
        handingOff = false;
        restoreBallSettings();
        startTween(globalCenter, globalRadius);
    }

    // "Ball here": the ball's centre (and a radius that fits the zoomed view) are set
    // to the focused bead for the drop, remembering the global ones — back in the
    // global view, Drop ball uses those again.
    void dropBallOnFocused()
    {
        if (focused < 0)
            return;
        if (!ballMoved)
        {
            ballMoved = true;
            savedBallCenter = glm::vec3(cfg.ballCenter[0], cfg.ballCenter[1], cfg.ballCenter[2]);
            savedBallRadius = cfg.ballRadius;
        }
        for (int k = 0; k < 3; ++k)
            cfg.ballCenter[k] = eqs[focused].pos[k];
        cfg.ballRadius = std::min(cfg.ballRadius, 0.5f * cfg.zoomRadius);
        dropBall();
    }

    void restoreBallSettings()
    {
        if (!ballMoved)
            return;
        ballMoved = false;
        for (int k = 0; k < 3; ++k)
            cfg.ballCenter[k] = savedBallCenter[k];
        cfg.ballRadius = savedBallRadius;
    }

    void startTween(const glm::vec3& center, float radius)
    {
        const engine::Camera& cam = app->camera();
        tween.active = true;
        tween.t = 0.0f;
        tween.duration = cfg.zoomSeconds;
        tween.c0 = cam.orbitCenter;
        tween.r0 = std::max(cam.orbitRadius, 1e-3f);
        tween.c1 = center;
        tween.r1 = std::max(radius, 1e-3f);
    }

    // The newest single, while "camera follows a single" is on and no bead is focused.
    const Curve* followedSingle() const
    {
        if (!cfg.followSingle || focused >= 0)
            return nullptr;
        for (auto it = curves.rbegin(); it != curves.rend(); ++it)
            if (it->kind == Kind::Single)
                return &*it;
        return nullptr;
    }

    // Runs after Camera::processInput, so the orbit position is recomputed here from the
    // (possibly tweened) centre and radius: the frame shows this frame's camera.
    void updateCamera(float dt)
    {
        engine::Camera& cam = app->camera();
        cam.orbitAngularSpeed = cfg.orbitSpeed;
        cam.speed = cfg.zoomSpeed > 0.0f ? cfg.zoomSpeed : std::max(1.0f, 0.5f * cam.orbitRadius);

        // follow a single: ease onto its head when it starts, back to where the camera was
        // when it stops (a focused bead takes over while zoomed)
        const Curve* followed = followedSingle();
        if ((followed != nullptr) != following)
        {
            following = followed != nullptr;
            if (following)
            {
                followReturn = cam.orbitCenter;
                startTween(followed->traj->head(), cam.orbitRadius);
            }
            else if (focused < 0)
                startTween(followReturn, cam.orbitRadius);
        }

        if (tween.active)
        {
            tween.t += dt;
            float s = smootherstep(tween.t / tween.duration);
            // the target may move while we fly (rho dragged, the followed head running):
            // aim at where it is now
            glm::vec3 target = focused >= 0 ? eqs[focused].pos
                             : following    ? followed->traj->head()
                                            : tween.c1;
            cam.orbitCenter = glm::mix(tween.c0, target, s);
            cam.orbitRadius = tween.r0 * std::pow(tween.r1 / tween.r0, s);
            if (tween.t >= tween.duration)
            {
                tween.active = false;
                returning = false;
                if (handingOff && focused >= 0)
                    handOff();
            }
        }
        else if (focused >= 0)
        {
            cam.orbitCenter = eqs[focused].pos;   // stay on it while parameters change
        }
        else if (following)
        {
            cam.orbitCenter = followed->traj->head();
        }
        else
            return;

        cam.position = cam.orbitCenter + cam.orbitRadius *
                       glm::vec3(std::sin(cam.orbitPhi) * std::sin(cam.orbitTheta),
                                 std::cos(cam.orbitPhi),
                                 std::sin(cam.orbitPhi) * std::cos(cam.orbitTheta));
        cam.front = glm::normalize(cam.orbitCenter - cam.position);
    }

    // Solid beads (halo = false) or their additive glow shells (halo = true).
    void renderEquilibria(const engine::Camera& camera, const glm::mat4& view,
                          const glm::mat4& projection, bool halo)
    {
        if (eqCount == 0)
            return;
        eqShader.use();
        eqShader.setMat4("view", view);
        eqShader.setMat4("projection", projection);
        eqShader.setVec3("u_eye", camera.position);
        eqShader.setInt("u_mode", halo ? 1 : 0);
        if (halo)
        {
            glBlendFunc(GL_ONE, GL_ONE);
            glDepthMask(GL_FALSE);
        }
        for (int i = 0; i < eqCount; ++i)
        {
            const EqView& e = eqs[i];
            if (halo && e.glow < 0.01f)
                continue;
            float pulse = 1.0f + 0.12f * std::sin(6.0f * clock);
            eqShader.setVec3("u_center", e.pos);
            eqShader.setFloat("u_radius", cfg.eqRadius * e.scale * (halo ? 2.2f * pulse : 1.0f));
            eqShader.setVec3("u_color", halo ? kHaloColor : kEquilibriumColor);
            eqShader.setFloat("u_glow", e.glow * (halo ? pulse : 1.0f));
            eqShader.setFloat("u_alpha", i == focused ? 1.0f : sceneFade);
            sphereMesh->draw(GL_TRIANGLES);
        }
        if (halo)
        {
            glDepthMask(GL_TRUE);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
    }

    // O, C_+, C_- beside each bead, typeset once per size.
    void drawEquilibriumLabels()
    {
        if (!cfg.showEqLabels)
            return;
        for (int i = 0; i < eqCount; ++i)
        {
            EqView& e = eqs[i];
            if (e.labelSize != cfg.eqLabelSize)
            {
                e.labelSize = cfg.eqLabelSize;
                e.label.set(e.labelTex, cfg.eqLabelSize, glm::vec3(1.0f, 0.45f, 0.40f));
            }
            float sx, sy;
            if (!toScreen(e.pos, sx, sy))
                continue;
            e.label.drawAt(sx + 10.0f + 0.3f * cfg.eqLabelSize,
                           sy - (float)e.label.height() - 2.0f,
                           (0.75f + 0.25f * e.glow) * (i == focused ? 1.0f : sceneFade));
        }
    }

    // Hovering a bead: its point, the linearization's eigenvalues and type.
    void drawEquilibriumTooltip()
    {
        if (hovered < 0 || ImGui::GetIO().WantCaptureMouse)
            return;
        const EqView& e = eqs[hovered];
        ImGui::BeginTooltip();
        ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.40f, 1.0f), "%s = (%.4f, %.4f, %.4f)",
                           e.eq.name.c_str(), e.pos.x, e.pos.y, e.pos.z);
        ImGui::TextUnformatted(e.kind.c_str());
        for (const auto& l : e.eig.values())
        {
            if (std::abs(l.imag()) < 1e-12)
                ImGui::Text("  lambda = %.6f", l.real());
            else
                ImGui::Text("  lambda = %.6f %c %.6f i", l.real(), l.imag() < 0 ? '-' : '+',
                            std::abs(l.imag()));
        }
        ImGui::TextDisabled("click to zoom in");
        ImGui::EndTooltip();
    }

    // ---- overlays -------------------------------------------------------------------

    // \phi^t(x_0) above every single: the flow map for the ODE time integrated since it
    // was dropped (negative when running in reverse). Same composition as vector_field.
    void drawTimeLabels()
    {
        if (!cfg.showTimeLabels)
            return;
        TimeLabelGlyphs& g = timeGlyphs;
        const float S = cfg.timeLabelSize;
        if (g.size != S)
        {
            // TeX proportions: superscripts at scriptstyle (0.7) raised ~0.42 em
            g.size = S;
            g.phi.set(R"(\phi)", S, kSingleColor);
            g.seedSymbol.set(R"((x_0))", S, kSingleColor);
            for (int i = 0; i < 12; ++i)
            {
                char ch = kTimeLabelChars[i];
                g.chars[i].set(ch == '-' ? std::string("{-}") : std::string(1, ch),
                               0.7f * S, kSingleColor);
            }
        }
        const float raise = 0.42f * S;
        const float pad = (float)engine::LatexImage::kPad;

        for (const Curve& c : curves)
        {
            float sx, sy;
            if (c.kind != Kind::Single || !toScreen(c.traj->head(), sx, sy))
                continue;
            double t = c.traj->time() - c.t0;
            if (std::abs(t) < 0.005)
                t = 0.0; // no "-0.00"
            char num[32];
            std::snprintf(num, sizeof(num), "%.2f", t);

            // one baseline: \phi, raised digits, (x_0)
            const engine::LatexImage* pieces[40];
            float xs[40], ys[40];
            int n = 0;
            float x = 0.0f;
            auto place = [&](const engine::LatexImage& img, float lift) {
                pieces[n] = &img;
                xs[n] = x - pad;
                ys[n] = -lift - img.baseline();
                ++n;
                x += img.advance();
            };
            place(g.phi, 0.0f);
            x += 0.04f * S;
            for (const char* p = num; *p && n < 38; ++p)
                if (const char* at = std::strchr(kTimeLabelChars, *p))
                    place(g.chars[at - kTimeLabelChars], raise);
            x += 0.06f * S;
            place(g.seedSymbol, 0.0f);

            float headRadius = 0.5f * cfg.particleSize * cfg.singleScale;
            float left = sx - 0.5f * x;
            float baseY = sy - headRadius - 0.35f * S;
            for (int i = 0; i < n; ++i)
                pieces[i]->drawAt(left + xs[i], baseY + ys[i], sceneFade);
        }
    }

    bool toScreen(const glm::vec3& p, float& sx, float& sy) const
    {
        const engine::Camera& cam = app->camera();
        glm::vec4 clip = cam.getProjectionMatrix() * cam.getViewMatrix() * glm::vec4(p, 1.0f);
        if (clip.w <= 1e-4f)
            return false;
        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        if (ndc.z < -1.0f || ndc.z > 1.0f)
            return false;
        ImVec2 display = ImGui::GetIO().DisplaySize;
        sx = (ndc.x * 0.5f + 0.5f) * display.x;
        sy = (0.5f - ndc.y * 0.5f) * display.y;
        return true;
    }

    // The system with its live parameters, top-right.
    void drawSystemOverlay()
    {
        if (!cfg.showSystemLatex)
            return;
        if (latexDirty)
        {
            latexDirty = false;
            std::string s = texNumber(cfg.sigma), r = texNumber(cfg.rho), b = texNumber(cfg.beta);
            systemLatex.set(R"(\begin{pmatrix}\dot{x} \\ \dot{y} \\ \dot{z}\end{pmatrix} = )"
                            R"(\begin{pmatrix})" + s + R"((y - x) \\ x()" + r +
                            R"( - z) - y \\ x y - )" + b + R"(z\end{pmatrix})", cfg.systemLatexSize);
        }
        if (systemLatex.empty())
            return;
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12.0f,
                                       vp->WorkPos.y + 12.0f),
                                ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.35f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##lorenzLatex", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
        systemLatex.draw();
        ImGui::End();
        ImGui::PopStyleVar();
    }

    // ---- view + settings ------------------------------------------------------------

    void applyCamera()
    {
        engine::Camera& cam = app->camera();
        cam.orbitTheta = cfg.camTheta;
        cam.orbitPhi = cfg.camPhi;
        cam.orbitRadius = cfg.camRadius > 0.0f ? cfg.camRadius : 3.0f * cfg.domainR;
        cam.orbitCenter = glm::vec3(cfg.camCenter[0], cfg.camCenter[1], cfg.camCenter[2]);
    }

    // The global view is what gets saved — while zoomed in, the one remembered on the
    // way in.
    bool saveCurrentSettings()
    {
        const engine::Camera& cam = app->camera();
        bool zoomed = focused >= 0 || (tween.active && !following);
        glm::vec3 c = zoomed ? globalCenter : following ? followReturn : cam.orbitCenter;
        cfg.camTheta = cam.orbitTheta;
        cfg.camPhi = cam.orbitPhi;
        cfg.camRadius = zoomed ? globalRadius : cam.orbitRadius;
        cfg.camCenter[0] = c.x;
        cfg.camCenter[1] = c.y;
        cfg.camCenter[2] = c.z;
        return saveSettingsFile(cfg, kSettingsPath);
    }

    // ---- trajectories -------------------------------------------------------------

    // Under the cursor on a camera-facing plane spawnDepth of the way to the orbit
    // centre, so it lands just in front of you.
    void dropSingleUnderCursor()
    {
        glm::vec3 origin, dir;
        if (!cursorRay(origin, dir))
            return;
        const engine::Camera& cam = app->camera();
        float denom = glm::dot(dir, cam.front);
        if (std::abs(denom) < 1e-4f)
            return;
        float toCenter = glm::dot(cam.orbitCenter - cam.position, cam.front);
        glm::vec3 planePoint = cam.position + cam.front * (cfg.spawnDepth * toCenter);
        float s = glm::dot(planePoint - origin, cam.front) / denom;
        if (s > 0.0f)
            addTrajectory(origin + dir * s, kSingleColor, Kind::Single);
    }

    void addTrajectory(const glm::vec3& seed, const glm::vec3& color, Kind kind)
    {
        if (curves.size() >= kMaxTrajectories)
            return;
        double s[3] = {seed.x, seed.y, seed.z};
        auto traj = std::make_unique<engine::Trajectory>(3, s, fieldTime, cfg.trailPoints);
        traj->minSpacing = 0.002f * cfg.domainR;
        curves.push_back({std::move(traj), seed, fieldTime, color, kind});
    }

    void removeKind(Kind kind)
    {
        std::erase_if(curves, [kind](const Curve& c) { return c.kind == kind; });
    }

    void removeBall()
    {
        std::erase_if(curves, [](const Curve& c) { return c.kind == Kind::Ball || c.withBall; });
    }

    // One ball at a time; dropping replaces it. Seeds on a Fibonacci sphere (the
    // clearest view of how the flow deforms it) or uniformly through the inside.
    void dropBall()
    {
        removeBall();
        glm::vec3 center(cfg.ballCenter[0], cfg.ballCenter[1], cfg.ballCenter[2]);
        // "single at center": a green single riding the middle of the ball, labelled and
        // followable like any click-dropped one, removed with the ball
        if (cfg.ballWithSingle)
        {
            addTrajectory(center, kSingleColor, Kind::Single);
            if (!curves.empty())
                curves.back().withBall = true;
        }
        std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
        const float golden = 2.39996323f;
        int n = cfg.ballCount;
        for (int k = 0; k < n && curves.size() < kMaxTrajectories; ++k)
        {
            glm::vec3 p(0.0f);
            if (cfg.ballFilled)
            {
                do
                    p = glm::vec3(unit(rng), unit(rng), unit(rng));
                while (glm::dot(p, p) > 1.0f);
            }
            else
            {
                float y = 1.0f - 2.0f * ((float)k + 0.5f) / (float)n;
                float r = std::sqrt(std::max(0.0f, 1.0f - y * y));
                float phi = golden * (float)k;
                p = glm::vec3(std::cos(phi) * r, y, std::sin(phi) * r);
            }
            addTrajectory(center + p * cfg.ballRadius, kBallColor, Kind::Ball);
        }
    }

    void restartTrajectories()
    {
        for (Curve& c : curves)
        {
            double s[3] = {c.seed.x, c.seed.y, c.seed.z};
            c.traj->reset(s, fieldTime);
            c.t0 = fieldTime;
        }
    }

    // ---- arrow grid ---------------------------------------------------------------

    // Sample the field on the domain cube (in parallel; LorenzSystem::eval is
    // stateless) and rebuild the instanced glyphs.
    void rebuildArrows()
    {
        arrowsDirty = false;
        auto t0 = std::chrono::steady_clock::now();
        gridPoints.clear();
        const float R = cfg.domainR;
        const float spacing = 2.0f * R / (float)cfg.gridN;
        for (int k = 0; k <= cfg.gridN; ++k)
            for (int j = 0; j <= cfg.gridN; ++j)
                for (int i = 0; i <= cfg.gridN; ++i)
                    gridPoints.emplace_back(-R + i * spacing, -R + j * spacing,
                                            cfg.domainZ - R + k * spacing);

        gridVectors.resize(gridPoints.size());
        std::transform(std::execution::par, gridPoints.begin(), gridPoints.end(),
                       gridVectors.begin(), [this](const glm::vec3& p) {
                           double x[3] = {p.x, p.y, p.z}, v[3];
                           system.eval(x, v);
                           return glm::vec3((float)v[0], (float)v[1], (float)v[2]);
                       });
        sampledMaxMag = 0.0f;
        for (const glm::vec3& v : gridVectors)
            sampledMaxMag = std::max(sampledMaxMag, glm::length(v));

        glyphs.clear();
        glyphs.barbs = cfg.barbedHeads;
        glyphs.headFraction = cfg.headFraction;
        glyphs.headWidth = cfg.headWidth;
        glyphs.addField(gridPoints, gridVectors, spacing * cfg.arrowScale,
                        std::max(sampledMaxMag, 1e-6f), cfg.hideBelow, cfg.lengthByMagnitude, 3);
        glyphs.upload();
        rebuildMs = std::chrono::duration<float, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();
    }

    static constexpr float kFocusedScale = 0.4f;   // focused bead size, of `bead size`
    static constexpr int kLiveRebuildArrows = 20000;
    static constexpr size_t kMaxTrajectories = 4000;

    engine::App* app = nullptr;
    engine::Shader shader;      // glyphs, trails, axes (vector_field's pair)
    engine::Shader eqShader;    // equilibrium beads + halos
    std::unique_ptr<engine::Mesh> axisMesh;
    std::unique_ptr<engine::Mesh> sphereMesh;
    engine::ArrowGlyphs glyphs;

    Settings cfg;
    std::string settingsStatus;
    int shotScale = 2;
    std::string shotStatus;

    engine::LorenzSystem system;
    engine::Integrator integrator;
    engine::Integrator::Field rhs;
    engine::Integrator singleIntegrator;   // singles, unless they use the Taylor method
    engine::TaylorIntegrator taylor;
    engine::TaylorIntegrator::Jet jet;     // LorenzSystem::jet
    std::vector<Curve> curves;
    double fieldTime = 0.0;
    bool playing = true;
    std::mt19937 rng{1234};

    std::array<EqView, 3> eqs;
    int eqCount = 0;
    int hovered = -1;           // bead under the cursor (or its UI row), this frame
    int uiHovered = -1;
    int focused = -1;           // bead the camera is zoomed onto
    float clock = 0.0f;         // drives the halo pulse
    CameraTween tween;
    glm::vec3 globalCenter{0.0f};
    float globalRadius = 90.0f;
    bool handingOff = false;          // zooming onto a bead to switch to lorenz_equilibrium
    bool returning = false;           // zooming out after coming back from it
    float sceneFade = 1.0f;           // currentSceneFade(), this frame
    bool following = false;           // the camera is tracking a single right now
    glm::vec3 followReturn{0.0f};     // orbit centre to ease back to when it stops
    engine::TrajectoryRunner runner;  // advances the curves, times integrate/upload/draw
    bool ballMoved = false;           // "Ball here" moved the ball; restored on leaving
    glm::vec3 savedBallCenter{0.0f};
    float savedBallRadius = 1.0f;
    TimeLabelGlyphs timeGlyphs;

    engine::LatexImage systemLatex;
    bool latexDirty = true;

    bool arrowsDirty = true;
    float sampledMaxMag = 0.0f;
    float rebuildMs = 0.0f;
    std::vector<glm::vec3> gridPoints;
    std::vector<glm::vec3> gridVectors;
};

} // namespace

std::unique_ptr<engine::Sim> makeLorenzFlowSim()
{
    return std::make_unique<LorenzFlowSim>();
}
