#include "LorenzManifoldGrowthSim.h"

#include <engine/App.h>
#include <engine/ArrowGlyphs.h>
#include <engine/Camera.h>
#include <engine/ConjugacyMap.h>
#include <engine/FlowKeys.h>
#include <engine/TrajectoryRunner.h>
#include <engine/Eigen3.h>
#include <engine/Integrator.h>
#include <engine/InvariantManifold.h>
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
#include <future>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

// ---- colors, small helpers ------------------------------------------------------

const glm::vec3 kBallColor(1.0f);
const glm::vec3 kSingleColor = {0.35f, 0.95f, 0.45f};   // click-dropped: always green
const glm::vec3 kDivergedColor = {0.95f, 0.25f, 0.25f}; // a head that blew up
const glm::vec3 kEquilibriumColor = {0.92f, 0.12f, 0.10f};
const glm::vec3 kHaloColor = {1.0f, 0.38f, 0.22f};
// the manifolds, the colors lorenz_equilibrium gives the same objects in V
const glm::vec3 kStableColor(0.45f, 0.72f, 1.0f);     // W^s
const glm::vec3 kUnstableColor(1.0f, 0.55f, 0.45f);   // W^u
const glm::vec3 kPopupTitleColor(1.0f, 0.45f, 0.40f);

const char* kSettingsPath = SIMS_DIR "lorenz_manifold_growth/settings.txt";

// How far the Taylor map is trusted: the largest |theta| where its relative invariance
// defect stays under kDefectOk (the same bar lorenz_equilibrium's check uses).
constexpr double kDefectOk = 1e-3;
constexpr double kRadiusSearchMax = 25.0;
constexpr int kCurveSteps = 160;          // points per half of a 1-D local manifold
constexpr int kRings = 28, kSegs = 96;    // the disk of a 2-D local manifold
constexpr int kLocalCapacity = kRings * kSegs * 6 + (kRings + 1) * kSegs + 8;
constexpr int kBranchPoints = 120000;     // a global branch keeps its whole curve
constexpr float kPopupGrace = 0.4f;       // seconds off bead and popup before it closes
constexpr float kReturnSeconds = 1.0f;    // camera ease back when a follow ends
// "Learn geodesic level sets": zoom onto the patch, then switch to that sim
const char* kLevelSetsSim = "learn_geodesic_level_sets";
constexpr float kLeaveSeconds = 1.2f;     // the zoom in (and back out on returning)
constexpr float kFramingScale = 3.2f;     // camera distance / patch radius there

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

// A shader source with the line holding `marker` replaced by `insert` (ConjugacyMap's
// GLSL goes into the surface vertex shader this way).
std::string loadWithInsert(const char* path, const char* marker, const std::string& insert)
{
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    std::string src = ss.str();
    size_t at = src.find(marker);
    if (at != std::string::npos)
    {
        size_t lineStart = src.rfind('\n', at);
        size_t lineEnd = src.find('\n', at);
        lineStart = lineStart == std::string::npos ? 0 : lineStart + 1;
        src.replace(lineStart, (lineEnd == std::string::npos ? src.size() : lineEnd) - lineStart,
                    insert);
    }
    return src;
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

// Quintic smoothstep: zero velocity and acceleration at both ends of a camera move.
float smootherstep(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (6.0f * t - 15.0f) + 10.0f);
}

constexpr int kTaylor = 3;   // singleMethod: Euler, RK2, RK4 (Integrator), then Taylor

// ---- settings -------------------------------------------------------------------

constexpr float kMinSpeed = 1e-4f;   // slowest flow (arrow keys ramp to it logarithmically)

struct Settings {
    float sigma = 10.0f;
    float rho = 28.0f;
    float beta = 8.0f / 3.0f;
    bool showSystemLatex = true;
    float systemLatexSize = 30.0f;  // the system overlay, px

    float eqRadius = 0.7f;        // marker size, world units
    bool showEqLabels = true;
    float eqLabelSize = 30.0f;
    float followSeconds = 0.8f;   // slide onto a newly followed head (0 = snap)

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

    float timeScale = 1.0f;       // flow speed, kMinSpeed..10 on a logarithmic scale
    bool reverse = false;
    int method = 2;               // Integrator::Method, RK4
    float maxStep = 0.005f;
    int singleMethod = 3;         // singles + manifold branches: Euler, RK2, RK4 or Taylor
    int taylorOrder = 20;
    int taylorTolExp = 14;        // Taylor tolerance 1e-taylorTolExp

    float ballCenter[3] = {1.0f, 1.0f, 25.0f};
    float ballRadius = 1.0f;
    int ballCount = 200;
    bool ballFilled = false;
    bool ballWithSingle = false;  // drop a followable single at the ball's centre too
    float eqBallRadius = 1.0f;    // popup balls: around x*, and on W_loc (capped at its r)
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

    int manifoldOrder = 12;       // K's Taylor order
    float locGrowSeconds = 1.5f;  // the local manifold grows out of x* over this
    float manifoldWidth = 3.0f;   // px, curves (local and global)
    float diskAlpha = 0.3f;       // a 2-D local manifold's fill
    float branchTime = 21.0f;     // a global branch flows this much field time, then stops
    float backSpeed = 15.0f;      // backward curves: arc speed cap, world units per field time
    float pauseBeyond = 12.5f;    // backward curves freeze this many domain radii out

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
        {"followSeconds", &s.followSeconds},
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
        {"eqBallRadius", &s.eqBallRadius},
        {"followSingle", nullptr, nullptr, &s.followSingle},
        {"spawnDepth", &s.spawnDepth}, {"singleScale", &s.singleScale},
        {"showTimeLabels", nullptr, nullptr, &s.showTimeLabels},
        {"timeLabelSize", &s.timeLabelSize},
        {"showTrails", nullptr, nullptr, &s.showTrails},
        {"trailBySpeed", nullptr, nullptr, &s.trailBySpeed},
        {"trailPoints", nullptr, &s.trailPoints},
        {"trailWidth", &s.trailWidth}, {"trailAlpha", &s.trailAlpha},
        {"particleSize", &s.particleSize},
        {"manifoldOrder", nullptr, &s.manifoldOrder},
        {"locGrowSeconds", &s.locGrowSeconds},
        {"manifoldWidth", &s.manifoldWidth},
        {"diskAlpha", &s.diskAlpha},
        {"branchTime", &s.branchTime}, {"backSpeed", &s.backSpeed},
        {"pauseBeyond", &s.pauseBeyond},
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
    s.timeScale = std::clamp(s.timeScale, kMinSpeed, 10.0f);
    s.domainR = std::clamp(s.domainR, 0.5f, 200.0f);
    s.method = std::clamp(s.method, 0, 2);
    s.systemLatexSize = std::clamp(s.systemLatexSize, 10.0f, 120.0f);
    s.singleMethod = std::clamp(s.singleMethod, 0, kTaylor);
    s.taylorOrder = std::clamp(s.taylorOrder, 2, engine::TaylorIntegrator::kMaxOrder);
    s.taylorTolExp = std::clamp(s.taylorTolExp, 4, 16);
    s.maxStep = std::clamp(s.maxStep, 1e-5f, 0.5f);
    s.followSeconds = std::clamp(s.followSeconds, 0.0f, 10.0f);
    s.manifoldOrder = std::clamp(s.manifoldOrder, 2, engine::ConjugacyMap::kMaxOrder);
    s.locGrowSeconds = std::clamp(s.locGrowSeconds, 0.05f, 10.0f);
    s.eqBallRadius = std::clamp(s.eqBallRadius, 0.01f, 20.0f);
    s.manifoldWidth = std::clamp(s.manifoldWidth, 1.0f, 8.0f);
    s.diskAlpha = std::clamp(s.diskAlpha, 0.02f, 1.0f);
    s.branchTime = std::clamp(s.branchTime, 0.1f, 1000.0f);
    s.backSpeed = std::clamp(s.backSpeed, 0.1f, 1000.0f);
    s.pauseBeyond = std::clamp(s.pauseBeyond, 0.5f, 50.0f);
}

// ---- the sim --------------------------------------------------------------------

// The global Lorenz flow (arrow field, balls and singles, the equilibria as red beads,
// as lorenz_flow) where the invariant manifolds of each equilibrium are grown: hover a
// bead and a popup offers, per side, "grow W_loc" (the Taylor patch of the
// parametrization method growing out of x*) and then "grow W" (the global manifold —
// for a curve, the flow carrying the ends of W_loc on; for a surface, not available
// until geodesic level sets are in). A watched branch carries the camera along.
// See obsidian_twin/sims/lorenz-manifold-growth.md.
class LorenzManifoldGrowthSim : public engine::Sim {
public:
    const char* name() const override { return "lorenz_manifold_growth"; }

    void init(engine::App& app) override
    {
        this->app = &app;
        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(SIMS_DIR "lorenz_manifold_growth/shaders/vertex.glsl",
                                SIMS_DIR "lorenz_manifold_growth/shaders/fragment.glsl");
        eqShader = engine::Shader(SIMS_DIR "lorenz_manifold_growth/shaders/equilibrium_vertex.glsl",
                                  SIMS_DIR "lorenz_manifold_growth/shaders/equilibrium_fragment.glsl");
        surfaceShader = engine::Shader::fromSources(
            loadWithInsert(SIMS_DIR "lorenz_manifold_growth/shaders/surface_vertex.glsl",
                           "@CONJUGACY_MAP", engine::ConjugacyMap::glsl()),
            loadWithInsert(SIMS_DIR "lorenz_manifold_growth/shaders/fragment.glsl", "@none", ""));
        surfaceShader.bindUniformBlock(engine::ConjugacyMap::kBlockName, engine::ConjugacyMap::kBinding);
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

        // back from learn_geodesic_level_sets: same parameters, what was grown, and the
        // camera still on the patch — then the eased zoom out to the view it left from
        const engine::App::Handoff& in = app.handoff();
        auto value = [&](const char* key, double fallback) {
            auto it = in.find(key);
            return it == in.end() ? fallback : it->second;
        };
        const bool back = in.count("fromLevelSets") > 0;
        if (back)
        {
            cfg.sigma = (float)value("sigma", cfg.sigma);
            cfg.rho = (float)value("rho", cfg.rho);
            cfg.beta = (float)value("beta", cfg.beta);
            cfg.manifoldOrder = std::clamp((int)value("order", cfg.manifoldOrder), 2, engine::ConjugacyMap::kMaxOrder);
        }
        else
            applyCamera();
        fitClipPlanes(cam);
        applyParams();
        computeManifoldsNow();   // at open the wait is part of loading
        if (back)
        {
            const int mask = (int)value("loc", 0.0);
            const int eq = (int)value("eq", 0.0), side = (int)value("side", 0.0);
            for (int i = 0; i < eqCount; ++i)
                for (int s = 0; s < 2; ++s)
                {
                    Side& S = eqs[i].side[s];
                    if (S.ok && (((mask >> (2 * i + s)) & 1) || (i == eq && s == side)))
                    {
                        S.local = true;
                        S.grow = 1.0f;
                    }
                }
            glm::vec3 center(cfg.camCenter[0], cfg.camCenter[1], cfg.camCenter[2]);
            float radius = cfg.camRadius > 0.0f ? cfg.camRadius : 3.0f * cfg.domainR;
            if (in.count("globalR"))
            {
                center = glm::vec3((float)value("globalCx", 0.0), (float)value("globalCy", 0.0),
                                   (float)value("globalCz", 0.0));
                radius = (float)value("globalR", radius);
            }
            focusEq = eq < eqCount ? eq : -1;
            focusSide = std::clamp(side, 0, 1);
            returning = true;
            sceneFade = 0.0f;
            startTween(center, radius);
            tween.duration = kLeaveSeconds;
        }
    }

    void update(float dt) override
    {
        // the first frame after init carries init's time (most of a second in debug): the
        // return zoom would jump past its start
        if (firstUpdate)
        {
            firstUpdate = false;
            dt = std::min(dt, 1.0f / 60.0f);
        }
        // Space toggles play/pause, unless ImGui has the keyboard
        const ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false))
            playing = !playing;
        // arrows: Up/Down speed (down to 0), Left reverse, Right forward
        engine::FlowKeys::handle(cfg.timeScale, cfg.reverse, dt, 10.0f, kMinSpeed);
        engine::FlowKeys::drawStatus(cfg.timeScale, cfg.reverse, playing);

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
            advanceCurves(many, h, false);   // integrate (parallel) + upload
            advanceCurves(singles, h, true);
            // the manifold branches: their direction is the manifold's own (forward on
            // W^u, backward on W^s), only the speed is the flow's; each side stops after
            // its flow time limit or while paused
            std::vector<engine::Trajectory*> forward, backward;
            for (int i = 0; i < eqCount; ++i)
                for (int side = 0; side < 2; ++side)
                {
                    Side& S = eqs[i].side[side];
                    if (!S.global || S.paused)
                        continue;
                    for (Branch& b : S.branch)
                        if (b.traj && std::abs(b.traj->time() - S.t0) < S.limit)
                            (side == kUnstable ? forward : backward).push_back(b.traj.get());
                }
            advanceCurves(forward, std::abs(h), true);
            advanceCurves(backward, -std::abs(h), true);
        }

        // hover: a bead under the cursor, unless the mouse is on the UI (the popup
        // included — its bead keeps glowing through popupEq)
        hovered = leaving ? -1 : io.WantCaptureMouse ? uiHovered : pickEquilibrium();
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse && !leaving)
        {
            if (hovered >= 0)
                togglePin(hovered);
            else
                dropSingleUnderCursor();
        }
        uiHovered = -1;   // the UI re-reports it while a row is hovered
        float ease = 1.0f - std::exp(-12.0f * dt);
        for (int i = 0; i < eqCount; ++i)
        {
            EqView& e = eqs[i];
            bool lit = i == hovered || i == popupEq;
            e.glow += ((lit ? 1.0f : 0.0f) - e.glow) * ease;
        }
        clock += dt;

        // the local manifolds grow out of x*
        for (int i = 0; i < eqCount; ++i)
            for (int side = 0; side < 2; ++side)
            {
                Side& S = eqs[i].side[side];
                if (eqs[i].ready && S.local && S.grow < 1.0f)
                    S.grow = std::min(1.0f, S.grow + dt / std::max(cfg.locGrowSeconds, 0.05f));
            }

        pollManifolds();

        updateCamera(dt);
        sceneFade = currentSceneFade();

        if (arrowsDirty)
            rebuildArrows();

        drawSystemOverlay();
        drawEquilibriumLabels();
        drawManifoldLabels();
        drawTimeLabels();
        drawManifoldPopup(dt);

        // 1 tapped: stop watching a branch if one is watched, else camera-follows-the-
        // newest-single on/off — detaching leaves the camera where it is (held past
        // App::kRecenterHoldSeconds, App calls recenterCamera instead); 2: arrow field
        if (!io.WantCaptureKeyboard &&
            (engine::FlowKeys::tapHold(ImGuiKey_1, dt, engine::App::kRecenterHoldSeconds).tapped ||
             engine::FlowKeys::tapHold(ImGuiKey_Keypad1, dt, engine::App::kRecenterHoldSeconds).tapped))
        {
            if (watchEq >= 0)
            {
                watchEq = -1;
                detachInPlace = true;
            }
            else
            {
                cfg.followSingle = !cfg.followSingle;
                detachInPlace = !cfg.followSingle;
            }
        }
        if (!io.WantCaptureKeyboard && (ImGui::IsKeyPressed(ImGuiKey_2, false) || ImGui::IsKeyPressed(ImGuiKey_Keypad2, false)))
            cfg.showArrows = !cfg.showArrows;
        // the arrow field fades in/out instead of cutting (key 2 or the checkbox)
        {
            const float target = cfg.showArrows ? 1.0f : 0.0f;
            if (arrowsFade < 0.0f)
                arrowsFade = target; // first frame: no fade from the saved state
            const float step = dt / kArrowFadeSeconds;
            arrowsFade += std::clamp(target - arrowsFade, -step, step);
        }
        // R: clear every trajectory (same as the "Clear all" button)
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_R, false))
            curves.clear();
        // B: drop a ball of trajectories (same as the button)
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_B, false))
            dropBall(glm::vec3(cfg.ballCenter[0], cfg.ballCenter[1], cfg.ballCenter[2]),
                     cfg.ballRadius, cfg.ballWithSingle, true);
        // V: a ball with a single at its centre, where a click would seed
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_V, false))
        {
            glm::vec3 p;
            if (cursorSeedPoint(p))
                dropBall(p, viewFitRadius(p), true, false);
        }

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
            shader.setFloat("u_alpha", sceneFade);
            glLineWidth(1.0f * px);
            axisMesh->draw(GL_LINES);
            shader.setFloat("u_alpha", 1.0f);
        }

        if (arrowsFade * sceneFade > 0.0f && !glyphs.empty())
        {
            // glyphs blend but never write depth, so they cannot hide a trajectory
            glDepthMask(GL_FALSE);
            shader.setInt("u_useRamp", 1);
            shader.setInt("u_useFade", 1);
            shader.setFloat("u_alpha", cfg.arrowAlpha * arrowsFade * sceneFade);
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

        renderLocalManifolds(view, projection, px);
        shader.use();

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
        renderBranches(px);
        shader.setFloat("u_alpha", 1.0f);
        runner.endDraw();
        glLineWidth(1.0f);

        // halos last: additive light that brightens whatever is around the bead
        renderEquilibria(camera, view, projection, true);
        glDisable(GL_BLEND);
    }

    // Hold 1 (App::kRecenterHoldSeconds): glide to the global home view, every follow off.
    void recenterCamera() override
    {
        cfg.followSingle = false;
        watchEq = -1;
        following = false;
        followedTraj = nullptr;
        detachInPlace = false;
        cfg.camRadius = 0.0f;
        cfg.camTheta = 0.6f;
        cfg.camPhi = 1.1f;
        cfg.camCenter[0] = cfg.camCenter[1] = 0.0f;
        cfg.camCenter[2] = cfg.domainZ;
        tween.active = false;
        app->camera().easeOrbitTo(glm::vec3(cfg.camCenter[0], cfg.camCenter[1], cfg.camCenter[2]),
                                  3.0f * cfg.domainR, 0.6f, 1.1f,
                                  engine::App::kRecenterEaseSeconds);
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
        bool showOther = ImGui::CollapsingHeader("Other settings (camera, integrator, manifolds, LaTeX, equilibria, arrows, trails, performance)");
        ImGui::PopStyleColor();
        if (showOther)
        {
            ImGui::Indent();
            drawViewUI();
            drawIntegratorUI();
            drawManifoldSettingsUI();
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
        ImGui::TextDisabled("(parameters, controls and camera)");
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
        std::string path = std::string(SIMS_DIR "../screenshots/lorenz_manifold_growth_") + stamp + ".png";
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
        int ball = -1;                 // drop id of the ball it belongs to (-1: none)
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

    static constexpr int kStable = 0, kUnstable = 1;   // Side index

    // One end of a 1-D manifold flowing on past W_loc: the orbit of the endpoint.
    struct Branch {
        std::unique_ptr<engine::Trajectory> traj;
        glm::vec3 seed{0.0f};
    };

    // One side (stable or unstable) of one equilibrium's manifold picture.
    struct Side {
        engine::InvariantManifold K;      // the Taylor map of W_loc (parametrization method)
        bool ok = false;
        std::string error;
        int dim = 0;                      // 1 (curve) or 2 (surface); 0 = none
        double radius = 0.0;              // |theta| up to which K is trusted (defect check)
        bool local = false;               // "grow W_loc" pressed
        float grow = 0.0f;                // W_loc growing out of x*, 0 -> 1
        bool global = false;              // "grow W" pressed (curves only)
        Branch branch[2];                 // the + and - ends
        int branchChoice = 0;             // which end "watch" follows
        double t0 = 0.0;                  // field time the branches started at
        double limit = 0.0;               // they stop at |t - t0| = limit (Extend adds more)
        bool paused = false;
        std::unique_ptr<engine::Mesh> mesh;   // W_loc: two strips (curve) or disk + ring loops
        int fillCount = 0;                // vertices of the disk's triangles
        glm::vec3 labelAnchor{NAN};       // world point the W label hangs at
        glm::vec3 end[2]{};               // world ends of a 1-D W_loc, + then -
    };

    // A worker's output for one equilibrium (computeManifolds): K on each side and the
    // radius it is trusted to.
    using Matrix3 = std::array<std::array<double, 3>, 3>;
    struct SideResult {
        engine::InvariantManifold K;
        bool ok = false;
        std::string error;
        double radius = 0.0;
    };
    struct EqResult {
        SideResult side[2];
    };

    // One equilibrium as drawn: where it is, its linearization, its hover state, its
    // manifolds.
    struct EqView {
        engine::LorenzSystem::Equilibrium eq;
        glm::vec3 pos{0.0f};
        engine::Eigen3 eig;
        std::string kind;
        float glow = 0.0f;              // eased toward 1 while hovered / the popup's
        engine::LatexImage label;       // O, C_+, C_-
        std::string labelTex;           // "O", "C_+", "C_-"
        float labelSize = 0.0f;
        Side side[2];                   // kStable, kUnstable
        // the manifolds are computed off the main thread (pollManifolds): until then the
        // sides hold nothing for this field and cannot be grown
        bool ready = false;
        std::future<EqResult> job;
        int jobGen = -1;                // manifoldGen the job was started for
        engine::ConjugacyMap map;       // K_s + K_u on the GPU: the surfaces' vertex shader
    };

    // The eased camera move: centre and radius from -> to (radius interpolated in log),
    // orbit angles left to the user. Here: the ease back when a follow ends.
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
        paramsDragging |= ImGui::IsItemActive();
        changed |= ImGui::DragFloat("rho", &cfg.rho, 0.05f, 0.0f, 300.0f, "%.3f");
        paramsDragging |= ImGui::IsItemActive();
        changed |= ImGui::DragFloat("beta", &cfg.beta, 0.01f, 0.01f, 50.0f, "%.4f");
        paramsDragging |= ImGui::IsItemActive();
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

    // One row per bead: hovering lights it, clicking pins its popup. Reset drops every
    // manifold.
    void drawEquilibriaUI()
    {
        sectionHeader("Equilibria");
        for (int i = 0; i < eqCount; ++i)
        {
            const EqView& e = eqs[i];
            const char* tag = popupEq == i && popupPinned ? "   [pinned]" : "";
            ImGui::Text("%s%s", e.eq.name.c_str(), tag);
            if (ImGui::IsItemHovered())
                uiHovered = i;
            if (ImGui::IsItemClicked())
                togglePin(i);
        }
        bool any = false;
        for (int i = 0; i < eqCount; ++i)
            any = any || eqs[i].side[0].local || eqs[i].side[1].local;
        if (any && ImGui::Button("Reset manifolds"))
            for (int i = 0; i < eqCount; ++i)
                for (int side = 0; side < 2; ++side)
                    resetSide(i, side);
    }

    // Look-and-feel of the beads, under Other settings.
    void drawEquilibriumLookUI()
    {
        sectionHeader("Equilibria");
        ImGui::SliderFloat("bead size", &cfg.eqRadius, 0.1f, 3.0f, "%.2f");
        ImGui::Checkbox("labels", &cfg.showEqLabels);
    }

    void drawManifoldSettingsUI()
    {
        sectionHeader("Manifolds");
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::InputInt("K order", &cfg.manifoldOrder))
        {
            cfg.manifoldOrder = std::clamp(cfg.manifoldOrder, 2, engine::ConjugacyMap::kMaxOrder);
            rebuildManifolds();
        }
        ImGui::SliderFloat("growth time", &cfg.locGrowSeconds, 0.1f, 5.0f, "%.1f s");
        ImGui::SliderFloat("width", &cfg.manifoldWidth, 1.0f, 8.0f, "%.1f px");
        ImGui::SliderFloat("surface opacity", &cfg.diskAlpha, 0.02f, 1.0f, "%.2f");
        ImGui::SliderFloat("branch time", &cfg.branchTime, 0.5f, 100.0f, "%.1f",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("backward speed cap", &cfg.backSpeed, 1.0f, 200.0f, "%.0f",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("pause beyond", &cfg.pauseBeyond, 1.0f, 50.0f, "%.1f x domain",
                           ImGuiSliderFlags_Logarithmic);
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
        ImGui::SliderFloat("speed", &cfg.timeScale, kMinSpeed, 10.0f, "%.4f x",
                           ImGuiSliderFlags_Logarithmic);
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
            dropBall(glm::vec3(cfg.ballCenter[0], cfg.ballCenter[1], cfg.ballCenter[2]),
                     cfg.ballRadius, cfg.ballWithSingle, true);
        ImGui::SameLine();
        if (ImGui::Button("Remove ball"))
            removeBall();
    }

    void drawSinglesUI()
    {
        sectionHeader("Single trajectories");
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

    // The many curves keep a cheap method; singles and the manifold branches (few, and
    // the ones you follow) may use the Taylor method, whose order and tolerance then
    // appear. For Taylor, max step h only spaces the trail points sampled from its
    // polynomial.
    void drawIntegratorUI()
    {
        sectionHeader("Integrator");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::Combo("ball", &cfg.method, "Euler\0RK2 (midpoint)\0RK4\0");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::Combo("singles, manifolds", &cfg.singleMethod, "Euler\0RK2 (midpoint)\0RK4\0Taylor\0");
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
        ImGui::SliderFloat("follow ease", &cfg.followSeconds, 0.0f, 3.0f, "%.2f s");
        if (ImGui::Button("Reset camera"))
            resetCamera();
    }

    // The global home view: camera defaults, no flight in progress, nothing followed.
    void resetCamera()
    {
        cfg.camRadius = 0.0f;
        cfg.camTheta = 0.6f;
        cfg.camPhi = 1.1f;
        cfg.camCenter[0] = cfg.camCenter[1] = 0.0f;
        cfg.camCenter[2] = cfg.domainZ;
        watchEq = -1;
        cfg.followSingle = false;
        tween.active = false;
        applyCamera();
    }

    // ---- the manifold popup -----------------------------------------------------

    // A typeset label, cached by string and size (the popup draws them every frame).
    const engine::LatexImage& tex(const std::string& latex, float size, const glm::vec3& color)
    {
        char key[32];
        std::snprintf(key, sizeof(key), "@%.1f@%.2f%.2f%.2f", size, color.r, color.g, color.b);
        auto it = texCache.find(latex + key);
        if (it == texCache.end())
        {
            it = texCache.emplace(latex + key, engine::LatexImage()).first;
            it->second.set(latex, size, color);
        }
        return it->second;
    }

    // A button whose face is typeset math; plain text if the typesetting failed.
    bool latexButton(const char* id, const std::string& latex, const char* fallback,
                     const glm::vec3& color = glm::vec3(0.95f))
    {
        const engine::LatexImage& img = tex(latex, kPopupButtonPx, color);
        if (img.empty())
            return ImGui::Button(fallback);
        return ImGui::ImageButton(id, (ImTextureID)(intptr_t)img.texture(),
                                  ImVec2((float)img.width(), (float)img.height()));
    }

    static constexpr float kPopupButtonPx = 17.0f;
    static constexpr float kPopupHeaderPx = 19.0f;

    static const char* sideLetter(int side) { return side == kStable ? "s" : "u"; }

    // W^{u}(O), W^{u}_{\mathrm{loc}}(O) ...
    std::string manifoldTex(const EqView& e, int side, bool loc) const
    {
        std::string t = std::string("W^{") + sideLetter(side) + "}";
        if (loc)
            t += "_{\\mathrm{loc}}";
        return t + "(" + e.labelTex + ")";
    }

    std::string manifoldAscii(const EqView& e, int side, bool loc) const
    {
        return std::string("W^") + sideLetter(side) + (loc ? "_loc" : "") + "(" + e.eq.name + ")";
    }

    void togglePin(int i)
    {
        if (popupEq == i && popupPinned)
            popupPinned = false;
        else
        {
            popupEq = i;
            popupPinned = true;
            popupAway = 0.0f;
            popupEntered = false;
        }
    }

    void closePopup()
    {
        popupEq = -1;
        popupPinned = false;
        popupEntered = false;
        popupAway = 0.0f;
        popupW = popupH = 0.0f;
    }

    // Projected radius of a bead, in pixels.
    float beadPixelRadius(const EqView& e) const
    {
        const engine::Camera& cam = app->camera();
        float depth = std::max(glm::dot(e.pos - cam.position, cam.front), 1e-3f);
        float halfH = depth * std::tan(glm::radians(0.5f * cam.fovDeg));
        return cfg.eqRadius / halfH * 0.5f * ImGui::GetIO().DisplaySize.y;
    }

    // The small window beside a hovered bead: its type, then per side "grow W_loc" and,
    // once that has grown, "grow W" (curves) / the not-yet surface tools. It stays while
    // the mouse is on the bead or on it (kPopupGrace of slack for the trip between), or
    // for good once pinned by a click on the bead.
    void drawManifoldPopup(float dt)
    {
        const ImGuiIO& io = ImGui::GetIO();
        if (hovered >= 0 && !popupPinned && hovered != popupEq)
        {
            popupEq = hovered;
            popupAway = 0.0f;
            popupEntered = false;
        }
        if (popupEq < 0)
            return;
        if (popupEq >= eqCount)
        {
            closePopup();
            return;
        }
        EqView& e = eqs[popupEq];

        const bool onPopup = popupW > 0.0f &&
                             io.MousePos.x >= popupX && io.MousePos.x < popupX + popupW &&
                             io.MousePos.y >= popupY && io.MousePos.y < popupY + popupH;
        popupEntered = popupEntered || onPopup;
        if (hovered == popupEq || onPopup || popupPinned)
            popupAway = 0.0f;
        else
            popupAway += dt;
        // kPopupGrace only covers the trip from the bead to the popup: once the mouse has
        // been on it, leaving it (not back onto the bead) closes it at once
        const float grace = popupEntered ? 0.0f : kPopupGrace;
        if (!popupPinned && popupAway > grace)
        {
            closePopup();
            return;
        }

        float sx, sy;
        const bool visible = toScreen(e.pos, sx, sy);
        if (!visible && !popupPinned)
        {
            closePopup();
            return;
        }
        const ImVec2 display = io.DisplaySize;
        float x = visible ? sx + beadPixelRadius(e) + 18.0f : display.x - popupW - 12.0f;
        float y = visible ? sy - 0.3f * popupH : 80.0f;
        if (popupW > 0.0f)
        {
            x = std::clamp(x, 8.0f, std::max(8.0f, display.x - popupW - 8.0f));
            y = std::clamp(y, 8.0f, std::max(8.0f, display.y - popupH - 8.0f));
        }
        ImGui::SetNextWindowPos(ImVec2(x, y), ImGuiCond_Always);
        ImGui::SetNextWindowBgAlpha(0.86f);
        ImGui::Begin("##manifoldPopup", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing |
                         ImGuiWindowFlags_NoNav | ImGuiWindowFlags_NoMove);
        {
            const ImVec2 p = ImGui::GetWindowPos(), s = ImGui::GetWindowSize();
            popupX = p.x;
            popupY = p.y;
            popupW = s.x;
            popupH = s.y;
        }
        ImGui::TextColored(ImVec4(kPopupTitleColor.r, kPopupTitleColor.g, kPopupTitleColor.b, 1.0f),
                           "%s", e.eq.name.c_str());
        ImGui::SameLine();
        ImGui::TextDisabled("%s", e.kind.c_str());
        if (popupPinned)
        {
            ImGui::SameLine();
            if (ImGui::SmallButton("x"))
            {
                ImGui::End();
                closePopup();
                return;
            }
        }
        // a ball of trajectories around x* (the panel's particle count / fill volume)
        if (ImGui::SmallButton("Ball"))
            dropBall(e.pos, cfg.eqBallRadius, false, false);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::SliderFloat("##eqBall", &cfg.eqBallRadius, 0.01f, 20.0f, "r = %.2f",
                           ImGuiSliderFlags_Logarithmic);
        drawSideSection(popupEq, kUnstable);
        drawSideSection(popupEq, kStable);
        ImGui::End();
    }

    void drawSideSection(int eqIndex, int side)
    {
        EqView& e = eqs[eqIndex];
        Side& S = e.side[side];
        const glm::vec3 color = side == kStable ? kStableColor : kUnstableColor;
        ImGui::PushID(side);
        ImGui::Spacing();
        ImGui::Separator();
        const engine::LatexImage& head = tex(manifoldTex(e, side, false), kPopupHeaderPx, color);
        if (head.empty())
            ImGui::TextColored(ImVec4(color.r, color.g, color.b, 1.0f), "%s",
                               manifoldAscii(e, side, false).c_str());
        else
            head.draw();
        ImGui::SameLine();
        if (!e.ready)
        {
            ImGui::TextDisabled("computing...");   // nothing to grow until it lands
            ImGui::PopID();
            return;
        }
        ImGui::TextDisabled("%s", S.dim == 1 ? "curve" : S.dim == 2 ? "surface" : "");
        if (S.ok)
        {
            ImGui::SameLine();
            if (ImGui::SmallButton("ball"))
                dropManifoldBall(e, side);
        }

        if (!S.ok)
            ImGui::TextDisabled("%s", S.error.c_str());
        else if (!S.local)
        {
            if (latexButton("##growLoc", "\\mathrm{Grow}\\;" + manifoldTex(e, side, true),
                            ("Grow " + manifoldAscii(e, side, true)).c_str()))
            {
                S.local = true;
                S.grow = 0.0f;
            }
        }
        else
        {
            ImGui::TextDisabled("r = %.2f", S.radius);
            if (S.grow >= 1.0f)
            {
                if (S.dim == 1)
                {
                    if (!S.global)
                    {
                        if (latexButton("##grow", "\\mathrm{Grow}\\;" + manifoldTex(e, side, false),
                                        ("Grow " + manifoldAscii(e, side, false)).c_str()))
                            startGlobal(eqIndex, side);
                    }
                    else
                    {
                        bool w = watchEq == eqIndex && watchSide == side;
                        if (ImGui::Checkbox("watch", &w))
                        {
                            if (w)
                            {
                                watchEq = eqIndex;
                                watchSide = side;
                            }
                            else
                            {
                                watchEq = -1;
                                detachInPlace = true;
                            }
                        }
                        ImGui::SameLine();
                        ImGui::RadioButton("+", &S.branchChoice, 0);
                        ImGui::SameLine();
                        ImGui::RadioButton("-", &S.branchChoice, 1);
                        double t = 0.0;
                        for (const Branch& b : S.branch)
                            if (b.traj)
                                t = std::max(t, std::abs(b.traj->time() - S.t0));
                        if (t >= S.limit - 1e-9)
                        {
                            if (ImGui::Button("Extend"))
                            {
                                S.limit += cfg.branchTime;
                                S.paused = false;
                            }
                        }
                        else if (ImGui::Button(S.paused ? "Resume" : "Pause"))
                            S.paused = !S.paused;
                        ImGui::SameLine();
                        ImGui::TextDisabled("t = %.1f", t);
                    }
                }
                else
                {
                    // surfaces: the global growth needs geodesic level sets (not yet)
                    ImGui::BeginDisabled();
                    latexButton("##grow", "\\mathrm{Grow}\\;" + manifoldTex(e, side, false),
                                ("Grow " + manifoldAscii(e, side, false)).c_str());
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    ImGui::TextDisabled("not yet");
                    ImGui::BeginDisabled(!app->hasSim(kLevelSetsSim));
                    if (ImGui::Button("Learn geodesic level sets"))
                        enterLevelSets(eqIndex, side);
                    ImGui::EndDisabled();
                }
            }
            if (ImGui::SmallButton("reset"))
                resetSide(eqIndex, side);
        }
        ImGui::PopID();
    }

    // ---- manifolds -------------------------------------------------------------------

    // New field or K order: every equilibrium's manifolds are stale. The global branches
    // belong to the old field and go; what is grown locally stays chosen and reappears
    // once its side is recomputed (pollManifolds).
    void rebuildManifolds()
    {
        ++manifoldGen;
        for (int i = 0; i < (int)eqs.size(); ++i)
        {
            eqs[i].ready = false;
            for (int side = 0; side < 2; ++side)
            {
                dropBranches(i, side);
                if (i >= eqCount)
                    eqs[i].side[side].local = eqs[i].side[side].ok = false;
            }
        }
    }

    // One equilibrium's manifolds for one field: K on both sides and the radius each is
    // trusted to — both the smaller trusted one, so neither reaches past the other (at
    // C+- the curve would go 10x the surface). Pure computation: runs on a worker.
    static EqResult computeManifolds(Matrix3 Jm, int order)
    {
        double J[3][3];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                J[r][c] = Jm[r][c];
        const auto Q = engine::LorenzSystem::quadratic();
        EqResult out;
        double common = 1e30;
        for (int side = 0; side < 2; ++side)
        {
            SideResult& S = out.side[side];
            S.ok = S.K.build(J, Q, side == kStable ? engine::InvariantManifold::Side::Stable
                                                   : engine::InvariantManifold::Side::Unstable,
                             order);
            S.error = S.ok ? std::string() : S.K.error();
            if (S.ok)
                common = std::min(common, trustedRadius(S.K));
        }
        for (SideResult& S : out.side)
            S.radius = S.ok ? common : 0.0;
        return out;
    }

    Matrix3 jacobianAt(const EqView& e) const
    {
        double J[3][3];
        system.jacobian(e.eq.x, J);
        Matrix3 Jm;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Jm[r][c] = J[r][c];
        return Jm;
    }

    // Finished workers: apply results for the current field, drop older ones. A stale
    // equilibrium gets a worker once the parameters are still (none while a slider is
    // held), so dragging only moves the field and the equilibria.
    void pollManifolds()
    {
        for (int i = 0; i < (int)eqs.size(); ++i)
        {
            EqView& e = eqs[i];
            if (e.job.valid() && e.job.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                EqResult r = e.job.get();
                if (e.jobGen == manifoldGen && i < eqCount)
                    applyManifolds(e, std::move(r));
            }
            if (i < eqCount && !e.ready && !e.job.valid() && !paramsDragging)
            {
                e.jobGen = manifoldGen;
                e.job = std::async(std::launch::async, computeManifolds, jacobianAt(e), cfg.manifoldOrder);
            }
        }
        paramsDragging = false;   // drawSystemUI sets it again while a slider is held
    }

    // At open: every equilibrium on this thread (the handoff back needs S.ok at once).
    void computeManifoldsNow()
    {
        for (int i = 0; i < eqCount; ++i)
            applyManifolds(eqs[i], computeManifolds(jacobianAt(eqs[i]), cfg.manifoldOrder));
    }

    // A worker's result into the sides: K, radius, meshes. Surfaces are drawn through
    // the conjugacy map (x* + K(theta) in the vertex shader), which needs both sides.
    void applyManifolds(EqView& e, EqResult r)
    {
        bool surface = false;
        for (int side = 0; side < 2; ++side)
        {
            Side& S = e.side[side];
            SideResult& R = r.side[side];
            S.K = std::move(R.K);
            S.ok = R.ok;
            S.dim = S.ok ? S.K.dim() : 0;
            S.error = R.error;
            S.radius = R.radius;
            surface = surface || S.dim == 2;
        }
        if (surface && !(e.side[0].ok && e.side[1].ok && e.map.set(e.side[0].K, e.side[1].K)))
            for (Side& S : e.side)
                if (S.dim == 2)
                {
                    S.ok = false;
                    S.dim = 0;
                    S.error = "no conjugacy map: " + e.map.error();
                }
        for (int side = 0; side < 2; ++side)
        {
            if (!e.side[side].ok)
                e.side[side].local = false;
            else
                buildLocal(e, side);
        }
        e.ready = true;
    }

    // Largest |theta| (bisection up to kRadiusSearchMax) where K's relative invariance
    // defect stays under kDefectOk all the way out: how far W_loc is drawn.
    static double trustedRadius(const engine::InvariantManifold& K)
    {
        // every radius out to r under the bar, checked in parallel (outermost first: it
        // fails most often, and a failure stops the rest)
        auto good = [&](double r) {
            static const int js[8] = {8, 7, 6, 5, 4, 3, 2, 1};
            return std::none_of(std::execution::par, js, js + 8, [&](int j) {
                return !K.relDefectBelow(r * j / 8.0, kDefectOk, 48);
            });
        };
        double lo = 0.0, hi = kRadiusSearchMax;
        if (good(hi))
            return hi;
        for (int it = 0; it < 16; ++it)
        {
            double mid = 0.5 * (lo + hi);
            (good(mid) ? lo : hi) = mid;
        }
        return lo;
    }

    // x* + K(s) for the point theta of E (relative to x*).
    static glm::vec3 kPoint(const Side& S, const glm::vec3& theta)
    {
        auto s = S.K.toParam({theta.x, theta.y, theta.z});
        auto p = S.K.eval(s);
        return glm::vec3((float)p[0], (float)p[1], (float)p[2]);
    }

    // The mesh of W_loc: a curve as two strips of kCurveSteps + 1 points from x* out to
    // +-radius along E (K on the CPU); a surface as the unit disk of E in rings x
    // segments of triangles, followed by every ring as a loop — surface_vertex.glsl
    // scales it to the grown radius and pushes it through K (ConjugacyMap::hInv).
    void buildLocal(EqView& e, int side)
    {
        Side& S = e.side[side];
        std::vector<float> v;
        auto vert = [&](const glm::vec3& p) { v.insert(v.end(), {p.x, p.y, p.z}); };
        S.labelAnchor = glm::vec3(NAN);
        S.fillCount = 0;
        const float r = (float)S.radius;
        if (S.dim == 1)
        {
            auto o = S.K.orthoBasis(0);
            const glm::vec3 e0((float)o[0], (float)o[1], (float)o[2]);
            for (float dir : {1.0f, -1.0f})
                for (int i = 0; i <= kCurveSteps; ++i)
                    vert(e.pos + kPoint(S, dir * r * (float)i / kCurveSteps * e0));
            S.end[0] = e.pos + kPoint(S, r * e0);
            S.end[1] = e.pos + kPoint(S, -r * e0);
            S.labelAnchor = e.pos + kPoint(S, 0.85f * r * e0);
        }
        else if (S.dim == 2)
        {
            auto o0 = S.K.orthoBasis(0), o1 = S.K.orthoBasis(1);
            const glm::vec3 e0((float)o0[0], (float)o0[1], (float)o0[2]);
            const glm::vec3 e1((float)o1[0], (float)o1[1], (float)o1[2]);
            std::vector<glm::vec3> pts((size_t)(kRings + 1) * kSegs);
            for (int ring = 0; ring <= kRings; ++ring)
                for (int k = 0; k < kSegs; ++k)
                {
                    const float a = 6.2831853f * (float)k / kSegs;
                    const float rr = (float)ring / kRings;   // theta / r: K is applied on the GPU
                    pts[(size_t)ring * kSegs + k] = rr * (std::cos(a) * e0 + std::sin(a) * e1);
                }
            auto at = [&](int ring, int k) { return pts[(size_t)ring * kSegs + (k % kSegs)]; };
            for (int ring = 0; ring < kRings; ++ring)
                for (int k = 0; k < kSegs; ++k)
                {
                    vert(at(ring, k));
                    vert(at(ring + 1, k));
                    vert(at(ring + 1, k + 1));
                    vert(at(ring, k));
                    vert(at(ring + 1, k + 1));
                    vert(at(ring, k + 1));
                }
            S.fillCount = kRings * kSegs * 6;
            for (int ring = 0; ring <= kRings; ++ring)
                for (int k = 0; k < kSegs; ++k)
                    vert(at(ring, k));
            S.labelAnchor = e.pos + kPoint(S, 0.92f * r * glm::normalize(e0 + e1));
        }
        if (!S.mesh)
            S.mesh = std::make_unique<engine::Mesh>(kLocalCapacity, std::vector<int>{3});
        S.mesh->setVertices(v.data(), std::min(v.size(), (size_t)kLocalCapacity * 3));
    }

    // "Grow W" on a curve: the two ends of W_loc flow on — forward on W^u, backward on
    // W^s — each as a trajectory that keeps its whole curve.
    void startGlobal(int eqIndex, int side)
    {
        EqView& e = eqs[eqIndex];
        Side& S = e.side[side];
        if (!S.ok || S.dim != 1)
            return;
        S.global = true;
        S.paused = false;
        S.t0 = fieldTime;
        S.limit = cfg.branchTime;
        for (int b = 0; b < 2; ++b)
        {
            const glm::vec3 seed = S.end[b];
            double s[3] = {seed.x, seed.y, seed.z};
            S.branch[b].seed = seed;
            S.branch[b].traj = std::make_unique<engine::Trajectory>(3, s, fieldTime, kBranchPoints);
            S.branch[b].traj->minSpacing = 0.002f * cfg.domainR;
        }
    }

    void dropBranches(int eqIndex, int side)
    {
        Side& S = eqs[eqIndex].side[side];
        S.global = false;
        for (Branch& b : S.branch)
            b.traj.reset();
        if (watchEq == eqIndex && watchSide == side)
            watchEq = -1;
    }

    void resetSide(int eqIndex, int side)
    {
        dropBranches(eqIndex, side);
        Side& S = eqs[eqIndex].side[side];
        S.local = false;
        S.grow = 0.0f;
    }

    // Where the scene is: the centre of the arrow cube.
    glm::vec3 sceneCenter() const { return glm::vec3(0.0f, 0.0f, cfg.domainZ); }

    // This frame's step for one curve. Forward: h. Backward the Lorenz flow blows up
    // in finite time, so the arc speed is capped at backSpeed (a change of time only:
    // the curve traced is the same, just slower) and a curve past pauseBeyond domain
    // radii from the scene stands still (0) — until the flow runs forward again.
    double curveStep(const engine::Trajectory& t, double h) const
    {
        if (h >= 0.0 || t.diverged())
            return h;
        const double* x = t.state();
        const glm::vec3 p((float)x[0], (float)x[1], (float)x[2]);
        if (glm::length(p - sceneCenter()) > cfg.pauseBeyond * cfg.domainR)
            return 0.0;
        double f[3];
        system.eval(x, f);
        const double v = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        return v > cfg.backSpeed ? h * cfg.backSpeed / v : h;
    }

    // Advance a list of curves by h of field time, each by its curveStep: the ones at
    // full h together (in parallel), the slowed ones one by one. singlesMethod: the
    // singles' method (Taylor by default), else the balls' `method`.
    void advanceCurves(const std::vector<engine::Trajectory*>& list, double h, bool singlesMethod)
    {
        if (list.empty())
            return;
        auto run = [&](const std::vector<engine::Trajectory*>& l, double step) {
            if (!singlesMethod)
                runner.advance(l, integrator, rhs, step);
            else if (cfg.singleMethod == kTaylor)
                runner.advance(l, taylor, jet, step, cfg.maxStep);
            else
                runner.advance(l, singleIntegrator, rhs, step);
        };
        std::vector<engine::Trajectory*> full;
        for (engine::Trajectory* t : list)
        {
            const double step = curveStep(*t, h);
            if (step == h)
                full.push_back(t);
            else if (step != 0.0)
                run({t}, step);
        }
        run(full, h);
    }

    // W_loc of every grown side, as far as it has grown: curves as lines, surfaces as a
    // translucent fill (no depth writes) with the current edge ring on top.
    void renderLocalManifolds(const glm::mat4& view, const glm::mat4& projection, float px)
    {
        shader.setInt("u_useRamp", 0);
        shader.setInt("u_useFade", 0);
        shader.setInt("u_shadeMode", 0);
        shader.setInt("u_round", 0);
        for (int i = 0; i < eqCount; ++i)
            for (int side = 0; side < 2; ++side)
            {
                const Side& S = eqs[i].side[side];
                if (!eqs[i].ready || !S.local || !S.ok || !S.mesh || S.dim != 1)
                    continue;
                const glm::vec3 color = side == kStable ? kStableColor : kUnstableColor;
                const float fade = i == focusEq && side == focusSide ? 1.0f : sceneFade;
                shader.setVec3("u_color", color);
                if (S.dim == 1)
                {
                    const int n = std::clamp(1 + (int)std::floor(S.grow * kCurveSteps), 1, kCurveSteps + 1);
                    shader.setFloat("u_alpha", fade);
                    glLineWidth(cfg.manifoldWidth * px);
                    S.mesh->drawArrays(GL_LINE_STRIP, n, 0);
                    S.mesh->drawArrays(GL_LINE_STRIP, n, kCurveSteps + 1);
                }
            }
        shader.setFloat("u_alpha", 1.0f);

        // surfaces: the whole disk at the grown radius, so it grows continuously
        surfaceShader.use();
        surfaceShader.setMat4("view", view);
        surfaceShader.setMat4("projection", projection);
        surfaceShader.setInt("u_useRamp", 0);
        surfaceShader.setInt("u_round", 0);
        surfaceShader.setFloat("u_alphaTail", 1.0f);
        for (int i = 0; i < eqCount; ++i)
            for (int side = 0; side < 2; ++side)
            {
                const EqView& e = eqs[i];
                const Side& S = e.side[side];
                if (!e.ready || !S.local || !S.ok || !S.mesh || S.dim != 2)
                    continue;
                const float fade = i == focusEq && side == focusSide ? 1.0f : sceneFade;
                e.map.bind();
                surfaceShader.setVec3("u_color", side == kStable ? kStableColor : kUnstableColor);
                surfaceShader.setVec3("u_center", e.pos);
                surfaceShader.setFloat("u_radius", (float)S.radius * S.grow);
                glDepthMask(GL_FALSE);
                surfaceShader.setFloat("u_alpha", cfg.diskAlpha * fade);
                S.mesh->drawArrays(GL_TRIANGLES, S.fillCount, 0);
                glDepthMask(GL_TRUE);
                surfaceShader.setFloat("u_alpha", 0.9f * fade);
                glLineWidth(0.7f * cfg.manifoldWidth * px);
                S.mesh->drawArrays(GL_LINE_LOOP, kSegs, S.fillCount + kRings * kSegs);   // the edge
            }
        glLineWidth(1.0f);
    }

    // The global branches: the curve each end has traced, and a bead at its tip.
    void renderBranches(float px)
    {
        for (int i = 0; i < eqCount; ++i)
            for (int side = 0; side < 2; ++side)
            {
                const Side& S = eqs[i].side[side];
                const glm::vec3 color = side == kStable ? kStableColor : kUnstableColor;
                for (const Branch& b : S.branch)
                {
                    if (!b.traj)
                        continue;
                    glLineWidth(cfg.manifoldWidth * px);
                    shader.setVec3("u_color", color);
                    shader.setFloat("u_alpha", sceneFade);
                    shader.setInt("u_useRamp", 0);
                    b.traj->draw();
                    shader.setFloat("u_pointSize", cfg.particleSize * 1.6f * px);
                    shader.setInt("u_round", 1);
                    b.traj->drawHead();
                    shader.setInt("u_round", 0);
                }
            }
    }

    // W^u_loc(O) / W^u(O) beside each grown local manifold, in its color.
    void drawManifoldLabels()
    {
        if (!cfg.showEqLabels)
            return;
        const float size = 0.8f * cfg.eqLabelSize;
        for (int i = 0; i < eqCount; ++i)
            for (int side = 0; side < 2; ++side)
            {
                const Side& S = eqs[i].side[side];
                if (!eqs[i].ready || !S.local || !S.ok || !std::isfinite(S.labelAnchor.x))
                    continue;
                float sx, sy;
                if (!toScreen(S.labelAnchor, sx, sy))
                    continue;
                const glm::vec3 color = side == kStable ? kStableColor : kUnstableColor;
                const engine::LatexImage& img = tex(manifoldTex(eqs[i], side, !S.global), size, color);
                if (img.empty())
                    continue;
                img.drawAt(sx + 8.0f, sy - (float)img.height() - 4.0f,
                           0.9f * S.grow * (i == focusEq && side == focusSide ? 1.0f : sceneFade));
            }
    }

    // ---- equilibria: picking, glow, labels -------------------------------------------

    // New sigma/rho/beta: the field, the equilibria, their linearizations and manifolds.
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
        if (popupEq >= eqCount)
            closePopup();   // C+- merged into the origin (rho <= 1)
        rebuildManifolds();
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

    void startTween(const glm::vec3& center, float radius)
    {
        const engine::Camera& cam = app->camera();
        tween.active = true;
        tween.t = 0.0f;
        tween.duration = kReturnSeconds;
        tween.c0 = cam.orbitCenter;
        tween.r0 = std::max(cam.orbitRadius, 1e-3f);
        tween.c1 = center;
        tween.r1 = std::max(radius, 1e-3f);
    }

    // "Learn geodesic level sets" on a grown surface: remember the view, drop every
    // follow, and zoom onto the patch while the rest of the scene fades; the switch
    // happens when the zoom lands (handOffLevelSets).
    void enterLevelSets(int eqIndex, int side)
    {
        if (leaving || !app->hasSim(kLevelSetsSim))
            return;
        const engine::Camera& cam = app->camera();
        globalCenter = tween.active ? tween.c1 : following ? followReturn : cam.orbitCenter;
        globalRadius = tween.active ? tween.r1 : cam.orbitRadius;
        cfg.followSingle = false;
        watchEq = -1;
        detachInPlace = true;   // the follow ending must not start its own ease back
        closePopup();
        focusEq = eqIndex;
        focusSide = side;
        leaving = true;
        returning = false;
        const float r = (float)eqs[eqIndex].side[side].radius;
        startTween(eqs[eqIndex].pos, kFramingScale * std::max(r, 0.5f));
        tween.duration = kLeaveSeconds;
    }

    // The zoom has landed: switch, handing over the parameters, which patch, its K
    // order and look, what is grown locally (bit 2 i + side) and the view to come back
    // to. The camera is shared and left where it is, so the switch shows no jump.
    void handOffLevelSets()
    {
        int mask = 0;
        for (int i = 0; i < eqCount; ++i)
            for (int s = 0; s < 2; ++s)
                if (eqs[i].side[s].local)
                    mask |= 1 << (2 * i + s);
        engine::App::Handoff out{
            {"sigma", cfg.sigma}, {"rho", cfg.rho}, {"beta", cfg.beta},
            {"eq", focusEq}, {"side", focusSide}, {"order", cfg.manifoldOrder},
            {"eqRadius", cfg.eqRadius}, {"fillAlpha", cfg.diskAlpha},
            {"loc", mask},
            {"globalCx", globalCenter.x}, {"globalCy", globalCenter.y},
            {"globalCz", globalCenter.z}, {"globalR", globalRadius},
        };
        if (!app->requestSwitch(kLevelSetsSim, std::move(out)))
        {
            leaving = false;
            focusEq = -1;
        }
    }

    // Opacity of everything but the focused patch and its bead: out over the end of the
    // zoom in, back in over the start of the zoom out after returning.
    float currentSceneFade() const
    {
        const float s = tween.active ? tween.t / tween.duration : 1.0f;
        auto ramp = [](float a, float b, float x) { return smootherstep((x - a) / (b - a)); };
        if (leaving)
            return 1.0f - ramp(0.35f, 0.95f, s);
        if (returning && tween.active)
            return ramp(0.05f, 0.6f, s);
        return 1.0f;
    }

    // What the camera rides: the watched branch's tip first, else the newest single
    // while "camera follows the newest" is on.
    const engine::Trajectory* followTarget() const
    {
        if (watchEq >= 0 && watchEq < eqCount)
        {
            const Side& S = eqs[watchEq].side[watchSide];
            const int b = std::clamp(S.branchChoice, 0, 1);
            if (S.global && S.branch[b].traj)
                return S.branch[b].traj.get();
        }
        if (!cfg.followSingle)
            return nullptr;
        for (auto it = curves.rbegin(); it != curves.rend(); ++it)
            if (it->kind == Kind::Single)
                return it->traj.get();
        return nullptr;
    }

    float baseFarPlane() const { return std::max(500.0f, 20.0f * cfg.domainR); }

    // Zoomed far out nothing may fall past the far plane: it reaches beyond the whole
    // scene (cube + where backward curves freeze) from wherever the camera is. The
    // near plane is left alone: the camera is shared and other sims rely on its default.
    void fitClipPlanes(engine::Camera& cam) const
    {
        const float scene = (cfg.pauseBeyond + 2.0f) * cfg.domainR;
        const float reach = cam.orbitRadius + glm::length(cam.orbitCenter - sceneCenter()) + scene;
        cam.farPlane = std::max(baseFarPlane(), 1.5f * reach);
    }

    // Runs after Camera::processInput, so the orbit position is recomputed here from the
    // (possibly tweened) centre and radius: the frame shows this frame's camera.
    void updateCamera(float dt)
    {
        engine::Camera& cam = app->camera();
        cam.orbitAngularSpeed = cfg.orbitSpeed;
        cam.speed = cfg.zoomSpeed > 0.0f ? cfg.zoomSpeed : std::max(1.0f, 0.5f * cam.orbitRadius);
        fitClipPlanes(cam);

        // follow a head: slide onto it (below) when it starts or the target changes;
        // ease back to where the camera was when it stops (unless detached in place)
        const engine::Trajectory* followed = followTarget();
        if ((followed != nullptr) != following)
        {
            following = followed != nullptr;
            if (following)
            {
                // started mid-flight (e.g. still easing back out): return to where that
                // flight was going, and let the slide take over from here
                followReturn = tween.active ? tween.c1 : cam.orbitCenter;
                tween.active = false;
            }
            else
            {
                followedTraj = nullptr;
                if (!detachInPlace)
                    startTween(followReturn, cam.orbitRadius);
                detachInPlace = false;
            }
        }

        if (tween.active)
        {
            tween.t += dt;
            float s = smootherstep(tween.t / tween.duration);
            cam.orbitCenter = glm::mix(tween.c0, tween.c1, s);
            cam.orbitRadius = tween.r0 * std::pow(tween.r1 / tween.r0, s);
            if (tween.t >= tween.duration)
            {
                tween.active = false;
                if (leaving)
                    handOffLevelSets();
                else if (returning)
                {
                    returning = false;
                    focusEq = -1;
                }
            }
        }
        else if (following)
        {
            // attached to the head at once; the offset at the moment it changed decays
            // with a cubic ease-out (same slide as vector_field)
            glm::vec3 head = followed->head();
            if (followed != followedTraj)
            {
                followedTraj = followed;
                easeOffset = cam.orbitCenter - head;
                easeT = 0.0f;
            }
            if (easeT < cfg.followSeconds)
            {
                easeT += dt;
                float u = 1.0f - std::clamp(easeT / cfg.followSeconds, 0.0f, 1.0f);
                cam.orbitCenter = head + easeOffset * (u * u * u);
            }
            else
                cam.orbitCenter = head;
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
            eqShader.setFloat("u_radius", cfg.eqRadius * (halo ? 2.2f * pulse : 1.0f));
            eqShader.setVec3("u_color", halo ? kHaloColor : kEquilibriumColor);
            eqShader.setFloat("u_glow", e.glow * (halo ? pulse : 1.0f));
            eqShader.setFloat("u_alpha", i == focusEq ? 1.0f : sceneFade);
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
                           (0.75f + 0.25f * e.glow) * (i == focusEq ? 1.0f : sceneFade));
        }
    }

    // ---- overlays -------------------------------------------------------------------

    // \phi^t(x_0) above every single: the flow map for the ODE time integrated since it
    // was dropped (negative when running in reverse). Same composition as vector_field.
    void drawTimeLabels()
    {
        if (!cfg.showTimeLabels || sceneFade < 1.0f)
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
                pieces[i]->drawAt(left + xs[i], baseY + ys[i], 1.0f);
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
        ImGui::Begin("##lorenzGrowthLatex", nullptr,
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

    // The view is what gets saved — while following, the one to come back to.
    bool saveCurrentSettings()
    {
        const engine::Camera& cam = app->camera();
        glm::vec3 c = following ? followReturn : cam.orbitCenter;
        cfg.camTheta = cam.orbitTheta;
        cfg.camPhi = cam.orbitPhi;
        cfg.camRadius = cam.orbitRadius;
        cfg.camCenter[0] = c.x;
        cfg.camCenter[1] = c.y;
        cfg.camCenter[2] = c.z;
        return saveSettingsFile(cfg, kSettingsPath);
    }

    // ---- trajectories -------------------------------------------------------------

    // Where a click seeds: under the cursor on a camera-facing plane spawnDepth of the
    // way to the orbit centre. False when the cursor ray misses that plane.
    bool cursorSeedPoint(glm::vec3& out) const
    {
        glm::vec3 origin, dir;
        if (!cursorRay(origin, dir))
            return false;
        const engine::Camera& cam = app->camera();
        float denom = glm::dot(dir, cam.front);
        if (std::abs(denom) < 1e-4f)
            return false;
        float toCenter = glm::dot(cam.orbitCenter - cam.position, cam.front);
        glm::vec3 planePoint = cam.position + cam.front * (cfg.spawnDepth * toCenter);
        float s = glm::dot(planePoint - origin, cam.front) / denom;
        if (s <= 0.0f)
            return false;
        out = origin + dir * s;
        return true;
    }

    void dropSingleUnderCursor()
    {
        glm::vec3 p;
        if (cursorSeedPoint(p))
            addTrajectory(p, kSingleColor, Kind::Single);
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

    // Radius for the ball V drops at p: a fixed fraction of the view's half-height at
    // p's depth, so the fresh ball fits on screen however far in or out you are.
    float viewFitRadius(const glm::vec3& p) const
    {
        const engine::Camera& cam = app->camera();
        float depth = std::max(glm::dot(p - cam.position, cam.front), 1e-3f);
        return kViewBallFraction * depth * std::tan(glm::radians(0.5f * cam.fovDeg));
    }
    static constexpr float kViewBallFraction = 0.12f;

    // Seeds on a Fibonacci sphere (the clearest view of how the flow deforms it) or
    // uniformly through the inside. replace: the previous ball goes first (B, the
    // button). Otherwise the new ball joins the others and only the oldest beyond
    // kMaxBalls is retired (V).
    void dropBall(const glm::vec3& center, float radius, bool withSingle, bool replace)
    {
        if (replace)
            removeBall();
        const size_t first = curves.size();
        // "single at center": a green single riding the middle of the ball, labelled and
        // followable like any click-dropped one, removed with the ball
        if (withSingle)
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
            addTrajectory(center + p * radius, kBallColor, Kind::Ball);
        }
        const int id = nextBallId++;
        for (size_t i = first; i < curves.size(); ++i)
            curves[i].ball = id;
        if (!replace)
            trimBalls();
    }

    // A ball of trajectories lying on W_loc: x* + K(theta) for theta in E within
    // min(eqBallRadius, trusted r) — evenly along the segment for a curve, a sunflower
    // over the disk for a surface — in the side's color, one ball like dropBall's.
    void dropManifoldBall(const EqView& e, int side)
    {
        const Side& S = e.side[side];
        if (!S.ok || S.dim < 1)
            return;
        const float rr = std::min(cfg.eqBallRadius, (float)S.radius);
        const glm::vec3 color = side == kStable ? kStableColor : kUnstableColor;
        auto o0 = S.K.orthoBasis(0);
        const glm::vec3 e0((float)o0[0], (float)o0[1], (float)o0[2]);
        glm::vec3 e1(0.0f);
        if (S.dim == 2)
        {
            auto o1 = S.K.orthoBasis(1);
            e1 = glm::vec3((float)o1[0], (float)o1[1], (float)o1[2]);
        }
        const size_t first = curves.size();
        const float golden = 2.39996323f;
        const int n = std::max(cfg.ballCount, 2);
        for (int k = 0; k < n && curves.size() < kMaxTrajectories; ++k)
        {
            glm::vec3 theta;
            if (S.dim == 1)
                theta = rr * (2.0f * (float)k / (float)(n - 1) - 1.0f) * e0;
            else
            {
                const float rad = rr * std::sqrt(((float)k + 0.5f) / (float)n);
                const float a = golden * (float)k;
                theta = rad * (std::cos(a) * e0 + std::sin(a) * e1);
            }
            addTrajectory(e.pos + kPoint(S, theta), color, Kind::Ball);
        }
        const int id = nextBallId++;
        for (size_t i = first; i < curves.size(); ++i)
            curves[i].ball = id;
        trimBalls();
    }

    // Keep the newest kMaxBalls balls (each with its centre single, if any).
    void trimBalls()
    {
        std::vector<int> ids;
        for (const Curve& c : curves)
            if (c.ball >= 0 && std::find(ids.begin(), ids.end(), c.ball) == ids.end())
                ids.push_back(c.ball);
        std::sort(ids.begin(), ids.end());
        while ((int)ids.size() > kMaxBalls)
        {
            const int oldest = ids.front();
            ids.erase(ids.begin());
            std::erase_if(curves, [oldest](const Curve& c) { return c.ball == oldest; });
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

    static constexpr int kLiveRebuildArrows = 20000;
    static constexpr size_t kMaxTrajectories = 4000;

    engine::App* app = nullptr;
    engine::Shader shader;      // glyphs, trails, axes, manifolds (vector_field's pair)
    engine::Shader eqShader;    // equilibrium beads + halos
    engine::Shader surfaceShader;   // 2-D local manifolds: K in the vertex stage
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
    engine::Integrator singleIntegrator;   // singles + branches, unless on the Taylor method
    engine::TaylorIntegrator taylor;
    engine::TaylorIntegrator::Jet jet;     // LorenzSystem::jet
    std::vector<Curve> curves;
    double fieldTime = 0.0;
    bool playing = true;
    std::mt19937 rng{1234};

    std::array<EqView, 3> eqs;
    int eqCount = 0;
    int manifoldGen = 0;           // bumped by every new field / K order (stale workers drop)
    bool paramsDragging = false;   // a sigma/rho/beta slider is held (no workers yet)
    int hovered = -1;           // bead under the cursor (or its UI row), this frame
    int uiHovered = -1;
    float clock = 0.0f;         // drives the halo pulse

    // the manifold popup: whose it is, pinned by a click, and its rect last frame
    int popupEq = -1;
    bool popupPinned = false;
    float popupAway = 0.0f;     // seconds the mouse has been off bead and popup
    bool popupEntered = false;  // the mouse has been on the popup since it opened
    float popupX = 0.0f, popupY = 0.0f, popupW = 0.0f, popupH = 0.0f;
    std::map<std::string, engine::LatexImage> texCache;

    // the watched branch: the camera rides its tip
    int watchEq = -1;
    int watchSide = kUnstable;

    CameraTween tween;
    // the walkthrough: zooming onto the patch (leaving) or back out of it (returning);
    // focusEq/focusSide stay lit while the rest fades (sceneFade)
    bool leaving = false;
    bool returning = false;
    int focusEq = -1;
    int focusSide = 0;
    float sceneFade = 1.0f;
    bool firstUpdate = true;          // its dt still holds init (and a switch): clamped
    glm::vec3 globalCenter{0.0f};     // the view to come back to
    float globalRadius = 90.0f;
    bool following = false;           // the camera is tracking a head right now
    bool detachInPlace = false;       // a follow ended by key 1 / unticking: stay put
    float arrowsFade = -1.0f;         // arrow-field opacity, follows showArrows (-1: unset)
    static constexpr float kArrowFadeSeconds = 0.35f;
    static constexpr int kMaxBalls = 3;   // V stacks balls up to this many
    int nextBallId = 0;
    glm::vec3 followReturn{0.0f};     // orbit centre to ease back to when it stops
    const engine::Trajectory* followedTraj = nullptr; // identity of the followed head
    glm::vec3 easeOffset{0.0f};       // orbitCenter - head when the slide began
    float easeT = 1e9f;               // seconds into the slide (>= followSeconds: locked on)
    engine::TrajectoryRunner runner;  // advances the curves, times integrate/upload/draw
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

std::unique_ptr<engine::Sim> makeLorenzManifoldGrowthSim()
{
    return std::make_unique<LorenzManifoldGrowthSim>();
}
