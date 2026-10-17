#include "LorenzEquilibriumSim.h"

#include <engine/App.h>
#include <engine/ArrowGlyphs.h>
#include <engine/Camera.h>
#include <engine/ConjugacyMap.h>
#include <engine/Eigen3.h>
#include <engine/FlowKeys.h>
#include <engine/FrameCopy.h>
#include <engine/Integrator.h>
#include <engine/InvariantManifold.h>
#include <engine/LatexImage.h>
#include <engine/LorenzSystem.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>
#include <engine/TaylorIntegrator.h>
#include <engine/Trajectory.h>
#include <engine/TrajectoryRunner.h>

#include <gtc/matrix_transform.hpp>
#include <imgui.h>
#include <algorithm>
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
#include <sstream>
#include <string>
#include <vector>

namespace {

// ---- colors, small helpers ------------------------------------------------------

const glm::vec3 kSeedColor(1.0f);
const glm::vec3 kSingleColor = {0.35f, 0.95f, 0.45f};   // click-dropped: always green
const glm::vec3 kDivergedColor = {0.95f, 0.25f, 0.25f};
const glm::vec3 kEquilibriumColor = {0.92f, 0.12f, 0.10f};   // same bead as lorenz_flow
const glm::vec3 kHaloColor = {1.0f, 0.38f, 0.22f};
const glm::vec3 kGlassColor = {0.72f, 0.86f, 1.0f};
const glm::vec3 kStableColor(0.25f, 0.45f, 1.0f);     // E^s, as in diagonal_linear
const glm::vec3 kUnstableColor(1.0f, 0.25f, 0.22f);   // E^u
const glm::vec3 kLinearArrowColor(0.92f, 0.92f, 0.98f);   // A(x - x*) next to f
// stage B: the three curves from each seed of the ring in E^s, and the overlays
const glm::vec3 kPsiColor(1.0f, 0.62f, 0.25f);       // psi^t, the linear flow (orange)
const glm::vec3 kFlatColor(0.35f, 0.95f, 0.45f);     // phi^t from the same point of E^s (green)
const glm::vec3 kOnWColor(0.45f, 0.72f, 1.0f);       // phi^t from x* + K_s(theta), on W^s (blue)
const glm::vec3 kLinkColor(0.85f, 0.85f, 0.92f);     // psi^t <-> phi^t from the same seed, same t
const glm::vec3 kFormulaColor(1.0f, 0.92f, 0.2f);    // x* + K_s(e^{Lambda t} s)
// stage C: the linear ball U and its image V = x* + h~^{-1}(U)
const glm::vec3 kUColor(1.0f, 0.70f, 0.40f);         // tinted like psi^t
const glm::vec3 kVColor(0.45f, 1.0f, 0.55f);         // tinted like phi^t
const glm::vec3 kWarnColor(1.0f, 0.7f, 0.25f);
const glm::vec3 kBadColor(1.0f, 0.3f, 0.25f);

// stage C check: relative invariance defect of K_s / K_u that still counts as exact,
// and as a usable approximation; a Jacobian determinant of h~^{-1} below kFoldDet means
// it is (about to be) folding over — no longer one-to-one, so no homeomorphism.
constexpr double kDefectOk = 1e-3;
constexpr double kDefectRough = 5e-2;
constexpr double kFoldDet = 0.05;
constexpr float kHgSeedFraction = 0.6f;
constexpr float kMorphSeedFraction = 0.5f;   // "Transform flow" seeds, of rho
constexpr float kMorphPauseSeconds = 0.7f;
constexpr float kMinHgRadius = 0.05f;
constexpr double kSafeSearchMax = 25.0;   // the safe-radius search stops here   // pair seeds on the sphere 0.6 rho
constexpr int kMaxWavesPerBall = 8;        // glass_fragment.glsl's kMaxWaves
constexpr int kShardsPerCrash = 14;          // fragments a crashed particle breaks into
constexpr int kMaxShardGroups = 64;
// "Show conjugacy" timeline (seconds; "Hide" plays it backwards): the rest of the scene
// fades to black around V by kHgFocusEnd, then from kHgSideStart the camera eases out
// and U fades in beside V until kHgSideEnd
constexpr float kHgFocusEnd = 0.7f;
constexpr float kHgSideStart = 1.2f;
constexpr float kHgSideEnd = 2.8f;

// Quintic smoothstep: zero velocity and acceleration at both ends.
float smootherstep(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * t * (t * (6.0f * t - 15.0f) + 10.0f);
}

// A shader file with the line containing `marker` replaced by `insert` (how the
// ConjugacyMap GLSL gets spliced into the deformed ball's vertex shader).
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

const char* kSettingsPath = SIMS_DIR "lorenz_equilibrium/settings.txt";
const char* kFlowSim = "lorenz_flow";   // "back to global view" goes there

constexpr float kEnterSeconds = 0.6f;   // content fades in after the handoff
constexpr float kLeaveSeconds = 0.35f;  // ...and out before handing back

// UI section title: a red labelled separator with breathing room above (same look as
// the other flow sims).
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

float smoothstep(float a, float b, float x)
{
    float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

constexpr float kMinSpeed = 1e-4f;   // slowest flow (arrow keys ramp to it logarithmically)

constexpr int kTaylor = 3;   // singleMethod: Euler, RK2, RK4 (Integrator), then Taylor

// What the arrow lattice in V shows (stage A): the field, its linearization at x*, both
// at once, or what the linearization leaves out — the quadratic part Q(u, u).
enum ArrowField { kFieldF = 0, kFieldLinear = 1, kFieldBoth = 2, kFieldNonlinear = 3 };

// Below this min |Re lambda| the HUD says "nearly non-hyperbolic" (C+- at rho = 28 sit
// at 0.094 — near the Hopf point rho ~ 24.74), below kFlat "non-hyperbolic".
constexpr double kNearlyFlat = 0.2;
constexpr double kFlat = 1e-3;

// ---- settings -------------------------------------------------------------------

struct Settings {
    float sigma = 10.0f;
    float rho = 28.0f;
    float beta = 8.0f / 3.0f;
    int eq = 1;                   // which equilibrium: 0 = O, 1 = C+, 2 = C-
    bool showSystemLatex = true;
    float systemLatexSize = 30.0f;     // the system overlay, px
    float hudSize = 24.0f;             // the HUD under it (its text scales along)
    float subspaceLabelSize = 30.0f;   // E^s, E^u

    float ballRadius = 3.0f;      // r of V = B_r(x*)
    bool showGlass = true;
    float glassStrength = 1.0f;
    float beadRadius = 0.06f;     // much smaller than in lorenz_flow: x* is a point here
    bool showEqLabel = true;
    float eqLabelSize = 30.0f;

    bool showSubspaces = true;    // stage A: E^s, E^u through x*
    float subspaceAlpha = 0.16f;
    int arrowField = kFieldF;
    bool showHud = true;          // A = Df(x*), eigenvalues, dims, warning

    // stage B: a ring of seeds in E^s
    float ringRadius = 0.6f;      // of r
    int ringCount = 12;           // when E^s is a plane (a line gets the two points +-)
    bool showPsi = true;
    bool showFlat = true;
    bool showOnW = true;
    bool showLinks = true;
    bool showFormula = false;     // markers at x* + K_s(e^{Lambda t} s)
    int manifoldOrder = 12;       // Taylor order of K_s
    bool showStageLabels = true;  // phi^t(x_0) / psi^t(theta_0) on every seed's pair
    float stageLabelSize = 30.0f;

    // stage C: Hartman-Grobman
    float hgRadius = 1.5f;        // rho of the linear ball U = B_rho(x*)
    int hgPairs = 60;
    float hgGap = 1.0f;           // space between U and V, in units of rho

    // "Transform flow": linear trajectories carried from U to V
    float morphSeconds = 3.0f;    // the transformation itself
    float morphFlowTime = 5.0f;   // fallback: most ODE time the linear flow runs first

    // the crash shockwave on the glass (Other settings -> Crash shockwave)
    float waveIntensity = 2.5f;   // flash
    float waveReach = 20.0f;      // propagation: degrees of arc the ring travels
    float waveForce = 0.08f;      // distortion: push, in ball radii on screen
    float waveThickness = 0.075f; // of the reach
    float waveFeather = 0.1f;
    float waveAberration = 0.02f; // color split, in ball radii on screen
    float waveSeconds = 1.0f;
    bool showHgLinks = true;
    bool showHgLabels = true;     // phi^t(x_0) / psi^t(theta_0) on the pairs
    bool showHgPredicted = false; // x* + h~^{-1}(psi^t - x*): where the interpolation puts phi^t

    bool showArrows = true;
    bool barbedHeads = false;
    bool lengthByMagnitude = false;
    int gridN = 9;
    float arrowScale = 0.8f;
    float headFraction = 0.36f;
    float headWidth = 0.14f;
    float arrowAlpha = 0.5f;
    float arrowTailAlpha = 0.25f;

    float timeScale = 0.5f;
    bool reverse = false;
    int method = 2;               // Integrator::Method, RK4
    float maxStep = 0.002f;
    int singleMethod = 3;         // singles: Euler, RK2, RK4 or Taylor (kTaylor)
    int taylorOrder = 20;
    int taylorTolExp = 14;        // Taylor tolerance 1e-taylorTolExp

    int seedCount = 200;
    float seedRadius = 0.5f;      // of r
    bool seedFilled = false;
    float singleScale = 2.0f;

    bool showTrails = true;
    bool trailBySpeed = false;
    int trailPoints = 1500;
    float trailWidth = 1.5f;
    float trailAlpha = 0.8f;
    float particleSize = 6.0f;

    // view when opened from the menu (camRadius <= 0: 3 r); a handoff keeps the camera
    float camTheta = 0.6f, camPhi = 1.1f, camRadius = 0.0f;
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
        {"eq", nullptr, &s.eq},
        {"showSystemLatex", nullptr, nullptr, &s.showSystemLatex},
        {"systemLatexSize", &s.systemLatexSize}, {"hudSize", &s.hudSize},
        {"subspaceLabelSize", &s.subspaceLabelSize},
        {"ballRadius", &s.ballRadius},
        {"showGlass", nullptr, nullptr, &s.showGlass},
        {"glassStrength", &s.glassStrength}, {"beadRadius", &s.beadRadius},
        {"showEqLabel", nullptr, nullptr, &s.showEqLabel},
        {"eqLabelSize", &s.eqLabelSize},
        {"showSubspaces", nullptr, nullptr, &s.showSubspaces},
        {"subspaceAlpha", &s.subspaceAlpha},
        {"arrowField", nullptr, &s.arrowField},
        {"showHud", nullptr, nullptr, &s.showHud},
        {"ringRadius", &s.ringRadius}, {"ringCount", nullptr, &s.ringCount},
        {"showPsi", nullptr, nullptr, &s.showPsi},
        {"showFlat", nullptr, nullptr, &s.showFlat},
        {"showOnW", nullptr, nullptr, &s.showOnW},
        {"showLinks", nullptr, nullptr, &s.showLinks},
        {"showFormula", nullptr, nullptr, &s.showFormula},
        {"manifoldOrder", nullptr, &s.manifoldOrder},
        {"showStageLabels", nullptr, nullptr, &s.showStageLabels},
        {"stageLabelSize", &s.stageLabelSize},
        {"hgRadius", &s.hgRadius}, {"hgPairs", nullptr, &s.hgPairs},
        {"hgGap", &s.hgGap},
        {"morphSeconds", &s.morphSeconds}, {"morphFlowTime", &s.morphFlowTime},
        {"waveIntensity", &s.waveIntensity}, {"waveReach", &s.waveReach},
        {"waveForce", &s.waveForce}, {"waveThickness", &s.waveThickness},
        {"waveFeather", &s.waveFeather}, {"waveAberration", &s.waveAberration},
        {"waveSeconds", &s.waveSeconds},
        {"showHgLinks", nullptr, nullptr, &s.showHgLinks},
        {"showHgLabels", nullptr, nullptr, &s.showHgLabels},
        {"showHgPredicted", nullptr, nullptr, &s.showHgPredicted},
        {"showArrows", nullptr, nullptr, &s.showArrows},
        {"barbedHeads", nullptr, nullptr, &s.barbedHeads},
        {"lengthByMagnitude", nullptr, nullptr, &s.lengthByMagnitude},
        {"gridN", nullptr, &s.gridN},
        {"arrowScale", &s.arrowScale}, {"headFraction", &s.headFraction},
        {"headWidth", &s.headWidth},
        {"arrowAlpha", &s.arrowAlpha}, {"arrowTailAlpha", &s.arrowTailAlpha},
        {"timeScale", &s.timeScale},
        {"reverse", nullptr, nullptr, &s.reverse},
        {"method", nullptr, &s.method}, {"maxStep", &s.maxStep},
        {"singleMethod", nullptr, &s.singleMethod}, {"taylorOrder", nullptr, &s.taylorOrder},
        {"taylorTolExp", nullptr, &s.taylorTolExp},
        {"seedCount", nullptr, &s.seedCount}, {"seedRadius", &s.seedRadius},
        {"seedFilled", nullptr, nullptr, &s.seedFilled},
        {"singleScale", &s.singleScale},
        {"showTrails", nullptr, nullptr, &s.showTrails},
        {"trailBySpeed", nullptr, nullptr, &s.trailBySpeed},
        {"trailPoints", nullptr, &s.trailPoints},
        {"trailWidth", &s.trailWidth}, {"trailAlpha", &s.trailAlpha},
        {"particleSize", &s.particleSize},
        {"cameraTheta", &s.camTheta}, {"cameraPhi", &s.camPhi},
        {"cameraRadius", &s.camRadius},
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
    s.eq = std::clamp(s.eq, 0, 2);
    s.arrowField = std::clamp(s.arrowField, 0, 3);
    s.ringCount = std::clamp(s.ringCount, 3, 64);
    s.ringRadius = std::clamp(s.ringRadius, 0.01f, 1.0f);
    s.manifoldOrder = std::clamp(s.manifoldOrder, 1, 30);
    s.stageLabelSize = std::clamp(s.stageLabelSize, 10.0f, 120.0f);
    s.hgRadius = std::clamp(s.hgRadius, 0.01f, 50.0f);
    s.hgPairs = std::clamp(s.hgPairs, 1, 400);
    s.hgGap = std::clamp(s.hgGap, 0.0f, 10.0f);
    s.waveReach = std::clamp(s.waveReach, 0.5f, 180.0f);
    s.waveSeconds = std::clamp(s.waveSeconds, 0.1f, 10.0f);
    s.morphSeconds = std::clamp(s.morphSeconds, 0.2f, 20.0f);
    s.morphFlowTime = std::clamp(s.morphFlowTime, 0.001f, 20.0f);
    s.timeScale = std::clamp(s.timeScale, kMinSpeed, 10.0f);
    s.gridN = std::clamp(s.gridN, 2, 30);
    s.seedCount = std::clamp(s.seedCount, 1, 1000);
    s.trailPoints = std::clamp(s.trailPoints, 32, 20000);
    s.ballRadius = std::clamp(s.ballRadius, 0.05f, 50.0f);
    s.method = std::clamp(s.method, 0, 2);
    s.systemLatexSize = std::clamp(s.systemLatexSize, 10.0f, 120.0f);
    s.hudSize = std::clamp(s.hudSize, 10.0f, 120.0f);
    s.subspaceLabelSize = std::clamp(s.subspaceLabelSize, 10.0f, 160.0f);
    s.singleMethod = std::clamp(s.singleMethod, 0, kTaylor);
    s.taylorOrder = std::clamp(s.taylorOrder, 2, engine::TaylorIntegrator::kMaxOrder);
    s.taylorTolExp = std::clamp(s.taylorTolExp, 4, 16);
    s.maxStep = std::clamp(s.maxStep, 1e-5f, 0.5f);
}

// ---- the sim --------------------------------------------------------------------

// The local picture at one Lorenz equilibrium x*: the field and trajectories inside a
// glass ball V = B_r(x*), everything clipped to it. Entered from lorenz_flow by
// clicking a bead (App::requestSwitch + a handoff: same parameters, same camera);
// Backspace hands back. Mode 2 of the Hartman-Grobman track — stages A/B/C land here
// (M4-M6). See obsidian_twin/sims/lorenz-equilibrium.md.
class LorenzEquilibriumSim : public engine::Sim {
public:
    const char* name() const override { return "lorenz_equilibrium"; }

    void init(engine::App& app) override
    {
        this->app = &app;
        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(SIMS_DIR "lorenz_equilibrium/shaders/vertex.glsl",
                                SIMS_DIR "lorenz_equilibrium/shaders/fragment.glsl");
        eqShader = engine::Shader(SIMS_DIR "lorenz_equilibrium/shaders/equilibrium_vertex.glsl",
                                  SIMS_DIR "lorenz_equilibrium/shaders/equilibrium_fragment.glsl");
        glassShader = engine::Shader(SIMS_DIR "lorenz_equilibrium/shaders/equilibrium_vertex.glsl",
                                     SIMS_DIR "lorenz_equilibrium/shaders/glass_fragment.glsl");
        hmShader = engine::Shader::fromSources(
            loadWithInsert(SIMS_DIR "lorenz_equilibrium/shaders/conjugacy_vertex.glsl",
                           "@CONJUGACY_MAP", engine::ConjugacyMap::glsl()),
            loadWithInsert(SIMS_DIR "lorenz_equilibrium/shaders/glass_fragment.glsl", "@none", ""));
        hmShader.bindUniformBlock(engine::ConjugacyMap::kBlockName, engine::ConjugacyMap::kBinding);
        morphShader = engine::Shader::fromSources(
            loadWithInsert(SIMS_DIR "lorenz_equilibrium/shaders/morph_vertex.glsl",
                           "@CONJUGACY_MAP", engine::ConjugacyMap::glsl()),
            loadWithInsert(SIMS_DIR "lorenz_equilibrium/shaders/fragment.glsl", "@none", ""));
        morphShader.bindUniformBlock(engine::ConjugacyMap::kBlockName, engine::ConjugacyMap::kBinding);
        hgMesh = std::make_unique<engine::Mesh>(kMaxHgPairs * 3, std::vector<int>{3, 1});   // pos, gradient
        shardMesh = std::make_unique<engine::Mesh>(kMaxShardGroups * kShardsPerCrash, std::vector<int>{3});
        headMesh = std::make_unique<engine::Mesh>(kMaxHgPairs * 2, std::vector<int>{3});
        // the glass is big on screen: its own finely tessellated sphere so the silhouette
        // stays round (the bead and halo are small, 64 x 40 is plenty for them)
        glassMesh = std::make_unique<engine::Mesh>(engine::Shapes::sphereVertices(256, 160, 1.0f),
                                                   engine::Shapes::sphereIndices(256, 160),
                                                   std::vector<int>{3, 3, 2});
        sphereMesh = std::make_unique<engine::Mesh>(engine::Shapes::sphereVertices(64, 40, 1.0f),
                                                    engine::Shapes::sphereIndices(64, 40),
                                                    std::vector<int>{3, 3, 2});

        subspaceMesh = std::make_unique<engine::Mesh>(kSubspaceCapacity, std::vector<int>{3});
        stageMesh = std::make_unique<engine::Mesh>(kMaxPairs * 3, std::vector<int>{3});

        rhs = [this](const double* x, double, double* dxdt) { system.eval(x, dxdt); };
        jet = [this](const double* x, double, int order, double* c) { system.jet(x, order, c); };
        // psi^t: the linear field A(x - x*) and its jet (c_1 = A(x - x*), c_{k+1} = A c_k / (k+1)),
        // so stage B's linear curves run on the same method as the true ones
        rhsLinear = [this](const double* x, double, double* d) {
            for (int i = 0; i < 3; ++i)
                d[i] = A[i][0] * (x[0] - xs[0]) + A[i][1] * (x[1] - xs[1]) + A[i][2] * (x[2] - xs[2]);
        };
        jetLinear = [this](const double* x, double, int order, double* c) {
            for (int i = 0; i < 3; ++i)
                c[i] = x[i];
            for (int k = 0; k < order; ++k)
            {
                const double* a = &c[3 * k];
                double u[3] = {a[0], a[1], a[2]};
                if (k == 0)
                    for (int i = 0; i < 3; ++i)
                        u[i] -= xs[i];
                for (int i = 0; i < 3; ++i)
                    c[3 * (k + 1) + i] = (A[i][0] * u[0] + A[i][1] * u[1] + A[i][2] * u[2]) / (k + 1);
            }
        };

        loadSettingsFile(cfg, kSettingsPath);

        // from lorenz_flow: its parameters, its bead, and the global view to return to;
        // the camera is left where the zoom put it
        const engine::App::Handoff& in = app.handoff();
        auto value = [&](const char* key, double fallback) {
            auto it = in.find(key);
            return it == in.end() ? fallback : it->second;
        };
        handedOver = in.count("eq") > 0;
        // the bead arrives at lorenz_flow's focused size and shrinks to bead size
        beadShown = (float)value("beadRadius", cfg.beadRadius);
        if (handedOver)
        {
            cfg.sigma = (float)value("sigma", cfg.sigma);
            cfg.rho = (float)value("rho", cfg.rho);
            cfg.beta = (float)value("beta", cfg.beta);
            cfg.eq = std::clamp((int)value("eq", cfg.eq), 0, 2);
            for (const char* key : {"globalCx", "globalCy", "globalCz", "globalR"})
                if (in.count(key))
                    returnView[key] = in.at(key);
        }

        engine::Camera& cam = app.camera();
        cam.mode = engine::Camera::Mode::Orbit;
        cam.inputEnabled = true;
        cam.up = glm::vec3(0.0f, 1.0f, 0.0f);
        cam.orbitRadiusMin = 0.05f;
        cam.orbitSlowZone = 0.0f;
        cam.farPlane = 500.0f;
        applyParams();
        if (!handedOver)
            frameCamera();
    }

    void update(float dt) override
    {
        const ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false))
            playing = !playing;
        engine::FlowKeys::handle(cfg.timeScale, cfg.reverse, dt, 10.0f, kMinSpeed);
        engine::FlowKeys::drawStatus(cfg.timeScale, cfg.reverse, playing);
        // Backspace, not Escape: App closes the window on Escape
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Backspace, false))
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
            // stage B: psi^t on the linear field, the two phi^t on the true one, all on the
            // singles' method so the comparison is fair
            if (!pairs.empty())
            {
                std::vector<engine::Trajectory*> lin, tru;
                for (StagePair& q : pairs)
                {
                    lin.push_back(q.lin.get());
                    tru.push_back(q.flat.get());
                    tru.push_back(q.onW.get());
                }
                if (cfg.singleMethod == kTaylor)
                {
                    runner.advance(lin, taylor, jetLinear, h, cfg.maxStep);
                    runner.advance(tru, taylor, jet, h, cfg.maxStep);
                }
                else
                {
                    runner.advance(lin, singleIntegrator, rhsLinear, h);
                    runner.advance(tru, singleIntegrator, rhs, h);
                }
            }
            // "Transform flow": the linear flow runs until the trajectories are long enough
            if (morphPhase == MorphPhase::Flowing && !morphTrajs.empty())
            {
                std::vector<engine::Trajectory*> m;
                for (auto& t : morphTrajs)
                    m.push_back(t.get());
                if (cfg.singleMethod == kTaylor)
                    runner.advance(m, taylor, jetLinear, h, cfg.maxStep);
                else
                    runner.advance(m, singleIntegrator, rhsLinear, h);
                morphOde += std::abs(h);
            }
            // stage C: psi^t from x* + theta, phi^t from x* + h~^{-1}(theta)
            if (!hgPairList.empty())
            {
                std::vector<engine::Trajectory*> lin, tru;
                for (HgPair& q : hgPairList)
                {
                    // a crashed pair stops dead while it dissolves; the others remember
                    // where they were, so a crash can be placed exactly on the border
                    if (q.linCrashed || q.truCrashed)
                        continue;
                    std::copy_n(q.lin->state(), 3, q.linPrev);
                    std::copy_n(q.tru->state(), 3, q.truPrev);
                    lin.push_back(q.lin.get());
                    tru.push_back(q.tru.get());
                }
                if (cfg.singleMethod == kTaylor)
                {
                    runner.advance(lin, taylor, jetLinear, h, cfg.maxStep);
                    runner.advance(tru, taylor, jet, h, cfg.maxStep);
                }
                else
                {
                    runner.advance(lin, singleIntegrator, rhsLinear, h);
                    runner.advance(tru, singleIntegrator, rhs, h);
                }
            }
        }

        if (!pairs.empty())
            rebuildStageMesh();
        updateHgPairs(dt);
        updateMorph(dt);
        updateBursts(dt);
        updateShards(dt);

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !io.WantCaptureMouse && !leaving &&
            !hgActive)
            dropSingleUnderCursor();

        // content fades in after the handoff, out before handing back
        clock += dt;
        if (leaving)
        {
            leaveT += dt;
            if (leaveT >= kLeaveSeconds && !switchRequested)
            {
                switchRequested = true;
                handBack();
            }
        }
        float in = handedOver ? smoothstep(0.0f, kEnterSeconds, clock) : 1.0f;
        float out = leaving ? 1.0f - smoothstep(0.0f, kLeaveSeconds, leaveT) : 1.0f;
        advanceHgTimeline(dt);
        hgFade = in * out;
        contentFade = hgFade * (1.0f - hgFocus);   // the usual scene goes black around V
        beadShown += (cfg.beadRadius - beadShown) * (1.0f - std::exp(-3.0f * dt));

        updateCamera();

        if (arrowsDirty)
            rebuildArrows();
        if (subspacesDirty)
            rebuildSubspaces();
        if (hgActive && hgArrowsDirty)
            rebuildHgArrows();

        drawSystemOverlay();
        drawEquilibriumLabel();
        drawSubspaceLabels();
        drawStageLabels();
        drawHgLabels();
        drawHgPairLabels();

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

        renderBead(camera, view, projection, false, xStar, 1.0f);
        if (hgActive && hgSide > 0.01f)
            renderBead(camera, view, projection, false, xStar + hgOffset(), hgSide * hgFade);

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
        // everything in this pass lives inside V
        shader.setVec3("u_clipCenter", xStar);
        shader.setFloat("u_clipRadius", cfg.ballRadius);

        if (cfg.showArrows && !glyphs.empty())
        {
            glDepthMask(GL_FALSE);
            shader.setInt("u_useRamp", 1);
            shader.setInt("u_useFade", 1);
            shader.setFloat("u_alpha", cfg.arrowAlpha * contentFade);
            shader.setFloat("u_alphaTail", cfg.arrowTailAlpha);
            glLineWidth(1.5f * px);
            glyphs.drawShafts(shader);
            shader.setInt("u_useFade", 0);
            shader.setInt("u_shadeMode", 1);
            glyphs.drawHeads(shader);
            shader.setInt("u_useRamp", 0);
            // "both": A(x - x*) in flat white over the colored f, from the same points —
            // they agree near x* and part ways toward the glass
            if (cfg.arrowField == kFieldBoth && !linearGlyphs.empty())
            {
                shader.setVec3("u_color", kLinearArrowColor);
                shader.setInt("u_useFade", 1);
                shader.setInt("u_shadeMode", 0);
                linearGlyphs.drawShafts(shader);
                shader.setInt("u_useFade", 0);
                shader.setInt("u_shadeMode", 1);
                linearGlyphs.drawHeads(shader);
            }
            shader.setInt("u_shadeMode", 0);
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
                shader.setFloat("u_alpha", cfg.trailAlpha * contentFade);
                c.traj->draw();
                shader.setInt("u_useRamp", 0);
            }
            shader.setFloat("u_alpha", contentFade);
            shader.setVec3("u_color", c.traj->diverged() ? kDivergedColor : c.color);
            shader.setFloat("u_pointSize", cfg.particleSize * k * px);
            shader.setInt("u_round", 1);
            c.traj->drawHead();
            shader.setInt("u_round", 0);
        }
        renderStageB(px);
        renderHgPairs(px);
        renderShards(px);
        renderMorph(view, projection, px);
        runner.endDraw();
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_clipRadius", 0.0f);   // the subspaces end exactly on the glass
        if (cfg.showSubspaces)
            renderSubspaces(px);
        glLineWidth(1.0f);

        renderBead(camera, view, projection, true, xStar, 1.0f);
        if (hgActive && hgSide > 0.01f)
            renderBead(camera, view, projection, true, xStar + hgOffset(), hgSide * hgFade);

        // the glass last: it tints whatever is inside and behind it
        if (cfg.showGlass)
        {
            glDepthMask(GL_FALSE);
            glassShader.use();
            glassShader.setMat4("view", view);
            glassShader.setMat4("projection", projection);
            glassShader.setVec3("u_eye", camera.position);
            glassShader.setVec3("u_center", xStar);
            glassShader.setFloat("u_radius", cfg.ballRadius);
            glassShader.setVec3("u_color", kGlassColor);
            glassShader.setFloat("u_alpha", cfg.glassStrength * contentFade);
            glassShader.setInt("u_waveCount", 0);
            glassMesh->draw(GL_TRIANGLES);
            glDepthMask(GL_TRUE);
        }
        renderHgBalls(camera, view, projection);
        renderMorphGhost(camera, view, projection);
        glDisable(GL_BLEND);
    }

    void drawUI() override
    {
        drawEquilibriumUI();
        drawSystemUI();
        drawLinearizationUI();
        drawStageBUI();
        drawHartmanGrobmanUI();
        drawFlowUI();
        drawSeedsUI();

        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.36f, 0.30f, 1.0f));
        bool showOther = ImGui::CollapsingHeader("Other settings (camera, integrator, ball, shockwave, LaTeX, look, arrows, trails, performance)");
        ImGui::PopStyleColor();
        if (showOther)
        {
            ImGui::Indent();
            drawViewUI();
            drawIntegratorUI();
            drawBallUI();
            drawShockwaveUI();
            drawLatexUI();
            drawLookUI();
            drawArrowUI();
            drawTrailUI();
            sectionHeader("Performance");
            runner.drawUI();
            ImGui::Unindent();
        }

        sectionHeader("Settings");
        if (ImGui::Button("Save settings"))
            settingsStatus = saveCurrentSettings() ? "saved" : "could not write settings file";
        if (!settingsStatus.empty())
        {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", settingsStatus.c_str());
        }

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
    void takeScreenshot()
    {
        std::time_t now = std::time(nullptr);
        std::tm local{};
        localtime_s(&local, &now);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
        std::string path = std::string(SIMS_DIR "../screenshots/lorenz_equilibrium_") + stamp + ".png";
        shotStatus = app->captureScreenshot(path, shotScale)
            ? "saved screenshots/" + std::filesystem::path(path).filename().string()
            : "screenshot failed (see console)";
    }

    enum class Kind { Seed, Single };

    struct Curve {
        std::unique_ptr<engine::Trajectory> traj;
        glm::vec3 seed;
        glm::vec3 color;
        Kind kind;
    };

    // ---- UI sections ------------------------------------------------------------

    void drawEquilibriumUI()
    {
        sectionHeader("Equilibrium");
        for (int i = 0; i < eqCount; ++i)
        {
            if (i > 0)
                ImGui::SameLine();
            if (ImGui::RadioButton(found[i].name.c_str(), cfg.eq == i) && cfg.eq != i)
            {
                cfg.eq = i;
                applyParams();
                curves.clear();   // seeds belong to the old ball
            }
        }
        if (app->hasSim(kFlowSim))
        {
            if (ImGui::Button("Back to global view"))
                backToGlobal();
            ImGui::SameLine();
            ImGui::TextDisabled("(Backspace)");
        }
    }

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
    }

    void drawBallUI()
    {
        sectionHeader("Ball V");
        if (ImGui::SliderFloat("radius r", &cfg.ballRadius, 0.1f, 15.0f, "%.2f"))
            arrowsDirty = subspacesDirty = true;
        ImGui::Checkbox("glass", &cfg.showGlass);
        if (cfg.showGlass)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140.0f);
            ImGui::SliderFloat("##glassStrength", &cfg.glassStrength, 0.1f, 2.0f, "strength %.2f");
        }
        ImGui::SetNextItemWidth(120.0f);
        if (ImGui::InputInt("K_s order", &cfg.manifoldOrder))
        {
            cfg.manifoldOrder = std::clamp(cfg.manifoldOrder, 1, 30);
            buildManifold();
        }
    }

    // Sizes of everything typeset: the system, the HUD, the x* label, E^s / E^u.
    void drawLatexUI()
    {
        sectionHeader("LaTeX");
        if (ImGui::SliderFloat("equation", &cfg.systemLatexSize, 14.0f, 72.0f, "%.0f px"))
            latexDirty = true;
        if (ImGui::SliderFloat("HUD", &cfg.hudSize, 14.0f, 72.0f, "%.0f px"))
            hudDirty = true;
        ImGui::SliderFloat("equilibrium label", &cfg.eqLabelSize, 14.0f, 72.0f, "%.0f px");
        ImGui::SliderFloat("E^s, E^u labels", &cfg.subspaceLabelSize, 14.0f, 72.0f, "%.0f px");
        ImGui::SliderFloat("phi^t, psi^t labels", &cfg.stageLabelSize, 14.0f, 72.0f, "%.0f px");
    }

    // Stage A: the linear picture at x* next to the true one.
    void drawLinearizationUI()
    {
        sectionHeader("Linearization");
        ImGui::Checkbox("E^s, E^u", &cfg.showSubspaces);
        if (cfg.showSubspaces)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            ImGui::SliderFloat("##subspaceAlpha", &cfg.subspaceAlpha, 0.0f, 0.6f, "planes %.2f");
        }
        ImGui::SameLine();
        ImGui::Checkbox("HUD", &cfg.showHud);
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::Combo("arrows", &cfg.arrowField, "f(x)\0A(x - x*)\0both\0f(x) - A(x - x*)\0"))
            arrowsDirty = true;
    }

    // Stage B: a ring of seeds in E^s, three curves each (labels in their colors).
    void drawStageBUI()
    {
        sectionHeader("Stable side: phi^t vs psi^t");
        if (!manifoldOk)
        {
            ImGui::TextDisabled("no stable manifold to show here: %s", Ws.error().c_str());
            return;
        }
        ImGui::SliderFloat("ring radius", &cfg.ringRadius, 0.05f, 1.0f, "%.2f of r");
        if (Ws.dim() == 2)
            ImGui::SliderInt("ring seeds", &cfg.ringCount, 3, kMaxPairs);
        if (ImGui::Button("Seed ring"))
            seedRing();
        ImGui::SameLine();
        if (ImGui::Button("Clear ring"))
            pairs.clear();
        auto colored = [](const char* label, bool* v, const glm::vec3& c) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(c.r, c.g, c.b, 1.0f));
            ImGui::Checkbox(label, v);
            ImGui::PopStyleColor();
        };
        colored("psi^t (linear)", &cfg.showPsi, kPsiColor);
        ImGui::SameLine();
        colored("phi^t from E^s", &cfg.showFlat, kFlatColor);
        ImGui::SameLine();
        colored("phi^t from W^s", &cfg.showOnW, kOnWColor);
        ImGui::Checkbox("labels", &cfg.showStageLabels);
        ImGui::SameLine();
        colored("links", &cfg.showLinks, kLinkColor);
        ImGui::SameLine();
        colored("check x* + K(e^{Lt} s)", &cfg.showFormula, kFormulaColor);
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
        {
            curves.clear();
            pairs.clear();
            hgPairList.clear();
        }
        ImGui::SliderFloat("speed", &cfg.timeScale, kMinSpeed, 10.0f, "%.4f x",
                           ImGuiSliderFlags_Logarithmic);
    }

    void drawSeedsUI()
    {
        sectionHeader("Trajectories in V");
        ImGui::SliderInt("particles", &cfg.seedCount, 1, 1000);
        ImGui::SliderFloat("seed radius", &cfg.seedRadius, 0.05f, 1.0f, "%.2f of r");
        ImGui::Checkbox("fill volume", &cfg.seedFilled);
        ImGui::SameLine();
        if (ImGui::Button("Drop seeds"))
            dropSeeds();
        ImGui::SameLine();
        if (ImGui::Button("Remove seeds"))
            removeKind(Kind::Seed);
        ImGui::TextDisabled("click inside V to drop a single");
        ImGui::SameLine();
        if (ImGui::Button("Clear singles"))
            removeKind(Kind::Single);
    }

    // Under Other settings: the bead, its label, the singles' thickness.
    void drawLookUI()
    {
        sectionHeader("Look");
        ImGui::SliderFloat("bead size", &cfg.beadRadius, 0.01f, 1.0f, "%.3f");
        ImGui::Checkbox("label", &cfg.showEqLabel);
        ImGui::SliderFloat("singles thickness", &cfg.singleScale, 1.0f, 4.0f, "%.1f x");
    }

    void drawArrowUI()
    {
        sectionHeader("Arrow field");
        ImGui::Checkbox("show arrows", &cfg.showArrows);
        arrowsDirty |= ImGui::SliderInt("grid N", &cfg.gridN, 2, 30);
        arrowsDirty |= ImGui::SliderFloat("arrow scale", &cfg.arrowScale, 0.1f, 1.5f);
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
            arrowsDirty |= ImGui::SliderFloat("head length", &cfg.headFraction, 0.1f, 0.7f,
                                              "%.2f of arrow");
            arrowsDirty |= ImGui::SliderFloat("head width", &cfg.headWidth, 0.04f, 0.5f,
                                              "%.2f of head length");
        }
        arrowsDirty |= ImGui::Checkbox("length by magnitude", &cfg.lengthByMagnitude);
        ImGui::SliderFloat("arrow alpha", &cfg.arrowAlpha, 0.05f, 1.0f);
        ImGui::SliderFloat("alpha gradient", &cfg.arrowTailAlpha, 0.0f, 1.0f, "tail %.2f of tip");
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
        ImGui::Combo("seeds", &cfg.method, "Euler\0RK2 (midpoint)\0RK4\0");
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
        ImGui::SliderFloat("zoom speed", &cfg.zoomSpeed, 0.0f, 20.0f,
                           cfg.zoomSpeed > 0.0f ? "%.1f /s" : "auto");
        if (ImGui::Button("Reset camera"))
        {
            cfg.camRadius = 0.0f;
            cfg.camTheta = 0.6f;
            cfg.camPhi = 1.1f;
            frameCamera();
        }
    }

    // ---- equilibrium, camera, handoff ---------------------------------------------

    // New parameters or a new choice of x*: the field, the point, the arrows.
    void applyParams()
    {
        system.sigma = cfg.sigma;
        system.rho = cfg.rho;
        system.beta = cfg.beta;
        found = system.equilibria();
        eqCount = (int)std::min<size_t>(found.size(), 3);
        cfg.eq = std::clamp(cfg.eq, 0, std::max(eqCount - 1, 0));   // C+- merge into O at rho <= 1
        const auto& x = found[cfg.eq].x;
        xStar = glm::vec3((float)x[0], (float)x[1], (float)x[2]);
        std::string tex = found[cfg.eq].name == "O" ? "O" : found[cfg.eq].name == "C+" ? "C_+" : "C_-";
        if (tex != labelTex)
        {
            labelTex = tex;
            labelSize = 0.0f;
        }
        double J[3][3];
        system.jacobian(found[cfg.eq].x, J);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                A[i][j] = J[i][j];
        eig.compute(J);
        for (int i = 0; i < 3; ++i)
            xs[i] = found[cfg.eq].x[i];
        buildManifold();
        arrowsDirty = true;
        latexDirty = true;
        subspacesDirty = true;
        hudDirty = true;
    }

    // ---- stage A: E^s, E^u, the HUD -------------------------------------------------

    // Orthonormal basis of E^s (sign < 0) or E^u (sign > 0), Gram-Schmidt over Eigen3's
    // real basis ({Re v, Im v} for a complex pair).
    std::vector<glm::vec3> subspaceBasis(int sign) const
    {
        std::vector<glm::vec3> out;
        for (const auto& b : eig.realBasis(sign))
        {
            glm::dvec3 v(b[0], b[1], b[2]);
            for (const glm::vec3& e : out)
                v -= glm::dot(v, glm::dvec3(e)) * glm::dvec3(e);
            if (glm::length(v) > 1e-9)
                out.push_back(glm::vec3(glm::normalize(v)));
        }
        return out;
    }

    // E^s (blue) and E^u (red) through x*, cut to V: a line is a diameter with two
    // arrows along it (in toward x* on E^s, out on E^u); a plane is a translucent disk
    // with its rim and two rings of A u arrows in it (A maps E to itself, so they lie in
    // the disk and show the rotation of a complex pair). All of R^3 (sink/source): none.
    void rebuildSubspaces()
    {
        subspacesDirty = false;
        subspaceDraws.clear();
        std::vector<float> v;
        auto vert = [&](const glm::vec3& p) { v.insert(v.end(), {p.x, p.y, p.z}); };
        const float r = cfg.ballRadius;
        constexpr int kSegments = 256;   // as fine as the glass, so the disk rim stays round

        for (int side = 0; side < 2; ++side)
        {
            const int sign = side == 0 ? -1 : 1;
            const glm::vec3 color = side == 0 ? kStableColor : kUnstableColor;
            engine::ArrowGlyphs& arrows = subspaceArrows[side];
            arrows.clear();
            arrows.headFraction = 0.4f;
            arrows.headWidth = 0.32f;
            std::vector<glm::vec3> e = subspaceBasis(sign);
            labelAnchor[side] = glm::vec3(NAN);
            dims[side] = (int)e.size();

            auto linearAt = [&](const glm::vec3& u) {
                glm::vec3 out(0.0f);
                for (int c = 0; c < 3; ++c)
                    out[c] = (float)(A[c][0] * u.x + A[c][1] * u.y + A[c][2] * u.z);
                return out;
            };

            if (e.size() == 1)
            {
                int first = (int)v.size() / 3;
                vert(xStar - r * e[0]);
                vert(xStar + r * e[0]);
                subspaceDraws.push_back({GL_LINES, first, 2, color, false});
                for (float s : {-1.0f, 1.0f})
                {
                    glm::vec3 u = s * 0.6f * r * e[0];
                    glm::vec3 d = glm::normalize(linearAt(u));
                    arrows.add(xStar + u - 0.09f * r * d, d, 0.18f * r, 1.0f);
                }
                labelAnchor[side] = xStar + 0.85f * r * e[0];
            }
            else if (e.size() == 2)
            {
                auto rim = [&](int k) {
                    float a = 6.2831853f * (float)k / kSegments;
                    return xStar + r * (std::cos(a) * e[0] + std::sin(a) * e[1]);
                };
                int first = (int)v.size() / 3;
                for (int k = 0; k < kSegments; ++k)
                {
                    vert(xStar);
                    vert(rim(k));
                    vert(rim(k + 1));
                }
                subspaceDraws.push_back({GL_TRIANGLES, first, kSegments * 3, color, true});
                first = (int)v.size() / 3;
                for (int k = 0; k < kSegments; ++k)
                    vert(rim(k));
                subspaceDraws.push_back({GL_LINE_LOOP, first, kSegments, color, false});
                for (float ring : {0.45f, 0.8f})
                    for (int k = 0; k < 10; ++k)
                    {
                        float a = 6.2831853f * ((float)k + (ring < 0.5f ? 0.5f : 0.0f)) / 10.0f;
                        glm::vec3 u = ring * r * (std::cos(a) * e[0] + std::sin(a) * e[1]);
                        glm::vec3 d = glm::normalize(linearAt(u));
                        arrows.add(xStar + u - 0.06f * r * d, d, 0.12f * r, 1.0f);
                    }
                labelAnchor[side] = xStar + 0.92f * r * glm::normalize(e[0] + e[1]);
            }
            arrows.upload();
        }
        subspaceMesh->setVertices(v.data(), std::min(v.size(), (size_t)kSubspaceCapacity * 3));
    }

    void renderSubspaces(float px)
    {
        shader.setInt("u_useRamp", 0);
        shader.setInt("u_useFade", 0);
        shader.setInt("u_shadeMode", 0);
        // lines and arrows first (opaque-ish, with depth), the disks last without depth
        // writes so trails show through them
        glLineWidth(2.5f * px);
        for (const SubspaceDraw& d : subspaceDraws)
        {
            if (d.fill)
                continue;
            shader.setVec3("u_color", d.color);
            shader.setFloat("u_alpha", 0.9f * contentFade);
            subspaceMesh->drawArrays(d.mode, d.count, d.first);
        }
        for (int side = 0; side < 2; ++side)
        {
            if (subspaceArrows[side].empty())
                continue;
            shader.setVec3("u_color", side == 0 ? kStableColor : kUnstableColor);
            shader.setFloat("u_alpha", 0.95f * contentFade);
            subspaceArrows[side].drawShafts(shader);
            shader.setInt("u_shadeMode", 1);
            subspaceArrows[side].drawHeads(shader);
            shader.setInt("u_shadeMode", 0);
        }
        glDepthMask(GL_FALSE);
        for (const SubspaceDraw& d : subspaceDraws)
        {
            if (!d.fill)
                continue;
            shader.setVec3("u_color", d.color);
            shader.setFloat("u_alpha", cfg.subspaceAlpha * contentFade);
            subspaceMesh->drawArrays(d.mode, d.count, d.first);
        }
        glDepthMask(GL_TRUE);
        shader.setFloat("u_alpha", 1.0f);
    }

    // E^s / E^u beside their line or disk, in its color.
    void drawSubspaceLabels()
    {
        if (!cfg.showSubspaces)
            return;
        for (int side = 0; side < 2; ++side)
        {
            if (std::isnan(labelAnchor[side].x))
                continue;
            if (subspaceLabelTypeset[side] != cfg.subspaceLabelSize)
            {
                subspaceLabelTypeset[side] = cfg.subspaceLabelSize;
                subspaceLabel[side].set(side == 0 ? "E^s" : "E^u", cfg.subspaceLabelSize,
                                        side == 0 ? kStableColor : kUnstableColor);
            }
            float sx, sy;
            if (toScreen(labelAnchor[side], sx, sy))
                subspaceLabel[side].drawAt(sx + 6.0f, sy - (float)subspaceLabel[side].height(),
                                           contentFade);
        }
    }

    // The HUD pieces, typeset when the linearization changes: A = Df(x*) with its
    // numbers and the eigenvalues (a conjugate pair as a +- b i).
    void typesetHud()
    {
        hudDirty = false;
        auto num = [](double x) { return texNumber((float)x); };
        std::string m = R"(A = Df(x^*) = \begin{pmatrix})";
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
                m += num(A[i][j]) + (j < 2 ? " & " : "");
            m += i < 2 ? R"( \\ )" : "";
        }
        m += R"(\end{pmatrix})";
        hudMatrix.set(m, cfg.hudSize);

        std::string l;
        const auto& vals = eig.values();
        for (int i = 0; i < 3; ++i)
        {
            if (!l.empty())
                l += R"(,\quad )";
            if (eig.isComplex(i) && i + 1 < 3)
            {
                l += R"(\lambda_{)" + std::to_string(i + 1) + "," + std::to_string(i + 2) + "} = " +
                     num(vals[i].real()) + R"( \pm )" + num(std::abs(vals[i].imag())) + "i";
                ++i;
            }
            else
                l += R"(\lambda_{)" + std::to_string(i + 1) + "} = " + num(vals[i].real());
        }
        hudEigen.set(l, cfg.hudSize);
    }

    void drawHud()
    {
        if (hudDirty)
            typesetHud();
        ImGui::SetWindowFontScale(cfg.hudSize / 24.0f);   // the plain-text lines follow
        ImGui::Separator();
        hudMatrix.draw();
        hudEigen.draw();
        ImGui::TextColored(ImVec4(kStableColor.r, kStableColor.g, kStableColor.b, 1.0f),
                           "dim E^s = %d", eig.dimStable());
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(kUnstableColor.r, kUnstableColor.g, kUnstableColor.b, 1.0f),
                           "   dim E^u = %d", eig.dimUnstable());

        // Hartman-Grobman needs no eigenvalue on the imaginary axis
        double flat = 1e30;
        for (const auto& l : eig.values())
            flat = std::min(flat, std::abs(l.real()));
        if (flat < kFlat)
            ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.25f, 1.0f),
                               "non-hyperbolic: Re lambda = 0, Hartman-Grobman does not apply");
        else if (flat < kNearlyFlat)
            ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.25f, 1.0f),
                               "nearly non-hyperbolic (min |Re lambda| = %.3f):\n"
                               "the conjugacy only holds in a small ball", flat);
        ImGui::SetWindowFontScale(1.0f);
    }

    // ---- stage B: phi^t vs psi^t from E^s -------------------------------------------

    // K_s for the current x* (parametrization method); the ring belongs to the old one.
    void buildManifold()
    {
        pairs.clear();
        manifoldOk = Ws.build(A, engine::LorenzSystem::quadratic(),
                              engine::InvariantManifold::Side::Stable, cfg.manifoldOrder);
        buildConjugacy();
    }

    // ---- stage C: Hartman-Grobman --------------------------------------------------

    // h~^{-1} = K_s + K_u at this x* (orders capped at ConjugacyMap's GLSL limit); the
    // pairs and the check belong to the old one.
    void buildConjugacy()
    {
        hgPairList.clear();
        stopMorph();
        hgCheckDirty = true;
        hgArrowsDirty = true;
        const int order = std::min(cfg.manifoldOrder, engine::ConjugacyMap::kMaxOrder);
        engine::InvariantManifold ws, wu;
        const auto Q = engine::LorenzSystem::quadratic();
        hmOk = ws.build(A, Q, engine::InvariantManifold::Side::Stable, order) &&
               wu.build(A, Q, engine::InvariantManifold::Side::Unstable, order) &&
               hm.set(ws, wu);
        if (!hmOk)
            hmError = !ws.valid() ? ws.error() : !wu.valid() ? wu.error() : hm.error();
    }

    // Largest relative invariance defect of K_s and K_u over the linear ball of radius
    // rho: each theta on the sphere |theta| = rho splits into its E^s and E^u parts, and
    // each part's manifold is checked where it is used (the defect grows with |theta|,
    // so the sphere is where it peaks).
    double hgDefect(double rho) const
    {
        const int n = 160;
        double worst = 0.0;
        for (int k = 0; k < n; ++k)
        {
            double y = 1.0 - 2.0 * (k + 0.5) / n, rad = std::sqrt(std::max(0.0, 1.0 - y * y));
            double phi = 2.39996323 * k;
            engine::ConjugacyMap::Vec3 theta = {rho * std::cos(phi) * rad, rho * y,
                                                rho * std::sin(phi) * rad};
            engine::ConjugacyMap::Param ps, pu;
            hm.split(theta, ps, pu);
            const engine::InvariantManifold* sides[2] = {&hm.stable(), &hm.unstable()};
            const engine::ConjugacyMap::Param params[2] = {ps, pu};
            for (int side = 0; side < 2; ++side)
            {
                const auto& W = *sides[side];
                auto K = W.eval(params[side]);
                double x[3] = {xs[0] + K[0], xs[1] + K[1], xs[2] + K[2]}, f[3];
                system.eval(x, f);
                double fl = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
                if (fl > 1e-12)
                    worst = std::max(worst, W.defect(params[side]) / fl);
            }
        }
        return worst;
    }

    // Smallest Jacobian determinant of h~^{-1} over the linear ball (central
    // differences on a lattice). It is 1 at x*; where it drops to 0 the map folds over
    // and stops being one-to-one.
    double hgMinDet(double rho) const
    {
        const int n = 7;
        const double e = 1e-4 * std::max(rho, 1.0);
        double worst = 1e30;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                for (int k = 0; k < n; ++k)
                {
                    engine::ConjugacyMap::Vec3 t = {rho * (2.0 * i / (n - 1) - 1.0),
                                                    rho * (2.0 * j / (n - 1) - 1.0),
                                                    rho * (2.0 * k / (n - 1) - 1.0)};
                    if (t[0] * t[0] + t[1] * t[1] + t[2] * t[2] > rho * rho * 1.0001)
                        continue;
                    double J[3][3];
                    for (int c = 0; c < 3; ++c)
                    {
                        auto a = t, b = t;
                        a[c] += e;
                        b[c] -= e;
                        auto ha = hm.hInv(a), hb = hm.hInv(b);
                        for (int r = 0; r < 3; ++r)
                            J[r][c] = (ha[r] - hb[r]) / (2.0 * e);
                    }
                    double det = J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1])
                               - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0])
                               + J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
                    worst = std::min(worst, det);
                }
        return worst;
    }

    // Is the linear ball of radius rho small enough for h~^{-1}: both Taylor maps still
    // solve the invariance equation there, and h~^{-1} does not fold? Plus the largest
    // radius that passes (bisection, up to 2 r).
    void runHgCheck()
    {
        hgCheckDirty = false;
        if (!hmOk)
            return;
        auto good = [&](double r) { return hgDefect(r) < kDefectOk && hgMinDet(r) >= kFoldDet; };
        double lo = 0.0, hi = kSafeSearchMax;
        if (good(hi))
            lo = hi;
        else
            for (int it = 0; it < 14; ++it)
            {
                double mid = 0.5 * (lo + hi);
                (good(mid) ? lo : hi) = mid;
            }
        hgCheck.safe = lo;
        hgCheck.safeIsMax = lo >= kSafeSearchMax;

        // the linear ball never goes past where the conjugacy holds at this x*
        const float maxRho = std::max((float)hgCheck.safe, kMinHgRadius);
        if (cfg.hgRadius > maxRho)
        {
            cfg.hgRadius = maxRho;
            hgArrowsDirty = true;
            if (hgActive)
                seedHgPairs();
        }
        const double rho = cfg.hgRadius;
        hgCheck.defect = hgDefect(rho);
        hgCheck.minDet = hgMinDet(rho);
        hgCheck.folds = hgCheck.minDet < kFoldDet;
        hgCheck.level = hgCheck.folds || hgCheck.defect > kDefectRough ? 2
                      : hgCheck.defect > kDefectOk ? 1 : 0;
    }

    // Where U sits: beside V along the screen-right of the moment "Show" was pressed,
    // centres 2 rho + gap apart.
    glm::vec3 hgOffset() const
    {
        return hgDir * (cfg.hgRadius * (2.0f + cfg.hgGap));
    }

    // Show: start the timeline (remembering the camera distance and which way is right
    // on screen) and seed the pairs; Hide: run it backwards — the pairs go at the end.
    void toggleConjugacy()
    {
        hgTarget = !hgTarget;
        if (hgTarget && !hgActive)
        {
            const engine::Camera& cam = app->camera();
            hgSavedRadius = std::max(cam.orbitRadius, 1e-3f);
            glm::vec3 right = glm::cross(cam.front, cam.up);
            hgDir = glm::length(right) > 1e-4f ? glm::normalize(right) : glm::vec3(1.0f, 0.0f, 0.0f);
            hgActive = true;
            hgArrowsDirty = true;
            seedHgPairs();
        }
    }

    void advanceHgTimeline(float dt)
    {
        const float before = hgP;
        hgP = hgTarget ? std::min(hgP + dt, kHgSideEnd) : std::max(hgP - dt, 0.0f);
        hgAnimating = hgP != before;
        hgFocus = smoothstep(0.0f, kHgFocusEnd, hgP);
        hgSide = smootherstep((hgP - kHgSideStart) / (kHgSideEnd - kHgSideStart));
        if (hgActive && !hgTarget && hgP <= 0.0f)
        {
            hgActive = false;
            hgPairList.clear();
            stopMorph();
        }
    }

    // The two arrow fields of stage C on one lattice theta in B_rho: in V the true field
    // at x* + h~^{-1}(theta), in U the linear one, A theta, at x* + theta (drawn shifted
    // by hgOffset). One color/length scale for both.
    void rebuildHgArrows()
    {
        hgArrowsDirty = false;
        hgArrowsU.clear();
        hgArrowsV.clear();
        if (!hmOk)
            return;
        const float rho = cfg.hgRadius;
        const int n = cfg.gridN;
        const float spacing = 2.0f * rho / (float)n;
        std::vector<glm::vec3> pu, vu, pv, vv;
        for (int k = 0; k <= n; ++k)
            for (int j = 0; j <= n; ++j)
                for (int i = 0; i <= n; ++i)
                {
                    glm::vec3 t(-rho + i * spacing, -rho + j * spacing, -rho + k * spacing);
                    if (glm::length(t) > rho)
                        continue;
                    glm::vec3 lin(0.0f);
                    for (int c = 0; c < 3; ++c)
                        lin[c] = (float)(A[c][0] * t.x + A[c][1] * t.y + A[c][2] * t.z);
                    pu.push_back(xStar + t);
                    vu.push_back(lin);
                    auto H = hm.hInv({t.x, t.y, t.z});
                    double x[3] = {xs[0] + H[0], xs[1] + H[1], xs[2] + H[2]}, f[3];
                    system.eval(x, f);
                    pv.push_back(glm::vec3((float)x[0], (float)x[1], (float)x[2]));
                    vv.push_back(glm::vec3((float)f[0], (float)f[1], (float)f[2]));
                }
        hgMaxMag = 1e-6f;
        for (const auto* list : {&vu, &vv})
            for (const glm::vec3& w : *list)
                hgMaxMag = std::max(hgMaxMag, glm::length(w));
        for (auto* g : {&hgArrowsU, &hgArrowsV})
        {
            g->barbs = cfg.barbedHeads;
            g->headFraction = cfg.headFraction;
            g->headWidth = cfg.headWidth;
        }
        hgArrowsU.addField(pu, vu, spacing * cfg.arrowScale, hgMaxMag, 0.0f, cfg.lengthByMagnitude, 3);
        hgArrowsV.addField(pv, vv, spacing * cfg.arrowScale, hgMaxMag, 0.0f, cfg.lengthByMagnitude, 3);
        hgArrowsU.upload();
        hgArrowsV.upload();
    }

    // Stage C labels: phi^t(x_0) over every green head in V, psi^t(theta_0) over every
    // orange one in U, and a boxed h^{-1} at the middle of every link — the link is the
    // map, from the linear orbit's point to its partner. Same typeset pieces as stage B.
    void drawHgPairLabels()
    {
        if (!hgActive || hgPairList.empty())
            return;
        const float inV = hgFocus * hgFade, inU = hgSide * hgFade;
        const glm::vec3 off = hgOffset();
        if (cfg.showHgLabels)
        {
            drawStageLabelsPrepare();
            StageLabelGlyphs& g = stageGlyphs;
            const float headRadius = 0.5f * cfg.particleSize * cfg.singleScale;
            for (const HgPair& q : hgPairList)
            {
                double t = q.tru->time() - q.t0;
                if (std::abs(t) < 0.005)
                    t = 0.0;
                char num[32];
                std::snprintf(num, sizeof(num), "%.2f", t);
                if (inV > 0.01f)
                    drawFlowLabel(q.tru->head(), g.phi, g.phiChars, g.x0, num, true, headRadius,
                                  inV * std::max(q.truFade, 0.0f));
                if (inU > 0.01f)
                    drawFlowLabel(q.lin->head() + off, g.psi, g.psiChars, g.theta0, num, true,
                                  headRadius, inU * std::max(q.linFade, 0.0f));
            }
        }
        if (!cfg.showHgLinks || inU < 0.01f)
            return;
        const float S = 0.8f * cfg.stageLabelSize;
        if (hInvLabelSize != S)
        {
            hInvLabelSize = S;
            hInvLabel.set(R"(\tilde h^{-1})", S, glm::vec3(1.0f));   // as in V's label
        }
        for (const HgPair& q : hgPairList)
        {
            if (!q.alive())
                continue;
            float sx, sy;
            if (!toScreen(0.5f * (q.lin->head() + off + q.tru->head()), sx, sy))
                continue;
            const float w = (float)hInvLabel.width(), h = (float)hInvLabel.height();
            hInvLabel.drawAt(sx - 0.5f * w, sy - 0.5f * h, inU);
        }
    }

    // ---- "Transform flow": the linear picture carried onto the true one --------------
    //
    // Seeds on the sphere kMorphSeedFraction * rho in U flow under psi^t (orange) until
    // the first one hits U's border (morphFlowTime of ODE time at most), pause, then
    // every trail point and particle travels to V at once — the vertex shader
    // (morph_vertex.glsl) sends theta to x* + shift (1 - s) + mix(theta, h~^{-1}(theta), s)
    // with s eased 0 -> 1 over morphSeconds — turning from orange to green, a ghost of U's
    // glass deforming into V alongside. They stay in V until the next transform, a
    // reseed or Hide.

    void startMorph()
    {
        hgPairList.clear();   // the stage is theirs
        morphTrajs.clear();
        const int n = std::clamp(cfg.hgPairs, 1, kMaxHgPairs);
        const double R = kMorphSeedFraction * cfg.hgRadius;
        for (int k = 0; k < n; ++k)
        {
            double y = 1.0 - 2.0 * (k + 0.5) / n, rad = std::sqrt(std::max(0.0, 1.0 - y * y));
            double ph = 2.39996323 * k;
            double seed[3] = {xs[0] + R * std::cos(ph) * rad, xs[1] + R * y, xs[2] + R * std::sin(ph) * rad};
            morphTrajs.push_back(makeStageTrajectory(seed));
        }
        morphPhase = MorphPhase::Flowing;
        morphOde = 0.0;
        morphClock = 0.0f;
        morphS = 0.0f;
    }

    void stopMorph()
    {
        morphTrajs.clear();
        morphPhase = MorphPhase::Off;
    }

    void updateMorph(float dt)
    {
        switch (morphPhase)
        {
        case MorphPhase::Flowing:
        {
            float far = 0.0f;
            for (const auto& t : morphTrajs)
                far = std::max(far, glm::length(t->head() - xStar));
            // the first particle to reach U's border starts the transform (max flow
            // time is only a fallback for a flow too slow to get there)
            if (far >= cfg.hgRadius || morphOde >= cfg.morphFlowTime)
            {
                morphPhase = MorphPhase::Paused;
                morphClock = 0.0f;
            }
            break;
        }
        case MorphPhase::Paused:
            morphClock += dt;
            if (morphClock >= kMorphPauseSeconds)
            {
                morphPhase = MorphPhase::Morphing;
                morphClock = 0.0f;
            }
            break;
        case MorphPhase::Morphing:
            morphClock += dt;
            morphS = smootherstep(morphClock / cfg.morphSeconds);
            if (morphClock >= cfg.morphSeconds)
            {
                morphS = 1.0f;
                morphPhase = MorphPhase::Done;
            }
            break;
        default:
            break;
        }
    }

    // Before the transform: in U with the usual shader (shifted, clipped to U). From it
    // on: every vertex through morph_vertex.glsl, in parallel on the GPU.
    void renderMorph(const glm::mat4& view, const glm::mat4& projection, float px)
    {
        if (morphPhase == MorphPhase::Off || morphTrajs.empty() || !hmOk)
            return;
        const float k = cfg.singleScale;
        const float a = hgFade * std::max(hgSide, morphPhase == MorphPhase::Done ? hgFocus : 0.0f);
        if (a < 0.01f)
            return;
        const glm::vec3 off = hgOffset();
        if (morphPhase == MorphPhase::Flowing || morphPhase == MorphPhase::Paused)
        {
            shader.setVec3("u_offset", off);
            shader.setVec3("u_clipCenter", xStar + off);
            shader.setFloat("u_clipRadius", cfg.hgRadius);
            shader.setInt("u_useRamp", 0);
            shader.setVec3("u_color", kPsiColor);
            for (const auto& t : morphTrajs)
            {
                glLineWidth(cfg.trailWidth * k * px);
                shader.setFloat("u_alpha", cfg.trailAlpha * a);
                t->draw();
                shader.setFloat("u_alpha", a);
                shader.setFloat("u_pointSize", cfg.particleSize * k * px);
                shader.setInt("u_round", 1);
                t->drawHead();
                shader.setInt("u_round", 0);
            }
            shader.setVec3("u_offset", glm::vec3(0.0f));
            shader.setFloat("u_clipRadius", 0.0f);
            shader.setFloat("u_alpha", 1.0f);
            shader.setFloat("u_pointSize", cfg.particleSize * px);
            return;
        }
        hm.bind();
        morphShader.use();
        morphShader.setMat4("view", view);
        morphShader.setMat4("projection", projection);
        morphShader.setVec3("u_center", xStar);
        morphShader.setVec3("u_shift", off * (1.0f - morphS));
        morphShader.setFloat("u_morph", morphS);
        morphShader.setVec3("u_color", glm::mix(kPsiColor, kFlatColor, morphS));
        morphShader.setInt("u_useRamp", 0);
        morphShader.setFloat("u_alphaTail", 1.0f);
        morphShader.setFloat("u_clipRadius", 0.0f);
        for (const auto& t : morphTrajs)
        {
            glLineWidth(cfg.trailWidth * k * px);
            morphShader.setFloat("u_alpha", cfg.trailAlpha * a);
            morphShader.setInt("u_round", 0);
            t->draw();
            morphShader.setFloat("u_alpha", a);
            morphShader.setFloat("u_pointSize", cfg.particleSize * k * px);
            morphShader.setInt("u_round", 1);
            t->drawHead();
        }
        shader.use();   // the rest of the pass carries on with the usual program
    }

    // U's glass travelling with the transform and deforming into V.
    void renderMorphGhost(const engine::Camera& camera, const glm::mat4& view,
                          const glm::mat4& projection)
    {
        if (morphPhase != MorphPhase::Morphing || !hmOk)
            return;
        glDepthMask(GL_FALSE);
        hm.bind();
        hmShader.use();
        hmShader.setMat4("view", view);
        hmShader.setMat4("projection", projection);
        hmShader.setVec3("u_eye", camera.position);
        hmShader.setVec3("u_center", xStar);
        hmShader.setFloat("u_radius", cfg.hgRadius);
        hmShader.setInt("u_morphOn", 1);
        hmShader.setFloat("u_morph", morphS);
        hmShader.setVec3("u_shift", hgOffset() * (1.0f - morphS));
        hmShader.setVec3("u_color", glm::mix(kUColor, kVColor, morphS));
        hmShader.setFloat("u_alpha", 0.8f * std::sin(3.1415926f * morphS) * hgFade);
        hmShader.setInt("u_waveCount", 0);
        glassMesh->draw(GL_TRIANGLES);
        hmShader.setInt("u_morphOn", 0);
        glDepthMask(GL_TRUE);
    }

    // ---- the crash: a shockwave over the glass where a pair hits its ball's rim -------

    // ball 0 = V, 1 = U; `sphereDir` is the impact on the unit sphere the ball is made
    // from (for V: theta / |theta|), `at` the impact in world space
    void spawnBurst(int ball, const glm::vec3& sphereDir, const glm::vec3& at, const glm::vec3& color)
    {
        Burst b;
        b.ball = ball;
        b.pos = at;
        b.dir = glm::length(sphereDir) > 1e-6f ? glm::normalize(sphereDir) : glm::vec3(0.0f, 0.0f, 1.0f);
        b.color = color;
        bursts.push_back(b);
    }

    void updateBursts(float dt)
    {
        for (Burst& b : bursts)
            b.age += dt;
        std::erase_if(bursts, [&](const Burst& b) { return b.age >= cfg.waveSeconds; });
    }

    // The particle breaking up where it crashed: a spray of small fragments in its color,
    // flung out in random directions, slowing down and fading with the shockwave.
    void spawnShards(const glm::vec3& at, const glm::vec3& color)
    {
        if ((int)shardGroups.size() >= kMaxShardGroups)
            shardGroups.erase(shardGroups.begin());
        ShardGroup g;
        g.color = color;
        std::normal_distribution<float> gauss(0.0f, 1.0f);
        std::uniform_real_distribution<float> speed(0.12f, 0.45f);
        for (int i = 0; i < kShardsPerCrash; ++i)
        {
            glm::vec3 d(gauss(rng), gauss(rng), gauss(rng));
            d = glm::length(d) > 1e-4f ? glm::normalize(d) : glm::vec3(0.0f, 1.0f, 0.0f);
            g.shards.push_back({at, d * speed(rng) * cfg.hgRadius});
        }
        shardGroups.push_back(std::move(g));
    }

    void updateShards(float dt)
    {
        for (ShardGroup& g : shardGroups)
        {
            g.age += dt;
            const float drag = std::exp(-2.5f * dt);
            for (Shard& sh : g.shards)
            {
                sh.pos += sh.vel * dt;
                sh.vel *= drag;
            }
        }
        std::erase_if(shardGroups, [&](const ShardGroup& g) { return g.age >= cfg.waveSeconds; });
        std::vector<float> v;
        for (const ShardGroup& g : shardGroups)
            for (const Shard& sh : g.shards)
                v.insert(v.end(), {sh.pos.x, sh.pos.y, sh.pos.z});
        if (!v.empty())
            shardMesh->setVertices(v.data(), v.size());
    }

    void renderShards(float px)
    {
        if (shardGroups.empty())
            return;
        shader.setVec3("u_offset", glm::vec3(0.0f));
        shader.setFloat("u_clipRadius", 0.0f);
        shader.setInt("u_useRamp", 0);
        shader.setInt("u_round", 1);
        int first = 0;
        for (const ShardGroup& g : shardGroups)
        {
            const float e = g.age / cfg.waveSeconds;
            shader.setVec3("u_color", glm::mix(glm::vec3(1.0f), g.color, std::min(1.0f, 3.0f * e)));
            shader.setFloat("u_alpha", std::pow(1.0f - e, 1.5f) * hgFade);
            shader.setFloat("u_pointSize", cfg.particleSize * 0.55f * (1.0f - 0.5f * e) * px);
            shardMesh->drawArrays(GL_POINTS, (int)g.shards.size(), first);
            first += (int)g.shards.size();
        }
        shader.setInt("u_round", 0);
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_pointSize", cfg.particleSize * px);
    }

    // Hand a glass shader this ball's newest waves (up to kMaxWavesPerBall) and, if any,
    // the scene copy and the settings — glass_fragment.glsl does the rest.
    void setWaves(const engine::Shader& sh, int ball, const glm::mat4& view,
                  const glm::mat4& projection)
    {
        int n = 0;
        for (auto it = bursts.rbegin(); it != bursts.rend() && n < kMaxWavesPerBall; ++it)
        {
            if (it->ball != ball)
                continue;
            const std::string i = "[" + std::to_string(n) + "]";
            sh.setVec3("u_waveDir" + i, it->dir);
            sh.setVec3("u_wavePos" + i, it->pos);
            sh.setFloat("u_waveT" + i, it->age / cfg.waveSeconds);
            sh.setVec3("u_waveColor" + i, it->color);
            ++n;
        }
        sh.setInt("u_waveCount", n);
        if (n == 0)
            return;
        sh.setFloat("u_waveReach", glm::radians(cfg.waveReach));
        sh.setFloat("u_waveForce", cfg.waveForce);
        sh.setFloat("u_waveThickness", cfg.waveThickness);
        sh.setFloat("u_waveFeather", cfg.waveFeather);
        sh.setFloat("u_waveAberration", cfg.waveAberration);
        sh.setFloat("u_waveFlash", cfg.waveIntensity);
        sceneCopy.bind(1);
        sh.setInt("u_scene", 1);
        sh.setVec4("u_viewport", (float)sceneCopy.viewportX(), (float)sceneCopy.viewportY(),
                   (float)sceneCopy.width(), (float)sceneCopy.height());
        sh.setMat4("u_viewProj", projection * view);
        sh.setFloat("u_proj11", projection[1][1]);
    }

    // Under Other settings: how the crash shockwave looks.
    void drawShockwaveUI()
    {
        sectionHeader("Crash shockwave");
        ImGui::SliderFloat("intensity", &cfg.waveIntensity, 0.0f, 8.0f, "%.2f");
        ImGui::SliderFloat("propagation", &cfg.waveReach, 0.5f, 120.0f, "%.1f deg",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SliderFloat("distortion", &cfg.waveForce, 0.0f, 0.4f, "%.3f");
        ImGui::SliderFloat("ring thickness", &cfg.waveThickness, 0.01f, 0.5f, "%.3f");
        ImGui::SliderFloat("feathering", &cfg.waveFeather, 0.01f, 0.5f, "%.3f");
        ImGui::SliderFloat("color split", &cfg.waveAberration, 0.0f, 0.1f, "%.3f");
        ImGui::SliderFloat("duration", &cfg.waveSeconds, 0.2f, 4.0f, "%.2f s");
    }

    // "U" over the linear ball, "V = h^{-1}(U)" over the deformed one.
    void drawHgLabels()
    {
        if (!hgActive)
            return;
        const float S = cfg.stageLabelSize;
        if (hgLabelSize != S)
        {
            hgLabelSize = S;
            hgLabelU.set("U", S, kUColor);
            hgLabelV.set(R"(V = \tilde h^{-1}(U))", S, kVColor);
            // the system each ball holds, between its name and the glass
            hgSystemU.set(R"(\dot\theta = Df(x^*)\,\theta)", 0.85f * S, kUColor);
            hgSystemV.set(R"(\dot x = f(x))", 0.85f * S, kVColor);
        }
        const engine::Camera& cam = app->camera();
        glm::vec3 up = glm::normalize(glm::cross(glm::cross(cam.front, cam.up), cam.front));
        const float lift = 1.12f * cfg.hgRadius;
        // stacked upward from just above the ball: the system, a gap, the name — in front
        // of the panels, so the HUD never hides them
        auto stack = [&](const glm::vec3& anchor, const engine::LatexImage& name,
                         const engine::LatexImage& system, float alpha) {
            float sx, sy;
            if (alpha < 0.01f || !toScreen(anchor, sx, sy))
                return;
            const float sysTop = sy - (float)system.height();
            system.drawAt(sx - 0.5f * system.width(), sysTop, alpha, true);
            name.drawAt(sx - 0.5f * name.width(), sysTop - 0.35f * S - (float)name.height(), alpha, true);
        };
        stack(xStar + lift * up, hgLabelV, hgSystemV, hgFocus * hgFade);
        stack(xStar + hgOffset() + lift * up, hgLabelU, hgSystemU, hgSide * hgFade);
    }

    // theta = h~(u): solve h~^{-1}(theta) = u by Newton from the given theta (last
    // frame's, so one or two steps suffice), Jacobian by forward differences.
    void hgForward(const engine::ConjugacyMap::Vec3& u, double* theta) const
    {
        for (int it = 0; it < 4; ++it)
        {
            engine::ConjugacyMap::Vec3 t = {theta[0], theta[1], theta[2]};
            auto H = hm.hInv(t);
            double r[3] = {H[0] - u[0], H[1] - u[1], H[2] - u[2]};
            double scale = std::max(1.0, std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]));
            if (std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) < 1e-10 * scale)
                return;
            double J[3][3];
            const double e = 1e-7 * scale;
            for (int c = 0; c < 3; ++c)
            {
                auto tc = t;
                tc[c] += e;
                auto Hc = hm.hInv(tc);
                for (int k = 0; k < 3; ++k)
                    J[k][c] = (Hc[k] - H[k]) / e;
            }
            double det = J[0][0] * (J[1][1] * J[2][2] - J[1][2] * J[2][1])
                       - J[0][1] * (J[1][0] * J[2][2] - J[1][2] * J[2][0])
                       + J[0][2] * (J[1][0] * J[2][1] - J[1][1] * J[2][0]);
            if (std::abs(det) < 1e-12)
                return;
            // delta = J^{-1} r by Cramer
            for (int c = 0; c < 3; ++c)
            {
                double M[3][3];
                for (int a = 0; a < 3; ++a)
                    for (int b = 0; b < 3; ++b)
                        M[a][b] = b == c ? r[a] : J[a][b];
                double dc = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                          - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                          + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
                theta[c] -= dc / det;
            }
        }
    }


    // Seed pairs on the sphere kHgSeedFraction * rho in the linear ball: psi^t from
    // x* + theta, phi^t from x* + h~^{-1}(theta). On E^s and E^u they stay paired
    // exactly (the parametrization method); in between h~^{-1} is an interpolation.
    void seedHgPairs()
    {
        hgPairList.clear();
        if (!hmOk)
            return;
        const int n = std::clamp(cfg.hgPairs, 1, kMaxHgPairs);
        const double R = kHgSeedFraction * cfg.hgRadius;
        for (int k = 0; k < n; ++k)
        {
            double y = 1.0 - 2.0 * (k + 0.5) / n, rad = std::sqrt(std::max(0.0, 1.0 - y * y));
            double phi = 2.39996323 * k;
            engine::ConjugacyMap::Vec3 theta = {R * std::cos(phi) * rad, R * y, R * std::sin(phi) * rad};
            auto H = hm.hInv(theta);
            HgPair q;
            // (theta is copied into q.theta0 / q.theta below)
            for (int c = 0; c < 3; ++c)
            {
                q.linSeed[c] = xs[c] + theta[c];
                q.truSeed[c] = xs[c] + H[c];
            }
            q.lin = makeStageTrajectory(q.linSeed);
            q.tru = makeStageTrajectory(q.truSeed);
            q.t0 = fieldTime;
            for (int c = 0; c < 3; ++c)
                q.theta0[c] = q.theta[c] = theta[c];
            hgPairList.push_back(std::move(q));
        }
    }

    // A pair dissolves when psi^t leaves the linear ball U — by the conjugacy phi^t
    // leaves V at that same instant — fading with the shockwave, then it is dropped.
    // Then the links (psi^t head -> phi^t head) and the predicted points
    // x* + h~^{-1}(psi^t - x*) for the pairs still inside.
    void updateHgPairs(float dt)
    {
        if (hgPairList.empty())
            return;
        const double rho = cfg.hgRadius;
        const glm::vec3 off = hgOffset();
        for (HgPair& q : hgPairList)
        {
            if (q.alive())
            {
                // psi^t hits U's border: the round sphere |psi^t - x*| = rho
                const double* l = q.lin->state();
                const double* x = q.tru->state();
                bool linHit = glm::length(q.lin->head() - xStar) > (float)rho;
                // phi^t hits V's border: the deformed surface x* + h~^{-1}(rho S^2), i.e.
                // |h~(phi^t - x*)| = rho, h~ by Newton from last frame's value
                double thetaPrev[3] = {q.theta[0], q.theta[1], q.theta[2]};
                hgForward({x[0] - xs[0], x[1] - xs[1], x[2] - xs[2]}, q.theta);
                bool truHit = norm3(q.theta) >= rho;
                if (linHit || truHit)
                {
                    // the one that hit explodes exactly on its border (the crossing
                    // between last frame and this one, by bisection), with a shockwave on
                    // its glass; its partner explodes in the same instant where it is
                    q.linCrashed = q.truCrashed = true;
                    if (linHit)
                    {
                        double c[3];
                        crossing(q.linPrev, l, c, [&](const double* y) {
                            double u[3] = {y[0] - xs[0], y[1] - xs[1], y[2] - xs[2]};
                            return norm3(u) - rho;
                        });
                        q.linAt = glm::vec3((float)c[0], (float)c[1], (float)c[2]);
                        spawnBurst(1, q.linAt - xStar, q.linAt + off, kPsiColor);
                    }
                    else
                        q.linAt = q.lin->head();
                    if (truHit)
                    {
                        double c[3], th[3] = {thetaPrev[0], thetaPrev[1], thetaPrev[2]};
                        crossing(q.truPrev, x, c, [&](const double* y) {
                            hgForward({y[0] - xs[0], y[1] - xs[1], y[2] - xs[2]}, th);
                            return norm3(th) - rho;
                        });
                        q.truAt = glm::vec3((float)c[0], (float)c[1], (float)c[2]);
                        spawnBurst(0, glm::vec3((float)th[0], (float)th[1], (float)th[2]), q.truAt,
                                   kFlatColor);
                    }
                    else
                        q.truAt = q.tru->head();
                    spawnShards(q.linAt + off, kPsiColor);
                    spawnShards(q.truAt, kFlatColor);
                }
            }
            // the head's dissolve, the shards and the shockwave share one clock
            if (q.linCrashed)
                q.linFade -= dt / cfg.waveSeconds;
            if (q.truCrashed)
                q.truFade -= dt / cfg.waveSeconds;
        }
        std::erase_if(hgPairList, [](const HgPair& q) { return q.linFade <= 0.0f && q.truFade <= 0.0f; });

        std::vector<float> v;
        hgLinkCount = 0;
        for (const HgPair& q : hgPairList)
            if (q.alive())
            {
                // psi^t lives in U, drawn shifted beside V: the link spans the gap
                const glm::vec3 a = q.lin->head() + off, b = q.tru->head();
                v.insert(v.end(), {a.x, a.y, a.z, 0.0f, b.x, b.y, b.z, 1.0f});
                ++hgLinkCount;
            }
        hgPredictedCount = 0;
        for (const HgPair& q : hgPairList)
            if (q.alive())
            {
                const double* l = q.lin->state();
                auto H = hm.hInv({l[0] - xs[0], l[1] - xs[1], l[2] - xs[2]});
                v.insert(v.end(), {(float)(xs[0] + H[0]), (float)(xs[1] + H[1]), (float)(xs[2] + H[2]), 0.0f});
                ++hgPredictedCount;
            }
        hgMesh->setVertices(v.data(), v.size());

        std::vector<float> heads;
        int slot = 0;
        for (HgPair& q : hgPairList)
        {
            q.linHead = q.truHead = -1;
            if (!q.linCrashed)
                continue;
            heads.insert(heads.end(), {q.linAt.x, q.linAt.y, q.linAt.z, q.truAt.x, q.truAt.y, q.truAt.z});
            q.linHead = slot++;
            q.truHead = slot++;
        }
        if (!heads.empty())
            headMesh->setVertices(heads.data(), heads.size());
    }

    static double norm3(const double* v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

    // The point on the segment a -> b where `side` (negative inside, positive outside)
    // changes sign, by bisection: a crash placed on the border, not one frame past it.
    template <class F>
    static void crossing(const double* a, const double* b, double* out, F side)
    {
        double lo = 0.0, hi = 1.0;
        for (int it = 0; it < 20; ++it)
        {
            double mid = 0.5 * (lo + hi), y[3];
            for (int c = 0; c < 3; ++c)
                y[c] = a[c] + mid * (b[c] - a[c]);
            (side(y) > 0.0 ? hi : lo) = mid;
        }
        for (int c = 0; c < 3; ++c)
            out[c] = a[c] + hi * (b[c] - a[c]);
    }

    void renderHgPairs(float px)
    {
        if (!hgActive)
            return;
        const float inV = hgFocus * hgFade, inU = hgSide * hgFade;
        const glm::vec3 off = hgOffset();

        // the two arrow fields: f in V (unclipped: it was sampled inside), A theta in U
        if (cfg.showArrows && !hgArrowsV.empty())
        {
            glDepthMask(GL_FALSE);
            shader.setFloat("u_magScale", 1.0f / hgMaxMag);
            shader.setInt("u_useRamp", 1);
            shader.setFloat("u_alphaTail", cfg.arrowTailAlpha);
            glLineWidth(1.5f * px);
            for (int side = 0; side < 2; ++side)
            {
                float a = side == 0 ? inV : inU;
                if (a < 0.01f)
                    continue;
                const engine::ArrowGlyphs& g = side == 0 ? hgArrowsV : hgArrowsU;
                shader.setVec3("u_offset", side == 0 ? glm::vec3(0.0f) : off);
                shader.setVec3("u_clipCenter", xStar + (side == 0 ? glm::vec3(0.0f) : off));
                shader.setFloat("u_clipRadius", side == 0 ? 0.0f : cfg.hgRadius);
                shader.setFloat("u_alpha", cfg.arrowAlpha * a);
                shader.setInt("u_useFade", 1);
                g.drawShafts(shader);
                shader.setInt("u_useFade", 0);
                shader.setInt("u_shadeMode", 1);
                g.drawHeads(shader);
                shader.setInt("u_shadeMode", 0);
            }
            shader.setInt("u_useRamp", 0);
            shader.setFloat("u_alphaTail", 1.0f);
            shader.setFloat("u_magScale", 1.0f / std::max(sampledMaxMag, 1e-6f));
            shader.setVec3("u_offset", glm::vec3(0.0f));
            glDepthMask(GL_TRUE);
        }
        if (hgPairList.empty())
        {
            shader.setFloat("u_clipRadius", 0.0f);
            return;
        }
        const float k = cfg.singleScale;
        float clipNow = 0.0f;   // the clip radius in force, restored after a frozen head
        // flare: 1 the instant a pair hits the rim, to 0 as it dissolves — the head
        // flashes white for a moment, then shrinks away into its shards while the trail
        // fades
        auto curve = [&](const engine::Trajectory& t, const glm::vec3& color, float fade,
                         float flare, int frozenHead) {
            const float flash = smoothstep(0.7f, 1.0f, flare);
            const float headScale = flare > 0.0f ? flare * (1.0f + 1.5f * flash) : 1.0f;
            if (cfg.showTrails)
            {
                glLineWidth(cfg.trailWidth * k * px);
                shader.setVec3("u_color", glm::mix(color, glm::vec3(1.0f), 0.5f * flash));
                shader.setFloat("u_alpha", cfg.trailAlpha * fade);
                t.draw();
            }
            shader.setVec3("u_color", t.diverged() ? kDivergedColor
                                                   : glm::mix(color, glm::vec3(1.0f), 0.8f * flash));
            shader.setFloat("u_alpha", fade);
            shader.setFloat("u_pointSize", cfg.particleSize * k * headScale * px);
            shader.setInt("u_round", 1);
            if (frozenHead >= 0)
            {
                // where it exploded, exactly on the border: no rim clipping for it
                shader.setFloat("u_clipRadius", 0.0f);
                headMesh->drawArrays(GL_POINTS, 1, frozenHead);
                shader.setFloat("u_clipRadius", clipNow);
            }
            else
                t.drawHead();
            shader.setInt("u_round", 0);
        };
        // phi^t in V; psi^t in U (shifted, clipped to U's sphere)
        shader.setFloat("u_clipRadius", 0.0f);
        for (const HgPair& q : hgPairList)
            curve(*q.tru, kFlatColor, std::max(q.truFade, 0.0f) * inV,
                  q.truCrashed ? std::max(q.truFade, 0.0f) : 0.0f, q.truHead);
        shader.setVec3("u_offset", off);
        shader.setVec3("u_clipCenter", xStar + off);
        shader.setFloat("u_clipRadius", cfg.hgRadius);
        clipNow = cfg.hgRadius;
        for (const HgPair& q : hgPairList)
            curve(*q.lin, kPsiColor, std::max(q.linFade, 0.0f) * inU,
                  q.linCrashed ? std::max(q.linFade, 0.0f) : 0.0f, q.linHead);
        shader.setVec3("u_offset", glm::vec3(0.0f));
        shader.setFloat("u_clipRadius", 0.0f);
        if (cfg.showHgLinks && hgLinkCount > 0 && inU > 0.01f)
        {
            // orange at psi^t's end, green at phi^t's
            glLineWidth(1.5f * px);
            shader.setInt("u_useRamp", 2);
            shader.setFloat("u_magScale", 1.0f);
            shader.setVec3("u_color", kPsiColor);
            shader.setVec3("u_color2", kFlatColor);
            shader.setFloat("u_alpha", 0.7f * inU);
            hgMesh->drawArrays(GL_LINES, 2 * hgLinkCount, 0);
            shader.setInt("u_useRamp", 0);
            shader.setFloat("u_magScale", 1.0f / std::max(sampledMaxMag, 1e-6f));
        }
        if (cfg.showHgPredicted && hgPredictedCount > 0)
        {
            glDisable(GL_DEPTH_TEST);
            shader.setVec3("u_color", kFormulaColor);
            shader.setFloat("u_alpha", inV);
            shader.setFloat("u_pointSize", 0.55f * cfg.particleSize * k * px);
            shader.setInt("u_round", 1);
            hgMesh->drawArrays(GL_POINTS, hgPredictedCount, 2 * hgLinkCount);
            shader.setInt("u_round", 0);
            glEnable(GL_DEPTH_TEST);
        }
        shader.setFloat("u_pointSize", cfg.particleSize * px);
    }

    // V = x* + h~^{-1}(U) (deformed on the GPU, green glass — red once the check says
    // the radius is too big) and, beside it, U (round, orange glass).
    void renderHgBalls(const engine::Camera& camera, const glm::mat4& view,
                       const glm::mat4& projection)
    {
        if (!hgActive || !hmOk)
            return;
        if (!bursts.empty())
            sceneCopy.capture();   // what the shockwaves bend
        glDepthMask(GL_FALSE);
        if (hgSide > 0.01f)
        {
            glassShader.use();
            glassShader.setMat4("view", view);
            glassShader.setMat4("projection", projection);
            glassShader.setVec3("u_eye", camera.position);
            glassShader.setVec3("u_center", xStar + hgOffset());
            glassShader.setFloat("u_radius", cfg.hgRadius);
            glassShader.setVec3("u_color", kUColor);
            glassShader.setFloat("u_alpha", 0.9f * hgSide * hgFade);
            setWaves(glassShader, 1, view, projection);
            glassMesh->draw(GL_TRIANGLES);
            glassShader.setInt("u_waveCount", 0);
        }
        if (hgFocus > 0.01f)
        {
            hm.bind();
            hmShader.use();
            hmShader.setMat4("view", view);
            hmShader.setMat4("projection", projection);
            hmShader.setVec3("u_eye", camera.position);
            hmShader.setVec3("u_center", xStar);
            hmShader.setFloat("u_radius", cfg.hgRadius);
            hmShader.setVec3("u_color", hgCheck.level == 2 ? kBadColor : kVColor);
            hmShader.setFloat("u_alpha", 1.1f * hgFocus * hgFade);
            setWaves(hmShader, 0, view, projection);
            glassMesh->draw(GL_TRIANGLES);
        }
        glDepthMask(GL_TRUE);
    }

    void drawHartmanGrobmanUI()
    {
        sectionHeader("Hartman-Grobman");
        if (!hmOk)
        {
            ImGui::TextColored(ImVec4(kBadColor.r, kBadColor.g, kBadColor.b, 1.0f),
                               "no conjugacy here: %s", hmError.c_str());
            return;
        }
        // the slider's top is the safe radius of this equilibrium, recomputed whenever x*
        // or the parameters change — once the drag ends (it is a few hundred ms in Debug)
        if (hgCheckDirty && !ImGui::IsAnyItemActive())
            runHgCheck();
        const float maxRho = std::max((float)hgCheck.safe, kMinHgRadius);
        ImGui::SliderFloat("linear ball radius", &cfg.hgRadius, kMinHgRadius, maxRho, "%.2f");
        cfg.hgRadius = std::clamp(cfg.hgRadius, kMinHgRadius, maxRho);
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            hgCheckDirty = true;
            hgArrowsDirty = true;
            stopMorph();
            if (hgActive)
                seedHgPairs();
        }

        // the check: is this neighborhood small enough for h?
        const glm::vec3 c = hgCheck.level == 0 ? kVColor : hgCheck.level == 1 ? kWarnColor : kBadColor;
        const ImVec4 col(c.r, c.g, c.b, 1.0f);
        if (hgCheck.folds)
            ImGui::TextColored(col, "too big: h^-1 folds over (det %.2f), not one-to-one", hgCheck.minDet);
        else if (hgCheck.level == 2)
            ImGui::TextColored(col, "too big: K_s, K_u off by %.0e here", hgCheck.defect);
        else if (hgCheck.level == 1)
            ImGui::TextColored(col, "approximate: K_s, K_u off by %.0e here", hgCheck.defect);
        else
            ImGui::TextColored(col, "ok: K_s, K_u exact to %.0e, h^-1 one-to-one", hgCheck.defect);
        ImGui::TextDisabled(hgCheck.safeIsMax ? "max radius: %.2f (works beyond)" : "max radius: %.2f",
                            maxRho);

        if (ImGui::Button(hgTarget ? "Hide conjugacy" : "Show conjugacy"))
            toggleConjugacy();
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        ImGui::SliderFloat("distance", &cfg.hgGap, 0.0f, 4.0f, "%.2f rho");
        if (!hgActive)
            return;
        ImGui::SameLine();
        if (ImGui::Button("Reseed"))
        {
            stopMorph();
            seedHgPairs();
        }
        ImGui::SameLine();
        if (ImGui::Button("Transform flow"))
            startMorph();
        if (morphPhase != MorphPhase::Off)
        {
            ImGui::SameLine();
            const char* phase = morphPhase == MorphPhase::Flowing ? "flowing (linear)"
                              : morphPhase == MorphPhase::Paused  ? "paused"
                              : morphPhase == MorphPhase::Morphing ? "transforming"
                                                                   : "in V";
            ImGui::TextDisabled("%s", phase);
        }
        ImGui::SetNextItemWidth(140.0f);
        ImGui::SliderFloat("transform time", &cfg.morphSeconds, 0.5f, 10.0f, "%.1f s");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(140.0f);
        ImGui::SliderFloat("max flow time", &cfg.morphFlowTime, 0.01f, 20.0f, "%.2f",
                           ImGuiSliderFlags_Logarithmic);
        ImGui::SetNextItemWidth(160.0f);
        // takes effect on the next Reseed / Transform flow, not by itself
        ImGui::SliderInt("pairs", &cfg.hgPairs, 1, kMaxHgPairs);
        auto colored = [](const char* label, bool* v, const glm::vec3& col3) {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(col3.r, col3.g, col3.b, 1.0f));
            ImGui::Checkbox(label, v);
            ImGui::PopStyleColor();
        };
        ImGui::Checkbox("labels##hg", &cfg.showHgLabels);
        ImGui::SameLine();
        colored("links (h^-1)", &cfg.showHgLinks, kLinkColor);
        ImGui::SameLine();
        colored("h^-1(psi^t)", &cfg.showHgPredicted, kFormulaColor);
    }

    std::unique_ptr<engine::Trajectory> makeStageTrajectory(const double* seed) const
    {
        auto t = std::make_unique<engine::Trajectory>(3, seed, fieldTime, cfg.trailPoints);
        t->minSpacing = 0.002f * cfg.ballRadius;
        return t;
    }

    // A ring of seeds theta in E^s around x* (the two points +-theta when E^s is a line),
    // each starting three curves at the same field time: psi^t from x* + theta under the
    // linear field, phi^t from that same point under the true field, and phi^t from
    // x* + K_s(theta) — the point of W^s the conjugacy pairs with theta.
    void seedRing()
    {
        pairs.clear();
        if (!manifoldOk)
            return;
        const int k = Ws.dim();
        const int n = k == 1 ? 2 : std::clamp(cfg.ringCount, 3, kMaxPairs);
        const double R = cfg.ringRadius * cfg.ballRadius;
        for (int i = 0; i < n; ++i)
        {
            double a = 6.283185307179586 * i / n;
            engine::InvariantManifold::Vec3 theta;
            for (int c = 0; c < 3; ++c)
                theta[c] = k == 2 ? R * (std::cos(a) * Ws.orthoBasis(0)[c] + std::sin(a) * Ws.orthoBasis(1)[c])
                                  : (i == 0 ? R : -R) * Ws.orthoBasis(0)[c];
            StagePair q;
            q.s = Ws.toParam(theta);
            engine::InvariantManifold::Vec3 K = Ws.eval(q.s);
            for (int c = 0; c < 3; ++c)
            {
                q.flatSeed[c] = xs[c] + theta[c];
                q.onSeed[c] = xs[c] + K[c];
            }
            q.t0 = fieldTime;
            q.lin = makeStageTrajectory(q.flatSeed);
            q.flat = makeStageTrajectory(q.flatSeed);
            q.onW = makeStageTrajectory(q.onSeed);
            pairs.push_back(std::move(q));
        }
        rebuildStageMesh();
    }

    // Links psi^t <-> phi^t from the same seed of E^s (same t: how far the true flow has
    // left the linear one), then the formula points x* + K(e^{Lambda t} s) — where the
    // blue heads (phi^t on W^s) must sit if K_s is right.
    void rebuildStageMesh()
    {
        std::vector<float> v;
        v.reserve(pairs.size() * 9);
        for (const StagePair& q : pairs)
            for (const glm::vec3& h : {q.lin->head(), q.flat->head()})
                v.insert(v.end(), {h.x, h.y, h.z});
        for (const StagePair& q : pairs)
        {
            engine::InvariantManifold::Vec3 K = Ws.eval(Ws.flowParam(q.s, q.onW->time() - q.t0));
            v.insert(v.end(), {(float)(xs[0] + K[0]), (float)(xs[1] + K[1]), (float)(xs[2] + K[2])});
        }
        stageMesh->setVertices(v.data(), v.size());
    }

    // phi^t(x_0) on every green head (the true flow) and psi^t(theta_0) on every orange
    // one (the linear flow), each pair from the same seed of E^s — the pair the link
    // joins. t is the ODE time since seeding, composed per frame from pieces typeset once
    // (the lorenz_flow / vector_field pattern), phi above its head, psi below.
    void drawStageLabels()
    {
        if (!cfg.showStageLabels || pairs.empty())
            return;
        drawStageLabelsPrepare();
        StageLabelGlyphs& g = stageGlyphs;
        const float headRadius = 0.5f * cfg.particleSize * cfg.singleScale;
        for (const StagePair& q : pairs)
        {
            double t = q.flat->time() - q.t0;
            if (std::abs(t) < 0.005)
                t = 0.0;   // no "-0.00"
            char num[32];
            std::snprintf(num, sizeof(num), "%.2f", t);
            if (cfg.showFlat)
                drawFlowLabel(q.flat->head(), g.phi, g.phiChars, g.x0, num, true, headRadius,
                              contentFade);
            if (cfg.showPsi)
                drawFlowLabel(q.lin->head(), g.psi, g.psiChars, g.theta0, num, false, headRadius,
                              contentFade);
        }
    }

    // The phi / psi / (x_0) / (theta_0) / digit pieces, typeset once per size.
    void drawStageLabelsPrepare()
    {
        StageLabelGlyphs& g = stageGlyphs;
        const float S = cfg.stageLabelSize;
        if (g.size != S)
        {
            g.size = S;
            g.phi.set(R"(\phi)", S, kFlatColor);
            g.x0.set(R"((x_0))", S, kFlatColor);
            g.psi.set(R"(\psi)", S, kPsiColor);
            g.theta0.set(R"((\theta_0))", S, kPsiColor);
            for (int i = 0; i < 12; ++i)
            {
                char ch = kTimeLabelChars[i];
                std::string piece = ch == '-' ? std::string("{-}") : std::string(1, ch);
                g.phiChars[i].set(piece, 0.7f * S, kFlatColor);
                g.psiChars[i].set(piece, 0.7f * S, kPsiColor);
            }
        }
    }

    // symbol^{t}(seed) centred over (above = true) or under a head.
    void drawFlowLabel(const glm::vec3& head, const engine::LatexImage& symbol,
                       const engine::LatexImage* chars, const engine::LatexImage& seed,
                       const char* num, bool above, float headRadius, float alpha)
    {
        float sx, sy;
        if (!toScreen(head, sx, sy))
            return;
        const float S = cfg.stageLabelSize;
        const float raise = 0.42f * S;
        const float pad = (float)engine::LatexImage::kPad;
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
        place(symbol, 0.0f);
        x += 0.04f * S;
        for (const char* c = num; *c && n < 38; ++c)
            if (const char* at = std::strchr(kTimeLabelChars, *c))
                place(chars[at - kTimeLabelChars], raise);
        x += 0.06f * S;
        place(seed, 0.0f);
        float left = sx - 0.5f * x;
        float baseY = above ? sy - headRadius - 0.35f * S : sy + headRadius + 1.05f * S;
        for (int i = 0; i < n; ++i)
            pieces[i]->drawAt(left + xs[i], baseY + ys[i], alpha);
    }

    void renderStageB(float px)
    {
        if (pairs.empty())
            return;
        const float k = cfg.singleScale;
        auto curve = [&](const engine::Trajectory& t, const glm::vec3& color) {
            if (cfg.showTrails)
            {
                glLineWidth(cfg.trailWidth * k * px);
                shader.setVec3("u_color", color);
                shader.setFloat("u_alpha", cfg.trailAlpha * contentFade);
                t.draw();
            }
            shader.setVec3("u_color", t.diverged() ? kDivergedColor : color);
            shader.setFloat("u_alpha", contentFade);
            shader.setFloat("u_pointSize", cfg.particleSize * k * px);
            shader.setInt("u_round", 1);
            t.drawHead();
            shader.setInt("u_round", 0);
        };
        for (const StagePair& q : pairs)
        {
            if (cfg.showOnW)
                curve(*q.onW, kOnWColor);
            if (cfg.showPsi)
                curve(*q.lin, kPsiColor);
            if (cfg.showFlat)
                curve(*q.flat, kFlatColor);
        }
        const int n = (int)pairs.size();
        if (cfg.showLinks)
        {
            glLineWidth(1.5f * px);
            shader.setVec3("u_color", kLinkColor);
            shader.setFloat("u_alpha", 0.7f * contentFade);
            stageMesh->drawArrays(GL_LINES, 2 * n, 0);
        }
        if (cfg.showFormula)
        {
            // on top of the blue heads they should cover: no depth test
            glDisable(GL_DEPTH_TEST);
            shader.setVec3("u_color", kFormulaColor);
            shader.setFloat("u_alpha", contentFade);
            shader.setFloat("u_pointSize", 0.55f * cfg.particleSize * k * px);
            shader.setInt("u_round", 1);
            stageMesh->drawArrays(GL_POINTS, n, 2 * n);
            shader.setInt("u_round", 0);
            glEnable(GL_DEPTH_TEST);
        }
        shader.setFloat("u_pointSize", cfg.particleSize * px);
    }

    void frameCamera()
    {
        engine::Camera& cam = app->camera();
        cam.orbitTheta = cfg.camTheta;
        cam.orbitPhi = cfg.camPhi;
        cam.orbitRadius = cfg.camRadius > 0.0f ? cfg.camRadius : 3.0f * cfg.ballRadius;
        cam.orbitCenter = xStar;
    }

    // The orbit stays centred on x* (it moves when rho is dragged). Runs after
    // Camera::processInput, so position/front are recomputed for this frame.
    void updateCamera()
    {
        engine::Camera& cam = app->camera();
        cam.orbitAngularSpeed = cfg.orbitSpeed;
        cam.speed = cfg.zoomSpeed > 0.0f ? cfg.zoomSpeed : std::max(0.3f, 0.5f * cam.orbitRadius);
        // the pair's midpoint, raised a little so the names + systems above the balls fit
        const glm::vec3 camUp = glm::normalize(glm::cross(glm::cross(cam.front, cam.up), cam.front));
        cam.orbitCenter = xStar + 0.5f * hgSide * hgOffset() + hgFocus * 0.45f * cfg.hgRadius * camUp;
        if (hgAnimating)
        {
            // log-interpolated: saved distance -> framing V -> framing U and V together
            float rV = 3.2f * cfg.hgRadius;
            float half = std::tan(glm::radians(0.5f * cam.fovDeg));
            float span = 0.5f * glm::length(hgOffset()) + 1.3f * cfg.hgRadius;
            float rBoth = 1.15f * std::max(span / (half * std::max(cam.aspect, 0.2f)),
                                           1.9f * cfg.hgRadius / half);
            float l = std::log(hgSavedRadius) +
                      hgFocus * (std::log(rV) - std::log(hgSavedRadius)) +
                      hgSide * (std::log(rBoth) - std::log(rV));
            cam.orbitRadius = std::exp(l);
        }
        cam.position = cam.orbitCenter + cam.orbitRadius *
                       glm::vec3(std::sin(cam.orbitPhi) * std::sin(cam.orbitTheta),
                                 std::cos(cam.orbitPhi),
                                 std::sin(cam.orbitPhi) * std::cos(cam.orbitTheta));
        cam.front = glm::normalize(cam.orbitCenter - cam.position);
    }

    // Fade out, then hand back to lorenz_flow (handBack) — which zooms out from x*.
    void backToGlobal()
    {
        if (!app->hasSim(kFlowSim) || leaving)
            return;
        leaving = true;
        leaveT = 0.0f;
    }

    void handBack()
    {
        engine::App::Handoff out = returnView;
        out["sigma"] = cfg.sigma;
        out["rho"] = cfg.rho;
        out["beta"] = cfg.beta;
        out["eq"] = cfg.eq;
        out["beadRadius"] = beadShown;   // lorenz_flow grows it back from here
        app->requestSwitch(kFlowSim, std::move(out));
    }

    bool saveCurrentSettings()
    {
        const engine::Camera& cam = app->camera();
        cfg.camTheta = cam.orbitTheta;
        cfg.camPhi = cam.orbitPhi;
        cfg.camRadius = cam.orbitRadius;
        return saveSettingsFile(cfg, kSettingsPath);
    }

    // The bead at x* (always lit — it is the one you zoomed onto), or its halo.
    void renderBead(const engine::Camera& camera, const glm::mat4& view,
                    const glm::mat4& projection, bool halo, const glm::vec3& center, float alpha)
    {
        float pulse = 1.0f + 0.12f * std::sin(6.0f * clock);
        eqShader.use();
        eqShader.setMat4("view", view);
        eqShader.setMat4("projection", projection);
        eqShader.setVec3("u_eye", camera.position);
        eqShader.setInt("u_mode", halo ? 1 : 0);
        eqShader.setVec3("u_center", center);
        eqShader.setFloat("u_radius", beadShown * (halo ? 2.2f * pulse : 1.0f));
        eqShader.setVec3("u_color", halo ? kHaloColor : kEquilibriumColor);
        eqShader.setFloat("u_glow", halo ? pulse : 1.0f);
        eqShader.setFloat("u_alpha", alpha);
        if (halo)
        {
            glBlendFunc(GL_ONE, GL_ONE);
            glDepthMask(GL_FALSE);
        }
        sphereMesh->draw(GL_TRIANGLES);
        if (halo)
        {
            glDepthMask(GL_TRUE);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }
    }

    // O, C_+ or C_- beside the bead, where lorenz_flow drew it.
    void drawEquilibriumLabel()
    {
        if (!cfg.showEqLabel)
            return;
        if (labelSize != cfg.eqLabelSize)
        {
            labelSize = cfg.eqLabelSize;
            label.set(labelTex, cfg.eqLabelSize, glm::vec3(1.0f, 0.45f, 0.40f));
        }
        float sx, sy;
        if (toScreen(xStar, sx, sy))
            label.drawAt(sx + 10.0f + 0.3f * cfg.eqLabelSize, sy - (float)label.height() - 2.0f);
    }

    // The system with its live parameters, top-right (as in lorenz_flow).
    void drawSystemOverlay()
    {
        if (!cfg.showSystemLatex && !cfg.showHud)
            return;
        if (latexDirty)
        {
            latexDirty = false;
            std::string s = texNumber(cfg.sigma), r = texNumber(cfg.rho), b = texNumber(cfg.beta);
            systemLatex.set(R"(\begin{pmatrix}\dot{x} \\ \dot{y} \\ \dot{z}\end{pmatrix} = )"
                            R"(\begin{pmatrix})" + s + R"((y - x) \\ x()" + r +
                            R"( - z) - y \\ x y - )" + b + R"(z\end{pmatrix})", cfg.systemLatexSize);
        }
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12.0f,
                                       vp->WorkPos.y + 12.0f),
                                ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.35f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##lorenzEqLatex", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
        if (cfg.showSystemLatex)
            systemLatex.draw();
        // while U is beside V its own label says theta' = Df(x*) theta, and the HUD would
        // sit right on top of it
        if (cfg.showHud && hgSide < 0.01f)
            drawHud();
        ImGui::End();
        ImGui::PopStyleVar();
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

    // ---- trajectories -------------------------------------------------------------

    // Under the cursor on the camera-facing plane through x*; only inside V.
    void dropSingleUnderCursor()
    {
        glm::vec3 origin, dir;
        if (!cursorRay(origin, dir))
            return;
        const engine::Camera& cam = app->camera();
        float denom = glm::dot(dir, cam.front);
        if (std::abs(denom) < 1e-4f)
            return;
        float s = glm::dot(xStar - origin, cam.front) / denom;
        glm::vec3 p = origin + dir * s;
        if (s > 0.0f && glm::length(p - xStar) < cfg.ballRadius)
            addTrajectory(p, kSingleColor, Kind::Single);
    }

    void addTrajectory(const glm::vec3& seed, const glm::vec3& color, Kind kind)
    {
        if (curves.size() >= kMaxTrajectories)
            return;
        double s[3] = {seed.x, seed.y, seed.z};
        auto traj = std::make_unique<engine::Trajectory>(3, s, fieldTime, cfg.trailPoints);
        traj->minSpacing = 0.002f * cfg.ballRadius;
        curves.push_back({std::move(traj), seed, color, kind});
    }

    void removeKind(Kind kind)
    {
        std::erase_if(curves, [kind](const Curve& c) { return c.kind == kind; });
    }

    // Seeds around x*, replacing the previous ones: a Fibonacci sphere of radius
    // seedRadius * r, or uniform through that ball.
    void dropSeeds()
    {
        removeKind(Kind::Seed);
        std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
        const float golden = 2.39996323f;
        const float R = cfg.seedRadius * cfg.ballRadius;
        const int n = cfg.seedCount;
        for (int k = 0; k < n && curves.size() < kMaxTrajectories; ++k)
        {
            glm::vec3 p(0.0f);
            if (cfg.seedFilled)
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
            addTrajectory(xStar + p * R, kSeedColor, Kind::Seed);
        }
    }

    void restartTrajectories()
    {
        for (Curve& c : curves)
        {
            double s[3] = {c.seed.x, c.seed.y, c.seed.z};
            c.traj->reset(s, fieldTime);
        }
        for (StagePair& q : pairs)
        {
            q.lin->reset(q.flatSeed, fieldTime);
            q.flat->reset(q.flatSeed, fieldTime);
            q.onW->reset(q.onSeed, fieldTime);
            q.t0 = fieldTime;
        }
        for (HgPair& q : hgPairList)
        {
            q.lin->reset(q.linSeed, fieldTime);
            q.tru->reset(q.truSeed, fieldTime);
            q.linCrashed = q.truCrashed = false;
            q.linFade = q.truFade = 1.0f;
            for (int c = 0; c < 3; ++c)
                q.theta[c] = q.theta0[c];
            q.t0 = fieldTime;
        }
    }

    // ---- arrow grid ---------------------------------------------------------------

    // The field on a lattice through V (points outside the ball dropped), sampled in
    // parallel.
    void rebuildArrows()
    {
        arrowsDirty = false;
        hgArrowsDirty = true;
        gridPoints.clear();
        const float R = cfg.ballRadius;
        const float spacing = 2.0f * R / (float)cfg.gridN;
        for (int k = 0; k <= cfg.gridN; ++k)
            for (int j = 0; j <= cfg.gridN; ++j)
                for (int i = 0; i <= cfg.gridN; ++i)
                {
                    glm::vec3 d(-R + i * spacing, -R + j * spacing, -R + k * spacing);
                    if (glm::length(d) <= R)
                        gridPoints.push_back(xStar + d);
                }

        gridVectors.resize(gridPoints.size());
        std::transform(std::execution::par, gridPoints.begin(), gridPoints.end(),
                       gridVectors.begin(), [this](const glm::vec3& p) {
                           double x[3] = {p.x, p.y, p.z}, v[3];
                           system.eval(x, v);
                           return glm::vec3((float)v[0], (float)v[1], (float)v[2]);
                       });
        // A(x - x*) at the same points, and what it leaves out
        linearVectors.resize(gridPoints.size());
        for (size_t k = 0; k < gridPoints.size(); ++k)
        {
            glm::vec3 u = gridPoints[k] - xStar;
            for (int c = 0; c < 3; ++c)
                linearVectors[k][c] = (float)(A[c][0] * u.x + A[c][1] * u.y + A[c][2] * u.z);
        }
        if (cfg.arrowField == kFieldNonlinear)
            for (size_t k = 0; k < gridPoints.size(); ++k)
                gridVectors[k] -= linearVectors[k];
        else if (cfg.arrowField == kFieldLinear)
            gridVectors = linearVectors;

        // f and A(x - x*) share one color/length scale so they compare; the remainder
        // (small near x*) gets its own
        sampledMaxMag = 0.0f;
        for (const glm::vec3& v : gridVectors)
            sampledMaxMag = std::max(sampledMaxMag, glm::length(v));
        if (cfg.arrowField == kFieldBoth)
            for (const glm::vec3& v : linearVectors)
                sampledMaxMag = std::max(sampledMaxMag, glm::length(v));

        for (engine::ArrowGlyphs* g : {&glyphs, &linearGlyphs})
        {
            g->clear();
            g->barbs = cfg.barbedHeads;
            g->headFraction = cfg.headFraction;
            g->headWidth = cfg.headWidth;
        }
        glyphs.addField(gridPoints, gridVectors, spacing * cfg.arrowScale,
                        std::max(sampledMaxMag, 1e-6f), 0.0f, cfg.lengthByMagnitude, 3);
        glyphs.upload();
        if (cfg.arrowField == kFieldBoth)
            linearGlyphs.addField(gridPoints, linearVectors, spacing * cfg.arrowScale,
                                  std::max(sampledMaxMag, 1e-6f), 0.0f, cfg.lengthByMagnitude, 3);
        linearGlyphs.upload();
    }

    static constexpr size_t kMaxTrajectories = 4000;
    static constexpr int kMaxPairs = 64;    // stage B ring seeds
    static constexpr int kMaxHgPairs = 400; // stage C
    static constexpr int kSubspaceCapacity = 2048;   // vertices: a disk + rim (4 x 256) + lines

    // Pieces of the stage B labels, typeset once per size.
    struct StageLabelGlyphs {
        float size = 0.0f;
        engine::LatexImage phi, x0, psi, theta0;
        engine::LatexImage phiChars[12], psiChars[12];   // superscript t, kTimeLabelChars order
    };
    static constexpr const char* kTimeLabelChars = "0123456789.-";

    // One stage C pair: psi^t in the linear ball, phi^t from its h~^{-1} image.
    struct HgPair {
        double linSeed[3] = {};   // x* + theta
        double truSeed[3] = {};   // x* + h~^{-1}(theta)
        double t0 = 0.0;          // field time it was seeded at
        std::unique_ptr<engine::Trajectory> lin, tru;
        // each crashes on its own ball's border, then stops and dissolves: psi^t when
        // |psi^t - x*| reaches rho, phi^t when |h~(phi^t - x*)| does (V's real border)
        bool linCrashed = false, truCrashed = false;
        float linFade = 1.0f, truFade = 1.0f;
        double theta0[3] = {};    // the seed theta
        double theta[3] = {};     // h~(phi^t - x*), followed frame to frame (Newton)
        double linPrev[3] = {}, truPrev[3] = {};   // states before this frame's step
        glm::vec3 linAt{0.0f}, truAt{0.0f};        // where each exploded (frozen head)
        int linHead = -1, truHead = -1;            // their slots in headMesh
        bool alive() const { return !linCrashed && !truCrashed; }
    };

    // The fragments of one crashed particle.
    struct Shard {
        glm::vec3 pos, vel;
    };
    struct ShardGroup {
        std::vector<Shard> shards;
        glm::vec3 color{1.0f};
        float age = 0.0f;
    };

    // One crash shockwave: which ball (0 = V, 1 = U), where on it, whose color, its age.
    struct Burst {
        int ball = 0;
        glm::vec3 dir{0.0f, 0.0f, 1.0f};   // on the ball's unit sphere
        glm::vec3 pos{0.0f};               // world
        glm::vec3 color{1.0f};
        float age = 0.0f;
    };

    struct HgCheck {
        double defect = 0.0;      // max relative invariance defect of K_s, K_u in U
        double minDet = 1.0;      // min det D h~^{-1} in U
        bool folds = false;
        int level = 0;            // 0 ok, 1 approximate, 2 too big
        double safe = 0.0;        // largest radius that passes
        bool safeIsMax = false;
    };

    // One seed theta of the stage B ring and its three curves.
    struct StagePair {
        engine::InvariantManifold::Param s{};   // theta's manifold parameter
        double flatSeed[3] = {};                // x* + theta, on E^s
        double onSeed[3] = {};                  // x* + K_s(s), on W^s
        double t0 = 0.0;
        std::unique_ptr<engine::Trajectory> lin, flat, onW;
    };

    struct SubspaceDraw {
        GLenum mode;
        int first, count;
        glm::vec3 color;
        bool fill;
    };

    engine::App* app = nullptr;
    engine::Shader shader;        // glyphs + trails, clipped to V
    engine::Shader eqShader;      // the bead at x* + its halo
    engine::Shader glassShader;   // the ball V
    std::unique_ptr<engine::Mesh> sphereMesh;   // bead + halo
    std::unique_ptr<engine::Mesh> glassMesh;    // the ball V
    engine::ArrowGlyphs glyphs;
    engine::ArrowGlyphs linearGlyphs;           // A(x - x*), shown with f in "both"
    std::unique_ptr<engine::Mesh> subspaceMesh;
    std::vector<SubspaceDraw> subspaceDraws;
    engine::ArrowGlyphs subspaceArrows[2];      // E^s, E^u: the linear flow on them
    engine::LatexImage subspaceLabel[2];
    float subspaceLabelTypeset[2] = {0.0f, 0.0f};   // size each was typeset at
    glm::vec3 labelAnchor[2]{glm::vec3(NAN), glm::vec3(NAN)};
    int dims[2] = {0, 0};
    bool subspacesDirty = true;

    Settings cfg;
    std::string settingsStatus;
    int shotScale = 2;
    std::string shotStatus;

    engine::LorenzSystem system;
    std::vector<engine::LorenzSystem::Equilibrium> found;
    int eqCount = 0;
    glm::vec3 xStar{0.0f};
    double A[3][3] = {};                        // Df(x*)
    double xs[3] = {};                          // x*, in double
    engine::Integrator::Field rhsLinear;        // A(x - x*)
    engine::TaylorIntegrator::Jet jetLinear;
    engine::InvariantManifold Ws;               // K_s, the local W^s
    bool manifoldOk = false;
    std::vector<StagePair> pairs;
    std::unique_ptr<engine::Mesh> stageMesh;    // links + formula points
    StageLabelGlyphs stageGlyphs;

    engine::Shader hmShader;                    // V: the sphere through h~^{-1} (GLSL)
    engine::ConjugacyMap hm;                    // h~^{-1} = K_s + K_u
    bool hmOk = false;
    std::string hmError;
    bool hgActive = false;                      // stage C on screen (timeline > 0)
    bool hgTarget = false;                      // Show (true) / Hide (false) pressed last
    float hgP = 0.0f;                           // timeline, 0 .. kHgSideEnd
    bool hgAnimating = false;
    float hgFocus = 0.0f;                       // 0 -> 1: the scene goes black around V
    float hgSide = 0.0f;                        // 0 -> 1: U appears beside V
    float hgFade = 1.0f;                        // handoff fades only
    glm::vec3 hgDir{1.0f, 0.0f, 0.0f};          // V -> U, screen-right when Show was pressed
    float hgSavedRadius = 9.0f;                 // camera distance to return to
    engine::ArrowGlyphs hgArrowsU, hgArrowsV;
    float hgMaxMag = 1.0f;
    bool hgArrowsDirty = true;
    engine::LatexImage hgLabelU, hgLabelV;
    engine::LatexImage hgSystemU, hgSystemV;   // theta' = Df(x*) theta, x' = f(x)
    engine::LatexImage hInvLabel;               // on the links
    float hInvLabelSize = 0.0f;
    std::vector<Burst> bursts;
    engine::FrameCopy sceneCopy;                // the frame so far, for the shockwaves

    enum class MorphPhase { Off, Flowing, Paused, Morphing, Done };
    MorphPhase morphPhase = MorphPhase::Off;
    std::vector<std::unique_ptr<engine::Trajectory>> morphTrajs;   // the linear flow carried over
    double morphOde = 0.0;                      // ODE time flowed so far
    float morphClock = 0.0f;                    // seconds in the current phase
    float morphS = 0.0f;                        // 0 in U -> 1 in V
    engine::Shader morphShader;                 // morph_vertex.glsl + fragment.glsl
    std::vector<ShardGroup> shardGroups;
    std::unique_ptr<engine::Mesh> shardMesh;
    std::unique_ptr<engine::Mesh> headMesh;     // frozen heads of crashed pairs
    float hgLabelSize = 0.0f;
    std::vector<HgPair> hgPairList;
    std::unique_ptr<engine::Mesh> hgMesh;       // links + predicted points
    int hgLinkCount = 0, hgPredictedCount = 0;
    HgCheck hgCheck;
    bool hgCheckDirty = true;
    engine::Eigen3 eig;
    engine::LatexImage hudMatrix, hudEigen;
    bool hudDirty = true;
    engine::Integrator integrator;
    engine::Integrator::Field rhs;
    engine::Integrator singleIntegrator;   // singles, unless they use the Taylor method
    engine::TaylorIntegrator taylor;
    engine::TaylorIntegrator::Jet jet;     // LorenzSystem::jet
    engine::TrajectoryRunner runner;
    std::vector<Curve> curves;
    double fieldTime = 0.0;
    bool playing = true;
    std::mt19937 rng{1234};

    bool handedOver = false;           // entered from lorenz_flow (camera kept)
    engine::App::Handoff returnView;   // lorenz_flow's global view, handed back on leaving
    bool leaving = false;              // fading out before handing back
    bool switchRequested = false;
    float leaveT = 0.0f;
    float clock = 0.0f;                // since init: entry fade, halo pulse
    float contentFade = 1.0f;          // glass, arrows, trails
    float beadShown = 0.06f;           // drawn bead radius, eased toward cfg.beadRadius

    engine::LatexImage label;          // O, C_+ or C_-
    std::string labelTex;
    float labelSize = 0.0f;
    engine::LatexImage systemLatex;
    bool latexDirty = true;

    bool arrowsDirty = true;
    float sampledMaxMag = 0.0f;
    std::vector<glm::vec3> gridPoints;
    std::vector<glm::vec3> gridVectors;
    std::vector<glm::vec3> linearVectors;
};

} // namespace

std::unique_ptr<engine::Sim> makeLorenzEquilibriumSim()
{
    return std::make_unique<LorenzEquilibriumSim>();
}
