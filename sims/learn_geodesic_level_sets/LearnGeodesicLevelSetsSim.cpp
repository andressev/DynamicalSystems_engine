#include "LearnGeodesicLevelSetsSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/InvariantManifold.h>
#include <engine/LatexImage.h>
#include <engine/LorenzSystem.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>
#include <engine/FlowKeys.h>
#include <engine/TaylorIntegrator.h>
#include <engine/Trajectory.h>

#include <gtc/quaternion.hpp>
#include <imgui.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace {

// ---- colors, small helpers ------------------------------------------------------

const glm::vec3 kEquilibriumColor = {0.92f, 0.12f, 0.10f};
// the manifolds, in lorenz_manifold_growth's colors
const glm::vec3 kStableColor(0.45f, 0.72f, 1.0f);     // W^s
const glm::vec3 kUnstableColor(1.0f, 0.55f, 0.45f);   // W^u
const glm::vec3 kVertexColor(1.0f, 0.84f, 0.22f);     // the mesh's vertices
const glm::vec3 kEdgeColor(0.86f, 0.90f, 0.96f);      // its edges
const glm::vec3 kPointColor(1.0f, 1.0f, 1.0f);        // the edge point r
const glm::vec3 kParticleColor(0.35f, 0.95f, 0.45f);  // the edge ball and its particles
const glm::vec3 kHitColor(1.0f, 0.70f, 0.16f);        // b_r(tau): where a particle meets F_r
const glm::vec3 kNeighborColor(1.0f, 0.58f, 0.22f);   // r^+, r^-
const glm::vec3 kNewColor(0.35f, 0.85f, 1.0f);        // r_new: the BVP solutions, their band
const glm::vec3 kPlaneColor(0.72f, 0.74f, 0.78f);     // the half-plane F_r

const char* kSettingsPath = SIMS_DIR "learn_geodesic_level_sets/settings.txt";
const char* kGrowthSim = "lorenz_manifold_growth";   // where the walkthrough is entered from

// The trusted radius, exactly as lorenz_manifold_growth finds it, so the patch handed
// over is the patch drawn here.
constexpr double kDefectOk = 1e-3;
constexpr double kRadiusSearchMax = 25.0;
constexpr int kMaxRings = 64, kMaxSegs = 256;
constexpr int kMaxEdge = 2048;          // edge points after the maxSpread / minSpread rule
// fill (center fan + quads) + edges (loops, spokes, diagonals) + points
constexpr int kCapacity = kMaxSegs * 3 + (kMaxRings - 1) * kMaxSegs * 6 +
                          kMaxRings * kMaxSegs * 4 + (kMaxRings - 1) * kMaxSegs * 2 +
                          1 + kMaxRings * kMaxSegs +
                          kMaxEdge * 16;   // the edge band, its lines and points
constexpr float kFramingScale = 3.2f;   // camera distance / patch radius
constexpr int kOverlayCapacity = 160;   // r, its neighbours, n (shaft + cone), F_r
constexpr int kBandCapacity = 24 * kMaxEdge;   // the new band: fill, lines, points, orbit
constexpr int kLayerCapacity = 1 << 20;         // accepted bands: fill, lines, edge loop + points
constexpr float kPickPixels = 14.0f;    // a click this close to an edge vertex picks it
constexpr int kEdgeSamples = 1024;      // the edge, sampled for dragging the ball along it
constexpr int kMaxParticles = 700;      // launched from the ball; the oldest goes first
constexpr int kMaxRing = 512;           // B: particles from the whole edge (subsampled past this)
constexpr int kRingPoints = 5000;       // their trails: fewer points, there are many of them
constexpr float kRingSpacing = 2e-3f;   // ...spaced this far apart (x radius): trails ~10 radii long
constexpr int kMaxShoot = 128;          // particles in one shot from r to the ball
constexpr float kBallStep = 0.25f;      // radians: how far the ball may slide in one frame
constexpr int kHitCapacity = 4 * kMaxParticles;   // per hit: the dot, the segment to r
constexpr int kParticlePoints = 60000;  // trail of each
constexpr float kMinParticleSpeed = 1e-6f;   // field time per second, arrow-key range
constexpr float kMaxParticleSpeed = 1.0f;

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

// Number for typeset labels: 2 decimals, trailing zeros dropped (0.250 -> 0.25, 1.00 -> 1).
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

// ---- settings -------------------------------------------------------------------

struct Settings {
    float sigma = 10.0f;
    float rho = 28.0f;
    float beta = 8.0f / 3.0f;
    int eq = 0;                   // O, C+, C- (the 2-D side of it is shown)
    int manifoldOrder = 12;       // K's Taylor order

    int rings = 10;               // the mesh: rings around x* ...
    int segs = 48;                // ... of this many vertices each
    bool showMesh = false;
    float fillAlpha = 0.3f;
    float edgeAlpha = 0.6f;
    float edgeWidth = 1.2f;       // px
    float vertexSize = 7.0f;      // px
    float boundaryWidth = 2.1f;   // px, the patch's edge
    float meshFadeSeconds = 0.35f;

    float planeScale = 3.0f;      // F_r reaches this many |r^+ - r^-| out from the edge
    float planeAlpha = 0.35f;
    float zoomScale = 1.6f;       // zoom onto r: camera distance / the plane's size

    float particleSpeed = 0.0005f; // field time per second: the edge is a small distance
    float ballSize = 16.0f;       // px, the ball on the edge
    float particleSize = 9.0f;    // px, a particle's head
    float particleTrail = 2.0f;   // px, its trail
    int shootCount = 32;          // "Shoot r -> ball": particles along the edge arc

    float eqRadius = 0.7f;        // bead size, world units
    bool showLabels = true;
    float labelSize = 30.0f;      // LaTeX sizes, px: the equilibrium label
    float patchLabelSize = 24.0f;      // W_loc
    float edgePointLabelSize = 30.0f;  // r and F_r (r^+-, n at 0.8 of it)
    float hitLabelSize = 24.0f;        // b_r(tau) and its distance to r
    bool showParams = true;            // the method's parameters, top-right
    float paramsLatexSize = 22.0f;

    // geodesic level sets: neighbouring points of the edge stay between these
    float maxSpread = 1.0f;
    float minSpread = 0.25f;
    // ... and the step between level sets, Delta_i, stays between these (the first ring
    // aims for deltaMax; a ring takes the smallest distance its points reached)
    float deltaMin = 0.25f;
    float deltaMax = 1.0f;
    // Delta_i adaptation (Krauskopf & Osinga 3.2): alpha_r, the angle at r between
    // p_r -> r and r -> b_r. A ring is redone at half Delta_i if any alpha_r >= alphaMax
    // or Delta_i alpha_r >= dAlphaMax (accepted anyway at deltaMin, with a warning); the
    // next ring doubles Delta_i if every alpha_r < alphaMin and Delta_i alpha_r < dAlphaMin
    float alphaMin = 0.3f, alphaMax = 0.4f;
    float dAlphaMin = 0.1f, dAlphaMax = 1.0f;
    int growRings = 10;                // "Grow rings": how many in one run
    bool showAxes = false;

    // orbit view around x* (camRadius <= 0: frame the patch)
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
        {"manifoldOrder", nullptr, &s.manifoldOrder},
        {"rings", nullptr, &s.rings}, {"segs", nullptr, &s.segs},
        {"showMesh", nullptr, nullptr, &s.showMesh},
        {"fillAlpha", &s.fillAlpha}, {"edgeAlpha", &s.edgeAlpha},
        {"edgeWidth", &s.edgeWidth}, {"vertexSize", &s.vertexSize},
        {"boundaryWidth", &s.boundaryWidth}, {"meshFadeSeconds", &s.meshFadeSeconds},
        {"planeScale", &s.planeScale}, {"planeAlpha", &s.planeAlpha},
        {"zoomScale", &s.zoomScale},
        {"particleSpeed", &s.particleSpeed}, {"ballSize", &s.ballSize},
        {"particleSize", &s.particleSize}, {"particleTrail", &s.particleTrail},
        {"shootCount", nullptr, &s.shootCount},
        {"eqRadius", &s.eqRadius},
        {"showLabels", nullptr, nullptr, &s.showLabels},
        {"labelSize", &s.labelSize}, {"patchLabelSize", &s.patchLabelSize},
        {"edgePointLabelSize", &s.edgePointLabelSize}, {"hitLabelSize", &s.hitLabelSize},
        {"showParams", nullptr, nullptr, &s.showParams},
        {"paramsLatexSize", &s.paramsLatexSize},
        {"maxSpread", &s.maxSpread}, {"minSpread", &s.minSpread},
        {"deltaMin", &s.deltaMin}, {"deltaMax", &s.deltaMax},
        {"alphaMin", &s.alphaMin}, {"alphaMax", &s.alphaMax},
        {"dAlphaMin", &s.dAlphaMin}, {"dAlphaMax", &s.dAlphaMax},
        {"growRings", nullptr, &s.growRings},
        {"showAxes", nullptr, nullptr, &s.showAxes},
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
    s.manifoldOrder = std::clamp(s.manifoldOrder, 2, 30);
    s.rings = std::clamp(s.rings, 1, kMaxRings);
    s.segs = std::clamp(s.segs, 3, kMaxSegs);
    s.fillAlpha = std::clamp(s.fillAlpha, 0.0f, 1.0f);
    s.edgeAlpha = std::clamp(s.edgeAlpha, 0.0f, 1.0f);
    s.edgeWidth = std::clamp(s.edgeWidth, 0.5f, 8.0f);
    s.vertexSize = std::clamp(s.vertexSize, 1.0f, 30.0f);
    s.boundaryWidth = std::clamp(s.boundaryWidth, 0.5f, 8.0f);
    s.meshFadeSeconds = std::clamp(s.meshFadeSeconds, 0.0f, 3.0f);
    s.labelSize = std::clamp(s.labelSize, 10.0f, 120.0f);
    s.patchLabelSize = std::clamp(s.patchLabelSize, 10.0f, 120.0f);
    s.edgePointLabelSize = std::clamp(s.edgePointLabelSize, 10.0f, 120.0f);
    s.hitLabelSize = std::clamp(s.hitLabelSize, 10.0f, 120.0f);
    s.paramsLatexSize = std::clamp(s.paramsLatexSize, 10.0f, 120.0f);
    s.maxSpread = std::clamp(s.maxSpread, 0.01f, 100.0f);
    s.minSpread = std::clamp(s.minSpread, 0.005f, 0.5f * s.maxSpread);
    s.deltaMax = std::clamp(s.deltaMax, 0.01f, 100.0f);
    s.deltaMin = std::clamp(s.deltaMin, 0.005f, s.deltaMax);
    s.alphaMax = std::clamp(s.alphaMax, 0.01f, 3.0f);
    s.alphaMin = std::clamp(s.alphaMin, 0.0f, s.alphaMax);
    s.dAlphaMax = std::clamp(s.dAlphaMax, 0.001f, 100.0f);
    s.dAlphaMin = std::clamp(s.dAlphaMin, 0.0f, s.dAlphaMax);
    s.growRings = std::clamp(s.growRings, 1, 1000);
    s.planeScale = std::clamp(s.planeScale, 0.5f, 20.0f);
    s.planeAlpha = std::clamp(s.planeAlpha, 0.0f, 1.0f);
    s.zoomScale = std::clamp(s.zoomScale, 0.5f, 20.0f);
    s.particleSpeed = std::clamp(s.particleSpeed, kMinParticleSpeed, kMaxParticleSpeed);
    s.ballSize = std::clamp(s.ballSize, 2.0f, 60.0f);
    s.particleSize = std::clamp(s.particleSize, 1.0f, 40.0f);
    s.particleTrail = std::clamp(s.particleTrail, 0.5f, 10.0f);
    s.shootCount = std::clamp(s.shootCount, 2, kMaxShoot);
}

// ---- the sim --------------------------------------------------------------------

// The walkthrough of geodesic level sets, entered from lorenz_manifold_growth's popup
// (App::requestSwitch, same parameters, the camera left framed on the patch): the 2-D
// local manifold of one equilibrium alone — W^s_loc(O) at the classic parameters —
// and, on request, the triangle mesh it is drawn with, vertices in green. The
// algorithm itself is not here yet. See obsidian_twin/sims/learn-geodesic-level-sets.md.
class LearnGeodesicLevelSetsSim : public engine::Sim {
public:
    const char* name() const override { return "learn_geodesic_level_sets"; }

    // The camera is shared: whatever comes next gets world-up back.
    ~LearnGeodesicLevelSetsSim() override
    {
        if (app)
            app->camera().up = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    void init(engine::App& app) override
    {
        this->app = &app;
        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(SIMS_DIR "learn_geodesic_level_sets/shaders/vertex.glsl",
                                SIMS_DIR "learn_geodesic_level_sets/shaders/fragment.glsl");
        eqShader = engine::Shader(SIMS_DIR "learn_geodesic_level_sets/shaders/equilibrium_vertex.glsl",
                                  SIMS_DIR "learn_geodesic_level_sets/shaders/equilibrium_fragment.glsl");
        axisMesh = std::make_unique<engine::Mesh>(engine::Shapes::axisVertices(1000.0f),
                                                  engine::Shapes::axisIndices(),
                                                  std::vector<int>{3});
        sphereMesh = std::make_unique<engine::Mesh>(engine::Shapes::sphereVertices(48, 32, 1.0f),
                                                    engine::Shapes::sphereIndices(48, 32),
                                                    std::vector<int>{3, 3, 2});
        patchMesh = std::make_unique<engine::Mesh>(kCapacity, std::vector<int>{3});
        overlayMesh = std::make_unique<engine::Mesh>(kOverlayCapacity, std::vector<int>{3});
        ballMesh = std::make_unique<engine::Mesh>(1, std::vector<int>{3});
        hitMesh = std::make_unique<engine::Mesh>(kHitCapacity, std::vector<int>{3});
        frontMesh = std::make_unique<engine::Mesh>(kMaxParticles + 64, std::vector<int>{3});
        bandMesh = std::make_unique<engine::Mesh>(kBandCapacity, std::vector<int>{3});
        layerMesh = std::make_unique<engine::Mesh>(kLayerCapacity, std::vector<int>{3});
        jet = [this](const double* x, double, int order, double* c) { system.jet(x, order, c); };

        loadSettingsFile(cfg, kSettingsPath);

        // from lorenz_manifold_growth: its parameters, which patch, its K order and bead
        // size; everything handed over goes back on leaving (its global view, what it
        // had grown). The camera is left where its zoom put it.
        const engine::App::Handoff& in = app.handoff();
        auto value = [&](const char* key, double fallback) {
            auto it = in.find(key);
            return it == in.end() ? fallback : it->second;
        };
        handedOver = in.count("eq") > 0;
        if (handedOver)
        {
            returnValues = in;
            cfg.sigma = (float)value("sigma", cfg.sigma);
            cfg.rho = (float)value("rho", cfg.rho);
            cfg.beta = (float)value("beta", cfg.beta);
            cfg.eq = std::clamp((int)value("eq", cfg.eq), 0, 2);
            side = std::clamp((int)value("side", 0.0), 0, 1);
            cfg.manifoldOrder = std::clamp((int)value("order", cfg.manifoldOrder), 2, 30);
            cfg.eqRadius = (float)value("eqRadius", cfg.eqRadius);
            cfg.fillAlpha = (float)value("fillAlpha", cfg.fillAlpha);
        }

        engine::Camera& cam = app.camera();
        cam.mode = engine::Camera::Mode::Orbit;
        cam.inputEnabled = true;
        cam.up = glm::vec3(0.0f, 1.0f, 0.0f);
        cam.orbitRadiusMin = 0.05f;
        cam.orbitSlowZone = 0.0f;
        cam.farPlane = 500.0f;
        rebuild();
        if (!handedOver)
            frameCamera(false);
    }

    void update(float dt) override
    {
        updateViewFrame(dt);   // first: picking and labels use this frame's camera
        clock += dt;
        const ImGuiIO& io = ImGui::GetIO();
        // Backspace, not Escape: App closes the window on Escape
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Backspace, false))
            handBack();
        // Space: particles play/pause; arrows: Up/Down speed, Left/Right direction
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false))
            playing = !playing;
        engine::FlowKeys::handle(cfg.particleSpeed, reverse, dt, kMaxParticleSpeed, kMinParticleSpeed);
        engine::FlowKeys::drawStatus(cfg.particleSpeed, reverse, playing);
        // B: the whole edge flows (backward on W^s) — the ring tears apart at once, which
        // is why the surface is not grown by flowing it
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_B, false))
            shootRing();
        // R: clear the particles
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_R, false))
            particles.clear();
        // M: the mesh on/off (same as the checkbox)
        if (!io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_M, false))
            cfg.showMesh = !cfg.showMesh;
        // 0: supersampled screenshot of the scene alone
        if (!io.WantCaptureKeyboard &&
            (ImGui::IsKeyPressed(ImGuiKey_0, false) || ImGui::IsKeyPressed(ImGuiKey_Keypad0, false)))
            takeScreenshot();

        // the mesh fades in and out instead of cutting
        const float target = cfg.showMesh ? 1.0f : 0.0f;
        const float step = cfg.meshFadeSeconds > 0.0f ? dt / cfg.meshFadeSeconds : 1.0f;
        meshFade += std::clamp(target - meshFade, -step, step);
        rFade += std::clamp((edgeIndex >= 0 ? 1.0f : 0.0f) - rFade, -step, step);

        // the ball on the edge: press and drag slides it along the edge, a double click
        // launches a particle from it. It comes before r's picking: on the ball, a
        // click is the ball's.
        ballHover = !io.WantCaptureMouse && !edgeCurve.empty() && overBall();
        if (ballHover && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            launchParticle();
        else if (ballHover && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            ballDragging = true;
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
            ballDragging = false;
        if (ballDragging)
            ballAngle = slideBall(io.MousePos.x, io.MousePos.y);
        // V with the cursor on the ball: shoot from r to it
        if (ballHover && !io.WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_V, false))
            shootFromR();

        // r: the edge vertex under the cursor lights; a click picks it and zooms in
        hoverEdge = io.WantCaptureMouse || ballHover || ballDragging ? -1 : pickEdge();
        if (hoverEdge >= 0 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
            chooseEdge(hoverEdge);
        // a double click also solves the geodesic BVP there
        if (hoverEdge >= 0 && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
            solveAt(hoverEdge);
        stepSolveAll();

        // the particles: the manifold's own direction of time — backward on W^s (into the
        // past, away from x*), forward on W^u — times the arrow keys' speed and direction
        syncHits();
        if (playing && !particles.empty())
        {
            const double h = (double)dt * cfg.particleSpeed * (side == 0 ? -1.0 : 1.0) *
                             (reverse ? -1.0 : 1.0);
            EdgeFrame f;
            const bool haveR = edgeFrame(edgeIndex, f);
            for (auto& P : particles)
            {
                engine::Trajectory& t = *P->traj;
                if (t.diverged() || P->hit)
                    continue;   // a particle that met F_r stays there
                const double* x = t.state();
                const glm::dvec3 x0(x[0], x[1], x[2]);
                const double t0 = t.time();
                t.integrate(taylor, jet, h, std::abs(h) / 8.0 + 1e-12);
                t.flush();
                if (haveR && !P->hit && !P->ring)
                    findHit(*P, f, x0, t0);
            }
        }

        engine::Camera& cam = app->camera();
        cam.orbitAngularSpeed = cfg.orbitSpeed;
        cam.speed = cfg.zoomSpeed > 0.0f ? cfg.zoomSpeed : std::max(0.5f, 0.5f * cam.orbitRadius);

        drawLabels();
        drawEdgeLabels();
        drawHitLabels();
        drawNewLabel();
        drawParamsOverlay();
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix();
        const float px = app->renderScale();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // the bead first: opaque, so the blended patch layers over it
        if (found)
        {
            eqShader.use();
            eqShader.setMat4("view", view);
            eqShader.setMat4("projection", projection);
            eqShader.setVec3("u_eye", camera.position);
            eqShader.setInt("u_mode", 0);
            eqShader.setVec3("u_center", eqPos);
            eqShader.setFloat("u_radius", cfg.eqRadius);
            eqShader.setVec3("u_color", kEquilibriumColor);
            eqShader.setFloat("u_glow", 0.0f);
            eqShader.setFloat("u_alpha", 1.0f);
            sphereMesh->draw(GL_TRIANGLES);
        }

        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setFloat("u_magScale", 1.0f);
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_alphaTail", 1.0f);
        shader.setInt("u_useFade", 0);
        shader.setInt("u_shadeMode", 0);
        shader.setInt("u_round", 0);
        shader.setInt("u_useRamp", 0);
        shader.setInt("u_instanced", 0);

        if (cfg.showAxes)
        {
            shader.setVec3("u_color", glm::vec3(0.35f, 0.35f, 0.42f));
            glLineWidth(1.0f * px);
            axisMesh->draw(GL_LINES);
        }

        if (fillCount > 0)
        {
            const glm::vec3 color = side == 0 ? kStableColor : kUnstableColor;
            // nothing on the patch writes depth: the fill, its edges and its vertices
            // never hide each other, only the bead hides what is behind it
            glDepthMask(GL_FALSE);
            shader.setVec3("u_color", color);
            shader.setFloat("u_alpha", cfg.fillAlpha);
            patchMesh->drawArrays(GL_TRIANGLES, fillCount, 0);
            if (meshFade > 0.0f)
            {
                shader.setVec3("u_color", kEdgeColor);
                shader.setFloat("u_alpha", cfg.edgeAlpha * meshFade);
                glLineWidth(cfg.edgeWidth * px);
                patchMesh->drawArrays(GL_LINES, edgeCount, fillCount);
            }
            shader.setVec3("u_color", color);
            shader.setFloat("u_alpha", 0.9f);
            glLineWidth(cfg.boundaryWidth * px);
            if (layer == 0)
                patchMesh->drawArrays(GL_LINE_LOOP, (int)edgeAngles.size(), boundaryFirst);
            if (meshFade > 0.0f)
            {
                shader.setVec3("u_color", kVertexColor);
                shader.setFloat("u_alpha", meshFade);
                shader.setFloat("u_pointSize", cfg.vertexSize * px);
                shader.setInt("u_round", 1);
                patchMesh->drawArrays(GL_POINTS, pointCount, fillCount + edgeCount);
                shader.setInt("u_round", 0);
            }
            if (layer > 0)
                renderLayers(color, px);
            renderEdgePoint(px);
            renderBand(px);
            renderParticles(px);
            renderHits(px);
            glDepthMask(GL_TRUE);
        }
        shader.setFloat("u_alpha", 1.0f);
        glLineWidth(1.0f);
        glDisable(GL_BLEND);
    }

    // Hold 1 (App::kRecenterHoldSeconds): glide back to the framing of the patch.
    void recenterCamera() override
    {
        frameCamera(true);
    }

    void drawUI() override
    {
        sectionHeader("Local manifold");
        static const char* kNames[] = {"O", "C+", "C-"};
        if (ImGui::BeginCombo("equilibrium", kNames[cfg.eq]))
        {
            for (int i = 0; i < eqTotal; ++i)
                if (ImGui::Selectable(kNames[i], i == cfg.eq) && i != cfg.eq)
                {
                    cfg.eq = i;
                    rebuild();
                    frameCamera(true);
                }
            ImGui::EndCombo();
        }
        if (!error.empty())
            ImGui::TextDisabled("%s", error.c_str());
        else
            ImGui::TextDisabled("r = %.2f", radius);
        if (app->hasSim(kGrowthSim))
        {
            if (ImGui::Button("Back to the flow"))
                handBack();
            ImGui::SameLine();
            ImGui::TextDisabled("(Backspace)");
        }

        sectionHeader("Mesh");
        ImGui::Checkbox("show mesh", &cfg.showMesh);
        ImGui::SameLine();
        ImGui::TextDisabled("(M)");
        bool remesh = false;
        remesh |= ImGui::SliderInt("rings", &cfg.rings, 1, kMaxRings);
        remesh |= ImGui::SliderInt("vertices per ring", &cfg.segs, 3, kMaxSegs);
        if (remesh)
            buildMesh();
        ImGui::TextDisabled("%d vertices, %d triangles, %d on the edge", pointCount,
                            fillCount / 3, (int)edgeAngles.size());

        sectionHeader("Geodesic level sets");
        bool respread = false;
        respread |= ImGui::SliderFloat("maxSpread", &cfg.maxSpread, 0.05f, 10.0f, "%.2f",
                                       ImGuiSliderFlags_Logarithmic);
        respread |= ImGui::SliderFloat("minSpread", &cfg.minSpread, 0.01f, 5.0f, "%.2f",
                                       ImGuiSliderFlags_Logarithmic);
        if (respread)
        {
            cfg.minSpread = std::min(cfg.minSpread, 0.5f * cfg.maxSpread);
            buildMesh();
        }
        bool redelta = false;
        redelta |= ImGui::SliderFloat("Delta_min", &cfg.deltaMin, 0.01f, 10.0f, "%.2f",
                                      ImGuiSliderFlags_Logarithmic);
        redelta |= ImGui::SliderFloat("Delta_max", &cfg.deltaMax, 0.01f, 10.0f, "%.2f",
                                      ImGuiSliderFlags_Logarithmic);
        if (redelta)
        {
            cfg.deltaMin = std::min(cfg.deltaMin, cfg.deltaMax);
            stepDelta = std::clamp(stepDelta, (double)cfg.deltaMin, (double)cfg.deltaMax);
            solutions.clear();   // they aimed for the old Delta
            mids.clear();
            bvpStatus.clear();
            paramsDirty = true;
        }
        ImGui::SliderFloat("alpha_min", &cfg.alphaMin, 0.0f, 1.5f, "%.2f rad");
        ImGui::SliderFloat("alpha_max", &cfg.alphaMax, 0.01f, 1.5f, "%.2f rad");
        ImGui::SliderFloat("(Delta alpha)_min", &cfg.dAlphaMin, 0.0f, 2.0f, "%.3f");
        ImGui::SliderFloat("(Delta alpha)_max", &cfg.dAlphaMax, 0.001f, 5.0f, "%.3f");
        cfg.alphaMin = std::min(cfg.alphaMin, cfg.alphaMax);
        cfg.dAlphaMin = std::min(cfg.dAlphaMin, cfg.dAlphaMax);
        ImGui::TextDisabled("Delta_i = %.4g   layer %d   eta = %.4g", stepDelta, layer, eta);
        if (growLeft > 0)
        {
            if (ImGui::Button("Stop growing"))
                stopGrow("stopped", -1);
            ImGui::SameLine();
            ImGui::TextDisabled("ring %d of %d", cfg.growRings - growLeft + 1, cfg.growRings);
        }
        else if (!edgeAngles.empty())
        {
            if (ImGui::Button("Grow rings"))
                startGrow();
            ImGui::SameLine();
            ImGui::SetNextItemWidth(120.0f);
            ImGui::SliderInt("##growRings", &cfg.growRings, 1, 200);
        }
        if (growWarnings > 0)
            ImGui::TextDisabled("%d ring(s) accepted at Delta_min over the angle bounds", growWarnings);

        sectionHeader("Particles");
        if (ImGui::Button(playing ? "Pause" : "Play"))
            playing = !playing;
        ImGui::SameLine();
        ImGui::TextDisabled("%s", engine::FlowKeys::kHint);
        ImGui::SliderFloat("speed", &cfg.particleSpeed, kMinParticleSpeed, kMaxParticleSpeed,
                           "%.6f", ImGuiSliderFlags_Logarithmic);
        ImGui::Checkbox("reverse", &reverse);
        float deg = glm::degrees(ballAngle);
        if (ImGui::SliderFloat("ball", &deg, 0.0f, 360.0f, "%.1f deg"))
            ballAngle = glm::radians(deg);
        if (ImGui::Button("Launch"))
            launchParticle();
        ImGui::SameLine();
        ImGui::BeginDisabled(edgeIndex < 0);
        if (ImGui::Button("Shoot r -> ball"))
            shootFromR();
        ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::SetNextItemWidth(110.0f);
        ImGui::SliderInt("##shootCount", &cfg.shootCount, 2, kMaxShoot, "%d particles");
        ImGui::SameLine();
        if (ImGui::Button("Clear particles"))
            particles.clear();
        ImGui::SameLine();
        ImGui::TextDisabled("(R)  %d", (int)particles.size());
        if (ImGui::Button("Whole edge"))
            shootRing();
        ImGui::SameLine();
        ImGui::TextDisabled("(B)");

        sectionHeader("Edge point");
        int j = edgeIndex;
        if (ImGui::SliderInt("r", &j, -1, (int)edgeAngles.size() - 1, j < 0 ? "none" : "vertex %d"))
        {
            if (j < 0)
                edgeIndex = -1;
            else
                chooseEdge(j);
        }
        if (edgeIndex >= 0)
        {
            if (ImGui::Button("Zoom to r"))
                zoomToEdge();
            ImGui::SameLine();
            if (ImGui::Button("Clear"))
                edgeIndex = -1;
            if (ImGui::Button("Solve geodesic BVP"))
                solveAtR();
        }
        if (!edgeAngles.empty())
        {
            if (edgeIndex >= 0)
                ImGui::SameLine();
            if (solveAllNext >= 0)
            {
                if (ImGui::Button("Stop"))
                    finishSolveAll();
                ImGui::SameLine();
                ImGui::TextDisabled("%d / %d", solveAllNext, (int)solveAllQueue.size());
            }
            else if (ImGui::Button("Solve all r"))
                startSolveAll();
        }
        if (ringComplete())
        {
            if (ImGui::Button("Accept ring"))
                acceptRing();
        }
        if (layer > 0)
        {
            ImGui::SameLine();
            ImGui::TextDisabled("layer %d", layer);
        }
        if (!solutions.empty())
        {
            if (ImGui::Button("Clear solutions"))
            {
                solutions.clear();
                mids.clear();
                bvpStatus.clear();
                paramsDirty = true;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("%d solved", (int)solutions.size());
        }
        if (!bvpStatus.empty())
            ImGui::TextDisabled("%s", bvpStatus.c_str());

        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.36f, 0.30f, 1.0f));
        bool showOther = ImGui::CollapsingHeader("Other settings (camera, look, Lorenz system)");
        ImGui::PopStyleColor();
        if (showOther)
        {
            ImGui::Indent();
            sectionHeader("Camera");
            ImGui::TextDisabled("A/D orbit, W/S tilt, Q/E zoom");
            ImGui::SliderFloat("orbit speed", &cfg.orbitSpeed, 0.1f, 4.0f, "%.2f rad/s");
            ImGui::SliderFloat("zoom speed", &cfg.zoomSpeed, 0.0f, 40.0f,
                               cfg.zoomSpeed > 0.0f ? "%.1f /s" : "auto");
            if (ImGui::Button("Frame the patch"))
                frameCamera(true);

            sectionHeader("Edge point");
            ImGui::SliderFloat("plane size", &cfg.planeScale, 0.5f, 20.0f, "%.1f x |r+ - r-|",
                               ImGuiSliderFlags_Logarithmic);
            ImGui::SliderFloat("plane opacity", &cfg.planeAlpha, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("zoom distance", &cfg.zoomScale, 0.5f, 20.0f, "%.1f x plane",
                               ImGuiSliderFlags_Logarithmic);

            sectionHeader("Particles");
            ImGui::SliderFloat("ball size", &cfg.ballSize, 2.0f, 60.0f, "%.0f px");
            ImGui::SliderFloat("particle size", &cfg.particleSize, 1.0f, 40.0f, "%.0f px");
            ImGui::SliderFloat("trail width", &cfg.particleTrail, 0.5f, 10.0f, "%.1f px");

            sectionHeader("Look");
            ImGui::SliderFloat("fill opacity", &cfg.fillAlpha, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("edge opacity", &cfg.edgeAlpha, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("edge width", &cfg.edgeWidth, 0.5f, 8.0f, "%.1f px");
            ImGui::SliderFloat("vertex size", &cfg.vertexSize, 1.0f, 30.0f, "%.0f px");
            ImGui::SliderFloat("boundary width", &cfg.boundaryWidth, 0.5f, 8.0f, "%.1f px");
            ImGui::SliderFloat("mesh fade", &cfg.meshFadeSeconds, 0.0f, 3.0f, "%.2f s");
            ImGui::SliderFloat("bead size", &cfg.eqRadius, 0.05f, 3.0f, "%.2f");
            ImGui::Checkbox("axes", &cfg.showAxes);

            sectionHeader("LaTeX");
            ImGui::Checkbox("labels", &cfg.showLabels);
            ImGui::SameLine();
            ImGui::Checkbox("parameters (top right)", &cfg.showParams);
            ImGui::SliderFloat("equilibrium", &cfg.labelSize, 10.0f, 80.0f, "%.0f px");
            ImGui::SliderFloat("W_loc", &cfg.patchLabelSize, 10.0f, 80.0f, "%.0f px");
            ImGui::SliderFloat("r, n, F_r", &cfg.edgePointLabelSize, 10.0f, 80.0f, "%.0f px");
            ImGui::SliderFloat("b_r(tau), distance", &cfg.hitLabelSize, 10.0f, 80.0f, "%.0f px");
            ImGui::SliderFloat("parameters", &cfg.paramsLatexSize, 10.0f, 80.0f, "%.0f px");

            sectionHeader("Lorenz system");
            bool changed = false;
            changed |= ImGui::SliderFloat("sigma", &cfg.sigma, 0.1f, 30.0f, "%.3f");
            changed |= ImGui::SliderFloat("rho", &cfg.rho, 0.1f, 60.0f, "%.3f");
            changed |= ImGui::SliderFloat("beta", &cfg.beta, 0.1f, 10.0f, "%.3f");
            changed |= ImGui::SliderInt("K order", &cfg.manifoldOrder, 2, 30);
            if (changed)
                rebuild();
            ImGui::Unindent();
        }

        sectionHeader("Settings");
        if (ImGui::Button("Save settings"))
            settingsStatus = saveCurrentSettings() ? "saved" : "could not write settings file";
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
    // ---- the patch ------------------------------------------------------------------

    // The field, the chosen equilibrium, K of both its sides and the radius they share
    // (the smaller trusted one, as lorenz_manifold_growth draws them); the patch is the
    // 2-D side — the one handed over, else whichever side is a surface.
    void rebuild()
    {
        system.sigma = cfg.sigma;
        system.rho = cfg.rho;
        system.beta = cfg.beta;
        std::vector<engine::LorenzSystem::Equilibrium> eqs = system.equilibria();
        eqTotal = (int)std::min<size_t>(eqs.size(), 3);
        error.clear();
        fillCount = edgeCount = pointCount = 0;
        found = eqTotal > 0;
        if (!found)
        {
            error = "no equilibria";
            return;
        }
        cfg.eq = std::clamp(cfg.eq, 0, eqTotal - 1);
        const auto& e = eqs[cfg.eq];
        eqPos = glm::vec3((float)e.x[0], (float)e.x[1], (float)e.x[2]);
        eqPosD = glm::dvec3(e.x[0], e.x[1], e.x[2]);
        eqTex = e.name == "O" ? "O" : e.name == "C+" ? "C_+" : "C_-";
        labelDirty = true;

        double J[3][3];
        system.jacobian(e.x, J);
        const auto Q = engine::LorenzSystem::quadratic();
        bool ok[2];
        for (int s = 0; s < 2; ++s)
            ok[s] = K[s].build(J, Q, s == 0 ? engine::InvariantManifold::Side::Stable
                                            : engine::InvariantManifold::Side::Unstable,
                               cfg.manifoldOrder);
        if (!(ok[side] && K[side].dim() == 2))
        {
            const int other = 1 - side;
            if (ok[other] && K[other].dim() == 2)
                side = other;
            else
            {
                error = "no 2-D local manifold here";
                return;
            }
        }
        radius = 1e30;
        for (int s = 0; s < 2; ++s)
            if (ok[s])
                radius = std::min(radius, trustedRadius(K[s]));
        buildMesh();
    }

    // Largest |theta| (bisection up to kRadiusSearchMax) where K's relative invariance
    // defect stays under kDefectOk all the way out (lorenz_manifold_growth's test).
    static double trustedRadius(const engine::InvariantManifold& K)
    {
        auto good = [&](double r) {
            for (int j = 1; j <= 8; ++j)
                if (K.maxRelDefect(r * j / 8.0, 48) >= kDefectOk)
                    return false;
            return true;
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

    // The mesh: x*, `rings` - 1 inner rings of `segs` vertices x* + K(theta) with theta on
    // circles of radius r i / rings in E, and the edge (theta at radius r) whose points
    // follow the method's spacing rule (spreadEdge). One buffer holds the triangles (a
    // fan around x*, two per quad between inner rings, the band between the last inner
    // ring and the edge merged by angle), their edges as line pairs (ring loops, spokes,
    // diagonals, the edge, the band's connectors), then every vertex as a point — the
    // edge's last, so they double as the boundary loop.
    void buildMesh()
    {
        fillCount = edgeCount = pointCount = 0;
        verts.clear();
        edgeAngles.clear();
        edgeCurve.clear();
        particles.clear();   // they started on the old patch
        solutions.clear();   // so did the BVP's
        mids.clear();
        clearLayers();
        ++meshVersion;
        if (!found || !error.empty())
            return;
        const engine::InvariantManifold& k = K[side];
        auto o0 = k.orthoBasis(0), o1 = k.orthoBasis(1);
        const glm::vec3 e0((float)o0[0], (float)o0[1], (float)o0[2]);
        const glm::vec3 e1((float)o1[0], (float)o1[1], (float)o1[2]);
        basis0 = e0;
        basis1 = e1;
        const int R = cfg.rings, S = cfg.segs;
        const float r = (float)radius;
        std::vector<glm::vec3>& pts = verts;
        pts.push_back(eqPos + onPatch(glm::vec3(0.0f)));
        for (int ring = 1; ring < R; ++ring)
            for (int j = 0; j < S; ++j)
            {
                const float a = 6.2831853f * (float)j / S;
                const float rr = r * (float)ring / R;
                pts.push_back(eqPos + onPatch(rr * (std::cos(a) * e0 + std::sin(a) * e1)));
            }
        std::vector<glm::vec3> edge;
        spreadEdge(edgeAngles, edge);
        const int E = (int)edge.size();
        pts.insert(pts.end(), edge.begin(), edge.end());
        edgeIndex = std::min(edgeIndex, E - 1);

        // vertex j of inner ring (0 = x*), of the edge; both wrap around
        auto at = [&](int ring, int j) -> const glm::vec3& {
            return ring == 0 ? pts[0] : pts[1 + (size_t)(ring - 1) * S + ((j % S + S) % S)];
        };
        auto ed = [&](int j) -> const glm::vec3& {
            return pts[1 + (size_t)(R - 1) * S + ((j % E + E) % E)];
        };

        std::vector<float> v;
        auto put = [&](const glm::vec3& p) { v.insert(v.end(), {p.x, p.y, p.z}); };
        std::vector<glm::vec3> bandLines;   // the band's connectors, added with the lines
        if (R == 1)
            for (int j = 0; j < E; ++j)
            {
                put(at(0, 0));
                put(ed(j));
                put(ed(j + 1));
                bandLines.insert(bandLines.end(), {at(0, 0), ed(j)});
            }
        else
        {
            for (int j = 0; j < S; ++j)
            {
                put(at(0, 0));
                put(at(1, j));
                put(at(1, j + 1));
            }
            for (int ring = 1; ring < R - 1; ++ring)
                for (int j = 0; j < S; ++j)
                {
                    put(at(ring, j));
                    put(at(ring + 1, j));
                    put(at(ring + 1, j + 1));
                    put(at(ring, j));
                    put(at(ring + 1, j + 1));
                    put(at(ring, j + 1));
                }
            // the band: both rings start at angle 0, walk them together by angle
            const int in = R - 1;
            auto innerAngle = [&](int i) { return 6.2831853f * (float)i / S; };
            auto edgeAngle = [&](int j) { return j < E ? edgeAngles[j] : 6.2831853f; };
            bandLines.insert(bandLines.end(), {at(in, 0), ed(0)});
            for (int i = 0, j = 0; i < S || j < E;)
            {
                if (j == E || (i < S && innerAngle(i + 1) <= edgeAngle(j + 1)))
                {
                    put(at(in, i));
                    put(at(in, i + 1));
                    put(ed(j));
                    bandLines.insert(bandLines.end(), {at(in, i + 1), ed(j)});
                    ++i;
                }
                else
                {
                    put(at(in, i));
                    put(ed(j + 1));
                    put(ed(j));
                    bandLines.insert(bandLines.end(), {at(in, i), ed(j + 1)});
                    ++j;
                }
            }
        }
        fillCount = (int)(v.size() / 3);
        for (int ring = 1; ring < R; ++ring)
            for (int j = 0; j < S; ++j)
            {
                put(at(ring, j));       // along the ring
                put(at(ring, j + 1));
                put(at(ring - 1, j));   // spoke inward
                put(at(ring, j));
                if (ring < R - 1)
                {
                    put(at(ring, j));   // the quad's diagonal
                    put(at(ring + 1, j + 1));
                }
            }
        for (int j = 0; j < E; ++j)
        {
            put(ed(j));                 // along the edge
            put(ed(j + 1));
        }
        for (const glm::vec3& p : bandLines)
            put(p);
        edgeCount = (int)(v.size() / 3) - fillCount;
        for (const glm::vec3& p : pts)
            put(p);
        pointCount = (int)pts.size();
        boundaryFirst = fillCount + edgeCount + 1 + (R - 1) * S;
        labelAnchor = eqPos + onPatch(0.92f * r * glm::normalize(e0 + e1));
        edgeCurve.resize(kEdgeSamples);
        for (int k = 0; k < kEdgeSamples; ++k)
            edgeCurve[k] = edgePoint(6.2831853f * (float)k / kEdgeSamples);
        patchMesh->setVertices(v.data(), std::min(v.size(), (size_t)kCapacity * 3));
        paramsDirty = true;
    }

    // The edge's points, by angle, under the geodesic-level-set spacing rule: start from
    // `segs` even angles; wherever two neighbours are farther apart than maxSpread a
    // point goes in midway (in angle, on the true edge), and a point closer than
    // minSpread to the one before it goes out — unless the gap that leaves would pass
    // maxSpread. Repeated until nothing changes. Angle 0 always stays (the band starts
    // there).
    void spreadEdge(std::vector<float>& angles, std::vector<glm::vec3>& points) const
    {
        struct Pt {
            float a;
            glm::vec3 p;
        };
        const float twoPi = 6.2831853f;
        const float maxS = cfg.maxSpread, minS = std::min(cfg.minSpread, 0.5f * cfg.maxSpread);
        std::vector<Pt> pts;
        for (int j = 0; j < cfg.segs; ++j)
        {
            const float a = twoPi * (float)j / cfg.segs;
            pts.push_back({a, edgePoint(a)});
        }
        for (int pass = 0; pass < 64; ++pass)
        {
            bool changed = false;
            std::vector<Pt> out;
            const int n = (int)pts.size();
            for (int i = 0; i < n; ++i)
            {
                out.push_back(pts[i]);
                const Pt& next = pts[(i + 1) % n];
                const float aNext = i + 1 == n ? next.a + twoPi : next.a;
                if (glm::length(next.p - pts[i].p) > maxS && (int)out.size() + (n - i) < kMaxEdge)
                {
                    const float am = 0.5f * (pts[i].a + aNext);
                    out.push_back({am, edgePoint(am)});
                    changed = true;
                }
            }
            pts.swap(out);
            out.clear();
            out.push_back(pts[0]);
            const int m = (int)pts.size();
            for (int i = 1; i < m; ++i)
            {
                const Pt& next = i + 1 < m ? pts[i + 1] : out[0];
                if (m - 1 > 3 && glm::length(pts[i].p - out.back().p) < minS &&
                    glm::length(next.p - out.back().p) <= maxS)
                {
                    changed = true;
                    continue;
                }
                out.push_back(pts[i]);
            }
            // the last point against the first (angle 0 stays)
            if (out.size() > 4 && glm::length(out.back().p - out[0].p) < minS &&
                glm::length(out[out.size() - 2].p - out[0].p) <= maxS)
            {
                out.pop_back();
                changed = true;
            }
            pts.swap(out);
            if (!changed)
                break;
        }
        angles.clear();
        points.clear();
        for (const Pt& q : pts)
        {
            angles.push_back(q.a);
            points.push_back(q.p);
        }
    }

    // maxSpread, minSpread top-right: which parameters the method runs with.
    void drawParamsOverlay()
    {
        if (!cfg.showParams)
            return;
        if (paramsDirty || paramsShownSize != cfg.paramsLatexSize ||
            paramsShownMax != cfg.maxSpread || paramsShownMin != cfg.minSpread ||
            paramsShownDMin != cfg.deltaMin || paramsShownDMax != cfg.deltaMax)
        {
            paramsDirty = false;
            paramsShownSize = cfg.paramsLatexSize;
            paramsShownMax = cfg.maxSpread;
            paramsShownMin = cfg.minSpread;
            paramsShownDMin = cfg.deltaMin;
            paramsShownDMax = cfg.deltaMax;
            std::string t = R"(\mathrm{maxSpread} = )" + texNumber(cfg.maxSpread) +
                            R"(,\quad \mathrm{minSpread} = )" + texNumber(cfg.minSpread) +
                            R"(,\quad \Delta_{\min} = )" + texNumber(cfg.deltaMin) +
                            R"(,\quad \Delta_{\max} = )" + texNumber(cfg.deltaMax);
            if (!solutions.empty())
            {
                char b[32];
                std::snprintf(b, sizeof(b), "%.4g", ringDelta);
                t += R"(,\quad \Delta_i = )" + std::string(b);
            }
            paramsLatex.set(t, cfg.paramsLatexSize);
        }
        if (paramsLatex.empty())
            return;
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12.0f, vp->WorkPos.y + 12.0f),
                                ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.35f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##levelSetParams", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
        paramsLatex.draw();
        ImGui::End();
        ImGui::PopStyleVar();
    }

    // K(theta) relative to x*, theta a point of E.
    glm::vec3 onPatch(const glm::vec3& theta) const
    {
        const engine::InvariantManifold& k = K[side];
        auto p = k.eval(k.toParam({theta.x, theta.y, theta.z}));
        return glm::vec3((float)p[0], (float)p[1], (float)p[2]);
    }

    // The edge at parameter a: x* + K(radius (cos a e0 + sin a e1)) at layer 0 — the
    // true curve, not the mesh's polygon — and the accepted ring's polyline after.
    glm::vec3 edgePoint(float a) const
    {
        if (layer > 0)
            return glm::vec3(edgePointD(a));
        return eqPos + onPatch((float)radius * (std::cos(a) * basis0 + std::sin(a) * basis1));
    }

    // ---- the geodesic BVP ---------------------------------------------------------------
    //
    // Krauskopf & Osinga 3.1. For an edge point r with its plane F_r (through r, normal
    // n = (r^+ - r^-)/2), the one-parameter family of BVPs in the field time tau (the
    // manifold's own direction of time): a start gamma(s) on the edge with
    //     G(s, tau) = n.(phi^tau(gamma(s)) - r) = 0,   b_r(tau) = phi^tau(gamma(s)).
    // From the trivial solution (s_r, 0) the branch s(tau) is continued in tau (unique
    // by the implicit function theorem, as dG/ds = n.gamma' != 0 there) until the first
    // tau with ||b_r(tau) - r|| = Delta. One branch: it goes to whichever side of r it
    // goes, or stays at r (the z-axis of W^s(O)). See math/geodesic-bvp.

    // The edge at parameter a, in double precision (the continuation steps get tiny).
    // Past layer 0 the edge is the accepted ring: linear between its points, by the
    // parameter each was given (arc length, scaled to 2 pi).
    glm::dvec3 edgePointD(double a) const
    {
        if (layer > 0)
        {
            int i, k;
            const double t = ringSpan(a, i, k);
            return glm::mix(ringPts[(size_t)i], ringPts[(size_t)k], t);
        }
        const glm::dvec3 th = radius * (std::cos(a) * glm::dvec3(basis0) + std::sin(a) * glm::dvec3(basis1));
        const engine::InvariantManifold& k = K[side];
        auto q = k.eval(k.toParam({th.x, th.y, th.z}));
        return eqPosD + glm::dvec3(q[0], q[1], q[2]);
    }

    // The accepted ring's stretch holding parameter a: from point i to point k, and how
    // far along it a sits (0..1).
    double ringSpan(double a, int& i, int& k) const
    {
        const int E = (int)ringParam.size();
        a = wrapAngle(a);
        i = (int)(std::upper_bound(ringParam.begin(), ringParam.end(), a) - ringParam.begin()) - 1;
        i = std::clamp(i, 0, E - 1);
        k = (i + 1) % E;
        const double a0 = ringParam[(size_t)i];
        const double a1 = i + 1 < E ? ringParam[(size_t)k] : 6.283185307179586;
        return a1 > a0 ? std::clamp((a - a0) / (a1 - a0), 0.0, 1.0) : 0.0;
    }

    // F's outward direction at edge parameter a, its part along nh (unit normal) removed:
    // K's radial direction at layer 0; past it, the direction the ring just grew in
    // (r_new minus its start on the previous edge), interpolated along the ring.
    glm::dvec3 outwardAt(double a, const glm::dvec3& nh) const
    {
        glm::dvec3 radial;
        if (layer > 0)
        {
            int i, k;
            const double t = ringSpan(a, i, k);
            radial = glm::mix(ringOut[(size_t)i], ringOut[(size_t)k], t);
        }
        else
        {
            const glm::dvec3 th = radius * (std::cos(a) * glm::dvec3(basis0) + std::sin(a) * glm::dvec3(basis1));
            const engine::InvariantManifold& k = K[side];
            auto q0 = k.eval(k.toParam({th.x, th.y, th.z}));
            auto q1 = k.eval(k.toParam({1.001 * th.x, 1.001 * th.y, 1.001 * th.z}));
            radial = glm::dvec3(q1[0] - q0[0], q1[1] - q0[1], q1[2] - q0[2]);
        }
        radial -= glm::dot(radial, nh) * nh;
        return glm::length(radial) > 1e-15 ? glm::normalize(radial) : radial;
    }

    // Where a BVP is posed: the point r (angle on the edge), F_r's normal and outward
    // direction, and how far the edge runs to the neighbours on each side.
    struct Anchor {
        double angle = 0.0;
        glm::dvec3 r{0.0}, n{0.0}, out{0.0};
        double gapPlus = 0.0, gapMinus = 0.0;   // angle to the neighbour (r^+, r^-)
    };

    static double wrapAngle(double a)
    {
        a = std::fmod(a, 6.283185307179586);
        return a < 0.0 ? a + 6.283185307179586 : a;
    }

    // The anchor at angle a between neighbours at aMinus and aPlus (F's normal from them).
    Anchor makeAnchor(double a, double aMinus, double aPlus) const
    {
        Anchor A;
        A.angle = a;
        A.r = edgePointD(a);
        A.n = 0.5 * (edgePointD(aPlus) - edgePointD(aMinus));
        A.gapPlus = wrapAngle(aPlus - a);
        A.gapMinus = wrapAngle(a - aMinus);
        A.out = outwardAt(a, glm::normalize(A.n));
        return A;
    }

    // phi^t(gamma(a)) for signed field time t: hand-stepped Taylor, the last step cut to
    // land exactly at t. `orbit` (optional) gets the path for drawing, steps capped at
    // `drawStep` world units so it looks smooth. False if it blows up.
    bool flowFrom(double a, double t, glm::dvec3& out, std::vector<glm::vec3>* orbit = nullptr,
                  double drawStep = 0.0) const
    {
        const glm::dvec3 s0 = edgePointD(a);
        double x[3] = {s0.x, s0.y, s0.z};
        if (orbit)
            orbit->push_back(glm::vec3(s0));
        const int order = taylor.order;
        std::vector<double> c((size_t)(order + 1) * 3);
        const double dir = t < 0.0 ? -1.0 : 1.0, total = std::abs(t);
        double done = 0.0;
        for (int step = 0; step < 20000 && done < total; ++step)
        {
            jet(x, dir * done, order, c.data());
            double h = std::min(taylor.stepSize(c.data(), 3), total - done);
            if (orbit && drawStep > 0.0)
            {
                const double speed = std::sqrt(c[3] * c[3] + c[4] * c[4] + c[5] * c[5]);
                h = std::min(h, drawStep / std::max(speed, 1e-12));
            }
            double x1[3];
            engine::TaylorIntegrator::evaluate(c.data(), 3, order, dir * h, x1);
            x[0] = x1[0];
            x[1] = x1[1];
            x[2] = x1[2];
            done = h >= total - done ? total : done + h;
            if (!std::isfinite(x[0] + x[1] + x[2]) || std::abs(x[0]) + std::abs(x[1]) + std::abs(x[2]) > 1e8)
                return false;
            if (orbit)
                orbit->push_back(glm::vec3((float)x[0], (float)x[1], (float)x[2]));
        }
        out = glm::dvec3(x[0], x[1], x[2]);
        return done >= total;
    }

    // One point of a branch: the start a on the edge (unwrapped, so the branch is
    // continuous), d = ||b - r||, tau (signed field time), b in F_r.
    struct Sample {
        double a = 0.0, d = 0.0, tau = 0.0;
        glm::dvec3 b{0.0};
    };

    // The BVP at |tau| = T: the start s with G(s, T) = 0, by secant from sGuess (the
    // predictor). False if it does not converge or slides more than two neighbour gaps.
    bool correct(const Anchor& A, double T, double sGuess, Sample& out) const
    {
        const glm::dvec3 nh = glm::normalize(A.n);
        const double t = (side == 0 ? -1.0 : 1.0) * T;
        const double gap = std::max(A.gapPlus, A.gapMinus);
        const double tol = 1e-12 * (1.0 + glm::length(A.n));
        auto G = [&](double sv, glm::dvec3& b) {
            return flowFrom(sv, t, b) ? glm::dot(b - A.r, nh) : std::nan("");
        };
        double s0 = sGuess, s1 = sGuess + 1e-7 * gap;
        glm::dvec3 b0, b1;
        double g0 = G(s0, b0);
        if (std::abs(g0) < tol)
        {
            out = {s0, glm::length(b0 - A.r), t, b0};
            return true;
        }
        double g1 = G(s1, b1);
        for (int it = 0; it < 40; ++it)
        {
            if (!std::isfinite(g0) || !std::isfinite(g1))
                return false;
            if (std::abs(g1) < tol)
            {
                out = {s1, glm::length(b1 - A.r), t, b1};
                return true;
            }
            if (g1 == g0)
                return false;
            const double s2 = s1 - g1 * (s1 - s0) / (g1 - g0);
            if (std::abs(s2 - sGuess) > 2.0 * gap)
                return false;
            s0 = s1;
            g0 = g1;
            s1 = s2;
            g1 = G(s1, b1);
        }
        return false;
    }

    // The point of the branch between lo and hi (|tau| increasing, d crossing D) where
    // d = D: bisection on |tau|, the start predicted linearly between the two.
    Sample bisectTau(const Anchor& A, Sample lo, Sample hi, double D) const
    {
        for (int it = 0; it < 60; ++it)
        {
            const double T0 = std::abs(lo.tau), T1 = std::abs(hi.tau), Tm = 0.5 * (T0 + T1);
            Sample M;
            if (!correct(A, Tm, 0.5 * (lo.a + hi.a), M))
                break;
            if (std::abs(M.d - D) < 1e-10 * std::max(D, 1.0))
                return M;
            (M.d < D ? lo : hi) = M;
        }
        return hi;
    }

    // The continued branch: its points, whether ||b - r|| reached the target, and the
    // farthest it got.
    struct Branch {
        std::vector<Sample> path;
        bool reached = false;
        double best = 0.0;
    };

    // Continue s(tau) from (s_r, 0) in tau: predictor along the last slope ds/dtau,
    // corrector at the new tau. A step is kept only if it converges and b moved
    // continuously (at most 0.15 target); else it halves. Stops at the first tau with
    // ||b - r|| = target (bisected), or where the step shrinks to nothing (broke).
    Branch continueTau(const Anchor& A, double target) const
    {
        Branch B;
        B.path.push_back({A.angle, 0.0, 0.0, A.r});
        double f[3];
        const double r3[3] = {A.r.x, A.r.y, A.r.z};
        system.eval(r3, f);
        const double speed = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
        double T = 0.0, dT = 0.02 * target / std::max(speed, 1e-12), slope = 0.0;
        const double minStep = 1e-9 * dT;
        for (int it = 0; it < 600; ++it)
        {
            const Sample last = B.path.back();
            Sample S;
            if (!correct(A, T + dT, last.a + slope * dT, S) || glm::length(S.b - last.b) > 0.15 * target)
            {
                dT *= 0.5;
                if (dT < minStep)
                    break;
                continue;
            }
            if (S.d >= target)
            {
                const Sample hit = bisectTau(A, last, S, target);
                B.path.push_back(hit);
                B.best = std::max(B.best, hit.d);
                B.reached = true;
                break;
            }
            slope = (S.a - last.a) / dT;
            B.path.push_back(S);
            B.best = std::max(B.best, S.d);
            T += dT;
            const double dd = S.d - last.d;
            dT *= dd > 1e-12 ? std::clamp(0.05 * target / dd, 0.5, 2.0) : 1.5;
        }
        return B;
    }

    // The point of a branch at distance D (<= its best): the first stretch of the path
    // that crosses D, bisected in tau.
    Sample pointAt(const Branch& B, const Anchor& A, double D) const
    {
        if (D <= 0.0 || B.path.size() < 2)
            return B.path.front();
        for (size_t k = 1; k < B.path.size(); ++k)
            if (B.path[k].d >= D)
                return std::abs(B.path[k].d - D) < 1e-10 * std::max(D, 1.0)
                           ? B.path[k]
                           : bisectTau(A, B.path[k - 1], B.path[k], D);
        return B.path.back();
    }

    // One solved point: its anchor, the branch it was continued along, and where it
    // sits at the ring's Delta (r_new, its start on the edge, tau, the orbit drawn).
    struct Solution {
        int edge = -1;          // edge vertex (for the main points)
        Anchor A;
        Branch branch;
        double achieved = 0.0;  // target if reached, else the farthest it got
        glm::dvec3 point{0.0};  // r_new
        double tau = 0.0, start = 0.0;
        std::vector<glm::vec3> orbit;
    };

    // Solve at an anchor: the branch continued out to target, or as far as it goes.
    // False when it could not take a single step.
    bool solveAnchor(const Anchor& A, double target, Solution& S) const
    {
        Branch B = continueTau(A, target);
        if (B.path.size() < 2)
            return false;
        S.A = A;
        S.branch = std::move(B);
        S.achieved = S.branch.reached ? target : S.branch.best;
        return true;
    }

    // "Solve geodesic BVP" at the chosen r.
    void solveAtR()
    {
        if (edgeIndex >= 0)
            solveAt(edgeIndex);
    }

    // Solve at edge vertex j, then re-cut the whole ring.
    void solveAt(int j)
    {
        if (j < 0 || j >= (int)edgeAngles.size())
            return;
        if (!solveVertex(j))
        {
            bvpStatus = "no solution: the branch could not be continued from r";
            return;
        }
        std::sort(solutions.begin(), solutions.end(),
                  [](const Solution& a, const Solution& b) { return a.edge < b.edge; });
        recutRing();
    }

    // The BVP at edge vertex j into `solutions` (replacing its old one); no re-cut.
    bool solveVertex(int j)
    {
        const int E = (int)edgeAngles.size();
        const Anchor A = makeAnchor(edgeAngles[(size_t)j], edgeAngles[(size_t)((j - 1 + E) % E)],
                                    edgeAngles[(size_t)((j + 1) % E)]);
        Solution S;
        if (!solveAnchor(A, stepDelta, S))
            return false;
        S.edge = j;
        solutions.erase(std::remove_if(solutions.begin(), solutions.end(),
                                       [&](const Solution& q) { return q.edge == j; }),
                        solutions.end());
        solutions.push_back(std::move(S));
        lastSolved = j;
        return true;
    }

    // "Solve all r": every edge vertex not solved yet, going on from the one after the
    // last solved (so after solving one by hand it carries on from there), a slice per
    // frame (kSolveAllBudgetMs) so the window stays live, then one re-cut. A new mesh
    // cancels it (its vertices are gone).
    void startSolveAll()
    {
        const int E = (int)edgeAngles.size();
        if (E == 0)
            return;
        solveAllQueue.clear();
        const int from = lastSolved >= 0 && lastSolved < E ? lastSolved + 1 : 0;
        for (int i = 0; i < E; ++i)
        {
            const int j = (from + i) % E;
            if (!solutionAt(j))
                solveAllQueue.push_back(j);
        }
        if (solveAllQueue.empty())
            return;
        solveAllNext = 0;
        solveAllVersion = meshVersion;
    }

    void stepSolveAll()
    {
        if (solveAllNext < 0)
            return;
        if (solveAllVersion != meshVersion)
        {
            solveAllNext = -1;
            return;
        }
        const auto start = std::chrono::steady_clock::now();
        const int n = (int)solveAllQueue.size();
        while (solveAllNext < n)
        {
            solveVertex(solveAllQueue[(size_t)solveAllNext++]);
            if (std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - start).count() >
                kSolveAllBudgetMs)
                break;
        }
        if (solveAllNext >= n)
            finishSolveAll();
    }

    // Done or stopped: what was solved so far, sorted and re-cut.
    void finishSolveAll()
    {
        const bool done = solveAllNext >= (int)solveAllQueue.size();
        solveAllNext = -1;
        std::sort(solutions.begin(), solutions.end(),
                  [](const Solution& a, const Solution& b) { return a.edge < b.edge; });
        if (!solutions.empty())
            recutRing();
        if (growLeft > 0 && done)
            growStep();
    }

    // edge vertex -> index into `solutions` (-1: not solved)
    std::vector<int> solutionIndex() const
    {
        std::vector<int> at(edgeAngles.size(), -1);
        for (size_t i = 0; i < solutions.size(); ++i)
            if (solutions[i].edge >= 0 && solutions[i].edge < (int)at.size())
                at[(size_t)solutions[i].edge] = (int)i;
        return at;
    }

    // ---- Delta_i adaptation (3.2) and the grow run --------------------------------------

    // alpha_r for a solved edge vertex: the angle at r between p_r -> r (p_r the point of
    // the previous ring whose BVP gave r; at layer 0, K's radial direction) and r -> b_r.
    double alphaOf(const Solution& q) const
    {
        glm::dvec3 in;
        if (layer > 0 && !ringPrev.empty())
        {
            int i, k;
            const double t = ringSpan(q.A.angle, i, k);
            in = q.A.r - glm::mix(ringPrev[(size_t)i], ringPrev[(size_t)k], t);
        }
        else
            in = outwardAt(q.A.angle, glm::normalize(q.A.n));
        const glm::dvec3 step = q.point - q.A.r;
        const double l = glm::length(in) * glm::length(step);
        return l > 0.0 ? std::acos(std::clamp(glm::dot(in, step) / l, -1.0, 1.0)) : 0.0;
    }

    // The ring against the bounds: the largest alpha_r and Delta_i alpha_r, where, and
    // whether the ring passes (ok) / is well inside (small, so Delta_i may double).
    struct RingCheck {
        bool ok = true, small = true;
        double maxAlpha = 0.0, maxDA = 0.0;
        int worst = -1, shortest = -1;
        int reversed = 0, reversedAt = -1;   // new segments against their old ones
    };

    RingCheck checkRing() const
    {
        RingCheck c;
        double least = 1e300;
        for (const Solution& q : solutions)
        {
            const double a = alphaOf(q), da = ringDelta * a;
            if (a > c.maxAlpha)
            {
                c.maxAlpha = a;
                c.worst = q.edge;
            }
            c.maxDA = std::max(c.maxDA, da);
            if (a >= cfg.alphaMax || da >= cfg.dAlphaMax)
                c.ok = false;
            if (a >= cfg.alphaMin || da >= cfg.dAlphaMin)
                c.small = false;
            if (q.achieved < least)
            {
                least = q.achieved;
                c.shortest = q.edge;
            }
        }
        // order (3.3): walking the ring, each new segment must run the way its old one
        // did; one that points back means two points passed each other in this step (a
        // concave stretch of the ring where the planes F_r converge within Delta_i)
        std::vector<const Solution*> seq;
        for (const Solution& q : solutions)   // sorted by edge vertex
        {
            seq.push_back(&q);
            auto m = mids.find(q.edge);
            if (m != mids.end())
                seq.push_back(&m->second);
        }
        const int n = (int)seq.size();
        for (int i = 0; n > 3 && i < n; ++i)
        {
            const Solution& a = *seq[(size_t)i];
            const Solution& b = *seq[(size_t)((i + 1) % n)];
            if (glm::dot(b.point - a.point, b.A.r - a.A.r) <= 0.0)
            {
                ++c.reversed;
                if (c.reversedAt < 0)
                    c.reversedAt = a.edge >= 0 ? a.edge : seq[(size_t)((i + n - 1) % n)]->edge;
            }
        }
        if (c.reversed > 0)
            c.ok = c.small = false;
        return c;
    }

    // "Grow rings": growRings rings, each solved (Solve all, a slice per frame) and
    // checked: halved while the angles fail and Delta_i > Delta_min (the branches are
    // kept, only the cut moves), accepted — with a warning if still failing at
    // Delta_min — and the next Delta_i set by Accept ring. Stops at a vertex with no
    // solution or a branch that broke below Delta_min, and opens that vertex.
    void startGrow()
    {
        if (edgeAngles.empty())
            return;
        growLeft = cfg.growRings;
        growWarnings = 0;
        if (ringComplete())
            growStep();
        else
            startSolveAll();
    }

    void growStep()
    {
        const int E = (int)edgeAngles.size();
        for (int j = 0; j < E; ++j)
            if (!solutionAt(j))
            {
                stopGrow("no solution: the branch could not be continued from this vertex", j);
                return;
            }
        RingCheck c = checkRing();
        while (!c.ok && stepDelta > cfg.deltaMin * (1.0 + 1e-9))
        {
            stepDelta = std::max(0.5 * stepDelta, (double)cfg.deltaMin);
            recutRing();
            c = checkRing();
        }
        // a branch that broke at (or under) Delta_min drags the whole ring down to it
        if (ringDelta <= cfg.deltaMin * (1.0 + 1e-9) && ringDelta < stepDelta * (1.0 - 1e-9))
        {
            char b[128];
            std::snprintf(b, sizeof(b), "branch broke at ||b - r|| = %.4g (Delta_min %.4g)", ringDelta,
                          (double)cfg.deltaMin);
            stopGrow(b, c.shortest);
            return;
        }
        // a point that grows backward (alpha_r past 90 degrees): the ring folds over
        if (c.maxAlpha > 1.5707963)
        {
            char b[128];
            std::snprintf(b, sizeof(b), "r_new grows backward: alpha_r = %.3g at Delta_i = %.4g", c.maxAlpha,
                          ringDelta);
            stopGrow(b, c.worst);
            return;
        }
        if (c.reversed > 0)
        {
            char b[128];
            std::snprintf(b, sizeof(b), "ring folds: %d new segment(s) reversed even at Delta_i = %.4g",
                          c.reversed, ringDelta);
            stopGrow(b, c.reversedAt);
            return;
        }
        if (!c.ok)
            ++growWarnings;
        acceptRing();
        if (--growLeft > 0)
            startSolveAll();
    }

    // End a grow run; with a vertex, select it (zoom in) so what went wrong is in view.
    void stopGrow(const std::string& why, int vertex)
    {
        growLeft = 0;
        if (solveAllNext >= 0)
            finishSolveAll();
        bvpStatus = why;
        if (vertex >= 0 && vertex < (int)edgeAngles.size())
            chooseEdge(vertex);
    }


    Solution* solutionAt(int j)
    {
        for (Solution& q : solutions)
            if (q.edge == j)
                return &q;
        return nullptr;
    }

    // The ring: Delta_i = the smallest distance any of its points reached (so they all
    // sit at the same geodesic distance), every point re-cut to it; between neighbours
    // whose new points are more than maxSpread apart, the edge point midway is solved
    // too (and counts toward the minimum). Repeated until the mids settle.
    void recutRing()
    {
        const int E = (int)edgeAngles.size();
        for (int pass = 0; pass < 4; ++pass)
        {
            ringDelta = stepDelta;
            for (const Solution& q : solutions)
                ringDelta = std::min(ringDelta, q.achieved);
            for (const auto& m : mids)
                ringDelta = std::min(ringDelta, m.second.achieved);
            auto place = [&](Solution& q) {
                const Sample p = pointAt(q.branch, q.A, ringDelta);
                q.point = p.b;
                q.tau = p.tau;
                q.start = p.a;
            };
            for (Solution& q : solutions)
                place(q);
            for (auto& m : mids)
                place(m.second);
            bool added = false;
            const std::vector<int> at = solutionIndex();
            for (const Solution& q : solutions)
            {
                const int j = q.edge, k = (j + 1) % E;
                const Solution* next = at[(size_t)k] >= 0 ? &solutions[(size_t)at[(size_t)k]] : nullptr;
                if (!next || mids.count(j) || glm::length(next->point - q.point) <= cfg.maxSpread)
                    continue;
                const double a = edgeAngles[(size_t)j];
                const double am = wrapAngle(a + 0.5 * wrapAngle(edgeAngles[(size_t)k] - a));
                Solution M;
                if (solveAnchor(makeAnchor(am, a, edgeAngles[(size_t)k]), stepDelta, M))
                {
                    M.edge = -1;
                    mids[j] = std::move(M);
                    added = true;
                }
            }
            if (!added)
                break;
        }
        // drop the mids whose pair lost a point
        {
            const std::vector<int> at = solutionIndex();
            for (auto it = mids.begin(); it != mids.end();)
            {
                if (at[(size_t)it->first] < 0 || at[(size_t)((it->first + 1) % E)] < 0)
                    it = mids.erase(it);
                else
                    ++it;
            }
        }
        for (Solution& q : solutions)
        {
            q.orbit.clear();
            glm::dvec3 end;
            flowFrom(q.start, q.tau, end, &q.orbit, 0.02 * cfg.deltaMax);
        }
        for (auto& m : mids)
        {
            m.second.orbit.clear();
            glm::dvec3 end;
            flowFrom(m.second.start, m.second.tau, end, &m.second.orbit, 0.02 * cfg.deltaMax);
        }
        char b[160];
        const Solution* cur = nullptr;
        for (const Solution& q : solutions)
            if (q.edge == edgeIndex)
                cur = &q;
        const RingCheck c = checkRing();
        if (cur)
            std::snprintf(b, sizeof(b), "r_new: tau = %.5g, ||b - r|| = %.5g%s, alpha_r = %.3g; Delta_i = %.4g%s",
                          cur->tau, glm::length(cur->point - cur->A.r),
                          cur->branch.reached ? "" : " (broke)", alphaOf(*cur), ringDelta,
                          ringDelta < cfg.deltaMin ? " < Delta_min" : "");
        else
            std::snprintf(b, sizeof(b), "Delta_i = %.4g, max alpha_r = %.3g (vertex %d), max Delta alpha = %.3g",
                          ringDelta, c.maxAlpha, c.worst, c.maxDA);
        bvpStatus = b;
        paramsDirty = true;
    }

    // The new band: for neighbouring solved edge points r, r^+ the triangles
    // r -> r^+ -> r_new and r^+ -> r^+_new -> r_new (through the mid point when one was
    // needed); the band's lines; every r_new; the current r's orbit to its r_new.
    void renderBand(float px)
    {
        if (solutions.empty())
            return;
        std::vector<float> v;
        auto put = [&](const glm::dvec3& p) { v.insert(v.end(), {(float)p.x, (float)p.y, (float)p.z}); };
        std::vector<glm::dvec3> lines;
        bandGeometry(v, lines);
        const int fill = (int)(v.size() / 3);
        for (const Solution& q : solutions)   // each r -> r_new
            lines.insert(lines.end(), {q.A.r, q.point});
        for (const glm::dvec3& p : lines)
            put(p);
        const int lineCount = (int)lines.size();
        const int pointsFirst = (int)(v.size() / 3);
        for (const Solution& q : solutions)
            put(q.point);
        for (const auto& m : mids)
            put(m.second.point);
        const int pointCount = (int)(v.size() / 3) - pointsFirst;
        const int orbitFirst = (int)(v.size() / 3);
        int orbitCount = 0;
        for (const Solution& q : solutions)
            if (q.edge == edgeIndex)
                for (const glm::vec3& p : q.orbit)
                {
                    v.insert(v.end(), {p.x, p.y, p.z});
                    ++orbitCount;
                }
        bandMesh->setVertices(v.data(), std::min(v.size(), (size_t)kBandCapacity * 3));

        shader.setVec3("u_color", kNewColor);
        shader.setFloat("u_alpha", 0.28f);
        bandMesh->drawArrays(GL_TRIANGLES, fill, 0);
        shader.setFloat("u_alpha", 0.75f);
        glLineWidth(1.4f * px);
        bandMesh->drawArrays(GL_LINES, lineCount, fill);
        if (orbitCount > 1)
        {
            shader.setFloat("u_alpha", 0.9f);
            glLineWidth(1.8f * px);
            bandMesh->drawArrays(GL_LINE_STRIP, orbitCount, orbitFirst);
        }
        shader.setInt("u_round", 1);
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_pointSize", 1.7f * cfg.vertexSize * px);
        bandMesh->drawArrays(GL_POINTS, pointCount, pointsFirst);
        shader.setInt("u_round", 0);
    }

    // The band's triangles (into tri, as floats) and lines (pairs): for neighbouring
    // solved edge points r, r^+ the triangles r -> r^+ -> r_new and r^+ -> r^+_new ->
    // r_new, through the mid point when one was needed.
    void bandGeometry(std::vector<float>& tri, std::vector<glm::dvec3>& lines) const
    {
        const int E = (int)edgeAngles.size();
        auto put = [&](const glm::dvec3& p) { tri.insert(tri.end(), {(float)p.x, (float)p.y, (float)p.z}); };
        const std::vector<int> at = solutionIndex();
        for (const Solution& q : solutions)
        {
            const int k = (q.edge + 1) % E;
            if (at[(size_t)k] < 0)
                continue;
            const Solution* next = &solutions[(size_t)at[(size_t)k]];
            const glm::dvec3 r = q.A.r, rp = next->A.r, rn = q.point, rpn = next->point;
            auto mid = mids.find(q.edge);
            if (mid == mids.end())
            {
                for (const glm::dvec3& p : {r, rp, rn, rp, rpn, rn})
                    put(p);
                lines.insert(lines.end(), {rn, rpn, rp, rn});
            }
            else
            {
                const glm::dvec3 m = mid->second.point;
                for (const glm::dvec3& p : {r, rp, m, r, m, rn, rp, rpn, m})
                    put(p);
                lines.insert(lines.end(), {rn, m, m, rpn, r, m, rp, m});
            }
        }
    }

    // ---- accepted rings: the edge moves out ---------------------------------------------

    // Every edge vertex solved (and no Solve all running): the ring can become the edge.
    bool ringComplete() const
    {
        return solveAllNext < 0 && !edgeAngles.empty() && solutions.size() == edgeAngles.size();
    }

    // "Accept ring": the solved ring becomes the edge. Its points (every r_new, the mids
    // between them) in order, each with the direction it grew in (r_new minus its start
    // on the old edge); re-spread by the maxSpread / minSpread rule (a point closer than
    // minSpread to the one before goes, unless the gap left would pass maxSpread; a gap
    // still over maxSpread gets the midpoint of the ring's polyline); a parameter by arc
    // length scaled to 2 pi. The band between the old edge and the ring joins the
    // accepted surface (layerTri / layerLines).
    void acceptRing()
    {
        if (!ringComplete())
            return;
        // the next Delta_i (3.2): doubled if every alpha_r is well inside the bounds
        const RingCheck c = checkRing();
        const double usedDelta = ringDelta;
        struct Pt {
            glm::dvec3 p, out, prev;   // r_new, the direction it grew in, its p_r
        };
        std::vector<Pt> pts;
        auto grown = [&](const Solution& q) { return Pt{q.point, q.point - edgePointD(q.start), q.A.r}; };
        for (const Solution& q : solutions)   // sorted by edge vertex
        {
            pts.push_back(grown(q));
            auto m = mids.find(q.edge);
            if (m != mids.end())
                pts.push_back(grown(m->second));
        }
        const double maxS = cfg.maxSpread, minS = std::min(cfg.minSpread, 0.5f * cfg.maxSpread);
        for (int pass = 0; pass < 64; ++pass)
        {
            bool changed = false;
            // a pair closer than minSpread loses one point: the one whose removal leaves
            // the smaller gap (never one over maxSpread). Cyclic and symmetric — any
            // point may go — so no point of the ring is privileged: a fixed survivor at
            // index 0 bent the ring there, ring after ring, in sheared stretches.
            for (size_t i = 0; pts.size() > 4 && i < pts.size();)
            {
                const size_t m = pts.size(), j = (i + 1) % m;
                if (glm::length(pts[j].p - pts[i].p) >= minS)
                {
                    ++i;
                    continue;
                }
                const double gapDropI = glm::length(pts[j].p - pts[(i + m - 1) % m].p);
                const double gapDropJ = glm::length(pts[(j + 1) % m].p - pts[i].p);
                size_t drop = gapDropI <= gapDropJ ? i : j;
                if (std::min(gapDropI, gapDropJ) > maxS)
                {
                    ++i;   // either removal would open a gap over maxSpread: keep both
                    continue;
                }
                pts.erase(pts.begin() + (std::ptrdiff_t)drop);
                changed = true;
                if (drop < i)
                    --i;   // stay on the pair that now starts at i
            }
            std::vector<Pt> out;
            const int n = (int)pts.size();
            for (int i = 0; i < n; ++i)
            {
                out.push_back(pts[(size_t)i]);
                const Pt& next = pts[(size_t)((i + 1) % n)];
                if (glm::length(next.p - pts[(size_t)i].p) > maxS && (int)out.size() + (n - i) < kMaxEdge)
                {
                    out.push_back({0.5 * (pts[(size_t)i].p + next.p), 0.5 * (pts[(size_t)i].out + next.out),
                                   0.5 * (pts[(size_t)i].prev + next.prev)});
                    ++linearInserts;
                    changed = true;
                }
            }
            pts.swap(out);
            if (!changed)
                break;
        }

        // the band joins the accepted surface, the old edge's loop with its lines
        std::vector<float> tri;
        std::vector<glm::dvec3> lines;
        bandGeometry(tri, lines);
        if ((layerTri.size() + tri.size()) / 3 + layerLines.size() + lines.size() + 3 * pts.size() >
            (size_t)kLayerCapacity)
        {
            bvpStatus = "accepted surface is full";
            return;
        }
        layerTri.insert(layerTri.end(), tri.begin(), tri.end());
        layerLines.insert(layerLines.end(), lines.begin(), lines.end());

        // the ring as the new edge: parameter by arc length, scaled to 2 pi
        const int n = (int)pts.size();
        std::vector<double> cum(n + 1, 0.0);
        for (int i = 0; i < n; ++i)
            cum[(size_t)i + 1] = cum[(size_t)i] + glm::length(pts[(size_t)((i + 1) % n)].p - pts[(size_t)i].p);
        ringPts.clear();
        ringOut.clear();
        ringPrev.clear();
        ringParam.clear();
        edgeAngles.clear();
        for (int i = 0; i < n; ++i)
        {
            // stored as the float the edge keeps, so edgePointD(edgeAngles[j]) is point j
            const float a = (float)(6.283185307179586 * cum[(size_t)i] / std::max(cum[(size_t)n], 1e-300));
            edgeAngles.push_back(a);
            ringParam.push_back((double)a);
            ringPts.push_back(pts[(size_t)i].p);
            ringOut.push_back(pts[(size_t)i].out);
            ringPrev.push_back(pts[(size_t)i].prev);
        }
        ++layer;
        eta += usedDelta;
        stepDelta = std::clamp(c.small ? 2.0 * usedDelta : usedDelta, (double)cfg.deltaMin, (double)cfg.deltaMax);
        solutions.clear();
        mids.clear();
        lastSolved = -1;
        ++meshVersion;   // hits and a running batch belonged to the old edge
        edgeIndex = std::min(edgeIndex, n - 1);
        for (int k = 0; k < kEdgeSamples; ++k)
            edgeCurve[(size_t)k] = edgePoint(6.2831853f * (float)k / kEdgeSamples);
        uploadLayers();
        char b[96];
        std::snprintf(b, sizeof(b), "layer %d: %d edge points, eta = %.4g, max alpha_r %.3g; next Delta_i = %.4g",
                      layer, n, eta, c.maxAlpha, stepDelta);
        bvpStatus = b;
        paramsDirty = true;
    }

    void clearLayers()
    {
        layer = 0;
        eta = 0.0;
        stepDelta = cfg.deltaMax;
        growLeft = 0;
        linearInserts = 0;
        ringPrev.clear();
        layerTri.clear();
        layerLines.clear();
        ringPts.clear();
        ringOut.clear();
        ringParam.clear();
        layerFill = layerLineCount = 0;
    }

    // The layer mesh: accepted bands' triangles, their lines, then the current edge as a
    // loop (its points double as the vertex dots).
    void uploadLayers()
    {
        std::vector<float> v = layerTri;
        layerFill = (int)(v.size() / 3);
        for (const glm::dvec3& p : layerLines)
            v.insert(v.end(), {(float)p.x, (float)p.y, (float)p.z});
        layerLineCount = (int)layerLines.size();
        for (const glm::dvec3& p : ringPts)
            v.insert(v.end(), {(float)p.x, (float)p.y, (float)p.z});
        layerMesh->setVertices(v.data(), std::min(v.size(), (size_t)kLayerCapacity * 3));
    }

    // The accepted surface as the patch is drawn: fill, lines (with the mesh), the edge
    // loop, its vertices (with the mesh).
    void renderLayers(const glm::vec3& color, float px)
    {
        shader.setVec3("u_color", color);
        shader.setFloat("u_alpha", cfg.fillAlpha);
        layerMesh->drawArrays(GL_TRIANGLES, layerFill, 0);
        if (meshFade > 0.0f)
        {
            shader.setVec3("u_color", kEdgeColor);
            shader.setFloat("u_alpha", cfg.edgeAlpha * meshFade);
            glLineWidth(cfg.edgeWidth * px);
            layerMesh->drawArrays(GL_LINES, layerLineCount, layerFill);
        }
        const int edgeFirst = layerFill + layerLineCount, E = (int)ringPts.size();
        shader.setVec3("u_color", color);
        shader.setFloat("u_alpha", 0.9f);
        glLineWidth(cfg.boundaryWidth * px);
        layerMesh->drawArrays(GL_LINE_LOOP, E, edgeFirst);
        if (meshFade > 0.0f)
        {
            shader.setVec3("u_color", kVertexColor);
            shader.setFloat("u_alpha", meshFade);
            shader.setFloat("u_pointSize", cfg.vertexSize * px);
            shader.setInt("u_round", 1);
            layerMesh->drawArrays(GL_POINTS, E, edgeFirst);
            shader.setInt("u_round", 0);
        }
    }

    // r_new beside the current r's solution, away from r on screen.
    void drawNewLabel()
    {
        if (!cfg.showLabels)
            return;
        const Solution* cur = nullptr;
        for (const Solution& q : solutions)
            if (q.edge == edgeIndex)
                cur = &q;
        if (!cur)
            return;
        if (newLabelSize != cfg.edgePointLabelSize)
        {
            newLabelSize = cfg.edgePointLabelSize;
            newLabel.set(R"(r_{\mathrm{new}})", cfg.edgePointLabelSize, kNewColor);
        }
        float px, py, rx, ry;
        if (newLabel.empty() || !toScreen(glm::vec3(cur->point), px, py) || !toScreen(glm::vec3(cur->A.r), rx, ry))
            return;
        glm::vec2 d(px - rx, py - ry);
        d = glm::length(d) > 1e-3f ? glm::normalize(d) : glm::vec2(1.0f, 0.0f);
        const glm::vec2 half(0.5f * (float)newLabel.width(), 0.5f * (float)newLabel.height());
        const glm::vec2 c = glm::vec2(px, py) + d * (12.0f + std::abs(d.x) * half.x + std::abs(d.y) * half.y);
        newLabel.drawAt(c.x - half.x, c.y - half.y);
    }

    // ---- the ball on the edge and its particles ----------------------------------------

    bool overBall() const
    {
        float sx, sy;
        if (!toScreen(edgePoint(ballAngle), sx, sy))
            return false;
        const ImVec2 m = ImGui::GetIO().MousePos;
        const float reach = std::max(kPickPixels, 0.5f * cfg.ballSize + 4.0f);
        return (sx - m.x) * (sx - m.x) + (sy - m.y) * (sy - m.y) < reach * reach;
    }

    // The ball's next angle while dragging: the point of the edge nearest the cursor on
    // screen, searched only within kBallStep of where the ball is (so it slides along
    // the edge and never leaps to another stretch that happens to pass near the cursor)
    // and projected onto each little segment between samples (so it moves continuously,
    // not from sample to sample).
    float slideBall(float mx, float my) const
    {
        const int N = (int)edgeCurve.size();
        if (N < 2)
            return ballAngle;
        const float perSample = 6.2831853f / (float)N;
        const int k0 = (int)std::floor(ballAngle / perSample);
        const int reach = (int)std::ceil(kBallStep / perSample);
        const glm::vec2 m(mx, my);
        float best = ballAngle, bestD = 1e30f;
        for (int k = k0 - reach; k <= k0 + reach; ++k)
        {
            float ax, ay, bx, by;
            if (!toScreen(edgeCurve[(size_t)((k % N + N) % N)], ax, ay) ||
                !toScreen(edgeCurve[(size_t)(((k + 1) % N + N) % N)], bx, by))
                continue;
            const glm::vec2 a(ax, ay), ab(bx - ax, by - ay);
            const float len2 = glm::dot(ab, ab);
            const float u = len2 > 1e-9f ? std::clamp(glm::dot(m - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
            const glm::vec2 q = a + u * ab;
            const float d = glm::dot(m - q, m - q);
            if (d < bestD)
            {
                bestD = d;
                best = ((float)k + u) * perSample;
            }
        }
        // stay within kBallStep of the last angle, wrapped into [0, 2 pi)
        best = ballAngle + std::clamp(best - ballAngle, -kBallStep, kBallStep);
        best = std::fmod(best, 6.2831853f);
        return best < 0.0f ? best + 6.2831853f : best;
    }

    // A particle from the ball's point of the edge (on the manifold), Taylor-integrated.
    void launchParticle()
    {
        launchAt(ballAngle, -1);
    }

    // "Shoot r -> ball": shootCount particles evenly along the edge from r to the ball
    // (the shorter way round), one batch — render joins their heads into the arc the
    // flow has carried them to.
    void shootFromR()
    {
        if (edgeIndex < 0 || edgeCurve.empty())
            return;
        const float a0 = edgeAngles[(size_t)edgeIndex];
        float span = std::fmod(ballAngle - a0, 6.2831853f);
        if (span > 3.14159265f)
            span -= 6.2831853f;
        else if (span < -3.14159265f)
            span += 6.2831853f;
        syncHits();   // before marking the first one: a change of r just now must not undo it
        const int n = std::clamp(cfg.shootCount, 2, kMaxShoot);
        const int batch = ++batchCounter;
        for (int i = 0; i < n; ++i)
            launchAt(a0 + span * (float)i / (float)(n - 1), batch);
        // the first starts at r itself, already in F_r: its b_r is r, at tau = 0
        EdgeFrame f;
        Particle& first = *particles[particles.size() - (size_t)std::min(n, (int)particles.size())];
        if (edgeFrame(edgeIndex, f) && first.batch == batch)
        {
            first.hit = true;
            first.hitPos = f.r;
            first.hitR = f.r;
            first.tau = 0.0;
            first.dist = 0.0;
            first.hitSeq = ++hitCounter;
            first.labelSize = 0.0f;
        }
    }

    // B: a particle from every edge point (at most kMaxRing, evenly picked), one batch
    // drawn as a closed loop through the heads.
    void shootRing()
    {
        const int E = (int)edgeAngles.size();
        if (E == 0)
            return;
        const int n = std::min(E, kMaxRing);
        const int batch = ++batchCounter;
        for (int i = 0; i < n; ++i)
            launchAt(edgeAngles[(size_t)((long long)i * E / n)], batch, true);
    }

    // A hit belongs to the r it was found with: a new r (or a new mesh) clears them.
    void syncHits()
    {
        if (edgeIndex == hitEdge && meshVersion == hitMeshVersion)
            return;
        hitEdge = edgeIndex;
        hitMeshVersion = meshVersion;
        for (auto& P : particles)
            P->hit = false;
    }

    void launchAt(float angle, int batch, bool ring = false)
    {
        if (edgeCurve.empty())
            return;
        const glm::vec3 p = edgePoint(angle);
        const double seed[3] = {p.x, p.y, p.z};
        auto P = std::make_unique<Particle>();
        P->traj = std::make_unique<engine::Trajectory>(3, seed, 0.0, ring ? kRingPoints : kParticlePoints);
        P->traj->minSpacing = (ring ? kRingSpacing : 1e-4f) * (float)radius;
        P->batch = batch;
        P->ring = ring;
        particles.push_back(std::move(P));
        if ((int)particles.size() > kMaxParticles)
            particles.erase(particles.begin());
    }

    // one particle: its curve and, once it has crossed F_r, where and when
    struct Particle {
        std::unique_ptr<engine::Trajectory> traj;
        bool hit = false;
        glm::vec3 hitPos{0.0f};       // b_r(tau)
        glm::vec3 hitR{0.0f};         // the r it was measured from
        double tau = 0.0;             // field time from the edge to b_r (signed: < 0 backward)
        double dist = 0.0;            // ||b_r(tau) - r||
        engine::LatexImage label, distLabel;
        float labelSize = 0.0f;       // typeset at (0: not yet)
        int hitSeq = 0;               // order of the hits: only the latest is labelled
        int batch = -1;               // its "Shoot r -> ball" shot (-1: a single launch)
        bool ring = false;            // from B (the whole edge): never stops at F_r
    };

    struct EdgeFrame;

    // Did this step, from x0 at time t0 to the particle's state now, cross F_r? The
    // plane through r with normal n, on its outer half ((x - r).out >= 0): a strict
    // sign change of (x - r).n, the crossing interpolated inside the step (a tiny one,
    // so the straight chord is the curve to well below the drawn size). A start exactly
    // in the plane (the ball at r) is not a hit; it has to come back to it.
    void findHit(Particle& P, const EdgeFrame& f, const glm::dvec3& x0, double t0)
    {
        const double* x = P.traj->state();
        const glm::dvec3 x1(x[0], x[1], x[2]);
        const glm::dvec3 r(f.r), n = glm::normalize(glm::dvec3(f.n));
        const double g0 = glm::dot(x0 - r, n), g1 = glm::dot(x1 - r, n);
        if (!((g0 < 0.0 && g1 > 0.0) || (g0 > 0.0 && g1 < 0.0)))
            return;
        const double l = g0 / (g0 - g1);
        const glm::dvec3 b = x0 + l * (x1 - x0);
        // the inner half is not F_r; a crossing right at r (a particle started next to it)
        // may round to a hair inside, so the boundary gets a small tolerance
        if (glm::dot(b - r, glm::dvec3(f.out)) < -0.05 * glm::length(glm::dvec3(f.n)))
            return;
        P.hit = true;
        P.hitPos = glm::vec3(b);
        P.hitR = f.r;
        P.tau = t0 + l * (P.traj->time() - t0);
        P.dist = glm::length(b - r);
        P.hitSeq = ++hitCounter;
        P.labelSize = 0.0f;   // typeset on the next drawHitLabels
    }

    // Every hit: the segment b_r(tau) -> r, then the golden dot — an additive glow
    // around a solid core, so it reads as emitting light.
    void renderHits(float px)
    {
        std::vector<float> v;
        auto put = [&](const glm::vec3& p) { v.insert(v.end(), {p.x, p.y, p.z}); };
        int n = 0;
        for (const auto& P : particles)
            if (P->hit && n < kMaxParticles)
            {
                put(P->hitPos);
                put(P->hitR);
                ++n;
            }
        if (n == 0)
            return;
        std::vector<float> dots;
        for (int i = 0; i < n; ++i)
            dots.insert(dots.end(), v.begin() + 6 * i, v.begin() + 6 * i + 3);
        v.insert(v.end(), dots.begin(), dots.end());
        hitMesh->setVertices(v.data(), std::min(v.size(), (size_t)kHitCapacity * 3));

        shader.setVec3("u_color", kHitColor);
        shader.setFloat("u_alpha", 0.85f);
        glLineWidth(2.0f * px);
        hitMesh->drawArrays(GL_LINES, 2 * n, 0);
        shader.setInt("u_round", 1);
        const float pulse = 1.0f + 0.08f * std::sin(5.0f * clock);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        for (float k : {3.6f, 2.4f})
        {
            shader.setFloat("u_alpha", k > 3.0f ? 0.18f : 0.35f);
            shader.setFloat("u_pointSize", k * pulse * cfg.particleSize * px);
            hitMesh->drawArrays(GL_POINTS, n, 2 * n);
        }
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        shader.setVec3("u_color", glm::mix(kHitColor, glm::vec3(1.0f), 0.35f));
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_pointSize", 1.3f * cfg.particleSize * px);
        hitMesh->drawArrays(GL_POINTS, n, 2 * n);
        shader.setInt("u_round", 0);
    }

    // b_r(tau = ...) beside the latest hit's dot, ||b_r(tau) - r|| = ... beside the
    // middle of its segment; older hits keep their dot and segment, unlabelled (their
    // labels would pile up on each other).
    void drawHitLabels()
    {
        if (!cfg.showLabels)
            return;
        auto fmt = [](double v) {
            char b[32];
            std::snprintf(b, sizeof(b), "%.4g", v);
            std::string t = b;
            const size_t e = t.find('e');
            if (e != std::string::npos)   // 1.2e-05 -> 1.2 \cdot 10^{-5}
            {
                const int ex = std::atoi(t.c_str() + e + 1);
                t = t.substr(0, e) + "\\cdot 10^{" + std::to_string(ex) + "}";
            }
            return t;
        };
        Particle* latest = nullptr;
        for (auto& P : particles)
            if (P->hit && (!latest || P->hitSeq > latest->hitSeq))
                latest = P.get();
        if (latest)
        {
            Particle* P = latest;
            if (P->labelSize != cfg.hitLabelSize)
            {
                P->labelSize = cfg.hitLabelSize;
                const float sz = cfg.hitLabelSize;
                P->label.set("b_{r}(\\tau = " + fmt(P->tau) + ")", sz, kHitColor);
                P->distLabel.set("\\left\\| b_{r}(\\tau) - r \\right\\| = " + fmt(P->dist), sz,
                                 glm::mix(kHitColor, glm::vec3(1.0f), 0.3f));
            }
            // on screen: b_r's label beyond the dot, away from r; the distance label
            // beside the segment's middle, on its upper side
            float bx, by, rx, ry;
            if (!toScreen(P->hitPos, bx, by) || !toScreen(P->hitR, rx, ry))
                return;
            glm::vec2 d(bx - rx, by - ry);
            d = glm::length(d) > 1e-3f ? glm::normalize(d) : glm::vec2(-1.0f, 0.0f);
            glm::vec2 perp(-d.y, d.x);
            if (perp.y > 0.0f)
                perp = -perp;
            auto place = [](const engine::LatexImage& img, glm::vec2 at, glm::vec2 dir, float push) {
                if (img.empty())
                    return;
                const glm::vec2 half(0.5f * (float)img.width(), 0.5f * (float)img.height());
                const glm::vec2 c = at + dir * (push + std::abs(dir.x) * half.x + std::abs(dir.y) * half.y);
                img.drawAt(c.x - half.x, c.y - half.y);
            };
            place(P->label, glm::vec2(bx, by), d, 14.0f);
            place(P->distLabel, 0.5f * glm::vec2(bx + rx, by + ry), perp, 8.0f);
        }
    }

    // Each shot's heads joined in launch order (a stopped one at its b_r): the edge arc
    // from r to the ball as the flow has carried it so far.
    void renderFronts(float px)
    {
        std::vector<float> v;
        std::vector<std::pair<int, int>> runs;   // (first vertex, count) per shot
        for (size_t i = 0; i < particles.size();)
        {
            const int batch = particles[i]->batch;
            size_t j = i;
            const int first = (int)(v.size() / 3);
            for (; j < particles.size() && particles[j]->batch == batch; ++j)
                if (batch >= 0)
                {
                    const glm::vec3 h = particles[j]->hit ? particles[j]->hitPos : particles[j]->traj->head();
                    v.insert(v.end(), {h.x, h.y, h.z});
                }
            if (batch >= 0 && particles[i]->ring && j - i > 2)   // the edge is a loop
                v.insert(v.end(), v.begin() + 3 * first, v.begin() + 3 * first + 3);
            const int count = (int)(v.size() / 3) - first;
            if (batch >= 0 && count >= 2)
                runs.push_back({first, count});
            i = j;
        }
        if (runs.empty())
            return;
        frontMesh->setVertices(v.data(), std::min(v.size(), (size_t)(kMaxParticles + 64) * 3));
        shader.setVec3("u_color", glm::mix(kParticleColor, glm::vec3(1.0f), 0.35f));
        shader.setFloat("u_alpha", 0.8f);
        glLineWidth(1.4f * cfg.particleTrail * px);
        for (const auto& r : runs)
            frontMesh->drawArrays(GL_LINE_STRIP, r.second, r.first);
        shader.setVec3("u_color", kParticleColor);
    }

    // The ball (always, on the edge) and every particle: trail, then head.
    void renderParticles(float px)
    {
        if (edgeCurve.empty())
            return;
        shader.setVec3("u_color", kParticleColor);
        shader.setInt("u_useRamp", 0);
        for (const auto& P : particles)
        {
            const engine::Trajectory* t = P->traj.get();
            shader.setFloat("u_alpha", 0.9f);
            glLineWidth(cfg.particleTrail * px);
            t->draw();
            shader.setFloat("u_alpha", 1.0f);
            shader.setFloat("u_pointSize", cfg.particleSize * px);
            shader.setInt("u_round", 1);
            t->drawHead();
            shader.setInt("u_round", 0);
        }
        renderFronts(px);
        const glm::vec3 b = edgePoint(ballAngle);
        ballMesh->setVertices(&b.x, 3);
        shader.setFloat("u_alpha", ballHover || ballDragging ? 1.0f : 0.9f);
        shader.setFloat("u_pointSize", (ballHover || ballDragging ? 1.2f : 1.0f) * cfg.ballSize * px);
        shader.setInt("u_round", 1);
        ballMesh->drawArrays(GL_POINTS, 1, 0);
        shader.setInt("u_round", 0);
        shader.setFloat("u_alpha", 1.0f);
    }

    // ---- the edge point r and its half-plane F_r --------------------------------------

    // Vertex j of the edge (the last ring), wrapping around.
    glm::vec3 edgeVertex(int j) const
    {
        const int R = cfg.rings, S = cfg.segs, E = (int)edgeAngles.size();
        if (layer > 0)
            return glm::vec3(ringPts[(size_t)((j % E + E) % E)]);
        return verts[1 + (size_t)(R - 1) * S + (size_t)((j % E + E) % E)];
    }

    // r = edge vertex j, its neighbours r^+ (j + 1) and r^- (j - 1), the normal
    // n = (r^+ - r^-) / 2 of F_r, and F_r's own frame: `out` points away from the patch
    // along it (the radial direction of K at r, minus its n part), `across` = n x out.
    struct EdgeFrame {
        glm::vec3 r, plus, minus, n, out, across;
        float size;   // of F_r: planeScale |r^+ - r^-|
    };

    bool edgeFrame(int j, EdgeFrame& f) const
    {
        if (j < 0 || verts.empty() || edgeAngles.empty())
            return false;
        f.r = edgeVertex(j);
        f.plus = edgeVertex(j + 1);
        f.minus = edgeVertex(j - 1);
        f.n = 0.5f * (f.plus - f.minus);
        const float len = glm::length(f.n);
        if (len < 1e-6f)
            return false;
        const glm::vec3 nh = f.n / len;
        const float a = edgeAngles[(size_t)(j % (int)edgeAngles.size())];
        const glm::vec3 out = glm::vec3(outwardAt(a, glm::dvec3(nh)));
        if (glm::length(out) < 0.5f)
            return false;
        f.out = out;
        f.across = glm::normalize(glm::cross(nh, f.out));
        f.size = cfg.planeScale * 2.0f * len;
        return true;
    }

    // The edge vertex within kPickPixels of the cursor (nearest), or -1.
    int pickEdge() const
    {
        if (verts.empty() || fillCount == 0)
            return -1;
        const ImVec2 m = ImGui::GetIO().MousePos;
        int best = -1;
        float bestD = kPickPixels * kPickPixels;
        for (int j = 0; j < (int)edgeAngles.size(); ++j)
        {
            float sx, sy;
            if (!toScreen(edgeVertex(j), sx, sy))
                continue;
            const float d = (sx - m.x) * (sx - m.x) + (sy - m.y) * (sy - m.y);
            if (d < bestD)
            {
                bestD = d;
                best = j;
            }
        }
        return best;
    }

    // r = edge vertex j: the mesh comes on (r^+- are its vertices) and the camera eases in.
    void chooseEdge(int j)
    {
        // the ball follows r along the edge, keeping its offset (so a shot r -> ball
        // spans the same stretch from the new r)
        if (edgeIndex >= 0 && edgeIndex < (int)edgeAngles.size() && j < (int)edgeAngles.size())
        {
            const float a = std::fmod(ballAngle + edgeAngles[(size_t)j] - edgeAngles[(size_t)edgeIndex],
                                      6.2831853f);
            ballAngle = a < 0.0f ? a + 6.2831853f : a;
        }
        edgeIndex = j;
        cfg.showMesh = true;
        zoomToEdge();
    }

    // Ease onto r, keeping the view angles, F_r in frame.
    void zoomToEdge()
    {
        EdgeFrame f;
        if (!edgeFrame(edgeIndex, f))
            return;
        // screen-up = `across` (in F_r, normal to the surface): the patch reads as a floor,
        // F_r as a wall standing on its edge. The camera looks at it from the side of n
        // and of the surface it is already on, diagonally from the patch side (so neither n
        // nor F_r is seen end-on) and a little from above.
        engine::Camera& cam = app->camera();
        const glm::vec3 now = cam.position - cam.orbitCenter;
        const glm::vec3 nh = glm::normalize(f.n);
        const glm::vec3 up = glm::dot(now, f.across) < 0.0f ? -f.across : f.across;
        const glm::vec3 toward = glm::dot(now, nh) < 0.0f ? -nh : nh;
        const glm::vec3 d = glm::normalize(0.6f * toward - 0.65f * f.out + 0.3f * up);
        const glm::vec3 z = glm::normalize(d - glm::dot(d, up) * up);
        const glm::mat3 R(glm::cross(up, z), up, z);   // columns: local x, y (up), z
        const float phi = std::acos(std::clamp(glm::dot(d, up), -1.0f, 1.0f));
        cam.easeOrbitTo(f.r + 0.35f * f.size * f.out, cfg.zoomScale * f.size, 0.0f, phi,
                        engine::App::kRecenterEaseSeconds);
        easeFrameTo(glm::quat_cast(R), engine::App::kRecenterEaseSeconds);
    }

    // ---- the orbit frame ----------------------------------------------------------------

    // Camera's orbit angles are about world +Y; here they are about viewFrame's axes
    // (local y = screen-up), so the view can stand upright on F_r. Camera places the
    // orbit in world axes before update; placeCamera re-places it in the frame. The
    // frame eases with the same smootherstep and duration as Camera::easeOrbitTo, run
    // alongside it.
    void easeFrameTo(const glm::quat& q, float seconds)
    {
        frameFrom = viewFrame;
        frameTo = q;
        frameT = 0.0f;
        frameDuration = std::max(seconds, 1e-3f);
    }

    static bool isIdentity(const glm::quat& q) { return std::abs(q.w) > 0.999999f; }

    void updateViewFrame(float dt)
    {
        if (frameDuration > 0.0f)
        {
            frameT += dt;
            const float t = std::clamp(frameT / frameDuration, 0.0f, 1.0f);
            const float s = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
            viewFrame = glm::slerp(frameFrom, frameTo, s);
            if (t >= 1.0f)
            {
                viewFrame = frameTo;
                frameDuration = 0.0f;
                if (backAfterFrame)
                {
                    backAfterFrame = false;
                    switchBack();
                }
            }
        }
        placeCamera();
    }

    void placeCamera()
    {
        engine::Camera& cam = app->camera();
        const glm::mat3 R = glm::mat3_cast(viewFrame);
        const float th = cam.orbitTheta, ph = cam.orbitPhi;
        const glm::vec3 local(std::sin(ph) * std::sin(th), std::cos(ph), std::sin(ph) * std::cos(th));
        cam.position = cam.orbitCenter + cam.orbitRadius * (R * local);
        cam.front = glm::normalize(cam.orbitCenter - cam.position);
        cam.up = R[1];
    }

    // The orbit angles in world axes that put the camera where it is now.
    void worldAngles(float& theta, float& phi) const
    {
        const engine::Camera& cam = app->camera();
        const glm::vec3 d = glm::normalize(cam.position - cam.orbitCenter);
        theta = std::atan2(d.x, d.z);
        phi = std::acos(std::clamp(d.y, -1.0f, 1.0f));
    }

    // The hovered edge vertex, and once r is chosen: F_r (fill, its three free sides, the
    // edge side through r), the chord r^- r^+, the points r, r^+-.
    void renderEdgePoint(float px)
    {
        std::vector<float> v;
        auto put = [&](const glm::vec3& p) { v.insert(v.end(), {p.x, p.y, p.z}); };
        const bool hover = hoverEdge >= 0 && hoverEdge != edgeIndex;
        if (hover)
            put(edgeVertex(hoverEdge));
        EdgeFrame f;
        const bool have = rFade > 0.0f && edgeFrame(edgeIndex, f);
        if (have)
        {
            const glm::vec3 A = f.size * f.out, B = 0.5f * f.size * f.across;
            const glm::vec3 c0 = f.r - B, c1 = f.r + B, c2 = f.r + B + A, c3 = f.r - B + A;
            for (const glm::vec3& p : {c0, c1, c2, c0, c2, c3})   // fill: 0..5
                put(p);
            for (const glm::vec3& p : {c1, c2, c2, c3, c3, c0})   // free sides: 6..11
                put(p);
            put(c0);                                              // edge side: 12, 13
            put(c1);
            put(f.minus);                                         // chord: 14, 15
            put(f.plus);
            put(f.r);                                             // points: 16, 17, 18
            put(f.plus);
            put(f.minus);
        }
        if (v.empty())
            return;
        overlayMesh->setVertices(v.data(), std::min(v.size(), (size_t)kOverlayCapacity * 3));
        const int o = hover ? 1 : 0;

        if (have)
        {
            shader.setVec3("u_color", kPlaneColor);
            shader.setFloat("u_alpha", cfg.planeAlpha * rFade);
            overlayMesh->drawArrays(GL_TRIANGLES, 6, o);
            shader.setFloat("u_alpha", 0.55f * rFade);
            glLineWidth(1.2f * px);
            overlayMesh->drawArrays(GL_LINES, 6, o + 6);
            shader.setFloat("u_alpha", 0.95f * rFade);
            glLineWidth(2.2f * px);
            overlayMesh->drawArrays(GL_LINES, 2, o + 12);
            shader.setVec3("u_color", kNeighborColor);
            shader.setFloat("u_alpha", 0.6f * rFade);
            glLineWidth(1.5f * px);
            overlayMesh->drawArrays(GL_LINES, 2, o + 14);
        }
        shader.setInt("u_round", 1);
        if (hover)
        {
            shader.setVec3("u_color", kPointColor);
            shader.setFloat("u_alpha", 0.8f);
            shader.setFloat("u_pointSize", 1.6f * cfg.vertexSize * px);
            overlayMesh->drawArrays(GL_POINTS, 1, 0);
        }
        if (have)
        {
            shader.setVec3("u_color", kNeighborColor);
            shader.setFloat("u_alpha", rFade);
            shader.setFloat("u_pointSize", 1.5f * cfg.vertexSize * px);
            overlayMesh->drawArrays(GL_POINTS, 2, o + 17);
            shader.setVec3("u_color", kPointColor);
            shader.setFloat("u_pointSize", 2.0f * cfg.vertexSize * px);
            overlayMesh->drawArrays(GL_POINTS, 1, o + 16);
        }
        shader.setInt("u_round", 0);
        shader.setFloat("u_alpha", 1.0f);
    }

    // r, r^+, r^- and F_r beside what they name.
    void drawEdgeLabels()
    {
        EdgeFrame f;
        if (!cfg.showLabels || rFade <= 0.0f || !edgeFrame(edgeIndex, f))
            return;
        if (edgeLabelSize != cfg.edgePointLabelSize)
        {
            edgeLabelSize = cfg.edgePointLabelSize;
            const float S = cfg.edgePointLabelSize, s = 0.8f * cfg.edgePointLabelSize;
            rLabel.set("r", S, kPointColor);
            plusLabel.set("r^{+}", s, kNeighborColor);
            minusLabel.set("r^{-}", s, kNeighborColor);
            planeLabel.set(R"(\mathcal{F}_{r})", S, kPlaneColor);
        }
        // screen directions of n and of `out` at r, so labels sit beside what they name
        // whichever way the camera looks
        float rx, ry;
        if (!toScreen(f.r, rx, ry))
            return;
        auto screenDir = [&](const glm::vec3& d) {
            float x, y;
            if (!toScreen(f.r + d, x, y))
                return glm::vec2(0.0f);
            glm::vec2 v(x - rx, y - ry);
            const float l = glm::length(v);
            return l > 1e-3f ? v / l : glm::vec2(0.0f);
        };
        const glm::vec2 dn = screenDir(0.1f * f.n), dout = screenDir(0.1f * f.size * f.out);
        // centred on the point p, pushed `push` px along dir plus half its own extent
        auto at = [&](const engine::LatexImage& img, const glm::vec3& p, glm::vec2 dir, float push) {
            float sx, sy;
            if (img.empty() || !toScreen(p, sx, sy))
                return;
            const glm::vec2 half(0.5f * (float)img.width(), 0.5f * (float)img.height());
            const glm::vec2 c = glm::vec2(sx, sy) +
                                dir * (push + std::abs(dir.x) * half.x + std::abs(dir.y) * half.y);
            img.drawAt(c.x - half.x, c.y - half.y, rFade);
        };
        at(rLabel, f.r, -dout, 10.0f);
        at(plusLabel, f.plus, dn, 8.0f);
        at(minusLabel, f.minus, -dn, 8.0f);
        at(planeLabel, f.r + 0.8f * f.size * f.out + 0.3f * f.size * f.across, glm::vec2(0.0f), 0.0f);
    }

    // ---- camera, labels, handoff ----------------------------------------------------

    // Orbit around x* at kFramingScale patch radii (or the saved distance); eased when
    // asked, else at once.
    void frameCamera(bool eased)
    {
        engine::Camera& cam = app->camera();
        const float dist = cfg.camRadius > 0.0f ? cfg.camRadius
                                                : kFramingScale * std::max((float)radius, 0.5f);
        if (eased)
        {
            cam.easeOrbitTo(eqPos, dist, cfg.camTheta, cfg.camPhi, engine::App::kRecenterEaseSeconds);
            easeFrameTo(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), engine::App::kRecenterEaseSeconds);
            return;
        }
        viewFrame = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        frameDuration = 0.0f;
        cam.orbitCenter = eqPos;
        cam.orbitRadius = dist;
        cam.orbitTheta = cfg.camTheta;
        cam.orbitPhi = cfg.camPhi;
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

    // O beside the bead, W^s_loc(O) beside the patch (lorenz_manifold_growth's look).
    void drawLabels()
    {
        if (!cfg.showLabels || !found)
            return;
        if (labelDirty || labelSize != cfg.labelSize || patchLabelShown != cfg.patchLabelSize)
        {
            labelDirty = false;
            labelSize = cfg.labelSize;
            patchLabelShown = cfg.patchLabelSize;
            eqLabel.set(eqTex, cfg.labelSize, glm::vec3(1.0f, 0.45f, 0.40f));
            patchLabel.set(std::string("W^{") + (side == 0 ? "s" : "u") + "}_{\\mathrm{loc}}(" +
                               eqTex + ")",
                           cfg.patchLabelSize, side == 0 ? kStableColor : kUnstableColor);
        }
        float sx, sy;
        if (!eqLabel.empty() && toScreen(eqPos, sx, sy))
            eqLabel.drawAt(sx + 10.0f + 0.3f * cfg.labelSize, sy - (float)eqLabel.height() - 2.0f, 0.75f);
        if (error.empty() && !patchLabel.empty() && toScreen(labelAnchor, sx, sy))
            patchLabel.drawAt(sx + 8.0f, sy - (float)patchLabel.height() - 4.0f, 0.9f * (1.0f - rFade));
    }

    // Back to lorenz_manifold_growth: everything it handed over, with the parameters as
    // they are now and fromLevelSets set — it zooms back out to its global view.
    void handBack()
    {
        if (!app->hasSim(kGrowthSim) || backAfterFrame)
            return;
        if (frameDuration > 0.0f || !isIdentity(viewFrame))
        {
            // turn back to world-up where the camera stands, then switch
            // (lorenz_manifold_growth orbits about world +Y)
            float th, ph;
            worldAngles(th, ph);
            engine::Camera& cam = app->camera();
            cam.easeOrbitTo(cam.orbitCenter, cam.orbitRadius, th, ph, kUprightSeconds);
            easeFrameTo(glm::quat(1.0f, 0.0f, 0.0f, 0.0f), kUprightSeconds);
            backAfterFrame = true;
            return;
        }
        switchBack();
    }

    void switchBack()
    {
        engine::App::Handoff out = returnValues;
        out["sigma"] = cfg.sigma;
        out["rho"] = cfg.rho;
        out["beta"] = cfg.beta;
        out["eq"] = cfg.eq;
        out["side"] = side;
        out["order"] = cfg.manifoldOrder;
        out["fromLevelSets"] = 1.0;
        app->requestSwitch(kGrowthSim, std::move(out));
    }

    bool saveCurrentSettings()
    {
        worldAngles(cfg.camTheta, cfg.camPhi);   // saved in world axes, whatever the frame
        cfg.camRadius = app->camera().orbitRadius;
        return saveSettingsFile(cfg, kSettingsPath);
    }

    // Scene only, at shotScale x the window, into <repo>/screenshots/ with a timestamp.
    void takeScreenshot()
    {
        std::time_t now = std::time(nullptr);
        std::tm local{};
        localtime_s(&local, &now);
        char stamp[32];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &local);
        std::string path = std::string(SIMS_DIR "../screenshots/learn_geodesic_level_sets_") + stamp + ".png";
        shotStatus = app->captureScreenshot(path, shotScale)
            ? "saved screenshots/" + std::filesystem::path(path).filename().string()
            : "screenshot failed (see console)";
    }

    engine::App* app = nullptr;
    engine::Shader shader;      // patch fill, edges, vertices, axes (the glyph pair)
    engine::Shader eqShader;    // the bead
    std::unique_ptr<engine::Mesh> axisMesh;
    std::unique_ptr<engine::Mesh> sphereMesh;
    std::unique_ptr<engine::Mesh> patchMesh;
    // accepted rings (acceptRing): how many, the bands between them, the current edge
    // (past layer 0) as points with their parameter and the direction each grew in
    std::unique_ptr<engine::Mesh> layerMesh;
    int layer = 0;
    std::vector<float> layerTri;
    std::vector<glm::dvec3> layerLines;
    int layerFill = 0, layerLineCount = 0;
    std::vector<glm::dvec3> ringPts, ringOut;
    std::vector<glm::dvec3> ringPrev;   // each edge point's p_r (the 3.2 angle check)
    std::vector<double> ringParam;
    // Delta_i (3.2): the step the current ring is solved for; eta the geodesic distance
    // of the current edge from the first one; the grow run
    double stepDelta = 1.0, eta = 0.0;
    int growLeft = 0, growWarnings = 0;
    int linearInserts = 0;              // accept-time midpoints not solved by a BVP

    Settings cfg;
    std::string settingsStatus;
    int shotScale = 2;
    std::string shotStatus;

    engine::LorenzSystem system;
    engine::InvariantManifold K[2];   // stable, unstable side of the chosen equilibrium
    int side = 0;                     // the 2-D one shown
    int eqTotal = 0;
    bool found = false;
    std::string error;                // why there is no patch
    glm::vec3 eqPos{0.0f};
    std::string eqTex = "O";
    double radius = 0.0;
    int fillCount = 0, edgeCount = 0, pointCount = 0, boundaryFirst = 0;
    glm::vec3 labelAnchor{0.0f};
    float meshFade = 0.0f;
    std::vector<glm::vec3> verts;     // the mesh's vertices: x*, then ring by ring
    glm::vec3 basis0{1.0f, 0.0f, 0.0f}, basis1{0.0f, 1.0f, 0.0f};   // E, orthonormal

    // the edge point r (edge vertex index, -1: none) and its construction
    std::unique_ptr<engine::Mesh> overlayMesh;
    int edgeIndex = -1;
    int hoverEdge = -1;
    float rFade = 0.0f;
    engine::LatexImage rLabel, plusLabel, minusLabel, planeLabel;
    float edgeLabelSize = 0.0f;
    float patchLabelShown = 0.0f;
    std::vector<float> edgeAngles;    // the edge's points, by angle in E (spreadEdge)
    glm::dvec3 eqPosD{0.0};           // x*, double (the BVP)

    // the geodesic BVP: solved edge points, the mids between them, the ring's Delta_i
    std::vector<Solution> solutions;  // sorted by edge vertex
    std::map<int, Solution> mids;     // keyed by the first edge vertex of the pair
    double ringDelta = 0.0;
    std::string bvpStatus;
    std::vector<int> solveAllQueue;   // "Solve all r": the vertices it still has to do
    int solveAllNext = -1;            // ...next index into it (-1: not running)
    int lastSolved = -1;              // edge vertex solved most recently
    int solveAllVersion = 0;          // the meshVersion it was started on
    static constexpr float kSolveAllBudgetMs = 12.0f;   // its slice of each frame
    std::unique_ptr<engine::Mesh> bandMesh;
    engine::LatexImage newLabel;
    float newLabelSize = 0.0f;

    // the method's parameters, top-right
    engine::LatexImage paramsLatex;
    bool paramsDirty = true;
    float paramsShownSize = 0.0f, paramsShownMax = 0.0f, paramsShownMin = 0.0f;
    float paramsShownDMin = 0.0f, paramsShownDMax = 0.0f;

    // the orbit frame (local y = screen-up) and its ease
    glm::quat viewFrame{1.0f, 0.0f, 0.0f, 0.0f};
    glm::quat frameFrom{1.0f, 0.0f, 0.0f, 0.0f}, frameTo{1.0f, 0.0f, 0.0f, 0.0f};
    float frameT = 0.0f, frameDuration = 0.0f;
    bool backAfterFrame = false;   // handBack waits for the turn upright

    // the ball on the edge and the particles launched from it
    std::vector<glm::vec3> edgeCurve;   // the edge, kEdgeSamples points
    std::unique_ptr<engine::Mesh> ballMesh;
    float ballAngle = 0.0f;             // where on the edge, radians in E
    bool ballHover = false;
    bool ballDragging = false;
    std::vector<std::unique_ptr<Particle>> particles;
    std::unique_ptr<engine::Mesh> hitMesh;
    int hitEdge = -1;                 // the r (and mesh) the hits were found with
    int meshVersion = 0, hitMeshVersion = 0;
    int hitCounter = 0;
    int batchCounter = 0;
    std::unique_ptr<engine::Mesh> frontMesh;   // the shots' head-to-head arcs
    float clock = 0.0f;               // the golden dots' pulse
    engine::TaylorIntegrator taylor;
    engine::TaylorIntegrator::Jet jet;  // LorenzSystem::jet
    bool playing = true;
    bool reverse = false;               // against the manifold's own direction of time
    static constexpr float kUprightSeconds = 0.5f;

    engine::LatexImage eqLabel, patchLabel;
    bool labelDirty = true;
    float labelSize = 0.0f;

    bool handedOver = false;
    engine::App::Handoff returnValues;   // what lorenz_manifold_growth handed over
};

} // namespace

std::unique_ptr<engine::Sim> makeLearnGeodesicLevelSetsSim()
{
    return std::make_unique<LearnGeodesicLevelSetsSim>();
}
