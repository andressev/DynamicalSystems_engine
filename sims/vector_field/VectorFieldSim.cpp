#include "VectorFieldSim.h"

#include <engine/App.h>
#include <engine/ArrowGlyphs.h>
#include <engine/Camera.h>
#include <engine/FlowKeys.h>
#include <engine/TrajectoryRunner.h>
#include <engine/Integrator.h>
#include <engine/LatexImage.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>
#include <engine/Trajectory.h>
#include <engine/VectorField.h>

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
#include <string>
#include <vector>

namespace {

// ---- presets, colors, small helpers ---------------------------------------------

struct Preset {
    const char* name;
    const char* comps[3];
};

const Preset kPresets2D[] = {
    {"damped pendulum",    {"y", "-sin(x) - 0.2y", "0"}},
    {"linear saddle",      {"x", "-y", "0"}},
    {"center (rotation)",  {"-y", "x", "0"}},
    {"spiral sink",        {"-0.3x - y", "x - 0.3y", "0"}},
    {"van der Pol",        {"y", "2(1 - x^2)y - x", "0"}},
    {"dipole",             {"x^2 - y^2", "2x y", "0"}},
    {"time-dependent",     {"y + 0.5sin(t)", "-x", "0"}},
};

const Preset kPresets3D[] = {
    {"Lorenz",             {"10(y - x)", "x(28 - z) - y", "x y - 2.6667z"}},
    {"Rossler",            {"-y - z", "x + 0.2y", "0.2 + z(x - 5.7)"}},
    // Thomas' field is slow (|f| ~ 1): times 10 so it runs at the pace of the others
    {"Thomas",             {"10(sin(y) - 0.19x)", "10(sin(z) - 0.19y)", "10(sin(x) - 0.19z)"}},
    {"rigid rotation",     {"-y", "x", "0.3"}},
    {"3D saddle",          {"x", "y", "-2z"}},
};

const glm::vec3 kBallColor(1.0f);
const glm::vec3 kSingleColor = {0.35f, 0.95f, 0.45f};   // click-dropped: always green
const glm::vec3 kDivergedColor = {0.95f, 0.25f, 0.25f}; // a head that blew up
const glm::vec3 kTangentColor = {1.0f, 0.22f, 0.18f};   // x' = f(x) at a single's head

const char* kSettingsPath2D = SIMS_DIR "vector_field/settings_2d.txt";
const char* kSettingsPath3D = SIMS_DIR "vector_field/settings_3d.txt";

// UI section title: a red labelled separator with breathing room above.
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

// ---- per-mode settings ----------------------------------------------------------

// Everything a mode (R2 or R3) remembers: the typed field, every control, and the
// view. The sim holds one per mode and swaps them on an R2/R3 switch, so each mode
// keeps its own state; each is saved to its own file.
constexpr float kMinSpeed = 1e-4f;   // slowest flow (arrow keys ramp to it logarithmically)

struct Settings {
    char comps[3][256] = {};

    bool showSystemLatex = true;
    bool showTimeLabels = true;
    float systemLatexSize = 30.0f;  // the x' = f(x) overlay, px

    bool showArrows = true;
    bool showAxes = true;
    bool barbedHeads = false;
    bool lengthByMagnitude = false;
    bool autoColorScale = true;
    int gridN = 26;
    float arrowScale = 0.8f;
    float headFraction = 0.36f;
    float headWidth = 0.14f;
    float hideBelow = 0.0f;
    float domainR = 20.0f;        // R3 sampling cube half-size
    float arrowAlpha = 0.6f;
    float arrowTailAlpha = 0.25f;
    float manualColorMax = 10.0f;

    float timeScale = 1.0f;       // flow speed, kMinSpeed..10 on a logarithmic scale
    bool reverse = false;
    int method = 2;               // Integrator::Method, RK4
    float maxStep = 0.01f;

    float ballCenter[3] = {1.0f, 1.0f, 0.0f};
    float ballRadius = 0.5f;
    int ballCount = 200;
    bool ballFilled = false;
    bool ballWithSingle = false;  // drop a followable single at the ball's centre too

    float spawnDepth = 0.3f;      // R3 click seeds: fraction of camera -> orbit center
    float singleScale = 2.0f;     // singles' trail width and head size multiplier
    float timeLabelSize = 36.0f;
    bool showSeedValues = true;   // phi^t((1, 2)) vs. the symbol phi^t(x_0)
    bool showTangents = true;     // red x' arrow on every single
    float tangentSize = 0.12f;    // of the camera distance (R3) / view half-height (R2)
    bool followSingle = false;    // R3: the orbit camera tracks a single
    bool followNewest = true;     // ...the most recently dropped one
    int followIndex = 0;          // ...else this one (drop order, 0-based)

    bool showTrails = true;
    bool trailBySpeed = false;
    int trailPoints = 1500;
    float trailWidth = 1.5f;
    float trailAlpha = 0.7f;
    float particleSize = 7.0f;

    // R3 orbit view (camRadius <= 0: frame the domain automatically)
    float camTheta = 0.6f, camPhi = 1.1f, camRadius = 0.0f;
    float camCenter[3] = {0.0f, 0.0f, 0.0f};
    float orbitSpeed = 1.2f;      // rad/s
    float zoomSpeed = 0.0f;       // world units/s, <= 0: scale with the domain
    float followSeconds = 0.8f;   // ease onto a newly followed single (0 = snap)

    // R2 pan/zoom window: centre and half-height
    float viewCenter[2] = {0.0f, 0.0f};
    float viewScale = 5.0f;
};

Settings defaultSettings(int dim)
{
    Settings s;
    const Preset& p = dim == 2 ? kPresets2D[0] : kPresets3D[0];
    for (int i = 0; i < 3; ++i)
        std::snprintf(s.comps[i], sizeof(s.comps[i]), "%s", p.comps[i]);
    if (dim == 3)
    {
        s.gridN = 12;
        s.domainR = 20.0f;
        s.ballCenter[0] = 1.0f; s.ballCenter[1] = 1.0f; s.ballCenter[2] = 25.0f;
        s.ballRadius = 1.0f;
        s.trailPoints = 3000;
        s.arrowAlpha = 0.35f;
    }
    return s;
}

// One persisted value, by key: exactly one of the pointers is set.
struct Setting {
    const char* key;
    float* f = nullptr;
    int* i = nullptr;
    bool* b = nullptr;
    char* text = nullptr;   // 256-byte buffer, stored as the rest of the line
};

std::vector<Setting> settingsTable(Settings& s)
{
    return {
        {"comp0", nullptr, nullptr, nullptr, s.comps[0]},
        {"comp1", nullptr, nullptr, nullptr, s.comps[1]},
        {"comp2", nullptr, nullptr, nullptr, s.comps[2]},
        {"showSystemLatex", nullptr, nullptr, &s.showSystemLatex},
        {"showTimeLabels", nullptr, nullptr, &s.showTimeLabels},
        {"showArrows", nullptr, nullptr, &s.showArrows},
        {"showAxes", nullptr, nullptr, &s.showAxes},
        {"barbedHeads", nullptr, nullptr, &s.barbedHeads},
        {"lengthByMagnitude", nullptr, nullptr, &s.lengthByMagnitude},
        {"autoColorScale", nullptr, nullptr, &s.autoColorScale},
        {"gridN", nullptr, &s.gridN},
        {"arrowScale", &s.arrowScale}, {"headFraction", &s.headFraction},
        {"headWidth", &s.headWidth}, {"hideBelow", &s.hideBelow},
        {"domainR", &s.domainR}, {"arrowAlpha", &s.arrowAlpha},
        {"arrowTailAlpha", &s.arrowTailAlpha}, {"manualColorMax", &s.manualColorMax},
        {"timeScale", &s.timeScale},
        {"reverse", nullptr, nullptr, &s.reverse},
        {"method", nullptr, &s.method}, {"maxStep", &s.maxStep},
        {"ballCenterX", &s.ballCenter[0]}, {"ballCenterY", &s.ballCenter[1]},
        {"ballCenterZ", &s.ballCenter[2]}, {"ballRadius", &s.ballRadius},
        {"ballCount", nullptr, &s.ballCount},
        {"ballFilled", nullptr, nullptr, &s.ballFilled},
        {"ballWithSingle", nullptr, nullptr, &s.ballWithSingle},
        {"spawnDepth", &s.spawnDepth}, {"singleScale", &s.singleScale},
        {"timeLabelSize", &s.timeLabelSize},
        {"systemLatexSize", &s.systemLatexSize},
        {"showSeedValues", nullptr, nullptr, &s.showSeedValues},
        {"showTangents", nullptr, nullptr, &s.showTangents},
        {"tangentSize", &s.tangentSize},
        {"followSingle", nullptr, nullptr, &s.followSingle},
        {"followNewest", nullptr, nullptr, &s.followNewest},
        {"followIndex", nullptr, &s.followIndex},
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
        {"followSeconds", &s.followSeconds},
        {"viewCenterX", &s.viewCenter[0]}, {"viewCenterY", &s.viewCenter[1]},
        {"viewScale", &s.viewScale},
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
        else if (e.b) out << (*e.b ? 1 : 0);
        else out << e.text;
        out << '\n';
    }
    return (bool)out;
}

// "key value" lines; text values are the rest of the line. Unknown keys are ignored,
// and the sizes a hand edit could blow up are clamped.
void loadSettingsFile(Settings& s, const char* path)
{
    std::ifstream in(path);
    if (!in)
        return;
    std::vector<Setting> table = settingsTable(s);
    std::string line;
    while (std::getline(in, line))
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        size_t sp = line.find(' ');
        if (sp == std::string::npos)
            continue;
        std::string key = line.substr(0, sp), value = line.substr(sp + 1);
        for (const Setting& e : table)
        {
            if (key != e.key)
                continue;
            double v = std::strtod(value.c_str(), nullptr);
            if (e.f) *e.f = (float)v;
            else if (e.i) *e.i = (int)v;
            else if (e.b) *e.b = v != 0.0;
            else std::snprintf(e.text, 256, "%s", value.c_str());
        }
    }
    s.gridN = std::clamp(s.gridN, 2, 80);
    s.ballCount = std::clamp(s.ballCount, 1, 1000);
    s.trailPoints = std::clamp(s.trailPoints, 32, 20000);
    s.timeScale = std::clamp(s.timeScale, kMinSpeed, 10.0f);
    s.domainR = std::clamp(s.domainR, 0.5f, 200.0f);
    s.method = std::clamp(s.method, 0, 2);
    s.systemLatexSize = std::clamp(s.systemLatexSize, 10.0f, 120.0f);
    s.maxStep = std::clamp(s.maxStep, 1e-5f, 0.5f);
    s.viewScale = std::clamp(s.viewScale, 1e-3f, 1e5f);
    s.followSeconds = std::clamp(s.followSeconds, 0.0f, 10.0f);
}

// ---- the sim --------------------------------------------------------------------

// Any typed vector field on R2 or R3: arrow glyphs, x' = f(x) typeset from the field
// itself, a ball of trajectories and click-dropped singles labelled phi^t(seed).
// See obsidian_twin/sims/vector-field.md.
class VectorFieldSim : public engine::Sim {
public:
    const char* name() const override { return "vector_field"; }

    ~VectorFieldSim() override
    {
        // R2 borrows the movement keys from the camera; hand them back
        if (app)
            app->camera().inputEnabled = true;
    }

    void init(engine::App& app) override
    {
        this->app = &app;
        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(SIMS_DIR "vector_field/shaders/vertex.glsl",
                                SIMS_DIR "vector_field/shaders/fragment.glsl");
        axisMesh = std::make_unique<engine::Mesh>(engine::Shapes::axisVertices(1000.0f),
                                                  engine::Shapes::axisIndices(),
                                                  std::vector<int>{3});

        // the field itself is the ODE right-hand side every trajectory integrates
        rhs = [this](const double* x, double t, double* dxdt) { field.eval(x, t, dxdt); };

        cfg = defaultSettings(2);
        stashed = defaultSettings(3);
        loadSettingsFile(cfg, kSettingsPath2D);
        loadSettingsFile(stashed, kSettingsPath3D);

        enterMode();
        applyField();
    }

    void update(float dt) override
    {
        handleViewKeys(dt);
        handleSeedClick();
        // Space toggles play/pause, unless ImGui has the keyboard (typing a field)
        if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false))
            playing = !playing;
        // arrows: Up/Down speed (down to 0), Left reverse, Right forward
        engine::FlowKeys::handle(cfg.timeScale, cfg.reverse, dt, 10.0f, kMinSpeed);
        engine::FlowKeys::drawStatus(cfg.timeScale, cfg.reverse, playing);

        integrator.method = (engine::Integrator::Method)cfg.method;
        integrator.maxStep = cfg.maxStep;

        if (playing)
        {
            double h = (double)dt * cfg.timeScale * (cfg.reverse ? -1.0 : 1.0);
            fieldTime += h;
            std::vector<engine::Trajectory*> trajs;
            trajs.reserve(curves.size());
            for (Curve& c : curves)
                trajs.push_back(c.traj.get());
            runner.advance(trajs, integrator, rhs, h);   // integrate (parallel) + upload
            if (field.usesTime())
                arrowsDirty = true;
        }

        if (dim == 3)
        {
            engine::Camera& cam = app->camera();
            cam.orbitAngularSpeed = cfg.orbitSpeed;
            cam.speed = cfg.zoomSpeed > 0.0f ? cfg.zoomSpeed
                                             : std::max(1.0f, cfg.domainR * 0.5f);
            followCamera(dt);
        }
        else
        {
            updateViewEase(dt);
            followCamera(dt);
            // the R2 glyph grid covers the visible window: pan, zoom or resize resamples
            glm::vec4 view(cfg.viewCenter[0], cfg.viewCenter[1], cfg.viewScale,
                           app->window().aspect());
            if (view != lastView2D)
            {
                lastView2D = view;
                arrowsDirty = true;
            }
        }

        if (arrowsDirty)
            rebuildArrows();
        rebuildTangents();   // after the camera moved: R3 sizes them by its distance

        drawSystemOverlay();
        drawTimeLabels();
        drawTangentLabels();

        // 1 tapped: camera follows a single on/off — detaching leaves the camera where
        // it is (held past App::kRecenterHoldSeconds, App calls recenterCamera instead);
        // 2: arrow field on/off
        if (!ImGui::GetIO().WantCaptureKeyboard &&
            (engine::FlowKeys::tapHold(ImGuiKey_1, dt, engine::App::kRecenterHoldSeconds).tapped ||
             engine::FlowKeys::tapHold(ImGuiKey_Keypad1, dt, engine::App::kRecenterHoldSeconds).tapped))
        {
            cfg.followSingle = !cfg.followSingle;
            if (!cfg.followSingle)
            {
                following = false; // in place: no return to the saved centre
                followedTraj = nullptr;
            }
        }
        if (!ImGui::GetIO().WantCaptureKeyboard && (ImGui::IsKeyPressed(ImGuiKey_2, false) || ImGui::IsKeyPressed(ImGuiKey_Keypad2, false)))
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
        if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_R, false))
            curves.clear();
        // B: drop a ball / disk of trajectories (same as the button)
        if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_B, false))
            dropBall(glm::vec3(cfg.ballCenter[0], cfg.ballCenter[1],
                               dim == 3 ? cfg.ballCenter[2] : 0.0f),
                     cfg.ballRadius, cfg.ballWithSingle, true);
        // V: a ball with a single at its centre, where a click would seed
        if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_V, false))
        {
            glm::vec3 p;
            if (cursorSeedPoint(p))
                dropBall(p, viewFitRadius(p), true, false);
        }

        // 0: supersampled screenshot of the scene alone (no panel, no LaTeX labels)
        if (!ImGui::GetIO().WantCaptureKeyboard &&
            (ImGui::IsKeyPressed(ImGuiKey_0, false) || ImGui::IsKeyPressed(ImGuiKey_Keypad0, false)))
            takeScreenshot();
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view, projection;
        viewProjection(camera, view, projection);
        // > 1 while App renders a screenshot at a multiple of the window size
        const float px = app->renderScale();

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // R2 is one plane: depth would only make same-z things fight, so draw order
        // decides (arrows, then each trail, then its head)
        if (dim == 2)
            glDisable(GL_DEPTH_TEST);

        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setFloat("u_magScale", 1.0f / std::max(colorMax(), 1e-6f));
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

        if (arrowsFade > 0.0f && (!glyphs.empty() || !glyphsFine.empty()))
        {
            // glyphs blend but never write depth, so they cannot hide a trajectory
            glDepthMask(GL_FALSE);
            glLineWidth(1.5f * px);
            auto drawBatch = [&](const engine::ArrowGlyphs& batch, float alphaMul) {
                if (batch.empty() || alphaMul <= 0.0f)
                    return;
                shader.setInt("u_useRamp", 1);
                shader.setInt("u_useFade", 1);
                shader.setFloat("u_alpha", cfg.arrowAlpha * alphaMul * arrowsFade);
                shader.setFloat("u_alphaTail", cfg.arrowTailAlpha * alphaMul);
                batch.drawShafts(shader);
                shader.setInt("u_useFade", 0);
                shader.setInt("u_shadeMode", 1);
                batch.drawHeads(shader);
                shader.setInt("u_shadeMode", 0);
            };
            drawBatch(glyphs, 1.0f);
            drawBatch(glyphsFine, fineAlpha); // R2 only: the next level fading in
            shader.setInt("u_useRamp", 0);
            shader.setInt("u_useFade", 0);
            shader.setFloat("u_alpha", 1.0f);
            shader.setFloat("u_alphaTail", 1.0f);
            glDepthMask(GL_TRUE);
        }

        runner.beginDraw();
        for (const Curve& c : curves)
        {
            // singles are the ones being studied: thicker trail and head
            float k = c.kind == Kind::Single ? cfg.singleScale : 1.0f;
            if (cfg.showTrails)
            {
                glLineWidth(cfg.trailWidth * k * px);
                shader.setVec3("u_color", c.color);
                shader.setInt("u_useRamp", cfg.trailBySpeed ? 1 : 0);
                shader.setFloat("u_alpha", cfg.trailAlpha);
                c.traj->draw();
                shader.setFloat("u_alpha", 1.0f);
                shader.setInt("u_useRamp", 0);
            }
            shader.setVec3("u_color", c.traj->diverged() ? kDivergedColor : c.color);
            shader.setFloat("u_pointSize", cfg.particleSize * k * px);
            shader.setInt("u_round", 1);
            c.traj->drawHead();
            shader.setInt("u_round", 0);
        }
        runner.endDraw();
        shader.setFloat("u_pointSize", cfg.particleSize * px);

        if (!tangents.empty())
        {
            shader.setInt("u_useRamp", 0);
            shader.setVec3("u_color", kTangentColor);
            glLineWidth(3.0f * px);
            tangents.drawShafts(shader);
            shader.setInt("u_shadeMode", 1);
            tangents.drawHeads(shader);
            shader.setInt("u_shadeMode", 0);
        }
        glLineWidth(1.0f);

        if (dim == 2)
            glEnable(GL_DEPTH_TEST);
        glDisable(GL_BLEND);
    }

    // Hold 1 (App::kRecenterHoldSeconds): glide to the home view, follow off — where
    // "Reset view" / "Reset camera" snap to.
    void recenterCamera() override
    {
        cfg.followSingle = false;
        following = false;
        followedTraj = nullptr;
        if (dim == 2)
        {
            viewEaseFrom = glm::vec3(cfg.viewCenter[0], cfg.viewCenter[1], cfg.viewScale);
            viewEaseTo = glm::vec3(0.0f, 0.0f, 5.0f);
            viewEaseT = 0.0f;
            viewEaseDuration = engine::App::kRecenterEaseSeconds;
            return;
        }
        cfg.camRadius = 0.0f;
        cfg.camTheta = 0.6f;
        cfg.camPhi = 1.1f;
        cfg.camCenter[0] = cfg.camCenter[1] = cfg.camCenter[2] = 0.0f;
        app->camera().easeOrbitTo(glm::vec3(0.0f), 3.0f * cfg.domainR, 0.6f, 1.1f,
                                  engine::App::kRecenterEaseSeconds);
    }

    // R2 counterpart of Camera::easeOrbitTo for the pan/zoom window (scale in log space).
    void updateViewEase(float dt)
    {
        if (viewEaseT >= viewEaseDuration)
            return;
        viewEaseT += dt;
        const float t = std::clamp(viewEaseT / viewEaseDuration, 0.0f, 1.0f);
        const float s = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
        cfg.viewCenter[0] = glm::mix(viewEaseFrom.x, viewEaseTo.x, s);
        cfg.viewCenter[1] = glm::mix(viewEaseFrom.y, viewEaseTo.y, s);
        cfg.viewScale = viewEaseFrom.z * std::pow(viewEaseTo.z / viewEaseFrom.z, s);
        if (t >= 1.0f)
            viewEaseDuration = 0.0f;
    }

    void drawUI() override
    {
        drawFieldUI();
        drawOverlayUI();
        drawFlowUI();
        drawBallUI();
        drawSinglesUI();

        // rarely-touched look-and-feel knobs, folded away until asked for
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.36f, 0.30f, 1.0f));
        bool showOther = ImGui::CollapsingHeader("Other settings (camera, integrator, LaTeX, arrows, trails, performance)");
        ImGui::PopStyleColor();
        if (showOther)
        {
            ImGui::Indent();
            drawViewUI();
            drawIntegratorUI();
            drawLatexUI();
            drawArrowUI();
            drawTrailUI();
            sectionHeader("Performance");
            runner.drawUI();
            ImGui::Unindent();
        }

        sectionHeader(dim == 2 ? "Settings (R2)" : "Settings (R3)");
        if (ImGui::Button(dim == 2 ? "Save R2 settings" : "Save R3 settings"))
            settingsStatus = saveCurrentSettings() ? "saved" : "could not write settings file";
        ImGui::SameLine();
        ImGui::TextDisabled("(field, controls and %s)", dim == 2 ? "view" : "camera");
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
        std::string path = std::string(SIMS_DIR "../screenshots/vector_field_") +
                           (dim == 2 ? "r2_" : "r3_") + stamp + ".png";
        shotStatus = app->captureScreenshot(path, shotScale)
            ? "saved screenshots/" + std::filesystem::path(path).filename().string()
            : "screenshot failed (see console)";
    }

    // Ball: the one ball (R2: disk) of seeds. Single: click-dropped, fatter, labelled.
    enum class Kind { Ball, Single };

    struct Curve {
        std::unique_ptr<engine::Trajectory> traj;
        glm::vec3 seed;
        double t0;                     // field time it was seeded at: phi's t = time - t0
        glm::vec3 color;
        Kind kind;
        engine::LatexImage seedLabel;  // singles: "(x, y[, z])" of the seed, typeset once
        float seedLabelSize = 0.0f;
        bool hasTangent = false;       // singles: tangent arrow drawn this frame
        glm::vec3 tangentTip{0.0f};
        bool withBall = false;         // the single at the ball's centre: goes with it
        int ball = -1;                 // drop id of the ball it belongs to (-1: none)
    };

    // Pieces of the \phi^t(seed) label, each typeset once and re-composed every frame,
    // so a t that changes every frame costs no typesetting (a full re-typeset is ~3 ms).
    struct TimeLabelGlyphs {
        float size = 0.0f;
        engine::LatexImage phi;
        engine::LatexImage seedSymbol; // (x_0), when the seed's values are hidden
        engine::LatexImage chars[12];  // superscript pieces, in kTimeLabelChars order
    };
    static constexpr const char* kTimeLabelChars = "0123456789.-";

    // ---- UI sections ------------------------------------------------------------

    void drawFieldUI()
    {
        sectionHeader("Vector field");
        int wanted = dim;
        ImGui::RadioButton("R2", &wanted, 2);
        ImGui::SameLine();
        ImGui::RadioButton("R3", &wanted, 3);
        ImGui::SameLine();
        ImGui::TextDisabled("(each keeps its own field, view and settings)");
        if (wanted != dim)
            switchMode(wanted);

        bool apply = false;
        const char* labels[3] = {"dx/dt", "dy/dt", "dz/dt"};
        for (int i = 0; i < dim; ++i)
        {
            ImGui::SetNextItemWidth(260.0f);
            apply |= ImGui::InputText(labels[i], cfg.comps[i], sizeof(cfg.comps[i]),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
        }
        if (ImGui::Button("Apply"))
            apply = true;
        ImGui::SameLine();
        ImGui::SetNextItemWidth(170.0f);
        if (ImGui::BeginCombo("preset", "choose..."))
        {
            const Preset* presets = dim == 2 ? kPresets2D : kPresets3D;
            int n = dim == 2 ? (int)std::size(kPresets2D) : (int)std::size(kPresets3D);
            for (int i = 0; i < n; ++i)
                if (ImGui::Selectable(presets[i].name))
                {
                    for (int k = 0; k < 3; ++k)
                        std::snprintf(cfg.comps[k], sizeof(cfg.comps[k]), "%s",
                                      presets[i].comps[k]);
                    apply = true;
                }
            ImGui::EndCombo();
        }
        if (apply)
            applyField();

        if (!parseError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", parseError.c_str());
        ImGui::PushTextWrapPos(0.0f);
        ImGui::TextDisabled("variables x, y%s, t   constants pi, e, tau   functions sin cos "
                            "tan sinh cosh tanh exp log sqrt abs   implicit products: 2x, "
                            "x y, (x-1)(x+1)",
                            dim == 3 ? ", z" : "");
        ImGui::PopTextWrapPos();
        ImGui::TextDisabled("field time t = %.3f", fieldTime);
    }

    void drawOverlayUI()
    {
        sectionHeader("LaTeX overlays");
        ImGui::Checkbox("show x' = f(x)", &cfg.showSystemLatex);
        ImGui::SameLine();
        ImGui::Checkbox("label singles with phi^t(x0)", &cfg.showTimeLabels);
        if (!systemLatex.error().empty())
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "LaTeX: %s",
                               systemLatex.error().c_str());
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
        sectionHeader(dim == 2 ? "Disk of trajectories" : "Ball of trajectories");
        ImGui::SetNextItemWidth(220.0f);
        if (dim == 2)
            ImGui::InputFloat2("center", cfg.ballCenter);
        else
            ImGui::InputFloat3("center", cfg.ballCenter);
        ImGui::SliderFloat("radius", &cfg.ballRadius, 0.01f, dim == 2 ? 5.0f : 20.0f, "%.2f");
        ImGui::SliderInt("particles", &cfg.ballCount, 1, 1000);
        ImGui::Checkbox(dim == 2 ? "fill disk" : "fill volume", &cfg.ballFilled);
        ImGui::SameLine();
        ImGui::Checkbox("single at center", &cfg.ballWithSingle);
        if (ImGui::Button(dim == 2 ? "Drop disk" : "Drop ball"))
            dropBall(glm::vec3(cfg.ballCenter[0], cfg.ballCenter[1],
                               dim == 3 ? cfg.ballCenter[2] : 0.0f),
                     cfg.ballRadius, cfg.ballWithSingle, true);
        ImGui::SameLine();
        if (ImGui::Button(dim == 2 ? "Remove disk" : "Remove ball"))
            removeBall();
    }

    void drawSinglesUI()
    {
        sectionHeader("Single trajectories");
        ImGui::TextDisabled("click in the view to drop one under the cursor");
        if (dim == 3)
            ImGui::SliderFloat("click depth", &cfg.spawnDepth, 0.05f, 1.0f,
                               "%.2f of the way to the orbit center");
        ImGui::SliderFloat("singles thickness", &cfg.singleScale, 1.0f, 4.0f, "%.1f x");
        ImGui::Checkbox("show x0 values", &cfg.showSeedValues);
        ImGui::SameLine();
        ImGui::TextDisabled(cfg.showSeedValues ? "phi^t(1, 2)" : "phi^t(x0)");
        ImGui::Checkbox("tangent vector x'", &cfg.showTangents);
        if (cfg.showTangents)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140.0f);
            ImGui::SliderFloat("##tangentSize", &cfg.tangentSize, 0.02f, 0.5f, "length %.2f");
        }
        if (ImGui::Button("Clear singles"))
            removeKind(Kind::Single);

        {
            int singles = countSingles();
            if (ImGui::Checkbox(dim == 3 ? "camera follows a single" : "view follows a single",
                                &cfg.followSingle) &&
                !cfg.followSingle)
                releaseFollow();
            if (cfg.followSingle)
            {
                ImGui::SameLine();
                ImGui::Checkbox("newest", &cfg.followNewest);
                if (!cfg.followNewest && singles > 0)
                {
                    int shown = std::clamp(cfg.followIndex, 0, singles - 1) + 1;
                    if (ImGui::SliderInt("follow #", &shown, 1, singles))
                        cfg.followIndex = shown - 1;
                }
                const char* help = dim == 3 ? "A/D/W/S orbit and Q/E zoom around the particle"
                                            : "Q/E zoom (panning is off while following)";
                ImGui::TextDisabled("%s", singles ? help : "(drop a single to follow it)");
            }
        }
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
        track(ImGui::SliderInt("grid N", &cfg.gridN, 2, dim == 2 ? 80 : 50));
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
        if (dim == 3)
            track(ImGui::DragFloat("domain radius", &cfg.domainR, 0.2f, 0.5f, 200.0f));
        ImGui::SliderFloat("arrow alpha", &cfg.arrowAlpha, 0.05f, 1.0f);
        ImGui::SliderFloat("alpha gradient", &cfg.arrowTailAlpha, 0.0f, 1.0f, "tail %.2f of tip");
        if (editedDone || (editedLive && glyphs.arrows() <= kLiveRebuildArrows))
            arrowsDirty = true;

        ImGui::Checkbox("auto color scale", &cfg.autoColorScale);
        if (!cfg.autoColorScale)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            ImGui::DragFloat("max |f|", &cfg.manualColorMax, 0.05f, 1e-3f, 1e6f);
        }
        ImGui::TextDisabled("sampled max |f| = %.4g   arrows = %d", sampledMaxMag,
                            glyphs.arrows() + glyphsFine.arrows());
        ImGui::TextDisabled("last rebuild %.2f ms (parallel sampling, instanced glyphs)", rebuildMs);
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

    void drawIntegratorUI()
    {
        sectionHeader("Integrator");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::Combo("method", &cfg.method, "Euler\0RK2 (midpoint)\0RK4\0");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::DragFloat("max step h", &cfg.maxStep, 0.0005f, 1e-5f, 0.5f, "%.4f");
        cfg.maxStep = std::clamp(cfg.maxStep, 1e-5f, 0.5f);
    }

    // Sizes of everything typeset: the equation top-right, and the labels on singles
    // (phi^t(x0) and the x' tangent label share one size).
    void drawLatexUI()
    {
        sectionHeader("LaTeX");
        if (ImGui::SliderFloat("equation", &cfg.systemLatexSize, 14.0f, 72.0f, "%.0f px"))
            latexDirty = true;
        ImGui::SliderFloat("labels", &cfg.timeLabelSize, 14.0f, 72.0f, "%.0f px");
    }

    void drawViewUI()
    {
        sectionHeader(dim == 2 ? "View" : "Camera");
        if (dim == 2)
        {
            ImGui::TextDisabled("WASD pan, Q/E zoom");
            ImGui::DragFloat2("view center", cfg.viewCenter, 0.05f);
            ImGui::DragFloat("half height", &cfg.viewScale, 0.05f, 1e-3f, 1e5f, "%.3f");
            ImGui::SliderFloat("follow ease", &cfg.followSeconds, 0.0f, 3.0f, "%.2f s");
            if (ImGui::Button("Reset view"))
            {
                cfg.viewCenter[0] = cfg.viewCenter[1] = 0.0f;
                cfg.viewScale = 5.0f;
            }
        }
        else
        {
            ImGui::TextDisabled("A/D orbit, W/S tilt, Q/E zoom");
            ImGui::SliderFloat("orbit speed", &cfg.orbitSpeed, 0.1f, 4.0f, "%.2f rad/s");
            ImGui::SliderFloat("zoom speed", &cfg.zoomSpeed, 0.0f,
                               std::max(20.0f, cfg.domainR * 2.0f),
                               cfg.zoomSpeed > 0.0f ? "%.1f /s" : "auto");
            ImGui::SliderFloat("follow ease", &cfg.followSeconds, 0.0f, 3.0f, "%.2f s");
            if (ImGui::Button("Reset camera"))
            {
                cfg.camRadius = 0.0f;
                cfg.camTheta = 0.6f;
                cfg.camPhi = 1.1f;
                cfg.camCenter[0] = cfg.camCenter[1] = cfg.camCenter[2] = 0.0f;
                applyCamera();
            }
        }
    }

    // ---- follow camera + tangent vectors ---------------------------------------------

    int countSingles() const
    {
        int n = 0;
        for (const Curve& c : curves)
            n += c.kind == Kind::Single ? 1 : 0;
        return n;
    }

    const Curve* followedSingle() const
    {
        int n = countSingles();
        if (!cfg.followSingle || n == 0)
            return nullptr;
        int want = cfg.followNewest ? n - 1 : std::clamp(cfg.followIndex, 0, n - 1);
        for (const Curve& c : curves)
            if (c.kind == Kind::Single && want-- == 0)
                return &c;
        return nullptr;
    }

    // R3: orbit around the followed head. The orbit offset (set by the camera's angles
    // and radius) is kept and the position moved with the centre in the same frame, so
    // the view neither lags nor stops following while ImGui has the keyboard.
    // R2: the pan/zoom window is centred on the head (zoom stays yours).
    // Whenever the followed single changes (a new one dropped, follow switched on, the
    // follow # moved) the centre slides onto the head over `followSeconds`: the camera
    // is attached to the new head at once (so it already rides its motion) plus the
    // offset it had at that moment, which decays with a cubic ease-out — full speed
    // away from the old spot, gentle arrival. A new single mid-slide restarts from the
    // current centre, so there is never a jump.
    void followCamera(float dt)
    {
        const Curve* c = followedSingle();
        if (!c)
        {
            following = false; // nothing to follow (e.g. singles cleared): panning is back
            followedTraj = nullptr;
            return;
        }
        glm::vec3 centre = dim == 2 ? glm::vec3(cfg.viewCenter[0], cfg.viewCenter[1], 0.0f)
                                    : app->camera().orbitCenter;
        glm::vec3 head = c->traj->head();
        if (c->traj.get() != followedTraj)
        {
            followedTraj = c->traj.get();
            easeOffset = centre - head;
            easeT = 0.0f;
        }
        if (easeT < cfg.followSeconds)
        {
            easeT += dt;
            float u = 1.0f - std::clamp(easeT / cfg.followSeconds, 0.0f, 1.0f);
            centre = head + easeOffset * (u * u * u); // ease-out: offset -> 0
        }
        else
            centre = head;

        if (dim == 2)
        {
            cfg.viewCenter[0] = centre.x;
            cfg.viewCenter[1] = centre.y;
            following = true;
            return;
        }
        engine::Camera& cam = app->camera();
        glm::vec3 offset = cam.position - cam.orbitCenter;
        cam.orbitCenter = centre;
        cam.position = centre + offset;
        cam.front = glm::normalize(-offset);
        following = true;
    }


    // Back to orbiting the saved centre, same angles and distance.
    void releaseFollow()
    {
        if (!following)
            return;
        following = false;
        followedTraj = nullptr;
        if (dim == 2)
            return; // the window simply stays where the particle left it
        engine::Camera& cam = app->camera();
        glm::vec3 offset = cam.position - cam.orbitCenter;
        cam.orbitCenter = glm::vec3(cfg.camCenter[0], cfg.camCenter[1], cfg.camCenter[2]);
        cam.position = cam.orbitCenter + offset;
    }

    // A red x' = f(x) arrow out of every single's head, direction of the flow there.
    // Constant size on screen: a fraction of the camera distance (R3) or of the view
    // half-height (R2). Rebuilt every frame — a handful of arrows.
    void rebuildTangents()
    {
        tangents.clear();
        for (Curve& c : curves)
            c.hasTangent = false;
        if (!cfg.showTangents || !field.valid())
        {
            tangents.upload();
            return;
        }

        const glm::vec3 eye = app->camera().position;
        for (Curve& c : curves)
        {
            c.hasTangent = false;
            if (c.kind != Kind::Single || c.traj->diverged())
                continue;
            glm::vec3 p = c.traj->head();
            double x[3] = {p.x, p.y, p.z}, v[3] = {0.0, 0.0, 0.0};
            field.eval(x, c.traj->time(), v);
            glm::vec3 vec((float)v[0], (float)v[1], dim == 3 ? (float)v[2] : 0.0f);
            float mag = glm::length(vec);
            if (!(mag > 1e-9f) || !std::isfinite(mag))
                continue; // at an equilibrium there is no direction to draw
            glm::vec3 dir = vec / mag;
            // constant on screen, but capped in world units so a far-out view does not
            // grow the arrow past the scene itself
            float len = cfg.tangentSize * (dim == 2 ? cfg.viewScale : glm::length(eye - p));
            len = std::min(len, dim == 2 ? kTangentMaxLen2D : kTangentMaxDomainFraction * cfg.domainR);
            tangents.add(p, dir, len, mag, dim);
            c.hasTangent = true;
            c.tangentTip = p + dir * len;
        }
        tangents.upload();
    }

    // \dot{x} just beyond each tangent arrow's tip, continuing the arrow's on-screen
    // direction, so it never sits on the arrowhead. Same size as the phi label, red,
    // typeset once.
    void drawTangentLabels()
    {
        if (!cfg.showTangents)
            return;
        float size = cfg.timeLabelSize;
        if (tangentLabelSize != size)
        {
            tangentLabelSize = size;
            tangentLabel.set(R"(\dot{x})", size, kTangentColor);
        }
        for (const Curve& c : curves)
        {
            float hx, hy, tx, ty;
            if (!c.hasTangent || !toScreen(c.traj->head(), hx, hy) ||
                !toScreen(c.tangentTip, tx, ty))
                continue;
            glm::vec2 d(tx - hx, ty - hy);
            float len = glm::length(d);
            d = len > 1e-3f ? d / len : glm::vec2(1.0f, 0.0f);
            // push the label's centre out until its box clears the tip in direction d
            float w = (float)tangentLabel.width(), h = (float)tangentLabel.height();
            float reach = 0.5f * (std::abs(d.x) * w + std::abs(d.y) * h) + 2.0f;
            tangentLabel.drawAt(tx + d.x * reach - 0.5f * w, ty + d.y * reach - 0.5f * h);
        }
    }

    // ---- overlays -------------------------------------------------------------------

    // World point -> ImGui screen pixels; false when behind the camera / out of range.
    bool toScreen(const glm::vec3& p, float& sx, float& sy) const
    {
        glm::mat4 view, projection;
        viewProjection(app->camera(), view, projection);
        glm::vec4 clip = projection * view * glm::vec4(p, 1.0f);
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

    // x' = f(x) top-right, typeset from the parsed field itself (Expression::latex), so
    // it shows exactly what is being integrated. Re-typeset only when the field changes.
    void drawSystemOverlay()
    {
        if (!cfg.showSystemLatex || !field.valid())
            return;
        if (latexDirty)
        {
            latexDirty = false;
            const char* dots[3] = {R"(\dot{x})", R"(\dot{y})", R"(\dot{z})"};
            std::vector<std::string> f = field.latex();
            std::string lhs = R"(\begin{pmatrix})", rhsTex = R"(\begin{pmatrix})";
            for (int i = 0; i < dim; ++i)
            {
                const char* sep = i + 1 < dim ? R"( \\ )" : "";
                lhs += std::string(dots[i]) + sep;
                rhsTex += f[i] + sep;
            }
            systemLatex.set(lhs + R"(\end{pmatrix} = )" + rhsTex + R"(\end{pmatrix})", cfg.systemLatexSize);
        }
        if (systemLatex.empty())
            return;

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12.0f,
                                       vp->WorkPos.y + 12.0f),
                                ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.35f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##fieldLatex", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
        systemLatex.draw();
        ImGui::End();
        ImGui::PopStyleVar();
    }

    // \phi^t(seed) above every single particle: the flow map applied to its seed for the
    // ODE time integrated since it was dropped (negative when running in reverse).
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

        for (Curve& c : curves)
        {
            if (c.kind != Kind::Single)
                continue;
            if (c.seedLabelSize != S)
            {
                c.seedLabelSize = S;
                std::string seed = "(" + texNumber(c.seed.x) + ", " + texNumber(c.seed.y);
                if (dim == 3)
                    seed += ", " + texNumber(c.seed.z);
                c.seedLabel.set(seed + ")", S, kSingleColor);
            }

            float sx, sy;
            if (!toScreen(c.traj->head(), sx, sy))
                continue;

            double t = c.traj->time() - c.t0;
            if (std::abs(t) < 0.005)
                t = 0.0; // no "-0.00"
            char num[32];
            std::snprintf(num, sizeof(num), "%.2f", t);

            // one baseline: \phi, raised digits, (seed)
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
            place(cfg.showSeedValues ? c.seedLabel : g.seedSymbol, 0.0f);

            // above the particle, unless its tangent arrow points up on screen — then
            // below, so the label and the arrow never cross
            float headRadius = 0.5f * cfg.particleSize * cfg.singleScale;
            float tx, ty;
            bool arrowUp = c.hasTangent && toScreen(c.tangentTip, tx, ty) && ty < sy;
            float left = sx - 0.5f * x;
            float baseY = arrowUp ? sy + headRadius + 0.95f * S
                                  : sy - headRadius - 0.35f * S;
            for (int i = 0; i < n; ++i)
                pieces[i]->drawAt(left + xs[i], baseY + ys[i]);
        }
    }

    // ---- field, modes, view -----------------------------------------------------------

    // Parse the typed components. A parse error keeps the previous field drawing.
    void applyField()
    {
        std::vector<std::string> sources;
        for (int i = 0; i < dim; ++i)
            sources.push_back(cfg.comps[i]);
        if (field.set(dim, sources, parseError))
        {
            parseError.clear();
            arrowsDirty = true;
            latexDirty = true;
        }
    }

    // R2 <-> R3: each mode keeps its own Settings, swapped in and out; curves do not
    // survive (a curve in R2 is not a curve in R3).
    void switchMode(int newDim)
    {
        following = false;   // enterMode() re-applies the R3 camera from settings
        followedTraj = nullptr;
        std::swap(cfg, stashed);
        dim = newDim;
        curves.clear();
        fieldTime = 0.0;
        settingsStatus.clear();
        enterMode();
        applyField();
    }

    // R3 orbits the shared camera; R2 drops it for an orthographic pan/zoom window and
    // borrows the movement keys (the camera stops listening while in R2).
    void enterMode()
    {
        engine::Camera& cam = app->camera();
        if (dim == 3)
        {
            cam.inputEnabled = true;
            cam.up = glm::vec3(0.0f, 1.0f, 0.0f);
            cam.orbitRadiusMin = 0.1f;
            cam.orbitSlowZone = 0.0f;
            cam.farPlane = std::max(500.0f, 20.0f * cfg.domainR);
            applyCamera();
        }
        else
        {
            cam.inputEnabled = false;
        }
        lastView2D = glm::vec4(0.0f);
        arrowsDirty = true;
    }

    void applyCamera()
    {
        engine::Camera& cam = app->camera();
        cam.orbitTheta = cfg.camTheta;
        cam.orbitPhi = cfg.camPhi;
        cam.orbitRadius = cfg.camRadius > 0.0f ? cfg.camRadius : 3.0f * cfg.domainR;
        cam.orbitCenter = glm::vec3(cfg.camCenter[0], cfg.camCenter[1], cfg.camCenter[2]);
    }

    bool saveCurrentSettings()
    {
        if (dim == 3)
        {
            // the orbit view lives in the camera; copy it in before writing
            const engine::Camera& cam = app->camera();
            cfg.camTheta = cam.orbitTheta;
            cfg.camPhi = cam.orbitPhi;
            cfg.camRadius = cam.orbitRadius;
            cfg.camCenter[0] = cam.orbitCenter.x;
            cfg.camCenter[1] = cam.orbitCenter.y;
            cfg.camCenter[2] = cam.orbitCenter.z;
        }
        return saveSettingsFile(cfg, dim == 2 ? kSettingsPath2D : kSettingsPath3D);
    }

    void viewProjection(const engine::Camera& camera, glm::mat4& view,
                        glm::mat4& projection) const
    {
        if (dim == 3)
        {
            view = camera.getViewMatrix();
            projection = camera.getProjectionMatrix();
            return;
        }
        float sx = cfg.viewScale * app->window().aspect(), sy = cfg.viewScale;
        view = glm::mat4(1.0f);
        projection = glm::ortho(cfg.viewCenter[0] - sx, cfg.viewCenter[0] + sx,
                                cfg.viewCenter[1] - sy, cfg.viewCenter[1] + sy, -1.0f, 1.0f);
    }

    // R2 pan/zoom on WASD + QE, in view units so panning slows as you zoom in.
    void handleViewKeys(float dt)
    {
        if (dim != 2 || ImGui::GetIO().WantCaptureKeyboard)
            return;
        const engine::Window& win = app->window();
        float pan = cfg.viewScale * dt;
        for (char k : {'A', 'D', 'S', 'W', 'Q', 'E'})
            if (win.keyDown(k))
                viewEaseDuration = 0.0f; // the user takes over mid-glide
        if (!following) // while following, the particle owns the centre; zoom stays
        {
            if (win.keyDown('A')) cfg.viewCenter[0] -= pan;
            if (win.keyDown('D')) cfg.viewCenter[0] += pan;
            if (win.keyDown('S')) cfg.viewCenter[1] -= pan;
            if (win.keyDown('W')) cfg.viewCenter[1] += pan;
        }
        if (win.keyDown('Q')) cfg.viewScale *= std::exp(-dt);
        if (win.keyDown('E')) cfg.viewScale *= std::exp(dt);
    }

    // Where a click seeds. R2: the point under the cursor. R3: under the cursor on a
    // camera-facing plane spawnDepth of the way to the orbit center, so it lands just in
    // front of you. False when the cursor ray misses that plane.
    bool cursorSeedPoint(glm::vec3& out) const
    {
        glm::vec2 ndc = app->window().cursorPosNormalized() * 2.0f - 1.0f;

        if (dim == 2)
        {
            glm::vec2 p = glm::vec2(cfg.viewCenter[0], cfg.viewCenter[1]) +
                          ndc * glm::vec2(cfg.viewScale * app->window().aspect(),
                                          cfg.viewScale);
            out = glm::vec3(p, 0.0f);
            return true;
        }

        const engine::Camera& cam = app->camera();
        glm::mat4 invVP = glm::inverse(cam.getProjectionMatrix() * cam.getViewMatrix());
        glm::vec4 nearH = invVP * glm::vec4(ndc, -1.0f, 1.0f);
        glm::vec4 farH = invVP * glm::vec4(ndc, 1.0f, 1.0f);
        if (nearH.w == 0.0f || farH.w == 0.0f)
            return false;
        glm::vec3 origin = glm::vec3(nearH) / nearH.w;
        glm::vec3 dir = glm::normalize(glm::vec3(farH) / farH.w - origin);
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

    void handleSeedClick()
    {
        if (!ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::GetIO().WantCaptureMouse)
            return;
        glm::vec3 p;
        if (cursorSeedPoint(p))
            addTrajectory(p, kSingleColor, Kind::Single);
    }

    // ---- trajectories -------------------------------------------------------------

    void addTrajectory(const glm::vec3& seed, const glm::vec3& color, Kind kind)
    {
        if (!field.valid() || curves.size() >= kMaxTrajectories)
            return;
        double s[3] = {seed.x, seed.y, seed.z};
        auto traj = std::make_unique<engine::Trajectory>(dim, s, fieldTime, cfg.trailPoints);
        traj->minSpacing = 0.002f * (dim == 2 ? cfg.viewScale : cfg.domainR);
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

    // One ball (R2: disk) at a time; dropping replaces it. Seeds sit evenly on the
    // boundary (R2: circle, R3: Fibonacci sphere — the clearest view of the deformation)
    // or uniformly through the inside.
    // Radius for the ball V drops at p: a fixed fraction of the view's half-height at
    // p's depth, so the fresh ball fits on screen however far in or out you are.
    float viewFitRadius(const glm::vec3& p) const
    {
        if (dim == 2)
            return kViewBallFraction * cfg.viewScale;
        const engine::Camera& cam = app->camera();
        float depth = std::max(glm::dot(p - cam.position, cam.front), 1e-3f);
        return kViewBallFraction * depth * std::tan(glm::radians(0.5f * cam.fovDeg));
    }
    static constexpr float kViewBallFraction = 0.12f;
    static constexpr float kTangentMaxLen2D = 2.0f;           // world units (R2)
    static constexpr float kTangentMaxDomainFraction = 0.3f;  // of domainR (R3)

    // replace: the previous ball goes first (B, the button). Otherwise the new ball
    // joins the others and only the oldest beyond kMaxBalls is retired (V).
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
                    p = glm::vec3(unit(rng), unit(rng), dim == 3 ? unit(rng) : 0.0f);
                while (glm::dot(p, p) > 1.0f);
            }
            else if (dim == 2)
            {
                float th = 6.28318530718f * (float)k / (float)n;
                p = glm::vec3(std::cos(th), std::sin(th), 0.0f);
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

    // Every curve back to its seed, starting again at the current field time (a
    // time-dependent field flows differently from a different t).
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

    float colorMax() const
    {
        return cfg.autoColorScale ? std::max(sampledMaxMag, 1e-6f) : cfg.manualColorMax;
    }

    // Resample the field on the lattice (R2: the visible window, snapped so glyphs stay
    // put while panning; R3: the domain cube) and rebuild the glyph geometry.
    void rebuildArrows()
    {
        arrowsDirty = false;
        sampledMaxMag = 0.0f;
        glyphs.clear();
        if (!field.valid())
        {
            glyphs.upload();
            return;
        }

        gridPoints.clear();
        finePoints.clear();
        fineAlpha = 0.0f;
        float spacing;
        if (dim == 2)
        {
            // Arrows sit on points of the plane, not of the screen: the grid is anchored
            // at the origin with a power-of-two spacing, so zooming magnifies it like the
            // curves. The spacing is the first level at or above the target screen spacing
            // s (gridN rows); the next finer level -- the points between -- fades in as the
            // zoom approaches the point where it becomes the level itself (t -> 2), so
            // arrows appear between their neighbours instead of the whole grid resampling.
            float sx = cfg.viewScale * app->window().aspect(), sy = cfg.viewScale;
            spacing = 2.0f * sy / (float)cfg.gridN;                      // target, screen-constant
            const float level = std::exp2(std::ceil(std::log2(spacing))); // in [s, 2s)
            const float t = level / spacing;
            fineAlpha = glm::smoothstep(1.0f, 2.0f, t);

            const float fine = 0.5f * level;
            const int i0 = (int)std::floor((cfg.viewCenter[0] - sx) / fine) - 1;
            const int j0 = (int)std::floor((cfg.viewCenter[1] - sy) / fine) - 1;
            const int i1 = (int)std::ceil((cfg.viewCenter[0] + sx) / fine) + 1;
            const int j1 = (int)std::ceil((cfg.viewCenter[1] + sy) / fine) + 1;
            for (int j = j0; j <= j1; ++j)
                for (int i = i0; i <= i1; ++i)
                {
                    glm::vec3 p(i * fine, j * fine, 0.0f);
                    const bool onLevel = (i % 2 == 0) && (j % 2 == 0);
                    if (onLevel)
                        gridPoints.push_back(p);
                    else if (fineAlpha > 0.0f)
                        finePoints.push_back(p);
                }
        }
        else
        {
            float R = cfg.domainR;
            spacing = 2.0f * R / (float)cfg.gridN;
            for (int k = 0; k <= cfg.gridN; ++k)
                for (int j = 0; j <= cfg.gridN; ++j)
                    for (int i = 0; i <= cfg.gridN; ++i)
                        gridPoints.emplace_back(-R + i * spacing, -R + j * spacing,
                                                -R + k * spacing);
        }

        // every sample is independent, so the field is evaluated across all cores
        // (VectorField::eval is thread-safe); the max is a serial pass afterwards
        auto t0 = std::chrono::steady_clock::now();
        auto sample = [this](const std::vector<glm::vec3>& points, std::vector<glm::vec3>& out) {
            out.resize(points.size());
            std::transform(std::execution::par, points.begin(), points.end(), out.begin(),
                           [this](const glm::vec3& p) {
                               double x[3] = {p.x, p.y, p.z}, v[3] = {0.0, 0.0, 0.0};
                               field.eval(x, fieldTime, v);
                               glm::vec3 vec((float)v[0], (float)v[1],
                                             dim == 3 ? (float)v[2] : 0.0f);
                               if (!std::isfinite(vec.x) || !std::isfinite(vec.y)
                                   || !std::isfinite(vec.z))
                                   vec = glm::vec3(0.0f);
                               return vec;
                           });
            for (const glm::vec3& v : out)
                sampledMaxMag = std::max(sampledMaxMag, glm::length(v));
        };
        sample(gridPoints, gridVectors);
        sample(finePoints, fineVectors);

        // uniform length by default: direction is the glyph's job, magnitude the ramp's.
        // The length follows the screen-constant target spacing, so arrows keep their
        // pixel size across zoom levels and never overlap once the fine level is in.
        for (engine::ArrowGlyphs* g : {&glyphs, &glyphsFine})
        {
            g->barbs = cfg.barbedHeads;
            g->headFraction = cfg.headFraction;
            g->headWidth = cfg.headWidth;
        }
        glyphs.addField(gridPoints, gridVectors, spacing * cfg.arrowScale, colorMax(),
                        cfg.hideBelow, cfg.lengthByMagnitude, dim);
        glyphs.upload();
        glyphsFine.clear();
        glyphsFine.addField(finePoints, fineVectors, spacing * cfg.arrowScale, colorMax(),
                            cfg.hideBelow, cfg.lengthByMagnitude, dim);
        glyphsFine.upload();
        rebuildMs = std::chrono::duration<float, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();
    }

    static constexpr int kLiveRebuildArrows = 20000;
    static constexpr size_t kMaxTrajectories = 4000;

    engine::App* app = nullptr;
    engine::Shader shader;
    std::unique_ptr<engine::Mesh> axisMesh;
    engine::ArrowGlyphs glyphs;
    engine::ArrowGlyphs glyphsFine;    // R2: the next finer grid level, drawn at fineAlpha
    float arrowsFade = -1.0f;         // arrow-field opacity, follows showArrows (-1: unset)
    glm::vec3 viewEaseFrom{0.0f}, viewEaseTo{0.0f}; // R2 recenter glide: (cx, cy, scale)
    float viewEaseT = 0.0f, viewEaseDuration = 0.0f;
    static constexpr float kArrowFadeSeconds = 0.35f;
    static constexpr int kMaxBalls = 3;   // V stacks balls up to this many
    int nextBallId = 0;
    float fineAlpha = 0.0f;

    int dim = 2;
    Settings cfg;       // the active mode's settings
    Settings stashed;   // the other mode's, swapped in on an R2/R3 switch
    std::string settingsStatus;
    int shotScale = 2;
    std::string shotStatus;

    engine::VectorField field;
    std::string parseError;
    engine::Integrator integrator;
    engine::Integrator::Field rhs;
    std::vector<Curve> curves;
    double fieldTime = 0.0;
    bool playing = true;
    std::mt19937 rng{1234};
    engine::TrajectoryRunner runner;  // advances the curves, times integrate/upload/draw

    engine::LatexImage systemLatex;   // x' = f(x), top-right
    bool latexDirty = true;
    TimeLabelGlyphs timeGlyphs;

    engine::ArrowGlyphs tangents;      // x' arrows on singles, rebuilt every frame
    engine::LatexImage tangentLabel;   // \dot{x}
    float tangentLabelSize = 0.0f;
    bool following = false;            // the camera is currently tracking a single
    const engine::Trajectory* followedTraj = nullptr; // identity of the followed single
    glm::vec3 easeOffset{0.0f};        // centre - head when the slide onto followedTraj began
    float easeT = 1e9f;                // seconds into that slide (>= followSeconds: locked on)

    bool arrowsDirty = true;
    float sampledMaxMag = 0.0f;
    float rebuildMs = 0.0f;       // last glyph rebuild: sample + upload, shown in the UI
    std::vector<glm::vec3> gridPoints;
    std::vector<glm::vec3> gridVectors;
    std::vector<glm::vec3> finePoints;   // R2 fine level (points between gridPoints)
    std::vector<glm::vec3> fineVectors;
    glm::vec4 lastView2D{0.0f};   // R2 window the glyphs were sampled for
};

} // namespace

std::unique_ptr<engine::Sim> makeVectorFieldSim()
{
    return std::make_unique<VectorFieldSim>();
}
