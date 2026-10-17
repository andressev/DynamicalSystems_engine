#include "DiagonalLinearSim.h"

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

#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

struct Preset {
    const char* name;
    float lambda[3];
};

const Preset kPresets[] = {
    {"sink",               {-1.0f, -2.0f, -3.0f}},
    {"source",             { 1.0f,  2.0f,  3.0f}},
    {"saddle (1 up, 2 down)", { 1.0f, -1.0f, -2.0f}},
    {"saddle (2 up, 1 down)", { 1.0f,  0.5f, -1.5f}},
    {"degenerate (center line)", { 0.0f, -1.0f,  1.0f}},
};

const glm::vec3 kStableColor(0.25f, 0.45f, 1.0f);
const glm::vec3 kUnstableColor(1.0f, 0.25f, 0.22f);
const glm::vec3 kCenterColor(0.65f, 0.65f, 0.70f);

const glm::vec3 kBallColor(1.0f);

// click-dropped trajectories are always green — clear of red/blue (subspaces) and
// white (ball)
const glm::vec3 kSingleColor = {0.35f, 0.95f, 0.45f};
// seeds spawned on a circle in E^s: a lighter stable blue, so they read as "on E^s"
const glm::vec3 kManifoldColor = {0.60f, 0.72f, 1.0f};
// x' = Ax at a single's head
const glm::vec3 kTangentColor = {1.0f, 0.22f, 0.18f};

// UI section title: a red labelled separator with breathing room above, so the long
// panel reads as distinct blocks.
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

const char* kSettingsPath = SIMS_DIR "diagonal_linear/settings.txt";

// The diagonal linear system x' = diag(l1, l2, l3) x on R3: arrow glyphs, the stable
// (blue) and unstable (red) eigenspaces, and balls of trajectories dropped into the
// flow. See obsidian_twin/sims/diagonal-linear.md.
class DiagonalLinearSim : public engine::Sim {
public:
    const char* name() const override { return "diagonal_linear"; }

    void init(engine::App& app) override
    {
        this->app = &app;
        glEnable(GL_PROGRAM_POINT_SIZE);

        shader = engine::Shader(SIMS_DIR "diagonal_linear/shaders/vertex.glsl",
                                SIMS_DIR "diagonal_linear/shaders/fragment.glsl");

        axisMesh = std::make_unique<engine::Mesh>(engine::Shapes::axisVertices(1000.0f),
                                                  engine::Shapes::axisIndices(),
                                                  std::vector<int>{3});
        subspaceMesh = std::make_unique<engine::Mesh>(kSubspaceCapacity, std::vector<int>{3});

        rhs = [this](const double* x, double, double* dxdt) {
            for (int i = 0; i < 3; ++i)
                dxdt[i] = (double)lambda[i] * x[i];
        };

        loadSettings();

        engine::Camera& cam = app.camera();
        cam.inputEnabled = true;
        cam.orbitCenter = glm::vec3(0.0f);
        cam.orbitRadius = 3.0f * domainR;
        cam.orbitRadiusMin = 0.1f;
        cam.orbitSlowZone = 0.0f;
        cam.farPlane = std::max(100.0f, 20.0f * domainR);
        applySavedCamera(cam);

        rebuildSubspaces();
        rebuildArrows();
    }

    void update(float dt) override
    {
        handleSeedClick();
        // Space toggles play/pause, unless ImGui has the keyboard (typing a field)
        if (!ImGui::GetIO().WantCaptureKeyboard && ImGui::IsKeyPressed(ImGuiKey_Space, false))
            playing = !playing;
        // arrows: Up/Down speed (down to 0), Left reverse, Right forward
        engine::FlowKeys::handle(timeScale, reverse, dt, 5.0f);
        engine::FlowKeys::drawStatus(timeScale, reverse, playing);

        integrator.method = (engine::Integrator::Method)method;
        integrator.maxStep = maxStep;
        if (playing)
        {
            double h = (double)dt * timeScale * (reverse ? -1.0 : 1.0);
            std::vector<engine::Trajectory*> trajs;
            trajs.reserve(curves.size());
            for (Curve& c : curves)
                trajs.push_back(c.traj.get());
            runner.advance(trajs, integrator, rhs, h);   // integrate (parallel) + upload
        }

        app->camera().speed = std::max(1.0f, domainR);
        followCamera();

        if (subspacesDirty)
            rebuildSubspaces();
        if (arrowsDirty)
            rebuildArrows();
        rebuildTangents();   // after the camera moved: sized by its distance

        drawSystemOverlay();
        drawTimeLabels();
        drawTangentLabels();
        drawSubspaceLabels();
    }

    void render(const engine::Camera& camera) override
    {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        shader.use();
        shader.setMat4("view", camera.getViewMatrix());
        shader.setMat4("projection", camera.getProjectionMatrix());
        shader.setFloat("u_magScale", 1.0f / std::max(colorMax(), 1e-6f));
        shader.setFloat("u_pointSize", particleSize);
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_alphaTail", 1.0f);
        shader.setInt("u_useFade", 0);
        shader.setInt("u_shadeMode", 0);
        shader.setInt("u_round", 0);
        shader.setInt("u_useRamp", 0);

        if (showAxes)
        {
            shader.setVec3("u_color", glm::vec3(0.35f, 0.35f, 0.42f));
            glLineWidth(1.0f);
            axisMesh->draw(GL_LINES);
        }

        // subspace lines sit exactly on the axes; LEQUAL lets them win the depth tie
        if (showSubspaces)
        {
            glDepthFunc(GL_LEQUAL);
            glLineWidth(subspaceLineWidth);
            for (const SubspaceDraw& s : subspaceDraws)
            {
                shader.setVec3("u_color", s.color);
                subspaceMesh->drawArrays(GL_LINES, s.lineCount, s.lineFirst);
            }
            if (showSubspaceArrows)
            {
                shader.setInt("u_shadeMode", 1);
                for (const SubspaceDraw& s : subspaceDraws)
                {
                    if (subspaceCones[s.kind].empty())
                        continue;
                    shader.setVec3("u_color", s.color);
                    subspaceCones[s.kind].drawHeads(shader);
                }
                shader.setInt("u_shadeMode", 0);
            }
            glDepthFunc(GL_LESS);
        }

        if (showArrows && !glyphs.empty())
        {
            // glyphs blend but never write depth, so they cannot hide a trajectory
            glDepthMask(GL_FALSE);
            shader.setInt("u_useRamp", 1);
            shader.setInt("u_useFade", 1);
            shader.setFloat("u_alpha", arrowAlpha);
            shader.setFloat("u_alphaTail", arrowTailAlpha);
            glLineWidth(1.5f);
            glyphs.drawShafts(shader);
            shader.setInt("u_useFade", 0);
            shader.setInt("u_shadeMode", 1);
            glyphs.drawHeads(shader);
            shader.setInt("u_shadeMode", 0);

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
            float k = c.kind == Kind::Single ? singleScale : 1.0f;
            shader.setVec3("u_color", c.color);
            if (showTrails)
            {
                glLineWidth(trailWidth * k);
                shader.setFloat("u_alpha", trailAlpha);
                c.traj->draw();
                shader.setFloat("u_alpha", 1.0f);
            }
            shader.setFloat("u_pointSize", particleSize * k);
            shader.setInt("u_round", 1);
            c.traj->drawHead();
            shader.setInt("u_round", 0);
        }
        runner.endDraw();
        shader.setFloat("u_pointSize", particleSize);

        if (!tangents.empty())
        {
            shader.setInt("u_useRamp", 0);
            shader.setVec3("u_color", kTangentColor);
            glLineWidth(3.0f);
            tangents.drawShafts(shader);
            shader.setInt("u_shadeMode", 1);
            tangents.drawHeads(shader);
            shader.setInt("u_shadeMode", 0);
        }
        glLineWidth(1.0f);

        // translucent planes last, without depth writes, so everything shows through
        if (showSubspaces)
        {
            glDepthMask(GL_FALSE);
            shader.setFloat("u_alpha", planeAlpha);
            for (const SubspaceDraw& s : subspaceDraws)
            {
                if (s.triCount == 0)
                    continue;
                shader.setVec3("u_color", s.color);
                subspaceMesh->drawArrays(GL_TRIANGLES, s.triCount, s.triFirst);
            }
            shader.setFloat("u_alpha", 1.0f);
            glDepthMask(GL_TRUE);
        }

        glDisable(GL_BLEND);
    }

    void drawUI() override
    {
        drawSystemUI();
        drawOverlayUI();
        drawSubspaceUI();
        drawFlowUI();
        drawBallUI();
        drawManifoldCircleUI();
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
            drawGlyphUI();
            drawTrailUI();
            sectionHeader("Performance");
            runner.drawUI();
            ImGui::Unindent();
        }

        sectionHeader("Settings");
        if (ImGui::Button("Save settings"))
            settingsStatus = saveSettings() ? "saved" : "could not write settings file";
        ImGui::SameLine();
        ImGui::TextDisabled("(includes the camera view)");
        if (!settingsStatus.empty())
        {
            ImGui::SameLine();
            ImGui::TextDisabled("%s", settingsStatus.c_str());
        }
    }

private:
    // Ball: the one ball of seeds. Single: click-dropped, drawn fatter and labelled.
    // Manifold: the circle of seeds spawned in E^s.
    enum class Kind { Ball, Single, Manifold };

    struct Curve {
        std::unique_ptr<engine::Trajectory> traj;
        glm::vec3 seed;
        glm::vec3 color;
        Kind kind;
        engine::LatexImage seedLabel;  // singles: "(x, y, z)" of the seed, typeset once
        float seedLabelSize = 0.0f;    // size seedLabel was typeset at
        bool hasTangent = false;       // singles: tangent arrow drawn this frame
        glm::vec3 tangentTip{0.0f};
        bool withBall = false;         // the single at the ball's centre: goes with it
    };

    // Pieces of the \phi^t(x_0) label, each typeset once: the label is re-composed
    // from them every frame, so a t that changes every frame costs no typesetting
    // (a full re-typeset measured ~3 ms in Debug — per single, per frame).
    struct TimeLabelGlyphs {
        float size = 0.0f;
        engine::LatexImage phi;       // \phi
        engine::LatexImage chars[12]; // superscript pieces, in kTimeLabelChars order
    };
    static constexpr const char* kTimeLabelChars = "0123456789.-";
    struct SubspaceDraw {
        int lineFirst = 0, lineCount = 0;
        int triFirst = 0, triCount = 0;
        glm::vec3 color{1.0f};
        int kind = 0;                       // 0 = E^s, 1 = E^u, 2 = E^c
        glm::vec3 labelAnchor{0.0f};        // where its E^s/E^u/E^c label sits
    };

    // ---- UI -----------------------------------------------------------------

    // Sizes of everything typeset: the equation, the phi^t labels (and x'), E^s/E^u/E^c.
    void drawLatexUI()
    {
        sectionHeader("LaTeX");
        if (ImGui::SliderFloat("equation", &systemLatexSize, 14.0f, 72.0f, "%.0f px"))
            latexLambda[0] = NAN;   // re-typeset
        ImGui::SliderFloat("phi^t labels", &timeLabelSize, 14.0f, 72.0f, "%.0f px");
        ImGui::SliderFloat("E^s, E^u labels", &subspaceLabelSize, 14.0f, 140.0f, "%.0f px");
    }

    void drawViewUI()
    {
        sectionHeader("Camera");
        ImGui::TextDisabled("A/D orbit, W/S tilt, Q/E zoom");
        if (ImGui::Button("Reset camera"))
        {
            engine::Camera& cam = app->camera();
            cam.orbitCenter = glm::vec3(0.0f);
            cam.orbitRadius = 3.0f * domainR;
            cam.orbitTheta = 0.6f;
            cam.orbitPhi = 1.1f;
        }
    }

    void drawIntegratorUI()
    {
        sectionHeader("Integrator");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::Combo("method", &method, "Euler\0RK2 (midpoint)\0RK4\0");
        ImGui::SetNextItemWidth(160.0f);
        ImGui::DragFloat("max step h", &maxStep, 0.0005f, 1e-5f, 0.5f, "%.4f");
        maxStep = std::clamp(maxStep, 1e-5f, 0.5f);
    }

    // World point -> ImGui screen pixels; false when it is behind the camera or outside
    // the depth range.
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

    // E^s / E^u / E^c typeset once in each subspace's color, centered on its anchor.
    void drawSubspaceLabels()
    {
        if (!showSubspaces || !showSubspaceLabels)
            return;
        if (subspaceLabelsSize != subspaceLabelSize)
        {
            subspaceLabelsSize = subspaceLabelSize;
            subspaceLabels[0].set("E^s", subspaceLabelSize, kStableColor);
            subspaceLabels[1].set("E^u", subspaceLabelSize, kUnstableColor);
            subspaceLabels[2].set("E^c", subspaceLabelSize, kCenterColor);
        }
        for (const SubspaceDraw& d : subspaceDraws)
        {
            float sx, sy;
            if (!toScreen(d.labelAnchor, sx, sy))
                continue;
            // up and to the right of the anchor, so the label never sits on its line
            const engine::LatexImage& img = subspaceLabels[d.kind];
            img.drawAt(sx + 6.0f, sy - img.height() - 2.0f);
        }
    }

    // \phi^t(x_0) above every single (click-dropped) particle, with x_0 written out as
    // the actual seed: the flow map applied to it for the ODE time integrated so far —
    // t = 0 at the seed, negative when running in reverse. Composed per frame from
    // TimeLabelGlyphs plus the per-curve seed image.
    void drawTimeLabels()
    {
        if (!showTimeLabels)
            return;

        TimeLabelGlyphs& g = timeGlyphs;
        const float S = timeLabelSize;
        if (g.size != S)
        {
            // TeX proportions: superscripts at scriptstyle (0.7) raised ~0.42 em
            g.size = S;
            g.phi.set(R"(\phi)", S, kSingleColor);
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
                c.seedLabel.set("(" + texNumber(c.seed.x) + ", " + texNumber(c.seed.y) +
                                    ", " + texNumber(c.seed.z) + ")",
                                S, kSingleColor);
            }

            float sx, sy;
            if (!toScreen(c.traj->head(), sx, sy))
                continue;

            double t = c.traj->time();
            if (std::abs(t) < 0.005)
                t = 0.0; // no "-0.00"
            char num[32];
            std::snprintf(num, sizeof(num), "%.2f", t);

            // lay the pieces out along one baseline: \phi, raised digits, (x0 values)
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
            place(c.seedLabel, 0.0f);

            // centered above the particle (baseline clears the head and the parenthesis
            // descender) — or below it when its tangent arrow points up on screen, so the
            // label and the arrow never cross
            float headRadius = 0.5f * particleSize * singleScale;
            float tx, ty;
            bool arrowUp = c.hasTangent && toScreen(c.tangentTip, tx, ty) && ty < sy;
            float left = sx - 0.5f * x;
            float baseY = arrowUp ? sy + headRadius + 0.95f * S
                                  : sy - headRadius - 0.35f * S;
            for (int i = 0; i < n; ++i)
                pieces[i]->drawAt(left + xs[i], baseY + ys[i]);
        }
    }

    // x' = Ax with the live A, typeset top-right. Re-typeset only when a lambda moved.
    void drawSystemOverlay()
    {
        if (!showSystemLatex)
            return;
        if (!std::equal(lambda, lambda + 3, latexLambda))
        {
            std::copy(lambda, lambda + 3, latexLambda);
            auto num = texNumber;
            std::string tex =
                R"(\dot{x} = \begin{pmatrix})" +
                num(lambda[0]) + R"( & 0 & 0 \\ 0 & )" + num(lambda[1]) +
                R"( & 0 \\ 0 & 0 & )" + num(lambda[2]) +
                R"(\end{pmatrix} \begin{pmatrix} x_1 \\ x_2 \\ x_3 )";
            systemLatex.set(tex, systemLatexSize);
        }
        if (systemLatex.empty())
            return;

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x - 12.0f,
                                       vp->WorkPos.y + 12.0f),
                                ImGuiCond_Always, ImVec2(1.0f, 0.0f));
        ImGui::SetNextWindowBgAlpha(0.35f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        ImGui::Begin("##systemLatex", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
        systemLatex.draw();
        ImGui::End();
        ImGui::PopStyleVar();
    }

    void drawSystemUI()
    {
        sectionHeader("System");
        ImGui::TextUnformatted("x' = diag(l1, l2, l3) x");
        bool changed = false;
        const char* labels[3] = {"l1 (x)", "l2 (y)", "l3 (z)"};
        for (int i = 0; i < 3; ++i)
            changed |= ImGui::SliderFloat(labels[i], &lambda[i], -3.0f, 3.0f, "%.2f");
        ImGui::SetNextItemWidth(200.0f);
        if (ImGui::BeginCombo("preset", "choose..."))
        {
            for (const Preset& p : kPresets)
            {
                if (ImGui::Selectable(p.name))
                {
                    std::copy(p.lambda, p.lambda + 3, lambda);
                    changed = true;
                }
            }
            ImGui::EndCombo();
        }

        if (changed)
        {
            subspacesDirty = true;
            arrowsDirty = true;
        }

        int ds = 0, du = 0, dc = 0;
        for (float l : lambda)
            (isCenter(l) ? dc : (l < 0.0f ? ds : du))++;
        ImGui::Text("dim E^s = %d   dim E^u = %d   dim E^c = %d", ds, du, dc);
    }

    void drawOverlayUI()
    {
        sectionHeader("LaTeX overlays");
        ImGui::Checkbox("show x' = Ax", &showSystemLatex);
        ImGui::SameLine();
        ImGui::Checkbox("label singles with phi^t(x0)", &showTimeLabels);
        if (!systemLatex.error().empty())
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "LaTeX: %s",
                               systemLatex.error().c_str());
    }

    void drawSubspaceUI()
    {
        sectionHeader("Invariant subspaces");
        ImGui::Checkbox("show subspaces", &showSubspaces);
        ImGui::SameLine();
        ImGui::Checkbox("axes", &showAxes);
        ImGui::SameLine();
        ImGui::Checkbox("labels", &showSubspaceLabels);
        if (showSubspaceLabels)
            subspacesDirty |= ImGui::SliderFloat("label position", &subspaceLabelAt, 0.05f,
                                                 1.1f, "%.2f of extent");
        if (showSubspaces)
        {
            ImGui::TextColored(ImVec4(kStableColor.r, kStableColor.g, kStableColor.b, 1.0f),
                               "stable");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(kUnstableColor.r, kUnstableColor.g, kUnstableColor.b, 1.0f),
                               "unstable");
            ImGui::SameLine();
            ImGui::TextDisabled("center");
            subspacesDirty |= ImGui::SliderFloat("subspace extent", &subspaceExtent, 0.5f,
                                                 30.0f, "%.1f");
            ImGui::SliderFloat("plane alpha", &planeAlpha, 0.0f, 0.6f);
            ImGui::SliderFloat("line width", &subspaceLineWidth, 1.0f, 6.0f);
            ImGui::Checkbox("flow arrows on subspaces", &showSubspaceArrows);
            if (showSubspaceArrows)
            {
                subspacesDirty |= ImGui::SliderInt("arrows per half-axis",
                                                   &subspaceArrowCount, 1, 8);
                subspacesDirty |= ImGui::SliderFloat("subspace arrow size",
                                                     &subspaceArrowSize, 0.01f, 0.2f,
                                                     "%.3f of extent");
            }
        }
    }

    void drawGlyphUI()
    {
        sectionHeader("Arrow field");
        ImGui::Checkbox("show arrows", &showArrows);

        bool editedLive = false;
        bool editedDone = false;
        auto track = [&](bool changed) {
            editedLive = editedLive || changed;
            editedDone = editedDone || ImGui::IsItemDeactivatedAfterEdit();
        };

        track(ImGui::SliderInt("grid N", &gridN, 2, 50));
        track(ImGui::SliderFloat("arrow scale", &arrowScale, 0.1f, 1.5f));

        ImGui::TextUnformatted("head:");
        ImGui::SameLine();
        int style = barbedHeads ? 1 : 0;
        ImGui::RadioButton("solid", &style, 0);
        ImGui::SameLine();
        ImGui::RadioButton("barbs", &style, 1);
        if ((style == 1) != barbedHeads)
        {
            barbedHeads = (style == 1);
            arrowsDirty = true;
        }
        if (!barbedHeads)
        {
            track(ImGui::SliderFloat("head length", &headFraction, 0.1f, 0.7f, "%.2f of arrow"));
            track(ImGui::SliderFloat("head width", &headWidth, 0.04f, 0.5f,
                                     "%.2f of head length"));
        }

        arrowsDirty |= ImGui::Checkbox("length by magnitude", &lengthByMagnitude);
        track(ImGui::SliderFloat("hide below", &hideBelow, 0.0f, 0.5f, "%.3f of max"));
        track(ImGui::DragFloat("domain radius", &domainR, 0.1f, 0.5f, 50.0f));
        ImGui::SliderFloat("arrow alpha", &arrowAlpha, 0.05f, 1.0f);
        ImGui::SliderFloat("alpha gradient", &arrowTailAlpha, 0.0f, 1.0f, "tail %.2f of tip");

        if (editedDone || (editedLive && glyphs.arrows() <= kLiveRebuildArrows))
            arrowsDirty = true;

        ImGui::Checkbox("auto color scale", &autoColorScale);
        if (!autoColorScale)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            if (ImGui::DragFloat("max |f|", &manualColorMax, 0.05f, 1e-3f, 1e6f))
                arrowsDirty = true;
        }
        ImGui::TextDisabled("sampled max |f| = %.4g   arrows = %d", sampledMaxMag, glyphs.arrows());
    }

    void drawFlowUI()
    {
        sectionHeader("Flow");
        if (ImGui::Button(playing ? "Pause" : "Play"))
            playing = !playing;
        ImGui::SameLine();
        ImGui::TextDisabled("%s", engine::FlowKeys::kHint);
        ImGui::SameLine();
        ImGui::Checkbox("reverse", &reverse);
        ImGui::SameLine();
        if (ImGui::Button("Restart"))
            restartTrajectories();
        ImGui::SameLine();
        if (ImGui::Button("Clear all"))
            curves.clear();
        ImGui::SliderFloat("speed", &timeScale, 0.0f, 5.0f, "%.2f x");
        ImGui::TextDisabled("%d trajectories total", (int)curves.size());
    }

    void drawBallUI()
    {
        sectionHeader("Ball of trajectories");
        ImGui::SetNextItemWidth(200.0f);
        ImGui::InputFloat3("center", ballCenter);
        ImGui::SliderFloat("radius", &ballRadius, 0.01f, 5.0f, "%.2f");
        ImGui::SliderInt("particles", &ballCount, 1, 1000);
        ImGui::Checkbox("fill volume", &ballFilled);
        ImGui::SameLine();
        ImGui::Checkbox("single at center", &ballWithSingle);
        if (ImGui::Button("Drop ball"))
            dropBall(glm::vec3(ballCenter[0], ballCenter[1], ballCenter[2]));
        ImGui::SameLine();
        if (ImGui::Button("Remove ball"))
            removeBall();
    }

    void drawSinglesUI()
    {
        sectionHeader("Single trajectories");
        ImGui::TextDisabled("click in the view to drop one under the cursor");
        ImGui::SliderFloat("click depth", &spawnDepth, 0.05f, 1.0f,
                           "%.2f of the way to the orbit center");
        ImGui::SliderFloat("singles thickness", &singleScale, 1.0f, 4.0f, "%.1f x");
        ImGui::Checkbox("tangent vector x'", &showTangents);
        if (showTangents)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(140.0f);
            ImGui::SliderFloat("##tangentSize", &tangentSize, 0.02f, 0.5f, "length %.2f");
        }
        if (ImGui::Button("Clear singles"))
            removeKind(Kind::Single);

        int singles = countSingles();
        if (ImGui::Checkbox("camera follows a single", &followSingle) && !followSingle)
            releaseFollow();
        if (followSingle)
        {
            ImGui::SameLine();
            ImGui::Checkbox("newest", &followNewest);
            if (!followNewest && singles > 0)
            {
                int shown = std::clamp(followIndex, 0, singles - 1) + 1;
                if (ImGui::SliderInt("follow #", &shown, 1, singles))
                    followIndex = shown - 1;
            }
            ImGui::TextDisabled("%s", singles ? "A/D/W/S orbit and Q/E zoom around the particle"
                                              : "(drop a single to follow it)");
        }
    }

    void drawTrailUI()
    {
        sectionHeader("Trails & particles");
        ImGui::Checkbox("show trails", &showTrails);
        if (ImGui::SliderInt("trail points", &trailPoints, 32, 5000))
            for (Curve& c : curves)
                c.traj->setMaxPoints(trailPoints);
        ImGui::SliderFloat("trail width", &trailWidth, 1.0f, 6.0f);
        ImGui::SliderFloat("trail alpha", &trailAlpha, 0.05f, 1.0f);
        ImGui::SliderFloat("particle size", &particleSize, 2.0f, 20.0f);
    }

    // Seeds on a circle of radius r around the origin inside E^s. The system is
    // diagonal, so a seed with zero unstable/center components keeps them zero forever:
    // these curves stay in E^s and visibly shrink into the origin.
    void drawManifoldCircleUI()
    {
        sectionHeader("Circle in the stable subspace E^s");
        std::vector<int> stable = stableAxes();
        ImGui::SliderFloat("circle radius", &manifoldRadius, 0.1f, 20.0f, "%.2f");
        ImGui::SliderInt("circle particles", &manifoldCount, 2, 200);

        ImGui::BeginDisabled(stable.empty());
        if (ImGui::Button("Spawn circle in E^s"))
            spawnManifoldCircle();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Remove circle"))
            removeKind(Kind::Manifold);

        if (stable.empty())
            ImGui::TextDisabled("no stable directions (no lambda < 0)");
        else if (stable.size() == 1)
            ImGui::TextDisabled("E^s is a line: the 'circle' is its two points at +-r");
        else if (stable.size() == 3)
            ImGui::TextDisabled("E^s is all of R3: circle drawn in the first two axes");
    }

    // ---- subspaces ------------------------------------------------------------

    static bool isCenter(float l) { return std::abs(l) < 1e-4f; }

    // E^s, E^u, E^c are spanned by the coordinate axes whose eigenvalue is negative,
    // positive, zero. Each becomes axis lines, plus a translucent square when it is a
    // plane, or the edges of a cube when it is all of R3.
    void rebuildSubspaces()
    {
        subspacesDirty = false;
        subspaceDraws.clear();
        std::vector<float> buf;
        for (engine::ArrowGlyphs& cones : subspaceCones)
        {
            cones.clear();
            cones.headFraction = 1.0f;   // all head, no shaft
            cones.headWidth = 0.45f;
        }
        float E = subspaceExtent;

        auto axis = [](int i) {
            glm::vec3 e(0.0f);
            e[i] = 1.0f;
            return e;
        };
        auto push = [&](const glm::vec3& p) { buf.insert(buf.end(), {p.x, p.y, p.z}); };
        auto vertices = [&]() { return (int)buf.size() / 3; };

        for (int kind = 0; kind < 3; ++kind)
        {
            std::vector<int> span;
            for (int i = 0; i < 3; ++i)
            {
                int k = isCenter(lambda[i]) ? 2 : (lambda[i] < 0.0f ? 0 : 1);
                if (k == kind)
                    span.push_back(i);
            }
            if (span.empty())
                continue;

            SubspaceDraw d;
            d.color = kind == 0 ? kStableColor : (kind == 1 ? kUnstableColor : kCenterColor);
            d.kind = kind;
            // label partway out along the + directions (subspaceLabelAt of the extent),
            // near the origin where the view usually is: on a line, its + half; on a
            // plane, its (+,+) diagonal; on the cube, the (+,+,+) diagonal
            for (int i : span)
                d.labelAnchor += axis(i) * (E * subspaceLabelAt);

            // cones along each half-axis pointing the way the flow goes: inward on E^s,
            // outward on E^u. E^c is at rest, so it gets none.
            if (kind != 2)
            {
                float coneLen = E * subspaceArrowSize;
                for (int i : span)
                    for (float side : {1.0f, -1.0f})
                        for (int k = 0; k < subspaceArrowCount; ++k)
                        {
                            glm::vec3 ray = axis(i) * side;
                            glm::vec3 dir = kind == 0 ? -ray : ray;
                            float at = E * ((float)k + 0.5f) / (float)subspaceArrowCount;
                            glm::vec3 start = ray * at - dir * (coneLen * 0.5f);
                            subspaceCones[kind].add(start, dir, coneLen, 0.0f);
                        }
            }

            d.lineFirst = vertices();
            for (int i : span)
            {
                push(axis(i) * E);
                push(axis(i) * -E);
            }
            if (span.size() == 2)
            {
                glm::vec3 u = axis(span[0]) * E, v = axis(span[1]) * E;
                glm::vec3 c[4] = {-u - v, u - v, u + v, -u + v};
                for (int k = 0; k < 4; ++k)
                {
                    push(c[k]);
                    push(c[(k + 1) % 4]);
                }
                d.lineCount = vertices() - d.lineFirst;

                d.triFirst = vertices();
                for (int k : {0, 1, 2, 0, 2, 3})
                    push(c[k]);
                d.triCount = 6;
            }
            else if (span.size() == 3)
            {
                for (int i = 0; i < 3; ++i)
                {
                    glm::vec3 a = axis((i + 1) % 3) * E, b = axis((i + 2) % 3) * E;
                    for (glm::vec3 off : {a + b, a - b, -a + b, -a - b})
                    {
                        push(off + axis(i) * E);
                        push(off - axis(i) * E);
                    }
                }
                d.lineCount = vertices() - d.lineFirst;
            }
            else
            {
                d.lineCount = vertices() - d.lineFirst;
            }
            subspaceDraws.push_back(d);
        }

        subspaceMesh->setVertices(buf.data(), buf.size(), 0);

        for (engine::ArrowGlyphs& cones : subspaceCones)
            cones.upload();
    }

    // ---- follow camera + tangent vectors ----------------------------------------

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
        if (!followSingle || n == 0)
            return nullptr;
        int want = followNewest ? n - 1 : std::clamp(followIndex, 0, n - 1);
        for (const Curve& c : curves)
            if (c.kind == Kind::Single && want-- == 0)
                return &c;
        return nullptr;
    }

    // Orbit around the followed head: keep the orbit offset and move position with the
    // centre in the same frame, so the view neither lags nor stops following while
    // ImGui has the keyboard (when the camera's processInput is skipped).
    void followCamera()
    {
        const Curve* c = followedSingle();
        if (!c)
        {
            following = false;
            return;
        }
        engine::Camera& cam = app->camera();
        glm::vec3 offset = cam.position - cam.orbitCenter;
        glm::vec3 head = c->traj->head();
        cam.orbitCenter = head;
        cam.position = head + offset;
        cam.front = glm::normalize(-offset);
        following = true;
    }

    // Back to orbiting the saved centre (origin if none), same angles and distance.
    void releaseFollow()
    {
        if (!following)
            return;
        following = false;
        engine::Camera& cam = app->camera();
        glm::vec3 offset = cam.position - cam.orbitCenter;
        cam.orbitCenter = std::isnan(camRadius)
                              ? glm::vec3(0.0f)
                              : glm::vec3(camCenter[0], camCenter[1], camCenter[2]);
        cam.position = cam.orbitCenter + offset;
    }

    // A red x' = Ax arrow out of every single's head, at constant on-screen size (a
    // fraction of the camera distance). Rebuilt every frame — a handful of arrows.
    void rebuildTangents()
    {
        tangents.clear();
        for (Curve& c : curves)
            c.hasTangent = false;
        if (!showTangents)
        {
            tangents.upload();
            return;
        }

        const glm::vec3 eye = app->camera().position;
        for (Curve& c : curves)
        {
            if (c.kind != Kind::Single || c.traj->diverged())
                continue;
            glm::vec3 p = c.traj->head();
            glm::vec3 vec(lambda[0] * p.x, lambda[1] * p.y, lambda[2] * p.z);
            float mag = glm::length(vec);
            if (!(mag > 1e-9f) || !std::isfinite(mag))
                continue; // at the equilibrium there is no direction to draw
            glm::vec3 dir = vec / mag;
            float len = tangentSize * glm::length(eye - p);
            tangents.add(p, dir, len, mag);
            c.hasTangent = true;
            c.tangentTip = p + dir * len;
        }
        tangents.upload();
    }

    // \dot{x} just beyond each tangent arrow's tip, continuing the arrow's on-screen
    // direction so it never sits on the arrowhead; phi-label size, typeset once.
    void drawTangentLabels()
    {
        if (!showTangents)
            return;
        if (tangentLabelSize != timeLabelSize)
        {
            tangentLabelSize = timeLabelSize;
            tangentLabel.set(R"(\dot{x})", timeLabelSize, kTangentColor);
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
            float w = (float)tangentLabel.width(), h = (float)tangentLabel.height();
            float reach = 0.5f * (std::abs(d.x) * w + std::abs(d.y) * h) + 2.0f;
            tangentLabel.drawAt(tx + d.x * reach - 0.5f * w, ty + d.y * reach - 0.5f * h);
        }
    }

    // ---- trajectories -----------------------------------------------------------

    void handleSeedClick()
    {
        if (!ImGui::IsMouseClicked(ImGuiMouseButton_Left) || ImGui::GetIO().WantCaptureMouse)
            return;

        // unproject the cursor ray and stop it on a camera-facing plane spawnDepth of
        // the way from the camera to the orbit center: the seed lands right under the
        // cursor, close in front of the camera, instead of somewhere deep in the scene
        glm::vec2 ndc = app->window().cursorPosNormalized() * 2.0f - 1.0f;
        const engine::Camera& cam = app->camera();
        glm::mat4 invVP = glm::inverse(cam.getProjectionMatrix() * cam.getViewMatrix());
        glm::vec4 nearH = invVP * glm::vec4(ndc, -1.0f, 1.0f);
        glm::vec4 farH = invVP * glm::vec4(ndc, 1.0f, 1.0f);
        if (nearH.w == 0.0f || farH.w == 0.0f)
            return;

        glm::vec3 origin = glm::vec3(nearH) / nearH.w;
        glm::vec3 dir = glm::normalize(glm::vec3(farH) / farH.w - origin);
        float denom = glm::dot(dir, cam.front);
        if (std::abs(denom) < 1e-4f)
            return;
        float toCenter = glm::dot(cam.orbitCenter - cam.position, cam.front);
        glm::vec3 planePoint = cam.position + cam.front * (spawnDepth * toCenter);
        float s = glm::dot(planePoint - origin, cam.front) / denom;
        if (s <= 0.0f)
            return;

        addTrajectory(origin + dir * s, kSingleColor, Kind::Single);
    }

    void removeKind(Kind kind)
    {
        std::erase_if(curves, [kind](const Curve& c) { return c.kind == kind; });
    }

    std::vector<int> stableAxes() const
    {
        std::vector<int> axes;
        for (int i = 0; i < 3; ++i)
            if (!isCenter(lambda[i]) && lambda[i] < 0.0f)
                axes.push_back(i);
        return axes;
    }

    // Replaces the previous circle. In a 2D (or 3D) E^s the seeds are evenly spaced on
    // the circle spanned by the first two stable axes; in a 1D E^s the circle is S^0,
    // the two points at +-r.
    void spawnManifoldCircle()
    {
        removeKind(Kind::Manifold);
        std::vector<int> axes = stableAxes();
        if (axes.empty())
            return;

        if (axes.size() == 1)
        {
            for (float side : {1.0f, -1.0f})
            {
                glm::vec3 p(0.0f);
                p[axes[0]] = side * manifoldRadius;
                addTrajectory(p, kManifoldColor, Kind::Manifold);
            }
            return;
        }

        for (int k = 0; k < manifoldCount; ++k)
        {
            float th = 6.28318530718f * (float)k / (float)manifoldCount;
            glm::vec3 p(0.0f);
            p[axes[0]] = manifoldRadius * std::cos(th);
            p[axes[1]] = manifoldRadius * std::sin(th);
            addTrajectory(p, kManifoldColor, Kind::Manifold);
        }
    }

    // Only one ball lives at a time: dropping a new one replaces the old. Seeds sit on
    // the sphere surface (Fibonacci lattice, evenly spread — the clearest view of the
    // ball deforming into an ellipsoid) or uniformly through its volume.
    void dropBall(const glm::vec3& center)
    {
        removeBall();
        // "single at center": a green single riding the middle of the ball, labelled and
        // followable like any click-dropped one, removed with the ball
        if (ballWithSingle)
        {
            addTrajectory(center, kSingleColor, Kind::Single);
            if (!curves.empty())
                curves.back().withBall = true;
        }
        std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
        const float golden = 2.39996323f;

        for (int k = 0; k < ballCount && curves.size() < kMaxTrajectories; ++k)
        {
            glm::vec3 p;
            if (ballFilled)
            {
                do
                    p = glm::vec3(unit(rng), unit(rng), unit(rng));
                while (glm::dot(p, p) > 1.0f);
            }
            else
            {
                float y = 1.0f - 2.0f * ((float)k + 0.5f) / (float)ballCount;
                float r = std::sqrt(std::max(0.0f, 1.0f - y * y));
                float phi = golden * (float)k;
                p = glm::vec3(std::cos(phi) * r, y, std::sin(phi) * r);
            }
            addTrajectory(center + p * ballRadius, kBallColor, Kind::Ball);
        }
    }

    void removeBall()
    {
        std::erase_if(curves, [](const Curve& c) { return c.kind == Kind::Ball || c.withBall; });
    }

    void addTrajectory(const glm::vec3& seed, const glm::vec3& color, Kind kind)
    {
        if (curves.size() >= kMaxTrajectories)
            return;
        double s[3] = {seed.x, seed.y, seed.z};
        auto traj = std::make_unique<engine::Trajectory>(3, s, 0.0, trailPoints);
        traj->minSpacing = 0.002f * domainR;
        curves.push_back({std::move(traj), seed, color, kind});
    }

    void restartTrajectories()
    {
        for (Curve& c : curves)
        {
            double s[3] = {c.seed.x, c.seed.y, c.seed.z};
            c.traj->reset(s, 0.0);
        }
    }

    // ---- settings file ------------------------------------------------------------

    // Every user-tunable value, by key. Plain "key value" lines, so the file is easy to
    // read or hand-edit; unknown or missing keys are ignored on load.
    struct Setting {
        const char* key;
        float* f = nullptr;
        int* i = nullptr;
        bool* b = nullptr;
    };

    std::vector<Setting> settingsTable()
    {
        return {
            {"lambda1", &lambda[0]}, {"lambda2", &lambda[1]}, {"lambda3", &lambda[2]},
            {"showSubspaces", nullptr, nullptr, &showSubspaces},
            {"showAxes", nullptr, nullptr, &showAxes},
            {"showSubspaceLabels", nullptr, nullptr, &showSubspaceLabels},
            {"subspaceLabelAt", &subspaceLabelAt},
            {"showSystemLatex", nullptr, nullptr, &showSystemLatex},
            {"showTimeLabels", nullptr, nullptr, &showTimeLabels},
            {"systemLatexSize", &systemLatexSize}, {"timeLabelSize", &timeLabelSize},
            {"subspaceLabelSize", &subspaceLabelSize},
            {"subspaceExtent", &subspaceExtent}, {"planeAlpha", &planeAlpha},
            {"subspaceLineWidth", &subspaceLineWidth},
            {"showSubspaceArrows", nullptr, nullptr, &showSubspaceArrows},
            {"subspaceArrowCount", nullptr, &subspaceArrowCount},
            {"subspaceArrowSize", &subspaceArrowSize},
            {"showArrows", nullptr, nullptr, &showArrows},
            {"gridN", nullptr, &gridN},
            {"arrowScale", &arrowScale},
            {"barbedHeads", nullptr, nullptr, &barbedHeads},
            {"headFraction", &headFraction}, {"headWidth", &headWidth},
            {"lengthByMagnitude", nullptr, nullptr, &lengthByMagnitude},
            {"hideBelow", &hideBelow}, {"domainR", &domainR},
            {"arrowAlpha", &arrowAlpha}, {"arrowTailAlpha", &arrowTailAlpha},
            {"autoColorScale", nullptr, nullptr, &autoColorScale},
            {"manualColorMax", &manualColorMax},
            {"timeScale", &timeScale},
            {"reverse", nullptr, nullptr, &reverse},
            {"method", nullptr, &method}, {"maxStep", &maxStep},
            {"ballCenterX", &ballCenter[0]}, {"ballCenterY", &ballCenter[1]},
            {"ballCenterZ", &ballCenter[2]},
            {"ballRadius", &ballRadius}, {"ballCount", nullptr, &ballCount},
            {"ballFilled", nullptr, nullptr, &ballFilled},
            {"ballWithSingle", nullptr, nullptr, &ballWithSingle},
            {"showTrails", nullptr, nullptr, &showTrails},
            {"trailPoints", nullptr, &trailPoints},
            {"trailWidth", &trailWidth}, {"trailAlpha", &trailAlpha},
            {"particleSize", &particleSize},
            {"singleScale", &singleScale},
            {"spawnDepth", &spawnDepth},
            {"showTangents", nullptr, nullptr, &showTangents},
            {"tangentSize", &tangentSize},
            {"followSingle", nullptr, nullptr, &followSingle},
            {"followNewest", nullptr, nullptr, &followNewest},
            {"followIndex", nullptr, &followIndex},
            {"cameraTheta", &camTheta}, {"cameraPhi", &camPhi},
            {"cameraRadius", &camRadius},
            {"cameraCenterX", &camCenter[0]}, {"cameraCenterY", &camCenter[1]},
            {"cameraCenterZ", &camCenter[2]},
            {"manifoldRadius", &manifoldRadius},
            {"manifoldCount", nullptr, &manifoldCount},
        };
    }

    // The orbit view as saved with the settings; NaN = nothing saved, keep the default
    // framing. Copied from the live camera on save, applied to it in init().
    void captureCamera()
    {
        const engine::Camera& cam = app->camera();
        camTheta = cam.orbitTheta;
        camPhi = cam.orbitPhi;
        camRadius = cam.orbitRadius;
        camCenter[0] = cam.orbitCenter.x;
        camCenter[1] = cam.orbitCenter.y;
        camCenter[2] = cam.orbitCenter.z;
    }

    void applySavedCamera(engine::Camera& cam) const
    {
        if (std::isnan(camRadius))
            return;
        cam.orbitTheta = camTheta;
        cam.orbitPhi = camPhi;
        cam.orbitRadius = std::max(camRadius, cam.orbitRadiusMin);
        cam.orbitCenter = glm::vec3(camCenter[0], camCenter[1], camCenter[2]);
    }

    bool saveSettings()
    {
        captureCamera();
        std::ofstream out(kSettingsPath);
        if (!out)
            return false;
        for (const Setting& s : settingsTable())
        {
            out << s.key << ' ';
            if (s.f) out << *s.f;
            else if (s.i) out << *s.i;
            else out << (*s.b ? 1 : 0);
            out << '\n';
        }
        return (bool)out;
    }

    void loadSettings()
    {
        std::ifstream in(kSettingsPath);
        if (!in)
            return;
        std::vector<Setting> table = settingsTable();
        std::string key;
        double value;
        while (in >> key >> value)
        {
            for (const Setting& s : table)
            {
                if (key != s.key)
                    continue;
                if (s.f) *s.f = (float)value;
                else if (s.i) *s.i = (int)value;
                else *s.b = value != 0.0;
            }
        }
        // a hand-edited file should not be able to ask for a 1000^3 grid
        gridN = std::clamp(gridN, 2, 50);
        ballCount = std::clamp(ballCount, 1, 1000);
        method = std::clamp(method, 0, 2);
        maxStep = std::clamp(maxStep, 1e-5f, 0.5f);
        systemLatexSize = std::clamp(systemLatexSize, 10.0f, 120.0f);
        timeLabelSize = std::clamp(timeLabelSize, 10.0f, 120.0f);
        subspaceLabelSize = std::clamp(subspaceLabelSize, 10.0f, 160.0f);
        subspaceArrowCount = std::clamp(subspaceArrowCount, 1, 8);
        subspaceArrowSize = std::clamp(subspaceArrowSize, 0.01f, 0.2f);
        trailPoints = std::clamp(trailPoints, 32, 5000);
        domainR = std::clamp(domainR, 0.5f, 50.0f);
    }

    // ---- arrow grid -------------------------------------------------------------

    float colorMax() const
    {
        return autoColorScale ? std::max(sampledMaxMag, 1e-6f) : manualColorMax;
    }

    void rebuildArrows()
    {
        arrowsDirty = false;
        sampledMaxMag = 0.0f;

        float spacing = 2.0f * domainR / (float)gridN;
        gridPoints.clear();
        gridVectors.clear();
        for (int k = 0; k <= gridN; ++k)
            for (int j = 0; j <= gridN; ++j)
                for (int i = 0; i <= gridN; ++i)
                {
                    glm::vec3 p(-domainR + i * spacing, -domainR + j * spacing,
                                -domainR + k * spacing);
                    glm::vec3 v(lambda[0] * p.x, lambda[1] * p.y, lambda[2] * p.z);
                    gridPoints.push_back(p);
                    gridVectors.push_back(v);
                    sampledMaxMag = std::max(sampledMaxMag, glm::length(v));
                }

        glyphs.clear();
        glyphs.barbs = barbedHeads;
        glyphs.headFraction = headFraction;
        glyphs.headWidth = headWidth;
        glyphs.addField(gridPoints, gridVectors, spacing * arrowScale, colorMax(), hideBelow,
                        lengthByMagnitude);
        glyphs.upload();
    }

    static constexpr int kSubspaceCapacity = 256;
    static constexpr int kLiveRebuildArrows = 20000;
    static constexpr size_t kMaxTrajectories = 4000;

    engine::App* app = nullptr;
    engine::Shader shader;
    std::unique_ptr<engine::Mesh> axisMesh;
    std::unique_ptr<engine::Mesh> subspaceMesh;
    engine::ArrowGlyphs subspaceCones[3]; // flow-direction cones per subspace kind (heads only)
    engine::ArrowGlyphs glyphs;          // the field's arrow grid

    float lambda[3] = {1.0f, -1.0f, -2.0f};
    engine::LatexImage systemLatex;           // x' = Ax overlay, top-right
    float latexLambda[3] = {NAN, NAN, NAN};   // lambdas systemLatex was typeset for
    bool showSystemLatex = true;
    bool showTimeLabels = true;               // \phi^t(x_0) above single particles
    float timeLabelSize = 36.0f;
    float systemLatexSize = 34.0f;            // the x' = Ax overlay, px
    TimeLabelGlyphs timeGlyphs;
    engine::Integrator integrator;
    int method = 2;                   // Integrator::Method, RK4
    float maxStep = 0.01f;
    engine::Integrator::Field rhs;
    std::vector<Curve> curves;
    std::string settingsStatus;

    // subspaces
    bool showSubspaces = true;
    bool showAxes = true;
    bool subspacesDirty = true;
    float subspaceExtent = 3.0f;
    float planeAlpha = 0.15f;
    float subspaceLineWidth = 3.0f;
    bool showSubspaceArrows = true;
    int subspaceArrowCount = 3;       // cones per half-axis
    float subspaceArrowSize = 0.06f;  // cone length, fraction of subspaceExtent
    std::vector<SubspaceDraw> subspaceDraws;
    bool showSubspaceLabels = true;
    float subspaceLabelSize = 68.0f;
    float subspaceLabelAt = 0.35f;            // label anchor, fraction of subspaceExtent
    float subspaceLabelsSize = 0.0f;          // size subspaceLabels were typeset at
    engine::LatexImage subspaceLabels[3];     // E^s, E^u, E^c

    // arrows
    bool showArrows = true;
    bool arrowsDirty = true;
    bool lengthByMagnitude = false;
    bool barbedHeads = false;
    int gridN = 8;
    float domainR = 3.0f;
    float arrowScale = 0.7f;
    float arrowAlpha = 0.6f;
    float arrowTailAlpha = 0.25f;
    float headFraction = 0.36f;
    float headWidth = 0.14f;
    float hideBelow = 0.0f;
    bool autoColorScale = true;
    float manualColorMax = 5.0f;
    float sampledMaxMag = 0.0f;
    std::vector<glm::vec3> gridPoints;
    std::vector<glm::vec3> gridVectors;

    // flow + balls
    bool playing = true;
    bool reverse = false;
    float timeScale = 1.0f;
    bool showTrails = true;
    int trailPoints = 600;
    float trailWidth = 1.5f;
    float trailAlpha = 0.6f;
    float particleSize = 6.0f;
    float singleScale = 2.0f;        // singles' trail width and head size multiplier
    float spawnDepth = 0.3f;         // click seeds: fraction of camera -> orbit center

    // tangent arrows on singles (rebuilt every frame) + the follow camera
    bool showTangents = true;
    float tangentSize = 0.12f;       // fraction of the camera distance
    engine::ArrowGlyphs tangents;    // x' arrows
    engine::LatexImage tangentLabel; // \dot{x}
    float tangentLabelSize = 0.0f;
    bool followSingle = false;
    bool followNewest = true;
    int followIndex = 0;             // drop order, 0-based, when not following the newest
    bool following = false;          // the camera is currently tracking a single

    // saved orbit view (see captureCamera / applySavedCamera); NaN until saved
    float camTheta = NAN, camPhi = NAN, camRadius = NAN;
    float camCenter[3] = {0.0f, 0.0f, 0.0f};
    float manifoldRadius = 5.0f;     // E^s circle
    int manifoldCount = 48;
    float ballCenter[3] = {1.0f, 1.0f, 1.0f};
    float ballRadius = 0.4f;
    int ballCount = 200;
    bool ballFilled = false;
    bool ballWithSingle = false;     // drop a followable single at the ball's centre too
    std::mt19937 rng{1234};
    engine::TrajectoryRunner runner; // advances the curves, times integrate/upload/draw
};

} // namespace

std::unique_ptr<engine::Sim> makeDiagonalLinearSim()
{
    return std::make_unique<DiagonalLinearSim>();
}
