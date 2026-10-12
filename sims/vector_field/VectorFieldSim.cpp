#include "VectorFieldSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/Integrator.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>
#include <engine/Trajectory.h>
#include <engine/VectorField.h>

#include <gtc/matrix_transform.hpp>
#include <imgui.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace {

// One field the user can pick from the presets combo, per dimension.
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
    {"Thomas",             {"sin(y) - 0.19x", "sin(z) - 0.19y", "sin(x) - 0.19z"}},
    {"rigid rotation",     {"-y", "x", "0.3"}},
    {"3D saddle",          {"x", "y", "-2z"}},
};

// Arrow glyphs. Two head styles: a solid head — a cone in R3, a filled triangle in
// R2 — which is what makes the direction obvious from up close, or cheap line barbs.
//
// Shaft vertices are pos(3) + magnitude(1) + fade(1), where fade runs 0 at the tail to
// 1 at the head so the shader can taper the shaft opacity. Head vertices add a fourth
// float: the facet lambert term, baked here on the CPU (a cone with no shading is an
// unreadable silhouette).
constexpr int kFloatsPerShaftVertex = 5;
constexpr int kFloatsPerHeadVertex = 6;
constexpr int kMaxConeSegments = 7;

// How much geometry one arrowhead is worth. A 51^3 lattice is ~133k arrows, and a
// full 7-segment capped cone each would be ~134 MB of vertices — so the cone gets
// coarser as the grid gets denser. It costs nothing visually: at that density a head
// is a couple of pixels wide.
struct HeadLod {
    int segments;
    bool cap;
};

HeadLod headLod(int arrows)
{
    if (arrows <= 20000) return {7, true};
    if (arrows <= 50000) return {5, true};
    if (arrows <= 90000) return {4, false};
    return {3, false};
}

// world-space light for the baked head shading, and how flat the darkest facet gets
const glm::vec3 kLightDir = glm::normalize(glm::vec3(0.35f, 0.75f, 0.55f));
const float kAmbient = 0.55f;

int shaftVerticesPerArrow(int dim, bool barbs) { return barbs ? (dim == 2 ? 6 : 10) : 2; }
int headVerticesPerArrow(int dim, bool barbs, HeadLod lod)
{
    if (barbs) return 0;
    return dim == 2 ? 3 : lod.segments * (lod.cap ? 6 : 3);
}

void pushVertex(std::vector<float>& out, const glm::vec3& p, float mag, float fade)
{
    out.push_back(p.x);
    out.push_back(p.y);
    out.push_back(p.z);
    out.push_back(mag);
    out.push_back(fade);
}

void pushHeadVertex(std::vector<float>& out, const glm::vec3& p, float mag, float shade)
{
    out.push_back(p.x);
    out.push_back(p.y);
    out.push_back(p.z);
    out.push_back(mag);
    out.push_back(1.0f);   // heads are always the solid end of the gradient
    out.push_back(shade);
}

float facetShade(const glm::vec3& normal)
{
    float lambert = std::max(0.0f, glm::dot(normal, kLightDir));
    return kAmbient + (1.0f - kAmbient) * lambert;
}

// An orthonormal pair spanning the plane perpendicular to dir.
void perpBasis(const glm::vec3& dir, glm::vec3& a, glm::vec3& b)
{
    glm::vec3 helper = std::abs(dir.z) < 0.9f ? glm::vec3(0.0f, 0.0f, 1.0f)
                                              : glm::vec3(1.0f, 0.0f, 0.0f);
    a = glm::normalize(glm::cross(dir, helper));
    b = glm::cross(dir, a);
}

// The tail sits on the sample point and the head points downstream, so a glyph reads
// as "the flow leaves here going that way" (the usual quiver convention).
void pushArrow(std::vector<float>& shafts, std::vector<float>& heads, int dim,
               const glm::vec3& p, const glm::vec3& dir, float len, float mag,
               bool barbs, float headFraction, float headWidthRatio, HeadLod lod)
{
    glm::vec3 tip = p + dir * len;

    if (barbs)
    {
        pushVertex(shafts, p, mag, 0.0f);
        pushVertex(shafts, tip, mag, 1.0f);

        float barb = len * 0.32f;
        float wide = len * 0.16f;
        glm::vec3 a, b(0.0f);
        if (dim == 2)
            a = glm::vec3(-dir.y, dir.x, 0.0f);
        else
            perpBasis(dir, a, b);

        // the barbs keep the tip opacity: fading them would eat the arrowhead, which
        // is the part that carries the direction
        glm::vec3 back = tip - dir * barb;
        pushVertex(shafts, tip, mag, 1.0f);
        pushVertex(shafts, back + a * wide, mag, 0.9f);
        pushVertex(shafts, tip, mag, 1.0f);
        pushVertex(shafts, back - a * wide, mag, 0.9f);
        if (dim == 3)
        {
            pushVertex(shafts, tip, mag, 1.0f);
            pushVertex(shafts, back + b * wide, mag, 0.9f);
            pushVertex(shafts, tip, mag, 1.0f);
            pushVertex(shafts, back - b * wide, mag, 0.9f);
        }
        return;
    }

    // solid head: the shaft stops at the head's base so it never pokes through
    float headLen = len * headFraction;
    float headRad = headLen * headWidthRatio;
    glm::vec3 base = tip - dir * headLen;

    pushVertex(shafts, p, mag, 0.0f);
    pushVertex(shafts, base, mag, 1.0f);

    if (dim == 2)
    {
        // a flat triangle is the honest 2D arrowhead; no lighting in the plane
        glm::vec3 side(-dir.y, dir.x, 0.0f);
        pushHeadVertex(heads, tip, mag, 1.0f);
        pushHeadVertex(heads, base + side * headRad, mag, 0.88f);
        pushHeadVertex(heads, base - side * headRad, mag, 0.88f);
        return;
    }

    glm::vec3 a, b;
    perpBasis(dir, a, b);

    glm::vec3 rim[kMaxConeSegments];
    for (int i = 0; i < lod.segments; ++i)
    {
        float th = 6.28318530718f * (float)i / (float)lod.segments;
        rim[i] = base + (a * std::cos(th) + b * std::sin(th)) * headRad;
    }

    float slant = headRad / std::sqrt(headRad * headRad + headLen * headLen);
    float axial = headLen / std::sqrt(headRad * headRad + headLen * headLen);
    float capShade = facetShade(-dir);

    for (int i = 0; i < lod.segments; ++i)
    {
        int nxt = (i + 1) % lod.segments;
        // one normal per side facet, taken at its middle: flat shading, which suits a
        // low-segment cone better than smooth normals would
        glm::vec3 mid = glm::normalize((rim[i] + rim[nxt]) * 0.5f - base);
        glm::vec3 normal = glm::normalize(mid * axial + dir * slant);
        float shade = facetShade(normal);

        pushHeadVertex(heads, tip, mag, shade);
        pushHeadVertex(heads, rim[i], mag, shade);
        pushHeadVertex(heads, rim[nxt], mag, shade);

        // base cap, so the head stays solid when seen from behind; the first thing
        // dropped on a dense grid, where nobody is looking at one head from behind
        if (lod.cap)
        {
            pushHeadVertex(heads, base, mag, capShade);
            pushHeadVertex(heads, rim[nxt], mag, capShade);
            pushHeadVertex(heads, rim[i], mag, capShade);
        }
    }
}

// Live arrow glyphs for a user-typed vector field on R2 or R3, with trajectories
// integrated in real time on top of them. See obsidian_twin/sims/vector-field.md.
class VectorFieldSim : public engine::Sim {
public:
    const char* name() const override { return "vector_field"; }

    ~VectorFieldSim() override
    {
        // R2 mode and the chase camera both borrow camera state; hand it back
        if (app)
        {
            app->camera().inputEnabled = true;
            app->camera().up = glm::vec3(0.0f, 1.0f, 0.0f);
        }
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

        applyMode();
        applyField();
    }

    void update(float dt) override
    {
        handleViewKeys(dt);
        handleSeedClick();

        if (playing)
        {
            double h = (double)dt * timeScale * (reverse ? -1.0 : 1.0);
            fieldTime += h;
            for (auto& traj : trajectories)
                traj->advance(integrator, rhs, h);
            if (field.usesTime())
                arrowsDirty = true;
        }

        if (dim == 3)
        {
            app->camera().orbitAngularSpeed = orbitSpeed;
            app->camera().speed = zoomSpeed;
        }
        followCamera(dt);

        // in R2 the glyph grid covers the visible window, so panning, zooming or a
        // window resize re-samples the field
        if (dim == 2)
        {
            glm::vec3 view(viewCenter, viewScale);
            float aspect = app->window().aspect();
            if (view != lastView2D || aspect != lastAspect)
            {
                lastView2D = view;
                lastAspect = aspect;
                arrowsDirty = true;
            }
        }

        if (arrowsDirty)
            rebuildArrows();
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view, projection;
        viewProjection(camera, view, projection);

        // everything here is alpha-blended: the arrow gradient, the trail's fading
        // tail and the soft rim of the round heads all need it
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        shader.use();
        shader.setMat4("view", view);
        shader.setMat4("projection", projection);
        shader.setFloat("u_magScale", 1.0f / std::max(colorMax(), 1e-6f));
        shader.setFloat("u_pointSize", headSize);
        shader.setFloat("u_alpha", 1.0f);
        shader.setFloat("u_alphaTail", 1.0f);
        shader.setInt("u_alphaMode", 0);
        shader.setInt("u_round", 0);
        shader.setInt("u_shadePoint", 0);
        shader.setInt("u_shadeMode", 0);
        shader.setInt("u_focusCount", 0);

        if (showAxes)
        {
            shader.setInt("u_useRamp", 0);
            shader.setVec3("u_color", glm::vec3(0.35f, 0.35f, 0.42f));
            glLineWidth(1.0f);
            axisMesh->draw(GL_LINES);
        }

        if (showArrows && arrowVertexCount > 0)
        {
            // Glyphs are decoration: they blend, but they must not write depth, or a
            // faded arrow in front of a curve would punch a hole in it.
            glDepthMask(GL_FALSE);
            setFocusUniforms();

            shader.setInt("u_useRamp", 1);
            shader.setInt("u_alphaMode", 1);          // fade along each glyph
            shader.setFloat("u_alpha", arrowAlpha);
            shader.setFloat("u_alphaTail", arrowTailAlpha);
            glLineWidth(arrowWidth);
            arrowMesh->drawArrays(GL_LINES, arrowVertexCount);

            if (headVertexCount > 0 && headMesh)
            {
                // solid arrowheads: real triangles, shaded per facet, so the direction
                // stays obvious when the camera is right on top of them
                shader.setInt("u_alphaMode", 0);      // the head is the solid end
                shader.setInt("u_shadeMode", 1);
                headMesh->drawArrays(GL_TRIANGLES, headVertexCount);
                shader.setInt("u_shadeMode", 0);
            }

            shader.setFloat("u_alpha", 1.0f);
            shader.setInt("u_focusCount", 0);
            glDepthMask(GL_TRUE);
        }

        glLineWidth(trailWidth);
        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            const engine::Trajectory& traj = *trajectories[i];

            // fade by vertex age, so the oldest end of the trail thins out instead of
            // ending in a hard stub
            shader.setInt("u_alphaMode", trailFade ? 2 : 0);
            shader.setFloat("u_alphaTail", trailTailAlpha);
            shader.setFloat("u_ageFirst", (float)traj.drawFirst());
            shader.setFloat("u_ageSpan", (float)std::max(traj.drawCount() - 1, 1));
            shader.setInt("u_useRamp", trailBySpeed ? 1 : 0);
            shader.setVec3("u_color", trailColor(i));
            traj.draw();

            shader.setInt("u_alphaMode", 0);
            shader.setFloat("u_alphaTail", 1.0f);
            shader.setInt("u_round", 1);              // discs, not squares
            shader.setInt("u_shadePoint", 1);
            shader.setInt("u_useRamp", 0);
            shader.setVec3("u_color", traj.diverged() ? glm::vec3(0.9f, 0.25f, 0.25f)
                                                      : glm::vec3(1.0f));
            traj.drawHead();
            shader.setInt("u_round", 0);
            shader.setInt("u_shadePoint", 0);
        }
        glLineWidth(1.0f);

        glDisable(GL_BLEND);
    }

    void drawUI() override
    {
        drawFieldUI();
        ImGui::Separator();
        drawGlyphUI();
        ImGui::Separator();
        drawFlowUI();
        ImGui::Separator();
        drawTrajectoryUI();
    }

private:
    // ---- UI sections -------------------------------------------------------

    void drawFieldUI()
    {
        ImGui::TextUnformatted("Vector field");
        int wanted = dim;
        ImGui::RadioButton("R2", &wanted, 2);
        ImGui::SameLine();
        ImGui::RadioButton("R3", &wanted, 3);
        if (wanted != dim)
        {
            dim = wanted;
            loadPreset(dim == 2 ? kPresets2D[0] : kPresets3D[0]);
            trajectories.clear();   // a curve in R2 is not a curve in R3
            applyMode();
            applyField();
        }

        bool apply = false;
        const char* labels[3] = {"dx/dt", "dy/dt", "dz/dt"};
        for (int i = 0; i < dim; ++i)
        {
            ImGui::SetNextItemWidth(220.0f);
            apply |= ImGui::InputText(labels[i], comps[i], sizeof(comps[i]),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
        }
        if (ImGui::Button("Apply"))
            apply = true;

        ImGui::SameLine();
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::BeginCombo("preset", "choose..."))
        {
            int n = dim == 2 ? (int)(sizeof(kPresets2D) / sizeof(Preset))
                             : (int)(sizeof(kPresets3D) / sizeof(Preset));
            for (int i = 0; i < n; ++i)
            {
                const Preset& p = dim == 2 ? kPresets2D[i] : kPresets3D[i];
                if (ImGui::Selectable(p.name))
                {
                    loadPreset(p);
                    apply = true;
                }
            }
            ImGui::EndCombo();
        }

        if (apply)
            applyField();

        if (!parseError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", parseError.c_str());

        ImGui::TextWrapped("Variables: x, y%s, t (time). Constants: pi, e, tau. "
                           "Functions: sin cos tan sinh cosh tanh exp log sqrt abs. "
                           "Implicit products work: 2x, x y, (x-1)(x+1).",
                           dim == 3 ? ", z" : "");
    }

    void drawGlyphUI()
    {
        ImGui::TextUnformatted("Glyphs");
        ImGui::Checkbox("show arrows", &showArrows);
        ImGui::SameLine();
        ImGui::Checkbox("axes", &showAxes);

        // Rebuilding the glyph grid is cheap for a small lattice and decidedly not
        // cheap for 51^3, so a slider drag rebuilds live while the grid is small and
        // waits for the mouse release once it isn't.
        bool editedLive = false;
        bool editedDone = false;
        auto track = [&](bool changed) {
            editedLive = editedLive || changed;
            editedDone = editedDone || ImGui::IsItemDeactivatedAfterEdit();
        };

        track(ImGui::SliderInt("grid N", &gridN, 4, dim == 2 ? 80 : 50));
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
            track(ImGui::SliderFloat("head length", &headFraction, 0.1f, 0.7f,
                                     "%.2f of arrow"));
            track(ImGui::SliderFloat("head width", &headWidth, 0.04f, 0.5f,
                                     "%.2f of head length"));
        }

        arrowsDirty |= ImGui::Checkbox("length by magnitude", &lengthByMagnitude);
        track(ImGui::SliderFloat("hide below", &hideBelow, 0.0f, 0.5f, "%.3f of max"));
        if (dim == 3)
            track(ImGui::DragFloat("domain radius", &domainR, 0.2f, 0.5f, 200.0f));
        ImGui::SliderFloat("arrow alpha", &arrowAlpha, 0.1f, 1.0f);

        if (editedDone || (editedLive && (int)gridPoints.size() <= kLiveRebuildArrows))
            arrowsDirty = true;

        ImGui::Checkbox("auto color scale", &autoColorScale);
        if (!autoColorScale)
        {
            ImGui::SameLine();
            ImGui::SetNextItemWidth(110.0f);
            ImGui::DragFloat("max |f|", &manualColorMax, 0.05f, 1e-3f, 1e6f);
        }
        int arrows = arrowVertexCount / shaftVerticesPerArrow(dim, barbedHeads);
        ImGui::TextDisabled("sampled max |f| = %.4g   arrows drawn = %d",
                            sampledMaxMag, arrows);
        if (!barbedHeads && dim == 3)
        {
            float megabytes = (float)headVertexCount * kFloatsPerHeadVertex
                              * sizeof(float) / (1024.0f * 1024.0f);
            ImGui::TextDisabled("heads: %d-segment cone%s, %.1f MB",
                                headDetail.segments, headDetail.cap ? " + cap" : "",
                                megabytes);
        }
        ImGui::SliderFloat("alpha gradient", &arrowTailAlpha, 0.0f, 1.0f,
                           "tail %.2f of tip");

        // brightness by distance to the nearest particle: the point of it is that a
        // dense grid stops burying the curves, which matters most in R3 but reads
        // nicely in the plane too
        ImGui::Checkbox("brighten near particles", &distanceFade);
        if (distanceFade)
        {
            float span = 4.0f * (dim == 3 ? domainR : viewScale);
            ImGui::SliderFloat("full-bright radius", &fadeNear, 0.0f, span, "%.2f");
            ImGui::SliderFloat("fully faded at", &fadeFar, 0.1f, span, "%.2f");
            ImGui::SliderFloat("minimum alpha", &fadeMinAlpha, 0.0f, 1.0f, "%.3f");
            ImGui::SliderFloat("falloff curve", &fadeCurve, 0.25f, 4.0f, "%.2f");
            fadeFar = std::max(fadeFar, fadeNear + 0.01f);
            if (trajectories.empty())
                ImGui::TextDisabled("(no particles yet - nothing to brighten around)");
        }
    }

    void drawFlowUI()
    {
        ImGui::TextUnformatted("Flow");
        if (ImGui::Button(playing ? "Pause" : "Play"))
            playing = !playing;
        ImGui::SameLine();
        ImGui::Checkbox("reverse", &reverse);
        ImGui::SameLine();
        if (ImGui::Button("Restart curves"))
            restartTrajectories();

        ImGui::SliderFloat("speed", &timeScale, 0.05f, 10.0f, "%.2f x");

        int method = (int)integrator.method;
        ImGui::SetNextItemWidth(160.0f);
        if (ImGui::Combo("method", &method, "Euler\0RK2 (midpoint)\0RK4\0"))
            integrator.method = (engine::Integrator::Method)method;

        ImGui::SetNextItemWidth(160.0f);
        ImGui::DragScalar("max step h", ImGuiDataType_Double, &integrator.maxStep,
                          0.0005f, nullptr, nullptr, "%.4f");
        integrator.maxStep = std::clamp(integrator.maxStep, 1e-5, 0.5);

        if (ImGui::SliderInt("trail points", &trailPoints, 64, 60000))
            for (auto& traj : trajectories)
                traj->setMaxPoints(trailPoints);
        if (ImGui::SliderFloat("min point spacing", &minSpacing, 0.0f, 0.5f, "%.3f"))
            for (auto& traj : trajectories)
                traj->minSpacing = minSpacing;

        ImGui::SliderFloat("trail width", &trailWidth, 1.0f, 6.0f);
        ImGui::SliderFloat("particle size", &headSize, 3.0f, 24.0f);
        ImGui::Checkbox("color trails by speed", &trailBySpeed);
        ImGui::SameLine();
        ImGui::Checkbox("fade tail", &trailFade);
        if (trailFade)
        {
            ImGui::SetNextItemWidth(140.0f);
            ImGui::SliderFloat("oldest point alpha", &trailTailAlpha, 0.0f, 1.0f);
        }
        ImGui::TextDisabled("field time t = %.3f", fieldTime);
    }

    void drawTrajectoryUI()
    {
        ImGui::TextUnformatted("Trajectories");
        ImGui::TextDisabled("click in the view to drop one%s",
                            dim == 3 ? " (on the plane through the orbit center)" : "");

        ImGui::SetNextItemWidth(190.0f);
        ImGui::InputFloat3("seed", seedInput);
        ImGui::SameLine();
        if (ImGui::Button("Add"))
            addTrajectory(glm::vec3(seedInput[0], seedInput[1], seedInput[2]));

        ImGui::SetNextItemWidth(100.0f);
        ImGui::SliderInt("##randomN", &randomCount, 1, 64);
        ImGui::SameLine();
        if (ImGui::Button("Random seeds"))
            addRandomTrajectories(randomCount);
        ImGui::SameLine();
        if (ImGui::Button("Clear all"))
            trajectories.clear();

        if (ImGui::Checkbox("camera follows", &followEnabled))
        {
            if (!followEnabled && dim == 3)
            {
                releaseRide();
                app->camera().orbitCenter = glm::vec3(0.0f);
            }
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f);
        ImGui::SliderInt("target", &followIndex, 0,
                         std::max(0, (int)trajectories.size() - 1));

        if (dim == 3)
        {
            if (ImGui::Checkbox("ride the particle", &rideCamera) && !rideCamera)
                releaseRide();
            if (followEnabled && rideCamera)
            {
                ImGui::SliderFloat("chase distance", &chaseDistance, 0.1f,
                                   std::max(10.0f, domainR * 2.0f), "%.2f");
                ImGui::SliderFloat("chase height", &chaseHeight, -domainR, domainR, "%.2f");
                ImGui::SliderFloat("chase smoothing", &chaseResponse, 0.2f, 20.0f,
                                   "%.1f /s");
                ImGui::TextDisabled("riding: Q/E pull in-out, W/S raise-lower");
            }
            else
            {
                ImGui::SliderFloat("orbit speed", &orbitSpeed, 0.1f, 4.0f, "%.2f rad/s");
                ImGui::SliderFloat("zoom speed", &zoomSpeed, 0.5f,
                                   std::max(20.0f, domainR * 2.0f), "%.1f /s");
            }
        }

        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            engine::Trajectory& traj = *trajectories[i];
            glm::vec3 p = traj.head();
            ImGui::PushID((int)i);
            if (dim == 2)
                ImGui::Text("%2d: (%7.3f, %7.3f)  |v| %.3g  %d pts%s", (int)i, p.x, p.y,
                            traj.speed(), traj.size(),
                            traj.diverged() ? "  blown up" : "");
            else
                ImGui::Text("%2d: (%6.2f, %6.2f, %6.2f)  |v| %.3g  %d pts%s", (int)i,
                            p.x, p.y, p.z, traj.speed(), traj.size(),
                            traj.diverged() ? "  blown up" : "");
            ImGui::SameLine();
            if (ImGui::SmallButton("follow"))
            {
                followIndex = (int)i;
                followEnabled = true;
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("x"))
            {
                trajectories.erase(trajectories.begin() + (long)i);
                seeds.erase(seeds.begin() + (long)i);
                ImGui::PopID();
                break;
            }
            ImGui::PopID();
        }
    }

    // ---- field + view state -------------------------------------------------

    void loadPreset(const Preset& p)
    {
        for (int i = 0; i < 3; ++i)
            std::snprintf(comps[i], sizeof(comps[i]), "%s", p.comps[i]);
    }

    // Parse the typed components. A parse error keeps the previously parsed field
    // drawing, exactly like the function box in riemann_sphere.
    void applyField()
    {
        std::vector<std::string> sources;
        for (int i = 0; i < dim; ++i)
            sources.push_back(comps[i]);

        if (field.set(dim, sources, parseError))
        {
            parseError.clear();
            arrowsDirty = true;
        }
    }

    // Camera/view setup per dimension: R3 orbits, R2 drops the 3D camera entirely
    // for a pan/zoom orthographic window (the movement keys are borrowed, so the
    // camera stops listening — the same deal as riemann_sphere's plane mode).
    void applyMode()
    {
        engine::Camera& cam = app->camera();
        if (dim == 3)
        {
            cam.inputEnabled = true;
            cam.orbitCenter = glm::vec3(0.0f);
            cam.orbitRadius = 3.0f * domainR;
            cam.orbitRadiusMin = 0.1f;
            cam.orbitSlowZone = 0.0f;
            cam.farPlane = std::max(500.0f, 20.0f * domainR);

            // zoom in world units and the chase rig scale with the scene; the orbit
            // rate is in radians and deliberately does not
            // a cube of solid-headed arrows gets dense fast; 12 per axis is the
            // readable default (the slider still goes to 26)
            gridN = 12;
            zoomSpeed = std::max(1.0f, domainR * 0.5f);
            distanceFade = true;   // a cube of glyphs badly needs it
            fadeNear = 0.0f;
            fadeFar = domainR * 1.3f;
            chaseDistance = std::max(0.5f, domainR * 0.3f);
            chaseHeight = domainR * 0.08f;
            cam.orbitAngularSpeed = orbitSpeed;
            cam.speed = zoomSpeed;
        }
        else
        {
            gridN = 26;
            // off by default in the plane: one layer of glyphs is not clutter, and
            // the fade distances are fixed world units, so zooming out would dim
            // everything. Still available from the checkbox.
            distanceFade = false;
            fadeNear = 0.0f;
            fadeFar = viewScale * 1.5f;
            releaseRide();
            cam.inputEnabled = false;
        }
        arrowsDirty = true;
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
        float sx = viewScale * app->window().aspect();
        float sy = viewScale;
        view = glm::mat4(1.0f);
        projection = glm::ortho(viewCenter.x - sx, viewCenter.x + sx,
                                viewCenter.y - sy, viewCenter.y + sy, -1.0f, 1.0f);
    }

    // R2 pan/zoom on WASD + QE, in domain units so panning slows as you zoom in.
    // While riding a trajectory the orbit camera is switched off, so the same keys
    // drive the chase rig instead.
    void handleViewKeys(float dt)
    {
        if (ImGui::GetIO().WantCaptureKeyboard)
            return;

        const engine::Window& win = app->window();

        if (dim == 2)
        {
            float pan = viewScale * dt;
            if (win.keyDown('A')) viewCenter.x -= pan;
            if (win.keyDown('D')) viewCenter.x += pan;
            if (win.keyDown('S')) viewCenter.y -= pan;
            if (win.keyDown('W')) viewCenter.y += pan;
            if (win.keyDown('Q')) viewScale *= std::exp(-dt);
            if (win.keyDown('E')) viewScale *= std::exp(dt);
            return;
        }

        if (rideActive)
        {
            if (win.keyDown('Q')) chaseDistance *= std::exp(-dt);
            if (win.keyDown('E')) chaseDistance *= std::exp(dt);
            if (win.keyDown('W')) chaseHeight += chaseDistance * dt;
            if (win.keyDown('S')) chaseHeight -= chaseDistance * dt;
            chaseDistance = std::clamp(chaseDistance, 0.05f, 1000.0f);
        }
    }

    void handleSeedClick()
    {
        if (!ImGui::IsMouseClicked(ImGuiMouseButton_Left) ||
            ImGui::GetIO().WantCaptureMouse)
            return;

        glm::vec2 ndc = app->window().cursorPosNormalized() * 2.0f - 1.0f;

        if (dim == 2)
        {
            glm::vec2 p = viewCenter + ndc * glm::vec2(viewScale * app->window().aspect(),
                                                       viewScale);
            addTrajectory(glm::vec3(p, 0.0f));
            return;
        }

        // R3: shoot the cursor ray and stop it on the plane through the orbit center
        // facing the camera — the one depth the click can unambiguously mean.
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

        float s = glm::dot(cam.orbitCenter - origin, cam.front) / denom;
        if (s <= 0.0f)
            return;
        addTrajectory(origin + dir * s);
    }

    void followCamera(float dt)
    {
        if (!followEnabled || trajectories.empty())
        {
            releaseRide();
            return;
        }

        int i = std::clamp(followIndex, 0, (int)trajectories.size() - 1);
        const engine::Trajectory& traj = *trajectories[i];
        glm::vec3 head = traj.head();

        if (dim == 2)
        {
            viewCenter = glm::vec2(head);       // the window rides along in R2
            releaseRide();
            return;
        }

        engine::Camera& cam = app->camera();
        if (!rideCamera)
        {
            cam.orbitCenter = head;             // orbit keys still work, around the head
            releaseRide();
            return;
        }

        // Ride: sit behind the particle along its own velocity and look at it. The
        // direction is smoothed with a frame-rate independent exponential filter —
        // raw velocity swings (Lorenz lobe transitions) would whip the camera around.
        glm::vec3 v = traj.velocity();
        float sp = glm::length(v);
        glm::vec3 dir = sp > 1e-6f ? v / sp : chaseDir;
        chaseDir = glm::normalize(glm::mix(chaseDir, dir, 1.0f - std::exp(-chaseResponse * dt)));

        // world up, unless we are flying nearly straight up or down — then lookAt
        // would be degenerate
        glm::vec3 up(0.0f, 1.0f, 0.0f);
        if (std::abs(glm::dot(chaseDir, up)) > 0.97f)
            up = glm::vec3(0.0f, 0.0f, 1.0f);

        glm::vec3 pos = head - chaseDir * chaseDistance + up * chaseHeight;
        cam.position = pos;
        cam.front = glm::normalize(head - pos);
        cam.up = up;
        cam.orbitCenter = head;   // leaves the orbit camera sane when the ride ends

        if (!rideActive)
        {
            // orbit input would recompute position from orbitCenter every frame and
            // undo all of the above
            cam.inputEnabled = false;
            rideActive = true;
        }
    }

    void releaseRide()
    {
        if (!rideActive)
            return;
        engine::Camera& cam = app->camera();
        cam.up = glm::vec3(0.0f, 1.0f, 0.0f);
        cam.inputEnabled = (dim == 3);
        rideActive = false;
    }

    // ---- trajectories -------------------------------------------------------

    void addTrajectory(const glm::vec3& seed)
    {
        if (!field.valid() || trajectories.size() >= kMaxTrajectories)
            return;

        double s[3] = {seed.x, seed.y, seed.z};
        auto traj = std::make_unique<engine::Trajectory>(dim, s, fieldTime, trailPoints);
        traj->minSpacing = minSpacing;
        trajectories.push_back(std::move(traj));
        seeds.push_back(seed);

        seedInput[0] = seed.x;
        seedInput[1] = seed.y;
        seedInput[2] = seed.z;
        followIndex = (int)trajectories.size() - 1;
    }

    void addRandomTrajectories(int n)
    {
        std::uniform_real_distribution<float> unit(-1.0f, 1.0f);
        for (int k = 0; k < n; ++k)
        {
            glm::vec3 seed(0.0f);
            if (dim == 2)
            {
                seed.x = viewCenter.x + unit(rng) * viewScale * app->window().aspect();
                seed.y = viewCenter.y + unit(rng) * viewScale;
            }
            else
            {
                seed = glm::vec3(unit(rng), unit(rng), unit(rng)) * domainR;
            }
            addTrajectory(seed);
        }
    }

    // Send every curve back to the point it was seeded at, at the current field time
    // (a time-dependent field flows differently from a different t).
    void restartTrajectories()
    {
        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            double s[3] = {seeds[i].x, seeds[i].y, seeds[i].z};
            trajectories[i]->reset(s, fieldTime);
        }
    }

    // ---- glyph grid ---------------------------------------------------------

    float colorMax() const
    {
        return autoColorScale ? std::max(sampledMaxMag, 1e-6f) : manualColorMax;
    }

    // Re-sample the field on a regular grid and rebuild the glyph geometry. A few
    // thousand AST evaluations, cheap enough to run every frame while a
    // time-dependent field animates.
    void rebuildArrows()
    {
        arrowsDirty = false;
        arrowVertexCount = 0;
        headVertexCount = 0;
        sampledMaxMag = 0.0f;
        if (!field.valid())
            return;

        gridPoints.clear();
        float spacing = 0.0f;

        if (dim == 2)
        {
            float sx = viewScale * app->window().aspect();
            float sy = viewScale;
            spacing = 2.0f * sy / (float)gridN;
            // snap the grid to multiples of `spacing` so glyphs stay anchored to the
            // plane while panning instead of crawling along with the window
            float x0 = std::floor((viewCenter.x - sx) / spacing) * spacing;
            float y0 = std::floor((viewCenter.y - sy) / spacing) * spacing;
            int nx = (int)std::ceil(2.0f * sx / spacing) + 2;
            int ny = gridN + 2;
            for (int j = 0; j < ny; ++j)
                for (int i = 0; i < nx; ++i)
                    gridPoints.emplace_back(x0 + i * spacing, y0 + j * spacing, 0.0f);
        }
        else
        {
            spacing = 2.0f * domainR / (float)gridN;
            for (int k = 0; k <= gridN; ++k)
                for (int j = 0; j <= gridN; ++j)
                    for (int i = 0; i <= gridN; ++i)
                        gridPoints.emplace_back(-domainR + i * spacing,
                                                -domainR + j * spacing,
                                                -domainR + k * spacing);
        }

        if ((int)gridPoints.size() > kMaxArrows)
            gridPoints.resize(kMaxArrows);

        // pass 1: sample, and find the magnitude that tops out the color ramp
        gridVectors.clear();
        gridVectors.reserve(gridPoints.size());
        for (const glm::vec3& p : gridPoints)
        {
            double x[3] = {p.x, p.y, p.z};
            double v[3] = {0.0, 0.0, 0.0};
            field.eval(x, fieldTime, v);
            glm::vec3 vec((float)v[0], (float)v[1], dim == 3 ? (float)v[2] : 0.0f);
            if (!std::isfinite(vec.x) || !std::isfinite(vec.y) || !std::isfinite(vec.z))
                vec = glm::vec3(0.0f);
            gridVectors.push_back(vec);
            sampledMaxMag = std::max(sampledMaxMag, glm::length(vec));
        }

        // pass 2: geometry. Uniform-length arrows by default — direction is the
        // glyph's job, magnitude is the ramp's.
        float scaleMax = colorMax();
        float cutoff = hideBelow * scaleMax;
        arrowSpacing = spacing;
        headDetail = headLod((int)gridPoints.size());
        arrowBuffer.clear();
        headBuffer.clear();
        arrowBuffer.reserve(gridPoints.size()
                            * (size_t)shaftVerticesPerArrow(dim, barbedHeads)
                            * kFloatsPerShaftVertex);
        headBuffer.reserve(gridPoints.size()
                           * (size_t)headVerticesPerArrow(dim, barbedHeads, headDetail)
                           * kFloatsPerHeadVertex);

        for (size_t i = 0; i < gridPoints.size(); ++i)
        {
            float mag = glm::length(gridVectors[i]);
            if (mag <= 1e-9f || mag < cutoff)
                continue;

            glm::vec3 dir = gridVectors[i] / mag;
            float len = spacing * arrowScale;
            if (lengthByMagnitude)
                len *= std::clamp(mag / std::max(scaleMax, 1e-6f), 0.05f, 1.0f);

            pushArrow(arrowBuffer, headBuffer, dim, gridPoints[i], dir, len, mag,
                      barbedHeads, headFraction, headWidth, headDetail);
        }

        int needed = (int)arrowBuffer.size() / kFloatsPerShaftVertex;
        if (needed == 0)
            return;

        if (!arrowMesh || arrowMesh->vertexCapacity() < needed)
            arrowMesh = std::make_unique<engine::Mesh>(needed + needed / 4 + 64,
                                                       std::vector<int>{3, 1, 1});

        arrowMesh->setVertices(arrowBuffer.data(), arrowBuffer.size(), 0);
        arrowVertexCount = needed;

        int heads = (int)headBuffer.size() / kFloatsPerHeadVertex;
        if (heads > 0)
        {
            if (!headMesh || headMesh->vertexCapacity() < heads)
                headMesh = std::make_unique<engine::Mesh>(heads + heads / 4 + 64,
                                                          std::vector<int>{3, 1, 1, 1});
            headMesh->setVertices(headBuffer.data(), headBuffer.size(), 0);
        }
        headVertexCount = heads;
    }

    // Distance falloff: the arrows near a particle stay solid, everything far from
    // every particle sinks toward `fadeMinAlpha`, so a dense R3 grid stops burying the
    // curves. The followed curve goes first, then the rest, up to the shader's 8.
    void setFocusUniforms()
    {
        if (!distanceFade || trajectories.empty())
        {
            shader.setInt("u_focusCount", 0);
            return;
        }

        int count = 0;
        int preferred = std::clamp(followIndex, 0, (int)trajectories.size() - 1);
        for (int pass = 0; pass < 2 && count < kMaxFocus; ++pass)
        {
            for (size_t i = 0; i < trajectories.size() && count < kMaxFocus; ++i)
            {
                bool isPreferred = (int)i == preferred;
                if ((pass == 0) != isPreferred)
                    continue;
                shader.setVec3("u_focus[" + std::to_string(count) + "]",
                               trajectories[i]->head());
                ++count;
            }
        }

        shader.setInt("u_focusCount", count);
        shader.setFloat("u_fadeNear", fadeNear);
        shader.setFloat("u_fadeFar", std::max(fadeFar, fadeNear + 1e-3f));
        shader.setFloat("u_fadeMin", fadeMinAlpha);
        shader.setFloat("u_fadeCurve", fadeCurve);
    }

    // evenly spaced hues, so several curves stay apart when trails are not colored
    // by speed
    glm::vec3 trailColor(size_t i) const
    {
        float h = std::fmod((float)i * 0.618034f, 1.0f) * 6.0f;
        float f = h - std::floor(h);
        switch ((int)h)
        {
        case 0:  return glm::vec3(1.0f, f, 0.0f);
        case 1:  return glm::vec3(1.0f - f, 1.0f, 0.0f);
        case 2:  return glm::vec3(0.0f, 1.0f, f);
        case 3:  return glm::vec3(0.0f, 1.0f - f, 1.0f);
        case 4:  return glm::vec3(f, 0.0f, 1.0f);
        default: return glm::vec3(1.0f, 0.0f, 1.0f - f);
        }
    }

    // 51^3 = 132651, the densest R3 lattice the grid N slider can ask for
    static constexpr int kMaxArrows = 140000;
    // above this, glyph-geometry sliders apply on release rather than every frame
    static constexpr int kLiveRebuildArrows = 20000;
    static constexpr int kMaxFocus = 8;   // matches kMaxFocus in the vertex shader
    static constexpr size_t kMaxTrajectories = 64;

    engine::App* app = nullptr;
    engine::Shader shader;
    std::unique_ptr<engine::Mesh> arrowMesh;
    std::unique_ptr<engine::Mesh> headMesh;
    std::unique_ptr<engine::Mesh> axisMesh;

    engine::VectorField field;
    engine::Integrator integrator;
    engine::Integrator::Field rhs;
    std::vector<std::unique_ptr<engine::Trajectory>> trajectories;
    std::vector<glm::vec3> seeds;   // where each curve started, for "Restart curves"

    int dim = 2;
    char comps[3][256] = {"y", "-sin(x) - 0.2y", "0"};
    std::string parseError;

    // glyph grid
    bool showArrows = true;
    bool showAxes = true;
    bool arrowsDirty = true;
    bool lengthByMagnitude = false;
    int gridN = 26;
    float arrowScale = 0.8f;
    float arrowAlpha = 1.0f;
    float arrowTailAlpha = 0.25f;   // opacity at the arrow's tail, 1 = no gradient
    float arrowWidth = 1.5f;
    bool barbedHeads = false;       // false = solid cone/triangle head, true = barbs
    float headFraction = 0.36f;     // head length as a fraction of the arrow length
    float headWidth = 0.14f;        // cone radius as a fraction of the head length
    HeadLod headDetail{7, true};    // chosen per rebuild from the arrow count
    float arrowSpacing = 1.0f;      // grid spacing of the last rebuild, for tip sizing
    // glyph brightness rides from full at a particle down to fadeMinAlpha at fadeFar
    bool distanceFade = false;      // applyMode turns it on in R3
    float fadeNear = 0.0f;          // radius of the fully-bright bubble (0 = none)
    float fadeFar = 26.0f;
    float fadeMinAlpha = 0.10f;
    float fadeCurve = 1.0f;         // 1 = linear, <1 hugs the particle, >1 reaches out
    float hideBelow = 0.0f;
    float domainR = 20.0f;       // R3 sampling cube half-size
    float sampledMaxMag = 0.0f;
    bool autoColorScale = true;
    float manualColorMax = 10.0f;
    int arrowVertexCount = 0;
    int headVertexCount = 0;
    std::vector<glm::vec3> gridPoints;
    std::vector<glm::vec3> gridVectors;
    std::vector<float> arrowBuffer;
    std::vector<float> headBuffer;

    // flow
    bool playing = true;
    bool reverse = false;
    float timeScale = 1.0f;
    double fieldTime = 0.0;
    int trailPoints = 4000;
    float minSpacing = 0.0f;
    float trailWidth = 2.0f;
    float headSize = 11.0f;
    bool trailBySpeed = true;
    bool trailFade = true;          // oldest end of the trail fades out
    float trailTailAlpha = 0.0f;

    // seeding + camera follow
    float seedInput[3] = {1.0f, 1.0f, 1.0f};
    int randomCount = 8;
    bool followEnabled = false;
    int followIndex = 0;
    std::mt19937 rng{1234};

    // R3 camera: orbit rates, and the chase rig used when riding a particle
    float orbitSpeed = 1.2f;        // rad/s
    float zoomSpeed = 8.0f;         // world units/s
    bool rideCamera = true;
    bool rideActive = false;        // the camera is currently being driven by us
    glm::vec3 chaseDir{0.0f, 0.0f, 1.0f};
    float chaseDistance = 6.0f;
    float chaseHeight = 1.5f;
    float chaseResponse = 2.5f;     // 1/s of the direction smoothing filter

    // R2 view window
    glm::vec2 viewCenter{0.0f, 0.0f};
    float viewScale = 5.0f;
    glm::vec3 lastView2D{0.0f};
    float lastAspect = 0.0f;
};

} // namespace

std::unique_ptr<engine::Sim> makeVectorFieldSim()
{
    return std::make_unique<VectorFieldSim>();
}
