#include "SurfaceRendererRayMarchedSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/Expression.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>
#include <engine/SolutionCurve.h>

#include <imgui.h>
#include <cmath>
#include <complex>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string readFile(const char* path)
{
    std::ifstream file(path);
    std::stringstream stream;
    stream << file.rdbuf();
    return stream.str();
}

// UI presets; all use the free parameter a so the slider distorts them live
struct SurfacePreset {
    const char* name;
    const char* f;
    float c, a, range;
};
constexpr SurfacePreset kPresets[] = {
    {"tangle cube", "x^4 + y^4 + z^4 - a*(x^2 + y^2 + z^2)", -11.8f, 5.0f, 4.0f},
    {"torus", "(a - sqrt(x^2 + y^2))^2 + z^2 - 1", 0.0f, 3.0f, 5.0f},
    {"rippled sphere", "x^2 + y^2 + z^2 + a*sin(3*x)*sin(3*y)*sin(3*z)", 4.0f, 0.8f, 3.5f},
    {"gyroid", "sin(a*x)*cos(a*y) + sin(a*y)*cos(a*z) + sin(a*z)*cos(a*x)", 0.0f, 1.5f, 4.0f},
    {"klein bottle",
     "(x^2+y^2+z^2+2*y-1)*((x^2+y^2+z^2-2*y-1)^2-8*z^2) + a*x*z*(x^2+y^2+z^2-2*y-1)",
     0.0f, 16.0f, 4.0f},
};

// Raymarched graphing calculator for implicit surfaces: the typed f(x,y,z)
// and the level c define the level set f = c, rendered per-pixel in the
// fragment shader 
class SurfaceRendererRayMarchedSim : public engine::Sim {
public:
    const char* name() const override { return "surface_renderer_ray_marched"; }

    // hand the camera back if destroyed mid-ride (sim switch from the menu)
    ~SurfaceRendererRayMarchedSim() override
    {
        if (app && followCam)
        {
            app->camera().inputEnabled = true;
            app->camera().up = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    void init(engine::App& app) override
    {
        this->app = &app;

        app.camera().inputEnabled = true; // in case a previous sim disabled it
        app.camera().orbitRadius = 9.0f;
        app.camera().orbitRadiusMin = 0.5f;
        app.camera().orbitSlowZone = 0.0f;

        quadMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::planeVertices(),
            engine::Shapes::planeIndices(),
            std::vector<int>{3});
        axisMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::axisVertices(),
            engine::Shapes::axisIndices(),
            std::vector<int>{3});

        vertexSrc = readFile(SIMS_DIR "surface_renderer_ray_marched/shaders/raymarchVertex.glsl");
        fragTemplate = readFile(SIMS_DIR "surface_renderer_ray_marched/shaders/raymarchTemplate.glsl");

        axisShader = engine::Shader(
            SIMS_DIR "surface_renderer_ray_marched/shaders/vertex.glsl",
            SIMS_DIR "surface_renderer_ray_marched/shaders/fragment.glsl");

        applyFunction(funcBuf);
        applyField(); // parse the default V so the demo works out of the box
    }

    void update(float dt) override
    {
        time += dt;
        if (particleDemo)
        {
            particle.update(dt, time);
            if (followCam)
                updateFollowCam(dt);
        }
    }

    // Chase camera riding the particle: hover followHeight above the surface
    // (along the normal), trail slightly behind the motion direction, look a
    // bit ahead of the particle. Exponentially smoothed against RK4 jitter.
    void updateFollowCam(float dt)
    {
        engine::Camera& cam = app->camera();
        glm::vec3 p = particle.position();

        glm::vec3 g = gradF(p, time);
        glm::vec3 n = glm::dot(g, g) > 1e-12f ? glm::normalize(g)
                                              : glm::vec3(0.0f, 1.0f, 0.0f);
        glm::vec3 vel = particle.field ? particle.field(p, time) : glm::vec3(0.0f);
        glm::vec3 tangential = vel - glm::dot(vel, n) * n;
        if (glm::dot(tangential, tangential) > 1e-10f)
            followDir = glm::normalize(tangential); // keep last dir at stagnation

        glm::vec3 wantPos = p + n * followHeight - followDir * (0.6f * followHeight);
        glm::vec3 wantFront = glm::normalize(p + followDir * 0.8f - wantPos);

        float k = 1.0f - std::exp(-8.0f * dt);
        camPosSmooth += (wantPos - camPosSmooth) * k;
        camFrontSmooth += (wantFront - camFrontSmooth) * k;
        camUpSmooth += (n - camUpSmooth) * k;

        cam.position = camPosSmooth;
        cam.front = glm::normalize(camFrontSmooth);
        cam.up = glm::normalize(camUpSmooth);
    }

    void setFollowCam(bool enabled)
    {
        followCam = enabled;
        app->camera().inputEnabled = !enabled; // WASD/QE belong to the orbit cam
        if (enabled)
        {
            // start the smoothing from wherever the camera currently is
            camPosSmooth = app->camera().position;
            camFrontSmooth = app->camera().front;
            camUpSmooth = app->camera().up;
        }
        else
        {
            app->camera().up = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix();
        glm::mat4 viewProj = projection * view;

        glLineWidth(4.0f);
        axisShader.use();
        axisShader.setMat4("model", glm::mat4(1.0f));
        axisShader.setMat4("view", view);
        axisShader.setMat4("projection", projection);
        axisMesh->draw(GL_LINES);

        // fullscreen pass; hits write gl_FragDepth, misses discard, so the
        // axis lines drawn above intersect the surface correctly
        raymarchShader.use();
        raymarchShader.setMat4("u_viewProj", viewProj);
        raymarchShader.setMat4("u_invViewProj", glm::inverse(viewProj));
        raymarchShader.setFloat("u_c", levelC);
        raymarchShader.setFloat("u_range", range);
        raymarchShader.setFloat("u_time", time);
        raymarchShader.setFloat("u_a", paramA);
        quadMesh->draw(GL_TRIANGLES);

        // real geometry: composites with the surface through gl_FragDepth
        if (particleDemo)
            particle.draw(view, projection);
    }

    void drawUI() override
    {
        if (ImGui::BeginCombo("preset",
                              presetIndex >= 0 ? kPresets[presetIndex].name : "custom"))
        {
            for (int i = 0; i < (int)(sizeof(kPresets) / sizeof(kPresets[0])); ++i)
            {
                if (ImGui::Selectable(kPresets[i].name, i == presetIndex))
                {
                    presetIndex = i;
                    snprintf(funcBuf, sizeof(funcBuf), "%s", kPresets[i].f);
                    levelC = kPresets[i].c;
                    paramA = kPresets[i].a;
                    range = kPresets[i].range;
                    applyFunction(funcBuf);
                    if (particleDemo)
                        resetParticle();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::TextUnformatted("f(x,y,z) =");
        ImGui::SameLine();
        bool apply = ImGui::InputText("##function", funcBuf, sizeof(funcBuf),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        apply |= ImGui::Button("Apply");
        if (apply)
        {
            presetIndex = -1; // hand-edited: no longer a preset
            applyFunction(funcBuf);
        }

        if (!parseError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", parseError.c_str());

        // c and a are uniforms: dragging them reshapes the surface live,
        // no recompile
        ImGui::DragFloat("c", &levelC, 0.02f);
        ImGui::DragFloat("a (distort)", &paramA, 0.02f);
        ImGui::DragFloat("domain radius", &range, 0.05f, 0.5f, 50.0f);

        ImGui::Separator();
        if (ImGui::Checkbox("flow demo (particle on the surface)", &particleDemo))
        {
            if (particleDemo)
                resetParticle();
            else if (followCam)
                setFollowCam(false); // hand the camera back with the demo
        }
        if (particleDemo)
        {
            ImGui::SameLine();
            if (ImGui::Button("reset##particle"))
                resetParticle();

            // the ambient field V, tangent-projected onto the surface
            ImGui::TextUnformatted("V =");
            ImGui::SameLine();
            bool applyV = false;
            float w = ImGui::GetContentRegionAvail().x * 0.22f;
            ImGui::SetNextItemWidth(w);
            applyV |= ImGui::InputText("##vx", vBufX, sizeof(vBufX), ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(w);
            applyV |= ImGui::InputText("##vy", vBufY, sizeof(vBufY), ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();
            ImGui::SetNextItemWidth(w);
            applyV |= ImGui::InputText("##vz", vBufZ, sizeof(vBufZ), ImGuiInputTextFlags_EnterReturnsTrue);
            ImGui::SameLine();
            applyV |= ImGui::Button("Apply##field");
            if (applyV)
                applyField();
            if (!fieldError.empty())
                ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", fieldError.c_str());

            ImGui::Checkbox("trail", &particle.trailEnabled);
            ImGui::DragFloat("step size", &particle.stepSize, 0.001f, 0.0005f, 0.2f, "%.4f");

            bool follow = followCam;
            if (ImGui::Checkbox("follow cam (ride the particle)", &follow))
                setFollowCam(follow);
            if (followCam)
            {
                ImGui::SameLine();
                ImGui::SetNextItemWidth(120.0f);
                ImGui::DragFloat("height##cam", &followHeight, 0.01f, 0.05f, 3.0f);
            }
        }

        ImGui::TextWrapped(
            "Level set f(x,y,z) = c, raymarched. Variables: x, y, z, t (time), "
            "a (the distort slider). Constants: i, pi, e, tau. The real part of "
            "the value is used. Teal side: f > c, amber side: f < c. Surface "
            "outside the domain radius is not searched.");
    }

private:
    // Parse the typed function, generate GLSL for it and hot-swap the
    // raymarching shader. On a parse error the previous shader keeps rendering.
    void applyFunction(const char* source)
    {
        engine::Expression expr =
            engine::Expression::parse(source, {"x", "y", "z", "t", "a"}, parseError);
        if (!expr.valid())
            return;

        // real coordinates enter the complex evaluator as vec2(v, 0)
        std::string fc = "vec2 fc(vec3 p) {\n    return "
                       + expr.glsl({"vec2(p.x, 0.0)", "vec2(p.y, 0.0)",
                                    "vec2(p.z, 0.0)", "vec2(u_time, 0.0)",
                                    "vec2(u_a, 0.0)"})
                       + ";\n}";
        std::string frag = fragTemplate;
        replaceOnce(frag, "//__COMPLEX_LIB__", engine::Expression::glslComplexLib());
        replaceOnce(frag, "//__F__", fc);

        raymarchShader = engine::Shader::fromSources(vertexSrc, frag);

        // keep a CPU copy of the same AST: the demo particle's field evaluates
        // it to flow on the exact surface the pixels are drawing
        cpuExpr = std::move(expr);
        setParticleField();
    }

    // F = Re f(p) - c, evaluated on the CPU through the parsed AST
    float evalF(const glm::vec3& p, float t) const
    {
        std::vector<std::complex<double>> vals = {
            {p.x, 0.0}, {p.y, 0.0}, {p.z, 0.0}, {t, 0.0}, {paramA, 0.0}};
        return (float)cpuExpr.eval(vals).real() - levelC;
    }

    glm::vec3 gradF(const glm::vec3& p, float t) const
    {
        const float e = 1e-3f;
        return glm::vec3(
                   evalF(p + glm::vec3(e, 0, 0), t) - evalF(p - glm::vec3(e, 0, 0), t),
                   evalF(p + glm::vec3(0, e, 0), t) - evalF(p - glm::vec3(0, e, 0), t),
                   evalF(p + glm::vec3(0, 0, e), t) - evalF(p - glm::vec3(0, 0, e), t))
               / (2.0f * e);
    }

    // Parse the three typed components of the ambient field V. Same contract
    // as f: on a parse error the previous valid field keeps running.
    void applyField()
    {
        const std::vector<std::string> vars = {"x", "y", "z", "t", "a"};
        std::string err;
        engine::Expression ex = engine::Expression::parse(vBufX, vars, err);
        if (!ex.valid()) { fieldError = "Vx: " + err; return; }
        engine::Expression ey = engine::Expression::parse(vBufY, vars, err);
        if (!ey.valid()) { fieldError = "Vy: " + err; return; }
        engine::Expression ez = engine::Expression::parse(vBufZ, vars, err);
        if (!ez.valid()) { fieldError = "Vz: " + err; return; }
        fieldError.clear();
        vExprX = std::move(ex);
        vExprY = std::move(ey);
        vExprZ = std::move(ez);
    }

    // Surface flow: the typed ambient field V projected onto the tangent
    // plane (v = V - (V.n)n), plus a feedback term -F grad/|grad|^2 that
    // pulls the particle back onto f = c, so plain RK4 stays glued to it.
    void setParticleField()
    {
        particle.field = [this](const glm::vec3& p, float t) -> glm::vec3 {
            if (!cpuExpr.valid() || !vExprX.valid())
                return glm::vec3(0.0f);
            glm::vec3 g = gradF(p, t);
            float len2 = glm::dot(g, g);
            if (len2 < 1e-12f)
                return glm::vec3(0.0f);
            glm::vec3 n = g / std::sqrt(len2);

            std::vector<std::complex<double>> vals = {
                {p.x, 0.0}, {p.y, 0.0}, {p.z, 0.0}, {t, 0.0}, {paramA, 0.0}};
            glm::vec3 V((float)vExprX.eval(vals).real(),
                        (float)vExprY.eval(vals).real(),
                        (float)vExprZ.eval(vals).real());

            glm::vec3 tangent = V - glm::dot(V, n) * n;
            return tangent - 4.0f * (evalF(p, t) / len2) * g;
        };
    }

    void resetParticle()
    {
        particle.reset(glm::vec3(0.5f * range, 0.3f, 0.5f));
    }

    static void replaceOnce(std::string& text, const std::string& placeholder,
                            const std::string& value)
    {
        size_t at = text.find(placeholder);
        if (at != std::string::npos)
            text.replace(at, placeholder.size(), value);
    }

    engine::App* app = nullptr;
    std::unique_ptr<engine::Mesh> quadMesh;
    std::unique_ptr<engine::Mesh> axisMesh;
    engine::Shader raymarchShader;
    engine::Shader axisShader;
    engine::Expression cpuExpr;      // CPU twin of the shader's f
    engine::SolutionCurve particle;
    bool particleDemo = false;
    // follow-cam state
    bool followCam = false;
    float followHeight = 0.4f;
    glm::vec3 followDir{1.0f, 0.0f, 0.0f};       // last motion direction
    glm::vec3 camPosSmooth{0.0f};
    glm::vec3 camFrontSmooth{0.0f, 0.0f, -1.0f};
    glm::vec3 camUpSmooth{0.0f, 1.0f, 0.0f};
    // typed ambient field V(x,y,z,t), one expression per component
    char vBufX[128] = "-y";
    char vBufY[128] = "x";
    char vBufZ[128] = "0.4";
    engine::Expression vExprX, vExprY, vExprZ;
    std::string fieldError;

    std::string vertexSrc;
    std::string fragTemplate;
    // default: the "tangle cube" preset (a = 5; topology changes as c crosses
    // its critical values, e.g. 0, -6.25, -12.5, -18.75 at a = 5)
    char funcBuf[256] = "x^4 + y^4 + z^4 - a*(x^2 + y^2 + z^2)";
    std::string parseError;
    int presetIndex = 0;
    float levelC = -11.8f;
    float paramA = 5.0f;
    float range = 4.0f;
    float time = 0.0f;
};

} // namespace

std::unique_ptr<engine::Sim> makeSurfaceRendererRayMarchedSim()
{
    return std::make_unique<SurfaceRendererRayMarchedSim>();
}
