#include "RiemannSphereSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/Expression.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>

#include <gtc/matrix_transform.hpp>
#include <imgui.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readFile(const char* path)
{
    std::ifstream file(path);
    std::stringstream stream;
    stream << file.rdbuf();
    return stream.str();
}

class RiemannSphereSim : public engine::Sim {
public:
    static constexpr float kSphereRadius = 2.5f;

    const char* name() const override { return "riemann_sphere"; }

    void init(engine::App& app) override
    {
        this->app = &app;

        // camera starts framing the whole sphere, stays outside it, and slows
        // down near its surface
        app.camera().orbitRadius = 7.0f;
        app.camera().orbitRadiusMin = kSphereRadius + 0.2f;
        app.camera().orbitSlowZone = 2.0f;

        sphereMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::sphereVertices(100, 100, kSphereRadius),
            engine::Shapes::sphereIndices(100, 100),
            std::vector<int>{3, 3, 2});
        planeMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::planeVertices(),
            engine::Shapes::planeIndices(),
            std::vector<int>{3});
        
        axisMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::axisVertices(),
            engine::Shapes::axisIndices(),
            std::vector<int>{3});

        vertexSrc = readFile(SIMS_DIR "riemann_sphere/shaders/vertex.glsl");
        fragTemplate = readFile(SIMS_DIR "riemann_sphere/shaders/domainColorTemplate.glsl");

        axisShader = engine::Shader(
            SIMS_DIR "riemann_sphere/shaders/vertex.glsl",
            SIMS_DIR "riemann_sphere/shaders/fragment.glsl");

        applyFunction(funcBuf);

        // sphere poles onto the Y axis
        model = glm::rotate(glm::mat4(1.0f), glm::radians(-90.f), glm::vec3(1.f, 0.f, 0.f));
    }

    void update(float dt) override
    {
        time += dt;
        mouse = app->window().cursorPosNormalized() * 2.0f - 1.0f; // [-1,1]^2

        // plane mode: the quad never moves — WASD displaces the window center
        // in the complex plane and Q/E scale it (both in domain units, so pan
        // speed shrinks as you zoom in)
        if (!sphereMode && !ImGui::GetIO().WantCaptureKeyboard) {
            const engine::Window& win = app->window();
            float pan = planeScale * dt;
            if (win.keyDown('A')) planeCenter.x -= pan;
            if (win.keyDown('D')) planeCenter.x += pan;
            if (win.keyDown('S')) planeCenter.y -= pan;
            if (win.keyDown('W')) planeCenter.y += pan;
            if (win.keyDown('Q')) planeScale *= std::exp(-dt); // zoom in
            if (win.keyDown('E')) planeScale *= std::exp(dt);  // zoom out
        }
    }

    void render(const engine::Camera& camera) override
    {
        // plane mode ignores the camera: the -90° X rotation alone maps the
        // XZ quad onto NDC, so it always fills the window exactly
        glm::mat4 view = sphereMode ? camera.getViewMatrix() : glm::mat4(1.0f);
        glm::mat4 projection = sphereMode ? camera.getProjectionMatrix() : glm::mat4(1.0f);

        functionShader.use();
        functionShader.setMat4("model", model);
        functionShader.setMat4("view", view);
        functionShader.setMat4("projection", projection);
        functionShader.setFloat("u_time", time);
        functionShader.setVec2("u_mouse", mouse);

        if (sphereMode) {
            sphereMesh->draw(GL_TRIANGLES);

            glLineWidth(4.0f);
            axisShader.use();
            axisShader.setMat4("model", model);
            axisShader.setMat4("view", view);
            axisShader.setMat4("projection", projection);
            axisMesh->draw(GL_LINES);
        }
        else {
            functionShader.setVec2("u_center", planeCenter);
            functionShader.setVec2("u_scale",
                glm::vec2(planeScale * app->window().aspect(), planeScale));
            planeMesh->draw(GL_TRIANGLES);
        }
    }

    // Hold 1 (App::kRecenterHoldSeconds): glide back to the view init() framed.
    void recenterCamera() override
    {
        app->camera().easeOrbitTo(glm::vec3(0.0f), 7.0f, 0.0f, 1.57079633f,
                                  engine::App::kRecenterEaseSeconds);
    }

    void drawUI() override
    {
        ImGui::TextUnformatted("f(z) =");
        ImGui::SameLine();
        bool apply = ImGui::InputText("##function", funcBuf, sizeof(funcBuf),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        apply |= ImGui::Button("Apply");
        if (apply)
            applyFunction(funcBuf);

        // shader hot-swap: coords() is baked into the fragment source, so a
        // mode change has to rebuild it; plane mode repurposes WASD/QE, so the
        // camera stops listening while it's active
        if (ImGui::Checkbox("Sphere mode", &sphereMode)) {
            app->camera().inputEnabled = sphereMode;
            applyFunction(funcBuf);
        }
        if (!sphereMode) {
            ImGui::SameLine();
            if (ImGui::Button("Reset pos")) {
                planeCenter = glm::vec2(0.0f);
                planeScale = 2.0f;
            }
            ImGui::SameLine();
            ImGui::TextDisabled("center %.3g%+.3gi  scale %.3g",
                                planeCenter.x, planeCenter.y, planeScale);
        }

        if (!parseError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", parseError.c_str());

        ImGui::TextWrapped(
            "Variables: z, t (time), m (mouse). Constants: i, pi, e, tau. "
            "Functions: sin cos tan sinh cosh tanh exp log sqrt conj abs arg re im. "
            "Implicit products work: 2z, (z-1)(z+1).");
    }

private:
    // Parse the typed function, generate GLSL for it and hot-swap the sphere
    // shader. On a parse error the previous shader keeps rendering.
    void applyFunction(const char* source)
    {
        engine::Expression expr = engine::Expression::parse(source, {"z", "t", "m"}, parseError);
        if (!expr.valid())
            return;

        std::string f = "vec2 f(vec2 z) {\n    return "
                      + expr.glsl({"z", "vec2(u_time, 0.0)", "u_mouse"}) + ";\n}";
        // stereographic projection from north pole (0,0,1) onto the complex
        // plane in sphere mode; plane mode maps the fullscreen quad through a
        // pan/zoom window (u_scale carries zoom * aspect so units stay square)
        const char* coords = sphereMode
            ? "vec2 coords(vec3 p) {\n"
              "    p = normalize(p);\n"
              "    return vec2(p.x, p.y) / (1.0 - p.z + 1e-6);\n"
              "}"
            : "uniform vec2 u_center;\n"
              "uniform vec2 u_scale;\n"
              "vec2 coords(vec3 p) {\n"
              "    return u_center + vec2(p.x, p.z) * u_scale;\n"
              "}";

        std::string frag = fragTemplate;
        replaceOnce(frag, "//__COMPLEX_LIB__", engine::Expression::glslComplexLib());
        replaceOnce(frag, "//__F__", f);
        replaceOnce(frag, "//__coords__", coords);

        functionShader = engine::Shader::fromSources(vertexSrc, frag);
    }

    static void replaceOnce(std::string& text, const std::string& placeholder,
                            const std::string& value)
    {
        size_t at = text.find(placeholder);
        if (at != std::string::npos)
            text.replace(at, placeholder.size(), value);
    }

    engine::App* app = nullptr;
    std::unique_ptr<engine::Mesh> sphereMesh;
    std::unique_ptr<engine::Mesh> planeMesh;
    std::unique_ptr<engine::Mesh> axisMesh;
    engine::Shader functionShader;
    engine::Shader axisShader;
    glm::mat4 model{1.0f};

    bool sphereMode = true;
    glm::vec2 planeCenter{0.0f}; // complex number at the screen center
    float planeScale = 2.0f;     // half-height of the visible domain window

    std::string vertexSrc;
    std::string fragTemplate;
    char funcBuf[256] = "z^2 + m";
    std::string parseError;
    float time = 0.0f;
    glm::vec2 mouse{0.0f};
};

} // namespace

std::unique_ptr<engine::Sim> makeRiemannSphereSim()
{
    return std::make_unique<RiemannSphereSim>();
}
