#include "RiemannSphereSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/Expression.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>

#include <gtc/matrix_transform.hpp>
#include <imgui.h>
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
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix();

        sphereShader.use();
        sphereShader.setMat4("model", model);
        sphereShader.setMat4("view", view);
        sphereShader.setMat4("projection", projection);
        sphereShader.setFloat("u_time", time);
        sphereShader.setVec2("u_mouse", mouse);
        sphereMesh->draw(GL_TRIANGLES);

        glLineWidth(4.0f);
        axisShader.use();
        axisShader.setMat4("model", model);
        axisShader.setMat4("view", view);
        axisShader.setMat4("projection", projection);
        axisMesh->draw(GL_LINES);
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
        std::string frag = fragTemplate;
        replaceOnce(frag, "//__COMPLEX_LIB__", engine::Expression::glslComplexLib());
        replaceOnce(frag, "//__F__", f);

        sphereShader = engine::Shader::fromSources(vertexSrc, frag);
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
    std::unique_ptr<engine::Mesh> axisMesh;
    engine::Shader sphereShader;
    engine::Shader axisShader;
    glm::mat4 model{1.0f};

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
