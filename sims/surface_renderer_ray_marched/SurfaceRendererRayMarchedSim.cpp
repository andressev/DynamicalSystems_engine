#include "SurfaceRendererRayMarchedSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/Expression.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>

#include <imgui.h>
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

// Raymarched graphing calculator for implicit surfaces: the typed f(x,y,z)
// and the level c define the level set f = c, rendered per-pixel in the
// fragment shader (no mesh — see docs/level-set-meshing-proposal.html §2).
class SurfaceRendererRayMarchedSim : public engine::Sim {
public:
    const char* name() const override { return "surface_renderer_ray_marched"; }

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
    }

    void update(float dt) override
    {
        time += dt;
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
        quadMesh->draw(GL_TRIANGLES);
    }

    void drawUI() override
    {
        ImGui::TextUnformatted("f(x,y,z) =");
        ImGui::SameLine();
        bool apply = ImGui::InputText("##function", funcBuf, sizeof(funcBuf),
                                      ImGuiInputTextFlags_EnterReturnsTrue);
        ImGui::SameLine();
        apply |= ImGui::Button("Apply");
        if (apply)
            applyFunction(funcBuf);

        if (!parseError.empty())
            ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.35f, 1.0f), "%s", parseError.c_str());

        // c is a uniform: dragging it sweeps the level set live, no recompile
        ImGui::DragFloat("c", &levelC, 0.02f);
        ImGui::DragFloat("domain radius", &range, 0.05f, 0.5f, 50.0f);

        ImGui::TextWrapped(
            "Level set f(x,y,z) = c, raymarched. Variables: x, y, z, t (time). "
            "Constants: i, pi, e, tau. The real part of the value is used. "
            "Teal side: f > c, amber side: f < c. Surface outside the domain "
            "radius is not searched.");
    }

private:
    // Parse the typed function, generate GLSL for it and hot-swap the
    // raymarching shader. On a parse error the previous shader keeps rendering.
    void applyFunction(const char* source)
    {
        engine::Expression expr =
            engine::Expression::parse(source, {"x", "y", "z", "t"}, parseError);
        if (!expr.valid())
            return;

        // real coordinates enter the complex evaluator as vec2(v, 0)
        std::string fc = "vec2 fc(vec3 p) {\n    return "
                       + expr.glsl({"vec2(p.x, 0.0)", "vec2(p.y, 0.0)",
                                    "vec2(p.z, 0.0)", "vec2(u_time, 0.0)"})
                       + ";\n}";
        std::string frag = fragTemplate;
        replaceOnce(frag, "//__COMPLEX_LIB__", engine::Expression::glslComplexLib());
        replaceOnce(frag, "//__F__", fc);

        raymarchShader = engine::Shader::fromSources(vertexSrc, frag);
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

    std::string vertexSrc;
    std::string fragTemplate;
    // default: the "tangle cube" quartic — sweeping c walks it through its
    // critical values (topology changes at c = 0, -6.25, -12.5, -18.75)
    char funcBuf[256] = "x^4 + y^4 + z^4 - 5*(x^2 + y^2 + z^2)";
    std::string parseError;
    float levelC = -11.8f;
    float range = 4.0f;
    float time = 0.0f;
};

} // namespace

std::unique_ptr<engine::Sim> makeSurfaceRendererRayMarchedSim()
{
    return std::make_unique<SurfaceRendererRayMarchedSim>();
}
