#include "RiemannSphereSim.h"

#include <engine/App.h>
#include <engine/Camera.h>
#include <engine/Mesh.h>
#include <engine/Shader.h>
#include <engine/Shapes.h>

#include <gtc/matrix_transform.hpp>
#include <imgui.h>
#include <string>

namespace {

class RiemannSphereSim : public engine::Sim {
public:
    const char* name() const override { return "riemann_sphere"; }

    void init(engine::App& app) override
    {
        this->app = &app;

        sphereMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::sphereVertices(100, 100, 2.5f),
            engine::Shapes::sphereIndices(100, 100),
            std::vector<int>{3, 3, 2});

        axisMesh = std::make_unique<engine::Mesh>(
            engine::Shapes::axisVertices(),
            engine::Shapes::axisIndices(),
            std::vector<int>{3});

        sphereShader = std::make_unique<engine::Shader>(
            SIMS_DIR "riemann_sphere/shaders/vertex.glsl",
            SIMS_DIR "riemann_sphere/shaders/complexFragment.glsl");

        axisShader = std::make_unique<engine::Shader>(
            SIMS_DIR "riemann_sphere/shaders/vertex.glsl",
            SIMS_DIR "riemann_sphere/shaders/fragment.glsl");

        // sphere poles onto the Y axis
        model = glm::rotate(glm::mat4(1.0f), glm::radians(-90.f), glm::vec3(1.f, 0.f, 0.f));
    }

    void update(float /*dt*/) override
    {
        if (followMouse)
        {
            glm::vec2 m = app->window().cursorPosNormalized();
            float mx = m.x, my = m.y;
            a[0] = mx * my;
            a[1] = mx - my;
            a[2] = mx * mx - my;
            a[3] = my * my - mx;
            a[4] = mx * mx * my;
            a[5] = my * my * mx;
            a[6] = mx - my * my;
            a[7] = mx * my * (mx - my);
        }
    }

    void render(const engine::Camera& camera) override
    {
        glm::mat4 view = camera.getViewMatrix();
        glm::mat4 projection = camera.getProjectionMatrix();

        sphereShader->use();
        sphereShader->setMat4("model", model);
        sphereShader->setMat4("view", view);
        sphereShader->setMat4("projection", projection);
        for (int i = 0; i < 8; ++i)
            sphereShader->setFloat("a[" + std::to_string(i) + "]", a[i]);
        sphereMesh->draw(GL_TRIANGLES);

        glLineWidth(4.0f);
        axisShader->use();
        axisShader->setMat4("model", model);
        axisShader->setMat4("view", view);
        axisShader->setMat4("projection", projection);
        axisMesh->draw(GL_LINES);
    }

    void drawUI() override
    {
        ImGui::Checkbox("Follow mouse", &followMouse);
        ImGui::TextUnformatted("f(z) = a0 + a1 z + ... + a7 z^7");
        ImGui::BeginDisabled(followMouse);
        for (int i = 0; i < 8; ++i)
        {
            std::string label = "a" + std::to_string(i);
            ImGui::SliderFloat(label.c_str(), &a[i], -2.0f, 2.0f);
        }
        ImGui::EndDisabled();
    }

private:
    engine::App* app = nullptr;
    std::unique_ptr<engine::Mesh> sphereMesh;
    std::unique_ptr<engine::Mesh> axisMesh;
    std::unique_ptr<engine::Shader> sphereShader;
    std::unique_ptr<engine::Shader> axisShader;
    glm::mat4 model{1.0f};

    bool followMouse = true;
    float a[8] = {0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}; // f(z) = z when sliders take over
};

} // namespace

std::unique_ptr<engine::Sim> makeRiemannSphereSim()
{
    return std::make_unique<RiemannSphereSim>();
}
