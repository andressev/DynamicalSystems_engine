#include <engine/App.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

namespace engine {

App::App(const char* title, int width, int height)
    : win(title, width, height)
{
    glEnable(GL_DEPTH_TEST);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    // Enlarge the whole UI; fonts stay crisp (1.92+ rasterizes at the scaled size).
    const float uiScale = 1.5f;
    ImGuiStyle& style = ImGui::GetStyle();
    style.ScaleAllSizes(uiScale);
    style.FontScaleMain = uiScale;

    ImGui_ImplGlfw_InitForOpenGL(win.handle(), true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

App::~App()
{
    activeSim.reset(); // destroy GL resources while the context is still alive
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

int App::run(const std::string& startSim)
{
    if (registry.empty())
    {
        std::cout << "App::run: no sims registered" << std::endl;
        return -1;
    }

    int startIndex = startSim.empty() ? 0 : registry.indexOf(startSim);
    if (startIndex < 0)
    {
        std::cout << "App::run: unknown sim '" << startSim << "'" << std::endl;
        return -1;
    }
    switchTo(startIndex);

    float lastTime = (float)glfwGetTime();

    while (!win.shouldClose())
    {
        win.pollEvents();

        float now = (float)glfwGetTime();
        float dt = now - lastTime;
        lastTime = now;

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureKeyboard)
        {
            if (glfwGetKey(win.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
                win.requestClose();
            cam.processInput(win.handle(), dt);
        }
        cam.aspect = win.aspect();

        activeSim->update(dt);

        glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        activeSim->render(cam);

        drawAppUI();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        win.swapBuffers();
    }
    return 0;
}

void App::switchTo(int index)
{
    activeSim.reset(); // free the old sim's GL resources before creating the new one
    activeSim = registry.createByIndex(index);
    activeIndex = index;
    activeSim->init(*this);
}

void App::drawAppUI()
{
    const auto& names = registry.names();

    // start collapsed each launch so the view opens unobstructed
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
    ImGui::Begin("Explorer");
    ImGui::Text("%.1f FPS (%.2f ms)", ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);

    if (names.size() > 1)
    {
        if (ImGui::BeginCombo("Simulation", names[activeIndex].c_str()))
        {
            for (int i = 0; i < (int)names.size(); ++i)
            {
                bool selected = (i == activeIndex);
                if (ImGui::Selectable(names[i].c_str(), selected) && !selected)
                    switchTo(i);
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    ImGui::Separator();
    activeSim->drawUI();
    ImGui::End();
}

} // namespace engine
