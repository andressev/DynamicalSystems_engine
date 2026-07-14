#include <engine/App.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

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
    for (const Thumbnail& t : thumbnails)
    {
        if (t.texture)
        {
            unsigned int id = t.texture;
            glDeleteTextures(1, &id);
        }
    }
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

    // several sims and no explicit start -> open on the picker menu instead
    int startIndex = startSim.empty()
        ? (registry.names().size() > 1 ? -1 : 0)
        : registry.indexOf(startSim);
    if (!startSim.empty() && startIndex < 0)
    {
        std::cout << "App::run: unknown sim '" << startSim << "'" << std::endl;
        return -1;
    }
    if (startIndex >= 0)
        switchTo(startIndex);
    loadThumbnails();

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
            if (activeSim)
                cam.processInput(win.handle(), dt);
        }
        cam.aspect = win.aspect();

        if (activeSim)
            activeSim->update(dt);

        glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (activeSim)
        {
            activeSim->render(cam);
            drawAppUI();
        }
        else
        {
            drawMenuUI();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // deferred so the sim's UI is never destroyed mid-frame
        if (wantMenu)
        {
            activeSim.reset();
            activeIndex = -1;
            wantMenu = false;
        }

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
    // start collapsed each launch so the view opens unobstructed
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
    ImGui::Begin("Explorer");
    ImGui::Text("%.1f FPS (%.2f ms)", ImGui::GetIO().Framerate, 1000.0f / ImGui::GetIO().Framerate);

    if (registry.names().size() > 1)
    {
        ImGui::SameLine();
        if (ImGui::SmallButton("menu"))
            wantMenu = true; // applied after the frame; see run()
    }

    ImGui::Separator();
    activeSim->drawUI();
    ImGui::End();
}

// Read each registered sim's thumbnail image (if any) into a GL texture for
// the picker menu. Called once in run(), after the GL context exists.
void App::loadThumbnails()
{
    const auto& paths = registry.thumbnailPaths();
    thumbnails.assign(paths.size(), {});

    for (size_t i = 0; i < paths.size(); ++i)
    {
        if (paths[i].empty())
            continue;
        int w = 0, h = 0, channels = 0;
        unsigned char* pixels = stbi_load(paths[i].c_str(), &w, &h, &channels, 4);
        if (!pixels)
            continue; // no image on disk -> plain named tile

        unsigned int tex = 0;
        glGenTextures(1, &tex);
        glBindTexture(GL_TEXTURE_2D, tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
        stbi_image_free(pixels);

        thumbnails[i] = {tex, w, h};
    }
}

// The sim picker: a centered grid of thumbnail tiles, one per registered sim.
// Shown at startup (multi-sim builds) and via the Explorer "menu" button.
void App::drawMenuUI()
{
    const auto& names = registry.names();
    ImGuiIO& io = ImGui::GetIO();

    ImGui::SetNextWindowPos(ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
                            ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin("##simMenu", nullptr,
                 ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoSavedSettings);

    ImGui::TextDisabled("Simulations:");
    ImGui::Spacing();

    const ImVec2 tile(220.0f, 124.0f);
    const int perRow = 3;
    const float tileOuterW = tile.x + 2.0f * ImGui::GetStyle().FramePadding.x;
    int clicked = -1;

    for (int i = 0; i < (int)names.size(); ++i)
    {
        if (i % perRow != 0)
            ImGui::SameLine();

        ImGui::PushID(i);
        ImGui::BeginGroup();

        bool pressed;
        if (i < (int)thumbnails.size() && thumbnails[i].texture != 0)
            pressed = ImGui::ImageButton("##thumb",
                                         (ImTextureID)(intptr_t)thumbnails[i].texture, tile);
        else
            pressed = ImGui::Button("(no preview)",
                                    ImVec2(tileOuterW, tile.y + 2.0f * ImGui::GetStyle().FramePadding.y));
        if (pressed)
            clicked = i;

        // sim name centered under its tile
        float textW = ImGui::CalcTextSize(names[i].c_str()).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + (tileOuterW - textW) * 0.5f);
        ImGui::TextUnformatted(names[i].c_str());

        ImGui::EndGroup();
        ImGui::PopID();
    }

    ImGui::End();

    if (clicked >= 0)
        switchTo(clicked);
}

} // namespace engine
