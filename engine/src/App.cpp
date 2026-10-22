#include <engine/App.h>
#include <engine/FlowKeys.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace engine {

namespace {

constexpr float kClearColor[4] = {0.06f, 0.06f, 0.08f, 1.0f};

// Minimal PNG encoder for screenshots: 8-bit RGB, rows unfiltered, deflate in stored
// (uncompressed) blocks — no zlib dependency, at the cost of raw-sized files.
// `rgb` is top-row first.
bool writePng(const std::string& path, const std::vector<unsigned char>& rgb, int w, int h)
{
    static uint32_t crcTable[256];
    static bool tableReady = false;
    if (!tableReady)
    {
        for (uint32_t n = 0; n < 256; ++n)
        {
            uint32_t c = n;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            crcTable[n] = c;
        }
        tableReady = true;
    }

    std::vector<unsigned char> raw; // zlib payload input: filter byte + row, per row
    raw.reserve((size_t)h * (w * 3 + 1));
    for (int y = 0; y < h; ++y)
    {
        raw.push_back(0);
        raw.insert(raw.end(), rgb.begin() + (size_t)y * w * 3, rgb.begin() + (size_t)(y + 1) * w * 3);
    }

    std::vector<unsigned char> z = {0x78, 0x01}; // zlib header, no compression
    for (size_t pos = 0; pos < raw.size() || pos == 0;)
    {
        size_t len = std::min<size_t>(65535, raw.size() - pos);
        bool last = pos + len == raw.size();
        z.push_back(last ? 1 : 0);
        z.push_back(len & 0xFF);
        z.push_back((len >> 8) & 0xFF);
        z.push_back(~len & 0xFF);
        z.push_back((~len >> 8) & 0xFF);
        z.insert(z.end(), raw.begin() + pos, raw.begin() + pos + len);
        pos += len;
        if (last)
            break;
    }
    uint32_t a = 1, b = 0; // adler32 of the uncompressed data
    for (unsigned char c : raw)
    {
        a = (a + c) % 65521;
        b = (b + a) % 65521;
    }
    uint32_t adler = (b << 16) | a;
    for (int i = 3; i >= 0; --i)
        z.push_back((adler >> (8 * i)) & 0xFF);

    std::ofstream out(path, std::ios::binary);
    if (!out)
        return false;
    auto put32 = [&](uint32_t v) {
        for (int i = 3; i >= 0; --i)
            out.put((char)((v >> (8 * i)) & 0xFF));
    };
    auto chunk = [&](const char* type, const std::vector<unsigned char>& data) {
        put32((uint32_t)data.size());
        uint32_t crc = 0xFFFFFFFFu;
        for (int i = 0; i < 4; ++i)
            crc = crcTable[(crc ^ (unsigned char)type[i]) & 0xFF] ^ (crc >> 8);
        for (unsigned char c : data)
            crc = crcTable[(crc ^ c) & 0xFF] ^ (crc >> 8);
        out.write(type, 4);
        out.write((const char*)data.data(), (std::streamsize)data.size());
        put32(crc ^ 0xFFFFFFFFu);
    };

    static const unsigned char signature[8] = {0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
    out.write((const char*)signature, 8);
    std::vector<unsigned char> ihdr(13, 0);
    for (int i = 0; i < 4; ++i)
    {
        ihdr[i] = (w >> (24 - 8 * i)) & 0xFF;
        ihdr[4 + i] = (h >> (24 - 8 * i)) & 0xFF;
    }
    ihdr[8] = 8; // bit depth
    ihdr[9] = 2; // color type: RGB
    chunk("IHDR", ihdr);
    chunk("IDAT", z);
    chunk("IEND", {});
    return (bool)out;
}

} // namespace

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

        cam.updateEase(dt); // a recenter glide in progress, before input and the sim

        ImGuiIO& io = ImGui::GetIO();
        if (!io.WantCaptureKeyboard)
        {
            if (glfwGetKey(win.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS)
                win.requestClose();
            // F11: borderless full screen on / off (the maximize button enters it too)
            if (ImGui::IsKeyPressed(ImGuiKey_F11, false))
                win.toggleFullscreen();
            if (activeSim)
            {
                cam.processInput(win.handle(), dt);
                // H hides/shows the controls panel in every sim (HUD overlays stay)
                if (ImGui::IsKeyPressed(ImGuiKey_H, false))
                {
                    showControls = !showControls;
                    controlsAuto = false;
                }
                // L shows / hides the sim's LaTeX board; PageDown / PageUp (what a
                // presentation clicker sends) turn its pages
                if (ImGui::IsKeyPressed(ImGuiKey_L, false))
                    board.toggle();
                if (board.visible() && ImGui::IsKeyPressed(ImGuiKey_PageDown, false))
                    board.nextPage();
                if (board.visible() && ImGui::IsKeyPressed(ImGuiKey_PageUp, false))
                    board.prevPage();
                // 1 held past kRecenterHoldSeconds: the sim's home view (a tap is the
                // sim's own: follow toggle)
                for (int key : {(int)ImGuiKey_1, (int)ImGuiKey_Keypad1})
                    if (FlowKeys::tapHold(key, dt, kRecenterHoldSeconds).held)
                        activeSim->recenterCamera();
            }
        }
        cam.aspect = win.aspect();
        if (activeSim)
            updateAutoPanel(dt);
        // the panel fades toward where showControls says it should be
        {
            const float step = dt / kPanelFadeSeconds, target = showControls ? 1.0f : 0.0f;
            panelFade += std::clamp(target - panelFade, -step, step);
        }

        if (activeSim)
            activeSim->update(dt);

        glClearColor(kClearColor[0], kClearColor[1], kClearColor[2], kClearColor[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (activeSim)
        {
            activeSim->render(cam);
            board.draw();   // under the panel
            if (panelFade > 0.0f)   // still drawn while it fades out
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
            board.close();
            pendingSwitch = -1;
        }
        else if (pendingSwitch >= 0)
        {
            int target = pendingSwitch;
            pendingSwitch = -1;
            handoffValues = std::move(pendingHandoff);
            pendingHandoff.clear();
            switchTo(target);
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
    board.setFile(std::string(SIMS_DIR) + registry.names()[index] + "/board.tex");
    activeSim->init(*this);
}

bool App::requestSwitch(const std::string& name, Handoff values)
{
    int index = registry.indexOf(name);
    if (index < 0)
        return false;
    pendingSwitch = index;
    pendingHandoff = std::move(values);
    return true;
}

bool App::captureScreenshot(const std::string& path, int scale)
{
    if (!activeSim || win.width() <= 0 || win.height() <= 0)
        return false;

    GLint maxSize = 0, maxSamples = 0;
    glGetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &maxSize);
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    scale = std::max(1, scale);
    while (scale > 1 && std::max(win.width(), win.height()) * scale > maxSize)
        --scale;
    const int w = win.width() * scale, h = win.height() * scale;
    const int samples = std::clamp((int)maxSamples, 1, 8);

    // [0] multisampled color + depth the sim renders into, [1] single-sample resolve target
    GLuint fbo[2] = {0, 0}, rb[3] = {0, 0, 0};
    glGenFramebuffers(2, fbo);
    glGenRenderbuffers(3, rb);
    glBindRenderbuffer(GL_RENDERBUFFER, rb[0]);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, rb[1]);
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_DEPTH24_STENCIL8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, rb[2]);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA8, w, h);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, fbo[1]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb[2]);
    bool ok = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, fbo[0]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, rb[0]);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rb[1]);
    ok = ok && glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;

    std::vector<unsigned char> pixels;
    if (ok)
    {
        glViewport(0, 0, w, h);
        glClearColor(kClearColor[0], kClearColor[1], kClearColor[2], kClearColor[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        captureScale = (float)scale;
        activeSim->render(cam);
        captureScale = 1.0f;

        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo[0]);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo[1]);
        glBlitFramebuffer(0, 0, w, h, 0, 0, w, h, GL_COLOR_BUFFER_BIT, GL_NEAREST);

        glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo[1]);
        pixels.resize((size_t)w * h * 3);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, win.width(), win.height());
    glDeleteFramebuffers(2, fbo);
    glDeleteRenderbuffers(3, rb);
    if (!ok)
    {
        std::cout << "App::captureScreenshot: offscreen framebuffer incomplete" << std::endl;
        return false;
    }

    // GL rows run bottom-up; PNG rows top-down
    const size_t row = (size_t)w * 3;
    std::vector<unsigned char> flipped(pixels.size());
    for (int y = 0; y < h; ++y)
        std::copy_n(pixels.begin() + (size_t)(h - 1 - y) * row, row, flipped.begin() + (size_t)y * row);

    std::error_code ec;
    std::filesystem::path parent = std::filesystem::path(path).parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent, ec);
    if (!writePng(path, flipped, w, h))
    {
        std::cout << "App::captureScreenshot: could not write " << path << std::endl;
        return false;
    }
    std::cout << "Screenshot " << w << "x" << h << " -> " << path << std::endl;
    return true;
}

bool App::onPanel(float mx, float my) const
{
    return panelW > 0.0f && mx >= panelX - kPanelMargin && mx < panelX + panelW + kPanelMargin &&
           my >= panelY - kPanelMargin && my < panelY + panelH + kPanelMargin;
}

// The hidden panel pulls in when the mouse reaches the top-left corner and goes again
// the moment the mouse is off it (a dragged slider or an open popup keep it). On the
// way from the corner to the panel, before the mouse has reached it, the stretch
// between them counts as "on". A panel shown with H (or by a sim) stays.
void App::updateAutoPanel(float)
{
    const ImGuiIO& io = ImGui::GetIO();
    const float mx = io.MousePos.x, my = io.MousePos.y;
    const bool inWindow = mx >= 0.0f && my >= 0.0f && mx < io.DisplaySize.x && my < io.DisplaySize.y;
    const bool moved = mx != lastMouseX || my != lastMouseY;
    lastMouseX = mx;
    lastMouseY = my;
    const bool inCorner = inWindow && mx < kPanelHotCorner && my < kPanelHotCorner;
    const bool over = inWindow && (controlsHovered || onPanel(mx, my));

    if (!showControls)
    {
        if (inCorner && moved && panelFade <= 0.0f)
        {
            showControls = controlsAuto = uncollapseControls = true;
            panelEntered = false;
        }
        else if (controlsAuto && panelFade > 0.0f && over)
            showControls = true;   // came back while it was fading out
        return;
    }
    if (!controlsAuto)
        return;
    if (over)
        panelEntered = true;
    // the box spanning the corner and the panel, for the approach
    const bool approaching = !panelEntered && inWindow &&
                             mx < std::max(panelX + panelW, kPanelHotCorner) + kPanelMargin &&
                             my < std::max(panelY + panelH, kPanelHotCorner) + kPanelMargin;
    const bool keep = over || inCorner || approaching || ImGui::IsAnyItemActive() ||
                      ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId);
    if (!keep)
    {
        showControls = false;   // fades out from here
        controlsHovered = false;
    }
}

void App::drawAppUI()
{
    // start collapsed each launch so the view opens unobstructed
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
    if (uncollapseControls)
    {
        uncollapseControls = false;
        ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
    }
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, std::clamp(panelFade, 0.0f, 1.0f));
    ImGui::Begin("Explorer");
    controlsHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows |
                                             ImGuiHoveredFlags_AllowWhenBlockedByActiveItem |
                                             ImGuiHoveredFlags_AllowWhenBlockedByPopup);
    {
        const ImVec2 p = ImGui::GetWindowPos(), s = ImGui::GetWindowSize();
        panelX = p.x;
        panelY = p.y;
        panelW = s.x;
        panelH = s.y;
    }
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
    ImGui::PopStyleVar();
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
    {
        handoffValues.clear(); // started from the menu: nothing handed over
        switchTo(clicked);
    }
}

} // namespace engine
