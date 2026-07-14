#ifndef ENGINE_APP_H
#define ENGINE_APP_H

#include <engine/Window.h>
#include <engine/Camera.h>
#include <engine/SimRegistry.h>
#include <memory>
#include <string>
#include <vector>

namespace engine {

// Owns the window, ImGui, the shared camera and the active sim, and runs the
// main loop. The same App serves the multi-sim explorer (picker shown when
// more than one sim is registered) and single-sim standalone executables.
class App {
public:
    App(const char* title = "Math Explorer", int width = 1280, int height = 720);
    ~App();
    App(const App&) = delete;
    App& operator=(const App&) = delete;

    SimRegistry& sims() { return registry; }
    Window& window() { return win; }
    Camera& camera() { return cam; }

    // Runs the main loop. With one registered sim (or an explicit startSim)
    // that sim starts immediately; with several and no startSim, the loop
    // opens on the thumbnail picker menu. Returns the process exit code.
    int run(const std::string& startSim = "");

private:
    // GL texture for one menu tile, loaded from SimRegistry's thumbnailPath
    struct Thumbnail {
        unsigned int texture = 0; // 0 = none; menu shows a plain named tile
        int width = 0;
        int height = 0;
    };

    Window win;
    Camera cam;
    SimRegistry registry;
    std::unique_ptr<Sim> activeSim;
    int activeIndex = -1;
    std::vector<Thumbnail> thumbnails;
    bool wantMenu = false; // set by the UI, applied after the frame's draw

    void switchTo(int index);
    void loadThumbnails();
    void drawAppUI();
    void drawMenuUI();
};

} // namespace engine

#endif
