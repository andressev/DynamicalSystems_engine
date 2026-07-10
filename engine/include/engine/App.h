#ifndef ENGINE_APP_H
#define ENGINE_APP_H

#include <engine/Window.h>
#include <engine/Camera.h>
#include <engine/SimRegistry.h>
#include <memory>
#include <string>

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

    // Runs the main loop with startSim active (first registered sim if empty).
    // Returns the process exit code.
    int run(const std::string& startSim = "");

private:
    Window win;
    Camera cam;
    SimRegistry registry;
    std::unique_ptr<Sim> activeSim;
    int activeIndex = -1;

    void switchTo(int index);
    void drawAppUI();
};

} // namespace engine

#endif
