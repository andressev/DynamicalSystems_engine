#ifndef ENGINE_APP_H
#define ENGINE_APP_H

#include <engine/Window.h>
#include <engine/Camera.h>
#include <engine/SimRegistry.h>
#include <map>
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

    // Render the active sim alone (no ImGui) into an offscreen multisampled buffer at
    // `scale` x the window's framebuffer size and write it to `path` as a PNG (parent
    // folders are created). Call from Sim::update once its state is current for the
    // frame; the sim's render() runs inside. Returns false on any GL or file failure.
    bool captureScreenshot(const std::string& path, int scale = 2);

    // 1 normally; the capture scale while captureScreenshot is rendering. Sims multiply
    // pixel-sized things (line widths, point sizes) by it so the image keeps their look.
    float renderScale() const { return captureScale; }

    // Values one sim hands to the next on a requestSwitch (parameters, a camera to
    // return to...): plain key -> number, so sims never include each other's headers.
    using Handoff = std::map<std::string, double>;

    // Switch to the registered sim `name` at the end of this frame (the current sim's UI
    // and update finish first), passing `values`. The shared Camera is left exactly as it
    // is, so a sim that keeps it on a handoff (no reframing in init) makes the switch
    // seamless. Returns false and does nothing when `name` isn't registered — e.g. in a
    // standalone build; check with hasSim() to offer the switch at all.
    bool requestSwitch(const std::string& name, Handoff values = {});
    bool hasSim(const std::string& name) const { return registry.indexOf(name) >= 0; }

    // What the sim that switched into this one handed over; empty when it was started
    // from the menu or at launch. Read it in init(); valid until the next switch.
    const Handoff& handoff() const { return handoffValues; }

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
    int pendingSwitch = -1; // requestSwitch target, applied after the frame's draw
    Handoff pendingHandoff;
    Handoff handoffValues;  // what the active sim was handed
    float captureScale = 1.0f;

    void switchTo(int index);
    void loadThumbnails();
    void drawAppUI();
    void drawMenuUI();
};

} // namespace engine

#endif
