#ifndef ENGINE_APP_H
#define ENGINE_APP_H

#include <engine/Window.h>
#include <engine/Camera.h>
#include <engine/LatexBoard.h>
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

    // Key 1 held this long recenters the camera: App calls Sim::recenterCamera once
    // when the hold reaches it (FlowKeys::tapHold). Sims treat a shorter press as a tap.
    static constexpr float kRecenterHoldSeconds = 1.0f;
    // How long a recenter glides (sims pass it to Camera::easeOrbitTo or their own view ease).
    static constexpr float kRecenterEaseSeconds = 1.0f;

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

    // The Explorer panel (App's FPS/menu row + the sim's drawUI). H toggles it; a sim
    // may hide it itself (e.g. when it starts a presentation-like animation). A hidden
    // panel also pulls in by itself when the mouse reaches the top-left corner of the
    // window and goes again the moment the mouse leaves it. It fades in and out over
    // kPanelFadeSeconds either way.
    bool controlsVisible() const { return showControls; }
    void setControlsVisible(bool on) { showControls = on; controlsAuto = false; }
    static constexpr float kPanelHotCorner = 48.0f;   // px from the top-left corner
    static constexpr float kPanelFadeSeconds = 0.2f;
    static constexpr float kPanelMargin = 6.0f;       // px around the panel still "on" it

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
    LatexBoard board;   // L: the active sim's board.tex as a white card; PageDown/PageUp page
    int activeIndex = -1;
    std::vector<Thumbnail> thumbnails;
    bool wantMenu = false; // set by the UI, applied after the frame's draw
    int pendingSwitch = -1; // requestSwitch target, applied after the frame's draw
    Handoff pendingHandoff;
    Handoff handoffValues;  // what the active sim was handed
    float captureScale = 1.0f;
    bool showControls = false; // the Explorer panel (sim controls); H toggles it — hidden at start
    bool controlsAuto = false;   // shown by the hot corner: hides as the mouse leaves
    bool controlsHovered = false; // the panel under the mouse last frame (drawAppUI)
    bool uncollapseControls = false; // open the panel on the next drawAppUI
    bool panelEntered = false;    // the mouse has reached the auto panel since the pull
    float panelFade = 0.0f;       // 0 hidden -> 1 shown, eased toward showControls
    float panelX = 0.0f, panelY = 0.0f, panelW = 0.0f, panelH = 0.0f;   // its rect (drawAppUI)
    float lastMouseX = -1.0f, lastMouseY = -1.0f;
    void updateAutoPanel(float dt);
    bool onPanel(float mx, float my) const;   // inside the rect, kPanelMargin around it

    void switchTo(int index);
    void loadThumbnails();
    void drawAppUI();
    void drawMenuUI();
};

} // namespace engine

#endif
