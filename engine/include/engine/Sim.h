#ifndef ENGINE_SIM_H
#define ENGINE_SIM_H

namespace engine {

class App;
class Camera;

// The contract every simulation implements. One sim = one self-contained
// folder under sims/ with its own shaders, built as its own static library.
class Sim {
public:
    virtual ~Sim() = default;

    virtual const char* name() const = 0;

    // Called once after the sim becomes active; the GL context is live.
    // Create meshes/shaders here, and keep the App& if you need window info later.
    virtual void init(App& app) = 0;

    // Advance math/state by dt seconds.
    virtual void update(float dt) = 0;

    // Issue draw calls using the shared camera.
    virtual void render(const Camera& camera) = 0;

    // Optional ImGui widgets, drawn inside the explorer panel.
    virtual void drawUI() {}

    // Put the shared camera back to this sim's home view (what its "Reset camera" /
    // "Reset view" button does). App calls it when key 1 is held past
    // App::kRecenterHoldSeconds — a tap of 1 stays the sim's own (follow toggle).
    virtual void recenterCamera() {}
};

} // namespace engine

#endif
