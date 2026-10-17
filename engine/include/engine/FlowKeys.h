#ifndef ENGINE_FLOW_KEYS_H
#define ENGINE_FLOW_KEYS_H

namespace engine {

// The keyboard playback controls every flow sim shares, so they behave the same in all
// of them: holding Up speeds the flow up, holding Down slows it down to 0 (frozen), Left
// runs time backwards, Right forwards again. The speed ramps smoothly while a key is held:
// kRate per second up to 1x (0 -> 1 in about 2 s), proportionally faster above. Ignored
// while ImGui has the keyboard (typing in a field). Call once per frame from
// Sim::update with the frame's dt.
struct FlowKeys {
    static constexpr float kRate = 0.5f;   // speed units per second at or below 1x
    static constexpr float kDecadeSeconds = 1.2f;   // log ramp: holding for this x10 / /10

    // Returns true when either value changed. With minScale > 0 the ramp is logarithmic
    // instead and the speed stays in [minScale, maxScale]: every factor of 10 takes
    // kDecadeSeconds of holding, so the slow end (0.01 -> 0.001 -> 0.0001) comes gently
    // instead of rushing past it.
    static bool handle(float& timeScale, bool& reverse, float dt, float maxScale = 10.0f,
                       float minScale = 0.0f);

    // Bottom-right corner readout: the current speed ("1.00x") and below it "normal" or
    // "reversed"; dimmed while paused. Call once per frame from Sim::update (it is an
    // ImGui overlay, so screenshots leave it out).
    static void drawStatus(float timeScale, bool reverse, bool playing = true);

    // One-line hint for the Flow section, next to the play button.
    static constexpr const char* kHint = "(space, arrows)";
};

} // namespace engine

#endif
