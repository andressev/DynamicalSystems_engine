#ifndef ENGINE_CAMERA_H
#define ENGINE_CAMERA_H

#include <glm.hpp>
#include <gtc/constants.hpp>

struct GLFWwindow;

namespace engine {

class Camera {
public:
    enum class Mode { Orbit, Free };

    Mode mode = Mode::Orbit;
    // When false, processInput ignores the keyboard entirely — for sims that
    // repurpose the movement keys. A sim that clears it must restore it.
    bool inputEnabled = true;
    glm::vec3 position{0.0f, 0.0f, 3.0f};
    glm::vec3 front{0.0f, 0.0f, -1.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    glm::vec3 orbitCenter{0.0f, 0.0f, 0.0f};
    // Translation rate: world units per second (free-mode movement, orbit zoom).
    // A sim whose scene is huge scales this up so crossing it doesn't take a minute.
    float speed = 1.5f;
    // Orbit rotation rate: RADIANS per second, deliberately separate from `speed`.
    // They are different units, and tying them together makes a large scene spin
    // absurdly fast the moment its zoom rate is raised.
    float orbitAngularSpeed = 1.5f;

    // Orbit constraints, tunable per sim in init(): the camera never gets closer
    // to orbitCenter than orbitRadiusMin, and when orbitSlowZone > 0 all orbit
    // movement slows in proportion to the remaining distance to orbitRadiusMin
    // (e.g. set orbitRadiusMin just above a sphere's surface to inspect it).
    float orbitRadiusMin = 0.1f;
    float orbitSlowZone = 0.0f;
    // Current orbit distance; sims may set it in init() to frame their scene.
    float orbitRadius = 3.0f;
    // Orbit angles (radians): theta around +Y from +Z, phi down from +Y (clamped away
    // from the poles each frame). Together with orbitRadius and orbitCenter they ARE the
    // orbit view — save these four to restore a camera, not position/front, which
    // orbit mode recomputes from them every frame.
    float orbitTheta = 0.0f;
    float orbitPhi = glm::half_pi<float>();

    // projection parameters; App updates aspect from the window each frame
    float fovDeg = 55.0f;
    float aspect = 4.0f / 3.0f;
    float nearPlane = 0.1f;
    float farPlane = 100.0f;

    void processInput(GLFWwindow* window, float dt);

    // Glide the orbit view to a target over `seconds` instead of snapping: smootherstep,
    // radius in log space, theta by the shortest way round. App advances it
    // (updateEase) every frame before processInput; any orbit key press cancels it so
    // the user is never fought. Sims use it for their recenter (Sim::recenterCamera).
    void easeOrbitTo(const glm::vec3& center, float radius, float theta, float phi,
                     float seconds);
    void updateEase(float dt);
    void cancelEase() { easeDuration = 0.0f; }
    bool easing() const { return easeT < easeDuration; }

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;

private:
    void freeMode(GLFWwindow* window, float cameraSpeed);
    void orbitMode(GLFWwindow* window, float dt);
    void placeOnOrbit(); // position/front from the four orbit values

    // easeOrbitTo state: from (0) and to (1) over easeDuration seconds
    float easeT = 0.0f, easeDuration = 0.0f;
    glm::vec3 easeC0{0.0f}, easeC1{0.0f};
    float easeR0 = 1.0f, easeR1 = 1.0f, easeTh0 = 0.0f, easeTh1 = 0.0f;
    float easePh0 = 0.0f, easePh1 = 0.0f;
};

} // namespace engine

#endif
