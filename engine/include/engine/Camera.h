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
    float speed = 1.5f;

    // Orbit constraints, tunable per sim in init(): the camera never gets closer
    // to orbitCenter than orbitRadiusMin, and when orbitSlowZone > 0 all orbit
    // movement slows in proportion to the remaining distance to orbitRadiusMin
    // (e.g. set orbitRadiusMin just above a sphere's surface to inspect it).
    float orbitRadiusMin = 0.1f;
    float orbitSlowZone = 0.0f;
    // Current orbit distance; sims may set it in init() to frame their scene.
    float orbitRadius = 3.0f;

    // projection parameters; App updates aspect from the window each frame
    float fovDeg = 55.0f;
    float aspect = 4.0f / 3.0f;
    float nearPlane = 0.1f;
    float farPlane = 100.0f;

    void processInput(GLFWwindow* window, float dt);

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;

private:
    void freeMode(GLFWwindow* window, float cameraSpeed);
    void orbitMode(GLFWwindow* window, float cameraSpeed);

    float orbitTheta = 0.0f;
    float orbitPhi = glm::half_pi<float>();
};

} // namespace engine

#endif
