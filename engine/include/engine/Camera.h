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
    glm::vec3 position{0.0f, 0.0f, 3.0f};
    glm::vec3 front{0.0f, 0.0f, -1.0f};
    glm::vec3 up{0.0f, 1.0f, 0.0f};
    glm::vec3 orbitCenter{0.0f, 0.0f, 0.0f};
    float speed = 1.5f;

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
    float orbitRadius = 3.0f;
};

} // namespace engine

#endif
